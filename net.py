import torch
import torch.nn as nn
import torchvision
import torch.optim as optim
import time
import copy
import torch.utils.data as tdata
import data_management as dm
from dataset import sample_size
import numpy as np
import random
import torch.nn.functional as F
import os.path as osp


def create_model():
	model = torchvision.models.vgg16_bn(pretrained=True)
	for param in model.parameters():
		param.requires_grad = False

	num_ftrs = model.classifier[6].in_features
	model.classifier[6] = nn.Linear(num_ftrs, 7)
	return model


def lossfunc(prediction, label):
	alfa = 1
	beta = 120
	pos_pred = prediction[:, :3]
	pos_label = label[:, :3]
	mseloss = F.mse_loss(pos_label, pos_pred)
	rot_pred = prediction[:, 3:]
	tmp = torch.zeros(rot_pred.shape, device=rot_pred.device)
	for i in range(rot_pred.shape[0]):
		tmp[i, :] = rot_pred[i, :] / torch.norm(rot_pred[i, :], p=2)
	# print(torch.norm(rot_pred))
	rot_label = label[:, 3:]
	rotate_loss = F.mse_loss(rot_label, tmp)
	#	# batchsize = label.shape[0]
	#	# rotate_loss = torch.bmm(rot_pred.view(batchsize, 1, 4), rot_label.view(batchsize, 4, 1))
	#	# rotate_loss = torch.abs(rotate_loss)
	#	# rotate_loss = torch.sum(rotate_loss)

	return alfa * mseloss + beta * rotate_loss


# return F.mse_loss(prediction, label)


class Dataset(tdata.Dataset):
	def __init__(self, dataset_name):
		super().__init__()
		self.images = np.load(f"{dataset_name}.npy")
		self.labels = np.load(f"{dataset_name}_labels.npy")
		self.transform = torchvision.transforms.Compose([
			torchvision.transforms.ToTensor(),
			torchvision.transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225])
		])

	def __len__(self):
		return self.images.shape[0]

	def __getitem__(self, item):
		# return {
		# 	'label' : np.asarray(self.labels[item]),
		# 	'image' : np.asarray(self.images[item]),
		# }
		image = np.asarray(self.images[item]).astype("f4") / 255
		# image = 2 * (image - 0.5)
		image = self.transform(image)
		return (image, np.asarray(self.labels[item]).astype("f4"))


def accuracy(prediction, labels):
	# diff = (prediction - labels) ** 2
	# norms = torch.norm(labels, p=2, dim=1)
	ret = torch.zeros((labels.shape[0]))
	diffs = torch.zeros((labels.shape[0]))
	for i in range(labels.shape[0]):
		diff = prediction[i, :] - labels[i, :]
		diffs[i] = torch.norm(diff[:3])
		ret[i] = torch.norm(diff) / torch.norm(labels[i, :])
	return torch.sum(ret) / ret.shape[0], torch.sum(diffs) / diffs.shape[0]

def relative_err(outputs, labels):
	ret = 0.0
	for i in range(labels.shape[0]):
		diff = outputs[i, :] - labels[i, :]
		tmp = torch.norm(diff[:3], p=2) / torch.norm(labels[i, :3])
		ret += tmp
	return ret

def diffs_distance(outputs, labels):
	ret = 0.0
	for i in range(labels.shape[0]):
		diff = outputs[i, :] - labels[i, :]
		ret += torch.norm(diff[:3], p=2)
	return ret

def cmonfuzzy(outputs, labels):
	ret = 0.0
	ret2 = 0.0
	for i in range(labels.shape[0]):
		diff = outputs[i, :] - labels[i, :]
		normadiffu = torch.norm(diff[:3], p=2)
		tmp =  normadiffu / torch.norm(labels[i, :3])
		ret += normadiffu
		ret2 += tmp
	return ret, ret2

##############
def train_model(model, criterion, optimizer, scheduler, dataloaders, dataset_sizes, num_epochs=25):
	since = time.time()
	device = torch.device("cuda:3")
	model.to(device)
	best_model_wts = copy.deepcopy(model.state_dict())
	best_loss = 10e9

	for epoch in range(num_epochs):
		print('Epoch {}/{}'.format(epoch, num_epochs - 1))
		print('-' * 10)

		# Each epoch has a training and validation phase
		for phase in ['train', 'val']:
			if phase == 'train':
				model.train()  # Set model to training mode
			else:
				model.eval()  # Set model to evaluate mode

			running_loss = 0.0
			running_corrects = 0
			running_relative_err = 0.0
			running_diffs = 0.0
			# Iterate over data.
			for inputs, labels in dataloaders[phase]:
				inputs = inputs.to(device)
				labels = labels.to(device)
				# zero the parameter gradients
				optimizer.zero_grad()

				# forward
				# track history if only in train
				with torch.set_grad_enabled(phase == 'train'):
					outputs = model(inputs)
					# _, preds = torch.max(outputs, 1)
					loss = criterion(outputs, labels)
					# backward + optimize only if in training phase
					if phase == 'train':
						loss.backward()
						optimizer.step()

				# statistics
				running_loss += loss.item() * inputs.size(0)
				running_corrects += torch.sum(outputs == labels.data)
				# running_relative_err += relative_err(outputs, labels)
				# running_diffs += diffs_distance(outputs, labels)
				tmp, tmp2 = cmonfuzzy(outputs, labels)
				running_relative_err += tmp2
				running_diffs += tmp
			#			if phase == 'train':
			#				scheduler.step()

			epoch_loss = running_loss / dataset_sizes[phase]
			# epoch_acc = running_corrects.double() / dataset_sizes[phase]
			avg_distance = running_diffs.double() / dataset_sizes[phase]
			avg_relative = running_relative_err.double() / dataset_sizes[phase]
			# epoch_acc, avg_distance = accuracy(outputs, labels)
			print('{}\tLoss: {:.4f}\tAvg Relative error: {:.4f}\tAvg dist {:.4f}'.format(
				phase, epoch_loss, avg_relative, avg_distance))

			# deep copy the model
			if phase == 'train' and epoch_loss < best_loss:
				print(f"new best loss {epoch_loss:.4f}, previous best was {best_loss:.4f}")
				best_loss = epoch_loss
				best_model_wts = copy.deepcopy(model.state_dict())
			elif epoch % 50 == 0:
				torch.save(model.state_dict(), f"preventivka_epoch_{epoch}")

		print()

	time_elapsed = time.time() - since
	print('Training complete in {:.0f}m {:.0f}s'.format(
		time_elapsed // 60, time_elapsed % 60))
	# print('Best val Acc: {:4f}'.format(best_acc))

	# load best model weights
	model.load_state_dict(best_model_wts)
	return model


def main():
	print("creating new model")
	model = create_model()
	if osp.isfile("pokracuj_odsud"):
		print("loading model from save")
		model.load_state_dict(torch.load("pokracuj_odsud", map_location="cuda:3"))
	else:
		print("save not found")

	optimizer = optim.SGD(model.classifier[6].parameters(), lr=0.0001, momentum=0.9)
	exp_lr_scheduler = optim.lr_scheduler.StepLR(optimizer, step_size=7, gamma=0.1)

	dataset_train = Dataset("trenovaci")
	dataset_validate = Dataset("validacni")

	datasets = {"train": dataset_train, "val": dataset_validate}
	dataloaders = {x: tdata.DataLoader(datasets[x], batch_size=64, shuffle=True) for x in ["train", "val"]}
	dataset_sizes = {x: len(datasets[x]) for x in ["train", "val"]}

	model_ft = train_model(model, lossfunc, optimizer, exp_lr_scheduler, dataloaders, dataset_sizes, num_epochs=800)
	torch.save(model_ft.state_dict(), f"tak_se_ukaz_modele")


if __name__ == "__main__":
	main()
