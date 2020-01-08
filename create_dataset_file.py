import data_management as dm
from dataset import sample_size
import time
from PIL import Image
import numpy as np
import random


def main():
	sequence = list(range(sample_size))
	sequence = [str(x) for x in sequence]
	random.shuffle(sequence)

	num_train_data = len(sequence) * 0.7
	num_train_data = int(np.floor(num_train_data))
	num_test_data = len(sequence) * 0.85
	num_test_data = int(np.floor(num_test_data))

	trenovaci_seq = sequence[:num_train_data]
	validacni_seq = sequence[num_train_data:num_test_data]
	testovaci_seq = sequence[num_test_data:]

	# with open("trenovaci.txt", "w") as f:
	# 	print("\n".join(trenovaci_seq, file=f))
	#
	# with open("validacni.txt", "w") as f:
	# 	print("\n".join(validacni_seq, file=f))
	#
	# with open("testovaci.txt", "w") as f:
	# 	print("\n".join(testovaci_seq, file=f))

	mapa = {"trenovaci": trenovaci_seq, "validacni": validacni_seq, "testovaci": testovaci_seq}

	for phase in ["trenovaci", "validacni", "testovaci"]:
		print(f"creating: {phase} dataset with {len(mapa[phase])} items")

		with open(f"{phase}.txt", "w") as f:
			print("\n".join(mapa[phase]), file=f)
		images = []
		labels = []

		for i in mapa[phase]:
			tmp_data = dm.load_pickle(f"data{i}")
			images.append(tmp_data["image"])
			labels.append(tmp_data["camera_pos"] + tmp_data["camera_angle"])

		images = np.array(images)
		labels = np.array(labels)

		np.save(f"{phase}.npy", images)
		np.save(f"{phase}_labels.npy", labels)

	# for i in range(sample_size):
	# 	tmp_data = dm.load_pickle(f"data{i}")
	# 	print(tmp_data)
	# 	print(tmp_data["image"].shape)
	# 	images.append(tmp_data["image"])
	# 	labels.append(tmp_data["camera_pos"] + tmp_data["camera_angle"])
	# 	img = Image.fromarray(tmp_data["image"], "RGB")
	# 	img.show()
	# time.sleep(4)

if __name__ == "__main__":
	main()
