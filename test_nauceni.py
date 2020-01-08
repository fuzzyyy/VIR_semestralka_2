import torch
import data_management as d
import net
import numpy as np


state_dict = torch.load("model9000", map_location="cuda:3")
device = torch.device("cuda:3")
model = net.create_model()
model.load_state_dict(state_dict)
model.to(device)
data = d.load_pickle("data999999")

model.eval()
image = np.array(data["image"], dtype="f4")
print(image.shape)
image = image.transpose((2, 0, 1))
image = torch.from_numpy(image)

image = image.to(device)
image = image.reshape((1, 3, 224, 224))
print(image.shape)
out = model(image)
print(f"output is: {out}")
print(f'labels were: {data["camera_pos"]}, {data["camera_angle"]}')
label = torch.from_numpy(np.array(data["camera_pos"] + data["camera_angle"], dtype="f4").reshape((1, 6)))
label = label.to(device)
print(out)
print(label)
err, dist = net.accuracy(out, label)
print(err)
print(dist)
