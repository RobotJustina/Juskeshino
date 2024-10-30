#! /usr/bin/env python3

import rospkg
import torchvision.transforms as transforms
import torch

from sklearn.model_selection import train_test_split
from torch.utils.data import TensorDataset, DataLoader
from torch.optim import SGD, AdamW

import numpy as np
import sys

import utils.utilities as l_util 
import utils.models as nn_models

import time

np.set_printoptions(threshold=sys.maxsize)


# print("hello")
# tf = time.time()
# print(tf - t0)

# Enable GPU mode
if torch.cuda.is_available():
    device = torch.device('cuda:0')
    print("device", device, "name: ", torch.cuda.get_device_name())
else:
    device = torch.device('cpu')


"""
Dataset
"""
# --- load files ---
pkg_name = 'machine_learning' #'mapless_nav'
pkg_path = rospkg.RosPack().get_path(pkg_name)
files_path = pkg_path + '/src/Data_gazebo/'
data = l_util.load_data(files_path)
print("Data shape:", data.shape)

# --- prepare X(data), Y(labels) ---
data_X = []
data_Y = []
data = l_util.removePattern(data, (data[:,6402]==0.) & (data[:, 6403] == 0.))
count = 0
for info in data:
    image = np.reshape(info[:-4], (80, 80))
    vect = np.zeros((80, 80))
    vect[0, :2] = info[-4:-2]
    #data_X.append([image, info[-4:-2]])  # img(80x80), vet(2)
    data_X.append([image, vect])  # img(80x80) ch0, vet(2) ch2
    data_Y.append(info[-2:])

data_X = np.asarray(data_X, dtype=np.float32)
data_Y = np.asarray(data_Y, dtype=np.float32)
print("Data_X shape:", data_X.shape)

## print(data_X[0,1]) 
# show random sample
#l_util.show_image_gray(data_X[np.random.randint(0,len(data_X)), 0])

# CLASSIFICATION problem
y_one_hot = l_util.one_hot_encode(data_Y, verbose=False)
y_one_hot = np.asarray(y_one_hot, dtype=np.float32)

# Normalization
data_X[:,0] = data_X[:,0] /100.
data_X = np.array(data_X)
# Split data
X_train, X_val , y_train, y_val= train_test_split(data_X, y_one_hot, train_size=0.7, shuffle=True)
X_val, X_test, y_val, y_test= train_test_split(X_train, y_train, train_size=0.5, shuffle=True)

X_train = torch.tensor(X_train, dtype=torch.float32, device=device)
X_val = torch.tensor(X_val, dtype=torch.float32, device=device)
X_test = torch.tensor(X_test, dtype=torch.float32, device=device)
y_train = torch.tensor(y_train, dtype=torch.float32, device=device)
y_val = torch.tensor(y_val, dtype=torch.float32, device=device)
y_test = torch.tensor(y_test, dtype=torch.float32, device=device)




"""
Hyperparameters
"""
batch_size = 16
learn_r = 4e-6 # 1e-3
epochs = 2


"""
Model
"""
model = nn_models.CNN_B()
model.to(device)

optimizer = AdamW(model.parameters(), lr=learn_r)
loss = torch.nn.MSELoss() # CrossEntropyLoss()

"""
Training
"""

train_loader = DataLoader(TensorDataset(X_train, y_train), batch_size=batch_size, shuffle=True)
valid_loader = DataLoader(TensorDataset(X_val, y_val), batch_size=batch_size, shuffle=True)
test_loader = DataLoader(TensorDataset(X_test, y_test), batch_size=batch_size, shuffle=True)

t0 = time.time()
for epoch in range(epochs):
    running_loss = 0.0
    for i, t_data in enumerate(train_loader, 0):
        x_inputs, y_labels = t_data[0].to(device), t_data[1].to(device)
        optimizer.zero_grad()

        predictions = model(x_inputs)
        loss_val = loss(predictions, y_labels)
        loss_val.backward()
        optimizer.step()

        running_loss += loss_val.item()
        if i % 2000 == 1999:
            print(f'[{epoch + 1}, {i + 1:5d}] loss: {running_loss / 2000:.3f}')
            running_loss = 0.0

print('Finished Training')