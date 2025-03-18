#!/usr/bin/env python3

import rospkg
import torch
import torchvision.transforms as transforms
from torch.utils.data import TensorDataset, DataLoader
from torch.optim import SGD, Adam
from sklearn.model_selection import train_test_split
import numpy as np
import sys
import rospy
import utils.utilities as l_util 
import utils.models as nn_models
from datetime import datetime
import time

import matplotlib.pyplot as plt
# """
# Enable GPU mode
# """
if torch.cuda.is_available():
    device = torch.device('cuda:0')
    print("device", device, "name: ", torch.cuda.get_device_name())
else:
    device = torch.device('cpu')


# """
# Dataset
# """
np.set_printoptions(threshold=sys.maxsize)

# --- load files ---
pkg_name = 'mapless_nav'
pkg_path = rospkg.RosPack().get_path(pkg_name)
files_path = pkg_path + '/scripts/TorchModels/data/'
files_path += "*.npz"

data = l_util.load_data(files_path)
print("Dataset samples:", data.shape)
# --- prepare X(data), Y(labels) ---
# keys: 'occ_grid', 'target', 'labels'
data_Y = []
data_X = []
x_ch_arr = []
ch_count = 0


# PARAM <stack_n_channels>: Modify channels number
stack_n_channels = 5
# PARAM <meters>: Modify matrix resolution number
# 1m = 20 pixels, max 8m
meters = 4.0

sub_rows = int(20*meters)
print("-- >")
matrix_shape = data[0].get('features').get('occ_grid').shape
print(f"Dataset matrix shape {matrix_shape}, range {matrix_shape[0]/20}[m]")
print(f"Shape selected: ({sub_rows}, {sub_rows}), range {meters}[m]")
img_center = matrix_shape[0]//2

for info in data:
    """
    data = dict{'features':{occ_grid:[nxn], 'target':[2]}, 'labels':[3]}
    # occ_grid dim(nxn) float32
    # target dim(2) = (distance_to_target, theta_to_target) float32
    # labels vect dim(3) = (l_vel_x, l_vel_y, a_vel_z) float32
    """
    features = info.get('features')
    #print(features)
    occ_grid = features.get('occ_grid')
    target = features.get('target')
    rows = occ_grid.shape[0]
    
    if sub_rows <= matrix_shape[0]:
        target = np.array([np.ones(sub_rows)*target[0], np.ones(sub_rows)*target[1]], dtype=np.float32)
        #print("occ_grid", occ_grid.shape)
        #print("target", target.shape)
        min_indx = img_center-sub_rows//2
        max_indx = img_center+sub_rows//2
        occ_grid = occ_grid[matrix_shape[0]-sub_rows:, min_indx:max_indx]
        #print("sampled_occ_grid", occ_grid.shape)
    else:
        cad = f"Can not rescale shape from {matrix_shape} to ({sub_rows}, {sub_rows})"
        rospy.logwarn(cad)
        target = np.array([np.ones(rows)*target[0], np.ones(rows)*target[1]], dtype=np.float32)

    features = np.vstack((occ_grid, target))
    features = np.array(features, dtype=np.float32)
    x_ch_arr.append(features)
    ch_count += 1
    
    if ch_count == stack_n_channels:
        # stack x
        x_ch_arr = np.array(x_ch_arr, dtype=np.float32)

        # TODO: delete
        # print("type x_ch_arr", type(x_ch_arr[0][0]))
        # print("shape x_ch_arr", x_ch_arr.shape)
        # print(x_ch_arr)
        # print()

        data_X.append(x_ch_arr)
        # batch takes last y val 
        data_Y.append(info.get('labels'))
        x_ch_arr = []
        ch_count = 0

data_Y = np.array(data_Y, dtype=np.float32)
data_X = np.array(data_X, dtype=np.float32)

# ignore lin_vel_y (move lateral) ---
lvel_x = data_Y[:, 0]
Avel_z = data_Y[:, 2]
data_Y = np.stack((lvel_x, Avel_z), axis=1)

print("Data_X shape:", data_X.shape)
print("Data_Y shape:", data_Y.shape)

# TODO: Delete
#np.savez(pkg_path + '/scripts/TorchModels/x_ch_dat.npz', data=data_X)
# show random sample
l_util.show_image_gray(data_X[np.random.randint(0,len(data_X)), 0])


# """
# Normalization
# """
# PARAM <normalize_label>: normalize velocity range
normalize_label = True
if normalize_label:
    # Normalization angular velocity range [-1, 1]
    data_Y[:, 1] = (data_Y[:, 1] - np.amin(data_Y[:, 1])) / np.ptp(data_Y[:, 1])
    data_Y[:, 1] = (data_Y[:, 1]*2) -1
    # normalization lin_vel_x to range[0, 1], data_Y[:, 0]
    data_Y[:, 0] = (data_Y[:, 0] - np.amin(data_Y[:, 0])) / np.ptp(data_Y[:, 0])

# Norm y
# for y in data_Y:
#     print(y)
# print("sample", data_Y.shape)
# ymin = np.amin(data_Y[:, 0])
# ymax = np.amax(data_Y[:, 0])
# print("lin_vel_x range: ", ymin, ymax)
# ymin = np.amin(data_Y[:, 1])
# ymax = np.amax(data_Y[:, 1])
# print("ang_vel_z range: ", ymin, ymax)

normalize_data = False
if normalize_data:
    print("normalize X")
    for i in range(len(data_X)):
        data_X[i, :stack_n_channels, :-2]= data_X[i, :stack_n_channels, :-2]/100
        # print(type(data_X[i]))
        # print(data_X[i])
        # print(data_X[i].shape)
        # print()



# Split data
X_train, X_val , y_train, y_val= train_test_split(data_X, data_Y, train_size=0.8, shuffle=True)
#X_val, X_test, y_val, y_test= train_test_split(X_train, y_train, train_size=0.5, shuffle=True)

X_train = torch.tensor(X_train, dtype=torch.float32, device=device)
X_val = torch.tensor(X_val, dtype=torch.float32, device=device)
#X_test = torch.tensor(X_test, dtype=torch.float32, device=device)
y_train = torch.tensor(y_train, dtype=torch.float32, device=device)
y_val = torch.tensor(y_val, dtype=torch.float32, device=device)
#y_test = torch.tensor(y_test, dtype=torch.float32, device=device)


# """
# Hyperparameters
# """
batch_size = 16
learn_r = 0.0001 # 1e-3
epochs = 15


# """
# Model
# """
model = nn_models.NN_82_80()
model.to(device)

optimizer = Adam(model.parameters(), lr=learn_r)
loss_fn = torch.nn.MSELoss() # CrossEntropyLoss()
##torch.nn.L1Loss()

"""
Training
"""

train_loader = DataLoader(TensorDataset(X_train, y_train), batch_size=batch_size, shuffle=True)
valid_loader = DataLoader(TensorDataset(X_val, y_val), batch_size=batch_size, shuffle=True)
#test_loader = DataLoader(TensorDataset(X_test, y_test), batch_size=batch_size, shuffle=True)

# Initializing in a separate cell so we can easily add more epochs to the same run
timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')

t0 = time.time()
best_vloss = 1_000_000.
print("\nTraining model", model.name)
for epoch in range(epochs):
    print(f'Epoch: {epoch+1}/{epochs}')

    # Make sure gradient tracking is on, and do a pass over the data
    model.train(True)
    running_loss = 0.
    train_loss = 0.

    for i, t_data in enumerate(train_loader):
        # Every data instance is an input + label pair
        x_inputs, y_labels = t_data[0].to(device), t_data[1].to(device)
        # Zero your gradients for every batch!
        optimizer.zero_grad()
        # Make predictions for this batch
        predictions = model(x_inputs)
        # Compute the loss and its gradients
        loss_val = loss_fn(predictions, y_labels)
        loss_val.backward()
        # Adjust learning weights
        optimizer.step()
        # Gather data and report
        running_loss += loss_val.item()
        if i % (batch_size*10) == (batch_size*10)-1:
            train_loss = running_loss / i # loss per batch
            print('   batch {} loss: {:.6f}'.format(i + 1, train_loss), end='\r')
            tb_x = epoch * len(train_loader) + i + 1
            running_loss = 0.
    print()

    running_vloss = 0.0
    # Set the model to evaluation mode, disabling dropout and using population
    # statistics for batch normalization.
    model.eval()

    # Disable gradient computation and reduce memory consumption.
    with torch.no_grad():
        for i, vdata in enumerate(valid_loader):
            #vinputs, vlabels = vdata 
            vinputs, vlabels = vdata[0].to(device), vdata[1].to(device)
            voutputs = model(vinputs)
            vloss = loss_fn(voutputs, vlabels)
            running_vloss += vloss

    avg_vloss = running_vloss /  (i + 1)
    print('   train loss: {:.6f}, valid loss: {:.6f}'.format(train_loss, avg_vloss))
    # Track best performance, and save the model's state
    if avg_vloss < best_vloss:
        best_vloss = avg_vloss
        model_path = 'model_{}_{}'.format(timestamp, epoch)
        ##torch.save(model.state_dict(), model_path)

tf = time.time()
from datetime import timedelta
t =  str(timedelta(seconds=tf - t0))[:-4]
print('\nTraining completed in:', t)

"""
Save model info
"""
# # --- save the model ---
save_path = './'+ model.name + '.pth'
torch.save(model.state_dict(), save_path)


"""
Evaluation
"""

data_train_iter = iter(train_loader)
images, labels = next(data_train_iter)
model.eval()

correct = 0
total = 0
with torch.no_grad():
    for data in train_loader:
        images, labels = data
        pred_out = model(images)
        _, predicted = torch.max(pred_out.data, 0)
        total += labels.size(0)
        correct += (predicted == labels).sum().item()

print(f'Accuracy train: {100 * correct // total}%')

