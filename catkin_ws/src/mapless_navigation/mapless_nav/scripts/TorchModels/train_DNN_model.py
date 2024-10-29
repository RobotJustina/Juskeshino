#! /usr/bin/env python3

import rospkg
import torchvision.transforms as transforms
import torch

import numpy as np

import utils.utilities as l_util 

from sklearn.model_selection import train_test_split
from torch.utils.data import TensorDataset, DataLoader
from torch.optim import SGD, AdamW

import time

# t0 = time.time()
# print("hello")
# tf = time.time()
# print(tf - t0)

# Enable GPU mode
if torch.cuda.is_available():
    device = torch.device('cuda:0')
    print("device", device, "name: ", torch.cuda.get_device_name())
else:
    device = torch.device('cpu')


# --- load files ---
pkg_name = 'machine_learning' #'mapless_nav'
pkg_path = rospkg.RosPack().get_path(pkg_name)
files_path = pkg_path + '/src/Data_gazebo/'
data = l_util.load_data(files_path)
print("Data shape:", data.shape)

data_X = []
data_Y = []
data = l_util.removePattern(data, (data[:,6402]==0.) & (data[:, 6403] == 0.))
for info in data:
    image = np.reshape(info[:-4], (80, 80))
    data_X.append([image, info[-4:-2]])  # img(80x80), vet(2)
    data_Y.append(info[-2:])

data_X = np.asarray(data_X, dtype=object)
data_Y = np.asarray(data_Y, dtype=np.float32)

# show random sample
#show_image_gray(data_X[np.random.randint(0,len(data_X)), 0])

# --- prepare X(data), Y(labels) ---
print(data_Y.shape)
un = np.unique(data_Y, axis=0, return_counts=True)
print(un)



"""
Parameters
"""
bathch_size = 4


#train_test_split()