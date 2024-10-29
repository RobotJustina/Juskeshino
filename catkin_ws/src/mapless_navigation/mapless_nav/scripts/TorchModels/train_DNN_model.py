#! /usr/bin/env python3

import rospkg
import torchvision.transforms as transforms
import torch

import numpy as np

from utils.utilities import load_data, show_image_gray

from sklearn.model_selection import train_test_split
from torch.utils.data import TensorDataset, DataLoader
from torch.optim import SGD, AdamW

import time

t0 = time.time()
print("hello")
tf = time.time()
print(tf - t0)

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
data = load_data(files_path)
print("Data shape:", data.shape)

images = []
vectors = []
for info in data:
    image = np.reshape(info[:-4], (80, 80))
    images.append(image) # 80x80 image
    vectors.append(info[-4:])
    

images = np.asarray(images)
vectors = np.asarray(vectors)
# show random sample
show_image_gray(images[np.random.randint(0,len(images))])



"""
Parameters
"""
bathch_size = 4


train_test_split()