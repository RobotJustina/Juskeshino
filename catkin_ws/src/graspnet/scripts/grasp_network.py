#!/usr/bin/env python3

import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
import numpy as np
import open3d

#Gpu config
gpu_number = 1
gpus = 0
gpu_arr = '0'
np.random.seed(int(time.time()))
torch.cuda.manual_seed(1)
torch.cuda.set_device(gpus)

# Gripper and pointcloud properties


#Model construction from saved model


#Load model to gpu



#Network definition

class GraspNetwork():
    def __init__(self):
        pass