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
BATCH_SIZE = 5
np.random.seed(int(time.time()))
torch.cuda.manual_seed(1)
torch.cuda.set_device(gpus)

# Gripper and pointcloud properties


#Dataset loaders



#Model construction from saved model


#Load model to gpu



#Network definition

class ResidualBlock(nn.Module):
        def __init__(self, in_channels, out_channels, stride = 1, downsample = None):
            super(ResidualBlock, self).__init__()
            self.conv1 = nn.Sequential(
                            nn.Conv2d(in_channels, out_channels, kernel_size = 3, stride = stride, padding = 1),
                            nn.BatchNorm2d(out_channels),
                            nn.ReLU())
            self.conv2 = nn.Sequential(
                            nn.Conv2d(out_channels, out_channels, kernel_size = 3, stride = 1, padding = 1),
                            nn.BatchNorm2d(out_channels))
            self.downsample = downsample
            self.relu = nn.ReLU()
            self.out_channels = out_channels

        def forward(self, x):
            residual = x
            out = self.conv1(x)
            out = self.conv2(out)
            if self.downsample:
                residual = self.downsample(x)
            out += residual
            out = self.relu(out)
            return out

class GraspNetwork(nn.Module):
    def __init__(self):
        super().__init__()
        self.conv1 = nn.Sequential(
                            nn.Conv2d(6, 32, kernel_size = 3, stride = 1),
                            nn.BatchNorm2d(32),
                            nn.ReLU())
        self.maxpool = nn.MaxPool2d(kernel_size = 3, stride = 2, padding = 1)
        self.reslayer1 = self._make_layer(block, 64, layers[0], stride = 1)
        self.reslayer2 = self._make_layer(block, 128, layers[1], stride = 2)
        self.reslayer3 = self._make_layer(block, 256, layers[2], stride = 2)
        self.reslayer4 = self._make_layer(block, 512, layers[3], stride = 2)
        self.fc1 = nn.Linear(512*3*3,1024)
        self.fc2 = nn.Linear(1024,128)
        self.fc3 = nn.Linear(128,6)
        
    def _make_residual_layer(self, block, planes, blocks, stride=1):
            downsample = None
            if stride != 1 or self.inplanes != planes:
                downsample = nn.Sequential(
                    nn.Conv2d(self.inplanes, planes, kernel_size=1, stride=stride),
                    nn.BatchNorm2d(planes),
                )
            layers = []
            layers.append(block(self.inplanes, planes, stride, downsample))
            self.inplanes = planes
            for i in range(1, blocks):
                layers.append(block(self.inplanes, planes))

            return nn.Sequential(*layers)
        
