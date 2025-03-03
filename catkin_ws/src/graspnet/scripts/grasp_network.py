#!/usr/bin/env python3

import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
import torchvision
import numpy as np
import open3d


# Gripper and pointcloud properties



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

class ResidualGraspNetwork(nn.Module):
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

class GraspNetwork(nn.Module):
    def __init__(self):
        super().__init__()
        self.conv1 = nn.Sequential(
                            nn.Conv2d(3, 32, kernel_size = 11, stride = 1),
                            nn.BatchNorm2d(32),
                            nn.ReLU())
        self.maxpool = nn.MaxPool2d(kernel_size = 2, stride = 2, padding = 0)
        self.conv2 = nn.Sequential(
                            nn.Conv2d(3, 64, kernel_size = 6, stride = 1),
                            nn.BatchNorm2d(64),
                            nn.ReLU())
        self.conv3 = nn.Sequential(
                            nn.Conv2d(3, 128, kernel_size = 4, stride = 2),
                            nn.BatchNorm2d(128),
                            nn.ReLU())
        self.conv4 = nn.Sequential(
                            nn.Conv2d(3, 256, kernel_size = 4, stride = 2),
                            nn.BatchNorm2d(256),
                            nn.ReLU())
        self.conv5 = nn.Sequential(
                            nn.Conv2d(3, 512, kernel_size = 3, stride = 2),
                            nn.BatchNorm2d(512),
                            nn.ReLU())
        self.conv6 = nn.Sequential(
                            nn.Conv2d(3, 512, kernel_size = 2, stride = 1, padding=2),
                            nn.BatchNorm2d(512),
                            nn.ReLU())
        self.maxpool2 = nn.MaxPool2d(kernel_size=2,stride=1,padding=0)
        self.fc1 = nn.Linear(5*5*512,8192)
        self.fc2 = nn.Linear(8192, 2048)
        self.fc3 = nn.Linear(2048,512)
        self.fc4 = nn.Linear(512,64)
        self.fc5 = nn.Linear(64,6)

    def forward(self, x):
        x = self.conv1(x)
        x = self.maxpool(x)
        x = self.conv2(x)
        x = self.conv3(x)
        x = self.conv4(x)
        x = self.conv5(x)
        x = self.conv6(x)
        x = self.maxpool2(x)
        x = self.fc1(x)
        x = self.fc2(x)
        x = self.fc3(x)
        x = self.fc4(x)
        x = self.fc5(x)

        return x

# Loss and optimizer definition

criterion = nn.HuberLoss()
optimzer = optim.SGD(GraspNetwork.parameters(),lr=0.001,momentum=0.8)