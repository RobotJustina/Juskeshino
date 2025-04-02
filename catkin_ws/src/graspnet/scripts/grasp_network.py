#!/usr/bin/env python3

import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
import torchvision
import geotorch as geo
import numpy as np
import copy
import numpy.lib.recfunctions as rf
import ros_numpy
from geometry_msgs.msg import Point
from scipy.spatial import cKDTree
import math

DEVICE = torch.device('cuda' if torch.cuda.is_available() else 'cpu')


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
                            #nn.Dropout2d(0.05),
                            nn.Conv2d(3, 32, kernel_size = 11, stride = 1),
                            nn.BatchNorm2d(32),
                            #nn.GELU()
                            )
        self.maxpool = nn.MaxPool2d(kernel_size = 2, stride = 2, padding = 0)
        self.conv2 = nn.Sequential(
                            nn.Conv2d(32, 64, kernel_size = 6, stride = 1),
                            nn.BatchNorm2d(64),
                            nn.ReLU())
        self.conv3 = nn.Sequential(
                            nn.Conv2d(64, 128, kernel_size = 4, stride = 2),
                            nn.BatchNorm2d(128),
                            nn.ReLU())
        self.conv4 = nn.Sequential(
                            nn.Conv2d(128, 256, kernel_size = 4, stride = 2),
                            nn.BatchNorm2d(256),
                            nn.ReLU())
        self.conv5 = nn.Sequential(
                            nn.Conv2d(256, 512, kernel_size = 3, stride = 2),
                            nn.BatchNorm2d(512),
                            nn.ReLU(),)
                            #nn.Dropout2d(0.1))
        self.conv6 = nn.Sequential(
                            nn.Conv2d(512, 512, kernel_size = 2, stride = 2, padding=1),
                            nn.BatchNorm2d(512),
                            #nn.GELU()
                            )
        self.maxpool2 = nn.MaxPool2d(kernel_size=2,stride=1,padding=0)
        self.lc  = nn.Sequential(
                            nn.Linear(5*5*512,8192),
                            nn.ReLU(),
                            nn.Dropout(0.2),
                            nn.Linear(8192,2048),
                            # nn.ReLU(),
                            # nn.Linear(4096, 2048),
                            nn.ReLU(),
                            nn.Linear(2048,512),
                            nn.ReLU(),
                            nn.Dropout(0.2),
                            nn.Linear(512,256),
                            nn.Linear(256,256),
                            nn.Linear(256,3))
                            #nn.Linear(64,3))
        self.nlc = nn.Sequential(
                            nn.Linear(5*5*512,8192),
                            nn.ReLU(),
                            nn.Dropout(0.2),
                            nn.Linear(8192,2048),
                            # nn.ReLU(),
                            # nn.Linear(4096, 2048),
                            nn.ReLU(),
                            nn.Linear(2048, 512),
                            nn.ReLU(),
                            nn.Dropout(0.2),
                            nn.Linear(512,512),
                            nn.ReLU(),
                            nn.Linear(512,256),
                            nn.ReLU(),
                            nn.Linear(256,4))
                            # nn.ReLU(),
                            # nn.Linear(64,4),
                            #geo.Sphere()
                            #nn.Tanh())
        #self.nlch = nn.Linear(256,4)
        #ten = getattr(self.nlch,)
        #self.manif = geo.Sphere(ten.size(),1)
        self.manif = geo.Sphere([1,4],1)
        self.manif.base = torch.tensor([0,0,0,1],dtype=torch.float32)
        #self.manif = geo.SphereEmbedded(self.nlc,1)
        #self.nlch = nn.Linear(256,4)
        #geo.sphere(self.nlch, tensor_name="output")
        #geo.SphereEmbedded(self.nlc,1)
        #geo.orthogonal()
        # self.fc1 = nn.Linear(5*5*512,8192)
        # self.fc2 = nn.Linear(8192, 2048)
        # self.fc3 = nn.Linear(2048,512)
        # self.fc4 = nn.Linear(512,64)
        # self.fc5 = nn.Linear(64,7)

    def forward(self, x):
        x = self.conv1(x)
        x = self.maxpool(x)
        x = F.gelu(x)
        x = self.conv2(x)
        x = self.conv3(x)
        x = self.conv4(x)
        x = self.conv5(x)
        x = self.conv6(x)
        x = self.maxpool2(x)
        x = F.gelu(x)
        x = torch.flatten(x,1)

        pos = self.lc(x)
        ori = self.nlc(x)
        ori = self.manif(ori)
        # x = self.fc1(x)
        # #x = F.tanh(x)
        # x = self.fc2(x)
        # #x = F.tanh(x)
        # x = self.fc3(x)
        # #x = F.tanh(x)
        # x = self.fc4(x)
        # #x = F.tanh(x)
        # x = self.fc5(x)
        # #x = F.tanh(x)

        return pos, ori

# Pointcloud conversion methods
def ros_pc2_to_npmatrix(pc):
    print(pc.header.frame_id)
    data = ros_numpy.point_cloud2.pointcloud2_to_array(pc)
    #debug_type(data,"Split points")
    rgb = ros_numpy.point_cloud2.split_rgb_field(data)
    #debug_type(rgb,"Split points + RGB")
    dt2 = np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('r', '<f4'), ('g', '<f4'), ('b', '<f4')])
    rgb = np.asarray(rgb).astype(dt2)
    rgb['r'] = np.divide(rgb['r'], 255)
    rgb['g'] = np.divide(rgb['g'], 255)
    rgb['b'] = np.divide(rgb['b'], 255)
    #debug_type(rgb, "Split points + Normalized RGB")
    return rgb

def npmatrix_to_torch(pcd):
    points = rf.structured_to_unstructured(pcd)
    points = torch.tensor(points[:,:,:3],dtype=torch.float32)
    points = torch.permute(points,(2,1,0))
    points.to(DEVICE)
    return points

def camera_link_to_optical_frame(pt):
    target_pt = Point()
    target_pt.x = -pt.y
    target_pt.y = -pt.z
    target_pt.z = pt.x
    return target_pt

def find_nearest_pt_in_pc(pc, pt):
    valid = False
    matrix = copy.deepcopy(pc)
    matrix = matrix.reshape(-1)
    matrix = rf.structured_to_unstructured(matrix)
    search_vec = np.array([pt.x, pt.y, pt.z])
    nearest = cKDTree(matrix[:,:3]).query(search_vec, k=1)[1]
    u = math.floor(nearest/pc.shape[1])
    v = nearest%pc.shape[1]
    if 100 < u < 380 and 100 < v < 540: valid = True
    print(u,v,valid)
    return u,v,valid

def cut_pc(u,v, pc):
    l, w = 200, 200
    cropped_pc = pc[(u - int(l/2)): (u + int(l/2)) , (v - int(l/2)) : (v + int(l/2))]
    return cropped_pc