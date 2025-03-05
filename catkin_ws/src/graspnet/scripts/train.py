#!/usr/bin/env python3

import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
import torch.utils
import torch.utils.data
import torch.utils.data.dataloader
import torchvision
import numpy as np
import open3d
from grasp_network import GraspNetwork
import os
import numpy.lib.recfunctions as rf
import gc

#Gpu config
gpu_number = 1
gpus = 0
gpu_arr = '0'
BATCH_SIZE = 5
#np.random.seed(int(time.time()))
#torch.cuda.manual_seed(1)
#torch.cuda.set_device(gpus)

# Device configuration
DEVICE = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

#Dataset loaders

DATASET_PATH = "/home/robocup/Juskeshino/catkin_ws/src/graspnet/dataset"
VAL_TO_TEST_RATIO = 0.1

class GraspDataset(torch.utils.data.Dataset):
    def __init__(self, set_type, path = DATASET_PATH):
        self.base_path = path
        self.dataset_type = set_type
        p_path = os.listdir(self.base_path)
        p_path.sort()
        p_path = np.array(p_path)
        if self.dataset_type == "test":
            self.data_file_names = p_path
            print("test")
        if self.dataset_type == "validate":
            index = np.random.choice(len(p_path), int(len(p_path)*VAL_TO_TEST_RATIO), replace=False) 
            self.data_file_names = p_path[index]
            print("validate")
        print(self.data_file_names)

    def __getitem__(self, index):
        data_path = os.path.join(self.base_path, self.data_file_names[index])
        data = np.load(data_path, allow_pickle=True)
        pcd = data['pcd']
        pcd = rf.structured_to_unstructured(pcd)
        points = torch.tensor(pcd[:,:,:3],dtype=torch.float32)
        points = torch.permute(points,(2,1,0))
        pose = torch.tensor(data['grasp'],dtype=torch.float32)
        return points, pose
    
    def __len__(self):
        return len(self.data_file_names)

def get_dataloaders():
    train_dataset = GraspDataset(set_type="test",path=DATASET_PATH)
    train_loader = torch.utils.data.DataLoader(train_dataset, BATCH_SIZE, shuffle=True)

    valid_dataset = GraspDataset(set_type="validate",path=DATASET_PATH)
    valid_loader = torch.utils.data.DataLoader(valid_dataset, BATCH_SIZE, shuffle=True)

    return train_loader, valid_loader

def train_network(num_epochs):
    train_loader, valid_loader = get_dataloaders()
    model = GraspNetwork().to(DEVICE)
    criterion = nn.HuberLoss()
    optimizer = optim.SGD(model.parameters(),lr=0.001,momentum=0.8)
    for epoch in range(num_epochs):
    min_loss = 0
        for batch, (points, target_pose) in enumerate(train_loader):
            points = points.to(DEVICE)
            target_pose = target_pose.to(DEVICE)

            output_pose = model(points)
            loss = criterion(output_pose,target_pose)
            
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()
            
            del points, target_pose, output_pose
            torch.cuda.empty_cache()
            gc.collect()
            
        with torch.no_grad():
            for points, target_pose in valid_loader:
                error = 0
                points = points.to(DEVICE)
                target_pose = target_pose.to(DEVICE)
                output_pose = model(points)
                error = torch.sum(abs(output_pose - target_pose),dim=0) + error
                error = error/len(points)
                print("Average absolute error for this batch: ",error)
                
        print ('Epoch [{}/{}], Loss: {:.4f}'.format(epoch+1, num_epochs, loss.item()))
    

            
# Loss and optimizer definition

#criterion = nn.HuberLoss()
#optimzer = optim.SGD(GraspNetwork.parameters(),lr=0.001,momentum=0.8)

def main():
    train_network(10)
    # dataset = GraspDataset(set_type="test",path=DATASET_PATH)
    # dataloader = torch.utils.data.DataLoader(dataset, BATCH_SIZE, shuffle=True)
    # train_features, train_labels = next(iter(dataloader))
    # print(f"Feature batch shape: {train_features.size()}")
    # print(f"Labels batch shape: {train_labels.size()}")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass