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
import copy
from grasp_network import GraspNetwork
import os
import numpy.lib.recfunctions as rf
import gc
import matplotlib as plt

#Gpu config
gpu_number = 1
gpus = 0
gpu_arr = '0'
BATCH_SIZE = 250
#np.random.seed(int(time.time()))
#torch.cuda.manual_seed(1)
#torch.cuda.set_device(gpus)

# Device configuration
DEVICE = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

#Dataset loaders

DATASET_PATH = "/home/robocup/Juskeshino/catkin_ws/src/graspnet/dataset_test"
MODELS_PATH = "/home/robocup/Juskeshino/catkin_ws/src/graspnet/models/"
VAL_TO_TEST_RATIO = 0.1

class GraspDataset(torch.utils.data.Dataset):
    def __init__(self, set_type, path = DATASET_PATH, samples = -1):
        self.base_path = path
        self.dataset_type = set_type
        p_path = os.listdir(self.base_path)
        if samples > 0:
            p_path = p_path[:samples]
        p_path.sort()
        p_path = np.array(p_path)
        self.data_file_names = p_path
        if self.dataset_type == "test":
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
        points = torch.nan_to_num(points,nan=0.0)
        points = torch.permute(points,(2,1,0))
        pose = torch.tensor(data['grasp'],dtype=torch.float32)
        return points, pose
    
    def __len__(self):
        return len(self.data_file_names)

def get_dataloaders(samples =-1):
    train_dataset = GraspDataset(set_type="test",path=DATASET_PATH,samples=samples)
    train_loader = torch.utils.data.DataLoader(train_dataset, BATCH_SIZE, shuffle=True)

    valid_dataset = GraspDataset(set_type="validate",path=DATASET_PATH,samples=samples)
    valid_loader = torch.utils.data.DataLoader(valid_dataset, BATCH_SIZE, shuffle=True)

    return train_loader, valid_loader

def train_network(num_epochs,model_name, model_path=None,samples =-1):
    torch.cuda.empty_cache()
    gc.collect()
    train_loader, valid_loader = get_dataloaders(samples)
    model = load_model(model_path)
    best_model = copy.deepcopy(model.state_dict())
    criterion = nn.HuberLoss(delta=0.625)
    optimizer = optim.SGD(model.parameters(),lr=0.001,momentum=0.8)
    min_loss = 1.5
    for epoch in range(num_epochs):
        model.train()
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
        
        model.eval()    
        with torch.no_grad():
            for points, target_pose in valid_loader:
                error = 0
                points = points.to(DEVICE)
                target_pose = target_pose.to(DEVICE)
                output_pose = model(points)
                loss = criterion(output_pose,target_pose)
                error = torch.sum(abs(output_pose - target_pose),dim=0) + error
                error = error/len(points)
                print("Average absolute error for this batch: ",error)
            if loss < min_loss:
                min_loss = loss
                best_model = copy.deepcopy(model.state_dict())       
        print ('Epoch [{}/{}], Loss: {:.4f}'.format(epoch+1, num_epochs, loss.item()))
    model_file = MODELS_PATH + model_name
    torch.save(best_model,model_file)
    print(min_loss)

def load_model(model_path=None):
    model = GraspNetwork()
    if model_path:
        model.load_state_dict(torch.load(model_path,weights_only=True))
    model.to(DEVICE)
    return model

def main():
    model_file = MODELS_PATH + 'model_nn.pt'
    train_network(70,"model_nn_5.pt",model_path=model_file)
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