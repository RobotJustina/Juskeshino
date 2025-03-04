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
import grasp_network
import os
import numpy.lib.recfunctions as rf

#Gpu config
gpu_number = 1
gpus = 0
gpu_arr = '0'
BATCH_SIZE = 5
#np.random.seed(int(time.time()))
#torch.cuda.manual_seed(1)
torch.cuda.set_device(gpus)

#Dataset loaders

DATASET_PATH = "/home/robocup/Juskeshino/catkin_ws/src/graspnet/dataset"

class GraspDataset(torch.utils.data.Dataset):
    def __init__(self, example_name, set_type, path = DATASET_PATH):
        self.base_path = path
        self.example_name = example_name
        self.dataset_type = set_type
        if self.dataset_type == "test":
            print("test")
        p_path = os.listdir(self.base_path)
        p_path.sort()
        self.data_file_names = np.array(p_path)
        print(self.data_file_names)

    def __getitem__(self, index):
        data_path = os.path.join(self.base_path, self.data_file_names[index])
        data = np.load(data_path, allow_pickle=True)
        pcd = data['pcd']
        pcd = rf.structured_to_unstructured(pcd)
        points = torch.tensor(pcd[:,:,:3],dtype=torch.float32)
        pose = torch.tensor(data['grasp'],dtype=torch.float32)
        return points, pose
    
    def __len__(self):
        return len(self.data_file_names)

def main():
    dataset = GraspDataset(set_type="test",example_name="example")
    dataloader = torch.utils.data.DataLoader(dataset, BATCH_SIZE, shuffle=True)
    train_features, train_labels = next(iter(dataloader))
    print(f"Feature batch shape: {train_features.size()}")
    print(f"Labels batch shape: {train_labels.size()}")
    #pts, pose = dataset[1]
    #print(pts, pose)
    #print(dataset)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass