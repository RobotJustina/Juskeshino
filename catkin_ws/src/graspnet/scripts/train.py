#!/usr/bin/env python3

import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
import torch.utils
import torch.utils.data
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

class GraspDataset(torch.utils.data.Dataset):
    def __init__(self, path, example_name, set_type):
        self.base_path = path
        self.example_name = example_name
        self.dataset_type = set_type 

    def __getitem__(self, index):
        file_name = self.example_name + str(index) +  ".p"
        data_path = os.path.join(self.base_path, file_name)
        data = np.load(data_path, allow_pickle=True)
        pcd = data['pcd']
        pcd = rf.structured_to_unstructured(pcd)
        points = torch.tensor(pcd[:,:3],dtype=torch.float32)
        pose = torch.tensor(data['grasp'],dtype=torch.float32)
        return points, pose


training_set = torch.utils.data.DataLoader()