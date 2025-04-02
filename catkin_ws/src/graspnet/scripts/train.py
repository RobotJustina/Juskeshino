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
import matplotlib.pyplot as plt
import math

#Gpu config
gpu_number = 1
gpus = 0
gpu_arr = '0'
BATCH_SIZE = 200
#np.random.seed(int(time.time()))
#torch.cuda.manual_seed(1)
#torch.cuda.set_device(gpus)

# Device configuration
DEVICE = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

#Dataset loaders

DATASET_PATH = "/home/robocup/Juskeshino/catkin_ws/src/graspnet/training_dataset/"
MODELS_PATH = "/home/robocup/Juskeshino/catkin_ws/src/graspnet/models/"
GRAPHS_PATH = "/home/robocup/Juskeshino/catkin_ws/src/graspnet/graphs/"
VAL_DATASET_PATH = "/home/robocup/Juskeshino/catkin_ws/src/graspnet/validate_dataset/"
VAL_TO_TEST_RATIO = 1
y_loss = {}  # loss history
y_loss['train'] = []
y_loss['val'] = []
y_err = {}
y_err['train'] = []
y_err['val'] = []
x_epoch = []

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
        # x,y,z,ox,oy,oz,w = data['grasp']
        # ang = math.acos(w)
        # c = w
        # s = math.sin(ang)
        # ox = ox/s
        # oy = oy/s
        # oz = oz/s
        # pose = torch.tensor([x,y,z,ox,oy,oz,c,s],dtype=torch.float32)
        return points, pose
    
    def __len__(self):
        return len(self.data_file_names)

def get_dataloaders(samples =-1):
    train_dataset = GraspDataset(set_type="test",path=DATASET_PATH,samples=samples)
    train_loader = torch.utils.data.DataLoader(train_dataset, BATCH_SIZE, shuffle=True)

    valid_dataset = GraspDataset(set_type="validate",path=VAL_DATASET_PATH,samples=samples)
    valid_loader = torch.utils.data.DataLoader(valid_dataset, BATCH_SIZE, shuffle=True)

    return train_loader, valid_loader

def draw_curve(current_epoch):
    fig = plt.figure()
    ax0 = fig.add_subplot(121, title="loss")
    ax1 = fig.add_subplot(122, title="top1err")
    #x_epoch.append(current_epoch)
    ax0.plot(x_epoch, y_loss['train'], 'bo-', label='train')
    ax0.plot(x_epoch, y_loss['val'], 'ro-', label='val')
    ax1.plot(x_epoch, y_loss['train'], 'bo-', label='train')
    ax1.plot(x_epoch, y_loss['val'], 'ro-', label='val')
    if current_epoch == 0:
        ax0.legend()
        ax1.legend()
    fig.savefig(os.path.join(GRAPHS_PATH, 'train.jpg'))

def train_network(num_epochs,model_name, model_path=None,samples =-1):
    torch.cuda.empty_cache()
    gc.collect()
    train_loader, valid_loader = get_dataloaders(samples)
    train_size = len(train_loader.dataset)
    val_size = len(valid_loader.dataset)
    model = load_model(model_path)
    best_model = copy.deepcopy(model.state_dict())
    criterion = nn.HuberLoss(delta=0.9)
    min_loss = 15000
    #optimizer = optim.SGD(model.parameters(),lr=0.00008,momentum=0.8)
    optimizer = optim.Adam(model.parameters(),lr=0.00008)
    for epoch in range(num_epochs):
        trunning_loss = 0
        vrunning_loss = 0
        model.train()
        for batch, (points, target_pose) in enumerate(train_loader):
            points = points.to(DEVICE)
            target_pose = target_pose.to(DEVICE)

            #output_pose = model(points)
            pos, ori = model(points)
            #loss = criterion(output_pose,target_pose)
            ploss = criterion(pos,target_pose[:,:3])
            oloss = criterion(ori,target_pose[:,3:])
            tloss = ploss + 2*oloss

            optimizer.zero_grad()
            tloss.backward()
            optimizer.step()
            
            trunning_loss += tloss.item() * points.shape[0]

            #del points, target_pose, output_pose
            del points, target_pose, pos, ori
            torch.cuda.empty_cache()
            gc.collect()
        
        model.eval()    
        with torch.no_grad():
            for points, target_pose in valid_loader:
                error = 0
                points = points.to(DEVICE)
                target_pose = target_pose.to(DEVICE)
                # output_pose = model(points)
                # loss = criterion(output_pose,target_pose)
                pos, ori = model(points)
                #loss = criterion(output_pose,target_pose)
                ploss = criterion(pos,target_pose[:,:3])
                oloss = criterion(ori,target_pose[:,3:])
                valloss = ploss + 2*oloss
                #output_pose = torch.cat((pos,ori),dim=1)
                # error = torch.sum(abs(output_pose - target_pose),dim=0) + error
                # error = error/len(points)
                
                # print("Average absolute error for this batch: ",error)
                vrunning_loss += valloss.item() * points.shape[0]
                
                del points, target_pose, pos, ori
                torch.cuda.empty_cache()
                gc.collect()
            if vrunning_loss < min_loss:
                min_loss = vrunning_loss
                best_model = copy.deepcopy(model.state_dict())       
        print ('Epoch [{}/{}], Training Loss: {:.4f}'.format(epoch+1, num_epochs, trunning_loss))
        print ('Epoch [{}/{}], Validation Loss: {:.4f}'.format(epoch+1, num_epochs, vrunning_loss))
        y_loss['train'].append(trunning_loss / train_size)
        y_loss['val'].append(vrunning_loss / val_size)
        x_epoch.append(epoch+1)
        #epoch_acc = running_corrects / dataset_sizes[phase]
    model_file = MODELS_PATH + model_name
    torch.save(best_model,model_file)
    draw_curve(num_epochs)
    print(min_loss)

def load_model(model_path=None):
    model = GraspNetwork()
    if model_path:
        model.load_state_dict(torch.load(model_path,weights_only=True))
    model.to(DEVICE)
    return model

def main():
    model_file = MODELS_PATH + 'model_nn.pt'
    train_network(50,"dual_model_5kc_wl_dh_tansphere_vt_3.pt",samples=300)
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