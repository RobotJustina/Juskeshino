#!/usr/bin/env python3

import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
import torch.utils
import numpy as np
import open3d
import copy

import torch.utils.data
from grasp_network import GraspNetwork
import os
import numpy.lib.recfunctions as rf
import gc
import matplotlib.pyplot as plt
import math
from sklearn.model_selection import train_test_split
from torch.utils.data import Subset

#Gpu config
gpu_number = 1
gpus = 0
gpu_arr = '0'
BATCH_SIZE = 100
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
VAL_TO_TEST_RATIO = 0.1
FULL_DATASET = -1
y_loss = {}  # loss history
y_loss['train'] = []
y_loss['val'] = []
pos_err = {}
pos_err['train'] = []
pos_err['val'] = []
ori_err = {}
ori_err['train'] = []
ori_err['val'] = []
x_epoch = []

class GraspDataset(torch.utils.data.Dataset):
    def __init__(self, set_type, path = DATASET_PATH, samples = -1):
        self.base_path = path
        self.dataset_type = set_type
        p_path = os.listdir(self.base_path)
        p_path.sort()
        p_path = np.array(p_path)
        if samples > 0:
            index = np.random.choice(len(p_path), samples, replace=False)
            p_path = p_path[index]
        self.data_file_names = p_path
        if self.dataset_type == "test":
            print("test")
        if self.dataset_type == "validate":
            index = np.random.choice(len(p_path), int(len(p_path)*VAL_TO_TEST_RATIO), replace=False) 
            self.data_file_names = p_path[index]
            print("validate")
        #print(self.data_file_names)

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

def get_dataloaders(train_path=DATASET_PATH, val_path=DATASET_PATH, n_samples =-1):
    if train_path == val_path:
        dataset = GraspDataset(set_type="test",path=train_path,samples=n_samples)
        train_idx, val_idx = train_test_split(list(range(len(dataset))), test_size=VAL_TO_TEST_RATIO)
        valid_dataset = Subset(dataset,val_idx)
        train_dataset = Subset(dataset,train_idx)
        print(len(train_dataset),len(valid_dataset))
    else:
        train_dataset = GraspDataset(set_type="test",path=train_path,samples=n_samples)
        valid_dataset = GraspDataset(set_type="validate",path=val_path,samples=n_samples)
    train_loader = torch.utils.data.DataLoader(train_dataset, BATCH_SIZE, shuffle=True)
    valid_loader = torch.utils.data.DataLoader(valid_dataset, BATCH_SIZE, shuffle=True)

    return train_loader, valid_loader

def draw_curve(current_epoch):
    fig = plt.figure()
    ax0 = fig.add_subplot(131, title="Loss")
    ax1 = fig.add_subplot(132, title="Position error")
    ax2 = fig.add_subplot(133, title="Orientation error")
    ax0.plot(x_epoch, y_loss['train'], 'bo-', label='train')
    ax0.plot(x_epoch, y_loss['val'], 'ro-', label='val')
    ax0.grid()
    ax1.plot(x_epoch, pos_err['train'], 'bo-', label='train')
    ax1.plot(x_epoch, pos_err['val'], 'ro-', label='val')
    ax1.set_ylabel('Average error [m]')
    ax1.grid()
    ax2.plot(x_epoch, ori_err['train'], 'bo-', label='train')
    ax2.plot(x_epoch, ori_err['val'], 'ro-', label='val')
    ax2.set_ylabel('Average error [u]')
    ax2.grid()
    fig.supxlabel('Epochs')
    fig.tight_layout()
    fig.savefig(os.path.join(GRAPHS_PATH, 'train_dropout_hk_FD.eps'))
    plt.show()

def train_network(num_epochs,model_name, model_path=None,samples =-1):
    torch.cuda.empty_cache()
    gc.collect()
    train_loader, valid_loader = get_dataloaders(n_samples=samples)
    train_size = len(train_loader.dataset)
    val_size = len(valid_loader.dataset)
    model = load_model(model_path)
    best_model = copy.deepcopy(model.state_dict())
    criterion = nn.HuberLoss(delta=0.9)
    exp_error = nn.L1Loss(reduction='mean')
    min_loss = 15000
    #optimizer = optim.SGD(model.parameters(),lr=0.00008,momentum=0.8)
    optimizer = optim.Adam(model.parameters(),lr=0.00008)
    for epoch in range(num_epochs):
        trunning_loss = 0
        vrunning_loss = 0
        tpos_err = 0
        vpos_err = 0
        tori_err = 0
        vori_err = 0
        model.train()
        for batch, (points, target_pose) in enumerate(train_loader):
            points = points.to(DEVICE)
            target_pose = target_pose.to(DEVICE)

            #output_pose = model(points)
            pos, ori = model(points)
            #loss = criterion(output_pose,target_pose)
            ploss = criterion(pos,target_pose[:,:3])
            oloss = criterion(ori,target_pose[:,3:])
            tloss = ploss + oloss

            optimizer.zero_grad()
            tloss.backward()
            optimizer.step()
            
            trunning_loss += tloss.item() * points.shape[0]
            tpos_err += exp_error(pos,target_pose[:,:3]).item() * points.shape[0]
            tori_err += exp_error(ori,target_pose[:,3:]).item() * points.shape[0]

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
                valloss = ploss + oloss
                #output_pose = torch.cat((pos,ori),dim=1)
                # error = torch.sum(abs(output_pose - target_pose),dim=0) + error
                # error = error/len(points)
                
                # print("Average absolute error for this batch: ",error)
                vrunning_loss += valloss.item() * points.shape[0]
                vpos_err += exp_error(pos,target_pose[:,:3]).item() * points.shape[0]
                vori_err += exp_error(ori,target_pose[:,3:]).item() * points.shape[0]

                del points, target_pose, pos, ori
                torch.cuda.empty_cache()
                gc.collect()
            if (vrunning_loss / val_size) < min_loss:
                min_loss = vrunning_loss / val_size
                saved_epoch = epoch + 1
                best_model = copy.deepcopy(model.state_dict())       
        print ('Epoch [{}/{}], Training Loss: {:.4f}'.format(epoch+1, num_epochs, trunning_loss / train_size))
        print ('Epoch [{}/{}], Validation Loss: {:.4f}'.format(epoch+1, num_epochs, vrunning_loss / val_size))
        y_loss['train'].append(trunning_loss / train_size)
        y_loss['val'].append(vrunning_loss / val_size)
        pos_err['train'].append(tpos_err / train_size)
        pos_err['val'].append(vpos_err / val_size)
        ori_err['train'].append(tori_err / train_size)
        ori_err['val'].append(vori_err / val_size)
        x_epoch.append(epoch+1)
        #epoch_acc = running_corrects / dataset_sizes[phase]
    model_file = MODELS_PATH + model_name + "_ep" + str(saved_epoch) + ".pt"
    torch.save(best_model,model_file)
    model_file = MODELS_PATH + model_name + "_ep" + str(num_epochs) + ".pt" 
    torch.save(model.state_dict(),model_file)
    draw_curve(num_epochs)
    print(min_loss)
    print(saved_epoch)

def load_model(model_path=None):
    model = GraspNetwork()
    if model_path:
        model.load_state_dict(torch.load(model_path,weights_only=True))
    model.to(DEVICE)
    return model

def main():
    model_file = MODELS_PATH + 'model_nn.pt'
    train_network(50,"dual_model_46ks_nwl_dh_tansphere_nplot_1",samples=500)
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