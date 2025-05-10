#!/usr/bin/env python3

import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
import torch.utils.data
import numpy as np
import copy
import numpy.lib.recfunctions as rf
import os
import sqlite3
import gc
import matplotlib.pyplot as plt
from mixture_density_network import MixtureDensityNetwork
from cnn_autoencoder import Encoder
from sklearn.model_selection import train_test_split
from torch.utils.data import Subset
from io import BytesIO


##Environment Config and paths
DEVICE = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
torch.backends.cudnn.benchmark = True

MODEL_NAME = 'dummy_cae_10ks'
DATASET_PATH = "/home/robocup/Juskeshino/catkin_ws/src/graspnet/training_dataset/"
MODELS_PATH = "/home/robocup/Juskeshino/catkin_ws/src/graspnet/models/"
GRAPHS_PATH = "/home/robocup/Juskeshino/catkin_ws/src/graspnet/graphs/"
VAL_DATASET_PATH = "/home/robocup/Juskeshino/catkin_ws/src/graspnet/validate_dataset/"
DATABASE_PATH = "/home/robocup/Juskeshino/catkin_ws/src/graspnet/" + '/grasp_database_quaternion.db'
VAL_TO_TEST_RATIO = 0.1
FULL_DATASET = -1
BATCH_SIZE = 192

##
y_loss = {}
y_loss['train'] = []
y_loss['val'] = []
pos_err = {}
pos_err['train'] = []
pos_err['val'] = []
x_epoch = []

class Position_head(nn.Module):
    def __init__(self):
        super().__init__()
        self.pmdn = MixtureDensityNetwork(1024,3,32,256)

    def forward(self, x):
        pos = self.pmdn.sample(x)
        return pos
    
    def prob_loss(self, x, y):
        return self.pmdn.loss(x,y)

class Position_network(nn.Module):
    def __init__(self, enc_state_dict=None):
        super().__init__()
        self.enc = Encoder()
        self.pmdn = MixtureDensityNetwork(1024,3,32,256)
        if enc_state_dict:
            self.enc.load_state_dict(enc_state_dict)
        #print(self)

    def forward(self, x):
        x = self.enc(x)
        pos = self.pmdn.sample(x)
        return x, pos


class SQL_GraspDataset(torch.utils.data.Dataset):
    def __init__(self, set_type, path = DATABASE_PATH, samples = -1):
        self.conn = sqlite3.connect(path)
        self.cursor = self.conn.cursor()
        self.dataset_type = set_type
        self.bytes_io = BytesIO
        if samples > 0:
            self.indices = list(range(1,samples+1))
        else:
            self.indices = list(range(1,self.max_len()))
        #print(self.indices)
    def tuple_to_tensors(self, qry):
        points_t = []
        pose_t = []
        for row in qry:
            x,y,z,pcd_binary = row
            pcd = np.load(self.bytes_io(pcd_binary))
            pcd = rf.structured_to_unstructured(pcd)
            #pcd = pcd[:,:,:3]
            #points = np.transpose(pcd,(2,1,0))
            #pose = [x,y,z,i,j,k,w]
            points = torch.tensor(pcd[:,:,:3],dtype=torch.float32)
            points = torch.nan_to_num(points,nan=0.0)
            points = torch.permute(points,(2,1,0))
            pose = torch.tensor([x,y,z],dtype=torch.float32)
            points_t.append(points)
            pose_t.append(pose)
        #points_t = torch.tensor(np.array(points_t),dtype=torch.float32)
        #pose_t = torch.tensor(np.array(pose_t),dtype=torch.float32)
        #print(points_t.shape)
        points_t = torch.stack(points_t,dim=0)
        pose_t = torch.stack(pose_t,dim=0)
        return points_t, pose_t
        #return points_t.squeeze(0), pose_t.squeeze(0)
    
    def query_to_sample(self, qry):
        samples = []
        for row in qry:
            x,y,z,i,j,k,w,pcd_binary = row
            pcd = np.load(self.bytes_io(pcd_binary))
            pcd = rf.structured_to_unstructured(pcd)
            points = torch.tensor(pcd[:,:,:3],dtype=torch.float32)
            #points = torch.nan_to_num(points,nan=0.0)
            points = torch.permute(points,(2,1,0))
            pose = torch.tensor([x,y,z,i,j,k,w],dtype=torch.float32)
            samples.append(points,pose)
        return samples

    # def get_single(self, idx: int):
    #     self.cursor.execute("SELECT x,y,z (SELECT pcd_binary FROM point_clouds_table WHERE point_clouds_table.pcd_id = grasps_table.pcd_id) FROM grasps_table WHERE grasp_id = {}".format(idx))
    #     qry = self.cursor.fetchall()
    #     self.conn.commit()
    #     points, pose = self.tuple_to_tensors(qry)
    #     return points,pose
    #     #sample = self.query_to_sample(qry)
    #     #return sample
    
    def get_list(self, idx: list):
        self.cursor.execute("SELECT x,y,z, (SELECT pcd_binary FROM point_clouds_table WHERE point_clouds_table.pcd_id = grasps_table.pcd_id) FROM grasps_table WHERE grasp_id in (%s)" % ",".join([str(x) for x in idx]))
        qry = self.cursor.fetchall()
        self.conn.commit()
        points, pose = self.tuple_to_tensors(qry)
        return points,pose
        #sample = self.query_to_sample(qry)
        #return sample
    
    def get_slice(self, idx: slice):
        (start, stop, step) = (
            idx.start,
            idx.stop,
            1 if idx.step is None else idx.step,
        )
        assert not start is None and not stop is None
        return self.get_list(list(range(start, stop, step)))
    
    def get_single(self, idx):
        self.cursor.execute("SELECT x,y,z, (SELECT pcd_binary FROM point_clouds_table WHERE point_clouds_table.pcd_id = grasps_table.pcd_id) FROM grasps_table WHERE grasp_id = {}".format(self.indices[idx]))
        x,y,z,pcd_binary = self.cursor.fetchone()
        self.conn.commit()
        pcd = np.load(self.bytes_io(pcd_binary))
        pcd = rf.structured_to_unstructured(pcd)
        points = torch.tensor(pcd[:,:,:3],dtype=torch.float32)
        points = torch.nan_to_num(points,nan=0.0)
        points = torch.permute(points,(2,1,0))
        pose = torch.tensor([x,y,z],dtype=torch.float32)
        return points, pose
    
    def __getitem__(self,index):
        return self.indices[index]
    
    def collate(self, index):
        if isinstance(index, slice):
            #print(self.get_slice(index).shape)
            return self.get_slice(index)
        if isinstance(index, list):
            #print(self.get_list(index).shape)
            return self.get_list(index)
        if isinstance(index, int):
            #print(self.get_single(index).shape)
            return self.get_single(index)
        raise ValueError("Type of %s not supported by __getitem()__" % str(index))
    
    def max_len(self):
        self.cursor.execute('SELECT seq FROM sqlite_sequence WHERE name="grasps_table"')
        last_id = self.cursor.fetchone()[0]
        self.conn.commit()
        return last_id
    
    def __len__(self):
        return len(self.indices)
    
def get_SQL_dataloaders(train_path=DATABASE_PATH, val_path=DATABASE_PATH, n_samples =-1):
    if train_path == val_path:
        dataset = SQL_GraspDataset(set_type="test",path=train_path,samples=n_samples)
        train_idx, val_idx = train_test_split(list(range(len(dataset))), test_size=VAL_TO_TEST_RATIO)
        valid_dataset = Subset(dataset,val_idx)
        train_dataset = Subset(dataset,train_idx)
        print(len(train_dataset),len(valid_dataset))
    else:
        train_dataset = SQL_GraspDataset(set_type="test",path=train_path,samples=n_samples)
        valid_dataset = SQL_GraspDataset(set_type="validate",path=val_path,samples=n_samples)
    train_loader = torch.utils.data.DataLoader(train_dataset, BATCH_SIZE, shuffle=True, num_workers=10, prefetch_factor=5, persistent_workers=True, pin_memory=True, generator=torch.Generator('cpu'), collate_fn=train_dataset.dataset.collate)
    print(len(train_loader))
    valid_loader = torch.utils.data.DataLoader(valid_dataset, BATCH_SIZE, shuffle=True, num_workers=3, prefetch_factor=3, persistent_workers=True, pin_memory=True, generator=torch.Generator('cpu'), collate_fn=valid_dataset.dataset.collate)

    return train_loader, valid_loader

def draw_curve():
    fig = plt.figure()
    ax0 = fig.add_subplot(121, title="Loss")
    ax1 = fig.add_subplot(122, title="Position error")
    ax0.plot(x_epoch, y_loss['train'], 'bo-', label='train')
    ax0.plot(x_epoch, y_loss['val'], 'ro-', label='val')
    ax0.grid()
    ax1.plot(x_epoch, pos_err['train'], 'bo-', label='train')
    ax1.plot(x_epoch, pos_err['val'], 'ro-', label='val')
    ax1.set_ylabel('Average error [m]')
    ax1.grid()
    fig.supxlabel('Epochs')
    fig.tight_layout()
    fig.savefig(os.path.join(GRAPHS_PATH, 'new_joined_head.eps'))
    plt.show()


def train_network(num_epochs,model_name,cae_file, model_path=None,samples =-1):
    torch.cuda.empty_cache()
    gc.collect()
    train_loader, valid_loader = get_SQL_dataloaders(n_samples=samples)
    train_size = len(train_loader.dataset)
    val_size = len(valid_loader.dataset)
    model = load_model(cae_path=cae_file)
    #best_model = copy.deepcopy(model.state_dict())
    exp_error = nn.L1Loss(reduction='mean')
    min_loss = 150000

    #Freeze gradient for encoder module parameters
    for param in model.enc.parameters():
        param.requires_grad = False

    optimizer = optim.AdamW([
                #{'params': model.enc.parameters()},
                {'params': model.pmdn.parameters(), 'lr': 0.0001}
            ],lr=0.0001,weight_decay=0.002)
    lrscheduler = torch.optim.lr_scheduler.OneCycleLR(optimizer=optimizer,max_lr=0.001,steps_per_epoch=len(train_loader),epochs=num_epochs)


    for epoch in range(num_epochs):
        trunning_loss = 0
        vrunning_loss = 0
        tpos_err = 0
        vpos_err = 0
        model.train()

        for batch, (pcd, target_pos) in enumerate(train_loader,0):
            pcd, target_pos = pcd.cuda(non_blocking=True), target_pos.cuda(non_blocking=True)
            x, pos = model(pcd)
            tloss = model.pmdn.loss(x,target_pos).mean()

            optimizer.zero_grad()
            tloss.backward()
            optimizer.step()
            lrscheduler.step()
            
            trunning_loss += tloss.item() * pcd.shape[0]
            tpos_err += exp_error(pos,target_pos).item() * pcd.shape[0]

            del pcd, pos, target_pos
            torch.cuda.empty_cache()
            gc.collect()
        
        model.eval()    
        with torch.no_grad():
            for pcd, target_pos in valid_loader:
                pcd = pcd.cuda(non_blocking=True)
                target_pos = target_pos.cuda(non_blocking=True)
                x, pos = model(pcd)

                valloss = model.pmdn.loss(x,target_pos).mean()

                vrunning_loss += valloss.item() * pcd.shape[0]
                vpos_err += exp_error(pos,target_pos).item() * pcd.shape[0]

                del pcd, pos, target_pos
                torch.cuda.empty_cache()
                gc.collect()
            if (vrunning_loss / val_size) < min_loss:
                min_loss = vrunning_loss / val_size
                saved_epoch = epoch + 1
                best_enc_model = copy.deepcopy(model.enc.state_dict())       
                best_pos_model = copy.deepcopy(model.pmdn.state_dict())       
        print ('Epoch [{}/{}], Training Loss: {:.4f}'.format(epoch+1, num_epochs, trunning_loss / train_size))
        print ('Epoch [{}/{}], Validation Loss: {:.4f}'.format(epoch+1, num_epochs, vrunning_loss / val_size))
        y_loss['train'].append(trunning_loss / train_size)
        y_loss['val'].append(vrunning_loss / val_size)
        pos_err['train'].append(tpos_err / train_size)
        pos_err['val'].append(vpos_err / val_size)
        x_epoch.append(epoch+1)
    model_file = MODELS_PATH + model_name + "_ep" + str(saved_epoch) + ".pt"
    torch.save({
        'encoder_state_dict': best_enc_model,
        'pos_state_dict': best_pos_model
    },model_file)
    model_file = MODELS_PATH + model_name + "_ep" + str(num_epochs) + ".pt" 
    torch.save({
        'encoder_state_dict': model.enc.state_dict(),
        'pos_state_dict': model.pmdn.state_dict()
    },model_file)
    draw_curve()
    print(min_loss)
    print(saved_epoch)

def load_model(model_path=None, cae_path=MODELS_PATH + 'dummy_cae_10ks_ep97_split_dict.pt'):
    enc_state_dict = torch.load(cae_path,weights_only=True)['encoder_state_dict']
    model = Position_network(enc_state_dict)
    if model_path:
        model.load_state_dict(torch.load(model_path,weights_only=True))
    model.to(DEVICE)
    return model

def load_and_split_model(model_path,save_path):
    model = load_model(model_path)
    torch.save({
        'encoder_state_dict': model.enc.state_dict(),
        'pos_state_dict': model.pmdn.state_dict()
    },save_path)

def quick_create_test():
    cae_file = MODELS_PATH + 'dummy_cae_10ks_ep97_split_dict.pt'
    enc_state_dict = torch.load(cae_file,weights_only=True)['encoder_state_dict']
    for param_tensor in enc_state_dict:
        print(param_tensor,'\t', enc_state_dict[param_tensor].size())
    pos_network = Position_network(enc_state_dict)

def main():
    #save_pos_file = MODELS_PATH + 'pos_network_orienc_5k_lr0008_ep25_split.pt'
    #pos_file = MODELS_PATH + 'pos_network_orienc_5k_lr0008_ep25.pt'
    enc_file = MODELS_PATH + 'orient_net_vmf_encgrad_kc_50k_onecycle_0008_kmeans2_ep20.pt'
    train_network(20,'pos_network_orienc_50k_lr001_mish_nograd',cae_file=enc_file,samples=50000)
    #load_and_split_model(pos_file,save_pos_file)

if __name__ == '__main__':
    main()