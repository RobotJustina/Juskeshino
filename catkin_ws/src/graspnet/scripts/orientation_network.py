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
os.environ['NUMEXPR_MAX_THREADS'] = '20'
os.environ['NUMEXPR_NUM_THREADS'] = '20'
import sqlite3
import gc
import matplotlib.pyplot as plt
import open3d
import geomstats.backend as gs
from geomstats.geometry.hypersphere import Hypersphere, HypersphereMetric
from mixture_density_network import MixtureDensityNetwork
from cnn_autoencoder import Encoder
from sklearn.model_selection import train_test_split
from torch.utils.data import Subset
from io import BytesIO
from torch.profiler import profile, record_function, ProfilerActivity


##Environment Config and paths
DEVICE = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
torch.backends.cudnn.benchmark = True
torch.backends.cuda.matmul.allow_tf32 = True
#torch.cuda.set_device('cuda:0')
#torch.set_default_tensor_type('torch.cuda.FloatTensor')
torch.set_default_dtype(torch.float32)
torch.set_flush_denormal(True)
#torch.set_default_device()
#torch.multiprocessing.set_start_method('spawn')

MODEL_NAME = 'dummy_cae_10ks'
DATASET_PATH = "/home/robocup/Juskeshino/catkin_ws/src/graspnet/training_dataset/"
MODELS_PATH = "/home/robocup/Juskeshino/catkin_ws/src/graspnet/models/"
GRAPHS_PATH = "/home/robocup/Juskeshino/catkin_ws/src/graspnet/graphs/"
VAL_DATASET_PATH = "/home/robocup/Juskeshino/catkin_ws/src/graspnet/validate_dataset/"
DATABASE_PATH = "/home/robocup/Juskeshino/catkin_ws/src/graspnet/" + '/grasp_database_quaternion.db'
VAL_TO_TEST_RATIO = 0.1
FULL_DATASET = -1
BATCH_SIZE = 128

##
y_loss = {}
y_loss['train'] = []
y_loss['val'] = []
ori_err = {}
ori_err['train'] = []
ori_err['val'] = []
x_epoch = []

class Orientation_head(nn.Module):
    def __init__(self):
        super().__init__()
        #self.omdn = MixtureDensityNetwork(1027,4,32,256).double()
        self.omdn = nn.Sequential(
            nn.Linear(1027,512),
            nn.Linear(512,512),
            nn.ReLU(),
            nn.Dropout(0.5),
            nn.Linear(512,256),
            nn.ReLU(),
            nn.Dropout(0.5),
            nn.Linear(256,256),
            nn.ReLU(),
            nn.Dropout(0.5),
            nn.Linear(256,256),
            nn.ReLU(),
            nn.Dropout(0.5),
            nn.Linear(256,128),
            nn.ReLU(),
            nn.Dropout(0.5),
            nn.Linear(128,4).double()
        )
        self.space = Hypersphere(3)
        self.BASE_POINT = torch.tensor([0.0,0.0,0.0,1.0],dtype=torch.float64).to(DEVICE)

    def forward(self, x, pos):
        x = torch.cat((x,pos),dim=1)
        ori = self.omdn(x)
        ori = self.space.metric.exp(ori,self.BASE_POINT)
        return ori
    
    def L1_loss(self, q1, q2):
        d = torch.abs(self.space.metric.dist(q1,q2)).mean()
        return d
    
    def L2_loss(self, q1,q2):
        d = torch.square(self.space.metric.dist(q1,q2)).mean()
        return d
    
    def prob_loss(self, x, y):
        return self.omdn.loss(x,y)
    

class Orientation_network(nn.Module):
    def __init__(self, enc_state_dict):
        super().__init__()
        self.enc = Encoder()
        self.enc.load_state_dict(enc_state_dict)
        #self.omdn = MixtureDensityNetwork(1027,4,32,256)
        self.omdn = nn.Sequential(
            nn.Linear(1027,512),
            nn.Linear(512,512),
            nn.ReLU(),
            nn.Dropout(0.5),
            nn.Linear(512,256),
            nn.ReLU(),
            nn.Dropout(0.5),
            nn.Linear(256,256),
            nn.ReLU(),
            nn.Dropout(0.5),
            nn.Linear(256,256),
            nn.ReLU(),
            nn.Dropout(0.5),
            nn.Linear(256,128),
            nn.ReLU(),
            nn.Dropout(0.5),
            nn.Linear(128,4).double()
        )
        self.space = Hypersphere(3)
        self.BASE_POINT = torch.tensor([0.0,0.0,0.0,1.0],dtype=torch.float64).to(DEVICE)
        #print(self)

    def forward(self, x, pos):
        x = self.enc(x)
        x = torch.cat((x,pos),dim=1)
        ori = self.omdn.sample(x)
        
        #x = self.omdn(x)
        #x = torch.cat((x,pos),dim=1)
        #ori = self.out(x)

        #ori = self.omdn(x)
        ori = self.space.metric.exp(ori,self.BASE_POINT)
        return x, ori
    
    def L1_loss(self, q1, q2):
        d = torch.abs(self.space.metric.dist(q1,q2)).mean()
        return d
    
    def L2_loss(self, q1,q2):
        d = torch.square(self.space.metric.dist(q1,q2)).mean()
        return d


class SQL_GraspDataset(torch.utils.data.Dataset):
    def __init__(self, set_type, path = DATABASE_PATH, samples = -1):
        self.conn = sqlite3.connect(path)
        self.cursor = self.conn.cursor()
        self.dataset_type = set_type
        self.bytes_io = BytesIO
        self.BASE_POINT = torch.tensor([0.0,0.0,0.0,1.0],dtype=torch.float64)
        self.space = Hypersphere(3)
        if samples > 0:
            self.indices = list(range(1,samples+1))
        else:
            self.indices = list(range(1,self.max_len()))
        #print(self.indices)
    def tuple_to_tensors(self, qry):
        points_t = []
        pos_t = []
        ori_t = []
        for row in qry:
            x,y,z,i,j,k,w,pcd_binary = row
            pcd = np.load(self.bytes_io(pcd_binary))
            pcd = rf.structured_to_unstructured(pcd)
            points = torch.tensor(pcd[:,:,:3],dtype=torch.float32)
            points = torch.nan_to_num(points,nan=0.0)
            points = torch.permute(points,(2,1,0))
            pos = torch.tensor([x,y,z],dtype=torch.float32)
            ori = torch.tensor([i,j,k,w],dtype=torch.float64)
            ori = self.space.metric.log(ori,self.BASE_POINT)
            #print(ori)
            points_t.append(points)
            pos_t.append(pos)
            ori_t.append(ori)
        return points_t, pos_t, ori_t
    
    def query_to_sample(self, qry):
        samples = []
        for row in qry:
            x,y,z,i,j,k,w,pcd_binary = row
            pcd = np.load(self.bytes_io(pcd_binary))
            pcd = rf.structured_to_unstructured(pcd)
            points = torch.tensor(pcd[:,:,:3],dtype=torch.float32)
            points = torch.nan_to_num(points,nan=0.0)
            points = torch.permute(points,(2,1,0))
            pose = torch.tensor([x,y,z,i,j,k,w],dtype=torch.float32)
            samples.append(points,pose)
        return samples
    
    def get_list(self, idx: list):
        self.cursor.execute("SELECT x,y,z,i,j,k,w, (SELECT pcd_binary FROM point_clouds_table WHERE point_clouds_table.pcd_id = grasps_table.pcd_id) FROM grasps_table WHERE grasp_id in (%s)" % ",".join([str(x) for x in idx]))
        qry = self.cursor.fetchall()
        self.conn.commit()
        points, pos, ori = self.tuple_to_tensors(qry)
        return points, pos, ori
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
        self.cursor.execute("SELECT x,y,z,i,j,k,w, (SELECT pcd_binary FROM point_clouds_table WHERE point_clouds_table.pcd_id = grasps_table.pcd_id) FROM grasps_table WHERE grasp_id = {}".format(self.indices[idx]))
        x,y,z,i,j,k,w,pcd_binary = self.cursor.fetchone()
        self.conn.commit()
        pcd = np.load(self.bytes_io(pcd_binary))
        pcd = rf.structured_to_unstructured(pcd)
        points = torch.tensor(pcd[:,:,:3],dtype=torch.float32)
        points = torch.nan_to_num(points,nan=0.0)
        points = torch.permute(points,(2,1,0))
        pos = torch.tensor([x,y,z],dtype=torch.float32)
        ori = torch.tensor([i,j,k,w],dtype=torch.float64)
        ori = self.space.metric.log(ori,self.BASE_POINT)
        #print(ori)
        return points, pos, ori
    
    def __getitem__(self, index):
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
    train_loader = torch.utils.data.DataLoader(train_dataset, BATCH_SIZE, shuffle=True, num_workers=14, prefetch_factor=6, persistent_workers=True, pin_memory=True, generator=torch.Generator('cpu'))
    print(len(train_loader))
    valid_loader = torch.utils.data.DataLoader(valid_dataset, BATCH_SIZE, shuffle=True, num_workers=6, prefetch_factor=5, persistent_workers=True, pin_memory=True, generator=torch.Generator('cpu'))

    return train_loader, valid_loader

def draw_curve():
    fig = plt.figure()
    ax0 = fig.add_subplot(121, title="Loss")
    ax1 = fig.add_subplot(122, title="Orientation geodesic error")
    ax0.plot(x_epoch, y_loss['train'], 'bo-', label='train')
    ax0.plot(x_epoch, y_loss['val'], 'ro-', label='val')
    ax0.grid()
    ax1.plot(x_epoch, ori_err['train'], 'bo-', label='train')
    ax1.plot(x_epoch, ori_err['val'], 'ro-', label='val')
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
    model = model.to(DEVICE)
    best_model = copy.deepcopy(model.state_dict())
    #exp_error = nn.L1Loss(reduction='mean')
    exp_error = model.L1_loss
    min_loss = 150000
    space = Hypersphere(3)
    BASE_POINT = torch.tensor([0.0,0.0,0.0,1.0],dtype=torch.float64).to(DEVICE)

    #Freeze gradient for encoder module parameters
    model.BASE_POINT.requires_grad = False
    for param in model.enc.parameters():
        param.requires_grad = False

    optimizer = optim.AdamW(model.parameters(),lr=0.0001,weight_decay=0.002)
    #lrscheduler = torch.optim.lr_scheduler.CosineAnnealingWarmRestarts(optimizer,5,2,0.000001)

    for epoch in range(num_epochs):
        trunning_loss = 0
        vrunning_loss = 0
        tori_err = 0
        vori_err = 0
        model.train()

        for batch, (pcd, pos, target_ori) in enumerate(train_loader,0):
            pcd, pos, target_ori = pcd.to(DEVICE, non_blocking = True), pos.to(DEVICE, non_blocking = True), target_ori.to(DEVICE, non_blocking = True)
            #print(pos)
            with torch.device(DEVICE):
                #print(target_ori)
                #print(model.space.belongs(target_ori))
                
                x, ori = model(pcd, pos)
                #tloss = model.omdn.loss(x,target_ori).mean()
                tloss = model.L2_loss(ori,target_ori)

                optimizer.zero_grad()
                tloss.backward()
                optimizer.step()
            
                trunning_loss += tloss.item() * pcd.shape[0]
                #tori_err += exp_error(ori,space.metric.exp(target_ori,BASE_POINT)).item() * pcd.shape[0]
                tori_err += exp_error(ori,target_ori).item() * pcd.shape[0]

                del pcd, ori, target_ori, pos
                torch.cuda.empty_cache()
                gc.collect()

        #lrscheduler.step()

        model.eval()    
        with torch.no_grad():
            for pcd, pos, target_ori in valid_loader:
                pcd = pcd.to(DEVICE, non_blocking = True)
                pos = pos.to(DEVICE, non_blocking = True)
                target_ori = target_ori.to(DEVICE, non_blocking = True)
                
                with torch.device(DEVICE):
                    x, ori = model(pcd, pos)
                    #valloss = model.omdn.loss(x,target_ori).mean()
                    valloss = model.L2_loss(ori, target_ori)

                    vrunning_loss += valloss.item() * pcd.shape[0]
                    #vori_err += exp_error(ori,space.metric.exp(target_ori,BASE_POINT)).item() * pcd.shape[0]
                    vori_err += exp_error(ori,target_ori).item() * pcd.shape[0]

                    del pcd, ori, target_ori, pos
                    torch.cuda.empty_cache()
                    gc.collect()
            if (vrunning_loss / val_size) < min_loss:
                min_loss = vrunning_loss / val_size
                saved_epoch = epoch + 1
                best_model = copy.deepcopy(model.state_dict())       
        print ('Epoch [{}/{}], Training Loss: {:.4f}'.format(epoch+1, num_epochs, trunning_loss / train_size))
        print ('Epoch [{}/{}], Validation Loss: {:.4f}'.format(epoch+1, num_epochs, vrunning_loss / val_size))
        print ('Epoch [{}/{}], Average error: {:.4f}'.format(epoch+1, num_epochs, vori_err / val_size))
        y_loss['train'].append(trunning_loss / train_size)
        y_loss['val'].append(vrunning_loss / val_size)
        ori_err['train'].append(tori_err / train_size)
        ori_err['val'].append(vori_err / val_size)
        x_epoch.append(epoch+1)
        
    model_file = MODELS_PATH + model_name + "_ep" + str(saved_epoch) + ".pt"
    torch.save({
        'encoder_state_dict': model.enc.state_dict(),
        'ori_state_dict': model.omdn.state_dict()
    },model_file)
    model_file = MODELS_PATH + model_name + "_ep" + str(num_epochs) + ".pt" 
    torch.save({
        'encoder_state_dict': model.enc.state_dict(),
        'ori_state_dict': model.omdn.state_dict()
    },model_file)
    draw_curve()
    print(min_loss)
    print(saved_epoch)

def load_model(model_path=None, cae_path=MODELS_PATH + 'dummy_cae_10ks_ep97_split_dict.pt'):
    enc_state_dict = torch.load(cae_path,weights_only=True)['encoder_state_dict']
    model = Orientation_network(enc_state_dict)
    if model_path:
        model.load_state_dict(torch.load(model_path,weights_only=True))
    model = model.float().cuda()
    return model

def load_and_split_model(model_path,save_path):
    model = load_model(model_path)
    torch.save({
        'encoder_state_dict': model.enc.state_dict(),
        'ori_state_dict': model.omdn.state_dict()
    },save_path)

def quick_create_test():
    cae_file = MODELS_PATH + 'dummy_cae_10ks_ep97_split_dict.pt'
    enc_state_dict = torch.load(cae_file,weights_only=True)['encoder_state_dict']
    for param_tensor in enc_state_dict:
        print(param_tensor,'\t', enc_state_dict[param_tensor].size())
    ori_network = Orientation_network(enc_state_dict)
    print(ori_network)

def main():
    cae_file = MODELS_PATH + 'dummy_cae_10ks_ep97_split_dict.pt'
    train_network(60,'orient_net_more_rest_tradfc_flush_50k_lrs',cae_file,samples=50000)
    #print(prof.key_averages().table(sort_by="cpu_time_total", row_limit=30))
            
    #quick_create_test()
    #ori_file = MODELS_PATH + 'dummy_ori_net_1ks_ep100.pt'
    #save_ori_file = MODELS_PATH +'dummy_ori_net_1ks_ep100_split.pt'
    #load_and_split_model(ori_file,save_ori_file)

if __name__ == '__main__':
    main()