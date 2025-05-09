#!/home/robocup/venvs/python3_11/bin/python

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
#import open3d
import geomstats.backend as gs
from geomstats.geometry.hypersphere import Hypersphere, HypersphereMetric
from mixture_density_network import MixtureDensityNetwork
from kernel_mixture_network import Kernel_Mixture_Network
from position_network import Position_head
from ori_network3 import Orientation_head
from cnn_autoencoder import Encoder
from sklearn.model_selection import train_test_split
from torch.utils.data import Subset
from io import BytesIO


##Environment Config and paths
DEVICE = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
torch.set_default_dtype(torch.float32)

MODEL_NAME = 'dummy_cae_10ks'
DATASET_PATH = "/home/robocup/Juskeshino/catkin_ws/src/graspnet/training_dataset/"
MODELS_PATH = "/home/robocup/Juskeshino/catkin_ws/src/graspnet/models/"
GRAPHS_PATH = "/home/robocup/Juskeshino/catkin_ws/src/graspnet/graphs/"
VAL_DATASET_PATH = "/home/robocup/Juskeshino/catkin_ws/src/graspnet/validate_dataset/"
DATABASE_PATH = "/home/robocup/Juskeshino/catkin_ws/src/graspnet/" + '/grasp_database_nm_test2.db'
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
ori_err = {}
ori_err['train'] = []
ori_err['val'] = []
x_epoch = []

class Joint_Mixture_Density_Grasp_Network(nn.Module):
    def __init__(self, enc_state_dict=None, pos_state_dict=None, ori_state_dict=None):
        super().__init__()
        self.enc = Encoder()
        self.pos_head = Position_head()
        self.ori_head = Orientation_head()
        self.space = Hypersphere(3)
        self.BASE_POINT = torch.tensor([0.0,0.0,0.0,1.0],dtype=torch.float32).to(DEVICE)
        if enc_state_dict:
            self.enc.load_state_dict(enc_state_dict)
        if pos_state_dict:
            self.pos_head.pmdn.load_state_dict(pos_state_dict)
        if ori_state_dict:
            self.ori_head.kmm.load_state_dict(ori_state_dict)

    def forward(self, x):
        x = self.enc(x)
        pos = self.pos_head(x)
        #print(type(x),type(pos), pos)
        #x = torch.cat((x,pos),dim=1)
        ori = self.ori_head(x,pos)
        #ori = self.space.metric.exp(ori,self.BASE_POINT)
        return pos, ori, x


class SQL_GraspDataset(torch.utils.data.Dataset):
    def __init__(self, set_type, path = DATABASE_PATH, samples = -1):
        self.conn = sqlite3.connect(path)
        self.cursor = self.conn.cursor()
        self.dataset_type = set_type
        self.bytes_io = BytesIO
        self.BASE_POINT = torch.tensor([0.0,0.0,0.0,1.0],dtype=torch.float32)
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
            ori = torch.tensor([i,j,k,w],dtype=torch.float32)
            ori = self.space.metric.log(ori,self.BASE_POINT).float()
            print(ori)
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
        ori = torch.tensor([i,j,k,w],dtype=torch.float32)
        ori = self.space.metric.log(ori,self.BASE_POINT).float()
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
    train_loader = torch.utils.data.DataLoader(train_dataset, BATCH_SIZE, shuffle=True, num_workers=10, pin_memory=True, generator=torch.Generator('cpu'))
    print(len(train_loader))
    valid_loader = torch.utils.data.DataLoader(valid_dataset, BATCH_SIZE, shuffle=True, num_workers=6, pin_memory=True, generator=torch.Generator('cpu'))

    return train_loader, valid_loader

def draw_curve():
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
    fig.savefig(os.path.join(GRAPHS_PATH, 'new_joined_head.eps'))
    plt.show()


def train_network(num_epochs,model_name,cae_file, model_path=None,samples =-1):
    torch.cuda.empty_cache()
    gc.collect()
    train_loader, valid_loader = get_SQL_dataloaders(n_samples=samples)
    train_size = len(train_loader.dataset)
    val_size = len(valid_loader.dataset)
    model = load_model(cae_file=cae_file)
    model = model.to(DEVICE)
    best_model = copy.deepcopy(model.state_dict())
    #exp_error = nn.L1Loss(reduction='mean')
    #exp_error = model.L1_loss
    min_loss = 150000

    #Freeze gradient for encoder module parameters
    for param in model.enc.parameters():
        param.requires_grad = False

    optimizer = optim.AdamW((model.pos_head.parameters(),model.ori_head.parameters()),lr=0.0001,weight_decay=0.0002)

    for epoch in range(num_epochs):
        trunning_loss = 0
        vrunning_loss = 0
        tpos_err = 0
        vpos_err = 0
        tori_err = 0
        vori_err = 0
        model.train()

        for batch, (pcd, target_pos, target_ori) in enumerate(train_loader,0):
            pcd, target_pos, target_ori = pcd.cuda(non_blocking=True), pos.cuda(non_blocking=True), target_ori.cuda(non_blocking=True)
            #print(pos)
            with torch.device(DEVICE):
                pos, ori, x = model(pcd, target_pos)
                tloss = model.pos_head.prob_loss(x,target_pos).mean()

            optimizer.zero_grad()
            tloss.backward()
            optimizer.step()
            
            trunning_loss += tloss.item() * pcd.shape[0]
            tori_err += exp_error(ori,target_ori).item() * pcd.shape[0]

            del pcd, ori, target_ori, pos
            torch.cuda.empty_cache()
            gc.collect()
        
        model.eval()    
        with torch.no_grad():
            for pcd, pos, target_ori in valid_loader:
                pcd = pcd.cuda(non_blocking=True)
                pos = pos.cuda(non_blocking=True)
                target_ori = target_ori.cuda(non_blocking=True)
                
                with torch.device(DEVICE):
                    x, ori = model(pcd, pos)
                    valloss = model.omdn.loss(x,target_ori).mean()

                vrunning_loss += valloss.item() * pcd.shape[0]
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
        y_loss['train'].append(trunning_loss / train_size)
        y_loss['val'].append(vrunning_loss / val_size)
        pos_err['train'].append(tpos_err / train_size)
        pos_err['val'].append(vpos_err / val_size)
        ori_err['train'].append(tori_err / train_size)
        ori_err['val'].append(vori_err / val_size)
        x_epoch.append(epoch+1)
    model_file = MODELS_PATH + model_name + "_ep" + str(saved_epoch) + ".pt"
    torch.save(best_model,model_file)
    model_file = MODELS_PATH + model_name + "_ep" + str(num_epochs) + ".pt" 
    torch.save(model.state_dict(),model_file)
    draw_curve()
    print(min_loss)
    print(saved_epoch)

def load_model(cae_file=None, pos_file=None, ori_file=None):
    enc_state_dict = None
    pos_state_dict = None
    ori_state_dict = None
    if cae_file: enc_state_dict = torch.load(cae_file,weights_only=True)['encoder_state_dict']
    if pos_file: pos_state_dict = torch.load(pos_file,weights_only=True)['pos_state_dict']
    if ori_file: ori_state_dict = torch.load(ori_file,weights_only=True)['ori_state_dict']
    model = Joint_Mixture_Density_Grasp_Network(enc_state_dict,pos_state_dict,ori_state_dict)
    model.float().to(DEVICE)
    return model

def quick_create_test():
    cae_file = MODELS_PATH + 'orient_net_vmf_encgrad_kc_50k_onecycle_0008_ep20.pt'
    save_ori_file = MODELS_PATH +'orient_net_vmf_encgrad_kc_50k_onecycle_0008_ep20.pt'
    save_pos_file = MODELS_PATH + 'pos_network_orienc_5k_lr0008_ep25_split.pt'
    # enc_state_dict = torch.load(cae_file,weights_only=True)['encoder_state_dict']
    # for param_tensor in enc_state_dict:
    #     print(param_tensor,'\t', enc_state_dict[param_tensor].size())
    jmdgn = load_model(cae_file,save_pos_file,save_ori_file)
    #print(jmdgn)
    return jmdgn

def main():
    #cae_file = MODELS_PATH + 'dummy_cae_10ks_ep97_split_dict.pt'
    #train_network(100,'dummy_joint_net_1ks',cae_file,samples=1000)
    quick_create_test()

if __name__ == '__main__':
    main()