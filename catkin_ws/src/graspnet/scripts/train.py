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
import sqlite3
from sklearn.model_selection import train_test_split
from torch.utils.data import Subset
from io import BytesIO
import geomstats.backend as gs
from geomstats.geometry.hypersphere import Hypersphere, HypersphereMetric

#Gpu config
gpu_number = 1
gpus = 0
gpu_arr = '0'
BATCH_SIZE = 300
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
DATABASE_PATH = "/home/robocup/Juskeshino/catkin_ws/src/graspnet/" + '/grasp_database_single_test.db'
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

class SQL_GraspDataset(torch.utils.data.Dataset):
    def __init__(self, set_type, path = DATABASE_PATH, samples = -1):
        self.conn = sqlite3.connect(path)
        self.cursor = self.conn.cursor()
        self.dataset_type = set_type
        self.bytes_io = BytesIO

    def tuple_to_tensors(self, qry):
        points_t = []
        pose_t = []
        for row in qry:
            x,y,z,i,j,k,w,pcd_binary = row
            pcd = np.load(self.bytes_io(pcd_binary))
            pcd = rf.structured_to_unstructured(pcd)
            #pcd = pcd[:,:,:3]
            #points = np.transpose(pcd,(2,1,0))
            #pose = [x,y,z,i,j,k,w]
            points = torch.tensor(pcd[:,:,:3],dtype=torch.float32)
            points = torch.permute(points,(2,1,0))
            pose = torch.tensor([x,y,z,i,j,k,w],dtype=torch.float32)
            points_t.append(points)
            pose_t.append(pose)
        #points_t = torch.tensor(np.array(points_t),dtype=torch.float32)
        #pose_t = torch.tensor(np.array(pose_t),dtype=torch.float32)
        #print(points_t.shape)
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

    def get_single(self, idx: int):
        self.cursor.execute("SELECT x,y,z,i,j,k,w, (SELECT pcd_binary FROM point_clouds_table WHERE point_clouds_table.pcd_id = grasps_table.pcd_id) FROM grasps_table WHERE grasp_id = {}".format(idx))
        qry = self.cursor.fetchall()
        self.conn.commit()
        points, pose = self.tuple_to_tensors(qry)
        return points,pose
        #sample = self.query_to_sample(qry)
        #return sample
    
    def get_list(self, idx: list):
        self.cursor.execute("SELECT x,y,z,i,j,k,w, (SELECT pcd_binary FROM point_clouds_table WHERE point_clouds_table.pcd_id = grasps_table.pcd_id) FROM grasps_table WHERE grasp_id in (%s)" % ",".join([str(x) for x in idx]))
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
    
    # def __getitem__(self, index):
    #     if isinstance(index, slice):
    #         #print(self.get_slice(index).shape)
    #         return self.get_slice(index)
    #     if isinstance(index, list):
    #         #print(self.get_list(index).shape)
    #         return self.get_list(index)
    #     if isinstance(index, int):
    #         #print(self.get_single(index).shape)
    #         return self.get_single(index)
    #     raise ValueError("Type of %s not supported by __getitem()__" % str(index))
    
    def __getitem__(self, idx):
        self.cursor.execute("SELECT x,y,z,i,j,k,w, (SELECT pcd_binary FROM point_clouds_table WHERE point_clouds_table.pcd_id = grasps_table.pcd_id) FROM grasps_table WHERE grasp_id = {}".format(idx +1))
        x,y,z,i,j,k,w,pcd_binary = self.cursor.fetchone()
        self.conn.commit()
        pcd = np.load(self.bytes_io(pcd_binary))
        pcd = rf.structured_to_unstructured(pcd)
        points = torch.tensor(pcd[:,:,:3],dtype=torch.float32)
        #points = torch.nan_to_num(points,nan=0.0)
        points = torch.permute(points,(2,1,0))
        pose = torch.tensor([x,y,z,i,j,k,w],dtype=torch.float32)
        return points, pose
    
    def __len__(self):
        self.cursor.execute('SELECT seq FROM sqlite_sequence WHERE name="grasps_table"')
        last_id = self.cursor.fetchone()[0]
        self.conn.commit()
        return last_id


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
    train_loader = torch.utils.data.DataLoader(train_dataset, BATCH_SIZE, shuffle=True, num_workers=0,generator=torch.Generator(DEVICE))
    print(len(train_loader))
    valid_loader = torch.utils.data.DataLoader(valid_dataset, BATCH_SIZE, shuffle=True, num_workers=0,generator=torch.Generator(DEVICE))

    return train_loader, valid_loader

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
    fig.savefig(os.path.join(GRAPHS_PATH, 'new_joined_head.eps'))
    plt.show()

def train_network(num_epochs,model_name, model_path=None,samples =-1):
    torch.cuda.empty_cache()
    gc.collect()
    train_loader, valid_loader = get_SQL_dataloaders(n_samples=samples)
    train_size = len(train_loader.dataset)
    val_size = len(valid_loader.dataset)
    model = load_model(model_path)
    best_model = copy.deepcopy(model.state_dict())
    criterion = nn.HuberLoss(delta=0.9)
    exp_error = nn.L1Loss(reduction='mean')
    min_loss = 15000
    #optimizer = optim.SGD(model.parameters(),lr=0.00008,momentum=0.8)
    optimizer = optim.AdamW(model.parameters(),lr=0.00007,weight_decay=0.01)
    space = Hypersphere(3)
    base_point = torch.tensor([0.0,0.0,0.0,1.0],dtype=torch.float32)
    for epoch in range(num_epochs):
        trunning_loss = 0
        vrunning_loss = 0
        tpos_err = 0
        vpos_err = 0
        tori_err = 0
        vori_err = 0
        model.train()
        #print(next(enumerate(train_loader)))
        #print(next(iter(train_loader)))
        for batch, sample in enumerate(train_loader,0):
            #print(len(sample))
            points, target_pose = sample
            # print(type(points))
            # print(len(points))
            # print(points.shape)
            #print(points[0])
            points = points.to(DEVICE)
            target_pose = target_pose.to(DEVICE)
            target_pose[:,3:] = space.metric.log(target_pose[:,3:],base_point)

            #output_pose = model(points)
            #pos, ori, x = model(points,target_pose[:,:3])
            #loss = criterion(output_pose,target_pose)
            #ploss = criterion(pos,target_pose[:,:3])
            #oloss = criterion(ori,target_pose[:,3:])

            #ploss = model.mdn.loss(x,target_pose[:,:3]).mean()
            #oloss = model.qmdn.loss(torch.cat((target_pose[:,:3],x),dim=1),space.metric.log(target_pose[:,3:],base_point)).mean()
            
            #tloss = (ploss + oloss)*0.5

            pos, ori, x = model(points)
            tloss = model.jmdn.loss(x,target_pose).mean()

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
                target_pose[:,3:] = space.metric.log(target_pose[:,3:],base_point)

                # output_pose = model(points)
                # loss = criterion(output_pose,target_pose)
                #pos, ori, x = model(points, target_pose[:,:3])
                #loss = criterion(output_pose,target_pose)
                #ploss = criterion(pos,target_pose[:,:3])
                #oloss = criterion(ori,target_pose[:,3:])

                #ploss = model.mdn.loss(x,target_pose[:,:3]).mean()
                #oloss = model.qmdn.loss(torch.cat((target_pose[:,:3],x),dim=1),space.metric.log(target_pose[:,3:],base_point)).mean()

                #valloss = (ploss + oloss)*0.5

                pos, ori, x = model(points)
                valloss = model.jmdn.loss(x,target_pose).mean()

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

def my_collate(batch):
    # #print(batch)
    # print(type(batch))
    # #print(batch[0][0])
    # print(type(batch[0][0]))
    # print(type(batch[0][0][0]))
    # print(len(batch[0][0]))
    # print(len(batch[0]))
    # print(len(batch))
    # print(batch[0][0][0].shape)
    data = torch.stack([item[0][0] for item in batch])
    print(data.shape)
    target = torch.stack([item[1][0] for item in batch])
    return [data, target]

def main():
    # model_file = MODELS_PATH + 'model_nn.pt'
    # train_network(50,"dual_model_46ks_nwl_dh_tansphere_fd",samples=FULL_DATASET)
    torch.set_default_dtype(torch.float32)
    torch.set_default_device(DEVICE)
    model_file = MODELS_PATH + 'dual_fb_mdn34_qmdn48_wmdn_model_sql_15ks_nwl_dh_tansphere_scratch_ypos_ep30.pt'
    train_network(50,"dual_fb_jmdn128_dropout_fullcov_wgp_model_sql_5ks_nwl_dh_tansphere_scratch",samples=FULL_DATASET)

    # conn = sqlite3.connect(DATABASE_PATH)
    # #db_uri = "sqlite:///catkin_ws/src/graspnet/" + 'grasp_database_test_cracker_box.db'
    # query_str = "SELECT x,y,z,i,j,k,w, (SELECT pcd_binary FROM point_clouds_table WHERE point_clouds_table.pcd_id = grasps_table.pcd_id) FROM grasps_table WHERE grasp_id < 200;"
    # sql_dataset = Dataset.from_sql(query_str,conn)
    # print(sql_dataset)
    # sql_dataset = sql_dataset.with_format('torch',device=DEVICE)
    # print(sql_dataset)
    # print(sql_dataset.shape)
    # #print(sql_dataset[0])
    # print(type(sql_dataset))

    # sdb = GraspDataset('train',DATASET_PATH)
    # sdb = SQL_GraspDataset('train',DATABASE_PATH)
    # # train_loader = torch.utils.data.DataLoader(sdb,5,shuffle=True)
    # print(sdb)
    # # dataiter = iter(train_loader)
    # # print(dataiter[0])
    # print(sdb[999])
    # # print(len(sdb[13]))
    # # print(sdb[1:3])
    # # print(len(sdb[1:3]))
    # # print(sdb[[1,2,3]])
    # # for i in range(len(sdb)):
    # #     print(sdb[i+1][1])

    # dataset = GraspDataset(set_type="test",path=DATASET_PATH)
    # dataloader = torch.utils.data.DataLoader(dataset, BATCH_SIZE, shuffle=True)
    # train_features, train_labels = next(iter(dataloader))
    # print(f"Feature batch shape: {train_features.size()}")
    # print(f"Labels batch shape: {train_labels.size()}")
    # 

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass