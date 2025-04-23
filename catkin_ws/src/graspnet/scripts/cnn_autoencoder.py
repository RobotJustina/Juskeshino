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
import open3d
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
DATABASE_PATH = "/home/robocup/Juskeshino/catkin_ws/src/graspnet/" + '/grasp_database_nm_test2.db'
VAL_TO_TEST_RATIO = 0.1
FULL_DATASET = -1
BATCH_SIZE = 200

y_loss = {}  # loss history
y_loss['train'] = []
y_loss['val'] = []
x_epoch = []

##Class definitions

#  defining encoder
class Encoder(nn.Module):
    def __init__(self):
        super().__init__()
        self.conv1 = nn.Sequential(
                                nn.Dropout2d(0.05),
                                nn.Conv2d(3, 32, kernel_size = 11, stride = 1, bias=False),
                                nn.BatchNorm2d(32),
                                nn.GELU()
                                )
        self.maxpool = nn.MaxPool2d(kernel_size = 2, stride = 2, padding = 0)
        self.conv2 = nn.Sequential(
                            nn.Dropout2d(0.05),
                            nn.Conv2d(32, 64, kernel_size = 6, stride = 1, bias=False),
                            nn.BatchNorm2d(64),
                            nn.ReLU())
        self.conv3 = nn.Sequential(
                            nn.Dropout2d(0.08),
                            nn.Conv2d(64, 128, kernel_size = 4, stride = 2, bias=False),
                            nn.BatchNorm2d(128),
                            nn.ReLU())
        self.conv4 = nn.Sequential(
                            nn.Dropout2d(0.08),
                            nn.Conv2d(128, 256, kernel_size = 4, stride = 2, bias=False),
                            nn.BatchNorm2d(256),
                            nn.ReLU())
        self.conv5 = nn.Sequential(
                            nn.Dropout2d(0.1),
                            nn.Conv2d(256, 512, kernel_size = 3, stride = 2, bias=False),
                            nn.BatchNorm2d(512),
                            nn.ReLU(),
                            nn.Dropout2d(0.1))
        self.conv6 = nn.Sequential(
                            nn.Dropout2d(0.15),
                            nn.Conv2d(512, 512, kernel_size = 2, stride = 2, padding=1, bias=False),
                            nn.BatchNorm2d(512),
                            nn.ReLU()
                            )
        self.maxpool2 = nn.MaxPool2d(kernel_size=2,stride=1,padding=0)
        self.fc = nn.Sequential(
                            nn.Dropout(0.5),
                            nn.ReLU(),
                            nn.Linear(5*5*512,1024)
        )

    def forward(self, x):
        x = self.conv1(x)
        x = self.maxpool(x)
        x = self.conv2(x)
        x = self.conv3(x)
        x = self.conv4(x)
        x = self.conv5(x)
        x = self.conv6(x)
        x = self.maxpool2(x)
        x = torch.flatten(x,1)
        x = self.fc(x)
        return x


#  defining decoder
class Decoder(nn.Module):
    def __init__(self):
        super().__init__()
        self.fc = nn.Sequential(
                            nn.Linear(1024,5*5*512),
                            nn.ReLU(),
                            nn.Dropout(0.5)
                            )
        self.upscale1 = nn.Upsample((6,6))
        self.convt1 = nn.Sequential(
                            nn.ConvTranspose2d(512, 512, kernel_size = 2, stride = 2, padding=1, bias=False),
                            nn.BatchNorm2d(512),
                            nn.ReLU(),
                            nn.Dropout2d(0.1)
                            )
        self.convt2 = nn.Sequential(
                            nn.ConvTranspose2d(512, 256, kernel_size = 3, stride = 2, bias=False),
                            nn.BatchNorm2d(256),
                            nn.ReLU(),
                            nn.Dropout2d(0.1))
        self.convt3 = nn.Sequential(
                            nn.ConvTranspose2d(256, 128, kernel_size = 4, stride = 2, bias=False),
                            nn.BatchNorm2d(128),
                            nn.ReLU(),
                            nn.Dropout2d(0.08))
        self.convt4 = nn.Sequential(
                            nn.ConvTranspose2d(128, 64, kernel_size = 4, stride = 2, bias=False),
                            nn.BatchNorm2d(64),
                            nn.ReLU(),
                            nn.Dropout2d(0.08))
        self.convt5 = nn.Sequential(
                            nn.ConvTranspose2d(64, 32, kernel_size = 6, stride = 1, bias=False),
                            nn.BatchNorm2d(32),
                            nn.ReLU(),
                            nn.Dropout2d(0.05))
        self.upscale2 = nn.Upsample((190,190))
        self.convt6 = nn.Sequential(
                            nn.ConvTranspose2d(32, 3, kernel_size = 11, stride = 1, bias=False),)
        

    def forward(self, x):
        x = self.fc(x)
        x = x.view(-1,512,5,5)
        x = self.upscale1(x)
        x = self.convt1(x)
        x = self.convt2(x)
        x = self.convt3(x)
        x = self.convt4(x)
        x = self.convt5(x)
        x = self.upscale2(x)
        x = self.convt6(x)
        return x


#  defining autoencoder
class Autoencoder(nn.Module):
    def __init__(self, encoder, decoder):
        super().__init__()
        self.encoder = encoder
        self.decoder = decoder

    def forward(self, x):
        encoded = self.encoder(x)
        decoded = self.decoder(encoded)
        return decoded
    
##Dataset classes and methods

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
    
    def __getitem__(self, idx):
        self.cursor.execute("SELECT pcd_binary FROM point_clouds_table WHERE point_clouds_table.pcd_id = {}".format(idx +1))
        pcd_binary = self.cursor.fetchone()[0]
        self.conn.commit()
        pcd = np.load(self.bytes_io(pcd_binary))
        pcd = rf.structured_to_unstructured(pcd)
        pcd = torch.tensor(pcd[:,:,:3],dtype=torch.float32)
        pcd = torch.nan_to_num(pcd,nan=0.0)
        pcd = torch.permute(pcd,(2,1,0))
        return pcd
    
    def __len__(self):
        self.cursor.execute('SELECT seq FROM sqlite_sequence WHERE name="point_clouds_table"')
        last_id = self.cursor.fetchone()[0]
        self.conn.commit()
        return last_id

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
    train_loader = torch.utils.data.DataLoader(train_dataset, BATCH_SIZE, shuffle=True, num_workers=1,pin_memory=True)#,generator=torch.Generator(DEVICE))
    print(len(train_loader))
    valid_loader = torch.utils.data.DataLoader(valid_dataset, BATCH_SIZE, shuffle=True, num_workers=1,pin_memory=True)#,generator=torch.Generator(DEVICE))

    return train_loader, valid_loader

##Define training function:
def train_network(num_epochs,model_name, model_path=None,samples =-1):
    torch.cuda.empty_cache()
    gc.collect()
    train_loader, valid_loader = get_SQL_dataloaders(n_samples=samples)
    train_size = len(train_loader.dataset)
    val_size = len(valid_loader.dataset)
    model = load_model(model_path)
    best_model = copy.deepcopy(model.state_dict())
    criterion = nn.HuberLoss(delta=1,reduction='mean')
    min_loss = 150000
    optimizer = optim.AdamW(model.parameters(),lr=0.0001,weight_decay=0.0002)

    for epoch in range(num_epochs):
        trunning_loss = 0
        vrunning_loss = 0
        tpos_err = 0
        vpos_err = 0
        tori_err = 0
        vori_err = 0
        model.train()

        for batch, pcd in enumerate(train_loader,0):
            pcd = pcd.cuda(non_blocking=True)
            out = model(pcd)
            tloss = criterion(out,pcd)

            optimizer.zero_grad()
            tloss.backward()
            optimizer.step()
            
            trunning_loss += tloss.item() * pcd.shape[0]

            del pcd, out
            torch.cuda.empty_cache()
            gc.collect()
        
        model.eval()    
        with torch.no_grad():
            for pcd in valid_loader:
                pcd = pcd.cuda(non_blocking=True)
                out = model(pcd)
                valloss = criterion(out,pcd)

                vrunning_loss += valloss.item() * pcd.shape[0]

                del pcd, out
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
        x_epoch.append(epoch+1)
    model_file = MODELS_PATH + model_name + "_ep" + str(saved_epoch) + ".pt"
    torch.save(best_model,model_file)
    model_file = MODELS_PATH + model_name + "_ep" + str(num_epochs) + ".pt" 
    torch.save(model.state_dict(),model_file)
    draw_curve(num_epochs)
    print(min_loss)
    print(saved_epoch)
    dst = valid_loader.dataset
    pcdt = dst[0].to(DEVICE)
    show_pcd_from_tensor(pcdt)
    model.eval()    
    with torch.no_grad():
        out = model(pcdt.unsqueeze(0))
    show_pcd_from_tensor(out.squeeze(0))

def load_model(model_path=None):
    enc = Encoder()
    dec = Decoder()
    model = Autoencoder(enc,dec)
    if model_path:
        model.load_state_dict(torch.load(model_path,weights_only=True))
    model.to(DEVICE)
    return model

##Utils

def draw_curve(current_epoch):
    fig = plt.figure()
    ax0 = fig.add_subplot(111, title="Loss")
    ax0.plot(x_epoch, y_loss['train'], 'bo-', label='train')
    ax0.plot(x_epoch, y_loss['val'], 'ro-', label='val')
    ax0.grid()
    fig.supxlabel('Epochs')
    fig.tight_layout()
    fig.savefig(os.path.join(GRAPHS_PATH, 'new_joined_head.eps'))
    plt.show()

def show_pcd_from_tensor(tensor):
    tensor = torch.permute(tensor,(2,1,0))
    rgb = tensor.detach().cpu().numpy()
    print(rgb.shape)
    rgb = rgb.reshape((-1,3))
    print(rgb.shape)
    #rgb = rf.structured_to_unstructured(rgb)
    rgb = rgb[~np.isnan(rgb).any(axis=1)]
    view_point_cloud = open3d.geometry.PointCloud()
    view_point_cloud.points = open3d.utility.Vector3dVector(copy.deepcopy(rgb[:,:3]))
    view_point_cloud.colors = open3d.utility.Vector3dVector(copy.deepcopy(rgb[:,:3]))
    open3d.visualization.draw_geometries([view_point_cloud])

def load_and_split_model(model_path,save_path):
    model = load_model(model_path)
    torch.save({
        'encoder_state_dict': model.encoder.state_dict(),
        'decoder_state_dict': model.decoder.state_dict()
    },save_path)

def quick_dummy_test():
    enc = Encoder()
    dec = Decoder()
    cnnae = Autoencoder(enc,dec)
    cnnae.to(DEVICE)
    z_test = torch.zeros((1,3,200,200)).to(DEVICE)
    print(cnnae(z_test).shape)
    train_network(100,MODEL_NAME)

def main():
    #quick_dummy_test()
    m_path = MODELS_PATH + 'dummy_cae_10ks_ep97.pt'
    save_path = MODELS_PATH + 'dummy_cae_10ks_ep97_split_dict.pt'
    load_and_split_model(m_path,save_path)

if __name__ == '__main__':
    main()