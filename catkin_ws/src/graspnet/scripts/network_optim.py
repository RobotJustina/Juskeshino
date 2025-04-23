#!/usr/bin/env python3

import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
import geotorch as geo
import numpy as np
import copy
import numpy.lib.recfunctions as rf
import ros_numpy
from geometry_msgs.msg import Point
from scipy.spatial import cKDTree
import math
import optuna
from optuna.trial import TrialState
import optuna.visualization as opvis
import torch.utils.data
import os
import numpy.lib.recfunctions as rf
import gc
import matplotlib.pyplot as plt
import math
from sklearn.model_selection import train_test_split
from torch.utils.data import Subset
from train import get_dataloaders

DEVICE = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
MAX_TRIAL_EPOCHS = 32
PRUNING_EPOCHS = 8
BATCH_SIZE = 100
DATASET_PATH = "/home/robocup/Juskeshino/catkin_ws/src/graspnet/training_dataset/"
MODELS_PATH = "/home/robocup/Juskeshino/catkin_ws/src/graspnet/models/"
GRAPHS_PATH = "/home/robocup/Juskeshino/catkin_ws/src/graspnet/graphs/"
VAL_DATASET_PATH = "/home/robocup/Juskeshino/catkin_ws/src/graspnet/validate_dataset/"
VAL_TO_TEST_RATIO = 0.1
FULL_DATASET = -1
ACTIVATION_FUNC_DICT = {'gelu':nn.GELU,'relu':nn.ReLU}
#OPTIMIZER_DICT = {'SGD':optim.sgd,'AdamW':optim.adamw}
#Network definition

class ResidualBlock(nn.Module):
        def __init__(self, in_channels, out_channels, stride = 1, downsample = None):
            super(ResidualBlock, self).__init__()
            self.conv1 = nn.Sequential(
                            nn.Conv2d(in_channels, out_channels, kernel_size = 3, stride = stride, padding = 1),
                            nn.BatchNorm2d(out_channels),
                            nn.ReLU())
            self.conv2 = nn.Sequential(
                            nn.Conv2d(out_channels, out_channels, kernel_size = 3, stride = 1, padding = 1),
                            nn.BatchNorm2d(out_channels))
            self.downsample = downsample
            self.relu = nn.ReLU()
            self.out_channels = out_channels

        def forward(self, x):
            residual = x
            out = self.conv1(x)
            out = self.conv2(out)
            if self.downsample:
                residual = self.downsample(x)
            out += residual
            out = self.relu(out)
            return out

class ResidualGraspNetwork(nn.Module):
    def __init__(self):
        super().__init__()
        self.conv1 = nn.Sequential(
                            nn.Conv2d(6, 32, kernel_size = 3, stride = 1),
                            nn.BatchNorm2d(32),
                            nn.ReLU())
        self.maxpool = nn.MaxPool2d(kernel_size = 3, stride = 2, padding = 1)
        self.reslayer1 = self._make_layer(block, 64, layers[0], stride = 1)
        self.reslayer2 = self._make_layer(block, 128, layers[1], stride = 2)
        self.reslayer3 = self._make_layer(block, 256, layers[2], stride = 2)
        self.reslayer4 = self._make_layer(block, 512, layers[3], stride = 2)
        self.fc1 = nn.Linear(512*3*3,1024)
        self.fc2 = nn.Linear(1024,128)
        self.fc3 = nn.Linear(128,6)
        
    def _make_residual_layer(self, block, planes, blocks, stride=1):
            downsample = None
            if stride != 1 or self.inplanes != planes:
                downsample = nn.Sequential(
                    nn.Conv2d(self.inplanes, planes, kernel_size=1, stride=stride),
                    nn.BatchNorm2d(planes),
                )
            layers = []
            layers.append(block(self.inplanes, planes, stride, downsample))
            self.inplanes = planes
            for i in range(1, blocks):
                layers.append(block(self.inplanes, planes))

            return nn.Sequential(*layers)

class GraspNetwork(nn.Module):
    def __init__(self,trial):
        super().__init__()
        #conv1_af = trial.suggest_categorical('C1_ActF',['gelu','relu'])
        flat1 = trial.suggest_int("cnn_out_neurons{}".format(1), 4096,8192)
        fkernels = trial.suggest_int("cnn_out_kernels{}".format(1), 128,512)

        self.conv1 = nn.Sequential(
                            #nn.Dropout2d(0.05),
                            nn.Conv2d(3, 32, kernel_size = 11, stride = 1),
                            nn.BatchNorm2d(32),
                            #ACTIVATION_FUNC_DICT[conv1_af]()
                            #nn.GELU()
                            )
        self.maxpool = nn.MaxPool2d(kernel_size = 2, stride = 2, padding = 0)
        self.conv2 = nn.Sequential(
                            nn.Conv2d(32, 64, kernel_size = 6, stride = 1),
                            nn.BatchNorm2d(64),
                            nn.ReLU())
        self.conv3 = nn.Sequential(
                            nn.Conv2d(64, 128, kernel_size = 4, stride = 2),
                            nn.BatchNorm2d(128),
                            nn.ReLU())
        self.conv4 = nn.Sequential(
                            nn.Conv2d(128, 256, kernel_size = 4, stride = 2),
                            nn.BatchNorm2d(256),
                            nn.ReLU())
        self.conv5 = nn.Sequential(
                            nn.Conv2d(256, 512, kernel_size = 3, stride = 2),
                            nn.BatchNorm2d(512),
                            nn.ReLU())
                            #,nn.Dropout2d(0.1))
        self.conv6 = nn.Sequential(
                            nn.Conv2d(512, fkernels, kernel_size = 2, stride = 2, padding=1),
                            nn.BatchNorm2d(fkernels),
                            #nn.GELU()
                            )
        self.maxpool2 = nn.MaxPool2d(kernel_size=2,stride=1,padding=0)
        self.lc  = nn.Sequential(
                            nn.Linear(5*5*fkernels,flat1),
                            nn.ReLU(),
                            nn.Dropout(0.5),
                            nn.Linear(flat1,2048),
                            # nn.ReLU(),
                            # nn.Linear(4096, 2048),
                            nn.ReLU(),
                            nn.Dropout(0.4),
                            nn.Linear(2048,512),
                            nn.ReLU(),
                            nn.Dropout(0.25),
                            nn.Linear(512,256),
                            nn.ReLU(),
                            nn.Dropout(0.25),
                            nn.Linear(256,256),
                            nn.Linear(256,3))
                            #nn.Linear(64,3))
        self.nlc = nn.Sequential(
                            nn.Linear(5*5*fkernels,flat1),
                            nn.ReLU(),
                            nn.Dropout(0.5),
                            nn.Linear(flat1,2048),
                            # nn.ReLU(),
                            # nn.Linear(4096, 2048),
                            nn.ReLU(),
                            nn.Dropout(0.5),
                            nn.Linear(2048, 512),
                            nn.ReLU(),
                            nn.Dropout(0.4),
                            nn.Linear(512,512),
                            nn.ReLU(),
                            nn.Dropout(0.25),
                            nn.Linear(512,256),
                            nn.ReLU(),
                            nn.Linear(256,4))
                            # nn.ReLU(),
                            # nn.Linear(64,4),
                            #geo.Sphere()
                            #nn.Tanh())
        #self.nlch = nn.Linear(256,4)
        #ten = getattr(self.nlch,)
        #self.manif = geo.Sphere(ten.size(),1)
        self.manif = geo.Sphere([1,4],1)
        self.manif.base = torch.tensor([0,0,0,1],dtype=torch.float32)
        #self.manif = geo.SphereEmbedded(self.nlc,1)
        #self.nlch = nn.Linear(256,4)
        #geo.sphere(self.nlch, tensor_name="output")
        #geo.SphereEmbedded(self.nlc,1)
        #geo.orthogonal()

    def forward(self, x):
        x = self.conv1(x)
        x = self.maxpool(x)
        x = F.gelu(x)
        x = self.conv2(x)
        x = self.conv3(x)
        x = self.conv4(x)
        x = self.conv5(x)
        x = self.conv6(x)
        x = self.maxpool2(x)
        x = F.gelu(x)
        x = torch.flatten(x,1)

        pos = self.lc(x)
        ori = self.nlc(x)
        ori = self.manif(ori)

        return pos, ori
    
def train_network(num_epochs, trial, train_loader, valid_loader):
    train_size = len(train_loader.dataset)
    val_size = len(valid_loader.dataset)
    model = GraspNetwork(trial).to(DEVICE)
    #best_model = copy.deepcopy(model.state_dict())
    criterion = nn.HuberLoss(delta=0.9)
    exp_error = nn.L1Loss(reduction='mean')
    min_loss = 15000

    suggested_weight_decay = trial.suggest_float('weight_decay',1e-4,1e-1, log=True)
    suggested_lr = trial.suggest_float('lr',1e-6,1e-3, log=True)
    #suggested_optimizer = trial.suggest_categorical('optimizer',['SGD','AdamW'])
    
    #optimizer = optim.SGD(model.parameters(),lr=0.00008,momentum=0.8)
    # if suggested_optimizer == 'SGD':
    #     suggested_momentum = trial.suggest_float('momentum',0.8,0.95,log=True)
    #     optimizer = optim.SGD(model.parameters(),lr=suggested_lr,momentum=suggested_momentum,nesterov=True,weight_decay=suggested_weight_decay)
    
    #optimizer = optim.Adam(model.parameters(),lr=0.00008)
    #if suggested_optimizer == 'AdamW':
    optimizer = optim.AdamW(model.parameters(),lr=suggested_lr,weight_decay=suggested_weight_decay)
    
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

            pos, ori = model(points)
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
                pos, ori = model(points)
                ploss = criterion(pos,target_pose[:,:3])
                oloss = criterion(ori,target_pose[:,3:])
                valloss = ploss + oloss

                vrunning_loss += valloss.item() * points.shape[0]
                vpos_err += exp_error(pos,target_pose[:,:3]).item() * points.shape[0]
                vori_err += exp_error(ori,target_pose[:,3:]).item() * points.shape[0]

                del points, target_pose, pos, ori
                torch.cuda.empty_cache()
                gc.collect()
            if (vrunning_loss / val_size) < min_loss:
                min_loss = vrunning_loss / val_size
                saved_epoch = epoch + 1
                #best_model = copy.deepcopy(model.state_dict())       
        trial.report(vrunning_loss / val_size, epoch)

        # Handle pruning based on the intermediate value.
        if epoch > PRUNING_EPOCHS:
            if trial.should_prune():
                raise optuna.exceptions.TrialPruned()
        
    print ('Trial number [{}], Validation Loss: {:.8f}'.format(trial.number,min_loss))
    return(min_loss)

def objective(trial):
    torch.cuda.empty_cache()
    gc.collect()
    train_loader, valid_loader = get_dataloaders(n_samples=4200)
    trial_loss = train_network(MAX_TRIAL_EPOCHS,trial,train_loader,valid_loader)
    return trial_loss

def main():
    study_number = 3
    study_id = "network_optim" + str(study_number) # Unique identifier of the study.
    study_storage = "sqlite:///catkin_ws/src/graspnet/{}.db".format(study_id)
    study = optuna.create_study(study_name=study_id,storage=study_storage, direction="minimize")
    study.optimize(objective, n_trials=200, timeout=None)

    pruned_trials = study.get_trials(deepcopy=False, states=[TrialState.PRUNED])
    complete_trials = study.get_trials(deepcopy=False, states=[TrialState.COMPLETE])

    print("Study statistics: ")
    print("  Number of finished trials: ", len(study.trials))
    print("  Number of pruned trials: ", len(pruned_trials))
    print("  Number of complete trials: ", len(complete_trials))

    print("Best trial:")
    trial = study.best_trial

    print("  Value: ", trial.value)

    print("  Params: ")
    for key, value in trial.params.items():
        print("    {}: {}".format(key, value))
    
    fig = opvis.plot_parallel_coordinate(study)
    plt.show()



if __name__ == '__main__':
    main()