#!/usr/bin/env python3

import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
import geotorch as geo
import numpy as np
import copy
import numpy.lib.recfunctions as rf
#import ros_numpy
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
import geomstats.backend as gs
from geomstats.geometry.hypersphere import Hypersphere
from sklearn.model_selection import train_test_split
from torch.utils.data import Subset
from cnn_autoencoder import Encoder
from ori_network3 import get_SQL_dataloaders, find_kmeans_centers
from kernel_mixture_network import Kernel_Mixture_Network

DEVICE = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
MAX_TRIAL_EPOCHS = 20
PRUNING_EPOCHS = 2
BATCH_SIZE = 192
DATASET_PATH = "/home/robocup/Juskeshino/catkin_ws/src/graspnet/training_dataset/"
MODELS_PATH = "/home/robocup/Juskeshino/catkin_ws/src/graspnet/models/"
BEST_MODEL_PATH = "/home/robocup/Juskeshino/catkin_ws/src/graspnet/models/best_ori_model.pt"
GRAPHS_PATH = "/home/robocup/Juskeshino/catkin_ws/src/graspnet/graphs/"
VAL_DATASET_PATH = "/home/robocup/Juskeshino/catkin_ws/src/graspnet/validate_dataset/"
VAL_TO_TEST_RATIO = 0.1
FULL_DATASET = -1
#ACTIVATION_FUNC_DICT = {'gelu':nn.GELU,'relu':nn.ReLU}
#OPTIMIZER_DICT = {'SGD':optim.sgd,'AdamW':optim.adamw}
#Network definition

class Orientation_head(nn.Module):
    def __init__(self):
        super().__init__()
        kcenters = torch.load(MODELS_PATH + 'kcenters_256.pt',weights_only=True)
        self.kmm = Kernel_Mixture_Network(1027,256,len(kcenters),kcenters,4,320)
        self.space = Hypersphere(3)
        self.BASE_POINT = torch.tensor([0.0,0.0,0.0,1.0],dtype=torch.float64).to(DEVICE)
        #print(self)

    def forward(self, x, pos):
        x = torch.cat((x,pos),dim=1)
        ori = self.kmm.sample(x)
        return ori
    
    def L1_loss(self, q1, q2):
        d = torch.abs(self.space.metric.dist(q1,q2)).mean()
        return d
    
    def L2_loss(self, q1,q2):
        d = torch.square(self.space.metric.dist(q1,q2)).mean()
        return d
    
    def prob_loss(self, x, y):
        return self.kmm.loss(x,y)
    

class Orientation_network(nn.Module):
    def __init__(self, trial, enc_state_dict = None, kcenters = None):
        super().__init__()
        self.enc = Encoder().float()
        if enc_state_dict:
            self.enc.load_state_dict(enc_state_dict)
        if kcenters == None:
            kcenters = torch.load(MODELS_PATH + 'kcenters_256.pt',weights_only=True)
            #print(kcenters)
            #print(kcenters.dtype)
        print(kcenters)
        self.kmm = Kernel_Mixture_Network(1027,256,len(kcenters),kcenters,4,trial.suggest_int('kappa',25,500))
        self.space = Hypersphere(3)
        self.BASE_POINT = torch.tensor([0.0,0.0,0.0,1.0],dtype=torch.float64).to(DEVICE)
        #print(self)

    def forward(self, x, pos):
        x = self.enc(x)
        x = torch.cat((x,pos),dim=1)
        ori = self.kmm(x)
        return x, ori
    
    def L1_loss(self, q1, q2):
        d = torch.abs(self.space.metric.dist(q1,q2)).mean()
        return d
    
    def L2_loss(self, q1,q2):
        d = torch.square(self.space.metric.dist(q1,q2)).mean()
        return d
    
    def prob_loss(self, x, y):
        return self.kmm.loss(x,y)
    
    def predict(self, x):
        return self.kmm.sample(x)
    
def train_network(num_epochs, trial, train_loader, valid_loader):
    global best_model_loss
    train_size = len(train_loader.dataset)
    val_size = len(valid_loader.dataset)
    #cae_path = MODELS_PATH + 'dummy_cae_10ks_ep97_split_dict.pt'
    cae_path = MODELS_PATH + 'conv_autoencoder_final_ep67.pt'
    enc_dict = torch.load(cae_path,weights_only=True)['encoder_state_dict']
    kcenters = torch.load(MODELS_PATH + 'kcenters_256.pt',weights_only=True)
    model = Orientation_network(trial, enc_dict, kcenters).to(DEVICE)
    min_loss = best_model_loss
    suggested_weight_decay = trial.suggest_float('weight_decay',1e-4,1e-1, log=False)
    suggested_initlr = trial.suggest_float('init_lr',1e-6,1e-5, log=False)
    suggested_maxlr = trial.suggest_float('max_lr',1e-4,1e-2, log=False)
    optimizer = optim.AdamW([
                {'params': model.enc.parameters()},
                {'params': model.kmm.wi_network.parameters(), 'lr': suggested_initlr}
            ],lr=suggested_initlr,weight_decay=suggested_weight_decay)
    #lrscheduler = torch.optim.lr_scheduler.ReduceLROnPlateau(optimizer,'min',factor=0.1,patience=3,threshold=)
    lrscheduler = torch.optim.lr_scheduler.OneCycleLR(optimizer=optimizer,max_lr=suggested_maxlr,steps_per_epoch=len(train_loader),epochs=num_epochs)
    for epoch in range(num_epochs):
        trunning_loss = 0
        vrunning_loss = 0
        model.train()

        for batch, (pcd, pos, target_ori) in enumerate(train_loader,0):
            cpu_ori = target_ori
            pcd, pos, target_ori = pcd.to(DEVICE, non_blocking = True), pos.to(DEVICE, non_blocking = True), target_ori.to(DEVICE, non_blocking = True)

            with torch.device(DEVICE):
                x, wi = model(pcd, pos)
                #ori = model.predict(x)
                #print(ori.shape)
                tloss = model.kmm.loss(x,cpu_ori).mean()
                #tloss = model.L2_loss(ori,target_ori)

                optimizer.zero_grad()
                tloss.backward()
                optimizer.step()
                lrscheduler.step()
            
                trunning_loss += tloss.item() * pcd.shape[0]

                del pcd, target_ori, cpu_ori, pos, x
                torch.cuda.empty_cache()
                gc.collect()

        #lrscheduler.step()

        model.eval()    
        with torch.no_grad():
            for pcd, pos, target_ori in valid_loader:
                pcd = pcd.to(DEVICE, non_blocking = True)
                pos = pos.to(DEVICE, non_blocking = True)
                #cpu_ori = copy.deepcopy(target_ori)
                cpu_ori = target_ori
                target_ori = target_ori.to(DEVICE, non_blocking = True)
                
                with torch.device(DEVICE):
                    x, wi = model(pcd, pos)
                    #ori = model.predict(x)
                    valloss = model.kmm.loss(x,cpu_ori).mean()
                    #valloss = model.L2_loss(ori, target_ori)

                    vrunning_loss += valloss.item() * pcd.shape[0]
                    #vori_err += exp_error(ori,space.metric.exp(target_ori,BASE_POINT)).item() * pcd.shape[0]
                    #vori_err += exp_error(ori,target_ori).item() * pcd.shape[0]

                    del pcd, target_ori, cpu_ori, pos, x
                    torch.cuda.empty_cache()
                    gc.collect()
            if (vrunning_loss / val_size) < min_loss:
                torch.save({
                    'encoder_state_dict': model.enc.state_dict(),
                    'ori_state_dict': model.kmm.state_dict()
                },BEST_MODEL_PATH)
                min_loss = vrunning_loss / val_size
                best_model_loss = vrunning_loss / val_size
        #print ('Epoch [{}/{}], Training Loss: {:.4f}'.format(epoch+1, num_epochs, trunning_loss / train_size))
        #print ('Epoch [{}/{}], Validation Loss: {:.4f}'.format(epoch+1, num_epochs, vrunning_loss / val_size))

        trial.report(vrunning_loss / val_size, epoch)

        # Handle pruning based on the intermediate value.
        if epoch > PRUNING_EPOCHS:
            if trial.should_prune():
                raise optuna.exceptions.TrialPruned()
            
    # if min_loss < best_model_loss:
    #     torch.save({
    #         'encoder_state_dict': best_enc_model,
    #         'ori_state_dict': best_ori_model
    #     },BEST_MODEL_PATH)

    print ('Trial number [{}], Validation Loss: {:.8f}'.format(trial.number,min_loss))
    return(min_loss)

def objective(trial):
    torch.cuda.empty_cache()
    gc.collect()
    train_loader, valid_loader = get_SQL_dataloaders(n_samples=4200)
    trial_loss = train_network(MAX_TRIAL_EPOCHS,trial,train_loader,valid_loader)
    return trial_loss

def main():
    global best_model_loss
    best_model_loss = 10
    study_number = 1
    study_id = "ori_network_optim" + str(study_number) # Unique identifier of the study.
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