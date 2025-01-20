#!/home/robocup/regnet/REGNet_for_3D_Grasping/env/bin/python
import argparse
import os
import sys
import pickle, copy

import torch
import torch.utils.data
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
import numpy as np
import open3d
from tensorboardX import SummaryWriter
from torch.optim.lr_scheduler import StepLR, MultiStepLR
import time

REGNET_PATH = '/home/robocup/regnet/REGNet_for_3D_Grasping'
sys.path.insert(0,REGNET_PATH)
from dataset_utils.get_regiondataset import get_grasp_allobj
from dataset_utils.eval_score.eval import eval_test, eval_validate
#from vis.vis_grasp import inv_transform_grasp
import utils
import glob

#Gpu config
gpu_number = 1
gpus = 0
gpu_arr = '0'
np.random.seed(int(time.time()))
torch.cuda.manual_seed(1)
torch.cuda.set_device(gpus)

# Gripper and pointcloud properties
all_points_num = 25600
obj_class_num = 43
width, height, depth = 0.08, 0.010, 0.06
table_height = 0.75
grasp_score_threshold = 0.5 # 0.3
center_num = 4000#64#128
score_thre = 0.5
group_num=256
group_num_more=2048
r_time_group = 0.1
r_time_group_more = 0.8
gripper_num = 64
use_theta = True
reg_channel = 10

gripper_params = [width, height, depth]
model_params   = [obj_class_num, group_num, gripper_num, grasp_score_threshold, depth, reg_channel]
params         = [center_num, score_thre, group_num, r_time_group, \
                    group_num_more, r_time_group_more, width, height, depth]
                    
#Model construction from saved model
score_model_path = REGNET_PATH + '/assets/models/final/score_42.model' 
region_model_path = REGNET_PATH + '/assets/models/final/region_42.model' 
load_saved_snet_flag = True
load_saved_rnet_flag = True
score_model, region_model, resume_epoch = utils.construct_net(model_params, 'test_one_file', gpu_num=gpus, 
                                load_score_flag=load_saved_snet_flag, score_path=score_model_path,
                                load_rnet_flag=load_saved_rnet_flag, rnet_path=region_model_path)
#Load model to gpu
score_model, region_model = utils.map_model(score_model, region_model, gpu_number, gpus, gpu_arr)

class RegnetModule():
    def __init__(self):
        self.eval_params    = [depth, width, table_height, gpus, center_num]
        self.params         = params
        self.gripper_params = gripper_params
        self.resume_epoch   = resume_epoch
        
    def process_pcd(self, pc_torch):   
        all_feature, output_score, _ = score_model(pc_torch)
        center_pc, center_pc_index, pc_group_index, pc_group, pc_group_more_index, \
                pc_group_more, _ = get_grasp_allobj(pc_torch, output_score, self.params, [], use_theta)

        grasp_stage2, keep_grasp_num_stage2, stage2_mask, _, _, _, select_grasp_class, select_grasp_score, \
                    select_grasp_class_stage2, keep_grasp_num_stage3, keep_grasp_num_stage3_score, \
                    stage3_mask, stage3_score_mask, _, _, _ = region_model(pc_group, pc_group_more, pc_group_index, \
                    pc_group_more_index, center_pc, center_pc_index, pc_torch, all_feature, self.gripper_params, None, [])
        return grasp_stage2, select_grasp_class, select_grasp_score, select_grasp_class_stage2, output_score
        
    def save_processed_grasps(self, pc_back, color_back, grasp_stage2, select_grasp_class, select_grasp_score, select_grasp_class_stage2, output_score, grasp_save_path):           
        record_stage2 = utils.eval_notruth(pc_back, color_back, grasp_stage2, select_grasp_class, select_grasp_score, select_grasp_class_stage2, output_score, self.eval_params, grasp_save_path)
        
    def test_file(self, epoch, pc_path, open3d_data=True):
        print("---------------Testing file with model epoch:", epoch, "------------------")
        score_model.eval()
        region_model.eval()
        torch.set_grad_enabled(False)
        pc_back, color_back, pc_torch, grasp_save_path = file_type_config_eval(open3d_data, pc_path)
        grasp_stage2, select_grasp_class, select_grasp_score, select_grasp_class_stage2, output_score = self.process_pcd(pc_torch)
        self.save_processed_grasps(pc_back, color_back, grasp_stage2, select_grasp_class, select_grasp_score, select_grasp_class_stage2, output_score, grasp_save_path)
    
def file_type_config_eval(open3d_data, pc_path):
    grasp_save_path = pc_path.replace('_data', '_data_predict')
    if open3d_data:
        data = open3d.io.read_point_cloud(pc_path)
        center_camera = np.array([0, 0, 1.658])
        data.transform(utils.local_to_global_transformation_quat(center_camera))
        pc = np.array(data.points)
        pc_color = np.array(data.colors)
        pc = np.c_[pc, pc_color]
        pc = pc[pc[:,0] < 0.26]
        pc = pc[pc[:,0] > -0.4]
        pc = pc[pc[:,2] < 1]
        pc = pc[pc[:,1] < 0.65]
        pc = pc[pc[:,1] > 0.2]
        grasp_save_path = grasp_save_path.replace('.pcd', '.p')
    else:
        data = np.load(pc_path, allow_pickle=True)
        print(data.keys())
        pc = data['view_cloud'].astype(np.float32)
        pc_color = data['view_cloud_color'].astype(np.float32)
        pc_score = data['view_cloud_score'].astype(np.float32)
        pc_label = data['view_cloud_label'].astype(np.float32)
        pc = np.c_[pc, pc_color]
        print("PC contents:")
        print(pc)
        print(pc[0])
        print(pc.shape)
        print(type(pc))
        print(pc[0].shape)
        print(type(pc[0]))
        print(pc[0,0].shape)
        print(type(pc[0,0]))
        print(pc[0,4].shape)
        print(type(pc[0,4]))
        
        print("PC SCORE contents:")
        print(pc_score)
        print(pc_score.shape)
        print(type(pc_score))
        print(pc_score[0])
        print(pc_score[0].shape)
        print(type(pc_score[0]))
        
        print("PC LABEL contents:")
        print(pc_label)
        print(pc_label.shape)
        print(type(pc_label))
        print(pc_label[0])
        print(pc_label[0].shape)
        print(type(pc_label[0]))
        
        
    pc_back, color_back = copy.deepcopy(pc[:,:3]), copy.deepcopy(pc[:,3:6])
    pc = utils.noise_color(pc)
    select_point_index = None
    if len(pc) >= all_points_num:
        select_point_index = np.random.choice(len(pc), all_points_num, replace=False)
    elif len(pc) < all_points_num:
        select_point_index = np.random.choice(len(pc), all_points_num, replace=True)
    pc = pc[select_point_index]
    print("PC numpy shape and dtype:")
    print(pc.shape)
    print(pc.dtype)
    print("PC elements shape and dtype:")
    print(pc[0,0], type(pc[0,0]))
    print(pc[0,1], type(pc[0,1]))
    print(pc[0,2], type(pc[0,2]))
    print(pc[0,3], type(pc[0,3]))
    print(pc[0,4], type(pc[0,4]))
    print(pc[0,5], type(pc[0,5]))

    pc_torch = torch.Tensor(pc).view(1, -1, 6)
    if gpus != -1:
        pc_torch = pc_torch.cuda()
        print("Converted tensor characteristics")
        print(pc_torch)
        print(pc_torch.shape)
        print(type(pc_torch))
    return pc_back, color_back, pc_torch, grasp_save_path

def nparray_pc_to_torch(pc):
    pc_back, color_back = copy.deepcopy(pc[:,:3]), copy.deepcopy(pc[:,3:6])
    pc = utils.noise_color(pc)
    select_point_index = None
    if len(pc) >= all_points_num:
        select_point_index = np.random.choice(len(pc), all_points_num, replace=False)
    elif len(pc) < all_points_num:
        select_point_index = np.random.choice(len(pc), all_points_num, replace=True)
    pc = pc[select_point_index]
    pc_torch = torch.Tensor(pc).view(1, -1, 6)
    if gpus != -1:
        pc_torch = pc_torch.cuda()
    print("Converted tensor characteristics")
    print(pc_torch)
    print(pc_torch.shape)
    print(type(pc_torch))
    return pc_back, color_back, pc_torch
                                                         
def main():
    refinedModule = RegnetModule()
    test_file_path = REGNET_PATH + '/test_file/virtual_data'
    open3d_data = True if 'real_data' in test_file_path else False
    print(open3d_data)
    if open3d_data:
        pc_paths = glob.glob(test_file_path+"/*.pcd",recursive=True)
    else:
        pc_paths = glob.glob(test_file_path+"/*.p",recursive=True)
    for pc_path in pc_paths:
        refinedModule.test_file(resume_epoch-1, pc_path, open3d_data)

if __name__ == "__main__":
    main()

