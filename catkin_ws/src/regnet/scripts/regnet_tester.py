#!/home/robocup/regnet/REGNet_for_3D_Grasping/env/bin/python

import rospy
import numpy as np
import ros_numpy
import glob
import torch
from regnetmodule import RegnetModule, REGNET_PATH
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2utils
import numpy.lib.recfunctions as rf
import regnetmodule as rm

DUMMY_FIELD_PREFIX = '__'
PROCESSED_PC_PATH = REGNET_PATH + '/ros'

def callback_pc2(msg):
    global pc2
    pc2 = msg

def main():
    global pc2
    refinedModule = RegnetModule()
    testfile()
    pc2 = PointCloud2()
    print("Starting regnet test node")
    rospy.init_node("regnet_node")
    rospy.Subscriber("/camera/depth_registered/points"   , PointCloud2, callback_pc2)
    loop = rospy.Rate(0.2)
    torch.cuda.empty_cache()
    pc_num = 0
    while not rospy.is_shutdown():
        rospy.sleep(1)
        pc_num = pc_num + 1
        np_pc = ros_pc2_to_nparray(pc2)
        pc_back, color_back, pc_torch = rm.nparray_pc_to_torch(np_pc)
        processed_pc_save_path = PROCESSED_PC_PATH + '/Processed_pc' + str(pc_num) + '.p'

        torch.cuda.empty_cache()
        grasp_stage2, select_grasp_class, select_grasp_score, select_grasp_class_stage2, output_score = refinedModule.process_pcd(pc_torch)
        refinedModule.save_processed_grasps(pc_back, color_back, grasp_stage2, select_grasp_class, select_grasp_score, select_grasp_class_stage2, output_score, processed_pc_save_path)
        
        print(pc2.fields)
        loop.sleep()

def ros_pc2_to_nparray(pc):
    data = ros_numpy.point_cloud2.pointcloud2_to_array(pc)
    data = data.reshape(-1)
    rgb = ros_numpy.point_cloud2.split_rgb_field(data)
    
    dt2 = np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('r', '<f4'), ('g', '<f4'), ('b', '<f4')])
    print(rgb)
    print(rgb.shape)
    print(rgb.dtype)
    print(type(rgb))
    
    rgb = np.asarray(rgb).astype(dt2)
    rgb['r'] = np.divide(rgb['r'], 255)
    rgb['g'] = np.divide(rgb['g'], 255)
    rgb['b'] = np.divide(rgb['b'], 255)
    
    rgb = rf.structured_to_unstructured(rgb)
    
    rgb = rgb[~np.isnan(rgb).any(axis=1)]
    
    print("Unstructured numpy array characteristics")
    print(rgb)
    print(rgb.shape)
    print(rgb.dtype)
    print(type(rgb))
    
    print("Element characteristics")
    print(rgb[0,0], type(rgb[0,0]))
    print(rgb[0,1], type(rgb[0,1]))
    print(rgb[0,2], type(rgb[0,2]))
    print(rgb[0,3], type(rgb[0,3]))
    print(rgb[0,4], type(rgb[0,4]))
    print(rgb[0,5], type(rgb[0,5]))
    
    return rgb

def ros_pc2_to_torch(pc):
    data = ros_numpy.point_cloud2.pointcloud2_to_array(pc)
    data = data.reshape(-1)
    rgb = ros_numpy.point_cloud2.split_rgb_field(data)
    dt2 = np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('r', '<f4'), ('g', '<f4'), ('b', '<f4')])
    print(rgb)
    print(rgb.shape)
    print(rgb.dtype)
    print(type(rgb))
    
    rgb = np.asarray(rgb).astype(dt2)
    rgb['r'] = np.divide(rgb['r'], 255)
    rgb['g'] = np.divide(rgb['g'], 255)
    rgb['b'] = np.divide(rgb['b'], 255)
    dt = rgb.dtype
    print(dt)
    print(type(dt))
    
    rgb = rf.structured_to_unstructured(rgb)
    print("Unstructured numpy array characteristics")
    print(rgb)
    print(rgb.shape)
    print(rgb.dtype)
    print(type(rgb))
    print("Element characteristics")
    print(rgb[0,0], type(rgb[0,0]))
    print(rgb[0,1], type(rgb[0,1]))
    print(rgb[0,2], type(rgb[0,2]))
    print(rgb[0,3], type(rgb[0,3]))
    print(rgb[0,4], type(rgb[0,4]))
    print(rgb[0,5], type(rgb[0,5]))
    
    pc_torch = torch.Tensor(rgb)
    #pc_torch = torch.Tensor(rgb).view(1, -1, 6)
    print("Converted tensor characteristics")
    print(pc_torch)
    print(pc_torch.shape)
    print(type(pc_torch))
    
    pc_torch = pc_torch[~torch.any(pc_torch.isnan(),dim=1)]
    
    pc_torch = pc_torch.cuda()
    print("Filtered tensor characteristics")
    print(pc_torch)
    print(pc_torch.shape)
    print(type(pc_torch))
    return pc_torch

def testfile():
    refinedModule = RegnetModule()
    test_file_path = REGNET_PATH + '/test_file/virtual_data'
    open3d_data = True if 'real_data' in test_file_path else False
    print(open3d_data)
    if open3d_data:
        pc_paths = glob.glob(test_file_path+"/*.pcd",recursive=True)
    else:
        pc_paths = glob.glob(test_file_path+"/*.p",recursive=True)
    for pc_path in pc_paths:
        refinedModule.test_file(refinedModule.resume_epoch-1, pc_path, open3d_data)

if __name__ == "__main__":
    main()
