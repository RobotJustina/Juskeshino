#!/home/robocup/regnet/REGNet_for_3D_Grasping/env/bin/python

import rospy 
import pickle
import ros_numpy
import numpy as np
import os
import open3d
import copy
import numpy.lib.recfunctions as rf
import torch
from sensor_msgs.msg import PointCloud2

MAX_POINTS = 25600
DATASET_PATH = 'catkin_ws/src/graspnet/dataset/'
gpus = 1

def ros_pc2_to_nparray(pc):
    data = ros_numpy.point_cloud2.pointcloud2_to_array(pc)
    data = data.reshape(-1)
    rgb = ros_numpy.point_cloud2.split_rgb_field(data)
    
    dt2 = np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('r', '<f4'), ('g', '<f4'), ('b', '<f4')])
    
    rgb = np.asarray(rgb).astype(dt2)
    rgb['r'] = np.divide(rgb['r'], 255)
    rgb['g'] = np.divide(rgb['g'], 255)
    rgb['b'] = np.divide(rgb['b'], 255)
    
    rgb = rf.structured_to_unstructured(rgb)
    rgb = rgb[~np.isnan(rgb).any(axis=1)]

    return rgb

def save_to_file(pc, color, grasp, gr_pose, obj_relative_pos, head_pose_q, obj_type, score, file_path):
    output_dict = {
        'points'             : pc, #np array of XYZ points of pointcloud w.r.p to camera_link
        'colors'             : color, #np array of RGB values of pointcloud in [0,1] range (necessary for open3d and tensor)
        'grasp'              : grasp, #[x, y, z, roll, pitch, yaw, w] array of final grip center pose w.r.p to camera_link
        'gripper_origin'     : gr_pose, #Pose of gr_left_arm_link7 w.r.p to camera_link
        'obj_relative_pos'   : obj_relative_pos, #Object relative position w.r.p to camera_link
        'head_pose_q'        : head_pose_q, #Array of head angles at capture time
        #'final_grasp_q'      : final_grasp_q,
        'obj_type'           : obj_type, #String of object type
        'scores'             : score, #Assigned score for grasp
    }
    print(file_path)
    if file_path:
        with open(file_path, 'wb') as file:
            pickle.dump(output_dict, file)

def show_pcd_from_file(path):
    data = np.load(os.path.abspath(path), allow_pickle=True)
    view = data['points']
    color = data['colors']
    grip = data['grasp']
    score = data['scores']
    obj_pos = data['obj_relative_pos']
    gr_pose = data['gripper_origin']
    print(grip)
    print(score)
    print(obj_pos)
    print(gr_pose)
    view_point_cloud = open3d.geometry.PointCloud()
    view_point_cloud.points = open3d.utility.Vector3dVector(view)
    view_point_cloud.colors = open3d.utility.Vector3dVector(color)
    open3d.visualization.draw_geometries([view_point_cloud])

def nparray_pc_to_torch(pc):
    pc_back, color_back = copy.deepcopy(pc[:,:3]), copy.deepcopy(pc[:,3:6])
    #pc = utils.noise_color(pc)
    select_point_index = None
    if len(pc) >= MAX_POINTS:
        select_point_index = np.random.choice(len(pc), MAX_POINTS, replace=False)
    elif len(pc) < MAX_POINTS:
        select_point_index = np.random.choice(len(pc), MAX_POINTS, replace=True)
    pc = pc[select_point_index]
    pc_torch = torch.Tensor(pc).view(1, -1, 6)
    if gpus != -1:
        pc_torch = pc_torch.cuda()
    print("Converted tensor characteristics")
    print(pc_torch)
    print(pc_torch.shape)
    print(type(pc_torch))
    return pc_back, color_back, pc_torch

def save_data_to_file(resp, file_num=1):
    pcd = resp.pointcloud
    pc = ros_pc2_to_nparray(pcd)
    pc, color, _ = nparray_pc_to_torch(pc)
    grasp = resp.final_grasp_q 
    gr_pose = resp.gripper_pose
    obj_relative_pos = resp.obj_relative_pos
    head_pose_q = resp.head_pose_q
    obj_type = resp.obj_type
    score = resp.score
    file_path = DATASET_PATH + "example_" + str(file_num) + ".p"
    save_to_file(pc, color, grasp, gr_pose, obj_relative_pos, head_pose_q, obj_type, score, file_path)

def main():
    print("Dataset utils started")
    rospy.init_node('dataset_utils')
    loop = rospy.Rate(1)
    while not rospy.is_shutdown():
        print("Choose command")
        command = input()
        if command == 'y':
            pcd = rospy.wait_for_message("/camera/depth_registered/points", PointCloud2)
            pc = ros_pc2_to_nparray(pcd)
            pc, color, _ = nparray_pc_to_torch(pc)
            grasp = [1, 1, 1, 1, 1, 1, 1]
            gr_pose = "siu"
            obj_relative_pos ="Object pos"
            head_pose_q = [0, 0]
            obj_type = "apple"
            score = 10
            file_path = DATASET_PATH + "test1.p"
            save_to_file(pc, color, grasp, gr_pose, obj_relative_pos, head_pose_q, obj_type, score, file_path)
        if command == 's':
            print("Show number")
            num = input()
            show_pcd_from_file(DATASET_PATH + "example_" + num + ".p")





if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass