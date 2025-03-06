#!/usr/bin/env python
import rospy 
import pickle
import ros_numpy
import numpy as np
import math
import os
import open3d
import copy
import numpy.lib.recfunctions as rf
import torch
import cv2
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Point
from gazebo_msgs.srv import GetModelState
from visualization_msgs.msg import Marker
from scipy.spatial import cKDTree

MAX_POINTS = 25600
DATASET_PATH = 'catkin_ws/src/graspnet/dataset/'
gpus = 1

def debug_type(obj, obj_name):
    print(obj_name + "Object characteristics:")
    print(obj)
    print(obj.shape)
    print(obj.dtype)
    print(type(obj))

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

def ros_pc2_to_npmatrix(pc):
    print(pc.header.frame_id)
    data = ros_numpy.point_cloud2.pointcloud2_to_array(pc)
    #debug_type(data,"Split points")
    rgb = ros_numpy.point_cloud2.split_rgb_field(data)
    #debug_type(rgb,"Split points + RGB")
    dt2 = np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('r', '<f4'), ('g', '<f4'), ('b', '<f4')])
    rgb = np.asarray(rgb).astype(dt2)
    rgb['r'] = np.divide(rgb['r'], 255)
    rgb['g'] = np.divide(rgb['g'], 255)
    rgb['b'] = np.divide(rgb['b'], 255)
    #debug_type(rgb, "Split points + Normalized RGB")
    return rgb

def save_to_file(pcd, grasp, gr_pose, obj_relative_pos, head_pose_q, obj_type, score, file_path):
    output_dict = {
        #'points'             : pc, #np array of XYZ points of pointcloud w.r.p to camera_link
        #'colors'             : color, #np array of RGB values of pointcloud in [0,1] range (necessary for open3d and tensor)
        'pcd'                : pcd,
        'grasp'              : grasp, #[x, y, z, roll, pitch, yaw, w] array of final grip center pose w.r.p to camera_link
        'gripper_origin'     : gr_pose, #Pose of gr_left_arm_link7 w.r.p to camera_link
        'obj_relative_pos'   : obj_relative_pos, #Object relative position w.r.p to camera_link
        'head_pose_q'        : head_pose_q, #Array of head angles at capture time
        'obj_type'           : obj_type, #String of object type
        'scores'             : score, #Assigned score for grasp
    }
    print(file_path)
    if file_path:
        with open(file_path, 'wb') as file:
            pickle.dump(output_dict, file)

def show_pcd_from_file(path):
    data = np.load(os.path.abspath(path), allow_pickle=True)
    pcd = data['pcd']
    pcd = pcd.reshape(-1)
    pcd = rf.structured_to_unstructured(pcd)
    pcd = pcd[~np.isnan(pcd).any(axis=1)]
    #view = data['points']
    #color = data['colors']
    grip = data['grasp']
    score = data['scores']
    obj_pos = data['obj_relative_pos']
    gr_pose = data['gripper_origin']
    print(grip)
    print(score)
    print(obj_pos)
    print(gr_pose)
    view_point_cloud = open3d.geometry.PointCloud()
    view_point_cloud.points = open3d.utility.Vector3dVector(copy.deepcopy(pcd[:,:3]))
    view_point_cloud.colors = open3d.utility.Vector3dVector(copy.deepcopy(pcd[:,3:6]))
    open3d.visualization.draw_geometries([view_point_cloud])

def show_pcd_from_npmatrix(mat):
    rgb = copy.deepcopy(mat)
    rgb = rgb.reshape(-1)
    rgb = rf.structured_to_unstructured(rgb)
    rgb = rgb[~np.isnan(rgb).any(axis=1)]
    view_point_cloud = open3d.geometry.PointCloud()
    debug_type(rgb, "Reshaped pcd ")
    view_point_cloud.points = open3d.utility.Vector3dVector(copy.deepcopy(rgb[:,:3]))
    view_point_cloud.colors = open3d.utility.Vector3dVector(copy.deepcopy(rgb[:,3:6]))
    open3d.visualization.draw_geometries([view_point_cloud])

def nparray_pc_to_torch(pc):
    pc_back, color_back = copy.deepcopy(pc[:,:3]), copy.deepcopy(pc[:,3:6])
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
    #pc = ros_pc2_to_nparray(pcd)
    #pc, color, _ = nparray_pc_to_torch(pc)
    pcd = ros_pc2_to_npmatrix(pcd)
    grasp = resp.final_grasp_q
    gr_pose = resp.gripper_pose
    obj_relative_pos = resp.obj_relative_pos
    obj_relative_pos = camera_link_to_optical_frame(obj_relative_pos)
    head_pose_q = resp.head_pose_q
    obj_type = resp.obj_type
    score = resp.score
    u, v, obj_in_range = find_nearest_pt_in_pc(pcd, obj_relative_pos)
    if obj_in_range:
        file_path = DATASET_PATH + "example_" + str(file_num) + ".p"

        cut_pcd = cut_pc(u,v,pcd)
        #show_rgb(cut_pc)

        save_to_file(cut_pcd, grasp, gr_pose, obj_relative_pos, head_pose_q, obj_type, score, file_path)
        return 1
    else:
        return 0

def camera_link_to_optical_frame(pt):
    target_pt = Point()
    target_pt.x = -pt.y
    target_pt.y = -pt.z
    target_pt.z = pt.x
    return target_pt

def create_marker_from_pt(pt):
    global marker_pub
    marker = Marker()
    marker.header.frame_id = "camera_rgb_optical_frame"
    marker.type = Marker.SPHERE
    marker.ns = "obje"
    marker.header.stamp = rospy.Time.now()
    marker.action = marker.ADD
    marker.id = 1
    marker.scale.x, marker.scale.y, marker.scale.z = 0.02, 0.2, 0.2
    marker.color.r, marker.color.g, marker.color.b, marker.color.a = 20, 50, 100, 1.0
    marker.lifetime = rospy.Duration(10)
    marker.pose.position = pt
    marker.pose.orientation.w = 1
    marker_pub.publish(marker)

def find_nearest_pt_in_pc(pc, pt):
    valid = False
    matrix = copy.deepcopy(pc)
    matrix = matrix.reshape(-1)
    matrix = rf.structured_to_unstructured(matrix)
    search_vec = np.array([pt.x, pt.y, pt.z])
    nearest = cKDTree(matrix[:,:3]).query(search_vec, k=1)[1]
    u = math.floor(nearest/pc.shape[1])
    v = nearest%pc.shape[1]
    if 100 < u < 380 and 100 < v < 540: valid = True
    print(u,v,valid)
    return u,v,valid

def find_pt_in_pc(position_obj, pc):
    min_dist = 100000
    u, v = 0, 0
    for i in range(len(pc)):
        for j in range(len(pc[0])):
            dist = np.linalg.norm(np.array([pc[i,j]['x'] - position_obj.x, pc[i,j]['y'] - position_obj.y, pc[i,j]['z'] - position_obj.z]))
            if dist < min_dist:
                min_dist = dist
                u = i
                v = j
    return u, v
                

def cut_pc(u,v, pc):
    l, w = 200, 200
    cropped_pc = pc[(u - int(l/2)): (u + int(l/2)) , (v - int(l/2)) : (v + int(l/2))]
    return cropped_pc


def show_rgb(pc):
    img_xyz = ros_numpy.point_cloud2.pointcloud2_to_array(pc)  # dim 480 x 640, 
    rgb_array = img_xyz['rgb'].copy()     # Pass a copy of rgb float32, 480 x 640
    rgb_array.dtype = np.uint32       # Config data type of elements from array
    r,g,b = ((rgb_array >> 16) & 255), ((rgb_array >> 8) & 255), (rgb_array & 255)  # 480 x 640 c/u
    img_bgr = cv2.merge((np.asarray(b,dtype='uint8'),np.asarray(g,dtype='uint8'),np.asarray(r,dtype='uint8')))
    cv2.imshow("imagen sin plano #1", img_bgr) #*****************
    cv2.waitKey(0)


def main():
    global marker_pub
    print("Dataset utils started")
    obj_shape = rospy.get_param("/obj","056_tennis_ball")
    get_object_relative_pose = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    marker_pub = rospy.Publisher("/vision/object_recognition/markers", Marker, queue_size = 10)
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
        if command == 'p':
            pcd = rospy.wait_for_message("/camera/depth_registered/points", PointCloud2)
            mat = ros_pc2_to_npmatrix(pcd)
            show_pcd_from_npmatrix(mat)
        if command == 'c':
            pcd = rospy.wait_for_message("/camera/depth_registered/points", PointCloud2)
            obj_pt = get_object_relative_pose(obj_shape,"justina::camera_link").pose.position
            obj_pt = camera_link_to_optical_frame(obj_pt)
            create_marker_from_pt(obj_pt)
        if command == 'r':
            pcd = rospy.wait_for_message("/camera/depth_registered/points", PointCloud2)
            obj_pt = get_object_relative_pose("justina_gripper","justina::camera_link").pose.position
            mat = ros_pc2_to_npmatrix(pcd)
            t_pt = camera_link_to_optical_frame(obj_pt)
            u, v = find_nearest_pt_in_pc(mat,t_pt)
            found_pt = Point(x=mat[u,v]['x'], y=mat[u,v]['y'], z=mat[u,v]['z'])
            create_marker_from_pt(found_pt)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass