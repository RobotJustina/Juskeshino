#!/usr/bin/env python3

import rospy 
import rospkg 
import random
import math
import os
import matplotlib.pyplot as plt
import tf.transformations as tft
import numpy as np
import vg
import tf2_ros
from tf2_geometry_msgs import PointStamped
from vision_msgs.srv import PreprocessPointCloud, PreprocessPointCloudRequest
from gazebo_msgs.msg import ModelState, ContactsState
from gazebo_msgs.srv import SetModelState, GetModelState
from std_msgs.msg import String, Float64MultiArray
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Pose, PointStamped, Quaternion  
from train import load_model, DEVICE
import geometry_msgs
import grasp_network as gn

#from ...manipulation.object_manipulation.scripts import dataset_utils as dutils
#import ...manipulation.object_manipulation.scripts.dataset_utils as dtutils
import torch

MODELS_PATH = "/home/robocup/Juskeshino/catkin_ws/src/graspnet/models/"

def broadcaster_frame_object(frame, child_frame, pose):   # Emite la transformacion en el frame base_link,
    #br = tf2_ros.TransformBroadcaster()
    br =  tf2_ros.StaticTransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()
    t.header.frame_id = frame
    t.child_frame_id = child_frame 
    t.header.stamp = rospy.Time.now()
    t.transform.translation.x = pose.position.x
    t.transform.translation.y = pose.position.y
    t.transform.translation.z = pose.position.z
    t.transform.rotation.x = pose.orientation.x
    t.transform.rotation.y = pose.orientation.y
    t.transform.rotation.z = pose.orientation.z
    t.transform.rotation.w = pose.orientation.w
    br.sendTransform(t)

def generate_random_pose():
    rpose = Pose()
    rpose.position.x = random.randint(210,310)/100
    rpose.position.y = random.randint(218,245)/100
    rpose.position.z = 0.745
    rpose.orientation.x = random.randint(-315,315)/100
    rpose.orientation.y = random.randint(-315,315)/100
    rpose.orientation.z = random.randint(-315,315)/100
    rpose.orientation.w = 0
    return rpose

def normalize(v):
    norm = np.linalg.norm(v)
    if norm == 0: 
       return v
    return v / norm

def tensor_to_pose(tensor):
    nppose = tensor.detach().cpu().numpy()
    nppose = nppose[0]
    quat = normalize(nppose[3:])
    rpose = Pose()
    rpose.position.x = nppose[0]
    rpose.position.y = nppose[1]
    rpose.position.z = nppose[2]
    rpose.orientation.x = quat[0]
    rpose.orientation.y = quat[1]
    rpose.orientation.z = quat[2]
    rpose.orientation.w = quat[3]
    return rpose

def change_gazebo_object_pose(state_msg, state_pose, mod_name):
    global set_state
    state_msg.model_name = mod_name
    state_msg.pose = state_pose
    #rospy.wait_for_service('/gazebo/set_model_state')
    try:
        #set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        resp = set_state( state_msg )

    except rospy.ServiceException:
        pass      

def create_origin_pose():
    jop = Pose()
    jop.position.x = 2.6
    jop.position.y = 1.8
    jop.position.z = 0.06
    jop.orientation.x = 0
    jop.orientation.y = 0
    jop.orientation.z = 0.7068252
    jop.orientation.w = 0.7073883
    return jop
    
def reset_simulation():
    global justina_origin_pose, obj_shape, msg_la, pub_la, pub_hd, msg_hd, pub_object, num_loops, left_gripper_made_contact, right_gripper_made_contact, grasp_attempts
    change_gazebo_object_pose(state_msg, generate_random_pose(), obj_shape)
    change_gazebo_object_pose(state_msg, justina_origin_pose, "justina")
    num_loops = 0
    left_gripper_made_contact = False
    right_gripper_made_contact = False
    #pub_la.publish(msg_la)
    pub_hd.publish(msg_hd)
    rospy.sleep(0.1)
    #pub_object.publish(obj_shape)
    #pub_object.publish(obj_shape)
    grasp_attempts = 0



def main():
    global ik_srv, state_msg, grasp_trajectory_found, justina_origin_pose, obj_shape, left_gripper_made_contact, right_gripper_made_contact, grasp_attempts, msg_la, pub_la, pub_hd, msg_hd, pub_object, num_loops
    global set_state
    state_msg = ModelState()
    deserialized_gripper_model_state = ModelState()
    justina_origin_pose = create_origin_pose()
    msg_la = Float64MultiArray()
    msg_hd = Float64MultiArray()
    msg_la.data = [-1.0, 0.2, 0.0, 1.55, 0.0, 1.24, 0.0]
    msg_hd.data = [0,-1.3]
    obj_shape = '056_tennis_ball'
    rospy.init_node('network_tester')
    print("Starting grip test")
    get_object_relative_pose = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    pub_la = rospy.Publisher("/hardware/left_arm/goal_pose", Float64MultiArray, queue_size=10)
    pub_hd = rospy.Publisher("/hardware/head/goal_pose", Float64MultiArray, queue_size=10)
    transform_pointcloud = rospy.ServiceProxy("/vision/point_cloud_to_base_link",PreprocessPointCloud)
    obj_shape = rospy.get_param("/obj","056_tennis_ball")
    rospy.sleep(1)
    loop = rospy.Rate(1)
    grasp_network = load_model(MODELS_PATH + "model_gelu_adam_bl.pt")
    grasp_network.eval()
    while not rospy.is_shutdown():
        print("Type r to reset sim to a random pose, and l to loop simulation for samples")
        command = input()
        if command == "r": 
            reset_simulation()
            pcd = rospy.wait_for_message("/camera/depth_registered/points", PointCloud2)
            #pcd = transform_pointcloud(PreprocessPointCloudRequest(pcd)).output_cloud
            obj_pt = get_object_relative_pose(obj_shape,"justina::camera_link").pose.position
            mat = gn.ros_pc2_to_npmatrix(pcd)
            #t_pt = obj_pt
            t_pt = gn.camera_link_to_optical_frame(obj_pt)
            u, v, object_in_range = gn.find_nearest_pt_in_pc(mat,t_pt)
            if object_in_range:
                pcd = gn.cut_pc(u,v,mat)
                pcd = gn.npmatrix_to_torch(pcd)
                pcd = pcd.unsqueeze(0).to(DEVICE)
                print(pcd)
                with torch.no_grad():
                    predicted_gripper_center_pose = grasp_network(pcd)
                print(predicted_gripper_center_pose)
                predicted_pose = tensor_to_pose(predicted_gripper_center_pose)
                #predicted_pose.orientation.normalize()
                broadcaster_frame_object("camera_rgb_optical_frame","grasp_frame",predicted_pose)
                #broadcaster_frame_object("base_link","grasp_frame",predicted_pose)
                print(predicted_pose)
            #print("sure")
        if command == "rb":
            reset_simulation()
            pcd = rospy.wait_for_message("/camera/depth_registered/points", PointCloud2)
            pcd = transform_pointcloud(PreprocessPointCloudRequest(pcd)).output_cloud
            obj_pt = get_object_relative_pose(obj_shape,"justina::base_link").pose.position
            mat = gn.ros_pc2_to_npmatrix(pcd)
            t_pt = obj_pt
            #t_pt = gn.c(obj_pt)
            u, v, object_in_range = gn.find_nearest_pt_in_pc(mat,t_pt)
            if object_in_range:
                pcd = gn.cut_pc(u,v,mat)
                pcd = gn.npmatrix_to_torch(pcd)
                pcd = pcd.unsqueeze(0).to(DEVICE)
                print(pcd)
                with torch.no_grad():
                    predicted_gripper_center_pose = grasp_network(pcd)
                print(predicted_gripper_center_pose)
                predicted_pose = tensor_to_pose(predicted_gripper_center_pose)
                #predicted_pose.orientation.normalize()
                #broadcaster_frame_object("camera_rgb_optical_frame","grasp_frame",predicted_pose)
                broadcaster_frame_object("base_link","grasp_frame",predicted_pose)
                print(predicted_pose)

        loop.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass