#!/usr/bin/env python

import rospy
import numpy as np
from pathlib import Path
from threading import Lock
from manip_msgs.srv import DataCapture, DataCaptureResponse
from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String, Float64MultiArray
from gazebo_msgs.srv import GetModelState
from geometry_msgs.msg import Point





def callback_capture(req):

    hd               = rospy.wait_for_message("/hardware/head/current_pose", Float64MultiArray)
    trajectory_found = rospy.wait_for_message("/manipulation/la_q_trajectory" , JointTrajectory)
    articular_array          = trajectory_found.points[-1].positions
    resp                   = DataCaptureResponse()
    resp.capture_status    = "Saved_grasp"
    resp.pointcloud       = rospy.wait_for_message("/camera/depth_registered/points", PointCloud2)
    resp.obj_relative_pose = get_object_relative_pose("apple","justina").pose
    resp.head_pose_q         = hd.data
    resp.final_grasp_q     = articular_array
    resp.obj_type          = "apple"
    
    return resp


    

def main():
    global pc2, hd, obj_pos, grasp_traj, obj_shape, status, trajectory_found, sem, flag_mutex, get_object_relative_pose
    trajectory_found = False
    pc2 = PointCloud2()
    #hd = np.array([0.0,0.0])
    #hd.astype(np.float64)
    hd = Float64MultiArray()
    sem = [False] * 2
    flag_mutex = Lock()
    obj_pos= Point() 
    grasp_traj = JointTrajectory()
    obj_shape = String()
    status = "Data saved successfully"
    print("Starting training data capture node")
    rospy.init_node("data_capture_node")
    rospy.Service('/manipulation/grasp/data_capture_service' ,DataCapture ,callback_capture)
    get_object_relative_pose = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState) 
    
     
    loop = rospy.Rate(2)
    while not rospy.is_shutdown():
        loop.sleep()

if __name__ == '__main__':
    main()
