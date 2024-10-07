#!/usr/bin/env python

import rospy
import rospkg
import rosbag 
import numpy as np
from pathlib import Path
from threading import Lock
from manip_msgs.srv import DataCapture, DataCaptureResponse
from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String, Float64MultiArray
from gazebo_msgs.srv import GetModelState
from geometry_msgs.msg import Point

def semaphore_update(i):
    global sem, trajectory_found
    flag_mutex.acquire()
    sem[i] = True
    if all(sem):
        for x in sem:
            sem[x] = False
        print("Grasp conditions saved")
        trajectory_found = False
    flag_mutex.release()

def open_files():
    global ft_pc2, ft_hd, ft_obj_pos, ft_grasp_traj, ft_obj_shape, st_pc2, st_hd, st_obj_pos, st_grasp_traj, st_obj_shape, ft_data, st_data
    data_folder = Path(__file__).resolve().parent.parent / "training_data"
    ft = data_folder / "found_trajectory" 
    st = data_folder / "successful_trajectory" 
    ft_pc2 = ft / "ft_pc2.bag"
    ft_pc2 = rosbag.Bag(ft_pc2, 'w')
    ft_data = ft / "ft_data.csv"
    ft_data = open(ft_data, 'a')
    st_data = st / "st_data.csv"
    st_data = open(st_data, 'a')
    ft_grasp_traj = ft / "ft_grasp_traj.bag"
    ft_grasp_traj = rosbag.Bag(ft_grasp_traj, 'w')
    st_pc2 = st / "st_pc2.bag"
    st_pc2 = rosbag.Bag(st_pc2, 'w')
    st_grasp_traj = st / "st_grasp_traj.bag"
    st_grasp_traj = rosbag.Bag(st_grasp_traj, 'w')


def close_files():
    ft_pc2.close()
    ft_data.close()
    ft_grasp_traj.close()
    st_pc2.close()
    st_data.close()
    st_grasp_traj.close()

def obj_position_to_string(obj):
    pos = np.array([obj.x,obj.y,obj.z], dtype=np.float64)
    pos_str = np.array2string(pos,separator=' ')
    return pos_str

def save_grasp_data(successful):
    data_str = ""
    hd_pos_str = np.array2string(hd, separator=' ')
    obj_pos_str = obj_position_to_string(get_object_relative_pose("apple","justina").pose.position)
    data_str = hd_pos_str + ',' + obj_pos_str + ',' + obj_shape.data + '\n'
    if successful:
        print(data_str)
        st_data.write(data_str)
        st_pc2.write('Pointcloud',pc2)
        st_grasp_traj.write('JointTrajectory',grasp_traj)
    else:
        print(data_str)
        ft_data.write(data_str)
        ft_pc2.write('Pointcloud',pc2)
        ft_grasp_traj.write('JointTrajectory',grasp_traj)

def callback_pc2(msg):
    global pc2, trajectory_found
    if trajectory_found:
        pc2 = msg
        semaphore_update(0)

def callback_head_angles(msg):
    global hd, trajectory_found
    if trajectory_found:
        hd = np.asarray(msg.data)
        hd.astype(np.float64)
        semaphore_update(1)

def callback_grasp_traj(msg):
    global grasp_traj
    grasp_traj = msg
    #grasp_traj = np.asarray(msg.points[-1])

def callback_object_shape(msg):
    global obj_shape
    obj_shape = msg

def callback_capture(req):
    global trajectory_found
    if req.capture_info.data == "Found_grasp":
        trajectory_found = True
        status.data = "Grasp Trajectory Found"
    elif req.capture_info.data == "Save_grasp":
        save_grasp_data(False)
        status.data = "Data saved successfully"
    elif req.capture_info.data == "Save_successful_grasp":
        save_grasp_data(True)
        status.data = "Data saved successfully"
    else:
        status.data = "Failed to save data"
    return DataCaptureResponse(status)    
    

def main():
    global pc2, hd, obj_pos, grasp_traj, obj_shape, status, trajectory_found, sem, flag_mutex, get_object_relative_pose
    trajectory_found = False
    pc2 = PointCloud2()
    hd = np.array([0.0,0.0])
    hd.astype(np.float64)
    sem = [False] * 2
    flag_mutex = Lock()
    obj_pos= Point() 
    grasp_traj = JointTrajectory()
    obj_shape = String()
    status = String(data="Data saved successfully")
    print("Starting training data capture node")
    rospy.init_node("capture_node")
    rospy.Service('/manipulation/grasp/capture' ,DataCapture ,callback_capture)
    get_object_relative_pose = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    rospy.Subscriber("/camera/depth_registered/points"   , PointCloud2, callback_pc2)
    rospy.Subscriber("/hardware/head/current_pose"   , Float64MultiArray, callback_head_angles)
    rospy.Subscriber("/manipulation/la_q_trajectory", JointTrajectory, callback_grasp_traj)
    rospy.Subscriber('/manipulation/grasp/object_shape' ,String ,callback_object_shape)
    open_files()   
    loop = rospy.Rate(2)
    while not rospy.is_shutdown():
        loop.sleep()
    rospy.on_shutdown(close_files)

if __name__ == '__main__':
    main()
