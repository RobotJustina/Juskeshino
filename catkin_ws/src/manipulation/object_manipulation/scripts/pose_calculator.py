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
from geometry_msgs.msg import Point, Pose
import tf.transformations as tft
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

def main():
    global pc2, hd, obj_pos, grasp_traj, obj_shape, status, trajectory_found, get_object_relative_pose
    obj_pose = Pose()
    obj_shape = 'apple'
    print("Starting pose calculator node")
    rospy.init_node("pose_calc_node")
    get_object_relative_pose = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
     
    loop = rospy.Rate(2)
    while not rospy.is_shutdown():
        print("Once the edited pose in gazebo is appropiate, type 'save' to get the relative pose of the gripper and the object")
        command = input()
        if command == "save": 
            obj_pose = get_object_relative_pose("justina_gripper::left_arm_grip_center",obj_shape).pose
            obj_pose.position.z
            print(obj_pose)
        loop.sleep()

if __name__ == '__main__':
    main()
