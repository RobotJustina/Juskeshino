#!/usr/bin/env python

import rospy
import numpy as np
import tf
import tf2_ros
from tf2_geometry_msgs import PointStamped
from pathlib import Path
from threading import Lock
from manip_msgs.srv import DataCapture, DataCaptureResponse
from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String, Float64MultiArray
from gazebo_msgs.srv import GetModelState
import geometry_msgs
from geometry_msgs.msg import Point, PoseStamped

REAL_GRASP_ATTEMPT = False

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

def callback_capture(req):
    global obj_shape, get_object_relative_pose, tf_listener,tf_buf
    og_pose = PoseStamped()
    og_pose.header.frame_id = "gm/gr_left_arm_grip_center"
    og_pose.pose.orientation.w = 1
    #listener.waitForTransform("gr_left_arm_grip_center","shoulders_left_link",rospy.Time(),rospy.Duration(2))
    #grip_center_pose = listener.transformPose("shoulders_left_link",)
    target_pt = tf_buf.transform(og_pose, "shoulders_left_link")
    print(target_pt)
    broadcaster_frame_object("shoulders_left_link","saved_gripper_center",target_pt.pose)
    hd               = rospy.wait_for_message("/hardware/head/current_pose", Float64MultiArray)
    #trajectory_found = rospy.wait_for_message("/manipulation/la_q_trajectory" , JointTrajectory)
    #articular_array          = trajectory_found.points[-1].positions
    resp                   = DataCaptureResponse()
    resp.capture_status    = "Saved_grasp"
    resp.pointcloud        = rospy.wait_for_message("/camera/depth_registered/points", PointCloud2)
    resp.obj_relative_pose = get_object_relative_pose(obj_shape,"justina::camera_link").pose
    resp.head_pose_q         = hd.data
    resp.final_grasp_q     = [0,1,0]
    resp.obj_type          = obj_shape
    #resp.arm               = False
    return resp


    

def main():
    global pc2, hd, obj_pos, grasp_traj, obj_shape, status, listener,tf_listener,tf_buf, get_object_relative_pose
    pc2 = PointCloud2()
    hd = []
    obj_pos= Point() 
    grasp_traj = JointTrajectory()
    obj_shape = rospy.get_param("/obj","056_tennis_ball")
    status = "Data saved successfully"
    #listener = tf.TransformListener()
    print("Starting training data capture node")
    get_object_relative_pose = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    rospy.init_node("data_capture_node")
    tf_buf = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buf)
    rospy.Service('/manipulation/grasp/data_capture_service' ,DataCapture ,callback_capture)
    loop = rospy.Rate(2)
    while not rospy.is_shutdown():
        loop.sleep()

if __name__ == '__main__':
    main()
