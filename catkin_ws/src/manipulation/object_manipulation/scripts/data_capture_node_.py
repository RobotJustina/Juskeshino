#!/usr/bin/env python

import rospy
import numpy as np
import tf
import tf2_ros
from tf2_geometry_msgs import PointStamped
from pathlib import Path
from threading import Lock
from manip_msgs.srv import DataCapture, DataCaptureResponse, InverseKinematicsPose2TrajRequest, InverseKinematicsPose2Traj
from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String, Float64MultiArray
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState, SetModelState
import geometry_msgs
from geometry_msgs.msg import Point, Pose, PoseStamped
import tf.transformations as tft

REAL_GRASP_ATTEMPT = False

GRIPPER_ORIGIN = ModelState()
GRIPPER_ORIGIN.model_name = "justina_gripper"
GRIPPER_ORIGIN.pose = Pose()
GRIPPER_ORIGIN.pose.orientation.w = 1
GRIPPER_ORIGIN.reference_frame = "world"


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
    global obj_shape, get_object_relative_pose, tf_listener,tf_buf, set_state
    og_pose = PoseStamped()
    og_pose.header.frame_id = "gm/gr_left_arm_grip_center"
    og_pose.pose.orientation.w = 1
    target_pt = tf_buf.transform(og_pose, "camera_rgb_optical_frame")
    #broadcaster_frame_object("camera_rgb_optical_frame","saved_gripper_center",target_pt.pose)
    hd               = rospy.wait_for_message("/hardware/head/current_pose", Float64MultiArray)
    x = target_pt.pose.position.x
    y = target_pt.pose.position.y
    z = target_pt.pose.position.z
    qx = target_pt.pose.orientation.x
    qy = target_pt.pose.orientation.y
    qz = target_pt.pose.orientation.z
    qw = target_pt.pose.orientation.w
    # roll,pitch,yaw = tft.euler_from_quaternion( [target_pt.pose.orientation.x , target_pt.pose.orientation.y , 
    #                                              target_pt.pose.orientation.z , target_pt.pose.orientation.w ])
    resp                   = DataCaptureResponse()
    resp.capture_status    = "Saved_grasp"
    resp.obj_relative_pos  = get_object_relative_pose(obj_shape,"justina::camera_link").pose.position
    resp.head_pose_q       = hd.data
    resp.final_grasp_q     = [x, y, z, qx, qy, qz, qw]
    resp.obj_type          = obj_shape
    resp.gripper_pose      = get_object_relative_pose("justina_gripper","justina::camera_link").pose
    set_state(GRIPPER_ORIGIN)
    rospy.sleep(0.15)
    resp.pointcloud        = rospy.wait_for_message("/camera/depth_registered/points", PointCloud2)
    resp.score             = 0
    #score_calculation(target_pt.pose)
    return resp


def score_calculation(msg_pose):
    ik = get_ik_la(msg_pose)
    print(ik)
    if ik is not None: 
        return sum(ik.positions)
    else: 
        return 0    


def get_ik_la(msg_pose):
    global ik_srv

    roll,pitch,yaw = tft.euler_from_quaternion( [msg_pose.orientation.x , msg_pose.orientation.y , 
                                                 msg_pose.orientation.z , msg_pose.orientation.w ])

    ik_msg = InverseKinematicsPose2TrajRequest()
    ik_msg.x         = msg_pose.position.x
    ik_msg.y         = msg_pose.position.y
    ik_msg.z         = msg_pose.position.z
    ik_msg.roll      = roll    
    ik_msg.pitch     = pitch
    ik_msg.yaw       = yaw
    ik_msg.duration  = 0
    ik_msg.time_step = 0.05
    try:
        resp_ik_srv = ik_srv(ik_msg)    # Envia al servicio de IK
        print("Approved pose")
        articular_array = resp_ik_srv.articular_trajectory.points[-1]
        return articular_array
    except:
        print("It was not possible to obtain the ik")


    

def main():
    global pc2, hd, obj_pos, grasp_traj, obj_shape, status,tf_listener,tf_buf, get_object_relative_pose, ik_srv, set_state
    pc2 = PointCloud2()
    hd = []
    obj_pos= Point() 
    grasp_traj = JointTrajectory()
    obj_shape = rospy.get_param("/obj","056_tennis_ball")
    status = "Data saved successfully"
    #listener = tf.TransformListener()
    print("Starting training data capture node")
    get_object_relative_pose = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    ik_srv = rospy.ServiceProxy('/manipulation/la_ik_trajectory', InverseKinematicsPose2Traj)
    rospy.init_node("data_capture_node")
    tf_buf = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buf)
    rospy.Service('/manipulation/grasp/data_capture_service' ,DataCapture ,callback_capture)
    loop = rospy.Rate(2)
    while not rospy.is_shutdown():
        loop.sleep()

if __name__ == '__main__':
    main()
