#!/usr/bin/env python

import rospy
import numpy as np
from pathlib import Path
from threading import Lock
from manip_msgs.srv import DataCapture, DataCaptureResponse
from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String, Float64MultiArray
from gazebo_msgs.srv import GetModelState, SetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Point, Pose
from io import BytesIO
import tf2_ros
import tf2_geometry_msgs
import geometry_msgs.msg
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

def serialize_msg(msg):
    buff = BytesIO()
    msg.serialize(buff)
    return buff.getvalue()

def main():
    global pc2, hd, obj_pos, grasp_traj, obj_shape, status, trajectory_found, get_object_relative_pose
    obj_pose = Pose()
    obj_shape = '056_tennis_ball'
    print("Starting pose calculator node")
    rospy.init_node("pose_calc_node")
    #rospy.wait_for_service("gazebo/spawn_sdf_model")
    get_object_relative_pose = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    set_model_state_service = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    gripper_model_state = ModelState(model_name="justina_gripper",reference_frame=obj_shape)
    rgbr =  tf2_ros.StaticTransformBroadcaster()
    rgt = geometry_msgs.msg.TransformStamped() 
    loop = rospy.Rate(2)
    POSE_DATA_PATH = "./catkin_ws/src/manipulation/object_manipulation/pose_data/"
    pose_num = 0

    while not rospy.is_shutdown():
        print("Once the edited pose in gazebo is appropiate, type 'save' to get the relative pose of the gripper and the object")
        command = input()
        if command == "save": 
            #obj_pose = get_object_relative_pose("justina_gripper::left_arm_grip_center",obj_shape).pose
            try:
                transform = tfBuffer.lookup_transform('object_frame','gm/gr_left_arm_grip_center', rospy.Time())
                transform.child_frame_id = 'relative_grasp'
                #print(transform)
                rgbr.sendTransform(transform)
                gr_pose = get_object_relative_pose("justina_gripper","056_tennis_ball").pose
                broadcaster_frame_object('object_frame','grasp_spawn',gr_pose)
                gripper_model_state.pose = gr_pose
                print(gripper_model_state)

                pose_num = pose_num + 1
                file_name = POSE_DATA_PATH + obj_shape + str(pose_num)

                serialized_gripper_model_state = serialize_msg(gripper_model_state)
                print(serialized_gripper_model_state)
                                
                # Open file in binary write mode
                binary_file = open(file_name, "wb")
                binary_file.write(serialized_gripper_model_state)
                binary_file.close()

                in_file = open(file_name, "rb") # opening for [r]eading as [b]inary
                file_serialized_gripper_model_state = in_file.read() # if you only wanted to read 512 bytes, do .read(512)
                in_file.close()
                
                print(file_serialized_gripper_model_state)

                deserialized_gripper_model_state = ModelState()
                deserialized_gripper_model_state.deserialize(file_serialized_gripper_model_state)
                print(deserialized_gripper_model_state)

            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rate.sleep()
                continue
        loop.sleep()

if __name__ == '__main__':
    main()
