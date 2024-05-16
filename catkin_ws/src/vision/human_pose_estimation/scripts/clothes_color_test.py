#!/usr/bin/env python3

import rospy
import numpy as np
import tf.transformations as tft
from sensor_msgs.msg import PointCloud2, Image
from geometry_msgs.msg import PointStamped, PoseStamped, Point, Pose
from vision_msgs.srv import *
from manip_msgs.srv import *
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray


def main():
    print("Test node color of clothes.........ʕ•ᴥ•ʔ")
    rospy.init_node("clothes_color_test")

    rospy.wait_for_service("/vision/human_pose_estimation/pointing_hands_status")
    fp_srv = rospy.ServiceProxy("/vision/human_pose_estimation/pointing_hands_status", FindPerson)

    #mensaje*****************************************************************
    fp_msg = FindPersonRequest()
    #pc = rospy.wait_for_message("/hardware/realsense/points", PointCloud2)
    pc = rospy.wait_for_message("/camera/depth_registered/points", PointCloud2)
    fp_msg.cloud = pc

    resp = fp_srv(fp_msg)

    print("*******",resp)

  
    loop = rospy.Rate(10)
    while not rospy.is_shutdown():
        #pc = rospy.wait_for_message("/hardware/realsense/points", PointCloud2)
        pc = rospy.wait_for_message("/hsrb/head_rgbd_sensor/depth_registered/rectified_points", PointCloud2)
        fp_msg.cloud = pc

        resp = fp_srv(fp_msg)
        print("*****lolo**",resp)

        loop.sleep()

if __name__ == '__main__':
    main()