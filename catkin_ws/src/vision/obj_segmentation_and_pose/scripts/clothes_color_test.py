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

    rospy.wait_for_service("/vision/obj_segmentation_and_pose/clothes_color")
    fp_srv = rospy.ServiceProxy("/vision/obj_segmentation_and_pose/clothes_color", FindPerson)

    #mensaje*****************************************************************
    fp_msg = FindPersonRequest()
    fp_msg.cloud = rospy.wait_for_message("/camera/depth_registered/points", PointCloud2, timeout=5.0)

    resp = fp_srv(fp_msg)

    print("*******",resp)

  
    loop = rospy.Rate(10)
    while not rospy.is_shutdown():
        pc = rospy.wait_for_message("/camera/depth_registered/points", PointCloud2, timeout=5.0)
        fp_msg.cloud = pc

        resp = fp_srv(fp_msg)
        print("*****lolo**",resp)

        loop.sleep()

if __name__ == '__main__':
    main()