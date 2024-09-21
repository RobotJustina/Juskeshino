#!/usr/bin/env python

import rospy
import rospkg
from manip_msgs.srv import DataCapture
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String

def callback_pc2(msg):
    global pc2
    pc2 = msg

def callback_capture(msg):
    print("siu")
    return "siu"

def main():
    global data, pc2
    data = "pos x resp just, pos hd, pc2, grasp, mov"
    print("Starting training data capture node")
    rospy.init_node("capture_node")
    rospy.Service('/manipulation/grasp/capture' ,DataCapture ,callback_capture)
    rospy.Subscriber("/camera/depth_registered/points"   , PointCloud2, callback_pc2)   
    loop = rospy.Rate(20)
    while not rospy.is_shutdown():
        loop.sleep()

if __name__ == '__main__':
    main()
