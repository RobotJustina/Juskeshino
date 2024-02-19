# -*- coding: utf-8 -*-
"""
Created on Sun Jan 19 15:17:33 2020

@author: oscar
"""

#!/usr/bin/env python


import cv2

import numpy as np
import rospy
import ros_numpy
import message_filters
from sensor_msgs.msg import Image,LaserScan, PointCloud2


from cv_bridge import CvBridge, CvBridgeError
bridge = CvBridge()



from std_msgs.msg import String

def callback(msg):
    
    points_data = ros_numpy.numpify(msg)
    
    image_data= points_data['rgb'].view((np.uint8, 4))[..., [2, 1, 0]]
    print( "got msg",image_data.shape)
    #cv2_img = bridge.imgmsg_to_cv2(img_msg, "bgr8")
    #print cv2_img.shape
    
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/hsrb/head_rgbd_sensor/depth_registered/rectified_points", PointCloud2, callback)


    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
