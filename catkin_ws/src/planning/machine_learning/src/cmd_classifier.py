#! /usr/bin/env python3

import rospy
import numpy as np
import cv2
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32

def callback_cmd(msg):
    global pub_class
    msg_class=Int32()
    if(msg.angular.z>0):
        msg_class.data=2
    elif(msg.angular.z<0):
        msg_class.data=3
    elif(msg.linear.x>0):
        msg_class.data=1
    else:
        msg_class.data=0
    pub_class.publish(msg_class)
    print("----------")
    #print(msg.linear.x)
    #print(msg.angular.z)

def main():
    global pub_class
    rospy.init_node("cmd_classifier")
    rospy.Subscriber("/cmd_vel", Twist, callback_cmd)
    pub_class=rospy.Publisher('/class', Int32, queue_size=10)
    print("cmd_classifier has been started")
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
