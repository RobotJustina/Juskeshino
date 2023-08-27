#! /usr/bin/env python3

import rospy
#import numpy as np
#import ros_numpy
from std_msgs.msg import Float32MultiArray

def grid_msg():
    rospy.init_node("grid_generator")
    pub_arr =rospy.Publisher("/grid", Float32MultiArray, queue_size=10)
    print("grid generator has been started")
    loop = rospy.Rate(10)

    while not rospy.is_shutdown():
        Arr=Float32MultiArray()
        Arr.data=[0.0, -0.5, 10, 10,0.1]
        pub_arr.publish(Arr)
        print("mensaje enviado")
        loop.sleep()

if __name__ == '__main__':
    try:
        grid_msg()
    except rospy.ROSInterruptException:
        pass
