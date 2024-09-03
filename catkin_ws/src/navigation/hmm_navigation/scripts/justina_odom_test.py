
# -*- coding: utf-8 -*-
#! /usr/bin/env python3

import rospy
import tf2_ros

from geometry_msgs.msg import Point, PoseStamped
from tf.transformations import euler_from_quaternion

if __name__ == '__main__':

    rospy.init_node('justina_odom_test')
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    pose_pub = rospy.Publisher("justina_wheel_odom", PoseStamped, queue_size=1)

    rate = rospy.Rate(2)
    while not rospy.is_shutdown():
        transformation = None
        try:
            transformation = tfBuffer.lookup_transform("odom", "base_link", rospy.Time(0), timeout=rospy.Duration(1))
            

        except: 
            rospy.WARN("Error al transformar la posicion del objeto al marco de referencia")
        
        base_quaternion = [transformation.transform.rotation.x,
                         transformation.transform.rotation.y, 
                         transformation.transform.rotation.z, 
                         transformation.transform.rotation.w]

        _,_,theta = euler_from_quaternion(base_quaternion)
        x = transformation.transform.rotation.x
        y = transformation.transform.rotation.y

        # base_odom = PoseStamped()
        # base_odom.pose.position = Point(x, y, 0)
        # base_odom.pose.orientation =  Point(0, 0, theta)
        
        # pose_pub.publish(base_odom)
        print(x, y, theta)



        rate.sleep()
    