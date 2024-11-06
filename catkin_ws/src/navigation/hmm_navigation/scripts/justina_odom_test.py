
# -*- coding: utf-8 -*-
#! /usr/bin/env python3

import rospy
import tf2_ros

from geometry_msgs.msg import Point, PoseStamped, PoseWithCovarianceStamped, PointStamped
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry



def pos_map_callback(msg):
    
    try:
        transformation_odom = tfBuffer.lookup_transform("map", "base_link", rospy.Time(0), timeout=rospy.Duration(1))
        pose_odom = tfBuffer.transform
    except: 
        rospy.WARN("Error al transformar la posicion del objeto al marco de referencia")
    
    base_quaternion = [transformation_odom.transform.rotation.x,
                        transformation_odom.transform.rotation.y, 
                        transformation_odom.transform.rotation.z, 
                        transformation_odom.transform.rotation.w]

    _,_,theta = euler_from_quaternion(base_quaternion)
    x = transformation_odom.point.x
    y = transformation_odom.point.y
    
    # base_odom = PoseStamped()
    # base_odom.pose.position = Point(x, y, 0)
    # base_odom.pose.orientation =  Point(0, 0, theta)
    

    base_odom = Odometry()
    base_odom.pose.pose.position.x = x
    base_odom.pose.pose.position.y = y
    base_odom.pose.pose.position.z = 0
    base_odom.pose.pose.orientation.x = transformation_odom.transform.rotation.x
    base_odom.pose.pose.orientation.y = transformation_odom.transform.rotation.y
    base_odom.pose.pose.orientation.z = transformation_odom.transform.rotation.z
    base_odom.pose.pose.orientation.w = transformation_odom.transform.rotation.w
    
    #base_odom.twist.twist.linear.x = 
    #base_odom.twist.twist.linear.y = 
    base_odom.twist.twist.angular.z = theta

    pose_map_pub.publish(base_odom)  
    print(x, y, theta)
    x2 = msg.pose.pose.position.x
    y2 = msg.pose.pose.position.y
    #print(x2, y2)
    


if __name__ == '__main__':

    rospy.init_node('justina_odom_test')
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    #pose_pub = rospy.Publisher("justina_wheel_odom", PoseStamped, queue_size=1)
    pose_odom_pub = rospy.Publisher("justina_wheel_odom", Odometry, queue_size=1)
    pose_map_pub = rospy.Publisher("justina_wheel_map", Odometry, queue_size=1)
    
    #rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, pos_map_callback)

    rate = rospy.Rate(2)
    while not rospy.is_shutdown():
        
        point_orig = PointStamped()
        point_orig.header.frame_id = "base_link"
        point_orig.point.x = 0.0
        point_orig.point.y = 0.0
        point_orig.point.z = 0.0


        try:
            #transform_pose = tfBuffer.transform(point_orig, "base_link", rospy.Duration(0))
            l_transformation_odom = tfBuffer.lookup_transform("odom", "base_link", rospy.Time(0), timeout=rospy.Duration(1))
            

        except: 
            rospy.WARN("Error al transformar la posicion del objeto al marco de referencia")
        
        base_quaternion = [l_transformation_odom.transform.rotation.x,
                         l_transformation_odom.transform.rotation.y, 
                         l_transformation_odom.transform.rotation.z, 
                         l_transformation_odom.transform.rotation.w]

        _,_,theta = euler_from_quaternion(base_quaternion)
        x = 0#transform_pose.point.x
        y = 0#transform_pose.point.y

        # base_odom = PoseStamped()
        # base_odom.pose.position = Point(x, y, 0)
        # base_odom.pose.orientation =  Point(0, 0, theta)
        

        base_odom = Odometry()
        base_odom.pose.pose.position.x = x
        base_odom.pose.pose.position.y = y
        base_odom.pose.pose.position.z = 0
        base_odom.pose.pose.orientation.x = l_transformation_odom.transform.rotation.x
        base_odom.pose.pose.orientation.y = l_transformation_odom.transform.rotation.y
        base_odom.pose.pose.orientation.z = l_transformation_odom.transform.rotation.z
        base_odom.pose.pose.orientation.w = l_transformation_odom.transform.rotation.w
        
        #base_odom.twist.twist.linear.x = 
        #base_odom.twist.twist.linear.y = 
        base_odom.twist.twist.angular.z = theta


        pose_odom_pub.publish(base_odom)
        print(x, y, theta)

        rate.sleep()
    
    