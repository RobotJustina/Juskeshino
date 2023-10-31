#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Apr 17 22:54:18 2019
@author: oscar
Edited on Fri October 27 2023
@editor: Daniel_V
"""
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PointStamped
import tf
import tf2_ros
import numpy as np


class PotentialFieldsFollower():
    def __init__(self):
        self.pub_velocity = None
        self.clicked_point_x = 0
        self.clicked_point_y = 0
        self.tf_buffer = tf2_ros.Buffer()
        self.max_laser_value = 10.0
        self.robot_radius = 0.5
        self.robot_speed = Twist()
        self.current_speed = Twist()
        self.counter = 0

        self.robot_speed.angular.z = 0
        rospy.Subscriber("/clicked_point", PointStamped,
                         self.read_point_callback)
        rospy.Subscriber("/hsrb/base_scan", LaserScan,
                         self.read_laser_callback)
        self.pub_velocity = rospy.Publisher(
            '/hsrb/command_velocity', Twist, queue_size=1)

        tf2_ros.TransformListener(self.tf_buffer)
        tf.TransformListener()

    def read_point_callback(self, point_msg):
        self.clicked_point_x = point_msg.point.x
        self.clicked_point_y = point_msg.point.y

    def read_laser_callback(self, laser_msg):
        lectures = np.asarray(laser_msg.ranges)
        lectures[np.isinf(lectures)] = self.max_laser_value

        delta_angles = 4.7124 / len(laser_msg.ranges)
        laser_degrees = np.arange(-2.3562, 2.3562, delta_angles)

        f_x, f_y = 0, 0.001
        for i, degree in enumerate(laser_degrees):
            f_x += (1/lectures[i])**2 * np.cos(degree)
            f_y += (1/lectures[i])**2 * np.sin(degree)

        f_th = np.arctan2(f_y, (f_x+0.000000000001)) + np.pi
        f_magnitude = np.linalg.norm((f_x, f_y))

        try:
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_footprint', rospy.Time(), rospy.Duration(5.0))

            pose = np.asarray((
                transform.transform.translation.x, transform.transform.translation.y,
                transform.transform.translation.z))

            quaternion = np.asarray((
                transform.transform.rotation.x, transform.transform.rotation.y,
                transform.transform.rotation.z, transform.transform.rotation.w))

            x, y = pose[0], pose[1]
            th = tf.transformations.euler_from_quaternion(quaternion)[2]
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print('Waiting for tf to publish')
            x, y = 0, 0
            th = 0

        d_xy = np.array((x, y)) - \
            np.array((self.clicked_point_x, self.clicked_point_y))
        euclidean_dist = np.linalg.norm(d_xy)

        attractor_x = (-x + self.clicked_point_x) / euclidean_dist
        attractor_y = (-y + self.clicked_point_y) / euclidean_dist
        attractor_th = np.arctan2(attractor_y, attractor_x)
        attractor_th = attractor_th - th
        mag_att = np.linalg.norm((attractor_x, attractor_y))
        to_tf_x = f_magnitude*np.cos(f_th)*.0025 + mag_att*np.cos(attractor_th)
        to_tf_y = f_magnitude*np.sin(f_th)*.0025 + mag_att*np.sin(attractor_th)
        to_tf_th = np.arctan2(to_tf_y, to_tf_x)

        if (to_tf_th > np.pi):
            to_tf_th = -np.pi - (to_tf_th - np.pi)

        if (to_tf_th < -np.pi):
            to_tf_th = (to_tf_th + 2*np.pi)

        if (self.clicked_point_x != 0 and self.clicked_point_y != 0):
            rospy.loginfo('f(x)=%f, f(y)=%f, f(th)=%f',
                          f_x, f_y, f_th*180/np.pi)
            rospy.loginfo("(pos_x, pos_y, ang_th) = (%f, %f, %f)",
                          x, y, th*180/3.1416)
            rospy.loginfo("(point_x, point_y): (%f, %f), distance= %f",
                          self.clicked_point_x, self.clicked_point_y, euclidean_dist)
            rospy.loginfo("attraction(x, y, th): (%f, %f, %f)",
                          attractor_x, attractor_y, (attractor_th)*180/np.pi)
            rospy.loginfo("to_tf: (%f, %f, %f)\n", to_tf_x,
                          to_tf_y, to_tf_th*180/np.pi)

            if (euclidean_dist < 0.5):
                self.robot_speed.linear.x = 0
                self.robot_speed.linear.y = 0
                self.robot_speed.angular.z = 0
                self.clicked_point_x = 0.0
                self.clicked_point_y = 0.0

            else:
                lin_multiply = 4.0
                ang_multiply = 10.0
                lin1 = 0.0015 * lin_multiply
                lin2 = 0.0003 * lin_multiply
                lin3 = 0.0005 * ang_multiply
                lin4 = 0.0035 * lin_multiply
                lin5 = 0.003 * ang_multiply

                if (abs(to_tf_th) < .7):  # .27
                    self.robot_speed.linear.x = min(
                        self.current_speed.linear.x + lin1, 0.5)
                    self.robot_speed.angular.z = 0
                    rospy.loginfo('lin')
                else:
                    if to_tf_th > -np.pi/2 and to_tf_th < 0:
                        rospy.loginfo('ang_V -')
                        self.robot_speed.linear.x = max(
                            self.current_speed.linear.x - lin2, 0.04)
                        self.robot_speed.angular.z = max(
                            self.current_speed.angular.z - lin3, -0.2)

                    if to_tf_th < np.pi/2 and to_tf_th > 0:
                        rospy.loginfo('ang_V +')
                        self.robot_speed.linear.x = max(
                            self.current_speed.linear.x - lin2, 0.04)
                        self.robot_speed.angular.z = min(
                            self.current_speed.angular.z + lin3, 0.2)

                    if to_tf_th < -np.pi/2:
                        rospy.loginfo('ang_V ---')
                        self.robot_speed.linear.x = max(
                            self.current_speed.linear.x - lin4, 0.001)
                        self.robot_speed.angular.z = max(
                            self.current_speed.angular.z - lin5, -0.5)

                    if to_tf_th > np.pi/2:
                        rospy.loginfo('ang_V +++')
                        self.robot_speed.linear.x = max(
                            self.current_speed.linear.x - lin4, 0.001)
                        self.robot_speed.angular.z = min(
                            self.current_speed.angular.z + lin5, 0.5)
        else:
            self.counter += 1
            if self.counter == 200:
                print('Waiting for goal clicked point')
                self.counter = 0

    def clicked_point_exist(self):
        flag = self.clicked_point_x != 0 and self.clicked_point_y != 0
        return flag

    def publish_speed(self):
        self.pub_velocity.publish(self.robot_speed)

    def update_current_speed(self):
        self.current_speed = self.robot_speed

    def reset_current_speed(self):
        self.current_speed = Twist()


if __name__ == '__main__':
    try:
        rospy.init_node('point_atractor_navigation', anonymous=True)
        rospy.loginfo('starting node: potential_field_navigator')
        navigator = PotentialFieldsFollower()

        rospy.Rate(10)
        rospy.loginfo('point_atractor_navigation active')
        while not rospy.is_shutdown():
            if (navigator.clicked_point_exist()):
                navigator.publish_speed()
                rospy.sleep(0.15)
                navigator.update_current_speed()
            else:
                navigator.reset_current_speed()

    except rospy.ROSInterruptException:
        pass
