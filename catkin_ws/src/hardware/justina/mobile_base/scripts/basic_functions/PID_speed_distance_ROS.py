#!/usr/bin/env python3

import rospy
import numpy as np
from roboclaw_3 import Roboclaw
import time 

ADDRESS = 0x80
ADDRESS1 = 0x81
roboclaw = Roboclaw("/dev/justinaRC15", 38400)
roboclaw1 = Roboclaw("/dev/justinaRC30", 38400)
roboclaw.Open()
roboclaw1.Open()
P = 1.0
I = 0.5
D = 0.1

roboclaw.SetM1VelocityPID(ADDRESS,P,I,D,10000)
roboclaw.SetM2VelocityPID(ADDRESS,P,I,D,10000)
roboclaw1.SetM1VelocityPID(ADDRESS1,P,I,D,10000)

qpps_per_meter = 10000

R = 0.2     # Distancia del centro del robot a las ruedas (m)
r = 0.15    # Radio de las ruedas (m)

alpha1, alpha2, alpha3 = np.deg2rad([-60, 60, 180])

matriz = np.array([

        [-np.sin(alpha1), np.cos(alpha1), R],

        [-np.sin(alpha2), np.cos(alpha2), R],

        [-np.sin(alpha3), np.cos(alpha3), R]

    ])

goal_x = 0.5
goal_y = 0.1
goal_w = 0.3


def move_distance ():

    velocities = np.array ([[goal_x], [goal_y], [goal_w]])
    wheel_speed = np.dot (matriz, velocities).flatten()
    speed_qpps = np.round(wheel_speed * qpps_per_meter).astype(int)

    roboclaw.SpeedDistanceM1M2(ADDRESS, speed_qpps[0], abs(speed_qpps[0]), 
                                              speed_qpps[1], abs(speed_qpps[1]), 1)
    
    roboclaw1.SpeedDistanceM1(ADDRESS1, speed_qpps[2], abs(speed_qpps[2]), 1)


    while roboclaw.ReadBuffers(ADDRESS)[1] or roboclaw1.ReadBuffers(ADDRESS1)[1]:
        rospy.sleep(0.1)

def PID_speed_distance_ROS():
    rospy.init_node("PID_speed_distance_ROS")
    
    rospy.sleep(2)  
    move_distance()  
    rospy.spin()  # Mantiene el nodo activo


if __name__ == "__main__":
    try:
        PID_speed_distance_ROS()
    





    
