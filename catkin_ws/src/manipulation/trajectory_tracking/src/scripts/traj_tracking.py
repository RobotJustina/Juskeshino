#!/usr/bin/env python3
import math
import time
import rospy
import tf
import tf.transformations as tft
import numpy
import urdf_parser_py.urdf
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PointStamped
from manip_msgs.srv import *
from tf.transformations import euler_from_quaternion
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def la_trajectory_tracking(joint_trajectory):

    global pubLaGoalPose
    print("Starting left arm trajectory tracking with " + str(len(joint_trajectory.points) ) + "points")
    i = 0
    #ts = joint_trajectory.points[1].time_from_start
    #print("ts=",joint_trajectory.points[1].time_from_start.nsecs)

    ts = 0.05
    #while not rospy.is_shutdown():
    for point in joint_trajectory.points:
        q1, q2, q3, q4, q5, q6, q7 = point.positions
        msg = Float64MultiArray()
        msg.data.append(q1)
        msg.data.append(q2)
        msg.data.append(q3)
        msg.data.append(q4)
        msg.data.append(q5)
        msg.data.append(q6)
        msg.data.append(q7)
        pubLaGoalPose.publish(msg)
        time.sleep(ts)  # Wait ts seconds while moving the arm to the desired position
        i += 1
        #if i >=50:
            #break
    


def ra_trajectory_tracking(joint_trajectory):
    global pubRaGoalPose
    print("Starting right arm trajectory tracking....")
    i = 0
    ts = joint_trajectory.points[1].time_from_start
    ts = 0.05
    for point in joint_trajectory.points:  
        q1, q2, q3, q4, q5, q6, q7 = point.positions 
        msg = Float64MultiArray()
        msg.data.append(q1)
        msg.data.append(q2)
        msg.data.append(q3)
        msg.data.append(q4)
        msg.data.append(q5)
        msg.data.append(q6)
        msg.data.append(q7)
        pubRaGoalPose.publish(msg)
        time.sleep(ts)  # Wait ts seconds while moving the arm to the desired position
        i += 1
        


def callback_la_q_traj(msg):
    print("the topic /manipulation/la_q_trajectory was called....")
    la_trajectory_tracking(msg)
    print("End of trajectory...")

def callback_ra_q_traj(msg):
    print("the topic /manipulation/ra_q_trajectory was called....")
    ra_trajectory_tracking(msg)
    print("End of trajectory...")

def main():
    global pubLaGoalPose, pubRaGoalPose
    print ("node for left and right arm trajectory tracking")
    rospy.init_node('arm_trajectory_tracking')
    pubLaGoalPose = rospy.Publisher("/hardware/left_arm/goal_pose" , Float64MultiArray, queue_size=10);
    pubRaGoalPose = rospy.Publisher("/hardware/right_arm/goal_pose", Float64MultiArray, queue_size=10);
    sub_la_traj = rospy.Subscriber("/manipulation/la_q_trajectory",JointTrajectory, callback_la_q_traj)
    sub_ra_traj = rospy.Subscriber("/manipulation/ra_q_trajectory",JointTrajectory, callback_ra_q_traj)
    loop = rospy.Rate(10)
    while not rospy.is_shutdown():
        #print("subscriptor a jointtraj activo...")
        loop.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    
