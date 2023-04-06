#! /usr/bin/env python3
import rospy
import numpy as np
import ros_numpy
import tf
import math
from geometry_msgs.msg import PointStamped

def callback_global_goal(msg):
    global goal_x
    global goal_y
    goal_x=msg.point.x
    goal_y=msg.point.y
    #print("Calculating path from robot pose to " + str([msg.point.x, msg.point.y]))
    #[robot_x, robot_y, robot_a] = get_robot_pose(listener)

def get_robot_pose(listener):
    try:
        ([x, y, z], rot) = listener.lookupTransform('map', 'base_link', rospy.Time(0))
        a = 2*math.atan2(rot[2], rot[3])
        a = a - 2*math.pi if a > math.pi else a
        return [x, y, a]
    except:
        pass
        return [0,0,0]

def main():
    global listener
    global goal_x
    global goal_y
    goal_y=0
    goal_x=0
    rospy.init_node("get_direction")
    rospy.Subscriber('/clicked_point', PointStamped, callback_global_goal)
    listener = tf.TransformListener()
    loop = rospy.Rate(0.1)
    #rospy.spin()
    while( not rospy.is_shutdown()):
        [robot_x, robot_y, robot_a]    = get_robot_pose(listener)
        ang_pos=math.atan2(goal_y-robot_y, goal_x-robot_x)
        if ang_pos > math.pi:
            ang_pos=ang_pos-2*math.pi
        ang=ang_pos-robot_a
        if ang > math.pi:
            ang=ang-2*math.pi
        if (ang>=0 and ang< math.pi/4.0) or (ang<0 and ang > -math.pi/4.0):
            print("Al frente")
        elif(ang>=math.pi/4.0 and ang< 3*math.pi/4.0):
            print("Izquierda")
        elif(ang>=3*math.pi/4.0 and ang<= math.pi) or (ang>=-math.pi and ang <= -3*math.pi/4.0):
            print("Atras")
        else:
            print("Derecha")
        #print(str(robot_x) +" "+ str(robot_y)+" "+str(robot_a)+" "+str(ang))
        #print(ang)
        loop.sleep()

if __name__ == '__main__':
    try:
        while not rospy.is_shutdown():
            main()
    except rospy.ROSInterruptException:
        pass
