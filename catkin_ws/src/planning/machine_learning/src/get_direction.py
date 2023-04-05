#! /usr/bin/env python3
import rospy
import numpy as np
import ros_numpy
import tf
import math
from geometry_msgs.msg import PointStamped

def callback_global_goal(msg):
    print("Calculating path from robot pose to " + str([msg.point.x, msg.point.y]))
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
    rospy.init_node("get_direction")
    rospy.Subscriber('/clicked_point', PointStamped, callback_global_goal)
    listener = tf.TransformListener()
    loop = rospy.Rate(2)
    #rospy.spin()
    while( not rospy.is_shutdown()):
        [robot_x, robot_y, robot_a]    = get_robot_pose(listener)
        print(str(robot_x) +" "+ str(robot_y)+" "+str(robot_a))
        loop.sleep()

if __name__ == '__main__':
    try:
        while not rospy.is_shutdown():
            main()
    except rospy.ROSInterruptException:
        pass
