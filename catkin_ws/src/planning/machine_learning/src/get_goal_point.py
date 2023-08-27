#! /usr/bin/env python3
import rospy
import numpy as np
import ros_numpy
import tf
import math
from std_msgs.msg import Int32
from geometry_msgs.msg import PointStamped
from actionlib_msgs.msg import GoalStatus
from std_msgs.msg import Bool

def callback_global_goal(msg):
    global goal_x
    global goal_y

    goal_x=msg.point.x
    goal_y=msg.point.y

def get_robot_pose(listener, frame):
    try:
        ([x, y, z], rot) = listener.lookupTransform(frame, 'base_link', rospy.Time(0))
        a = 2*math.atan2(rot[2], rot[3])
        a = a - 2*math.pi if a > math.pi else a
        return [x, y, a]
    except:
        pass
        return [0,0,0]

def odom_map_comparison():
    global listener
    global goal_x
    global goal_y
    [robot_x, robot_y, robot_a]    = get_robot_pose(listener, "map")
    print("Map:")
    dx,dy=goal_x-robot_x, goal_y-robot_y
    print(dx,dy)

def main():
    global listener
    global goal_x
    global goal_y
    msg_offset=Int32()
    rospy.init_node("get_goal_point")

    listener = tf.TransformListener()

    rospy.Subscriber('/clicked_point', PointStamped, callback_global_goal)
    #pub_off = rospy.Publisher("/offset", Int32, queue_size=10)

    loop = rospy.Rate(0.1)
    loop.sleep()
    [goal_x, goal_y, temp] = get_robot_pose(listener,"map") ##Wait for the current pose
    while not rospy.is_shutdown():
        odom_map_comparison()
        #offset=set_offset()
        #msg_offset.data=int(offset)
        #pub_off.publish(msg_offset)
        loop.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
