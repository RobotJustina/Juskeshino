#! /usr/bin/env python3
import rospy
import numpy as np
import ros_numpy
import tf
import math
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PointStamped


def callback_global_goal(msg):
    global goal_x
    global goal_y

    goal_x = msg.point.x
    goal_y = msg.point.y


def get_robot_pose(listener, frame):
    try:
        ([x, y, z], rot) = listener.lookupTransform(
            frame, 'base_link', rospy.Time(0))
        a = 2*math.atan2(rot[2], rot[3])
        a = a - 2*math.pi if a > math.pi else a
        return [x, y, a]
    except:
        pass
        return [0, 0, 0]


def get_direction():
    global listener
    global goal_x
    global goal_y
    [robot_x, robot_y, robot_a] = get_robot_pose(listener, "odom")
    ang_pos = math.atan2(goal_y-robot_y, goal_x-robot_x)
    d = math.sqrt((goal_y-robot_y)**2 + (goal_x-robot_x)**2)
    if ang_pos > math.pi:
        ang_pos = ang_pos-2*math.pi
    ang = ang_pos-robot_a

    if (ang >= math.pi):
        ang = ang-2*math.pi
    if (ang < -math.pi):
        ang = ang+2*math.pi
    print("-------------")
    print("Angulo", ang)
    print("Distancia", d)
    return [d, ang]


def main():
    global listener
    global goal_x
    global goal_y
    rospy.init_node("get_goal_point")

    listener = tf.TransformListener()

    rospy.Subscriber('/clicked_point', PointStamped, callback_global_goal)
    pub_goal = rospy.Publisher("/NN_goal", Float32MultiArray, queue_size=10)

    loop = rospy.Rate(50)
    listener.waitForTransform(
        "odom", "base_link", rospy.Time(), rospy.Duration(4.0))
    msg = Float32MultiArray()
    [goal_x, goal_y, temp] = get_robot_pose(
        listener, "odom")  # Wait for the current pose
    while not rospy.is_shutdown():
        data_goal = get_direction()
        msg.data = data_goal
        # msg_offset.data=int(offset)
        pub_goal.publish(msg)
        loop.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
