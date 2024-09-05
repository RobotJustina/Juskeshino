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
    #offset=set_offset()
    #msg_offset.data=int(offset)

    #pub_off.publish(msg_offset)

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

def set_offset():
    global listener
    global goal_x
    global goal_y
    [robot_x, robot_y, robot_a]    = get_robot_pose(listener)
    ang_pos=math.atan2(goal_y-robot_y, goal_x-robot_x)
    d=math.sqrt( (goal_y-robot_y)**2 + (goal_x-robot_x)**2 )
    if ang_pos > math.pi:
        ang_pos=ang_pos-2*math.pi
    ang=ang_pos-robot_a
    if ang >= math.pi:
        ang=ang-2*math.pi
    if ang < -math.pi:
        ang=ang+2*math.pi
    if (ang>=0 and ang< math.pi/4.0) or (ang<0 and ang > -math.pi/4.0):
        #print("Al frente")
        offset=0
    elif(ang>=math.pi/4.0 and ang< 3*math.pi/4.0):
        #print("Izquierda")
        offset=32
    elif(ang>=3*math.pi/4.0 and ang<= math.pi) or (ang>=-math.pi and ang <= -3*math.pi/4.0):
        #print("Atras")
        offset=64
    else:
        #print("Derecha")
        offset=96
    if(d<0.2):
        offset+=500
    print(d)
    return offset

def main():
    global listener
    global goal_x
    global goal_y
    #global msg_offset
    msg_offset=Int32()
    #goal_y=0
    #goal_x=0
    rospy.init_node("get_direction")
    rospy.Subscriber('/clicked_point', PointStamped, callback_global_goal)
    #rospy.Subscriber("/simple_move/goal_reached", GoalStatus, callback_goal);
    pub_off = rospy.Publisher("/offset", Int32, queue_size=10)
    loop = rospy.Rate(10)
    listener = tf.TransformListener()
    for i in range(5):
        loop.sleep()
    [goal_x, goal_y, temp] = get_robot_pose(listener)
    while(not rospy.is_shutdown()):
        #set_offset()
        #print(str(robot_x) +" "+ str(robot_y)+" "+str(robot_a)+" "+str(ang))
        #print(ang)
        offset=set_offset()
        msg_offset.data=int(offset)
        pub_off.publish(msg_offset)
        loop.sleep()

if __name__ == '__main__':
    try:
        while not rospy.is_shutdown():
            main()
    except rospy.ROSInterruptException:
        pass
