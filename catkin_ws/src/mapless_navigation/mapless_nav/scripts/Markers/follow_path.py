#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import Point, PointStamped, PoseStamped
from geometry_msgs.msg import Twist
from juskeshino_tools.JuskeshinoNavigation import JuskeshinoNavigation
import math
th_d = 0.1


def update_delta(p):
    x, y, theta = JuskeshinoNavigation.getRobotPoseWrtOdom()
    pos = p.pose.position
    dx = pos.x - x
    dy = pos.y - y
    angle_to_goal = math.atan2(dy, dx)
    angle_to_goal = angle_to_goal - 2 * \
        math.pi if angle_to_goal > math.pi else angle_to_goal
    return angle_to_goal, theta, dx, dy


def pathFollowerCalback(msg):
    m = Path()
    first = True
    count = 0

    for p in msg.poses:
        count += 1
        if first:
            first = False
        else:
           
            angle_to_goal, theta, dx, dy = update_delta(p)
            delta_angle = angle_to_goal - theta
            distance = math.sqrt(math.pow(dx, 2) + math.pow(dy, 2))
            print("a_goal", angle_to_goal)
            print("d_a", delta_angle)
            while abs(delta_angle) > 0.3 and distance < 0.3:
                print("a_goal", angle_to_goal)
                print("d_a", delta_angle)
                # dir = (delta_angle) / abs(delta_angle)
                delta_angle = delta_angle - 2*math.pi if delta_angle > math.pi else delta_angle
                JuskeshinoNavigation.startMoveDistAngle(0, delta_angle)
                angle_to_goal, theta, dx, dy = update_delta(p)
                delta_angle = angle_to_goal - theta
            print("dist", distance)
            while distance > 0.3:
                print("dist", distance)
                print("d_a", delta_angle)
                speed_factor = 1.2
                if abs(delta_angle) > 0.25:
                    JuskeshinoNavigation.startMoveDistAngle(0.05*abs(delta_angle), delta_angle*1.1) #0.1 -- d_a
                else:
                    JuskeshinoNavigation.startMoveDist(distance*speed_factor)
                angle_to_goal, theta, dx, dy = update_delta(p)
                distance = math.sqrt(math.pow(dx, 2) + math.pow(dy, 2))
                delta_angle = angle_to_goal - theta
                delta_angle = delta_angle - 2*math.pi if delta_angle > math.pi else delta_angle
                
            print()
    print("-------")


def main():
    global pub

    rospy.init_node('follow_path')
    rospy.logwarn("follow path")
    JuskeshinoNavigation.setNodeHandle()

    rospy.Subscriber('/mapless/goal_path', Path, pathFollowerCalback)
    pub = rospy.Publisher("/hardware/mobile_base/cmd_vel", Twist, queue_size=1)

    rospy.spin()


if __name__ == '__main__':
    main()
