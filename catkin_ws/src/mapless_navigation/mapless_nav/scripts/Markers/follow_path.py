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
    print(f"robot: ({x:.2f}, {y:.2f}, {theta:.2f})")
    pos = p.pose.position
    print(f"point: ({pos.x:.2f}, {pos.y:.2f}, {p.pose.orientation.z:.2f})")
    dx = pos.x - x
    dy = pos.y - y
    angle_to_goal = math.atan2(dy, dx)
    return angle_to_goal, theta, dx, dy


def followPathCallback(msg):
    global pub
    print("navigating")
    m = Path()
    first = True
    speed = Twist()
    count = 0
    for p in msg.poses:
        if first:
            first = False
        else:

            angle_to_goal, theta, dx, dy = update_delta(p)
            distance = math.sqrt(math.pow(dx, 2) + math.pow(dy, 2))
            while distance >= 0.2:
                delta_angle = angle_to_goal - theta
                if abs(delta_angle) > 0.2:
                    print("d_a:", delta_angle)
                    scale = 0.1
                    dir = (delta_angle) / abs(delta_angle)
                    angSpeed = min(0.5, math.pow(delta_angle, 2) / scale)
                    speed.angular.z = dir*angSpeed
                    #print("a_sp", angSpeed)
                    speed.linear.x = math.pow(.1 / delta_angle, 2)
                    print("linear:", speed.linear.x)
                    print()
                elif abs(delta_angle) > 1.6:
                    speed.linear.x = 0.01
                else:
                    speed.angular.z = 0.0
                    speed.linear.x = 0.3
                pub.publish(speed)

                angle_to_goal, theta, dx, dy = update_delta(p)
                distance = math.sqrt(math.pow(dx, 2) + math.pow(dy, 2))


                
            # print(pos)
            print("-------->")
            print()
            
            # speed.linear.x = 0.0
            # speed.angular.z = 0.0
            # pub.publish(speed)
        count +=1
        # if count == 2:
        #     break
    print(">>>")
    print()
    speed.linear.x = 0.0
    speed.angular.z = 0.0
    pub.publish(speed)


def main():
    global pub
    rospy.init_node('follow_path')
    rospy.logwarn("follow path")
    rospy.Subscriber('/mapless/goal_path', Path, followPathCallback)
    pub = rospy.Publisher("/hardware/mobile_base/cmd_vel", Twist, queue_size=1)

    rospy.spin()


if __name__ == '__main__':
    main()
