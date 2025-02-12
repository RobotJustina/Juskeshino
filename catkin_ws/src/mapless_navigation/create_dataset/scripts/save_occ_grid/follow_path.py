#!/usr/bin/env python3
import rospy
import tf
import tf2_ros
from nav_msgs.msg import Path
from geometry_msgs.msg import Point, PointStamped, PoseStamped
from geometry_msgs.msg import Twist
from juskeshino_tools.JuskeshinoNavigation import JuskeshinoNavigation
import math
import numpy as np



def getRobotPoseWrtOdom():
    listener = tf.TransformListener()
    try:
        listener.waitForTransform('odom', 'base_link', rospy.Time(0), rospy.Duration(2.0))
        ([x, y, z], rot) = listener.lookupTransform('odom', 'base_link', rospy.Time(0))

        a = 2*math.atan2(rot[2], rot[3])
        print("a", a, np.sign(a))
        if abs(a) > math.pi:
            a = a - (np.sign(a)*2*math.pi)
        print("a_", a)
        print()
        return [x, y, a]
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        rospy.logwarn('Failed to get the transformation:\n%s' % str(e))			
        return [0, 0, 0]




def update_delta(p):

    listener = tf.TransformListener()
    try:
        listener.waitForTransform('odom', 'base_link', rospy.Time(0), rospy.Duration(2.0))
        ([x, y, z], rot) = listener.lookupTransform('odom', 'base_link', rospy.Time(0))

        theta = 2*math.atan2(rot[2], rot[3])
        if abs(theta) > math.pi:
            theta = theta - (np.sign(theta)*2*math.pi)
        pos = p.pose.position
        dx = pos.x - x
        dy = pos.y - y
        angle_to_goal = math.atan2(dy, dx)
        delta_angle = angle_to_goal - theta
        if abs(delta_angle) > math.pi:
            delta_angle = delta_angle - (np.sign(delta_angle)*2*math.pi)

        distance = math.sqrt(math.pow(dx, 2) + math.pow(dy, 2))
        return delta_angle, distance
    
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        rospy.logwarn('Failed to get the transformation:\n%s' % str(e))			
        return 0, 0, 0, 0

def pathFollowerCalback(msg):
    m = Path()
    first = True
    JuskeshinoNavigation.setNodeHandle()
    for p in msg.poses:
        if first:
            first = False
        else:

            delta_angle, distance = update_delta(p)

            # print("a_goal", angle_to_goal)
            # print("d_a", delta_angle)
            while abs(delta_angle) > 0.3 and distance < 0.3:
                print(">><<")
                # print("a_goal", angle_to_goal)
                # print("d_a", delta_angle)
                # dir = (delta_angle) / abs(delta_angle)

                JuskeshinoNavigation.startMoveDistAngle(0, delta_angle)
                delta_angle, distance = update_delta(p)
                print("delta_angle", delta_angle)
                if abs(delta_angle) > math.pi:
                    delta_angle = delta_angle - (np.sign(delta_angle)*2*math.pi)

                print("delta_angle", delta_angle)
            #print("dist", distance)
            while distance > 0.3:
                print("--")
                print("dist", distance)
                print("d_a", delta_angle)
                speed_factor = 1.2
                if abs(delta_angle) > 0.25:
                    JuskeshinoNavigation.startMoveDistAngle(
                        0.05*abs(delta_angle), delta_angle*1.1)
                else:
                    JuskeshinoNavigation.startMoveDist(distance*speed_factor)
                delta_angle, distance = update_delta(p)
                print("delta_angle", delta_angle)
            print()
    print("-------")


def main():
    global pub, listener

    
    rospy.init_node('follow_path')
    rospy.logwarn("follow path")
    JuskeshinoNavigation.setNodeHandle()
    listener = tf.TransformListener()

    rospy.Subscriber('/mapless/goal_path', Path, pathFollowerCalback)
    pub = rospy.Publisher("/hardware/mobile_base/cmd_vel", Twist, queue_size=1)

    rospy.spin()


if __name__ == '__main__':
    main()
