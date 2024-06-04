#!/usr/bin/env python
import rospy
import time
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray, Int8

SAFE_DIST = 1

def move_base(linear, angular, t):
    rate = rospy.Rate(1/t)
    cmd = Twist()
    cmd.linear.x = linear
    cmd.angular.z = angular
    pub_cmd_vel.publish(cmd)
    rate.sleep()
    pub_cmd_vel.publish(Twist())

def callback_april_tag(msg):
    dist, angle = msg.data
    if (angle > 0.1) or (angle < -0.1):
        sign = 1 if angle < 0 else -1
        move_base(0, sign * 0.2, 0.1)
    else:
        if dist > SAFE_DIST:
            move_base(0.5, 0, 0.1)

# def callback_approach(msg):
#     move_base(0.1, 0, 1)

def main():
    global pub_cmd_vel
    rospy.init_node("approach_april_tag")
    # rospy.Subscriber("/approach", Int8, callback_approach)
    rospy.Subscriber("vision/fiducial_markers/april_tag_dist_angle", Float32MultiArray, callback_april_tag)
    pub_cmd_vel = rospy.Publisher("/hardware/mobile_base/cmd_vel", Twist, queue_size=10)
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
