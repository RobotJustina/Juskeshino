#!/usr/bin/env python
import rospy
import time
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray, Int8

SAFE_DIST = 1
moving = False
moving_angular = False
moving_towards = False
moving_lateral = False

def move_base(linear_x, linear_y, angular):
    cmd = Twist()
    cmd.linear.x = linear_x
    cmd.linear.y = linear_y
    cmd.angular.z = angular
    pub_cmd_vel.publish(cmd)

def callback_april_tag(msg):
    dist, angle, cX, cY = msg.data
    if abs(angle) > 0.05:
        print(f"Moving angularly: {msg.data}")
        sign = 1 if angle < 0 else -1
        move_base(0, 0, sign * 0.08)
        return None
    else:
        print("Stopping angular movement")
        pub_cmd_vel.publish(Twist())

    if cX < 310:
        move_base(0, 0.2, 0)
        return None
    elif cX > 330:
        move_base(0, -0.2, 0)
        return None
    else:
        print("Stopping lateral movement")
        pub_cmd_vel.publish(Twist())

    if dist > SAFE_DIST:
        print(f"Moving linearly: {msg.data}")
        move_base(0.1, 0, 0)
    else:
        print("Stopping linear movement")
        pub_cmd_vel.publish(Twist())

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
