#!/usr/bin/env python
import rospy
import time
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray, Int8

SAFE_DIST = 1
moving = False
moving_angular = True

def move_base(linear, angular):
    cmd = Twist()
    cmd.linear.x = linear
    cmd.angular.z = angular
    pub_cmd_vel.publish(cmd)

def callback_april_tag(msg):
    global moving
    dist, angle = msg.data
    if (angle > 0.05) or (angle < -0.05):
        print(f"Moving angularly: {msg.data}")
        moving_angular = True
        sign = 1 if angle < 0 else -1
        move_base(0, sign * 0.1)
    else:
        print("Stopping angular movement")
        moving_angular = False
        pub_cmd_vel.publish(Twist())

    if moving_angular:
        return None

    if dist > SAFE_DIST:
        print(f"Moving linearly: {msg.data}")
        move_base(0.1, 0)
    else:
        print("Stopping linear movement")
        pub_cmd_vel.publish(Twist())
    
    print() 

    # global moving_angular
    # global moving

    # if moving:
    #     return None

    # dist, angle = msg.data
    # if ((angle > 0.1) or (angle < -0.1)) and moving_angular:
    #     sign = 1 if angle < 0 else -1
    #     moving = True
    #     move_base(0, sign * 0.03, 0.1)
    #     moving = False
    # else:
    #     moving_angular = False
    #     print(f"Stop moving at angular: {angle}")
    #     # if dist > SAFE_DIST:
    #     #     move_base(0.1, 0, 0.1)

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
