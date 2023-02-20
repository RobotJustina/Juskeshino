#! /usr/bin/env python3
#
# Position 3.2597, 6.2913, 0.13
# Orientacion 0,0,-0.707,0.707

#  z=-0.13
# Pos  4.73, 6.13, 0.13
# Ori 0,0, -0.62, 0.78

# z=-0.145
# Pos 4.83 6.09 0.13
#Ori 0 0 -0.72 0.69

### mover simple_move/goal_dist_lateral

import rospy
import numpy as np
from sensor_msgs.msg   import LaserScan
from geometry_msgs.msg import Twist


def Last_pub():
    pub_cmd_vel.publish(Twist())
    print("Se cierra el nodo")

def callback_laser_scan(msg):
    #
    # TODO:
    # Do something to detect if there is an obstacle in front of the robot.
    #
    global obstacle
    #obstacle=msg.ranges[len(msg.ranges)//2]<1.0 185
    #np.random.randint(0, len(msg))
    obstacle=msg.ranges[len(msg.ranges)-1]<1.0
    print(len(msg.ranges))
    return

rospy.init_node("RL")
rospy.Subscriber("/hardware/scan", LaserScan, callback_laser_scan)
pub_cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
loop = rospy.Rate(10)
rospy.on_shutdown(Last_pub)


def main():
    global obstacle
    #rospy.init_node("RL")
    #rospy.Subscriber("/hardware/scan", LaserScan, callback_laser_scan)
    #pub_cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    #loop = rospy.Rate(10)
    #rospy.on_shutdown(Last_pub)

    obstacle=False
    cmd_vel=Twist()
    while True:
        #
        # TODO:
        # Declare a Twist message and assign the appropiate speeds:
        # Move forward if there is no obstacle in front and stop otherwise.
        # Publish the message.
        #
        if obstacle:
            cmd_vel=Twist()
        else:
            cmd_vel.linear.y=0.2
            cmd_vel.angular.z=-0.13
        pub_cmd_vel.publish(cmd_vel)
        loop.sleep()

if __name__ == '__main__':
    try:
        while not rospy.is_shutdown():
            main()
    except rospy.ROSInterruptException:
        pass
