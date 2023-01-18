#!/usr/bin/env python3

import sys 
import rospy
from std_msgs.msg import Float64MultiArray

def main():

    rospy.init_node("torque_pub")

    pubGoalTorque = rospy.Publisher("/hardware/arm/goal_torque", Float64MultiArray, queue_size=1)

    msgGoalTorque = Float64MultiArray()
    msgGoalTorque.data = [0, 0, 0, 0, 0, 0, 0]
    #rospy.sleep(2)

    loop = rospy.Rate(20)

    while not rospy.is_shutdown():
        print("Publicando par...")
        #msgGoalTorque.data = [4, 3]
        pubGoalTorque.publish(msgGoalTorque)
        #rospy.sleep(0.05)
        loop.sleep()

        


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass



