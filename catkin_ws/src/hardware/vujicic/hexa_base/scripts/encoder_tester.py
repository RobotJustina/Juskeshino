#!/usr/bin/env python
import rospy
from roboclaw_3 import Roboclaw


if __name__ == "__main__":
    rospy.init_node('encTest')
    rate=rospy.Rate(10)
    print("Try to ")
    base=Roboclaw("/dev/vujicicRC15", 115200)
    base.Open()
    print("Openport")
    while not rospy.is_shutdown():
        enc1f = base.ReadEncM2(128)
        enc2f = base.ReadEncM1(128)
        #enc1l = base.ReadEncM1(self.rc_address_lateral)
        print(f'Encoder Rclawf Motor 1: {enc1f}')
        print(f'Encoder Rcalwf Motor 2: {enc2f}')
        #print(f'Encoder Rclawl Motor 1: {enc1l}')
        rate.sleep()
