#!/usr/bin/env python
import rospy
from roboclaw_3 import Roboclaw


if __name__ == "__main__":
    rospy.init_node('encTest')
    rate=rospy.Rate(10)
    print("Try to connect roboclaws")
    rcl=Roboclaw("/dev/vujicicRC15", 115200)
    rcl.Open()
    print("Openport Roboclaw lateral")
    rcf=Roboclaw("/dev/vujicicRC30", 115200)
    rcf.Open()
    print("Openport Roboclaw frontal")
    rcf.ResetEncoders(128)
    rcl.ResetEncoders(128)
    print("Reset encoders")
    while not rospy.is_shutdown():
        enc2l = rcl.ReadEncM2(128)
        enc2f = rcf.ReadEncM2(128)
        enc1f = rcf.ReadEncM1(128)
        print(f'Encoder Rclawl M2: {enc2l}')
        print(f'Encoder Rcalwf M2: {enc2f}')
        print(f'Encoder Rclawf M1: {enc1f}')
        rate.sleep()
