#!/usr/bin/env python
import rospy
<<<<<<< HEAD
import time
import sys
import csv
=======
>>>>>>> 6166866463f8f0b4e6c1c123b79fdc18d079c3fa
from roboclaw_3 import Roboclaw


if __name__ == "__main__":
<<<<<<< HEAD
    right=0
    left=0
    front=0
=======
>>>>>>> 6166866463f8f0b4e6c1c123b79fdc18d079c3fa
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
<<<<<<< HEAD
    motor=str(rospy.get_param("~motor", "front"))
    #start_time= time.time()
    while not rospy.is_shutdown():
        if motor== "right":
            right = rcf.ReadEncM2(128)
            print(f'Encoder right: {right}')
        elif motor== "left":
            left = rcf.ReadEncM1(128)
            print(f'Encoder left: {left}')
        elif motor == "rear":
            rear = rcl.ReadEncM2(128)
            print(f'Encoder rear: {rear}')
        #end_time= time.time()
        #final_time= end_time-start_time
        #if final_time >=1:
        #    qpps = rear[1]/final_time
        #    print(f'tiempo final: {final_time}')  
        #    print(f'qpps: {qpps}')
        #    sys.exit(0)    
=======
    while not rospy.is_shutdown():
        enc2l = rcl.ReadEncM2(128)
        enc2f = rcf.ReadEncM2(128)
        enc1f = rcf.ReadEncM1(128)
        print(f'Encoder Rclawl M2: {enc2l}')
        print(f'Encoder Rcalwf M2: {enc2f}')
        print(f'Encoder Rclawf M1: {enc1f}')
>>>>>>> 6166866463f8f0b4e6c1c123b79fdc18d079c3fa
        rate.sleep()
