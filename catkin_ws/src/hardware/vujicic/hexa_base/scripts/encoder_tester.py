#!/usr/bin/env python
import rospy
import time
import sys
import csv
from roboclaw_3 import Roboclaw


if __name__ == "__main__":
    right=0
    left=0
    front=0
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
        rate.sleep()
