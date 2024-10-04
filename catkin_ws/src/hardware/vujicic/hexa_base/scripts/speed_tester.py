#!/usr/bin/env python
import rospy
from roboclaw_3 import Roboclaw
     
def main():
    rospy.init_node('speedTest')
    rate=rospy.Rate(10)
    right=int(rospy.get_param("~right", 0))
    left=int(rospy.get_param("~left", 0))
    rear=int(rospy.get_param("~rear", 0))
    print("Try to connect roboclaws")
    rcl=Roboclaw("/dev/vujicicRC15", 115200)
    rcl.Open()
    print("Openport Roboclaw lateral")
    rcf=Roboclaw("/dev/ttyACM0", 115200)
    rcf.Open()
    print("Openport Roboclaw frontal")
    print(f'Speed Rcalwf M1: {left}')
    print(f'Speed Rcalwf M2: {right}')
    print(f'Speed Rcalwl M2: {rear}')
    while not rospy.is_shutdown():
        if right>0:
            rcf.ForwardM2(128,right)
        else:
            rcf.BackwardM2(128,right)
            
        if left>0:
            rcf.ForwardM1(128,left)
        else:
            rcf.BackwardM1(128,left)
            
        if rear>0:
            rcl.ForwardM2(128,rear)
        else:
            rcf.BackwardM2(128,rear)
        rate.sleep()
    print("Parando motores")
    rcf.ForwardM2(128,0)
    rcf.ForwardM1(128,0)
    rcl.ForwardM2(128,0)
    print("Motores parados")
        
if __name__ == "__main__":
    main()
    
