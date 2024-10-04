#!/usr/bin/env python
import rospy
from roboclaw_3 import Roboclaw
import signal 
import sys
rcl=None
rcf=None
qpps=980
def signal_handler(sig, frame):
    rospy.loginfo("parando los motores")
    reset_motors()
    rospy.signal_shutdown("Test ended")
    sys.exit(0)     

def reset_motors():
    global rcf,rcl
    
    rcf.SetM1VelocityPID(128,0,0,0,qpps)
    #rcf.SetM2VelocityPID(128,0,0,0,qpps)
    #rcl.SetM2VelocityPID(128,0,0,0,qpps)
    rcf.SpeedM1(128,0)
    #rcf.SpeedM2(128,0)
    #rcl.SpeedM2(128,0)
    rospy.loginfo("Motores parados")
          
def main():
    global rcf,rcl, rear, front, left, P, I, D,qpps
    rospy.init_node('PIDTest')
    signal.signal(signal.SIGINT, signal_handler)
    rate=rospy.Rate(10)
    right=int(rospy.get_param("~right", 0))
    rear=int(rospy.get_param("~rear", 0))
    left=int(rospy.get_param("~left", 0))
    P=int(rospy.get_param("~P", 0))
    I=int(rospy.get_param("~I", 0))
    D=int(rospy.get_param("~D", 0))
    print(P,I,D)
    print("Try to connect roboclaws")
    rcl=Roboclaw("/dev/vujicicRC15", 115200)
    rcl.Open()
    print("Openport Roboclaw lateral")
    rcf=Roboclaw("/dev/vujicicRC30", 115200)
    rcf.Open()
    print("Openport Roboclaw frontal")
    #print(f'Speed rear: {rear}')
    #print(f'Speed right: {right}')
    print(f'Speed left: {left}')
    rcf.SetM1VelocityPID(128,P,I,D,qpps)
    #rcf.SetM2VelocityPID(128,P,I,D,qpps)
    #rcl.SetM2VelocityPID(128,P,I,D,qpps)
    while not rospy.is_shutdown():
        rcf.SpeedM1(128,left)
        #rcf.SpeedM2(128,right)
        #rcl.SpeedM2(128,rear)
            
        rate.sleep()
        
if __name__ == "__main__":
    main()
