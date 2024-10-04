#!/usr/bin/env python
import rospy
from roboclaw_3 import Roboclaw
import signal 
import sys
rcl=None
rcf=None
def signal_handler(sig, frame):
    rospy.loginfo("parando los motores")
    try: 
        reset_motors()
    except: 
        print("can't reset motors")
    rospy.signal_shutdown("Test ended")
    sys.exit(0)     

def reset_motors():
    global rcf,rcl, rear,front,left, P, I, D
    front=0
    rear=0
    left=0
    rcf.SpeedM1(128,0)
    rcf.SpeedM2(128,0)
    rcl.SpeedM2(128,0)
    rcf.SetM1VelocityPID(128,P,I,D,0)
    rcf.SetM2VelocityPID(128,P,I,D,0)
    rcl.SetM2VelocityPID(128,P,I,D,0)
    rospy.loginfo("Motores parados")
          
def main():
    global rcf,rcl, rear, front, left, P, I, D
    rospy.init_node('PIDTest')
    signal.signal(signal.SIGINT, signal_handler)
    rate=rospy.Rate(10)
    front=int(rospy.get_param("~front", 0))
    rear=int(rospy.get_param("~rear", 0))
    left=int(rospy.get_param("~left", 0))
    P=int(rospy.get_param("~P", 0))
    I=int(rospy.get_param("~I", 0))
    D=int(rospy.get_param("~D", 0))
    print("Try to connect roboclaws")
    
    try: 
        rcl=Roboclaw("/dev/vujicicRC15", 115200)
        rcl.Open()
        print("Openport Roboclaw lateral")
    except: 
        print("can't connect vujicicRC15")
    try: 
        rcf=Roboclaw("/dev/vujicicRC30", 115200)
        rcf.Open()
        print("Openport Roboclaw frontal")
    except: 
        print("can't connect vujicicRC30")
        
    print(f'Speed Rcalwf M1: {front}')
    print(f'Speed Rcalwf M2: {rear}')
    print(f'Speed Rcalwl M2: {left}')
    
    try: 
        rcf.SetM1VelocityPID(128,P,I,D,front)
        print("PID Front set")
    except: 
        print("can't set Front PID")
        
    
    try: 
        rcf.SetM2VelocityPID(128,P,I,D,rear)
        print("PID Rear set")
    except: 
        print("can't set Rear PID")
        
    try: 
        rcl.SetM1VelocityPID(128,P,I,D,left)
        print("PID Left set")
    except: 
        print("can't set Left PID")
        
    while not rospy.is_shutdown():
    
        try: 
            rcf.SpeedM1(128,front)
            print("front vel Set")
        except: 
            print("Can't set front vel")
        
        try: 
            rcf.SpeedM2(128,rear)
            print("rear vel Set")
        except: 
            print("Can't set rear vel")
        
        try: 
            rcl.SpeedM2(128,left)
            print("left vel Set")
        except: 
            print("Can't set left vel")
            
        rate.sleep()
        
if __name__ == "__main__":
    main()
