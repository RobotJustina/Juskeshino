#!/usr/bin/env python

from roboclaw_3 import Roboclaw
import time 

ADDRESS = 0x80
roboclaw = Roboclaw("/dev/ttyACM0", 38400)
roboclaw.Open()
P = 0x00000000
I = 0x00000000
D = 0x00000000

roboclaw.SetM1VelocityPID(ADDRESS,P,I,D,10000)
roboclaw.SetM2VelocityPID(ADDRESS,P,I,D,10000)

roboclaw.SpeedM1(ADDRESS, 1000)
roboclaw.SpeedM2(ADDRESS, 1000)
time.sleep(5.0)
roboclaw.SpeedM1(ADDRESS, 0)
roboclaw.SpeedM2(ADDRESS, 0)




    
