#!/usr/bin/env python

import time
from roboclaw_3 import Roboclaw

ADDRESS = 0x80
vel = 127
roboclaw = Roboclaw("/dev/justinaRC15", 38400)
roboclaw.Open()

def mover_motor():
    
        roboclaw.ForwardM1(ADDRESS,vel)
        time.sleep(1.0)
        roboclaw.BackwardM1(ADDRESS,vel)
        time.sleep(1.0)
        roboclaw.ForwardM1(ADDRESS, 0)
        
        roboclaw.ForwardM2(ADDRESS,vel)
        time.sleep(1.0)
        roboclaw.BackwardM2(ADDRESS,vel)
        time.sleep(1.0) 
        roboclaw.ForwardM2(ADDRESS, 0)
                
if __name__ == '__main__':
    mover_motor()
