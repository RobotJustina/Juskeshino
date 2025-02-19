#!/usr/bin/env python

from roboclaw_3 import Roboclaw

ADDRESS = 0x80

roboclaw = Roboclaw("/dev/ttyACM0", 38400)
roboclaw.Open()

encoder1 = roboclaw.ReadEncM1(ADDRESS)
encoder2 = roboclaw.ReadEncM2(ADDRESS)
print("encoders:", encoder1, encoder2)

    
