#!/usr/bin/env python

from roboclaw_3 import Roboclaw
import numpy as np
import time 
import sys


ADDRESS = 0x80
speed = 127
roboclaw = Roboclaw("/dev/justinaRC15", 38400)
roboclaw.Open()
iteraciones = 20

def calculate_qpps(duration=5.0):

    roboclaw.ForwardM1(ADDRESS, 0)
    roboclaw.ForwardM2(ADDRESS, 0)
    time.sleep(2.0)
    _,enc1_init,__ = roboclaw.ReadEncM1(ADDRESS)
    _,enc2_init,__ = roboclaw.ReadEncM2(ADDRESS)
    roboclaw.ForwardM1(ADDRESS, speed)
    roboclaw.ForwardM2(ADDRESS, speed)
    time.sleep(duration)
    _,enc1_end,__ = roboclaw.ReadEncM1(ADDRESS)
    _,enc2_end,__ = roboclaw.ReadEncM2(ADDRESS)
    roboclaw.ForwardM1(ADDRESS, 0)
    roboclaw.ForwardM2(ADDRESS, 0)
    qpps1= (enc1_end - enc1_init)/ duration
    qpps2= (enc2_end - enc2_init)/ duration
    print ("qpps1:", qpps1)
    print ("qpps2:", qpps2)
    return qpps1, qpps2


def get_mean_qpps(iterations, duration):
    qpps1s = []
    qpps2s = []
    for i in range(iterations):
        qpps1, qpps2 = calculate_qpps(duration)
        qpps1s.append(qpps1)
        qpps2s.append(qpps2)
    qpps1s = np.asarray(qpps1s)
    qpps2s = np.asarray(qpps2s)
    print("QPPS1:", np.mean(qpps1s), np.std(qpps1s))
    print("QPPS2:", np.mean(qpps2s), np.std(qpps2s))


if __name__ == "__main__":
    try:
        duration=float(sys.argv[1])
    except:
        duration=5.0

    get_mean_qpps(1, duration)

