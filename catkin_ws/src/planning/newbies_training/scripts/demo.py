#!/usr/bin/env python3
import rospy
import rospkg
import time
from juskeshino_tools.JuskeshinoNavigation import JuskeshinoNavigation
from juskeshino_tools.JuskeshinoVision import JuskeshinoVision
from juskeshino_tools.JuskeshinoHardware import JuskeshinoHardware
from juskeshino_tools.JuskeshinoSimpleTasks import JuskeshinoSimpleTasks
from juskeshino_tools.JuskeshinoHRI import JuskeshinoHRI
from juskeshino_tools.JuskeshinoManipulation import JuskeshinoManipulation
from juskeshino_tools.JuskeshinoKnowledge import JuskeshinoKnowledge

def main():
    rospy.init_node("demo")
    rate = rospy.Rate(10)
    rospack = rospkg.RosPack()
    JuskeshinoNavigation.setNodeHandle()
    JuskeshinoVision.setNodeHandle()
    JuskeshinoHardware.setNodeHandle()
    #JuskeshinoSimpleTasks.setNodeHandle()
    JuskeshinoHRI.setNodeHandle()
    #JuskeshinoManipulation.setNodeHandle()
    #JuskeshinoKnowledge.setNodeHandle()
    JuskeshinoHRI.say("hello")
    #JuskeshinoNavigation.moveDist(1.5, 7)
    JuskeshinoNavigation.getCloseXYA(0,0,0,10)
    JuskeshinoNavigation.getCloseXYA(5.46, 2.15, 1.5, 10)
    JuskeshinoHRI.say("bye")

if __name__ == "__main__":
    main()
