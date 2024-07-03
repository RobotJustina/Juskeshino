#!/usr/bin/env python3
import rospy
import yaml
import numpy
import smach
import time
import smach_ros
from juskeshino_tools.JuskeshinoNavigation import JuskeshinoNavigation
from juskeshino_tools.JuskeshinoHardware import JuskeshinoHardware
from juskeshino_tools.JuskeshinoSimpleTasks import JuskeshinoSimpleTasks
from juskeshino_tools.JuskeshinoVision import JuskeshinoVision
from juskeshino_tools.JuskeshinoHRI import JuskeshinoHRI
from juskeshino_tools.JuskeshinoManipulation import JuskeshinoManipulation
from juskeshino_tools.JuskeshinoKnowledge import JuskeshinoKnowledge
from sensor_msgs.msg import LaserScan

PREPARE_GRIP  = [-0.69, 0.2, 0, 1.55, 0, 1.16, 0]

def main():
    rospy.init_node("storing_groceries")
    rate = rospy.Rate(0.2)
    JuskeshinoNavigation.setNodeHandle()
    JuskeshinoHardware.setNodeHandle()
    JuskeshinoVision.setNodeHandle()
    JuskeshinoSimpleTasks.setNodeHandle()
    JuskeshinoManipulation.setNodeHandle()
    JuskeshinoHRI.setNodeHandle()
    JuskeshinoKnowledge.setNodeHandle()
         
    JuskeshinoHRI.say("Moving arm to prepare")
    JuskeshinoHardware.moveLeftArmWithTrajectory(PREPARE_GRIP, 10)
    time.sleep(3)
    JuskeshinoHRI.say("I am going to grasp the object")
    ret = JuskeshinoManipulation.dynamic_grasp_left_arm()
    if ret:
        JuskeshinoHRI.say("Object grasped succesfully")
    else:
        JuskeshinoHRI.say("Failed to graps object")




if __name__ == "__main__":
    exit(main())
