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


from segmentation.srv import *
global segmentation_server
segmentation_server = rospy.ServiceProxy('/segment', Segmentation)

def main():
    rospy.init_node("test_shelf")
    JuskeshinoNavigation.setNodeHandle()
    JuskeshinoHardware.setNodeHandle()
    JuskeshinoVision.setNodeHandle()
    JuskeshinoSimpleTasks.setNodeHandle()
    JuskeshinoManipulation.setNodeHandle()
    JuskeshinoHRI.setNodeHandle()
    JuskeshinoKnowledge.setNodeHandle()

    JuskeshinoHRI.say('Scanning shelf')
    print('Scanning shelf')
    recog_objects, img = JuskeshinoVision.detectAndRecognizeObjects()
    rospy.sleep(2.5)
    if recog_objects is not None:        
        num_of_objects = 0
        for obj in recog_objects:
            num_of_objects+=1
            print(f"{num_of_objects}-> ID: {obj.id}")
            print(f"- Category: {obj.category}")
            print(f"- Confidene: {obj.confidence}")
            print(f"- Size: {obj.size}")
            print(f"X: {obj.x}, Y: {obj.y}")
            print(f"Width: {obj.width}, Height: {obj.height}")
            print(f"Graspable: {obj.graspable}")
            
    #Check for the position of each object and start from that 
    else:
        print('Cannot detect objects, I will try again...')
        JuskeshinoHRI.say('Cannot detect objects, I will try again...')
    

if __name__ == "__main__":
    exit(main())