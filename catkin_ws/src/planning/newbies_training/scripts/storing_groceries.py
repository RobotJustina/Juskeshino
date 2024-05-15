#!/usr/bin/env python3
import rospy
import math
from juskeshino_tools.JuskeshinoNavigation import JuskeshinoNavigation
from juskeshino_tools.JuskeshinoHardware import JuskeshinoHardware
from juskeshino_tools.JuskeshinoSimpleTasks import JuskeshinoSimpleTasks
from juskeshino_tools.JuskeshinoVision import JuskeshinoVision
from juskeshino_tools.JuskeshinoHRI import JuskeshinoHRI
from juskeshino_tools.JuskeshinoManipulation import JuskeshinoManipulation
from juskeshino_tools.JuskeshinoKnowledge import JuskeshinoKnowledge

def main():
    rospy.init_node("mike_test")
    rate = rospy.Rate(1)
    JuskeshinoNavigation.setNodeHandle()
    JuskeshinoHardware.setNodeHandle()
    JuskeshinoVision.setNodeHandle()
    JuskeshinoSimpleTasks.setNodeHandle()
    JuskeshinoHRI.setNodeHandle()
    JuskeshinoManipulation.setNodeHandle()
    JuskeshinoKnowledge.setNodeHandle()

    #TODO
    # --- START NAVIGATION
    # 1. WAIT FOR THE DOOR TO OPEN
    # 2. NAVIGATE TO THE TESTING AREA (TABLE)
    # 3. IDENTIFY TABLE
    # 4. CLASSIFY OBJECTS
    # 5. TAKE OBJECT FROM THE TABLE TO THE CABINET
    # 6. IDENTIFY CATEGORY IN CABINET
    # 7. LEAVE THE OBJECT IN THE CORRECT CATEGORY

    JuskeshinoHRI.say("I will start moving")
    JuskeshinoNavigation.moveDist(0.1, 7000)
    first = [math.pi/2, 0, 0, 0, 0, 0, 0]
    JuskeshinoHardware.moveLeftArmWithTrajectory(first, 10)
    rate.sleep()
    first = [0, 0, 0, 0, 0, 0, 0]
    JuskeshinoHardware.moveLeftArmWithTrajectory(first, 10)
    JuskeshinoHRI.say("bye")

if __name__ == "__main__":
    main()