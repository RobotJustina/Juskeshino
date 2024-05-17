#!/usr/bin/env python3
import rospy
import math
import numpy
from juskeshino_tools.JuskeshinoNavigation import JuskeshinoNavigation
from juskeshino_tools.JuskeshinoHardware import JuskeshinoHardware
from juskeshino_tools.JuskeshinoSimpleTasks import JuskeshinoSimpleTasks
from juskeshino_tools.JuskeshinoVision import JuskeshinoVision
from juskeshino_tools.JuskeshinoHRI import JuskeshinoHRI
from juskeshino_tools.JuskeshinoManipulation import JuskeshinoManipulation
from juskeshino_tools.JuskeshinoKnowledge import JuskeshinoKnowledge
from sensor_msgs.msg import LaserScan

def doorIsOpened(timeout=100):
    msg = rospy.wait_for_message('/hardware/scan', LaserScan)
    if numpy.mean(msg.ranges[(int)(len(msg.ranges)*0.5 - 10):(int)(len(msg.ranges)*0.5 + 10)]) < 1.0:
        JuskeshinoHRI.say("The door is closed")
        print("The door is closed")
        return False
    else:
        JuskeshinoHRI.say("The door is opened")
        print("The door is opened")
        return True
def main():
    rospy.init_node("mike_test")
    rate = rospy.Rate(0.2)
    JuskeshinoNavigation.setNodeHandle()
    JuskeshinoHardware.setNodeHandle()
    JuskeshinoVision.setNodeHandle()
    print("Continue")
    JuskeshinoSimpleTasks.setNodeHandle()
    # JuskeshinoManipulation.setNodeHandle()
    # print("Continue")
    JuskeshinoHRI.setNodeHandle()
    print("Continue")
   
    JuskeshinoKnowledge.setNodeHandle()

    # #TODO
    # # --- START NAVIGATION
    # # 1. WAIT FOR THE DOOR TO OPEN

    # while not doorIsOpened():
    #     JuskeshinoHRI.say("Door is closed. Waiting for door to open")
    #     print("Door is closed. Waiting for door to open")
    
    # rate.sleep()
    # print("Ready for the next step")
    # JuskeshinoHRI.say("Ready for the next step")

    # # 2. NAVIGATE TO THE TESTING AREA (TABLE)
    
    # JuskeshinoNavigation.getClose("desk_justina", timeout=300)

    # 3. IDENTIFY TABLE
    response = JuskeshinoVision.findTableEdge()
    if response is None:
        JuskeshinoHRI.say("Ready for the next step")
        print("Cannot find the table")
    else:
        print(response)

    # 4. CLASSIFY OBJECTS
    # 5. TAKE OBJECT FROM THE TABLE TO THE CABINET
    # 6. IDENTIFY CATEGORY IN CABINET
    # 7. LEAVE THE OBJECT IN THE CORRECT CATEGORY

if __name__ == "__main__":
    main()


