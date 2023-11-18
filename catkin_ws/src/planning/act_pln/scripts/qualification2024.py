#!/usr/bin/env python
import rospy
import geometry_msgs
import vision_msgs
from juskeshino_tools.JuskeshinoNavigation import JuskeshinoNavigation
from juskeshino_tools.JuskeshinoVision import JuskeshinoVision
from juskeshino_tools.JuskeshinoHardware import JuskeshinoHardware
from juskeshino_tools.JuskeshinoSimpleTasks import JuskeshinoSimpleTasks
from juskeshino_tools.JuskeshinoHRI import JuskeshinoHRI

def main():
    print("INITIALIZING QUALIFICATION 2024 TEST BY MARCOSOFT...")
    rospy.init_node("act_pln")
    rate = rospy.Rate(10)

    JuskeshinoNavigation.setNodeHandle()
    JuskeshinoVision.setNodeHandle()
    JuskeshinoHardware.setNodeHandle()
    JuskeshinoSimpleTasks.setNodeHandle()
    JuskeshinoHRI.setNodeHandle()

    
    if not JuskeshinoSimpleTasks.waitForTheDoorToBeOpen(10):
        print("Door never opened")
        return 
    if not JuskeshinoNavigation.getCloseXYA(3.2, 5.6, -1.57, 30):
        print("Cannot get close to goal position")
    if not JuskeshinoHardware.moveHead(0,-1, 5):
        print("Cannot move head")
    if not JuskeshinoSimpleTasks.alignWithTable():
        print("Cannot align with table")
    print("Sending goal traj")
    JuskeshinoHardware.moveLeftArmWithTrajectory([-1.2, 0.2, 0, 1.6, 0, 1.1, 0], 10)
    print("Goal traj reached")
    [objs, img] = JuskeshinoVision.detectAndRecognizeObjects()
    print(len(objs))
    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == "__main__":
    main()
