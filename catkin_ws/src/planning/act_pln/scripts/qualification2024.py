#!/usr/bin/env python
import rospy
import geometry_msgs
import vision_msgs
from juskeshino_tools.JuskeshinoNavigation import JuskeshinoNavigation
from juskeshino_tools.JuskeshinoVision import JuskeshinoVision
from juskeshino_tools.JuskeshinoHardware import JuskeshinoHardware
from juskeshino_tools.JuskeshinoSimpleTasks import JuskeshinoSimpleTasks

def main():
    print("INITIALIZING QUALIFICATION 2024 TEST BY MARCOSOFT...")
    rospy.init_node("act_pln")
    rate = rospy.Rate(10)

    JuskeshinoNavigation.setNodeHandle()
    JuskeshinoVision.setNodeHandle()
    JuskeshinoHardware.setNodeHandle()
    JuskeshinoSimpleTasks.setNodeHandle()

    if not JuskeshinoNavigation.getCloseXYA(5.5, 2.3, 1.57, 30000):
        print("Cannot get close to goal position")
    if not JuskeshinoHardware.moveHead(0,-1, 5000):
        print("Cannot move head")
    if not JuskeshinoSimpleTasks.alignWithTable():
        print("Cannot align with table")
    print("Sending goal traj")
    JuskeshinoHardware.moveLeftArmWithTrajectory([-1.2, 0.2, 0, 1.6, 0, 1.1, 0], 10000)
    print("Goal traj reached")
    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == "__main__":
    main()
