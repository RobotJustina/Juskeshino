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

    print(JuskeshinoHardware.moveHead(0,-1, 5000))
    lines = JuskeshinoVision.findTableEdge()
    print(lines)
    JuskeshinoSimpleTasks.alignWithTable()
    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == "__main__":
    main()
