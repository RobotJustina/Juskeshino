#!/usr/bin/env python
import rospy
import geometry_msgs
import vision_msgs
from juskeshino_tools.JuskeshinoNavigation import JuskeshinoNavigation

def main():
    print("INITIALIZING QUALIFICATION 2024 TEST BY MARCOSOFT...")
    rospy.init_node("act_pln")
    rate = rospy.Rate(10)

    JuskeshinoNavigation.setNodeHandle(None)
    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == "__main__":
    main()
