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
    print("INITIALIZING GPSR 2024 TEST NODE BY ITZEL.........ฅ^•ﻌ•^ฅ")
    rospy.init_node("gpsr_test")
    rate = rospy.Rate(10)

    # Se subcribe a los servicios y topicos necesarios para manipulacion, navegacion,vision, etc...
    JuskeshinoNavigation.setNodeHandle()
    JuskeshinoVision.setNodeHandle()
    JuskeshinoHardware.setNodeHandle()
    JuskeshinoSimpleTasks.setNodeHandle()
    JuskeshinoHRI.setNodeHandle()
    JuskeshinoManipulation.setNodeHandle()
    JuskeshinoKnowledge.setNodeHandle()

    print("ACT-PLN.-> GPSR_test 2024")    
    JuskeshinoHRI.say("I am ready for the GPSR test")



    return 
    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == "__main__":
    main()
