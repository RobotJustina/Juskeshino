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
    print("INITIALIZING CARRY MY LUGGAGE 2024 TEST NODE BY ITZEL..............ヾ(๑╹◡╹)ﾉ")
    rospy.init_node("carry_my_luggage_test")
    rate = rospy.Rate(10)

    rospack = rospkg.RosPack()
    #locations_default = rospack.get_path("config_files") + "/known_locations_objects.yaml"
    locations_default = rospack.get_path("config_files") + "/known_locations_simul.yaml"
    locations_file = rospy.get_param("~locations", locations_default)

    # Se subcribe a los servicios necesarios para manipulacion, navegacion,vision, etc...
    JuskeshinoNavigation.setNodeHandle()
    JuskeshinoVision.setNodeHandle()
    JuskeshinoHardware.setNodeHandle()
    JuskeshinoSimpleTasks.setNodeHandle()
    JuskeshinoHRI.setNodeHandle()
    JuskeshinoManipulation.setNodeHandle()
    JuskeshinoKnowledge.setNodeHandle()
    JuskeshinoKnowledge.loadLocations(locations_file)


    JuskeshinoHRI.say("I am ready for the carry my luggage test")
    JuskeshinoHRI.say("Please point at the bag that you want me to carry.")
    # Find bag
    JuskeshinoHRI.say("Tell me, robot yes, when you are pointing at the bag")
    # Enable speech recognition
    JuskeshinoHRI.getLastRecognizedSentence()
    




    return 
    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == "__main__":
    main()
