#!/usr/bin/env python3
import rospy
import rospkg
import time
import numpy as np
from juskeshino_tools.JuskeshinoNavigation import JuskeshinoNavigation
from juskeshino_tools.JuskeshinoVision import JuskeshinoVision
from juskeshino_tools.JuskeshinoHardware import JuskeshinoHardware
from juskeshino_tools.JuskeshinoSimpleTasks import JuskeshinoSimpleTasks
from juskeshino_tools.JuskeshinoHRI import JuskeshinoHRI
from juskeshino_tools.JuskeshinoManipulation import JuskeshinoManipulation
from juskeshino_tools.JuskeshinoKnowledge import JuskeshinoKnowledge
import rospkg
rospack = rospkg.RosPack()

from gpsr_modules import gpsr_info
from gpsr_modules.clips_functions import load_clips_file, load_objects_facts, load_furniture_facts, load_door_facts, load_human_facts, load_robot_facts, load_room_facts, load_arm_facts
from gpsr_modules.general_states import MyObject

####DICTIONARIES
##Objects
#( room (type Room) (name nil) (room nil) (zone nil) (center 0 0 0) (num 1) )
objects_info = gpsr_info.load_objects_info(location="robocup")

#( item (type Objects) (name nil) (room nil) (zone nil) (attributes nil) (lower base) (upper nothing) (possession nobody) (num 1) )
room_info = gpsr_info.load_room_info(location="robocup")

#( item (type Human) (name nil) (room nil) (zone nil) (attributes nil) (objs nil) (num 1) )
human_info = gpsr_info.load_human_info(location="robocup")

#( item (type Robot) (name %s) (zone %s) (pose %f %f %f) )
robot_info = [
    MyObject(name='robot', synonyms=['robot', 'robot'], location='front entrance', pose=[0.0, 0.0, 0.0])
]

#( Arm (name %s) (num %d) )
arm_info = [
    MyObject(name='left', synonyms=['arm', 'arm'], num=1)
]
####


def main():
    print("INITIALIZING GPSR TEST In a Tenshily manner...")
    rospy.init_node("gpsr")
    rate = rospy.Rate(10)

    # Se subcribe a los servicios necesarios para manipulacion, navegacion,vision, etc...
    # JuskeshinoNavigation.setNodeHandle()
    # JuskeshinoVision.setNodeHandle()
    # JuskeshinoHardware.setNodeHandle()
    # JuskeshinoSimpleTasks.setNodeHandle()
    # JuskeshinoHRI.setNodeHandle()
    # JuskeshinoManipulation.setNodeHandle()
    # JuskeshinoKnowledge.setNodeHandle()

    #Start CLIPS
    _file = rospack.get_path('ros_clips') + '/../data/clp/gpsr_nofacts_files.dat'
    load_clips_file(_file)
    
    #load facts
    load_objects_facts(objects_info, room_info)
    #load_furniture_facts(furniture_info, room_info)
    load_human_facts(human_info, room_info)
    load_room_facts(room_info)
    #load_door_facts(door_info)
    load_arm_facts(arm_info)
    
    #update and load robot facts
    #robot_info[0].pose = omni_base.pose()
    load_robot_facts(robot_info)
    print("CLIPS stuff loaded")
    
    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == "__main__":
    main()
