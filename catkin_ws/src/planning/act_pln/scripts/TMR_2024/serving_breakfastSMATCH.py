#!/usr/bin/env python3
import rospy
import rospkg
import time
import numpy as np
import math
import yaml
import smach_ros
from juskeshino_tools.JuskeshinoNavigation import JuskeshinoNavigation
from juskeshino_tools.JuskeshinoVision import JuskeshinoVision
from juskeshino_tools.JuskeshinoHardware import JuskeshinoHardware
from juskeshino_tools.JuskeshinoSimpleTasks import JuskeshinoSimpleTasks
from juskeshino_tools.JuskeshinoHRI import JuskeshinoHRI
from juskeshino_tools.JuskeshinoManipulation import JuskeshinoManipulation
from juskeshino_tools.JuskeshinoKnowledge import JuskeshinoKnowledge

# Left arm
HOME              = [0,0,0,0,0,0,0]
PREPARE           = [-0.8, 0.2, 0.0, 1.55, 0.0, 1.24, 0.0]#[-0.69, 0.2, 0.0, 1.55, 0.0, 1.16, 0.0]
PREPARE_TOP_GRIP  = [-1.25, 0.3, 0, 2.4, 0, 0.7,0]
PREPARE_SERVING   = [0.91, 0.4, -0.5, 1.45, 0, 0.16, 0.5]
SERVING           = [0.91, 0.4, -0.5, 1.45, 0, 0.16, -1.6]
LEAVE_CEREAL      = [0.54, 0.28, -0.13, 1.45, 0, 0, 0]
LEAVE_MILK        = [0.44, 0.18, -0.03, 1.45, 0, 0, 0]
LEAVE_BOWL        = [0.6,  0.6, -0.8, 1.7, 0, 0.2, 0]

# RIght arm
PREPARE_RA    = [-0.8, -0.1, 0.0, 1.3, 1.3,0, 0.0]

OBJECTS_TABLE = "desk_justina"
EAT_TABLE     = "desk_takeshi" 

# Gripper_aperture
GRIP_MILK   = 0.3
GRIP_BOWL   = -0.1
GRIP_CEREAL = 0.0


def serving_breakfast(object):
    print("PREPARE TOP")
    JuskeshinoHRI.say("Prepare arm")
    time.sleep(0.5)
    JuskeshinoHardware.moveLeftArmWithTrajectory(PREPARE_TOP_GRIP, 10)
    JuskeshinoHRI.say("Prepare serving")
    time.sleep(0.5)
    JuskeshinoHardware.moveLeftArmWithTrajectory(PREPARE_SERVING, 10)
    
    #JuskeshinoNavigation.moveDist(0.3, 7)      # mueve la base adelante con el brazo levantado y extendido
    if object =="milk": 
        JuskeshinoHRI.say("Serving milk")
    else: 
        JuskeshinoHRI.say("Serving cereal")
    time.sleep(0.5)
    JuskeshinoHardware.moveLeftArmWithTrajectory(SERVING, 10)
    JuskeshinoHRI.say("Prepare serving")
    time.sleep(0.5)
    JuskeshinoHardware.moveLeftArmWithTrajectory(PREPARE_SERVING, 10)
    time.sleep(0.5)

    if object =="milk":
        JuskeshinoHRI.say("leave milk")
        time.sleep(0.5)
        JuskeshinoHardware.moveLeftArmWithTrajectory(LEAVE_MILK, 10)
        #JuskeshinoNavigation.moveDist(0.3, 7)
        JuskeshinoHardware.moveLeftGripper(0.7, 2.0)
        
    else:
        JuskeshinoHRI.say("move right")
        time.sleep(0.5)
        JuskeshinoNavigation.moveLateral(0.22, 10)
        JuskeshinoHRI.say("leave cereal")
        time.sleep(0.5)
        JuskeshinoHardware.moveLeftArmWithTrajectory(LEAVE_CEREAL, 10)
        
        #JuskeshinoNavigation.moveDist(0.3, 7)
        #time.sleep(0.2)#
        JuskeshinoHardware.moveLeftGripper(0.7, 2.0)


def approach_to_table():
    JuskeshinoHRI.say("I will try to align with table")
    i = 0
    while i <4:
        if (not JuskeshinoSimpleTasks.alignWithTable()):
            JuskeshinoHRI.say("Cannot align with table")
            time.sleep(0.3)
            JuskeshinoNavigation.moveDist(0.10, 10)
        
        else:
            JuskeshinoHRI.say("Align with table")
            break
        i = i+1


def points_actual_to_points_target(point_in, f_actual, f_target):
    global listener
    point_msg = PointStamped()  
    point_msg.header.frame_id = f_actual   # frame de origen
    point_msg.header.stamp = rospy.Time() # la ultima transformacion
    point_msg.point.x = point_in[0]
    point_msg.point.y = point_in[1]
    point_msg.point.z = point_in[2]

    listener.waitForTransform(f_actual, f_target, rospy.Time(), rospy.Duration())
    point_target_frame = listener.transformPoint(f_target, point_msg)
    new_point = point_target_frame.point
    return [ new_point.x , new_point.y , new_point.z ]

def modify_location(pose_obj, location):  # position_obj is in frame 'base_link'
    pose_obj = pose_actual_to_pose_target(pose_obj , "base_link" , "map")
    pos_obj.position.x


def categorize_object(obj_id):      # Recibe un string que indica el nombre del objeto encontrado

    print("In categories")
    try:
        f = open('Juskeshino/catkin_ws/src/config_files/groceries_classification_rm.yaml' ,'r')
        data = yaml.safe_load(f)
    except:
        data = {}
        print("KnownLocations.->Cannot load locations from file ")

    for category, items in data.items():
        if obj_id.lower() in [item.lower() for item in items]:
            rospy.loginfo(f"--->>> Detected {obj_id} matches an item in the '{category}' category.")
            # Do something with the matched object
            return category  
    return False









class Initial(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succed','failed'])
        # Se subcribe a los servicios necesarios para manipulacion, navegacion,vision, etc...
        JuskeshinoManipulation.setNodeHandle()
        JuskeshinoNavigation.setNodeHandle()
        JuskeshinoVision.setNodeHandle()
        JuskeshinoHardware.setNodeHandle()
        JuskeshinoSimpleTasks.setNodeHandle()
        JuskeshinoHRI.setNodeHandle()
        JuskeshinoKnowledge.setNodeHandle()
        self.tries = 0


    def execute(self, userdata):
        print('\n> STATE <: INITIAL')
        self.tries += 1
        print(f'Try {self.tries}')
        









# --------------------------------------------------
# Entry point
if __name__ == '__main__':
    print("Justina STATE MACHINE...")
    sm = smach.StateMachine(outcomes=['END'])
    sis = smach_ros.IntrospectionServer(
        'SMACH_VIEW_SERVER', sm, '/SM_SB')
    sis.start()

    with sm:

        smach.StateMachine.add("INITIAL", Initial(),              
                               transitions={'failed':'INITIAL', 'succ':'WAIT_DOOR_OPENED'})

        smach.StateMachine.add("FINAL", Final(),
                               transitions={'failed':'INITIAL', 'succ':'END'})
                        


    outcome = sm.execute()




        


