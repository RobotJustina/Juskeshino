#!/usr/bin/env python3
import rospy
import yaml
import numpy
import smach
import time
import smach_ros
from juskeshino_tools.JuskeshinoNavigation import JuskeshinoNavigation
from juskeshino_tools.JuskeshinoHardware import JuskeshinoHardware
from juskeshino_tools.JuskeshinoSimpleTasks import JuskeshinoSimpleTasks
from juskeshino_tools.JuskeshinoVision import JuskeshinoVision
from juskeshino_tools.JuskeshinoHRI import JuskeshinoHRI
from juskeshino_tools.JuskeshinoManipulation import JuskeshinoManipulation
from juskeshino_tools.JuskeshinoKnowledge import JuskeshinoKnowledge
from sensor_msgs.msg import LaserScan
#TO DO :
# microphone
# table height

DESK = "table"
SIMUL_DESK = 'simul_desk'
TOP_SHELF=[1.28, 0.04, 0.0, 2.45 , 0.0, -1.2, 0.0]
MIDDLE_SHELF=[1.28, 0.0, 0.0, 2.15, 0.0, -1.2, 0.0]
LOW_SHELF=[0.31, 0.1, -0.1, 0.35, 0.0, 1.16, 0.0]
PREPARE_GRIP  = [-0.69, 0.2, 0, 1.55, 0, 1.16, 0]
HOME=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
HOLD_OBJ = [0.38, 0.19, -0.01, 1.57, 0 , 0.25, 0.0 ]
GET_CLOSE_TO_TABLE = 0.4
TABLE_TORSO_HEIGHT = 0.12

def matching_objects(obj):
    category = ''
    thinness = None
    graspable = None
    obj_yaml = '/home/mike/Juskeshino/catkin_ws/src/config_files/groceries_classification_test.yaml'
    try:
        f = open(obj_yaml,'r')
        data = yaml.safe_load(f)
    except:
        data = {}
        print("KnownLocations.->Cannot load locations from file " + obj_yaml)
    for categ, items in data.items():
        for item in items:
            for key, value in item.items():
                if key==obj:
                    thinness = value['thin']
                    graspable = value['graspable']
                    obj_name = key
                    category = categ

    return category, thinness, graspable, obj_name

class RecognizeObjects(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                            outcomes=['succed', 'failed','tries'],
                            output_keys=['object_output','object'])
        self.tries = 0

        # The input and output data of a state is called 'userdata'
    def execute(self,userdata):
        self.tries += 1
        
        # CONTINUE ATTEMPTS OR KILL
        if self.tries < 20:
            rospy.logwarn('\n--> STATE 4 <: Recognizing objects')
            JuskeshinoHRI.say("I will start to recognize the objects in the table.")
            JuskeshinoHardware.moveHead(0,-0.8, 5)
            recog_objects, img = JuskeshinoVision.detectAndRecognizeObjects()

            if recog_objects is not None:        
                for obj in recog_objects:
                    obj=recog_objects[0]
                    print(f"Object of interest is {obj.id}")
                    x,y,z = obj.pose.position.x, obj.pose.position.y, obj.pose.position.z
                    x,y,z = JuskeshinoSimpleTasks.transformPoint(x,y,z, "shoulders_left_link", "base_link")
                    print(f"The centroid of {obj.id} is x:{x} y:{y} z:{z}")
                    obj_oriented = JuskeshinoVision.getObjectOrientation(obj)
                    userdata.object_output=[x,y,z]
                    userdata.object=obj_oriented
                    print('Object found: ', obj_oriented.id, obj_oriented.pose, obj_oriented.object_state, obj_oriented.size, obj_oriented.graspable)
                    return 'succed'
            
            JuskeshinoHRI.say('I could not detect objects, I will try again')
            return 'tries'    
        
        print('After a lot of tries, I could not detect objects')
        return 'failed'
    
class Classification(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                            outcomes=['succed', 'failed'],
                            input_keys=['object_output','object'])
        self.tries = 0

    # The input and output data of a state is called 'userdata'
    def execute(self,userdata):
        self.tries += 1
        if self.tries < 20:    
            rospy.logwarn('\n--> STATE 5 <: Matching object with its classification')
            obj=userdata.object
            category, thinness, graspable, _ = matching_objects(obj.id)
            if category:
                prompt = "I found" + obj.id + " which is part of the category " + category
                JuskeshinoHRI.say(prompt)
                return'succed'
        return 'failed'
    
class AlignWithObject(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                            outcomes=['succed', 'failed'],
                            input_keys=['object_output','object'])
        self.tries = 0

        # The input and output data of a state is called 'userdata'
    def execute(self,userdata):
        self.tries += 1
        if self.tries < 10:
            rospy.logwarn('\n--> STATE 6 <: I am going to align with the object')
            obj=userdata.object
            JuskeshinoHRI.say("I am ready to pick the ",obj.id)
            JuskeshinoSimpleTasks.handling_location_la(obj.pose.position)
            return'succed'
        return 'failed'

class GiveMeDObject(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succed', 'failed']
                             ,input_keys=['object'])
        
        # The input and output data of a state is called 'userdata'
    def execute(self,userdata):
        rospy.logwarn('\n--> STATE <: GIVE ME THE OBJECT')
        obj=userdata.object
        JuskeshinoHRI.say((f"Please, help me. Put the {obj.id} in my hand and wait for the gripper closes "))
        JuskeshinoHardware.moveLeftArmWithTrajectory(PREPARE_GRIP, 10)
        JuskeshinoHRI.say("I am ready to pick the ",obj.id )
        rospy.sleep(2)
        success=JuskeshinoManipulation.dynamic_grasp_left_arm()
        rospy.sleep(0.5)
        if success:
            JuskeshinoHRI.say("Thank you")
            return 'succed'
        return 'failed'

class GraspObject(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                            outcomes=['succed', 'failed', 'help'],
                            input_keys=['object_output','object'])
        self.tries = 0

        # The input and output data of a state is called 'userdata'
    def execute(self,userdata):
        self.tries += 1
        if self.tries <6:
            rospy.logwarn('\n--> STATE 7 <: Picking up the target object, attempt: ' + str(self.tries))
            # GET OBJECT LOCATION AND CALCULATE TRAJECTORY
            obj=userdata.object
            x,y,z=userdata.object_output
            #x,y,z = JuskeshinoSimpleTasks.transformPoint(x,y,z, "shoulders_left_link", "base_link")
            JuskeshinoHRI.say("I am ready to pick the "+ obj.id)
            JuskeshinoHardware.moveTorso(TABLE_TORSO_HEIGHT , timeout = 5.0)
            rospy.sleep(1)
            JuskeshinoHardware.moveLeftArmWithTrajectory(PREPARE_GRIP, 10)
            JuskeshinoHardware.moveLeftGripper(0.7, 100.0)

            manip_method = rospy.get_param("~manip_method")
            if manip_method == "best":
                # EXECUTE TRAJECTORY
                [response, success] = JuskeshinoManipulation.GripLa(obj)
            else:
                response = JuskeshinoManipulation.laIk([x,y,z, 0,-1.5,0])
                if response is None or response is False:
                    success = False
                else:
                    success = True
            if success:
                JuskeshinoHRI.say("Object found correctly")
                JuskeshinoHardware.moveLeftArmWithTrajectory(response.articular_trajectory,10)
                success=JuskeshinoManipulation.dynamic_grasp_left_arm()
                JuskeshinoHardware.moveLeftArmWithTrajectory(HOLD_OBJ, 10)
                JuskeshinoHRI.say("Verifying...")
                JuskeshinoHardware.moveLeftArmWithTrajectory(PREPARE_GRIP, 10)
                rospy.sleep(0.5)
                if not success:
                    JuskeshinoHRI.say("I couldn't grasp the " + obj.id )
                    return 'help'
                else:
                    JuskeshinoHRI.say("I took correctly the " + obj.id )
                    return 'succed'
                
            else:
                print("No possible poses found")
                return 'help'
            
        return 'failed'

class FailedGrasp(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                            outcomes=['succed', 'failed', 'help'],
                            input_keys=['object_output','object'])
        self.tries = 0

        # The input and output data of a state is called 'userdata'
    def execute(self,userdata):
        self.tries += 1
        if self.tries<6:
            obj=userdata.object
            JuskeshinoHRI.say("Object found")
            [response, success] = JuskeshinoManipulation.GripLa(obj)
            JuskeshinoHardware.moveLeftGripper(0.9, 100.0)
            # JuskeshinoHardware.moveTorso(0.14 , 5.0)
            JuskeshinoHardware.moveLeftArmWithTrajectory(response.q,10)
            print("Closing gripper")
            success=JuskeshinoManipulation.dynamic_grasp_left_arm()
            print (success)
            JuskeshinoHardware.moveLeftArmWithTrajectory(HOLD_OBJ, 9)
            JuskeshinoHRI.say("Verifying...")
            JuskeshinoHardware.moveLeftArmWithTrajectory(PREPARE_GRIP, 10)
            rospy.sleep(2) 
            if not success:
                JuskeshinoHRI.say(f"Please, help me. Put the {obj.id} in my hand until the gripper closes ")
                
                return 'help'
            else:
                JuskeshinoHRI.say("I took correctly the "+ obj.id )
                return 'succed'
        return 'failed'

def main():
    rospy.init_node("storing_groceries")
    rate = rospy.Rate(0.2)
    JuskeshinoNavigation.setNodeHandle()
    JuskeshinoHardware.setNodeHandle()
    JuskeshinoVision.setNodeHandle()
    JuskeshinoSimpleTasks.setNodeHandle()
    JuskeshinoManipulation.setNodeHandle()
    JuskeshinoHRI.setNodeHandle()
    JuskeshinoKnowledge.setNodeHandle()
         
    #-------------------CREATING SMACH state machine------------------------------

    rospy.loginfo('Starting state machine...')

    #   The outcome in the initialization of the state machine 'END' represents the 
    #    possible end point or result of the state machine's execution
    sm = smach.StateMachine(outcomes=['END'])

    # For viewing state machine (ROS)
    sis = smach_ros.IntrospectionServer('SMACH_VIEW_SERVER', sm, '/SM_STORING')
    sis.start()

    with sm:
        # Add states to the container
        smach.StateMachine.add('RECOGNIZE_OBJ',RecognizeObjects(), 
        transitions={'failed':'END',
                     'succed':'CLASSIFICATION', 
                     'tries':'RECOGNIZE_OBJ'})
        
        smach.StateMachine.add('CLASSIFICATION',Classification(), 
        transitions={'failed':'RECOGNIZE_OBJ',
                     'succed':'ALIGNE_WITH_OBJ'})

        smach.StateMachine.add('ALIGNE_WITH_OBJ',AlignWithObject(), 
        transitions={'failed':'TRANSPORT_OBJECT',
                     'succed':'GRASP_OBJ'})

        smach.StateMachine.add('GIVE_ME_D_OBJ',GiveMeDObject(), 
        transitions={'failed':'GIVE_ME_D_OBJ',
                     'succed':'TRANSPORT_OBJECT'})
        
        smach.StateMachine.add('GRASP_OBJ',GraspObject(), 
        transitions={'failed':'FAILED_GRASP',
                     'help': 'GIVE_ME_D_OBJ',
                     'succed':'TRANSPORT_OBJECT'})
        
        smach.StateMachine.add('FAILED_GRASP',FailedGrasp(), 
        transitions={'failed':'GIVE_ME_D_OBJ',
                     'succed':'TRANSPORT_OBJECT',
                     'help': 'GIVE_ME_D_OBJ'})
                     
    # Execute SMACH plan
    outcome = sm.execute()
    
if __name__ == "__main__":
    main()