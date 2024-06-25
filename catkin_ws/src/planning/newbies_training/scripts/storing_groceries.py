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
# Veify if the object was succesfully graspped
# State for asking someone to take the object if point above is True **
# Not print all the categories, only the one the target category
# State for align with cabinet **
# State for leave the object **

DESK = "desk_justina"
SIMUL_DESK = 'simul_desk'
PREPARE_GRIP  = [-1.3, 0.2, 0, 2.2, 0, 1.24, 0]
# PREPARE_GRIP  = [-0.8, 0.2, 0, 1.55, 0, 1.24, 0]
PREPARE_TOP_GRIP=[-1.25, 0.3, 0, 2.4, 0, 0.7,0] #TINY OBJ
HOLD_OBJ = [0.38, 0.19, -0.01, 1.57, 0 , 0.25, 0.0 ]

def matching_objects(obj):
    obj_yaml =rospy.get_param("~categories")
    try:
        f = open(obj_yaml,'r')
        data = yaml.safe_load(f)
    except:
        data = {}
        print("KnownLocations.->Cannot load locations from file " + obj_yaml)
    for category, items in data.items():
        if obj.lower() in [item.lower() for item in items]:
            rospy.loginfo(f"--->>> Detected {obj} matches an item in the '{category}' category.")
            # Do something with the matched object
            return category  
    return False
# TODO
    # 1. WAIT FOR THE DOOR TO OPEN         
    # 2. NAVIGATE TO THE TESTING AREA (TABLE)
    # 3. IDENTIFY TABLE
    # 4. CLASSIFY OBJECTS
    # 5. TAKE OBJECT FROM THE TABLE TO THE CABINET
    # 6. IDENTIFY CATEGORY IN CABINET
    # 7. LEAVE THE OBJECT IN THE CORRECT CATEGORY
                #    response = JuskeshinoVision.findTableEdge()
                # if response is None:
                #     JuskeshinoHRI.say("Ready for the next step")
                #     print("Cannot find the table")
                # else:
                # print(response)
class WaitForTheDoor(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                            outcomes=['succed','failed'])
        self.tries = 0

    # The input and output data of a state is called 'userdata'
    def execute(self,userdata):
        self.tries += 1
        if self.tries<15:
            rospy.logwarn('\n--> STATE 1 <: Wait for the door opened')

            #JuskeshinoHRI.say("Door is closed. Waiting for door to open")
            print("Waiting for door")
            rospy.sleep(2.5)
        msg = rospy.wait_for_message('/hardware/scan', LaserScan)
        numb=numpy.mean(msg.ranges[(int)(len(msg.ranges)*0.5 - 10):(int)(len(msg.ranges)*0.5 + 10)]) 
        print (numb)
        if numpy.mean(msg.ranges[(int)(len(msg.ranges)*0.5 - 10):(int)(len(msg.ranges)*0.5 + 10)]) < 1.0:
            JuskeshinoHRI.say("The door is closed")
            print("The door is closed")
            self.tries = 0
            print(msg)
            return 'failed'
        JuskeshinoHRI.say("The door is open")
        rospy.sleep(2.5)
        print("The door is opened")
        return 'succed'

class NavigateToTable(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                            outcomes=['succed', 'failed'])
        self.tries = 0

        # The input and output data of a state is called 'userdata'
    def execute(self,userdata):
        self.tries += 1
        if self.tries<20:
            rospy.logwarn('\n--> STATE 2 <: Reaching the table')
            JuskeshinoHRI.say(" I am moving to the table")
            JuskeshinoHardware.moveHead(0,-1, 5)
            JuskeshinoNavigation.getClose(DESK, 120)#real
            time.sleep(0.5)
            #JuskeshinoNavigation.getClose('simul_desk', 120) #simul
            
            return 'succed'
        return 'failed'
            #JuskeshinoNavigation.
class FindTable(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                            outcomes=['succed', 'failed','tries'])
        self.tries = 0
        # The input and output data of a state is called 'userdata'
    def execute(self,userdata):
        self.tries += 1
        if self.tries<10:
            JuskeshinoHRI.say(" I am trying to localize the table")
            rospy.logwarn('\n--> STATE 3 <: Finding table')
            #Findin bounding table
            if(not JuskeshinoHardware.moveHead(0, -1, 100.0)):
                JuskeshinoHardware.moveHead(0, -1, 100.0)
            #response = JuskeshinoVision.findTableEdge()
            time.sleep(0.5)
            JuskeshinoNavigation.moveDist(0.15, timeout=5)

            return'succed'
        return'tries'



class RecognizeObjects(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                            outcomes=['succed', 'failed','tries'],
                            output_keys=['object_output','object'])
        self.tries = 0

        # The input and output data of a state is called 'userdata'
    def execute(self,userdata):
        self.tries += 1
        if self.tries < 20:
            rospy.logwarn('\n--> STATE 4 <: Recognizing objects')
            JuskeshinoHRI.say("Looking for an object")
            recog_objects, img = JuskeshinoVision.detectAndRecognizeObjects()
            rospy.sleep(2.5)
            i=0
            if recog_objects is not None:        
                #object_id=target_object.id
                for obj in recog_objects:
                    obj=recog_objects [i]
                    #i =+ 1
                    print(obj.id)
                    print(obj.pose.position)
                    print(obj.header.frame_id)
                    x,y,z = obj.pose.position.x, obj.pose.position.y, obj.pose.position.z
                    x,y,z = JuskeshinoSimpleTasks.transformPoint(x,y,z, "shoulders_left_link", "base_link")
                    print(x,y,z)
                    obj_oriented = JuskeshinoVision.getObjectOrientation(obj)
                    userdata.object_output=[x,y,z]
                    userdata.object=obj_oriented
                    #JuskeshinoHRI.say("I found: " + obj_oriented.id )
                    print('Object found: ', obj_oriented.id, obj_oriented.pose, obj_oriented.object_state, obj_oriented.size, obj_oriented.graspable)
                    return 'succed'
            
            print('Cannot detect objects, I will try again...')
            JuskeshinoHRI.say('Cannot detect objects, I will try again...')
            return 'failed'    
        
        print('After a lot of tries, I could not detect objects')
        return 'tries'
    
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
            category=matching_objects(obj.id)
            if category:
                prompt = "I found" + obj.id + " which is part of the category " + category
                JuskeshinoHRI.say(prompt)
                print("---------------------I found:", obj.id, "which is part of the category: ", category)
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
        if self.tries < 4:
            
            rospy.logwarn('\n--> STATE 6 <: I am going to align with the object')
            obj=userdata.object
            JuskeshinoHRI.say("I am ready to pick the ",obj.id)
            JuskeshinoSimpleTasks.handling_location_la(obj.pose.position)
            return'succed'
        print("Cannot align robot with object :(")
        return 'failed'

class GraspObject(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                            outcomes=['succed', 'failed'],
                            input_keys=['object_output','object'])
        self.tries = 0

        # The input and output data of a state is called 'userdata'
    def execute(self,userdata):
        self.tries += 1
        if self.tries == 1:
            rospy.logwarn('\n--> STATE 7 <: Picking up the target object, attempt: ' + str(self.tries))
            obj=userdata.object
            JuskeshinoHRI.say("I am going to pick the "+ obj.id)
            
            print('Trying to pick: '+ obj.id)
            x,y,z=userdata.object_output
            JuskeshinoHRI.say("Moving arm to prepare")
            print("Moving arm to prepare")
            #JuskeshinoHardware.moveTorso(0.09 , 5.0)
            if obj.category == 'CUBIC':
                JuskeshinoHardware.moveLeftArmWithTrajectory(PREPARE_TOP_GRIP, 10)
            else:
                JuskeshinoHardware.moveLeftArmWithTrajectory(PREPARE_GRIP, 10)
            JuskeshinoHardware.moveLeftGripper(0.7, 100.0)
            print("Detected object : " + str([obj.id, obj.category, obj.object_state, obj.pose.position]))   
            print('------------>>>',x,y,z)                    
            [response, success] = JuskeshinoManipulation.GripLa(obj)
            if success:
                JuskeshinoHRI.say("Object found correctly")
                print("Object position: ",obj.pose.position)
                #JuskeshinoHardware.moveTorso(0.14 , 5.0)
                JuskeshinoHardware.moveLeftArmWithTrajectory(response.articular_trajectory,10)
                print("Closing gripper")
                if obj.category == 'CUBIC':
                    JuskeshinoHardware.moveLeftGripper(0.0 , 3.0)
                else:
                    JuskeshinoHardware.moveLeftGripper(0.05 , 3.0) 
                JuskeshinoHardware.moveLeftArmWithTrajectory(HOLD_OBJ, 10)
                JuskeshinoHardware.moveLeftArmWithTrajectory(PREPARE_GRIP, 10)
                time.sleep(0.5)     
                JuskeshinoNavigation.getClose(DESK, 10)
                recog_objects, img = JuskeshinoVision.detectAndRecognizeObjects()
                for objc in recog_objects:
                    if obj and objc:
                        JuskeshinoHRI.say("I couldn't grasp the object, a human is going to take the "+ obj.id )
                        time.sleep(2)
                        return 'succed'
                    else:
                        return 'succed'
                
        
        print("No possible poses found")
        return 'failed'
#0,-1.5,0
class FailedGrasp(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                            outcomes=['succed', 'failed'],
                            input_keys=['object_output','object'])
        self.tries = 0

        # The input and output data of a state is called 'userdata'
    def execute(self,userdata):
        self.tries += 1
        if self.tries<4:
            objc=userdata.object
            JuskeshinoHRI.say("Object found")
            [response, success] = JuskeshinoManipulation.GripLa(objc)
            JuskeshinoHardware.moveLeftGripper(0.9, 100.0)
            JuskeshinoHardware.moveLeftArmWithTrajectory(response.q,10)
            print("Closing gripper")
            JuskeshinoHardware.moveLeftGripper(0.05 , 3.0) 
            JuskeshinoHardware.moveLeftArmWithTrajectory(PREPARE_GRIP, 10)
            time.sleep(0.5)
            recog_objects, img = JuskeshinoVision.detectAndRecognizeObjects()
            for obj in recog_objects:
                if obj and objc:
                    JuskeshinoHRI.say("I couldn't grasp the object, a human is going to take the "+ obj.id )
                    return 'failed'
            return 'succed'
        
    
class TransportObject(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                            outcomes=['succed', 'failed'])
        self.tries = 0

        # The input and output data of a state is called 'userdata'
    def execute(self,userdata):
        self.tries += 1
        if self.tries<20:
            rospy.logwarn('\n--> STATE 8 <: Transporting object to cabinet')
            JuskeshinoHRI.say(" I am moving to the cabinet")
            rospy.sleep(3) 
            JuskeshinoNavigation.getClose('scan_cabinet', 120)
            
            return 'succed'
        
class ScanCabinet(smach.State):
    def __init__(self):
        smach.State.__init__(
            self, outcomes=['succed', 'failed', 'tries'],
                    input_keys=['object_output','object'],
                    output_keys=['object_shelf'])
        self.tries = 0

    def execute(self, userdata):
        # request = segmentation_server.request_class()
        self.tries += 1
        if self.tries < 10:
            JuskeshinoHardware.moveHead(0,-0.35, 5)
            rospy.sleep(2.5)
            print('Scanning shelf')
            tar_obj=userdata.object
            rospy.logwarn('\n--> STATE 9 <: Scanning cabinet')
            JuskeshinoHRI.say("Scanning cabinet")
            recog_objects, img = JuskeshinoVision.detectAndRecognizeObjects()
            rospy.sleep(2.5)       
            #object_id=target_object.id
            if recog_objects is not None:
                for obj in recog_objects:
                    userdata.object_shelf=obj
                    print("------->",obj.id)
                    print(obj.pose.position)
                    category1=matching_objects(obj.id)
                    category2=matching_objects(tar_obj.id)
                    JuskeshinoHRI.say('Matching with shelf...')
                    print("*******",category1)
                    print(category2)
                    
                    for recog_obj in recog_objects:  # Iterate over all objects to find matching categories
                        recog_category = matching_objects(recog_obj.id)
                        if recog_category and category2 == recog_category:
                            category1 = category2  # Set category1 to match category2
                            obj = recog_obj  # Set obj to the matching object
                            #centroid maths
                            JuskeshinoSimpleTasks.handling_location_la(obj.pose.position)
                            userdata.object_shelf=obj
                            if obj.pose.position.z>1.3:
                                JuskeshinoHRI.say("The object is part of the top shelf")
                                print("The object is part of the top shelf")
                                return 'succed'
                                #variable shared with the next state giving q for left arm to leave the object 'object_shelf'
                            if 1.3 > obj.pose.position.z > 0.8:
                                JuskeshinoHRI.say("The object is part of the middle shelf")
                                return 'succed'
                                print("The object is part of the middle shelf")
                            if 0.8 > obj.pose.position.z > 0.2:
                                JuskeshinoHRI.say("The object is part of the low shelf")
                                print("The object is part of the low shelf")    
                                return 'succed'
                        else:
                            #Check for the position of each object and start from that 
                            print('Cannot match objects, I will try again...')
                            #JuskeshinoNavigation.moveDist(0.1, 10)
                            return 'failed'    
            
        print('I could not detect objects')
        return 'tries'
    #check the centroid of shelf obj and leave the object on the side
    #Align with shelf state

class LeaveObject(smach.State):
    def __init__(self):
        smach.State.__init__(
            self, outcomes=['succed', 'failed', 'tries'],
                    input_keys=['object_shelf'])
        self.tries = 0

    def execute(self, userdata):
        # request = segmentation_server.request_class()
        self.tries += 1
        if self.tries < 8:
            JuskeshinoHardware.moveHead(-0,35, 5)
            
            obj_shelf=userdata.object_shelf
            rospy.logwarn('\n--> STATE 10 <: Leaving object')
            JuskeshinoNavigation.moveDist(0.3, timeout=5)
            # JuskeshinoNavigation.getClose('SHELF', 10)
            JuskeshinoHRI.say("Leaving object")
            [response, success] = JuskeshinoManipulation.GripLa(obj_shelf)
            if success :
                print("Object position: ",obj_shelf.pose.position)
                #JuskeshinoHardware.moveTorso(0.14 , 5.0)
                JuskeshinoHardware.moveLeftArmWithTrajectory(response.articular_trajectory,10)
                print("Opening gripper")
                JuskeshinoHardware.moveLeftGripper(0.9, 100.0)
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
        smach.StateMachine.add('WAIT_FOR_THE_DOOR', WaitForTheDoor(), 
        transitions={'failed':'WAIT_FOR_THE_DOOR', 
                     'succed':'NAVIGATE_TO_TABLE'})

        smach.StateMachine.add('NAVIGATE_TO_TABLE', NavigateToTable(), 
        transitions={'failed':'NAVIGATE_TO_TABLE', 
                     'succed':'FIND_TABLE'})

        smach.StateMachine.add('FIND_TABLE',FindTable(), 
        transitions={'failed':'NAVIGATE_TO_TABLE', 
                     'succed':'RECOGNIZE_OBJ',
                     'tries' : 'FIND_TABLE'})   

        smach.StateMachine.add('RECOGNIZE_OBJ',RecognizeObjects(), 
        transitions={'failed':'FIND_TABLE',
                     'succed':'CLASSIFICATION', 
                     'tries':'RECOGNIZE_OBJ'})
        
        smach.StateMachine.add('CLASSIFICATION',Classification(), 
        transitions={'failed':'RECOGNIZE_OBJ',
                     'succed':'ALIGNE_WITH_OBJ'})

        smach.StateMachine.add('ALIGNE_WITH_OBJ',AlignWithObject(), 
        transitions={'failed':'FIND_TABLE',
                     'succed':'GRASP_OBJ'})

        smach.StateMachine.add('GRASP_OBJ',GraspObject(), 
        transitions={'failed':'FAILED_GRASP',
                     'succed':'TRANSPORT_OBJECT'})
        
        smach.StateMachine.add('FAILED_GRASP',FailedGrasp(), 
        transitions={'failed':'TRANSPORT_OBJECT',
                     'succed':'TRANSPORT_OBJECT'})
        
        smach.StateMachine.add('TRANSPORT_OBJECT',TransportObject(), 
        transitions={'failed':'NAVIGATE_TO_TABLE',
                     'succed':'SCAN_CABINET'})
  
        smach.StateMachine.add('SCAN_CABINET',ScanCabinet(), 
        transitions={'failed':'SCAN_CABINET',
                     'succed':'LEAVE_OBJ',
                     'tries':'SCAN_CABINET'})

        smach.StateMachine.add('LEAVE_OBJ',LeaveObject(), 
        transitions={'failed':'LEAVE_OBJ',
                     'succed':'END',
                     'tries':'LEAVE_OBJ'})
                     
    # Execute SMACH plan
    outcome = sm.execute()
    
if __name__ == "__main__":
    main()