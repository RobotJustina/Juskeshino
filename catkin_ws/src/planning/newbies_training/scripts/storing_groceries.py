#!/usr/bin/env python3
import rospy
import math
import numpy
from juskeshino_tools.JuskeshinoNavigation import JuskeshinoNavigation
from juskeshino_tools.JuskeshinoHardware import JuskeshinoHardware
from juskeshino_tools.JuskeshinoSimpleTasks import JuskeshinoSimpleTasks
from juskeshino_tools.JuskeshinoVision import JuskeshinoVision
from juskeshino_tools.JuskeshinoHRI import JuskeshinoHRI
from juskeshino_tools.JuskeshinoManipulation import JuskeshinoManipulation
from juskeshino_tools.JuskeshinoKnowledge import JuskeshinoKnowledge
from sensor_msgs.msg import LaserScan
# TODO
    # 1. WAIT FOR THE DOOR TO OPEN         
    # 2. NAVIGATE TO THE TESTING AREA (TABLE)
    # 3. IDENTIFY TABLE
    # 4. CLASSIFY OBJECTS
    # 5. TAKE OBJECT FROM THE TABLE TO THE CABINET
    # 6. IDENTIFY CATEGORY IN CABINET
    # 7. LEAVE THE OBJECT IN THE CORRECT CATEGORY
   
class Wait_for_the_door(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                            outcomes=['succed', 'failed', 'counter'])
        self.counter = 0

    # The input and output data of a state is called 'userdata'
    def execute(self):
        self.counter += 1
        if self.counter==1:
            rospy.logwarn('\n--> STATE 1 <: Wait for the door opened')

            JuskeshinoHRI.say("Door is closed. Waiting for door to open")
            print("Door is closed. Waiting for door to open")
            rate.sleep()

        msg = rospy.wait_for_message('/hardware/scan', LaserScan)
        if numpy.mean(msg.ranges[(int)(len(msg.ranges)*0.5 - 10):(int)(len(msg.ranges)*0.5 + 10)]) < 1.0:
            JuskeshinoHRI.say("The door is closed")
            print("The door is closed")
            return 'failed'
        else:
            JuskeshinoHRI.say("The door is opened")
            print("The door is opened")
            return 'succed'
        self.counter = 0

class Navigate_to_table(smach.State):
    def __init__(self):
            smach.State.__init__(self, 
                                outcomes=['succed', 'failed', 'counter'])
            self.counter = 0

        # The input and output data of a state is called 'userdata'
        def execute(self):
            self.counter += 1
            if self.counter==1:
                rospy.logwarn('\n--> STATE 2 <: Reaching the table')
                JuskeshinoHRI.say(" I am moving to the table")
                response = JuskeshinoVision.findTableEdge()
                if response is None:
                    JuskeshinoHRI.say("Ready for the next step")
                    print("Cannot find the table")
                else:
                print(response)
           

def main():
    rospy.init_node("storing_groceries")
    rate = rospy.Rate(0.2)
    JuskeshinoNavigation.setNodeHandle()
    JuskeshinoHardware.setNodeHandle()
    JuskeshinoVision.setNodeHandle()
    JuskeshinoSimpleTasks.setNodeHandle()
    # JuskeshinoManipulation.setNodeHandle()
    # print("Continue")
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
        smach.StateMachine.add('WAIT_FOR_THE_DOOR', Wait_for_the_door(), 
        transitions={'failed':'WAIT_FOR_THE_DOOR', 
                     'succed':''})
        smach.StateMachine.add('NAVIGATE_TO_TABLE', Navigate_to_table(), 
        transitions={'failed':'NAVIGATE_TO_TABLE', 
                     'succed':'END'})                     
        
  
    # Execute SMACH plan
    outcome = sm.execute()
if __name__ == "__main__":
    main()


