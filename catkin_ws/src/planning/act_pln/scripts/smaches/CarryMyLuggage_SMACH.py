#!/usr/bin/env python3

# import rospy
# import time
# import numpy as np

from smach_utils_justina import *

from juskeshino_tools.JuskeshinoNavigation import JuskeshinoNavigation
from juskeshino_tools.JuskeshinoVision import JuskeshinoVision
from juskeshino_tools.JuskeshinoHardware import JuskeshinoHardware
from juskeshino_tools.JuskeshinoSimpleTasks import JuskeshinoSimpleTasks
from juskeshino_tools.JuskeshinoHRI import JuskeshinoHRI
from juskeshino_tools.JuskeshinoManipulation import JuskeshinoManipulation
from juskeshino_tools.JuskeshinoKnowledge import JuskeshinoKnowledge



class Initial(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed'],
                             output_keys = ['mode', 'l_arm_home'], input_keys=['bag', 'l_arm_home'] )
        self.tries = 0
        # topic/services subscriptions: manipulation, vision, navigation, else
        JuskeshinoNavigation.setNodeHandle()
        JuskeshinoVision.setNodeHandle()
        JuskeshinoHardware.setNodeHandle()
        JuskeshinoSimpleTasks.setNodeHandle()
        JuskeshinoHRI.setNodeHandle()
        #JuskeshinoManipulation.setNodeHandle()
        JuskeshinoKnowledge.setNodeHandle()

    def execute(self, userdata):
        print('\n> STATE <: INITIAL')
        print("INITIALIZING CARRY MY LUGGAGE 2024 .......ᐠ( ᐢ ᵕ ᐢ )ᐟ")
        self.tries += 1
        print(f'Try {self.tries}')

        # Neutral position
        userdata.l_arm_home = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        JuskeshinoHardware.moveLeftArmWithTrajectory(userdata.l_arm_home, 6)
        head.set_named_target('neutral')
        print('I am ready for carry my luggage task.')
        voice.talk('I am ready for carry my luggage task.')

        JuskeshinoVision.enableHumanPose(False)
        JuskeshinoHRI.enableLegFinder(False)
        return 'succ'


class GotoLivingRoom(smach.State):  
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed', 'tries'])
        self.tries = 0
        self.time_out = 30
        self.attempts = 2

    def execute(self, userdata):
        self.tries += 1
        if self.tries == 1:
            self.time_out = 30
            print("\n")
            rospy.logwarn('--> STATE <: Navigate to living room')
            voice.talk('Navigating to, living room')
        
        if self.tries == self.attempts + 1: 
            rospy.logerr('Navigation Failed, I can not reach the living room')
            voice.talk('Navigation Failed, I can not reach the living room')
            return 'failed'

        print(f'Try {self.tries} of {self.attempts} attempts')        
        res = omni_base.move_base(known_location='living_room', time_out=self.time_out)
        print("res:", res)
        
        if res == 3:  # Success
            self.tries = 0
            return 'succ'
        elif res == 1 or res == 8:
            if self.tries < self.attempts-1:
                rospy.logerr('Navigation Failed, retrying')
                voice.talk('Navigation Failed, retrying')
            self.time_out = 25
            return 'tries'
        else:
            return 'failed'


class FindHuman(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed', 'tries'])
        self.tries = 0
        self.catkin_disable = False

    def execute(self, userdata):
        self.tries += 1
        
        if self.tries == 1:
            print("\n")
            rospy.logwarn('--> STATE <: Find human')
            head.set_joint_values([0.0, -0.1])  # TODO: ensure head is up before scan 
            rospy.sleep(0.3)
            head.set_joint_values([0.0, -0.1])
            print("looking front o_o")

            if self.catkin_disable:
                JuskeshinoVision.enableHumanPose(True)
            else:  # Use catkin_extras version
                detect_human_to_tf()  # First scan is empty
        if self.tries == 2:
            print("looking right o_=")
            head.set_named_target('right')
            rospy.sleep(0.3)
            head.set_named_target('right')
        if self.tries == 3:
            print("looking right =_o")
            head.set_named_target('left')
            rospy.sleep(0.3)
            head.set_named_target('left')

        print(f'Try {self.tries}')
        print('Scanning for human, please do not move.')
        voice.talk('Scanning for human, please do not move.')

        if self.catkin_disable:
            p_h = JuskeshinoVision.pointingHand()
            h_detect = JuskeshinoVision.humanDetector()
            print("h_detect:\n", h_detect)
            print("p_h:", p_h)
        else:  # Use catkin_extras version
            human_pose = detect_human_to_tf()
            print("human_pose", human_pose)

            if human_pose:
                print("I find you, please stay there.")
                voice.talk("I find you, please stay there.")
                rospy.sleep(0.3)
                return 'succ'
            
        if self.tries >= 4:
            if self.catkin_disable:
                JuskeshinoVision.enableHumanPose(False)
            print("I can't find any human. Please go in front of me.")
            voice.talk("I can't find any human. Please go in front of me.")
            return 'failed'
        
        rospy.sleep(1.0)
        print("I can't find human, retrying.")
        voice.talk("I can't find human. retrying.")
        head.set_joint_values([0.0, -0.1])
        rospy.sleep(0.3)
        return 'tries'


class PointingBag(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed', 'tries'])
        self.tries = 0

    def execute(self, userdata):
        self.tries += 1
        
        if self.tries == 1:
            print("\n")
            rospy.logwarn('--> STATE <: Pointing Bag')

        
        print('Please point at the bag with your hand.')
        voice.talk('Please, point at the bag with your hand.')
        rospy.sleep(2.0)
        res = pointing_detect_server.call()

        if self.tries > 3:
            return 'failed'

        if (res.x_r + res.y_r) == 0 and (res.x_l + res.y_l) == 0:
            print('I did not find a pointing arm, I will try again')
            voice.talk('I did not find a pointing arm, I will try again')
            return 'tries'
        
        print('I see where you are pointing')
        voice.talk('I see where you are pointing')
        rospy.sleep(0.8)
        print(type(res))
        print("res.x_r, res.y_r", res.x_r, res.y_r)
        print("res.x_l, res.y_l", res.x_l, res.y_l)

        if res.x_r == -1: 
            tf_man.pub_static_tf(pos=[res.x_l, res.y_l,0], rot =[0,0,0,1], point_name='pointing_')
        else: 
            tf_man.pub_static_tf(pos=[res.x_r, res.y_r,0], rot =[0,0,0,1], point_name='pointing_')
        # TODO: Fix dancing bug
        ## First locate bag then move to bag


        head.turn_base_gaze(tf='pointing_', to_gaze='base_link') #'arm_flex_link'
        head.to_tf('pointing_')
        # There is now a "human TF with humans aprrox location(face)"
        return 'succ'


class AskForBag(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed', 'tries'])
        self.tries = 0
        self.offerSit = [0.3, 0.2, -0.4, 1.7, 0.0, -0.4, 1.5]
        self.getBag = [0.3, 0.2, -0.5, 1.7, 0.0, 0.3, 0.0, 0.4]
        self.carryBag = [-0.69, 0.2, 0.0, 1.55, 0.0, 1.16, 0.0]


    def execute(self, userdata):
        self.tries += 1
        
        if self.tries > 4:
            return 'failed'   
             
        if self.tries == 1:
            print("\n")
            rospy.logwarn('--> STATE <: Ask for Bag')
            JuskeshinoHardware.moveLeftArmWithTrajectory(self.offerSit, 6)
            print('Can you put the bag over my hand? Please')
            voice.talk('Can you put the bag over my hand?. Please')
            JuskeshinoHardware.moveLeftGripper(0, 2.0)
            JuskeshinoHardware.moveLeftArmWithTrajectory(self.getBag, 6)
        
        print('Tell me: Justina yes, when you put the bag')
        voice.talk('Tell me. Justina yes, when you put the bag')
        answer = JuskeshinoHRI.waitForNewSentence(6)
        print("voice: ", answer)
        if "YES" in answer:
            JuskeshinoHardware.moveLeftArmWithTrajectory(self.carryBag, 6)
            head.set_named_target('face_to_face')
            rospy.sleep(0.3)
            head.set_named_target('face_to_face')

            return 'succ'
        
        return 'tries'

# TODO: DEBUG ------>>>
class FindLegs(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed', 'tries'])
        self.tries = 0

    def execute(self, userdata):
        self.tries += 1  

        if self.tries > 6:  # TODO: many times
            print("I can't found you, I stop trying to follow you")
            voice.talk("I can't found you, I stop trying to follow you")
            print("Lets try again")
            voice.talk("Lets try again")
            self.tries = 0
            return 'failed'
        
        if self.tries == 1:
            print("\n")
            rospy.logwarn('--> STATE <: Find legs')
            JuskeshinoVision.enableHumanPose(True)
            #print("I will start to follow you. Please stand in front of me")
            #voice.talk("I will start to follow you. Please stand in front of me")
            
        human_detector = JuskeshinoVision.humanDetector()
        print("CML-PLN.-> human detector is :__", human_detector)
        if human_detector:
            print("I'm going to follow you, please say here is the car if we reached the final location")
            voice.talk("I'm going to follow you, please say. here is the car, if we reached the final location")
            self.tries = 0
            return 'succ'
        else:
            print("I can't found you, please stand in front of me")
            voice.talk("I can't found you, please stand in front of me")
    
        return 'tries'


class FolowwHuman(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed', 'tries'])
        self.tries = 0

    def execute(self, userdata):
        self.tries += 1  

        if self.tries == 1:
            print("\n")
            rospy.logwarn('--> STATE <: Follow human')
            JuskeshinoHRI.enableLegFinder(True)

        legs_found = JuskeshinoHRI.frontalLegsFound()
        print("CML-PLN.-> Legs in front?___", legs_found.data)
        time.sleep(1)

        # TODO: here move
        command_voice = JuskeshinoHRI.waitForNewSentence(10)
        print("command_voice: ", command_voice)
        if "CAR" in command_voice:
            JuskeshinoHRI.enableHumanFollower(False)
            return 'succ'        

        if(legs_found):
            print("ACT-PLN.-> found legs")
            print("ACT-PLN.-> HumanFollower enable")
            JuskeshinoHRI.enableHumanFollower(True)
        else:
            rospy.logwarn("ACT-PLN.-> Not found legs")
            print("I can't found you, please stand in front of me")
            voice.talk("I can't found you, please stand in front of me")
            JuskeshinoHRI.enableHumanFollower(False)
            return 'tries'

        return 'failed'


if __name__ == '__main__':
    print("Justina STATE MACHINE...")
    # State machine, final state "END"
    sm = smach.StateMachine(outcomes=['END'])
    # sm.userdata.clear = False
    sis = smach_ros.IntrospectionServer(
        'SMACH_VIEW_SERVER', sm, '/SM_RECEPTIONIST')
    sis.start()
    
    with sm:
        smach.StateMachine.add("INITIAL", Initial(), transitions={'failed':'INITIAL', 'succ':'GOTO_LIVING_ROOM'})#'GOTO_LIVING_ROOM'})

        smach.StateMachine.add("GOTO_LIVING_ROOM", GotoLivingRoom(),            
                               transitions={'failed':'FIND_HUMAN', 'tries':'GOTO_LIVING_ROOM', 'succ':'FIND_HUMAN'})
        
        smach.StateMachine.add("FIND_HUMAN", FindHuman(), 
                               transitions={'failed':'FIND_HUMAN', 'succ':'POINTING_BAG', 'tries':'FIND_HUMAN'})
        
        smach.StateMachine.add("POINTING_BAG", PointingBag(), 
                               transitions={'failed':'END', 'succ':'ASK_FOR_BAG', 'tries':'POINTING_BAG'})
        
        smach.StateMachine.add("ASK_FOR_BAG", AskForBag(), 
                               transitions={'failed':'END', 'succ':'FIND_LEGS', 'tries':'ASK_FOR_BAG'})
        
        smach.StateMachine.add("FIND_LEGS", FindLegs(), 
                               transitions={'failed':'FIND_LEGS', 'succ':'FOLLOW_HUMAN', 'tries':'FIND_LEGS'})

        smach.StateMachine.add("FOLLOW_HUMAN", FolowwHuman(), 
                               transitions={'failed':'FIND_LEGS', 'succ':'END', 'tries':'FOLLOW_HUMAN'})


    outcome = sm.execute()