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
                             output_keys = ['mode', 'l_arm_home', 'confirm_list', 'negation_list', 'speech_time'], 
                             input_keys=['bag', 'l_arm_home', 'confirm_list', 'negation_list', 'speech_time'] )
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

        global vosk_enable
        if vosk_enable:
            print("VOSK ENABLE -->")
            userdata.confirm_list = ['yes', 'robot yes', 'jack', 'juice', 'justina yes', 'yeah', 'correct', 'affirmative']
            userdata.negation_list = ['no', 'robot no','not','now','nope','justina no', 'incorrect', 'negative']
            userdata.speech_time = 6
            print("speech timeout: ", userdata.speech_time)
        else:
            userdata.confirm_list = ["YES", "YEAH", "ROBOT YES", "JUSTINA YES", "JUICE"]
            userdata.negation_list = ['NO', 'ROBOT NO','NOPE','JUSTINA NO']

        gram = userdata.confirm_list + userdata.negation_list

        print("** confirmation list: ") 
        print(userdata.confirm_list)  
        print("** negation list: ") 
        print(userdata.negation_list)

        # Neutral position
        if self.tries ==1:
            set_grammar(gram)
            userdata.l_arm_home = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            JuskeshinoHardware.moveLeftArmWithTrajectory(userdata.l_arm_home, 6)
            head.set_named_target('neutral')

        print('I am ready for carry my luggage task.')
        voice.talk('I am ready for carry my luggage task.')

        JuskeshinoVision.enableHumanPose(False)
        JuskeshinoHRI.enableLegFinder(False)
        return 'succ'


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
            human_pose = detect_human_to_tf(3)
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


        #head.turn_base_gaze(tf='pointing_', to_gaze='base_link') #'arm_flex_link'
        head.to_tf('pointing_')
        # There is now a "human TF with humans aprrox location(face)"
        return 'succ'


class GotoLivingRoom(smach.State):  
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed', 'tries'])
        self.tries = 0
        self.time_out = 30
        self.attempts = 2
        self.location = "pickup"   # TODO: Define place where to go  
        self.loc_name = self.location.replace('_', ' ')

    def execute(self, userdata):
        self.tries += 1
        if self.tries == 1:
            self.time_out = 30
            print("\n")
            rospy.logwarn('--> STATE <: Navigate to ' + self.loc_name)
            voice.talk('Navigating to ' + self.loc_name)
        
        if self.tries == self.attempts + 1: 
            rospy.logerr('Navigation Failed, I can not reach the ' + self.loc_name)
            voice.talk('Navigation Failed, I can not reach the ' + self.loc_name)
            return 'failed'

        print(f'Try {self.tries} of {self.attempts} attempts')        
        res = omni_base.move_base(known_location=self.location, time_out=self.time_out)
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

class AskForBag(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed', 'tries'],
                             input_keys=['speech_time', 'confirm_list'])
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
            head.set_named_target('neutral')
            print('Can you put the bag over my hand? Please')
            voice.talk('Can you put the bag over my hand?. Please')
            JuskeshinoHardware.moveLeftGripper(0, 2.0)
            JuskeshinoHardware.moveLeftArmWithTrajectory(self.getBag, 6)
        
        print('Tell me: Justina yes, when you put the bag')
        voice.talk('Tell me. Justina yes, when you put the bag')

        
        
        if vosk_enable:
                rospy.logwarn('Listening now (Cc')
                confirmation = get_keywords_speech(userdata.speech_time)
        else: 
            JuskeshinoHRI.getLastRecognizedSentence()
            rospy.sleep(0.3)
            confirmation = JuskeshinoHRI.waitForNewSentence(8) # 10 is to much?
        
        print("confirmation:", confirmation)
        if confirmation  in userdata.confirm_list:
            JuskeshinoHardware.moveLeftArmWithTrajectory(self.carryBag, 6)
            head.set_named_target('face_to_face')
            rospy.sleep(0.3)
            head.set_named_target('face_to_face')
            print("I will start to follow you. Please stand in front of me")
            voice.talk("I will start to follow you. Please stand in front of me")
            return 'succ'            
        else:
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
            JuskeshinoHRI.enableLegFinder(True)
            #print("I will start to follow you. Please stand in front of me")
            #voice.talk("I will start to follow you. Please stand in front of me")
            
        legs_found = JuskeshinoHRI.frontalLegsFound()
        print("CML-PLN.-> human detector is :__", legs_found.data)
        if legs_found.data:
            print("Please say here is the car if we reached the final location")
            voice.talk("Please say. here is the car, if we reached the final location")
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
            print("I follow you")
            voice.talk("I follow you")
        

        legs_found = JuskeshinoHRI.frontalLegsFound()
        print("CML-PLN.-> Legs in front?___", legs_found.data)
        time.sleep(1)       

        if(legs_found.data):
            print("ACT-PLN.-> found legs")
            print("ACT-PLN.-> HumanFollower enable")
            if self.tries % 7 == 0:
                print("I follow you")
                voice.talk("I follow you")
            

            command_voice = JuskeshinoHRI.getLastRecognizedSentence()
            print("--> command_voice: ", command_voice)
            if command_voice != None and "CAR" in command_voice:
                JuskeshinoHRI.enableHumanFollower(False)
                JuskeshinoHRI.enableLegFinder(False)
                return 'succ' 
                
            JuskeshinoHRI.enableHumanFollower(True)
            return 'tries'
        else:
            rospy.logwarn("ACT-PLN.-> Not found legs")
            print("I can't found you, please stand in front of me")
            voice.talk("I can't found you, please stand in front of me")
            JuskeshinoHRI.enableHumanFollower(False)
            JuskeshinoHRI.enableLegFinder(False)
            self.tries = 0
            return 'failed'


class AskArrive(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed', 'tries'])
        self.tries = 0
        self.attempts = 2
        
    def execute(self, userdata):
        self.tries += 1  

        if self.tries == 3:
            print('I am having trouble understanding you')
            voice.talk ('I am having trouble understanding you')
            return 'failed'

        if self.tries == 1:
            print("\n")
            rospy.logwarn('--> STATE <: Ask if we arrive human')
            print('Here is the place?')
            voice.talk ('Here is the place?')


        print(f'Try {self.tries} of {self.attempts} attempts')

        print('Tell me: Justina yes, if we arrived')
        voice.talk('Tell me. Justina yes, if we arrived')
        answer = JuskeshinoHRI.waitForNewSentence(6)
        print("voice: ", answer)
        if "YES" in answer:
            return "succ"
        if "NO" in answer:
            return "failed"
        else:
            return "tries"


class DeliverBag(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed', 'tries'])
        self.tries = 0
        self.getBag = [0.3, 0.2, -0.5, 1.7, 0.0, 0.3, 0.0, 0.4]
        self.l_arm_home = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    def execute(self, userdata):
        self.tries += 1  

        if self.tries == 1:
            print("\n")
            rospy.logwarn('--> STATE <: Deliver bag')
            JuskeshinoHardware.moveLeftArmWithTrajectory(self.getBag, 6)
            rospy.sleep(1.0)
            print('You can take the bag')
            voice.talk('You can take the bag')

            rospy.sleep(5.0)
            print('Task completed, thanks for watching')
            voice.talk("Task completed, thanks for watching")
            JuskeshinoHardware.moveLeftArmWithTrajectory(self.l_arm_home, 6)
            rospy.sleep(3.0)
            return 'succ'


if __name__ == '__main__':
    print("Justina STATE MACHINE...")
    # State machine, final state "END"
    sm = smach.StateMachine(outcomes=['END'])
    # sm.userdata.clear = False
    sis = smach_ros.IntrospectionServer(
        'SMACH_VIEW_SERVER', sm, '/SM_RECEPTIONIST')
    sis.start()
    
    with sm:
        smach.StateMachine.add("INITIAL", Initial(), transitions={'failed':'INITIAL', 'succ':'FIND_HUMAN'})#'GOTO_LIVING_ROOM'})

        
        smach.StateMachine.add("FIND_HUMAN", FindHuman(), 
                               transitions={'failed':'FIND_HUMAN', 'succ':'POINTING_BAG', 'tries':'FIND_HUMAN'})
        
        smach.StateMachine.add("POINTING_BAG", PointingBag(), 
                               transitions={'failed':'END', 'succ':'GOTO_LIVING_ROOM', 'tries':'POINTING_BAG'})
        
        smach.StateMachine.add("GOTO_LIVING_ROOM", GotoLivingRoom(),            
                               transitions={'failed':'ASK_FOR_BAG', 'tries':'GOTO_LIVING_ROOM', 'succ':'ASK_FOR_BAG'})
        
        smach.StateMachine.add("ASK_FOR_BAG", AskForBag(), 
                               transitions={'failed':'END', 'succ':'FIND_LEGS', 'tries':'ASK_FOR_BAG'})
        
        smach.StateMachine.add("FIND_LEGS", FindLegs(), 
                               transitions={'failed':'FIND_LEGS', 'succ':'FOLLOW_HUMAN', 'tries':'FIND_LEGS'})

        smach.StateMachine.add("FOLLOW_HUMAN", FolowwHuman(), 
                               transitions={'failed':'FIND_LEGS', 'succ':'ASK_ARRIVE', 'tries':'FOLLOW_HUMAN'})

        smach.StateMachine.add("ASK_ARRIVE", AskArrive(), 
                               transitions={'failed':'FIND_LEGS', 'succ':'DELIVER_BAG', 'tries':'ASK_ARRIVE'})
        
        smach.StateMachine.add("DELIVER_BAG", DeliverBag(), 
                               transitions={'failed':'DELIVER_BAG', 'succ':'END', 'tries':'DELIVER_BAG'})

    outcome = sm.execute()