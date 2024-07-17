#!/usr/bin/env python3
import rospy
from juskeshino_tools.JuskeshinoNavigation import JuskeshinoNavigation
from juskeshino_tools.JuskeshinoVision import JuskeshinoVision
from juskeshino_tools.JuskeshinoHardware import JuskeshinoHardware
from juskeshino_tools.JuskeshinoSimpleTasks import JuskeshinoSimpleTasks
from juskeshino_tools.JuskeshinoHRI import JuskeshinoHRI
from juskeshino_tools.JuskeshinoManipulation import JuskeshinoManipulation
from juskeshino_tools.JuskeshinoKnowledge import JuskeshinoKnowledge
from hri_msgs.msg import RecognizedSpeech
import grammar_checker

SM_INIT = 0
SM_WAIT_FOR_DOOR = 10
SM_WAITING_FOR_COMMAND = 20 
SM_WAIT_FOR_CONFIRMATION = 30
SM_EXECUTE = 40
SM_END = 1000

ACTION_NAVIGATE = 'navigate'
ACTION_TAKE = 'take'
ACTION_FIND_PERSON = 'find_person'
ACTION_FIND_GESTURE = 'find_gesture'
ACTION_FIND_CLOTHES = 'find_clothes'
ACTION_FOLLOW = 'follow'
ACTION_SAY = 'say'
ACTION_LEAVE_OBJECT = 'leave_object'

PREPARE_GRIP  = [-0.69, 0.2, 0, 1.55, 0, 1.16, 0]
HOME=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
TABLE_TORSO_HEIGHT = 0.12

def navigate(goal):
    goal = goal.lower().replace(" ", "_")
    goal_to_say = goal.lower().replace("_", " ")
    print("GPSR.->Navigating to ", goal)
    JuskeshinoHRI.startSay("I am goint to the " + goal_to_say)
    if not JuskeshinoNavigation.getClose(goal, 15):
        if not JuskeshinoNavigation.getClose(goal, 15):
            print("GPSR.->Cannot arrive to goal point ", goal)
        else:
            print("GPSR.->Arrived to goal point ", goal)
    else:
        print("GPSR.->Arrived to goal point ", goal)
    JuskeshinoHardware.moveHead(0,0,3)
    JuskeshinoHRI.say("I arrived to the " + goal_to_say)

def take(obj_name):
    print("GPRS.->Trying to take object: ", obj_name)
    obj_to_say = obj_name.lower()
    obj_name = obj_name.lower().replace(" ", "_")
    JuskeshinoHRI.say("I will try to find the " + obj_to_say) 
    result = JuskeshinoSimpleTasks.object_search_orientation(obj_name)
    if result is None:
        print("GPSR.->Cannot find object ", obj_name)
        JuskeshinoHRI.say("I am sorry. I cannot find the " + obj_to_say)
        return
    found_obj, img = result
    print("GPSR.->Found Object: ", found_obj.id, found_obj.pose)
    JuskeshinoHRI.say("I found the " + obj_to_say)
    JuskeshinoSimpleTasks.handling_location_la(found_obj.pose.position)

    x,y,z=found_obj.pose.position.x, found_obj.pose.position.y, found_obj.pose.position.z
    x,y,z = JuskeshinoSimpleTasks.transformPoint(x,y,z, "shoulders_left_link", "base_link")
    JuskeshinoHRI.say("I am ready to pick the "+ obj_to_say)
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
        success = not (response is None or response is False)
    if not success:
        print("GPSR.->Could not take the ", obj_name)
        JuskeshinoHRI.say("I am sorry. I could not take the " + obj_to_say)
        JuskeshinoHardware.moveLeftArmWithTrajectory(HOME, 10)
        return
    
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
    
def find_person(person_name):
    print("GPSR.->Trying to detect person: ", person_name)
    JuskeshinoSimpleTasks.findHumanAndApproach(60)

def find_gesture(gesture):
    print("GPSR.->Trying to detect the ", gesture)
    JuskeshinoHRI.say("I will try to find the " + gesture.replace("_", " "))
    JuskeshinoSimpleTasks.findHumanAndApproach(60)

def find_clothes(clothes):
    print("GPSR.->Trying to detect the ", clothes)
    JuskeshinoHRI.say("I will try to find the " + clothes.replace("_", " "))
    JuskeshinoSimpleTasks.findHumanAndApproach(60)

def follow(dest):
    no_destination = dest is None or dest == ""
    print("GPSR.->Starting following")
    JuskeshinoHRI.say("I will try to following you. Please stand in front of me")
    if not JuskeshinoHRI.waitForFrontalLegsFound(15):
        JuskeshinoHRI.say("Human, please stand closer in front of me")
        if not JuskeshinoHRI.waitForFrontalLegsFound(15):
            JuskeshinoHRI.say("Human, please stand closer in front of me")
            if not JuskeshinoHRI.waitForFrontalLegsFound(10):
                JuskeshinoHRI.say("Human, I could not found you. I am sorry.")
                JuskeshinoHRI.enableLegFinder(False)
                return
    JuskeshinoHRI.say("Human, I found you. Please say. Robot stop following me. When you want me to stop following you")
    JuskeshinoHRI.enableHumanFollower(True)
    JuskeshinoHRI.say("Human, you can walk")
    cmd = ""
    loop = rospy.Rate(10)
    max_attempts = 1500
    while not rospy.is_shutdown() and cmd == "":
        loop.sleep()
        
    

def say(text):
    print("GPSR.->Saying ", text)
    JuskeshinoHRI.say(text)

def leave_object():
    print("GPSR.->Leaving object")
    JuskeshinoHRI.say("I am going to leave the object")
    JuskeshinoHardware.moveLeftArm([0.38, 0.19, -0.01, 1.57, 0 , 0.25, 0.0 ])
    JuskeshinoHardware.moveLeftGripper(0.8, 5.0)
    JuskeshinoHardware.moveLeftGripper(0.1, 5.0)
    JuskeshinoHardware.moveLeftArm([0,0,0,0,0,0,0])
    
    
def main():
    print("INITIALIZING GPSR TEST Not in a Tenshily manner...")
    rospy.init_node("act_pln")
    rate = rospy.Rate(10)   
    
    # Se subcribe a los servicios necesarios para manipulacion, navegacion,vision, etc..
    JuskeshinoNavigation.setNodeHandle()
    JuskeshinoVision.setNodeHandle()
    JuskeshinoHardware.setNodeHandle()
    JuskeshinoSimpleTasks.setNodeHandle()
    JuskeshinoHRI.setNodeHandle()
    # JuskeshinoManipulation.setNodeHandle()
    # JuskeshinoKnowledge.setNodeHandle()
    # rospy.Subscriber("/hri/sp_rec/recognized", RecognizedSpeech, callback_sp_rec)
    state = SM_INIT
    current_action = 0
    hardwired_tasks = {ACTION_NAVIGATE: navigate,
                       ACTION_TAKE: take,
                       ACTION_FIND_PERSON: find_person,
                       ACTION_FIND_GESTURE: find_gesture,
                       ACTION_FIND_CLOTHES: find_clothes,
                       ACTION_FOLLOW: follow,
                       ACTION_SAY: say,
                       ACTION_LEAVE_OBJECT: }
    cmd = grammar_checker.Command()
    while not rospy.is_shutdown():
        if state == SM_INIT:
            print("GPSR.->Starting GPSR Test")
            state = SM_WAIT_FOR_DOOR
        elif state == SM_WAIT_FOR_DOOR:
            print("GPSR.->Waiting for the door to be opened. ")
            JuskeshinoHRI.say("I'm waiting for the door to be open")
            if JuskeshinoSimpleTasks.waitForTheDoorToBeOpen(30):
                print("GPSR.->Door opened detected")
                JuskeshinoHRI.say("I can see now that the door is open")
                JuskeshinoNavigation.moveDist(1.0, 5.0)
                navigate("start_position")
                state = SM_WAITING_FOR_COMMAND
        elif state == SM_WAITING_FOR_COMMAND:
            print("GPSR.->Waiting for command")
            JuskeshinoHRI.say("Please tell me what do you want me to do")
            JuskeshinoHRI.clearRecognizedSentences()
            cmd.actions = []
            cmd.sentence = JuskeshinoHRI.waitForNewSentence(15)
            cmdType = grammar_checker.getCommnandType(cmd)
            if cmd != '' and cmd is not None and cmdType is not None:
                print("GPSR.->Recognized command: ", cmd.sentence, "  of Type: ", cmdType)
                print("GPSR.->List of actions: ", cmd.actions)
                JuskeshinoHRI.say("Did you say")
                JuskeshinoHRI.say(cmd.sentence)
                JuskeshinoHRI.say("Please answer robot yes or robot no.")
                state = SM_WAIT_FOR_CONFIRMATION
        elif state == SM_WAIT_FOR_CONFIRMATION:
            print("GPSR.->Waiting for confirmation")
            sentence = JuskeshinoHRI.waitForNewSentence(10)
            if sentence == "ROBOT YES" or sentence == "JUSTINA YES":
                print("GPSR.-> Received confirmation: YES")
                JuskeshinoHRI.say("O K. I'm going to execute the command")
                current_action = 0
                state = SM_EXECUTE
            else:
                print("GPSR.->Received cancelation. ")
                JuskeshinoHRI.say("O K.")
                state = SM_WAITING_FOR_COMMAND

        elif state == SM_EXECUTE:
            print("Executing action number ", current_action)
            action, predicate = cmd.actions[current_action]
            hardwired_tasks[action](predicate)
            current_action += 1
            if current_action >= len(cmd.actions):
                state = SM_END
                

        elif state == SM_END:
            print("GPSR.->Command executed succesfully")
            JuskeshinoHRI.startSay("I have executed the command.")
            navigate('start_location')
            state = SM_INIT
        rate.sleep()


if __name__ == "__main__":
    main()
