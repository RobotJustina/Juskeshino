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
SM_WAIT_FOR_CONFIRMATION = 10
SM_EXECUTE = 20
SM_END = 1000

ACTION_NAVIGATE = 'navigate'
ACTION_TAKE = 'take'
ACTION_FIND_PERSON = 'find_person'
ACTION_FIND_GESTURE = 'find_gesture'
ACTION_FIND_CLOTHES = 'find_clothes'
ACTION_FOLLOW = 'follow'
ACTION_SAY = 'say'

def navigate(goal):
    goal = goal.lower().replace(" ", "_")
    print("GPSR.->Navigating to ", goal)
    JuskeshinoHRI.startSay("I am goint to the " + goal)
    if not JuskeshinoNavigation.getClose(goal, 15):
        if not JuskeshinoNavigation.getClose(goal, 15):
            print("GPSR.->Cannot arrive to goal point ", goal)
        else:
            print("GPSR.->Arrived to goal point ", goal)
    else:
        print("GPSR.->Arrived to goal point ", goal)
    JuskeshinoHRI.say("I arrived to the " + goal)

def take(obj_name):
    print("GPRS.->Trying to take object: ", obj_name)
    
def find_person(person_name):
    print("GPSR.->Trying to detect person: ", person_name)

def find_gesture(gesture):
    print("GPSR.->Trying to detect the ", gesture)

def find_clothes(clothes):
    print("GPSR.->Trying to detect the ", clothes)

def follow(dest):
    print("GPSR.->Starting following")

def say(text):
    print("GPSR.->Saying ", text)
    JuskeshinoHRI.say(text)
    
def main():
    print("INITIALIZING GPSR TEST Not in a Tenshily manner...")
    rospy.init_node("act_pln")
    rate = rospy.Rate(10)   
    
    # Se subcribe a los servicios necesarios para manipulacion, navegacion,vision, etc..
    JuskeshinoNavigation.setNodeHandle()
    # JuskeshinoVision.setNodeHandle()
    # JuskeshinoHardware.setNodeHandle()
    # JuskeshinoSimpleTasks.setNodeHandle()
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
                       ACTION_SAY: say}
    cmd = grammar_checker.Command()
    while not rospy.is_shutdown():
        if state == SM_INIT:
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
                state = SM_INIT

        elif state == SM_EXECUTE:
            print("Executing action number ", current_action)
            action, predicate = cmd.actions[current_action]
            hardwired_tasks[action](predicate)
            current_action += 1
            if current_action >= len(cmd.actions):
                state = SM_END
                

        elif state == SM_END:
            print("GPSR.->Command executed succesfully")
            JuskeshinoHRI.startSay("I have executed the command. I am going to the start position")
            navigate('start')
            state = SM_INIT
        rate.sleep()


if __name__ == "__main__":
    main()
