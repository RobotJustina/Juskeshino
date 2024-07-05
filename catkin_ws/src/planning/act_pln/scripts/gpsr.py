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
SM_END = 1000

def main():
    print("INITIALIZING GPSR TEST Not in a Tenshily manner...")
    cmd = grammar_checker.Command()
    #cmd.sentence = "GIVE IT TO ANGEL IN THE KITCHEN"
    #success = grammar_checker.deliverObjToNameAtBeac(cmd)
    # cmd.sentence = "GIVE IT TO THE WAVING PERSON IN THE KITCHEN"
    # success = grammar_checker.deliverObjToPrsInRoom(cmd)
    # cmd.sentence = "DELIVER IT TO ME"
    # success = grammar_checker.deliverObjToMe(cmd)
    # cmd.sentence = "PLACE IT ON THE BEDSIDE TABLE"
    # success = grammar_checker.placeObjOnPlcmt(cmd)
    # cmd.sentence = "GUIDE THEM TO THE BED"
    # success = grammar_checker.guidePrsToBeacon(cmd)
    #cmd.sentence = "FOLLOW THEM TO THE KITCHEN"
    #success = grammar_checker.foundPers(cmd)
    # cmd.sentence = "FIND THE PRINGLES AND TAKE IT AND PLACE IT ON THE BEDSIDE TABLE"
    # success = grammar_checker.findObj(cmd)
    # cmd.sentence = "MEET ANGEL AND FOLLOW THEM TO THE KITCHEN"
    # success = grammar_checker.meetName(cmd)
    # cmd.sentence = "FIND THE WAVING PERSON AND FOLLOW THEM TO THE KITCHEN"
    # success = grammar_checker.findPrs(cmd)
    # print(success)
    # print(cmd.sentence, cmd.actions)
    
    # return 
    rospy.init_node("gpsr")
    rate = rospy.Rate(10)   
    
    # Se subcribe a los servicios necesarios para manipulacion, navegacion,vision, etc..
    # JuskeshinoNavigation.setNodeHandle()
    # JuskeshinoVision.setNodeHandle()
    # JuskeshinoHardware.setNodeHandle()
    # JuskeshinoSimpleTasks.setNodeHandle()
    JuskeshinoHRI.setNodeHandle()
    # JuskeshinoManipulation.setNodeHandle()
    # JuskeshinoKnowledge.setNodeHandle()
    # rospy.Subscriber("/hri/sp_rec/recognized", RecognizedSpeech, callback_sp_rec)
    state = SM_INIT
    while not rospy.is_shutdown():
        if state == SM_INIT:
            print("GPSR.->Waiting for command")
            JuskeshinoHRI.say("Please tell me what do you want me to do")
            JuskeshinoHRI.clearRecognizedSentences()
            sentence = JuskeshinoHRI.waitForNewSentence(15)
            cmd = grammar_checker.Command()
            cmd.sentence = sentence
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
            cmd = JuskeshinoHRI.waitForNewSentence(10)
            if cmd == "ROBOT YES" or cmd == "JUSTINA YES":
                print("GPSR.->Received confirmation: YES")
                JuskeshinoHRI.say("O K. I'm going to execute the command")
                state = SM_END
            else:
                JuskeshinoHRI.say("O K")
                print("GPSR.->Received cancelation of command")
                state = SM_INIT

        elif state == SM_END:
            print("GPSR performed succesfully")
            break
        rate.sleep()


if __name__ == "__main__":
    main()
