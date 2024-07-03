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
            JuskeshinoHRI.say("Please tell me what do you want me to do")
            JuskeshinoHRI.clearRecognizedSentences()
            cmd = JuskeshinoHRI.waitForNewSentence(15)
            cmdType = grammar_checker.getCommnandType(cmd)
            if cmd != '' and cmd is not None and cmdType is not None:
                print("Recognized command: ", cmd, "  of Type: ", cmdType)
                JuskeshinoHRI.say("Did you say")
                JuskeshinoHRI.say(cmd)
                JuskeshinoHRI.say("Please answer robot yes or robot no.")
                state = SM_WAIT_FOR_CONFIRMATION
        elif state == SM_WAIT_FOR_CONFIRMATION:
            cmd = JuskeshinoHRI.waitForNewSentence(10)
            if cmd == "ROBOT YES" or cmd == "JUSTINA YES":
                JuskeshinoHRI.say("O K. I'm going to execute the command")
                state = SM_END
            else:
                JuskeshinoHRI.say("O K")
                state = SM_INIT

        elif state == SM_END:
            print("GPSR performed succesfully")
            break
        rate.sleep()


if __name__ == "__main__":
    main()