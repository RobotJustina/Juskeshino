#!/usr/bin/env python

import spacy
#import stanza
import argparse

#ROS
import std_msgs.msg
from conceptual_deps.msg import StringArray
from ros_whisper_vosk.srv import GetSpeech
from conceptual_deps.srv import GetConDep, GetTextConDep

from modules.condep_parser import CondepParser

#import pyttsx3
import rospy
import rospkg

#parser = argparse.ArgumentParser()
#parser.add_argument("--whisper", default=False, action='store_true', help="Whether to use Vosk (default) or Whisper")
#parser.add_argument("--stanza", default=False, action='store_true', help="Whether to use spaCy (default) or Stanza")

#args = parser.parse_args(rospy.myargv()[1:])

# print("------------------------------------------------")
# print(args)
# print("------------------------------------------------")

#is_whisper = args.whisper
#is_stanza = args.stanza

#Start Client
#service_name = "speech_recognition/vosk_service"
#if (is_whisper):
#    service_name = "speech_recognition/whisper_service"

#rospy.wait_for_service(service_name)
#speech_service = rospy.ServiceProxy(service_name, GetSpeech)

#Start ROS Publishers
pub_final = rospy.Publisher('conceptual_dependencies/final_result', StringArray, queue_size=10)

# #Callbacks
# def callbackConDepService(req):

#     _conceptual_deps = []
#     speech_text = ""
#     try:
#         speech_text = speech_service().data
#     except:
#         print("An error occurred!")
#         conceptual_deps = StringArray(_conceptual_deps)
#         return conceptual_deps

#     #Process text to CD
#     cds = []
#     if (is_stanza):
#         print("Sample code with stanza. TBD")
#     else:
        
#         cds = CondepParser(speech_text)

#     #Generate final CDs List
#     conceptual_deps = StringArray(cds)
#     print(conceptual_deps)

#     pub_final.publish(conceptual_deps)
#     return conceptual_deps

def callbackTextConDepService(req):

    _conceptual_deps = []
    speech_text = ""
    try:
        speech_text = req.text
        speech_text.lstrip()
        print("You wrote: " + speech_text)
    except:
        print("An error occurred!")
        conceptual_deps = StringArray(_conceptual_deps)
        return conceptual_deps

    #Process text to CD
    cds = []
    # if (is_stanza):
    #     print("Sample code with stanza. TBD")
    # else:
        
    cds = CondepParser(speech_text)

    #Generate final CDs List
    conceptual_deps = StringArray(cds)
    print(conceptual_deps)

    pub_final.publish(conceptual_deps)
    return conceptual_deps

#Main
def main():
    rospy.init_node('condep_service')
  
    #Start ROS Services
    # rospy.Service("conceptual_dependencies/condep_service", GetConDep, callbackConDepService)
    rospy.Service("conceptual_dependencies/text_condep_service", GetTextConDep, callbackTextConDepService)
    
    rospy.loginfo("Conceptual Dependencies Service Initialized")
    
    #Infinite loop
    rospy.spin()

if __name__ == "__main__":
    main()
