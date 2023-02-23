#!/usr/bin/env python

import spacy
import stanza
import argparse

#ROS
import std_msgs.msg
from conceptual_deps.msg import StringArray
from conceptual_deps.srv import GetConDep, GetTextConDep

import rospy
import rospkg

#Start Client
service_name = "conceptual_dependencies/text_condep_service"
rospy.wait_for_service(service_name)
condep_service = rospy.ServiceProxy(service_name, GetTextConDep)

#Callbacks
def callbackConDepClient(msg):

    condeps = condep_service(msg.data).cds.data

    for i in range(len(condeps)):
    	print(condeps[i])


#Main
def main():
    rospy.init_node('condep_client')
  
    #Start ROS Services
    rospy.Subscriber("conceptual_dependencies/condep_text_client", std_msgs.msg.String, callbackConDepClient)
    
    rospy.loginfo("Conceptual Dependencies Client Example Initialized")
    
    #Infinite loop
    rospy.spin()

if __name__ == "__main__":
    main()
