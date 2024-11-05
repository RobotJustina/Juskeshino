#!/usr/bin/env python3

from vision_msgs.srv import FindGesture,FindGestureResponse
import rospy

def callback(req):
    gestos ={1:"ALGO",2:"OTRO"}
    print(req.gesture_id)
    print(req)
    print("Hola Mundo")
    res = FindGestureResponse()

    return res
#---------------------------

def gesture_server():
	#---Parte para cargar lo necesario en inferencia con OpenPose y Markov---
	rospy.loginfo("Gesture recognition service available (NO DOCKER)")                    # initialize a ROS node
	s = rospy.Service('recognize_gesture', FindGesture, callback) 
	print("Gesture recognition service available")

	rospy.spin()
#---

#========================================
if __name__ == "__main__":
    rospy.init_node('gesture_recog')	
    gesture_server()
