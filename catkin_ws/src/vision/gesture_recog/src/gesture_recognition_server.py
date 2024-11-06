#!/usr/bin/env python3

from vision_msgs.srv import FindGesture,FindGestureResponse
import rospy
from utils_gesture import *
# -----------------------------------
def callback(req):
	n_people_max=3
	response = FindGestureResponse()
	#HUMAN DETECTION
	if req.gesture_id == 0:     
		opWrapper,datum=init_openPose(n_people=n_people_max)
		xyz_response = human_detection(req.cloud) # NO usa openpose, basado del paquete human_detector
		response = xyz_response
	# =========	
	# WAVING DETECTION
	elif req.gesture_id == 1:
		opWrapper,datum=init_openPose(n_people=n_people_max)
		response = waving_detection(req.cloud,opWrapper,datum)
	
	# =========		
	# POINTING DETECTION
	elif req.gesture_id == 2:
		opWrapper,datum=init_openPose(n_people=1)
		"""
		Retorna x,y,0 de la extrapolacion de mano y codo que se detectó levantada,
		'mano_levantada' indica si fue izquierda (0) o derecha (1)

		NO publica ninguna TF
		NO regresa la extrapolacion

		Si NO detectó persona o al obtener x,y,z obtuvo NaN, retorna False
		"""
		mano, codo, img = pointing_detection(req.cloud,opWrapper,datum)
		# Opcion para no usar OpenPose, del paquete de human_detector
		#xyz_extrapolacion, mano_levantada = detect_pointing_OPENCV(req.cloud)
		if mano is not False:
			response = 1
			response.person.pose.append(mano)
			# Pendiente de llenar msg
	
	# =========	
	# HANDLING DETECTION/ACTION, por definir/desarrolar
	elif req.gesture_id == 3:
		opWrapper,datum=init_openPose(n_people=1)
		response = handling_detection(req.cloud,opWrapper,datum)
	
	# =========		
	#RECIEVING ACTION/DETECTION, por definir/desarrolar
	elif req.gesture_id == 4:
		opWrapper,datum=init_openPose(n_people=1)
		response = recieving_detection(req.cloud,opWrapper,datum)
	
	# =========		
	else:
		print("Opcion No Valida")
		return False

	return response
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
