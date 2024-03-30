#!/usr/bin/env python

import rospy
import os
import numpy as np
import face_recognition
from vision_msgs.msg import *
from vision_msgs.srv import *

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Header
import cv2 as cv

CAMERA_JUSTINA_TOPIC = '/camera/depth_registered/rgb/image_raw'

def handle_face_training(cv_image, req):
    face_encodings = []
    # Encontrar la cara en la imagen
    face_locations = face_recognition.face_locations(cv_image)
    face_encodings = face_recognition.face_encodings(cv_image, face_locations)
    size = len(face_encodings)
    # Agregar la codificacion de la cara a la lista
    face_encodings.extend(face_encodings)
    # Envia la respuesta del servicio
    response = FaceTrainResponse()
    # Guardar la imagen y las codificaciones de la cara en disco
    if size == 1:
        # Guardar la imagen J
        print("if**")
        cv.imwrite(os.path.expanduser('~/Juskeshino/catkin_ws/src/vision/face_reco_pkg/Train_faces/Image/'+req.name.data+'.jpg'), cv_image)
        # Guardar las codificaciones de la cara en un archivo de texto
        with open(os.path.expanduser('~/Juskeshino/catkin_ws/src/vision/face_reco_pkg/Train_faces/Text/'+req.name.data+'.txt'), 'a') as f:
            np.savetxt(f, face_encodings[0])
        response.success = True
        response.message = "Cara entrenada con exito con el nombre " + req.name.data
    else: 
        print("else**")
        response.success = False
        response.message = "No hay un invitado o hay mas de uno en la escena. " 
    return response

def image_convert(ros_image):  
    bridge = CvBridge()
    try:
        # Convertir la imagen a formato OpenCV
        cv_image = bridge.imgmsg_to_cv2(ros_image , 'bgr8')
        return cv_image
    except:
        print("error")
        return None

def callback(req):
    image_msg = rospy.wait_for_message(CAMERA_JUSTINA_TOPIC, Image)
    cv_image = image_convert(image_msg)
    #cv.imshow("Show image was called....", cv_image) #*****************
    #cv.waitKey(0)
    resp = handle_face_training(cv_image ,req)
    return resp

def main():
    print("Face train node.............ʕ•ᴥ•ʔ")
    rospy.init_node("face_train_node")
    rospy.Service('/vision/training_face/name', FaceTrain, callback)
    
    loop = rospy.Rate(30)
    while not rospy.is_shutdown():
        loop.sleep()

if __name__ == '__main__':
    main()
