#!/usr/bin/env python

import rospy
import rospkg
import cv2 as cv
import face_recognition
import os
import numpy as np

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from vision_msgs.srv import *
from vision_msgs.msg import *


CAMERA_TOPIC_JUSTINA = '/camera/depth_registered/rgb/image_raw'

def face_rec(cv_image, known_face_encodings, known_face_names):
        global image_pub
        
        rgb_image = cv.cvtColor(cv_image, cv.COLOR_BGR2RGB)
        face_locations = face_recognition.face_locations(rgb_image)
        face_encodings = face_recognition.face_encodings(rgb_image, face_locations)

        face_names = []
        bridge = CvBridge()
        for face_encoding in face_encodings:
            matches = face_recognition.compare_faces(known_face_encodings, face_encoding)
            print(matches)
            name = "Unknown"
            if True in matches:
                first_match_index = matches.index(True)
                name = known_face_names[first_match_index]
            face_names.append(name)

        print(face_names)

        for (top, right, bottom , left), name in zip(face_locations, face_names):
            cv.rectangle(cv_image, (left, top), (right, bottom), (0, 255, 0), 2)
            cv.putText(cv_image, name, (left, top - 10), cv.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 0), 2)

        cv.imshow("Image Window", cv_image)
        cv.waitKey(0)


def load_known_faces():
        rospack = rospkg.RosPack()
        Image_path = rospack.get_path("face_reco_pkg") + "/Train_faces/Image/"
        known_face_encodings = []
        known_face_names = []

        for filename in os.listdir(Image_path):
            name = os.path.splitext(filename)[0]                        #
            print ("name is:______", name)
            image_path = os.path.join(Image_path , filename)            #
            image = face_recognition.load_image_file(image_path)        
            try:
                face_encoding = face_recognition.face_encodings(image)[0]   
                known_face_encodings.append(face_encoding)
                known_face_names.append(name)
                return known_face_encodings, known_face_names
            except IndexError:
                print(IndexError)
                return None, None


def image_convert(ros_image):  
    bridge = CvBridge()
    try:
        # Convertir la imagen a formato OpenCV
        cv_image = bridge.imgmsg_to_cv2(ros_image , 'bgr8')
        return cv_image
    except:
        print("error")
        return None
    

def recognize_face(known_face_names ,known_face_encodings , cv_image, req):
    face_names_recog = []
    if req.is_face_recognition_enabled:
        rgb_image = cv.cvtColor(cv_image, cv.COLOR_BGR2RGB)
        face_locations = face_recognition.face_locations(rgb_image)
        face_encodings = face_recognition.face_encodings(rgb_image, face_locations)

        face_names = []
        for face_encoding in face_encodings:
            matches = face_recognition.compare_faces(known_face_encodings, face_encoding)
            name = "Unknown"
            if True in matches:
                first_match_index = matches.index(True)
                name = known_face_names[first_match_index]
            face_names.append(name)

        face_names_recog = face_names

        response = FaceRecogResponse()
        response.names = face_names_recog
        return response
    else:
        response = FaceRecogResponse()
        void = []
        void.append("false")
        response.names = void
        return response
    

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
    

def callback_recognize_face(req):
    image_msg = rospy.wait_for_message(CAMERA_TOPIC_JUSTINA , Image)
    cv_image = image_convert(image_msg)
    #cv.imshow("Show image was called....", cv_image) #*****************
    #cv.waitKey(0)
    face_encodings, face_names = load_known_faces()
    resp = recognize_face(face_names ,face_encodings , cv_image, req)
    face_rec(cv_image, face_encodings, face_names)
    return resp


def callback(req):
    image_msg = rospy.wait_for_message(CAMERA_TOPIC_JUSTINA , Image)
    cv_image = image_convert(image_msg)
    #cv.imshow("Show image was called....", cv_image) #*****************
    #cv.waitKey(0)
    resp = handle_face_training(cv_image ,req)
    return resp


def main():
    global image_pub
    print("Face recognice node.............ʕ•ᴥ•ʔ")
    rospy.init_node('face_recognition_node')

    rospy.Service('/vision/recognize_face/names', FaceRecog, callback_recognize_face)
    rospy.Service('/vision/training_face/name', FaceTrain, callback)
    
    loop = rospy.Rate(30)
    while not rospy.is_shutdown():
        loop.sleep()

if __name__ == '__main__':
    main()
