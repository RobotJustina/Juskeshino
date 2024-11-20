#!/usr/bin/env python3

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

CAMERA_TOPIC_JUSTINA = '/camera/rgb/image_color'
# real                               '/camera/rgb/image_color'
# simulacion justina_gazebo          '/camera/depth_registered/rgb/image_raw' 
# simulacion justina_gazebo_lab      '/usb_cam/image_raw'



def face_rec(cv_image, known_face_encodings, known_face_names):
        global img
        rgb_image = cv.cvtColor(cv_image, cv.COLOR_BGR2RGB)
        face_locations = face_recognition.face_locations(rgb_image)
        face_encodings = face_recognition.face_encodings(rgb_image, face_locations)

        face_names = []
        for face_encoding in face_encodings:
            matches = face_recognition.compare_faces(known_face_encodings, face_encoding)
            print(matches)
            name = "Unknown"
            if True in matches:
                first_match_index = matches.index(True)
                name = known_face_names[first_match_index]
            face_names.append(name)

        print("face names",face_names)

        
        for (top, right, bottom , left), name in zip(face_locations, face_names):
            cv.rectangle(cv_image, (left, top), (right, bottom), (0, 255, 0), 2)
            cv.putText(cv_image, str(name), (left, top - 10), cv.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 0), 2)

        img = cv_image

        


def load_known_faces():
        rospack = rospkg.RosPack()
        Image_path = rospack.get_path("face_reco_pkg") + "/Train_faces/Image/"
        known_face_encodings = []
        known_face_names = []

        for filename in os.listdir(Image_path):
            name = os.path.splitext(filename)[0]   
            image_path = os.path.join(Image_path , filename) 
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
        cv.imwrite(os.path.expanduser('~/Juskeshino/catkin_ws/src/vision/face_reco_pkg/Train_faces/Image/'+req.name.data+'.jpg'), cv_image)
        # Guardar las codificaciones de la cara en un archivo de texto
        with open(os.path.expanduser('~/Juskeshino/catkin_ws/src/vision/face_reco_pkg/Train_faces/Text/'+req.name.data+'.txt'), 'a') as f:
            np.savetxt(f, face_encodings[0])
        response.success = True
        print("Face Recognition Node.->Iface successfully trained with the name: ", req.name.data)
    else: 
        response.success = False
        print("There is no guest or there is more than one in the scene....")
    return response
    

def callbackRecognizeFace(req):
    image_msg = rospy.wait_for_message(CAMERA_TOPIC_JUSTINA , Image)
    cv_image = image_convert(image_msg)
    face_encodings, face_names = load_known_faces()
    print("faces names", face_names)
    resp = recognize_face(face_names ,face_encodings , cv_image, req)
    face_rec(cv_image, face_encodings, face_names)
    return resp


def callbackTrainingFace(req):
    image_msg = rospy.wait_for_message(CAMERA_TOPIC_JUSTINA , Image)
    cv_image = image_convert(image_msg)
    resp = handle_face_training(cv_image ,req)
    return resp


def main():
    global img
    print("INITIALIZING FACE RECOGNITION AND TRAIN NODE.......ʕ•ᴥ•ʔ")
    rospy.init_node('face_recognition_node')

    rospy.Service('/vision/face_reco_pkg/recognize_face', FaceRecog, callbackRecognizeFace)
    rospy.Service('/vision/face_reco_pkg/training_face'  , FaceTrain,  callbackTrainingFace)
    
    loop = rospy.Rate(30)
    img = np.zeros((480, 640, 3), np.uint8)
    while not rospy.is_shutdown():
        cv.imshow("Face recognition", img)
        cv.waitKey(10)
        loop.sleep()

if __name__ == '__main__':
    main()
