#!/usr/bin/env python
import rospy
import tf
import face_recognition
import cv2
import os
import numpy
import ros_numpy
from cv_bridge import CvBridge
from vision_msgs.srv import *

def cloud_to_img_and_xyz(msg_cloud):
    arr = ros_numpy.point_cloud2.pointcloud2_to_array(msg_cloud)
    rgb_arr = arr['rgb'].copy()
    rgb_arr.dtype = numpy.uint32
    r = numpy.asarray(((rgb_arr >> 16) & 255), dtype='uint8')
    g = numpy.asarray(((rgb_arr >>  8) & 255), dtype='uint8')
    b = numpy.asarray(((rgb_arr      ) & 255), dtype='uint8')
    img_bgr = cv2.merge((b,g,r))
    return [img_bgr, arr]

def callback_find_persons(req):
    global img_result, dataset
    print("FaceRecognizer.->Trying to find people in image")
    known_names = []
    known_patterns = []
    for k in list(dataset.keys()):
        for e in dataset[k]:
            known_names.append(k)
            known_patterns.append(e)
    img_result, xyz = cloud_to_img_and_xyz(req.cloud)
    face_locations = face_recognition.face_locations(img_result)
    encodings = face_recognition.face_encodings(img_result, face_locations)[0]
    for loc in face_locations:
        cv2.rectangle(img_result, (loc[1], loc[0]), (loc[3], loc[2]), (0,255,0), 2)
    print("No of encodings: ", len(encodings))
    print("Number of known patterns: ", len(known_patterns), known_patterns[0])
    for e in encodings:
        dists = face_recognition.face_distance(known_patterns, e)
        print("Distances for pattern")
        print(dists)

def load_dataset(path):
    dataset = {}
    print("FaceRecognizer.->Loading dataset from " + path)
    for folder_name in next(os.walk(path))[1]:
        folder = path + "/" + folder_name
        dataset[folder_name.lower()] = []
        for file_name in next(os.walk(folder))[2]:
            f = folder + "/" + file_name
            img = face_recognition.load_image_file(f)
            enc = face_recognition.face_encodings(img)[0]
            dataset[folder_name.lower()].append(enc)
    print("FaceRecognizer.->Loaded dataset with " + str(len(list(dataset.keys()))) + " face patterns")
    for k in list(dataset.keys()):
        print("FaceRecognizer.-> " + k + " has " + str(len(dataset[k])) + " images")
    return dataset 

def main():
    global img_result, dataset
    print("INITIALIZING FACE RECOGNIZER BY MARCOSOFT...")
    rospy.init_node("face_recognizer")
    loop = rospy.Rate(10)
    dataset_folder = rospy.get_param("~dataset", ".")
    rospy.Service("/vision/face_recog/find_persons", FindPersons, callback_find_persons)
    
    dataset = load_dataset(dataset_folder)
    img_result = numpy.zeros((480,640,3))

    while not rospy.is_shutdown() and cv2.waitKey(10) != 27:
        cv2.imshow("Recognized faces", img_result)
        loop.sleep()

if __name__ == "__main__":
    main()
