#!/usr/bin/env python
import rospy
import tf
import face_recognition
import cv2
import os
import numpy
import ros_numpy
import datetime
from cv_bridge import CvBridge
from vision_msgs.srv import *
from vision_msgs.msg import *

def cloud_to_img_and_xyz(msg_cloud):
    arr = ros_numpy.point_cloud2.pointcloud2_to_array(msg_cloud)
    rgb_arr = arr['rgb'].copy()
    rgb_arr.dtype = numpy.uint32
    r = numpy.asarray(((rgb_arr >> 16) & 255), dtype='uint8')
    g = numpy.asarray(((rgb_arr >>  8) & 255), dtype='uint8')
    b = numpy.asarray(((rgb_arr      ) & 255), dtype='uint8')
    img_bgr = cv2.merge((b,g,r))
    return [img_bgr, arr]

def match_encodings(known_names, known_patterns, observed_patterns):
    distances = []
    for op in observed_patterns:
        distances.append(face_recognition.face_distance(known_patterns, op))
    matches = []
    for op in observed_patterns:
        best_match_idx = numpy.argmin([numpy.min(distances[i]) for i in range(len(distances))])
        best_distance = numpy.min(distances[best_match_idx])
        matches.append([known_names[best_match_idx], best_distance])
        for j in range(len(distances)):
            if known_names[j] == known_names[best_match_idx]:
                distances[j] = [float("inf") for i in range(len(distances[j]))]
    return matches

def callback_train_face(req):
    # This function only takes a photo and stores it in the corresponding folder
    global dataset_folder, img_result
    print("FaceRecognizer.->Trying to store photo in " + dataset_folder + "/" + req.id)
    if not req.id.isidentifier():
        print("FaceRecognizer.->Person name is not a valid name. Please don't operate Justina if you are not qualified enough")
        return False
    img_result, xyz = cloud_to_img_and_xyz(req.cloud)
    face_locations = face_recognition.face_locations(img_result)
    face_sizes = [abs((loc[3] - loc[1])*(loc[2]-loc[0])) for loc in face_locations]
    print("Max size: ", numpy.max(face_sizes))
    if len(face_locations) < 1 or numpy.max(face_sizes) < 5000:
        print("FaceRecognizer.->Cannot find any face or they are too small")
        return False
    largest_face_loc = face_locations[numpy.argmax(face_sizes)]
    mask = numpy.zeros(img_result.shape, dtype=numpy.uint8)
    mask[largest_face_loc[0]:largest_face_loc[2], largest_face_loc[3]:largest_face_loc[1]] = 255
    img_result = cv2.bitwise_and(img_result, mask)
    names = [folder_name for folder_name in next(os.walk(dataset_folder))[1]]
    if req.id.lower() not in names:
        os.makedirs(dataset_folder + "/" + req.id.lower())
    folder_name = dataset_folder + "/" + req.id.lower()
    file_name = folder_name + "/" + datetime.datetime.now().isoformat() + ".jpg"
    cv2.imwrite(file_name, img_result)

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
    encodings = face_recognition.face_encodings(img_result, face_locations)
    for loc in face_locations:
        cv2.rectangle(img_result, (loc[1], loc[0]), (loc[3], loc[2]), (0,255,0), 2)
    matches = match_encodings(known_names, known_patterns, encodings)
    resp = FindPersonsResponse()
    for i,(m,d) in enumerate(matches):
        cv2.putText(img_result, m+" " +str(d)[0:5], (face_locations[i][3], face_locations[i][0]-5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 1, cv2.LINE_AA)
        person = Person()
        person.name = m
        person.id = m
        resp.persons.append(person)
    return resp
    

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
    global img_result, dataset, dataset_folder
    print("INITIALIZING FACE RECOGNIZER BY MARCOSOFT...")
    rospy.init_node("face_recognizer")
    loop = rospy.Rate(10)
    dataset_folder = rospy.get_param("~dataset", ".")
    rospy.Service("/vision/face_recog/find_persons", FindPersons, callback_find_persons)
    rospy.Service("/vision/face_recog/train_person", FindPerson, callback_train_face)
    
    dataset = load_dataset(dataset_folder)
    img_result = numpy.zeros((480,640,3))

    while not rospy.is_shutdown() and cv2.waitKey(10) != 27:
        cv2.imshow("Recognized faces", img_result)
        loop.sleep()

if __name__ == "__main__":
    main()
