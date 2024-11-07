#!/usr/bin/env python3
import rospy
import numpy
import ros_numpy
import cv2
import face_recognition
from vision_msgs.srv import FindPerson, FindPersonResponse


def callback_find_person(req):
    global result
    print("request recive")
    arr = ros_numpy.point_cloud2.pointcloud2_to_array(req.cloud)
    rgb_arr = arr['rgb'].copy()
    rgb_arr.dtype = numpy.uint32
    r = numpy.asarray(((rgb_arr >> 16) & 255), dtype='uint8')
    g = numpy.asarray(((rgb_arr >>  8) & 255), dtype='uint8')
    b = numpy.asarray(((rgb_arr      ) & 255), dtype='uint8')
    result = cv2.merge((b,g,r))
    face_locations = face_recognition.face_locations(result)
    for y1, x1, y2, x2 in face_locations:
        cv2.rectangle(result, (x1,y1), (x2,y2), (0,255,0), 2)
    print(face_locations)

def main():
    global result
    print("INITIALZING FACE RECOG NODE")
    rospy.init_node("face_recognition")
    s = rospy.Service('/vision/find_person', FindPerson, callback_find_person)
    loop=rospy.Rate(10)
    result=numpy.zeros((480,640))
    while not rospy.is_shutdown():
        cv2.imshow("Results", result)
        cv2.waitKey(10)
        loop.sleep()


if __name__ == '__main__':
 
    main()


