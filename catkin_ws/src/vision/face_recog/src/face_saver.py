#!/usr/bin/env python3

#Basic binary face detector
from pathlib import Path
import face_recognition
import cv2
import rospy
import numpy as np
import struct
import os
from cv_bridge import CvBridge
from std_msgs.msg import Header
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import String
from geometry_msgs.msg import Point, Pose, PoseStamped 

def callback_image(msg):
    global bridge, cv_img
    cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

def callback_pointcloud(msg):
    global pcloud, frameid
    pcloud = msg
    frameid = msg.header.frame_id
    
def pixel_to_3D_point(pc2, u, v):
    height = pc2.height
    width = pc2.width
    p = Point()
    array_position = v*pc2.row_step + u*pc2.point_step
    apx = array_position + pc2.fields[0].offset
    apy = array_position + pc2.fields[1].offset
    apz = array_position + pc2.fields[2].offset
    #print(pc2.data[apx:apx+4])
    #print(struct.unpack('<f',pc2.data[apx:apx+4]))
    q = np.frombuffer(pc2.data,dtype=np.float32,count=3,offset=apx)
    p.x = struct.unpack('<f',pc2.data[apx:apx+4])[0]
    p.y = struct.unpack('<f',pc2.data[apy:apy+4])[0]
    p.z = struct.unpack('<f',pc2.data[apz:apz+4])[0]
    return p
    
def block_midpoint(blk):
    xm = int((blk[3]+blk[1])/2)
    ym = int((blk[2]+blk[0])/2)
    return [xm,ym]

def build_posestamped_msg(frid, pos):
    hdr = Header()
    hdr.frame_id = frid
    fpose = PoseStamped()
    fpose.header = hdr
    fpose.pose.position = pos
    fpose.pose.orientation.x = 0.0
    fpose.pose.orientation.y = 0.0
    fpose.pose.orientation.z = 0.0
    fpose.pose.orientation.w = 1.0
    return fpose

def callback_save(msg):
    print("Saving " + str(msg))
def main():
    global bridge, face_bridge, cv_img, pcloud, face_point, frameid, face_locations
    path="/home/robocup/Juskeshino/catkin_ws/src/vision/face_recog/src/faces"
    img = Image()
    bridge = CvBridge()
    face_bridge = CvBridge()
    pcloud = PointCloud2()
    cv_img = np.zeros((640,480,3),np.uint8)
    face_point = Point()
    rospy.init_node("face_saver")
    rospy.Subscriber('/camera/depth_registered/save' ,String ,callback_save )
    #rospy.Subscriber("/camera/depth_registered/rgb/image_raw", Image, callback_image) # Simulación
    rospy.Subscriber("/camera/rgb/image_color", Image, callback_image) # Real
    rospy.Subscriber("/camera/depth_registered/points", PointCloud2, callback_pointcloud)
    face_name=rospy.wait_for_message('/camera/depth_registered/save' , String)
    name=face_name.data
    print("Trying saving the image")
    face_locations = face_recognition.face_locations(cv_img)
    trys=0
    while not rospy.is_shutdown() and trys<5:
        face_locations = face_recognition.face_locations(cv_img)
        if len(face_locations)>0:
            print(name + "Face detected")
            print(face_locations)
            name=name + ".png"
            os.chdir(path)
            cv2.imwrite(name, cv_img)
            trys=6
        else:
            trys+=1
    if len(face_locations)==0:
        print("There are no faces in this image")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
