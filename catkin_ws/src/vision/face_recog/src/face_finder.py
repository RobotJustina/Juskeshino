#!/usr/bin/env python3

#Basic binary face detector
from pathlib import Path
import face_recognition
import cv2
import rospy
import numpy as np
import struct
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

def known_faces():
    global extensiones
    directorio = Path('/home/robocup/Juskeshino/catkin_ws/src/vision/face_recog/src/faces')
    extensiones = ['*.jpg', '*.jpeg', '*.png']
    faces=[]
    for ext in extensiones:
        for archivo in directorio.glob(ext):
            faces.append(archivo)
    return faces

def quitar_terminaciones(nombre_archivo):
    return nombre_archivo.replace('.png', '').replace('.jpg', '').replace('.jpeg', '')


def identify(known, unknown):
    global extensiones, face_locations
    known_load=[]
    known_encoding=[]
    k=0
    font = cv2.FONT_HERSHEY_SIMPLEX
    fontScale = 1
    color = (255, 0, 0) #in rgb
    thickness = 2
    unknown_encoding = face_recognition.face_encodings(unknown)
    for i in known:
        known_load.append(face_recognition.load_image_file(i))
    for i in known_load:
        known_encoding.append(face_recognition.face_encodings(i)[0])
    for i in known_encoding:
        for j in unknown_encoding:
            results = face_recognition.compare_faces([j], i)
            
            if results[0] == [True]:
                Name=quitar_terminaciones(known[k].name)
                print("find: " + Name)
                print(face_locations)
                unknown=cv2.putText(unknown, Name, block_midpoint(face_locations[k-1]), font, fontScale, color, thickness)
            else:
                Name="Unknown"
                print("find: " + Name)
                print(face_locations)
                unknown=cv2.putText(unknown, Name, block_midpoint(face_locations[k-1]), font, fontScale, color, thickness)
        k+=1
    k=0
    return unknown

def callback_recognize(msg):
    print("Reconocimiento iniciado")
def main():
    global bridge, face_bridge, cv_img, pcloud, face_point, frameid, face_locations
    print("Basic face detector")
    img = Image()
    bridge = CvBridge()
    face_bridge = CvBridge()
    pcloud = PointCloud2()
    cv_img = np.zeros((640,480,3),np.uint8)
    face_point = Point()
    rospy.init_node("face_finder")
    rospy.Subscriber('/camera/depth_registered/recognize' ,String ,callback_recognize )
    rospy.Subscriber("/camera/depth_registered/rgb/image_raw", Image, callback_image) # Simulación
    rospy.Subscriber("/camera/rgb/image_color", Image, callback_image) # Real
    #rospy.Subscriber("/camera/depth_registered/points", PointCloud2, callback_pointcloud)
    rospy.wait_for_message('/camera/depth_registered/recognize' , String)
    loop = rospy.Rate(2)
    while not rospy.is_shutdown():
        face_locations = face_recognition.face_locations(cv_img)
        if len(face_locations)>0:
            print("Face detected")
            print(face_locations)
            cv_img=identify(known_faces(), cv_img)
            cv2.imshow("face found",cv_img)
            cv2.waitKey(1)
            loop.sleep()
        else:
            print("There are no faces in this image")
        loop.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
