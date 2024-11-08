#!/usr/bin/env python3

import rospy
import ros_numpy
import cv2
import numpy as np
import tf
from geometry_msgs.msg import PointStamped, Point
from vision_msgs.srv import *
from juskeshino_tools.JuskeshinoSimpleTasks import JuskeshinoSimpleTasks
from juskeshino_tools.JuskeshinoVision import JuskeshinoVision

SENSOR_FRAME = 'camera_rgb_optical_frame'
BL = 'base_link'

"""
    Nodo para detectar el color de las prendas de una persona, recibe un mensaje tipo
    HumanPoseEstimatorResult y regresa el color que esta mayoritariamente presente en
    la prenda.
"""
def point_actual2point_target(pointxyz, f_actual, f_target):
    global pose_in, listener
    # Empaqueta msg, convierte orientacion de frame 'realsense_link' a frame 'base_link'
    ps_msg = PointStamped()
    ps_msg.header.frame_id = f_actual   # frame de origen
    ps_msg.header.stamp = rospy.Time(0)  # la ultima transformacion
    ps_msg.point = pointxyz
    new_ps = listener.transformPoint(f_target, ps_msg)
    new_pointxyz = new_ps.point
    return new_pointxyz


def clothes_color(hsv):
    h = hsv[0]  # 180
    s = hsv[1]  # 255
    v = hsv[2]  # 255
    
    if (s <= 40): # de negro a blanco
        
        if (v >= 0 )  and (v < 27) : color = 'black'
        if (h >= 27 ) and (v < 92) : color = 'gray'
        else: color = 'white'
        
    else:
        if (h >= 0  ) and (h < 10 ): color = 'red'
        if (h >= 10 ) and (h < 25 ): color = 'orange'
        if (h >= 25 ) and (h < 34 ): color = 'yellow'                        
        if (h >= 34 ) and (h < 70 ): color = 'green'            
        if (h >= 70 ) and (h < 125): color = 'blue'            
        if (h >= 125) and (h < 160): color = 'purple'
        if (h >= 160) and (h < 180): color = 'red'      
    return color
        

def color_histogram(img_bgr, mask):
    # Obtener histograma de color de la imagen para cada canal
    img_hsv = cv2.cvtColor(img_bgr,cv2.COLOR_BGR2HSV)
    Hh = cv2.calcHist([img_hsv],[0], mask ,[180],[0,180])
    Sh = cv2.calcHist([img_hsv],[1], mask ,[256],[0,256])
    Vh = cv2.calcHist([img_hsv],[2], mask ,[256],[0,256])
    #print("Hh, Sh, Vh", Hh, Sh, Vh)
    h = Hh.flatten().tolist() 
    s = Sh.flatten().tolist() 
    v = Vh.flatten().tolist()
    #print("h,s,v mayor valor", h,s,v)
    maxh = max(h)  #lista
    maxs = max(s)  #lista
    maxv = max(v)  #list
    H = h.index(maxh)   #lista
    S = s.index(maxs)   #lista
    V = v.index(maxv)   #lista
    #print("indice de mayor valor", H,S,V)
    return [H,S,V]


def callback(req):
    global listener , img_bgr
    pe_msg = HumanPoseEstimatorResultRequest()
    
    clt_transform = rospy.ServiceProxy("/vision/point_cloud_to_base_link", PreprocessPointCloud)
    req_trans = PreprocessPointCloudRequest()
    req_trans.input_cloud = req.cloud
    resp_trans = clt_transform(req_trans)
    cloud = resp_trans.output_cloud

    arr_pc = ros_numpy.point_cloud2.pointcloud2_to_array(cloud)
    [closest_human_pose, frame_id] = JuskeshinoSimpleTasks.GetClosestHumanPose(arr_pc , 8)
    if closest_human_pose != None:
        # Extrayendo valores rgb de la nube de puntoss
        rgb_arr = arr_pc['rgb'].copy()
        rgb_arr.dtype = np.uint32
        # unir los canales
        r,g,b = ((rgb_arr >> 16) & 255), ((rgb_arr >> 8) & 255), (rgb_arr & 255)
        img_bgr = cv2.merge((np.asarray(b,dtype='uint8'),np.asarray(g,dtype='uint8'),np.asarray(r,dtype='uint8')))    
        # Extrayendo valores XYZ de la nube de puntos
        copy_pc_array = arr_pc.copy()
        x_pc = arr_pc['x'].copy()
        y_pc = arr_pc['y'].copy()
        z_pc = arr_pc['z'].copy()
        new_pc = cv2.merge((np.asarray(x_pc),np.asarray(y_pc),np.asarray(z_pc)))

        # Asigna limites de distancia en x,y,z para segmentar el torso y pierna***************
        try:
            # Hombros
            keypoint1 = Point()
            r_sho = closest_human_pose.keypoints_array[2].keypoint_coordinates.position
            keypoint2 = Point()
            l_sho = closest_human_pose.keypoints_array[5].keypoint_coordinates.position
            # codo
            keypoint3 = Point()
            r_elb = closest_human_pose.keypoints_array[3].keypoint_coordinates.position  
            # rodillas
            keypoint4 = Point() #l_knee
            r_kne= closest_human_pose.keypoints_array[9].keypoint_coordinates.position  
        except:
            print("Clothes_color_srv.-->Some body part was not found, please go back a few steps")
            resp = FindPersonResponse()
            resp.person.shirt = None
            resp.person.pants = None
            print("Clothes_color_srv.-> Please call service again........................")
            return resp


        
        # In 'base_link'
        point_ra     = point_actual2point_target(r_sho, SENSOR_FRAME ,BL)
        point_la     = point_actual2point_target(l_sho, SENSOR_FRAME ,BL)
        point_r_elb  = point_actual2point_target(r_elb, SENSOR_FRAME ,BL)
        point_l_knee = point_actual2point_target(r_kne, SENSOR_FRAME ,BL)
        # TORSO
        # profundidad, medida experimental     
        x1_min = point_ra.x - 0.2
        x1_max = point_ra.x + 0.2
        # de hombro derecho a izquierdo
        y1_min = point_la.y
        y1_max = point_ra.y
        # altura de hombro a codo
        z1_min = point_r_elb.z  
        z1_max = point_ra.z
        # PANTS 
        x2_min = point_l_knee.x - 0.2
        x2_max = point_l_knee.x + 0.2
        # derecha = max,   izquierdo = min
        y2_min = point_l_knee.y - 0.06
        y2_max = point_l_knee.y + 0.06
        # altura de rodilla + 0,1 m
        z2_min = point_l_knee.z  
        z2_max = point_l_knee.z + 0.2

        # limites en base link frame
        lower_bound_shirt = np.array([x1_min, y1_min, z1_min])
        upper_bound_shirt = np.array([x1_max, y1_max, z1_max])
        lower_bound_pants = np.array([x2_min, y2_min, z2_min])
        upper_bound_pants = np.array([x2_max, y2_max, z2_max])
        # rango de la matriz de coordenadas
        torso_mask = cv2.inRange(new_pc , lower_bound_shirt, upper_bound_shirt)
        leg_l_mask = cv2.inRange(new_pc , lower_bound_pants, upper_bound_pants)
        
        img_shirt = cv2.bitwise_and(img_bgr,img_bgr, mask=torso_mask)
        img_pants = cv2.bitwise_and(img_bgr,img_bgr, mask=leg_l_mask)

        hsv_shirt = color_histogram(img_bgr, torso_mask)
        hsv_pants = color_histogram(img_bgr, leg_l_mask)
        print("hist: t shirt ", hsv_shirt)
        print("hist: pants ", hsv_pants)


        color_shirt = clothes_color(hsv_shirt)
        color_pants = clothes_color(hsv_pants)

        print("color shirt", color_shirt)
        print("color pants", color_pants)

        img_bgr = cv2.putText(img_bgr ,"color T-shirt:"+str(color_shirt),(200,150),cv2.FONT_HERSHEY_SIMPLEX,0.9,(0,0,0),3)
        img_bgr = cv2.putText(img_bgr ,"color Pants  :"+str(color_pants),(200,350),cv2.FONT_HERSHEY_SIMPLEX,0.9,(0,0,0),3)

        resp = FindPersonResponse()
        resp.person.shirt = color_shirt
        resp.person.pants = color_pants
        
        return resp
    else:
        resp = FindPersonResponse()                                             
        resp.person.shirt = None
        resp.person.pants = None
        print("Clothes_color_srv.-> Error identifying person, please call service again........................")
        return resp


def main():
    global listener, img_bgr
    print("Node that detects the color of a human's clothing.......(๑╹ω╹๑ )")
    rospy.init_node("clothes_color_srv")
    rospy.Service("/vision/clothes_color", FindPerson, callback) 
    JuskeshinoSimpleTasks.setNodeHandle()
    JuskeshinoVision.setNodeHandle()
    img_bgr = np.zeros((480, 640, 3), np.uint8)

    listener = tf.TransformListener()
    loop = rospy.Rate(10)

    while not rospy.is_shutdown():
        cv2.imshow("Clothes Color - Recognition Result", img_bgr)
        cv2.waitKey(10)
        loop.sleep()

if __name__ == '__main__':
    main()