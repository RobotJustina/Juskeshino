#!/usr/bin/env python3

import rospy
import ros_numpy
import cv2
import numpy as np
import time
import math
from std_msgs.msg import Bool
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PointStamped, Point
from vision_msgs.msg import *
from vision_msgs.srv import *
from juskeshino_tools.JuskeshinoVision import JuskeshinoVision
from juskeshino_tools.JuskeshinoSimpleTasks import JuskeshinoSimpleTasks

"""
0.nose, 1.neck , 2.r_sho , 3.r_elb , 4.r_wri , 5.l_sho, 6.l_elb, 7.l_wri, 8.r_hip, 
9.r_knee, 10.r_ank , 11.l_hip, 12.l_knee, 13.l_ank, 14.r_eye, 15.l_eye, 16.r_ear
"""




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


def clothes_color(hsv):
    h , s , v = hsv[0], hsv[1], hsv[2]
    if (s <= 5):
        if (s < 11):
            if (v >= 0 ) and (v < 27) : color = 'black'
            if (h >= 27 ) and (v < 92) : color = 'gray'
            else: color = 'white'
        return color
    else:
        if (h >= 0  ) and (h < 10 ) : color = 'red'   
        if (h >= 10 ) and (h < 25 ) : color = 'orange'    
        if (h >= 25 ) and (h < 34 ) : color = 'yellow'
        if (h >= 34 ) and (h < 70 ) : color = 'green'
        if (h >= 70 ) and (h < 125) : color = 'blue'
        if (h >= 125) and (h < 160) : color = 'purple'
        if (h >= 160) and (h < 180) : color = 'red'          
    return color


def callback(req):
    resp = FindPersonResponse()
    #resp.person.shirt = color_shirt
    #resp.person.pants = 'color_pants'
    return resp



def main():
    global pubHumanPoseEnable
    print("Obj_segmentation_and_pose --> Node that detects the color of a human's clothing.......(๑╹ω╹๑ )")
    rospy.init_node("clothes_color_srv")
    rospy.Service("/vision/obj_segmentation_and_pose/clothes_color", FindPerson, callback) 

    JuskeshinoVision.setNodeHandle()
    JuskeshinoSimpleTasks.setNodeHandle()
    [human_pose, su_frame_id, point_cloud] = JuskeshinoSimpleTasks.GetClosestHumanPose(10)
    print("frame id pc", su_frame_id)

    r_sho, l_sho, r_hip, l_hip = Point(), Point(), Point(), Point()
    # shoulders
    r_sho = human_pose.keypoints_array[2].keypoint_coordinates.position
    l_sho = human_pose.keypoints_array[5].keypoint_coordinates.position
    # hips
    r_hip = human_pose.keypoints_array[8].keypoint_coordinates.position 
    l_hip = human_pose.keypoints_array[11].keypoint_coordinates.position  

    point_ra     = JuskeshinoSimpleTasks.transformPoint( r_sho, "base_link", su_frame_id )
    point_la     = JuskeshinoSimpleTasks.transformPoint( l_sho, "base_link", su_frame_id )

    # T-shirt   
    # profundidad  
    x1_min = point_ra.x - 0.0
    x1_max = point_ra.x + 0.2
    # de hombro derecho a izquierdo
    y1_min = point_la.y
    y1_max = point_ra.y
    # altura de hombro a codo
    z1_max = point_ra.z
    z1_min = point_ra.z - 0.2  

    min_valor_torso = Point(x=x1_min, y=y1_min, z=z1_min)
    max_valor_torso = Point(x=x1_max, y=y1_max, z = z1_max)
    print("min val torso bl", min_valor_torso)
    print("max val torso bl", max_valor_torso)

    min_vt     = JuskeshinoSimpleTasks.transformPoint( min_valor_torso , su_frame_id , "base_link" )
    max_vt     = JuskeshinoSimpleTasks.transformPoint( max_valor_torso , su_frame_id , "base_link" )
    
    min_valor_torso  = np.array([min_vt.x, min_vt.y, min_vt.z])
    max_valor_torso  = np.array([max_vt.x, max_vt.y, max_vt.z])

    print("min val torso camera", min_valor_torso)
    print("max val torso camera", max_valor_torso)

    # Extrayendo valores rgb de la nube de puntos
    pc_array = ros_numpy.point_cloud2.pointcloud2_to_array(point_cloud)         
    rgb_arr = pc_array['rgb'].copy()
    rgb_arr.dtype = np.uint32
    # unir los canales
    r,g,b = ((rgb_arr >> 16) & 255), ((rgb_arr >> 8) & 255), (rgb_arr & 255)
    img_bgr = cv2.merge((np.asarray(b,dtype='uint8'),np.asarray(g,dtype='uint8'),np.asarray(r,dtype='uint8')))    
    # Extrayendo valores XYZ de la nube de puntos
    
    xyz = np.zeros((480, 640, 3))
    for i in range(480):
        for j in range(640):
            xyz[i,j][0] = pc_array[i,j][0]
            xyz[i,j][1] = pc_array[i,j][1]
            xyz[i,j][2] = pc_array[i,j][2]
            #print( pc_array[i,j][0] , pc_array[i,j][1], pc_array[i,j][2])

    # nube con respecto a base link **********************************************************************
    #new_pc = transformPointCloud2(copy_pc_array) 
    
    """
    x_arr = pc_array['x']
    y_arr = pc_array['y']
    z_arr = pc_array['z']

    xyz = cv2.merge((np.asarray(x_arr),np.asarray(y_arr),np.asarray(z_arr)))
    """

    # rango de la matriz de coordenadas
    torso_mask = cv2.inRange(xyz , min_valor_torso, max_valor_torso)
    print("torso mask", np.shape(torso_mask), np.size(torso_mask), np.ndim(torso_mask))

    cv2.imshow("mask", torso_mask)
    cv2.waitKey(4)
    time.sleep(10)

    

    #print("xyz",  np.shape(xyz), np.size(xyz), np.ndim(xyz))
    #print("torso mask", np.shape(torso_mask), np.size(torso_mask), np.ndim(torso_mask))
    #print("img rgb", np.shape(img_bgr), np.size(img_bgr), np.ndim(img_bgr))
    

    #leg_l_mask = cv2.inRange(new_pc , min_valor_leg_l , max_valor_leg_l)

    #pc_t_shirt = cv2.bitwise_and(xyz, xyz, mask=torso_mask)

    #new_pc = cv2.merge(pc_t_shirt, pc_array['rgb'])
    exit(0)
    #img_pants = cv2.bitwise_and(img_bgr,img_bgr, mask=leg_l_mask)
    #cv2.imshow("shirt", img_shirt)
    #cv2.waitKey(0) 
    #cv2.imshow("leg ", img_pants )
    #cv2.waitKey(0)
    """
    print("*************************************")
    hsv_shirt = color_histogram(img_bgr, torso_mask)
    #hsv_pants = color_histogram(img_bgr, leg_l_mask)
    print("hist: ", hsv_shirt)

    color_shirt = clothes_color(hsv_shirt)
    #color_pants = clothes_color(hsv_pants)

    print("color shirt", color_shirt)
    #print("color pants", color_pants)
    #color_clothes = [hsv_shirt, hsv_pants]
    """
    

    

    loop = rospy.Rate(30)
    while not rospy.is_shutdown():

        
        loop.sleep()

if __name__ == '__main__':
    main()