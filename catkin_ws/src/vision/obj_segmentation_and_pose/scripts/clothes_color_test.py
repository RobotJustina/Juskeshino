#!/usr/bin/env python3

import rospy
import ros_numpy
import cv2
import numpy as np
import tf.transformations as tft
import tf
import tf2_ros
from sensor_msgs.msg import PointCloud2
from vision_msgs.srv import *
from std_msgs.msg import Float64MultiArray

"""
    Nodo para detectar el color de las prendas de una persona, recibe un mensaje tipo
    HumanPoseEstimatorResult y regresa el color que esta mayoritariamente presente en
    la prenda.
"""

def clothes_color(msg):
    print("**********************************************")
    #print(msg.coordinates_array.coordinates_array[0].keypoints_array[2].keypoint_coordinates.position.x)
    #print(msg.coordinates_array.coordinates_array[0].keypoints_array[5].keypoint_coordinates)
    
    pc = rospy.wait_for_message("/hardware/realsense/points", PointCloud2)
    
    # tranformar la nube de puntos al sistema  "base_link"
    #.....................................................
    # transformar las coordenadas keypoint al sistema "base_link"
    #.....................................................

    arr = ros_numpy.point_cloud2.pointcloud2_to_array(pc)
    # Extrayendo coordenadas xyz de la nube de puntos
    x_arr = arr['x'].copy()
    y_arr = arr['y'].copy()
    z_arr = arr['z'].copy()

    # unir los canales
    array_xyz = cv2.merge((np.asarray(x_arr),np.asarray(y_arr),np.asarray(z_arr)))
    #print("type", type(x_arr))
    #print("shape", np.shape(x_arr))

    # Extrayendo valores rgb de la nube de puntos
    # ..........................................
    # unir los canales
    # ..........................................
    #mostrar la imagen

    # obteniendo rangos a partir del servicio pose_estimator
    # profundidad, medida experimental 
    x_min = msg.coordinates_array.coordinates_array[0].keypoints_array[2].keypoint_coordinates.position.x
    x_max = msg.coordinates_array.coordinates_array[0].keypoints_array[5].keypoint_coordinates.position.x + 0.05 
    # de hombro derecho a izquierdo
    y_min = msg.coordinates_array.coordinates_array[0].keypoints_array[2].keypoint_coordinates.position.y
    y_max = msg.coordinates_array.coordinates_array[0].keypoints_array[5].keypoint_coordinates.position.y

    # altura de hombro a codo
    z_min = msg.coordinates_array.coordinates_array[0].keypoints_array[3].keypoint_coordinates.position.z   
    z_max = msg.coordinates_array.coordinates_array[0].keypoints_array[5].keypoint_coordinates.position.z
        
    # Asigna limites de distancia en x,yz para segmentar el torso 
    min_valor = np.array([x_min, y_min, z_min]) 
    max_valor = np.array([x_max, y_max, z_max])

    
    # Usando cv2.inRange()
    # Comprueba si losimg_bin = cv2.inRange(img_hsv,min_valor, max_valor) elementos de la matriz se encuentran entre los elementos de otras dos matrices
    torso_mask = cv2.inRange(array_xyz , min_valor, max_valor)

    # Obtener histograma de color de la imagen

    color_clothes = []
    # convertir la nube de puntos 
    return color_clothes


def main():
    global pe_srv
    print("Nodo para probar el servicio clothes color.......")
    rospy.init_node("test_color")
    rospy.wait_for_service("/pose_estimator_srv")
    pe_srv = rospy.ServiceProxy("/pose_estimator_srv", HumanPoseEstimatorResult)
    pe_msg = HumanPoseEstimatorResultRequest()


  
    loop = rospy.Rate(10)
    while not rospy.is_shutdown():
        
        clothes_color(pe_srv(pe_msg))



        loop.sleep()

if __name__ == '__main__':
    main()