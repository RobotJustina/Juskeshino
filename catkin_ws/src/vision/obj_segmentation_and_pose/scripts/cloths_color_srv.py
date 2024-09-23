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

def callback(req):
    [human_poses, closest_human_idx] = JuskeshinoSimpleTasks.GetHumanPoseArray(10)

    return 

def main():
    global pubHumanPoseEnable
    print("Obj_segmentation_and_pose --> Node that detects the color of a human's clothing.......(๑╹ω╹๑ )")
    rospy.init_node("clothes_color_srv")
    rospy.Service("/vision/obj_segmentation_and_pose/clothes_color", FindPerson, callback) 

    JuskeshinoVision.setNodeHandle()
    JuskeshinoSimpleTasks.setNodeHandle()
    human_pose = JuskeshinoSimpleTasks.GetClosestHumanPose(10)

    # Hombros
    keypoint1 = Point()
    keypoint1 = human_pose.keypoints_array[2].keypoint_coordinates.position
    keypoint2 = Point()
    keypoint2 = human.keypoints_array[5].keypoint_coordinates.position
    # codo
    keypoint3 = Point()
    keypoint3 = human.keypoints_array[3].keypoint_coordinates.position  

    # rodillas
    
    keypoint4 = Point() #l_knee
    keypoint4 = human.keypoints_array[12].keypoint_coordinates.position  
    
                    [x, y, z] = JuskeshinoSimpleTasks.transformPoint( k.keypoint_coordinates.position.x ,
                                                                      k.keypoint_coordinates.position.y, 
                                                                      k.keypoint_coordinates.position.z, "base_link", human_posess.header.frame_id )
                    

    point_ra     = point_actual2point_target(keypoint1, sensor_frame ,base_link)
    point_la     = point_actual2point_target(keypoint2, sensor_frame ,base_link)
    point_r_elb  = point_actual2point_target(keypoint3, sensor_frame ,base_link)

        # TORSO
    # obteniendo rangos a partir del servicio pose_estimator
    # Hombro derecho - hombro izquierdo..............................yr-yl
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
    
    x2_min = point_l_knee.x 
    x2_max = point_l_knee.x + 0.09
    # derecha = max,   izquierdo = min
    y2_min = point_l_knee.y - 0.04
    y2_max = point_l_knee.y + 0.04
    # altura de rodilla + 0,1 m
    z2_min = point_l_knee.z  
    z2_max = point_l_knee.z + 0.2
    

    min_valor_torso = np.array([x1_min, y1_min, z1_min]) 
    max_valor_torso = np.array([x1_max, y1_max, z1_max])

    
    min_valor_leg_l = np.array([x2_min, y2_min, z2_min]) 
    max_valor_leg_l = np.array([x2_max, y2_max, z2_max])

    # reconstruir la imagen con la nube de puntos
    


    

    loop = rospy.Rate(30)
    while not rospy.is_shutdown():

        
        loop.sleep()

if __name__ == '__main__':
    main()