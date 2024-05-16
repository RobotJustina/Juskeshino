#!/usr/bin/env python3
import rospy
import ros_numpy
import cv2
import numpy as np
import tf
import math
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PointStamped, Point
from vision_msgs.srv import *
from std_msgs.msg import Bool


FRAME_ID_CAMERA = "camera_rgb_optical_frame" 
CAMERA_TOPIC     = "/camera/depth_registered/points"


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



def transformPointCloud2(pc):
    sensor_frame = FRAME_ID_CAMERA
    # tranformar la nube de puntos al sistema  "base_link'
    x_arr = pc['x']
    y_arr = pc['y']
    z_arr = pc['z']

    i,j = 0,0
    p = Point()
    for i in range(480):
        for j in range(640):
            if np.isnan(x_arr[i,j]) or np.isnan(y_arr[i,j]) or np.isnan(z_arr[i,j]): 'print("punto nan")'
            else: 
                p.x, p.y, p.z = x_arr[i,j], y_arr[i,j] , z_arr[i,j]
                new_frame_p = point_actual2point_target(p, sensor_frame, 'base_link')
                x_arr[i,j], y_arr[i,j] , z_arr[i,j] = new_frame_p.x, new_frame_p.y, new_frame_p.z

    new_pc = cv2.merge((np.asarray(x_arr),np.asarray(y_arr),np.asarray(z_arr)))

    pc['x'] = x_arr
    pc['y'] = y_arr
    pc['z'] = z_arr
    return new_pc  


"""
def pointingHandsDetector(person):
    # Hombros
    point_r_sho = Point()
    point_r_sho = person.keypoints_array[2].keypoint_coordinates.position
    point_l_sho = Point()
    point_l_sho = person.keypoints_array[5].keypoint_coordinates.position
    # codo
    point_r_elb = Point()
    point_r_elb = person.keypoints_array[3].keypoint_coordinates.position  
    point_l_elb = Point()
    point_l_elb = person.keypoints_array[3].keypoint_coordinates.position
    # munecas
    point_r_wri = Point() #l_knee
    point_r_wri = person.keypoints_array[12].keypoint_coordinates.position  
    point_l_wri = Point() #l_knee
    point_l_wri = person.keypoints_array[12].keypoint_coordinates.position  
    
    sensor_frame = FRAME_ID_CAMERA

    # Conversion de puntos de un sistema a otro
    point_r_sho = point_actual2point_target(point_r_sho , sensor_frame ,'base_link')
    point_l_sho = point_actual2point_target(point_r_sho , sensor_frame ,'base_link')
    point_r_elb = point_actual2point_target(point_r_sho , sensor_frame ,'base_link')
    point_l_elb = point_actual2point_target(point_r_sho , sensor_frame ,'base_link')
    point_r_wri = point_actual2point_target(point_r_sho , sensor_frame ,'base_link')
    point_l_wri = point_actual2point_target(point_r_sho , sensor_frame ,'base_link')

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
    

            for person in people:
                person_id = person.person_id
                keypoints = {kpt.keypoint_name:kpt for kpt in person.keypoints_array if kpt.keypoint_name in ['neck', 'r_elb', 'r_wri', 'l_elb', 'l_wri', 'r_hip', 'l_hip']}

                neck  = [keypoints["neck"].keypoint_coordinates.position.x,
                         keypoints["neck"].keypoint_coordinates.position.y, 
                         keypoints["neck"].keypoint_coordinates.position.z] 

                r_wri = [keypoints["r_wri"].keypoint_coordinates.position.x,
                         keypoints["r_wri"].keypoint_coordinates.position.y, 
                         keypoints["r_wri"].keypoint_coordinates.position.z]
   
                l_wri = [keypoints["l_wri"].keypoint_coordinates.position.x,
                         keypoints["l_wri"].keypoint_coordinates.position.y,
                         keypoints["l_wri"].keypoint_coordinates.position.z]

                r_elb = [keypoints["r_elb"].keypoint_coordinates.position.x,
                         keypoints["r_elb"].keypoint_coordinates.position.y, 
                         keypoints["r_elb"].keypoint_coordinates.position.z]
   
                l_elb = [keypoints["l_elb"].keypoint_coordinates.position.x,
                         keypoints["l_elb"].keypoint_coordinates.position.y,
                         keypoints["l_elb"].keypoint_coordinates.position.z]

                r_hip = [keypoints["r_hip"].keypoint_coordinates.position.x,
                         keypoints["r_hip"].keypoint_coordinates.position.y,
                         keypoints["r_hip"].keypoint_coordinates.position.z]

                l_hip = [keypoints["l_hip"].keypoint_coordinates.position.x,
                         keypoints["l_hip"].keypoint_coordinates.position.y,
                         keypoints["l_hip"].keypoint_coordinates.position.z]

                # add c_hip
                c_hip_x = (r_hip[0] + l_hip[0]) / 2
                c_hip_y = (r_hip[1] + l_hip[1]) / 2
                c_hip_z = (r_hip[2] + l_hip[2]) / 2

                # vectors for pointing gesture detection
                r_elbow2wrist = np.array([r_wri[0] - r_elb[0], \
                                         r_wri[1] - r_elb[1], \
                                         r_wri[2] - r_elb[2]])

                l_elbow2wrist = np.array([l_wri[0] - l_elb[0], \
                                         l_wri[1] - l_elb[1], \
                                         l_wri[2] - l_elb[2]])

                r_camera2elbow = np.array([r_wri[0] - 0, \
                                           r_wri[1] - 0, \
                                           r_wri[2] - 0])

                l_camera2elbow = np.array([l_wri[0] - 0, \
                                           l_wri[1] - 0, \
                                           l_wri[2] - 0])

"""



def callback(req):
    global listener , pe_srv

    pe_msg = HumanPoseEstimatorResultRequest()
    msg = pe_srv(pe_msg)
    pc = rospy.wait_for_message(CAMERA_TOPIC , PointCloud2)
    arr_pc = ros_numpy.point_cloud2. pointcloud2_to_array(pc)

    print(" Numero de personas detectadas: ",len(msg.coordinates_array.coordinates_array))
    human_list = []

    # filtrando arreglos vacios
    for i in range(len(msg.coordinates_array.coordinates_array)):
        if len(msg.coordinates_array.coordinates_array[i].keypoints_array) < 1:
            print("Lista vacia")
            continue

        human = vision_msgs.msg._HumanCoordinates.HumanCoordinates()
        human.keypoints_array = msg.coordinates_array.coordinates_array[i].keypoints_array
        human_list.append(human)

    print("longitud de lista nueva de personas", len(human_list))
    print(human_list)
    person_dist_list = []
    guy = human_list[0]
    
    if len(human_list) > 1:
        # take the Euclidean distance from the nearest nose
        for person in human_list:
            x = person.keypoints_array[0].keypoint_coordinates.position.x
            y = person.keypoints_array[0].keypoint_coordinates.position.y
            z = person.keypoints_array[0].keypoint_coordinates.position.z
            person_dist = np.sqrt((x**2 + y**2 + z**2))
            person_dist_list.append(person_dist)

        print("DISTANCIAS", person_dist_list)
        min_dist =  min(person_dist_list)
        print("distancia minima: ", min_dist)
        human_idx = person_dist_list.index(min_dist)
        guy = human_list[human_idx]

    # Vectores de los brazos **************
    #pointingHandsDetector(guy)

    resp = FindPersonResponse()
    return resp



def main():
    global pe_srv, listener
    print("POINTING HANDS NODE BY ITZEL.................(๑╹ω╹๑ )")
    rospy.init_node("pointing_hand_srv")
    rospy.Service("/vision/human_pose_estimation/pointing_hands_status", FindPerson, callback) 

    rospy.wait_for_service("/pose_estimator_srv")
    pe_srv = rospy.ServiceProxy("/pose_estimator_srv", HumanPoseEstimatorResult)
    
    #pe_msg = HumanPoseEstimatorResultRequest()
    listener = tf.TransformListener()

  
    loop = rospy.Rate(10)
    while not rospy.is_shutdown():
        
        #clothes_color_srv(pe_srv(pe_msg))



        loop.sleep()

if __name__ == '__main__':
    main()