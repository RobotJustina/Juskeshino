#!/usr/bin/env python3

import rospy
import math
import numpy as np
import tf.transformations as tft
import tf2_ros
import tf
from geometry_msgs.msg import PoseStamped, Point, PointStamped, Pose
from vision_msgs.srv import *
from manip_msgs.srv import *
from visualization_msgs.msg import Marker, MarkerArray
import geometry_msgs.msg

MAXIMUM_GRIP_LENGTH = 0.17

def pose_actual_to_pose_target(pose, f_actual, f_target):
    global listener
    poseStamped_msg = PoseStamped()  
    poseStamped_msg.header.frame_id = f_actual   # frame de origen
    poseStamped_msg.header.stamp = rospy.Time(0)  # la ultima transformacion
    poseStamped_msg.pose = pose
    new_poseStamped = listener.transformPose(f_target, poseStamped_msg)
    new_pose = new_poseStamped.pose
    return new_pose


def points_actual_to_points_target(point_in, f_actual, f_target):
    global listener
    point_msg = PointStamped()  
    point_msg.header.frame_id = f_actual   # frame de origen
    point_msg.header.stamp = rospy.Time(0)  # la ultima transformacion
    point_msg.point.x = point_in[0]
    point_msg.point.y = point_in[1]
    point_msg.point.z = point_in[2]
    point_target_frame = listener.transformPoint(f_target, point_msg)
    new_point = point_target_frame.point
    return [ new_point.x , new_point.y , new_point.z ]




def obj_grip(grip_point , obj_pose, rotacion, obj_state , object_frame):
    """
        grip_point: punto origen de los sistemas candidatos, entra en el sistema base_link
        obj_pose:   orientacion del objeto en msg Pose en frame 'base_link'
        rotacion:   roll , pitch o yaw: 'R', 'P', 'Y', tipo de rotacion a realizar en el object_frame
        obj_state: 'horizontal' o 'vertical'
        object_frame: el frame en el cual se van a generar los candidatos
    """
    grasp_candidates_quaternion = []
    candidate_grasp = Pose()
    marker_array_publish(grip_point , 'base_link', 0, 7)    # publica el punto de agarre para brazo izquierdo

    if obj_state == "horizontal": 
        num_candidates = 5
        grip_point[2] = grip_point[2] + 0.05  # 12 cm por encima del objeto (z_base_link)

    

    else: num_candidates = 3
    #marker_array_publish(grip_point , 'base_link', 0, 7)    # publica el punto de agarre para brazo izquierdo
    obj_pose_frame_object = pose_actual_to_pose_target(obj_pose , 'base_link', object_frame) # pose en frame 'object'
    
    R, P, Y = tft.euler_from_quaternion([obj_pose_frame_object.orientation.x ,  # pose expresada en RPY
                                             obj_pose_frame_object.orientation.y ,
                                             obj_pose_frame_object.orientation.z, 
                                             obj_pose_frame_object.orientation.w])
    for j in range(num_candidates):      # genera 5 candidatos
        q_gripper = tft.quaternion_from_euler(R,P,Y,'sxyz')  # Pose en frame 'object' cuaterniones
        obj_pose_frame_object.orientation.x = q_gripper[0]
        obj_pose_frame_object.orientation.y = q_gripper[1]
        obj_pose_frame_object.orientation.z = q_gripper[2]
        obj_pose_frame_object.orientation.w = q_gripper[3]
        if rotacion == "P": P = P + np.deg2rad(-14) #horizontal grip
        else:
            if rotacion == "R": R = R + np.deg2rad(10)  #vertical grip
        candidate_grasp = pose_actual_to_pose_target(obj_pose_frame_object , object_frame, 'base_link')    # pose en frame 'base_link'
        candidate_grasp.position.x = grip_point[0] 
        candidate_grasp.position.y = grip_point[1] 
        candidate_grasp.position.z = grip_point[2] 
        broadcaster_frame_object('base_link', 'test_candidates'+str(j)+obj_state , candidate_grasp )  # emite la pose en 'base_link'
        rospy.sleep(1.0)
        grasp_candidates_quaternion.append(candidate_grasp )     # guarda el candidato en frame bl

    print("len candidates grip box", len(grasp_candidates_quaternion))
    return grasp_candidates_quaternion



def grip_rules(obj_pose, type_obj, obj_state, size):
    if type_obj == 'box': return box(obj_pose, size, obj_state )
    else: return prism(obj_pose, obj_state)
        

    if size.y <= MAXIMUM_GRIP_LENGTH:
        print("size object < MAX LENGHT GRIP")
        grasp_candidates_quaternion = prism(obj_pose, obj_state, angle, size)
        return grasp_candidates_quaternion 
    else:
        print("size object > MAX LENGHT GRIP")
        return box(obj_pose, size, obj_state)



def box(obj_pose, size, obj_state):
    grasp_candidates_quaternion = []
    #grip_point = [0, 0, size.z/3]      # punto en frame object
    if obj_state == 'horizontal':   # se elige el punto mas alto 
        print("Horizontal box")
        """
        MT = tft.quaternion_matrix([obj_pose.orientation.x ,obj_pose.orientation.y, obj_pose.orientation.z, obj_pose.orientation.w])
        axis_x_obj = np.asarray( [MT[0,0], MT[1,0], MT[2,0]]) 
        axis_z_point = [0,0,1]  # garantiza que agarre es completamente horizontal
        axis_y_point = np.cross(axis_z_point , axis_x_obj) / np.linalg.norm( np.cross(axis_z_point , axis_x_obj) )
        RM = np.asarray( [axis_x_obj, axis_y_point, axis_z_point] ) 
        RM = RM.T
        TM = [[RM[0,0], RM[0,1] , RM[0,2], 0],  # construccion de la matriz de transformación del objeto
              [RM[1,0], RM[1,1] , RM[1,2], 0],
              [RM[2,0], RM[2,1] , RM[2,2], 0], 
              [      0,        0,       0, 1]]
        
        gripper_quaternion = tft.quaternion_from_matrix( TM ) 
        obj_pose.orientation.x = gripper_quaternion[0]
        obj_pose.orientation.y = gripper_quaternion[1]
        obj_pose.orientation.z = gripper_quaternion[2]
        obj_pose.orientation.w = gripper_quaternion[3]
        """
        #broadcaster_frame_object('base_link', 'horizontal_box' , obj_pose )  # emite la pose en 'base_link'
        #rospy.sleep(1.0)
        grip_point = points_actual_to_points_target([0, 0, size.z/3], 'object', 'base_link')
        return obj_grip(grip_point , obj_pose, "P", obj_state , 'object')

    else:  # VERTICAL object
        grip_point1 = points_actual_to_points_target([0, 0, size.z/3] , 'object', 'base_link')
        grip_point2 = points_actual_to_points_target([size.x/3, 0, 0]  , 'object', 'base_link')     # punto en frame object
        poses_list1 = obj_grip(grip_point1 , obj_pose, "R", obj_state ,'object')
        obj_pose_frame_object = pose_actual_to_pose_target(obj_pose , 'base_link', 'object') # pose en frame 'object'
        R, P, Y = tft.euler_from_quaternion([obj_pose_frame_object.orientation.x ,  # pose expresada en RPY
                                             obj_pose_frame_object.orientation.y ,
                                             obj_pose_frame_object.orientation.z, 
                                             obj_pose_frame_object.orientation.w])
        P = P + np.deg2rad(90)
        q_gripper = tft.quaternion_from_euler(R,P,Y,'sxyz')  # Pose en frame 'object' cuaterniones
        obj_pose_frame_object.orientation.x = q_gripper[0]
        obj_pose_frame_object.orientation.y = q_gripper[1]
        obj_pose_frame_object.orientation.z = q_gripper[2]
        obj_pose_frame_object.orientation.w = q_gripper[3]
        obj_pose_frame_bl = pose_actual_to_pose_target(obj_pose , 'base_link', 'object')
        broadcaster_frame_object('base_link', 'test', obj_pose_frame_bl)  # emite la pose en 'base_link'
        rospy.sleep(1.0)

        poses_list2 = obj_grip(grip_point2 , obj_pose_frame_bl, "P", 'horizontal' ,'object')


        return poses_list1 + poses_list2



def prism(obj_pose, obj_state):   
    global listener
    grasp_candidates_quaternion = []
    epsilon = 0.01 # radio de la circunferencia

    obj_centroid = np.asarray([obj_pose.position.x , obj_pose.position.y, obj_pose.position.z]) # origen de la circunferencia
    MT = tft.quaternion_matrix([obj_pose.orientation.x ,obj_pose.orientation.y, obj_pose.orientation.z, obj_pose.orientation.w])
    axis_x_obj = np.asarray( [MT[0,0], MT[1,0], MT[2,0]])   # eje x del objeto vector normal al plano que corta al objet
    
    step_size = np.deg2rad(20)
    range_points = np.deg2rad(360)          # rango dentro del cual se generan los candidatos 360 grados
    num_points = int(range_points / step_size) 
    theta_offset = np.deg2rad(0)
    theta = theta_offset
    count = 0
    id = 0
    points = []

    axis_x_point = axis_x_obj / np.linalg.norm( (axis_x_obj) )  # solo toma la 1pca para crear frame de objeto

    if obj_state == 'horizontal':  # *******************************************************************
        print("Horizontal grip prism............")
        grip_point = [obj_pose.position.x , obj_pose.position.y, obj_pose.position.z]  

        axis_z_point = [0,0,1]  # garantiza que agarre es completamente horizontal
        axis_y_point = np.cross(axis_z_point , axis_x_point) / np.linalg.norm( np.cross(axis_z_point , axis_x_point) )
        RM = np.asarray( [axis_x_point, axis_y_point, axis_z_point] ) 
        RM = RM.T
        TM = [[RM[0,0], RM[0,1] , RM[0,2], 0],  # construccion de la matriz de transformación del objeto
              [RM[1,0], RM[1,1] , RM[1,2], 0],
              [RM[2,0], RM[2,1] , RM[2,2], 0], 
              [      0,        0,       0, 1]]
        
        gripper_quaternion = tft.quaternion_from_matrix( TM ) 
        obj_pose.orientation.x = gripper_quaternion[0]
        obj_pose.orientation.y = gripper_quaternion[1]
        obj_pose.orientation.z = gripper_quaternion[2]
        obj_pose.orientation.w = gripper_quaternion[3]
        broadcaster_frame_object('base_link', 'horizontal_prism' , obj_pose )  # emite la pose en 'base_link'
        rospy.sleep(1.0)
        return obj_grip(grip_point , obj_pose, "P", obj_state ,  'horizontal_prism')
        
    else:
        print("Vertical grip.................")
        for i in range( num_points):   # generación de puntos alrededor del objeto
            point = np.asarray([ 0, epsilon*np.sin(theta), epsilon*np.cos(theta)  ])
            point = points_actual_to_points_target(point, 'object', 'base_link')
            points.append(point)
            marker_array_publish(point, 'base_link', count, id)
            count += 1
            id += 1
            theta = theta - step_size 
        print("number of points", len(points))
        j = 0

        for p in points:   # generacion de frames candidatos
            axis_z_point = (p - obj_centroid ) / np.linalg.norm( (p - obj_centroid ) )
            axis_y_point = np.cross(axis_z_point , axis_x_point) / np.linalg.norm( np.cross(axis_z_point , axis_x_point) )
            # los cuaterniones necesarios para generar el frame del gripper
            RM = np.asarray( [axis_x_point, axis_y_point, axis_z_point] ) 
            RM = RM.T
            TM = [[RM[0,0], RM[0,1] , RM[0,2], 0],
                [RM[1,0], RM[1,1] , RM[1,2],   0],
                [RM[2,0], RM[2,1] , RM[2,2],   0], 
                [      0,        0,       0,   1]]
        
            q_gripper = tft.quaternion_from_matrix( TM ) 
            candidate_grasp = Pose()
            candidate_grasp.position.x = p[0] 
            candidate_grasp.position.y = p[1] 
            candidate_grasp.position.z = p[2] 
            candidate_grasp.orientation.x = q_gripper[0]
            candidate_grasp.orientation.y = q_gripper[1]
            candidate_grasp.orientation.z = q_gripper[2]
            candidate_grasp.orientation.w = q_gripper[3]
            grasp_candidates_quaternion.append(candidate_grasp) 
            broadcaster_frame_object('base_link', 'vertical_prism'+str(j) , candidate_grasp )  # emite la pose en 'base_link'
            rospy.sleep(1.0)
            j += 1
        print("number of candidates vertical grip prism", len(grasp_candidates_quaternion))
        return grasp_candidates_quaternion
        


def arow_marker(p_offset, p1, frame_id, ns, id, color):  # P1 y P2
    point0 = Point()
    point0.x = p_offset[0] 
    point0.y = p_offset[1] 
    point0.z = p_offset[2] 
    marker1 = Marker()
    marker1.header.frame_id = frame_id
    marker1.type = Marker.ARROW
    marker1.ns = ns
    marker1.header.stamp = rospy.Time.now()
    marker1.action = marker1.ADD
    marker1.id = id
    # body radius, Head radius, hight head
    marker1.scale.x, marker1.scale.y, marker1.scale.z = 0.009, 0.05, 0.02 
    marker1.color.r , marker1.color.g , marker1.color.b, marker1.color.a = color, 0.0, 100.0, 1.0
    point1 = Point()
    point1.x = p_offset[0] + p1[0]
    point1.y = p_offset[1] + p1[1]
    point1.z = p_offset[2] + p1[2]
    marker1.points = [point0, point1]
    marker_pub.publish(marker1)



def marker_array_publish(pointxyz, target_frame, count, id):
    #print("point grip", pointxyz)
    global marker_array_pub
    MARKERS_MAX = 100
    marker_array = MarkerArray()
    marker = Marker()
    marker.header.frame_id = target_frame
    marker.type = marker.SPHERE
    marker.action = marker.ADD
    marker.scale.x , marker.scale.y,marker.scale.z = 0.03, 0.03, 0.03
    marker.color.a , marker.color.r, marker.color.g, marker.color.b = 1.0, 1.0 , 0.0 ,1.0
    marker.pose.orientation.w = 1.0
    marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = pointxyz[0], pointxyz[1], pointxyz[2]
    
    if(count > MARKERS_MAX): marker_array.markers.pop(0)
    marker_array.markers.append(marker)
    # Renumber the marker IDs
    for m in marker_array.markers:
        m.id = id
        id += 1
    # Publish the MarkerArray
    marker_array_pub.publish(marker_array)



def broadcaster_frame_object(frame, child_frame, pose):
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()
    t.header.frame_id = frame
    t.child_frame_id = child_frame 
    t.header.stamp = rospy.Time.now()
    t.transform.translation.x = pose.position.x
    t.transform.translation.y = pose.position.y
    t.transform.translation.z = pose.position.z
    t.transform.rotation.x = pose.orientation.x
    t.transform.rotation.y = pose.orientation.y
    t.transform.rotation.z = pose.orientation.z
    t.transform.rotation.w = pose.orientation.w
    br.sendTransform(t)
    print("graficando frames en Rviz..........")



def convert_frame_of_candidates_poses(pose_list_q, obj_state):
    new_pose_q_list = []
    new_pose_rpy_list = []
    for pos in pose_list_q:
        new_pose = pose_actual_to_pose_target(pos, 'base_link' , 'shoulders_left_link')
        new_pose_q_list.append(new_pose)
        x , y, z = new_pose.position.x , new_pose.position.y , new_pose.position.z
        roll,pitch,yaw = tft.euler_from_quaternion( [new_pose.orientation.x , new_pose.orientation.y , 
                                          new_pose.orientation.z , new_pose.orientation.w ])
        pose_shoulder_frame = np.asarray([x ,y ,z , roll, pitch , yaw])
        new_pose_rpy_list.append(pose_shoulder_frame)
    return new_pose_rpy_list


    
def evaluating_possibility_grip(pose_rpy, pose_quaternion, obj_state):
    """
        Evalua la existencia de los candidatos de agarre en el espacio articular y regresa una trayectoria
        desde la posicion actual del grippe hasta el origen del frame candidato aprobado h
    """
    ik_msg = InverseKinematicsPose2TrajRequest()
    print("Evaluating the possibility of grip given the position of the object...")
    i = 0
    for pose1 in pose_rpy:  
        ik_msg.x = pose1[0] 
        ik_msg.y = pose1[1]
        ik_msg.z = pose1[2] + 0.04
        ik_msg.roll = pose1[3]
        ik_msg.pitch = pose1[4]
        ik_msg.yaw = pose1[5]
        ik_msg.duration = 5
        ik_msg.time_step = 0.02
        
        try: 
            resp_ik_srv = ik_srv(ik_msg)    # Envia al servicio de IK
            print("Suitable pose found.....................")
            if obj_state == 'horizontal':
                # genera una segunda trayectoria en vertical
                print("ULTIMO PUNTO DE LA TRAYECTORIA", resp_ik_srv.articular_trajectory.points[-1].positions[0])
                guess = [resp_ik_srv.articular_trajectory.points[-1].positions[0],
                         resp_ik_srv.articular_trajectory.points[-1].positions[1],
                         resp_ik_srv.articular_trajectory.points[-1].positions[2],
                         resp_ik_srv.articular_trajectory.points[-1].positions[3],
                         resp_ik_srv.articular_trajectory.points[-1].positions[4],
                         resp_ik_srv.articular_trajectory.points[-1].positions[5],
                         resp_ik_srv.articular_trajectory.points[-1].positions[6]]
                
                ik_msg.x = pose1[0] 
                ik_msg.y = pose1[1]
                ik_msg.z = pose1[2] - 0.05
                ik_msg.roll = pose1[3]
                ik_msg.pitch = pose1[4]
                ik_msg.yaw = pose1[5]
                ik_msg.duration = 3
                ik_msg.time_step = 0.02
                ik_msg.initial_guess = guess

                resp_2_ik_srv = ik_srv(ik_msg)    # Envia al servicio de IK
                print("Second trajectory found.....................")
                resp_ik_srv.articular_trajectory.points = resp_ik_srv.articular_trajectory.points + resp_2_ik_srv.articular_trajectory.points
                return resp_ik_srv.articular_trajectory , pose_quaternion[i] , pose1, True
            
            return resp_ik_srv.articular_trajectory , pose_quaternion[i] , pose1, True

        except:
            i = i + 1 
            print("Pose no apta........")  
            continue
    return None, None, None, False



def callback(req):
    global listener, ik_srv
    resp = BestGraspTrajResponse()
    obj_state = req.recog_object.main_axis_tilt     #"horizontal"
    print("OBJ STATE ", obj_state)

    pose_list_q = grip_rules(req.recog_object.pose, req.recog_object.category, obj_state, req.recog_object.size )
    print("lista d eposes", len(pose_list_q))
    if len(pose_list_q) <= 0:
        print("object is no graspable")
        return resp
    
    candidates_poses_rpy = convert_frame_of_candidates_poses(pose_list_q, obj_state)
    trajectory, pose, rpy_pose, graspable = evaluating_possibility_grip(candidates_poses_rpy , pose_list_q, obj_state)

    if graspable:
        broadcaster_frame_object('base_link', 'suitable_pose' , pose)
        rospy.sleep(1.0)
        resp.articular_trajectory = trajectory
        resp.graspable = True
        return resp

    else: 
        print("No se encontraron poses posibles...................")
        resp.graspable = False
        return resp



def main():
    global listener , ik_srv, marker_pub, marker_array_pub
    print("Node to grab objects based on their orientation..............ʕ•ᴥ•ʔ")
    rospy.init_node("gripper_orientation_for_grasping")
    rospy.Service("/vision/get_best_grasp_traj", BestGraspTraj, callback)
    listener = tf.TransformListener()
    # se suscribe al servicio /manipulation/ik_trajectory
    rospy.wait_for_service( '/manipulation/la_ik_trajectory' )
    ik_srv = rospy.ServiceProxy( '/manipulation/la_ik_trajectory' , InverseKinematicsPose2Traj )
    marker_pub = rospy.Publisher("/vision/object_recognition/markers", Marker, queue_size = 10) 
    marker_array_pub = rospy.Publisher("/vision/obj_reco/marker_array", MarkerArray, queue_size = 10) 
    
    loop = rospy.Rate(30)
    while not rospy.is_shutdown():
        loop.sleep()

if __name__ == '__main__':
    main()


