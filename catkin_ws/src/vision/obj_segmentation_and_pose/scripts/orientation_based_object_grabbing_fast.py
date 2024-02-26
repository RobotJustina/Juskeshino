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

MAXIMUM_GRIP_LENGTH = 0.15


def move_base_to_improve_grip(coord_object): #posición del objeto en el frame 'object'
    """
    if (coord_object[1] < 0.03): # Evalua la posición del objeto en y, si es menor a un valor
        #mueve base a la derecha
        while coord_object
    """
    print("lalalala")
    





def pose_actual_to_pose_target(pose, f_actual, f_target):
    global listener
    poseStamped_msg = PoseStamped()  
    poseStamped_msg.header.frame_id = f_actual   # frame de origen
    poseStamped_msg.header.stamp = rospy.Time()  # la ultima transformacion
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


def flat_objects_grip(obj_pose, size_obj):
    grip_point = []     # se define el punto de agarre
    # Si 1pca < 0.1 m : small_obj()
    # si no:            top_grip()
    return 



def generates_candidates(grip_point , obj_pose, rotacion, obj_state , name_frame, step, num_candidates):    # Cambiar nombre por generates_candidates
    """
        grip_point:     Vector que contiene  el punto origen de los sistemas candidatos, entra en el frame 'object'.
        obj_pose:       Orientacion del objeto en msg Pose en frame 'object', expresado en cuaterniones.
        rotacion:       Es un string: roll , pitch o yaw: 'R', 'P', 'Y', tipo de rotacion a realizar en el object_frame.
        obj_state:      Es un string que  indica si el objeto esta 'horizontal' o 'vertical'.
        object_frame:   Es el frame en el cual se van a generar los candidatos de retorno.
        step:           Grados de distancia entre un candidato y otro, pueden ser negativos o positivos.
        num_candidates: Es la cantidad de candidatos que se desea generar.

        Retorna una lista de poses candidatas expresadas en cuaterniones (msg Pose)
    """
    grasp_candidates_quaternion = []
    j = 0
    print("GRIP POINT:______", grip_point)

    if obj_state == "horizontal": 
        grip_point_bl = points_actual_to_points_target(grip_point, 'object', 'base_link')
        grip_point_bl[2] = grip_point_bl[2] + 0.13  # 10 cm por encima del objeto (z_base_link)
        grip_point = points_actual_to_points_target(grip_point_bl, 'base_link', 'object')
        marker_array_publish(grip_point, 'object', 59, 56)
        

    obj_pose_frame_object = obj_pose 

    R, P, Y = tft.euler_from_quaternion([obj_pose_frame_object.orientation.x ,  # pose expresada en RPY para realizar rotaciones
                                             obj_pose_frame_object.orientation.y ,
                                             obj_pose_frame_object.orientation.z, 
                                             obj_pose_frame_object.orientation.w])
    print("RPY: ",R,P,Y)
    
    for j in range(num_candidates):      # genera candidatos
        q_gripper = tft.quaternion_from_euler(R,P,Y,'sxyz')  # Pose en frame 'object' cuaterniones
        obj_pose_frame_object.orientation.x = q_gripper[0]
        obj_pose_frame_object.orientation.y = q_gripper[1]
        obj_pose_frame_object.orientation.z = q_gripper[2]
        obj_pose_frame_object.orientation.w = q_gripper[3]
        
        if rotacion == "P": 
            P = P + np.deg2rad(step) #horizontal grip
            #print("Rotacion en Pitch")

        else:
            if rotacion == "R": R = R + np.deg2rad(step)  #vertical grip
            #print("Rotacion en Roll")

        
        obj_pose_frame_object.position.x = grip_point[0]
        obj_pose_frame_object.position.y = grip_point[1]
        obj_pose_frame_object.position.z = grip_point[2]

        if debug:
            #print("Graficando " + name_frame+str(j)+obj_state)
            broadcaster_frame_object('object', name_frame+str(j)+obj_state , obj_pose_frame_object )

        grasp_candidates_quaternion.append(obj_pose_frame_object )     # guarda el candidato en frame bl

    return grasp_candidates_quaternion  # REGRESA A LOS CANDIDATOS EN FRAME OBJECT




def grip_rules(obj_pose, type_obj, obj_state, size, grip_point):

    if (size.z <= MAXIMUM_GRIP_LENGTH) and (size.y <= MAXIMUM_GRIP_LENGTH) and (size.x >= 0.13):
        print("The object will be GRABBED AS PRISM..................")
        return prism(obj_pose, obj_state)
    else:
        if size.x < 0.13:
            print("Object too SMALL, SUPERIOR GRIP will be made")
            return small_obj(obj_pose, obj_state , grip_point)
            
        else:
            print("size object > MAX LENGHT GRIP")
            print("The object will be GRABBED as BOX....................")
            return box(obj_pose, size, obj_state )




def top_grip(grip_point): 
    global debug  
    #obj_pose_frame_object = pose_actual_to_pose_target(obj_pose , 'base_link', 'object') # Transforma pose en frame 'object' para generar candidatos

    obj_state = "horizontal"
    # Primera lista de candidatos******************************************************************************
    obj_pose_1 = Pose()
    obj_pose_1.position.x, obj_pose_1.position.y, obj_pose_1.position.z = 0, 0, 0
    obj_pose_1.orientation.x = 0.0
    obj_pose_1.orientation.y = 0.0
    obj_pose_1.orientation.z = 0.0
    obj_pose_1.orientation.w = 1.0

    pose_list1 = generates_candidates(grip_point , obj_pose_1, "P", obj_state ,  'c1', step = -12, num_candidates = 6)
        

    # Segunda lista de candidatos******************************************************************************
    obj_pose_2 = Pose()
    obj_pose_2.position.x, obj_pose_2.position.y, obj_pose_2.position.z = 0, 0, 0
    obj_pose_2.orientation.x = 0.0
    obj_pose_2.orientation.y = 0.0
    obj_pose_2.orientation.z = 0.0
    obj_pose_2.orientation.w = 1.0
        
    pose_list2 = generates_candidates(grip_point , obj_pose_2, "P", obj_state , 'C2', step = 10, num_candidates = 5)
        
    # Tercera lista de candidatos.................................
    obj_pose_3 = Pose()
    obj_pose_3.position.x, obj_pose_3.position.y, obj_pose_3.position.z = 0, 0, 0
    obj_pose_3.orientation.x = 0.0
    obj_pose_3.orientation.y = 0.0
    obj_pose_3.orientation.z = 0.0
    obj_pose_3.orientation.w = 1.0
        
    #print("obj pose 3", obj_pose_3)
        
    R, P, Y = tft.euler_from_quaternion([obj_pose_3.orientation.x ,  # pose expresada en RPY para realizar rotaciones
                                            obj_pose_3.orientation.y ,
                                            obj_pose_3.orientation.z, 
                                            obj_pose_3.orientation.w])
    Y = Y + np.deg2rad(180) # Realiza un yaw de 180 grados 
        
        
    q_gripper = tft.quaternion_from_euler(R,P,Y,'sxyz')  # Pose en frame 'object' cuaterniones
    obj_pose_3.orientation.x = q_gripper[0]
    obj_pose_3.orientation.y = q_gripper[1]
    obj_pose_3.orientation.z = q_gripper[2]
    obj_pose_3.orientation.w = q_gripper[3]
    obj_pose_3.position.x = 0
    obj_pose_3.position.y = 0
    obj_pose_3.position.z = 0
        
    pose_list3 = generates_candidates(grip_point, obj_pose_3 , "P", obj_state , 'c3', step = -10, num_candidates = 4)

    # Cuarta lista de candidatos.................................
    obj_pose_4 = Pose()
    obj_pose_4.position.x, obj_pose_4.position.y, obj_pose_4.position.z = 0, 0, 0
    obj_pose_4.orientation.x = 0.0
    obj_pose_4.orientation.y = 0.0
    obj_pose_4.orientation.z = 0.0
    obj_pose_4.orientation.w = 1.0
        
    R, P, Y = tft.euler_from_quaternion([obj_pose_4.orientation.x ,  # pose expresada en RPY para realizar rotaciones
                                            obj_pose_4.orientation.y ,
                                            obj_pose_4.orientation.z, 
                                            obj_pose_4.orientation.w])
    Y = Y + np.deg2rad(180) # Realiza un yaw de 180 grados 
            
    q_gripper = tft.quaternion_from_euler(R,P,Y,'sxyz')  # Pose en frame 'object' cuaterniones
    obj_pose_4.orientation.x = q_gripper[0]
    obj_pose_4.orientation.y = q_gripper[1]
    obj_pose_4.orientation.z = q_gripper[2]
    obj_pose_4.orientation.w = q_gripper[3]
    obj_pose_4.position.x = 0
    obj_pose_4.position.y = 0
    obj_pose_4.position.z = 0

    pose_list4 = generates_candidates(grip_point , obj_pose_4 , "P", obj_state , 'c4', step = 10, num_candidates = 4)

    print("Num Candidates:____", len(pose_list2 + pose_list1 + pose_list3 + pose_list4))
        
    return pose_list2 + pose_list1 + pose_list3 + pose_list4




def small_obj(obj_pose, obj_state, grip_point):
    global debug

    obj_state = "horizontal"
    
    # Primera lista de candidatos******************************************************************************
    obj_pos_1 = Pose()
    obj_pos_1.position.x, obj_pos_1.position.y, obj_pos_1.position.z = 0, 0, 0
    obj_pos_1.orientation.x = 0.0
    obj_pos_1.orientation.y = 0.0
    obj_pos_1.orientation.z = 0.0
    obj_pos_1.orientation.w = 1.0
    pose_list1 = generates_candidates([0,0,0] , obj_pos_1, "P", obj_state ,  'c1', step = -12, num_candidates = 7)
    
    # Segunda lista de candidatos******************************************************************************
    obj_pose_2 = Pose()
    obj_pose_2.position.x, obj_pose_2.position.y, obj_pose_2.position.z = 0, 0, 0
    obj_pose_2.orientation.x = 0.0
    obj_pose_2.orientation.y = 0.0
    obj_pose_2.orientation.z = 0.0
    obj_pose_2.orientation.w = 1.0

    R, P, Y = tft.euler_from_quaternion([obj_pose_2.orientation.x ,  # pose expresada en RPY para realizar rotaciones
                                                obj_pose_2.orientation.y ,
                                                obj_pose_2.orientation.z, 
                                                obj_pose_2.orientation.w])
    Y = Y + np.deg2rad(45) # Realiza un yaw de 90 grados 
        
        
    q_gripper = tft.quaternion_from_euler(R,P,Y,'sxyz')  # Pose en frame 'object' cuaterniones
    obj_pose_2.orientation.x = q_gripper[0]
    obj_pose_2.orientation.y = q_gripper[1]
    obj_pose_2.orientation.z = q_gripper[2]
    obj_pose_2.orientation.w = q_gripper[3]
    obj_pose_2.position.x = 0
    obj_pose_2.position.y = 0
    obj_pose_2.position.z = 0
        
    pose_list2 = generates_candidates([0,0,0] , obj_pose_2, "P", obj_state , 'C2', step = -12, num_candidates = 7)
        

    # Tercera lista de candidatos.................................
    obj_pose_3 = Pose()
    obj_pose_3.position.x, obj_pose_3.position.y, obj_pose_3.position.z = 0, 0, 0
    obj_pose_3.orientation.x = 0.0
    obj_pose_3.orientation.y = 0.0
    obj_pose_3.orientation.z = 0.0
    obj_pose_3.orientation.w = 1.0
        
    R, P, Y = tft.euler_from_quaternion([obj_pose_3.orientation.x ,  # pose expresada en RPY para realizar rotaciones
                                                obj_pose_3.orientation.y ,
                                                obj_pose_3.orientation.z, 
                                                obj_pose_3.orientation.w])
    Y = Y + np.deg2rad(-90) # Realiza un yaw de 90 grados 
        
        
    q_gripper = tft.quaternion_from_euler(R,P,Y,'sxyz')  # Pose en frame 'object' cuaterniones
    obj_pose_3.orientation.x = q_gripper[0]
    obj_pose_3.orientation.y = q_gripper[1]
    obj_pose_3.orientation.z = q_gripper[2]
    obj_pose_3.orientation.w = q_gripper[3]
    obj_pose_3.position.x = 0
    obj_pose_3.position.y = 0
    obj_pose_3.position.z = 0
        
    pose_list3 = generates_candidates([0,0,0], obj_pose_3 , "P", obj_state , 'c3', step = -14, num_candidates = 7)
    print("Num Candidates:____", len(pose_list2 + pose_list1 + pose_list3 ))#+ pose_list4))        
    
    return pose_list1 + pose_list2 + pose_list3 #+ pose_list4
    




def box(obj_pose, size, obj_state):     # obj_pose  esta referenciada a 'base_link'
    global debug

    # HORIZONTAL **************************************************************************************************************************************
    if obj_state == 'horizontal':  
        print("Horizontal box")
        # genera punto de agarre superior ************************

        # Primera lista de candidatos******************************************************************************
        """
        obj_pos_1 = Pose()
        obj_pos_1.position.x, obj_pos_1.position.y, obj_pos_1.position.z = 0, 0, 0
        obj_pos_1.orientation.x = 0.0
        obj_pos_1.orientation.y = 0.0
        obj_pos_1.orientation.z = 0.0
        obj_pos_1.orientation.w = 1.0
        """
        grip_point_top = [0, 0, size.z/3]

        #pose_list1 = generates_candidates(grip_point_top , obj_pos_1, "P", obj_state ,  'c1', step = -12, num_candidates = 7)
        candidates_h_list_1 = top_grip(grip_point_top) 

        # Segunda lista de candidatos****VERTICAL GRIP**************************************************************************
        obj_pose_2 = Pose()
        obj_pose_2.position.x, obj_pose_2.position.y, obj_pose_2.position.z = 0, 0, 0
        obj_pose_2.orientation.x = 0.0
        obj_pose_2.orientation.y = 0.0
        obj_pose_2.orientation.z = 0.0
        obj_pose_2.orientation.w = 1.0

        R, P, Y = tft.euler_from_quaternion([obj_pose_2.orientation.x ,  # pose expresada en RPY para realizar rotaciones
                                                    obj_pose_2.orientation.y ,
                                                    obj_pose_2.orientation.z, 
                                                    obj_pose_2.orientation.w])
        P = P + np.deg2rad(-90) 
            
            
        q_gripper = tft.quaternion_from_euler(R,P,Y,'sxyz')  # Pose en frame 'object' cuaterniones
        obj_pose_2.orientation.x = q_gripper[0]
        obj_pose_2.orientation.y = q_gripper[1]
        obj_pose_2.orientation.z = q_gripper[2]
        obj_pose_2.orientation.w = q_gripper[3]
        obj_pose_2.position.x = 0
        obj_pose_2.position.y = 0
        obj_pose_2.position.z = 0
        # genera punto de agarre lateral
        grip_point_side = [-size.x/3, 0, 0]
            
        candidates_h_list_2 = generates_candidates(grip_point_side , obj_pose_2, "P", 'vertical' , 'C2', step = -12, num_candidates = 4)

        return candidates_h_list_1  + candidates_h_list_2

    
    # VERTICAL GRASP *************************************************************************************************************************************************
    else:  # VERTICAL object, lateral grip

        # Primera lista de candidatos******************************************************************************
        obj_pos_1 = Pose()
        obj_pos_1.position.x, obj_pos_1.position.y, obj_pos_1.position.z = 0, 0, 0
        obj_pos_1.orientation.x = 0.0
        obj_pos_1.orientation.y = 0.0
        obj_pos_1.orientation.z = 0.0
        obj_pos_1.orientation.w = 1.0
        grip_point_side = [0, 0, size.z/4]
        candidates_v_list = generates_candidates(grip_point_side , obj_pose, "R", obj_state ,'object', step = 10, num_candidates = 5)  

        return candidates_v_list # En frame 'object'



def prism(obj_pose, obj_state):   #pose del objeto en frame 'object'
    global listener, debug
    grasp_candidates_quaternion = []
    epsilon = 0.01 # radio de la circunferencia

    obj_centroid = [0,0,0]#np.asarray([obj_pose.position.x , obj_pose.position.y, obj_pose.position.z]) # origen de la circunferencia
    axis_x_candidate = [1,0,0]#np.asarray( [MT[0,0], MT[1,0], MT[2,0]])   # eje x del objeto vector normal al plano que corta al objet
    
    step_size = np.deg2rad(15)
    range_points = np.deg2rad(360)          # rango dentro del cual se generan los candidatos 360 grados
    num_points = int(range_points / step_size) 
    theta_offset = np.deg2rad(-10)
    theta = theta_offset
    count = 0
    id = 0
    points = []

    # HORIZONTAL **************************************************************************************************
    if obj_state == 'horizontal': 
        print("Horizontal grip prism............")  # Agarre horizontal
        grip_point = [0,0,0]#[obj_pose.position.x , obj_pose.position.y, obj_pose.position.z] #debe estar en el frame 'base_link'
        return top_grip(grip_point)

    # VERTICAL ***************************************************************************************************    
    else:
        print("Vertical grip.................")
        for i in range( num_points):   # generación de puntos alrededor del objeto
            point = np.asarray([ 0, epsilon*np.sin(theta), epsilon*np.cos(theta)  ])
            #point = points_actual_to_points_target(point, 'object', 'base_link')
            points.append(point)
            marker_array_publish(point, 'object', count, id)
            count += 1
            id += 1
            theta = theta - step_size 
        print("number of points", len(points))
        j = 0

        for p in points:   # generacion de frames candidatos
            axis_z_candidate = (p - obj_centroid ) / np.linalg.norm( (p - obj_centroid ) )
            axis_y_candidate = np.cross(axis_z_candidate , axis_x_candidate) / np.linalg.norm( np.cross(axis_z_candidate , axis_x_candidate) )
            
            # Matriz de transformacion que representa al frame candidato creado en el frame 'object'
            RM = np.asarray( [axis_x_candidate, axis_y_candidate, axis_z_candidate] ) 
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
            if debug:
                print("vertical_prism"+str(j))
                broadcaster_frame_object('object', 'vertical_prism'+str(j) , candidate_grasp ) 
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



def convert_frame_of_candidates_poses(pose_list_q, o):
    """
        Convierte las poses de entrada que estan respecto de 'base_link' en poses
        con respecto a 'shoulders_left_link' 
        Argumentos de entrada
        pose_list_q: lista de poses candidatas en cuaterniones
        Argumentos de salida
        new_pose_rpy_list: 
    """
    new_pose_q_list = []        # Poses en cuaterniones para 
    new_pose_rpy_list = []      # Poses en RPY para el servicio de cinematica inversa
    for pos in pose_list_q: # para cada candidato de la lista de entrada
        new_pose = pose_actual_to_pose_target(pos, 'base_link' , 'shoulders_left_link')
        new_pose_q_list.append(new_pose)
        # Se extrae la informacion de la posicion del objeto respecto al frame de hombro izqu
        x , y, z = new_pose.position.x , new_pose.position.y , new_pose.position.z
        roll,pitch,yaw = tft.euler_from_quaternion( [new_pose.orientation.x , new_pose.orientation.y , 
                                          new_pose.orientation.z , new_pose.orientation.w ])
        pose_shoulder_frame = np.asarray([x ,y ,z , roll, pitch , yaw])
        new_pose_rpy_list.append(pose_shoulder_frame)
    return new_pose_rpy_list


def pose_for_ik_service(pose_in_frame_object):  # Regresa un arreglo [x,y,z,R,P,Y]

    new_pose = pose_actual_to_pose_target(pose_in_frame_object, 'object' , 'shoulders_left_link')    
    
    # Se extrae la informacion de la posicion del objeto respecto al frame de hombro izqu
    x , y, z = new_pose.position.x , new_pose.position.y , new_pose.position.z
    roll,pitch,yaw = tft.euler_from_quaternion( [new_pose.orientation.x , new_pose.orientation.y , 
                                          new_pose.orientation.z , new_pose.orientation.w ])
    pose_shoulder_frame = np.asarray([x ,y ,z , roll, pitch , yaw])
    candidate = pose_shoulder_frame

    return candidate, new_pose




    
def evaluating_possibility_grip(candidate_quaternion_list, obj_state):
    """
        Evalua la existencia de los candidatos de agarre en el espacio articular y regresa una trayectoria
        desde la posicion actual del gripper hasta el origen del primer frame candidato aprobado h
    """
    global ik_srv
    ik_msg = InverseKinematicsPose2TrajRequest()


    print("Evaluating grip candidates...")
    i = 0
    for pose1 in candidate_quaternion_list:  # rellena el mensaje para el servicio IK
        if obj_state == "vertical":
            pose_xyzrpy, new_pos = pose_for_ik_service(pose1)
            ik_msg.x = pose_xyzrpy[0] 
            ik_msg.y = pose_xyzrpy[1]
            ik_msg.z = pose_xyzrpy[2]
            ik_msg.roll = pose_xyzrpy[3]
            ik_msg.pitch = pose_xyzrpy[4]
            ik_msg.yaw = pose_xyzrpy[5]
            ik_msg.duration = 4
            ik_msg.time_step = 0.06
            try:
                resp_ik_srv = ik_srv(ik_msg)
                print("Suitable pose for vertical object found.....................")
                return resp_ik_srv.articular_trajectory , candidate_quaternion_list[i] , pose_xyzrpy, True
            except:
                i = i + 1 
                print("Discarded candidate")  
                continue


        else:   # objeto con 1pca horizontal
            pose1, new_pos = pose_for_ik_service(pose1)
            ik_msg.x = pose1[0] 
            ik_msg.y = pose1[1]
            ik_msg.z = pose1[2]
            ik_msg.roll = pose1[3]      # (0, -90,0)
            ik_msg.pitch = pose1[4]
            ik_msg.yaw = pose1[5]
            ik_msg.duration = 7
            ik_msg.time_step = 0.09
            try:    # intenta la primera trayectoria
                
                resp_ik_srv = ik_srv(ik_msg)    # Envia al servicio de IK
                print("Suitable pose 1 for horizontal object found.....................")
                #print("ultimo punto de la trayectoria")
                #print(resp_ik_srv.articular_trajectory.points[-1])
                #return resp_ik_srv.articular_trajectory , pose_quaternion[i] , pose1, True
            except:
                print("candidato no aprobado")
                continue
    
            try:
                print("genera una segunda trayectoria para objeto horizontal")
                # el ultimo punto de la 1a trayectoria es el primero de la segunda
                #print("ULTIMO PUNTO DE LA TRAYECTORIA", resp_ik_srv.articular_trajectory.points[-1].positions)
                guess = [resp_ik_srv.articular_trajectory.points[-1].positions[0],
                        resp_ik_srv.articular_trajectory.points[-1].positions[1],
                        resp_ik_srv.articular_trajectory.points[-1].positions[2],
                        resp_ik_srv.articular_trajectory.points[-1].positions[3],
                        resp_ik_srv.articular_trajectory.points[-1].positions[4],
                        resp_ik_srv.articular_trajectory.points[-1].positions[5],
                        resp_ik_srv.articular_trajectory.points[-1].positions[6]]
                    
                # Ultimo punto de la segunda trayectoria
                ik_msg.x = pose1[0] 
                ik_msg.y = pose1[1]
                ik_msg.z = pose1[2] - 0.06
                ik_msg.roll = pose1[3]
                ik_msg.pitch = pose1[4]
                ik_msg.yaw = pose1[5]
                ik_msg.duration = 2
                ik_msg.time_step = 0.02
                ik_msg.initial_guess = guess

                resp_2_ik_srv = ik_srv(ik_msg)    # Envia al servicio de IK
                print("Second trajectory found.....................")
                resp_ik_srv.articular_trajectory.points = resp_ik_srv.articular_trajectory.points + resp_2_ik_srv.articular_trajectory.points
                return resp_ik_srv.articular_trajectory , candidate_quaternion_list[i] , new_pos, True
            except:
                i = i + 1 
                print("Pose 2 no apta........")  
                continue
            
    return None, None, None, False



def callback(req):
    global listener #, ik_srv
    resp = BestGraspTrajResponse()              
    obj_state = req.recog_object.object_state    


    print("CENTROID:_____ ")
    print(req.recog_object.pose.position)
    print("BB CATEGORY:_____ ")
    print(req.recog_object.category)  
    print("STATE:_____ ", req.recog_object.object_state)
    print("SIZE:_____ ") 
    print(req.recog_object.size)

    grip_point = [req.recog_object.pose.position.x, req.recog_object.pose.position.y, req.recog_object.pose.position.z]

    # Retorna lista de candidatos en frame 'object'
    pose_list_quaternion = grip_rules(req.recog_object.pose, req.recog_object.category, obj_state, req.recog_object.size , grip_point )
    if len( pose_list_quaternion) <= 0:
        print("object is no graspable")
        return resp
    
    trajectory, pose, rpy_pose, graspable = evaluating_possibility_grip(pose_list_quaternion , obj_state)
   
    if graspable:
        print("Graphing in RViz suitable pose for object manipulation...")
        broadcaster_frame_object('object', 'suitable_pose' , pose)
        resp.articular_trajectory = trajectory
        resp.graspable = True
        return resp

    else: 
        print("No possible poses found :'(...................")
        resp.graspable = False
        return resp



def main():
    global listener , ik_srv, marker_pub, marker_array_pub, debug
    debug = True
    print("Node to grab objects based on their orientation by Iby..............ʕ•ᴥ•ʔ")
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


