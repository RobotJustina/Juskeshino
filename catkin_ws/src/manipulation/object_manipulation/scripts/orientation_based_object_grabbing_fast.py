#!/usr/bin/env python3

import rospy
import numpy as np
import tf.transformations as tft
import tf2_ros
import tf
import math
from geometry_msgs.msg import PoseStamped, Point, PointStamped, Pose
from vision_msgs.srv import *
from manip_msgs.srv import *
from visualization_msgs.msg import Marker, MarkerArray
import geometry_msgs.msg

MAXIMUM_GRIP_LENGTH = 0.14
MINIMUM_HEIGHT_PRISM = 0.13
MAXIMUM_CUBE_SIZE = 0.13
Z_OFFSET = 0.12
    

def descarte_forced_poses(obj_pose):   
    vector_z_obj = [obj_pose.position.x, obj_pose.position.y]

    R, P, Y = tft.euler_from_quaternion([obj_pose.orientation.x ,  # pose expresada en RPY para realizar rotaciones
                                             obj_pose.orientation.y ,
                                             obj_pose.orientation.z, 
                                             obj_pose.orientation.w])
    RM = tft.euler_matrix(R, P, Y)
    vector_z_obj = [RM[0][2] , RM[1][2]]
    angle_z_obj = np.rad2deg(math.atan2(vector_z_obj[1] , vector_z_obj[0]))

    if (angle_z_obj > 150) or (angle_z_obj < -150): descarte = False
    else:
        descarte = True
        print("GraspLa.-> Forced pose discarded")

    return descarte



def pose_actual_to_pose_target(pose, f_actual, f_target):
    global listener
    poseStamped_msg = PoseStamped()  
    poseStamped_msg.header.frame_id = "object"   # frame de origen
    poseStamped_msg.header.stamp = rospy.Time()  # la ultima transformacion
    poseStamped_msg.pose = pose

    try:
        listener.waitForTransform("object", "shoulders_left_link", rospy.Time(0), rospy.Duration(10.0))
        print("waitfor ..despues")
        new_poseStamped = listener.transformPose('shoulders_left_link', poseStamped_msg)
        new_pose = new_poseStamped.pose
        return new_pose
    
    except:
        print("Best_Grasp_Node.-> Could not get the pose in the desired frame")
        return -1


    

def points_actual_to_points_target(point_in, f_actual, f_target):
    global listener
    point_msg = PointStamped()  
    point_msg.header.frame_id = f_actual   # frame de origen
    point_msg.header.stamp = rospy.Time() # la ultima transformacion
    point_msg.point.x = point_in[0]
    point_msg.point.y = point_in[1]
    point_msg.point.z = point_in[2]

    listener.waitForTransform(f_actual, f_target, rospy.Time(), rospy.Duration())
    point_target_frame = listener.transformPoint(f_target, point_msg)
    new_point = point_target_frame.point
    return [ new_point.x , new_point.y , new_point.z ]




def generates_candidates(grip_point , obj_pose, rotacion, obj_state , name_frame, step, num_candidates, type_obj = None):    # Cambiar nombre por generates_candidates
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
    global debug 
    j = 0

    if obj_state == "horizontal":   # 13 cm por encima del objeto (z_base_link)
        grip_point_bl = points_actual_to_points_target(grip_point, 'object', 'base_link')

        if type_obj == "BOWL":
            grip_point_bl[2] = grip_point_bl[2] + 0.17

        else:grip_point_bl[2] = grip_point_bl[2] + Z_OFFSET

        grip_point = points_actual_to_points_target(grip_point_bl, 'base_link', 'object')
        if debug:
            marker_array_publish(grip_point, 'object', 59, 56)


    #obj_pose_frame_object = Pose()
    #obj_pose_frame_object = obj_pose
    grasp_candidates_quaternion = []
    q_gripper = []



    R, P, Y = tft.euler_from_quaternion([obj_pose.orientation.x ,  # pose expresada en RPY para realizar rotaciones
                                             obj_pose.orientation.y ,
                                             obj_pose.orientation.z, 
                                             obj_pose.orientation.w])
    
    for j in range(num_candidates):      # genera candidatos
        obj_pose_frame_object = Pose()
        obj_pose_frame_object.orientation.x = 0
        obj_pose_frame_object.orientation.y = 0
        obj_pose_frame_object.orientation.z = 0
        obj_pose_frame_object.orientation.w = 1
        obj_pose_frame_object.position.x = 0
        obj_pose_frame_object.position.y = 0
        obj_pose_frame_object.position.z = 0
        q_gripper = tft.quaternion_from_euler(R,P,Y,'sxyz')  # Pose en frame 'object' cuaterniones
        obj_pose_frame_object.orientation.x = q_gripper[0]
        obj_pose_frame_object.orientation.y = q_gripper[1]
        obj_pose_frame_object.orientation.z = q_gripper[2]
        obj_pose_frame_object.orientation.w = q_gripper[3]

    
        
        if rotacion == "P": 
            P = P + np.deg2rad(step) #horizontal grip
            #print("Best_Grasp_Node.-> Rotacion en Pitch", np.rad2deg(P))

        else:
            if rotacion == "R": R = R + np.deg2rad(step)  #vertical grip
            #print("Best_Grasp_Node.-> Rotacion en Roll", np.rad2deg(R))
        
        obj_pose_frame_object.position.x = grip_point[0]
        obj_pose_frame_object.position.y = grip_point[1]
        obj_pose_frame_object.position.z = grip_point[2]
        
        if debug:
            print("Best_Grasp_Node.-> emitiendo pose........." + name_frame+str(j)+obj_state , grip_point)
            broadcaster_frame_object('object', name_frame+str(j)+obj_state , obj_pose_frame_object )
        
        grasp_candidates_quaternion.append(obj_pose_frame_object )     # guarda el candidato en frame bl
        
        
    #trajectory, pose, rpy_pose, graspable = evaluating_possibility_grip(grasp_candidates_quaternion , "horizontal")


    return grasp_candidates_quaternion  # REGRESA A LOS CANDIDATOS EN FRAME OBJECT




def grip_rules(obj_pose, type_obj, obj_state, size, grip_point):

    if(type_obj == "BOWL"):
        print("Best_Grasp_Node.->The object will be GRABBED AS BOWL.................")
        return cubic_and_bowl_obj(obj_pose, obj_state , grip_point, size, type_obj)
    
    if(type_obj == "CUBIC"):
        print("Best_Grasp_Node.->The object will be GRABBED AS CUBE.................")
        return cubic_and_bowl_obj(obj_pose, obj_state , grip_point, size, type_obj)

    if(type_obj == "PRISM"):
        print("Best_Grasp_Node.->The object will be GRABBED AS PRISM..................")
        return prism(obj_pose, obj_state)
    
    if(type_obj == "BOX"):
        print("Best_Grasp_Node.-> The object will be GRABBED as BOX....................")
        return box(obj_pose, size, obj_state )
    
    # Parche temporal
    if(type_obj == "BIG"):
        print("Best_Grasp_Node.-> The object  BIG will be GRABBED as BOX....................")
        return box(obj_pose, size, obj_state )
    
    else:
        print("Error identificando forma del objeto.................")
        poses_list = []
        return poses_list



def top_grip(grip_point): 
    """
    Disenia agarrea superiores, para cuando la primera componente del objeto es horizontal
    """
    global debug  

    obj_state = "horizontal"
    # Primera lista de candidatos******************************************************************************
    obj_pose_1 = Pose()
    obj_pose_1.position.x, obj_pose_1.position.y, obj_pose_1.position.z = 0, 0, 0
    obj_pose_1.orientation.x = 0.0
    obj_pose_1.orientation.y = 0.0
    obj_pose_1.orientation.z = 0.0
    obj_pose_1.orientation.w = 1.0

    pose_list1 = generates_candidates(grip_point , obj_pose_1, "P", obj_state ,  'c1', step = -12, num_candidates = 6, type_obj = None)
        

    # Segunda lista de candidatos******************************************************************************
    obj_pose_2 = Pose()
    obj_pose_2.position.x, obj_pose_2.position.y, obj_pose_2.position.z = 0, 0, 0
    obj_pose_2.orientation.x = 0.0
    obj_pose_2.orientation.y = 0.0
    obj_pose_2.orientation.z = 0.0
    obj_pose_2.orientation.w = 1.0
        
    pose_list2 = generates_candidates(grip_point , obj_pose_2, "P", obj_state , 'C2', step = 10, num_candidates = 5, type_obj = None)
        
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
    Y = Y + np.deg2rad(180) # Realiza un yaw de 180 grados 
        
        
    q_gripper = tft.quaternion_from_euler(R,P,Y,'sxyz')  # Pose en frame 'object' cuaterniones
    obj_pose_3.orientation.x = q_gripper[0]
    obj_pose_3.orientation.y = q_gripper[1]
    obj_pose_3.orientation.z = q_gripper[2]
    obj_pose_3.orientation.w = q_gripper[3]
    obj_pose_3.position.x = 0
    obj_pose_3.position.y = 0
    obj_pose_3.position.z = 0
        
    pose_list3 = generates_candidates(grip_point, obj_pose_3 , "P", obj_state , 'c3', step = -10, num_candidates = 4, type_obj = None)

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

    pose_list4 = generates_candidates(grip_point , obj_pose_4 , "P", obj_state , 'c4', step = 10, num_candidates = 4, type_obj = None)

    print("Num Candidates:____", len(pose_list2 + pose_list1 + pose_list3 + pose_list4))
        
    return pose_list2 + pose_list1 + pose_list3 + pose_list4




def cubic_and_bowl_obj(obj_pose, obj_state , grip_point, size, type_obj):
    """
    Construye los candidatos de agarre para un objeto pequenio
    """
    global debug

    obj_state = "horizontal"

    # Lista de candidatos******************************************************************************
    obj_pos_2 = Pose()
    if (type_obj == "CUBIC"):
        obj_pos_2.position.x, obj_pos_2.position.y, obj_pos_2.position.z = 0, 0, 0
        num_candidates = 8 # debe ser par
    else:
        obj_pos_2.position.x, obj_pos_2.position.y, obj_pos_2.position.z = 0 , size.z/2 , 0
        #print("GRIP POINT BOWL: ", obj_pos_2)
        marker_array_publish(grip_point, 'object', 59, 56)
        num_candidates = 10
        

    obj_pos_2.orientation.x = 0.0
    obj_pos_2.orientation.y = 0.0
    obj_pos_2.orientation.z = 0.0
    obj_pos_2.orientation.w = 1.0
    step = -15
    pose_list = generates_candidates([obj_pos_2.position.x , obj_pos_2.position.y ,obj_pos_2.position.z] , obj_pos_2, "P", obj_state ,  'c2', step, num_candidates, type_obj)
 
    return pose_list 
    



def box(obj_pose, size, obj_state):   
    """
    Construye los candidatos de agarre para un objeto tipo caja
    """
    global debug

    # HORIZONTAL **************************************************************************************************************************************
    if obj_state == 'horizontal':  
        print("Best_Grasp_Node.-> Horizontal box")
        # genera punto de agarre superior ************************

        # Primera lista de candidatos******************************************************************************
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
        marker_array_publish(grip_point_side , 'object', 59, 56)
            
        candidates_h_list_2 = generates_candidates(grip_point_side , obj_pose_2, "P", 'vertical' , 'C2', step = -12, num_candidates = 4, type_obj = None)

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
        grip_point_side = [size.x/6, 0, size.z/2]
        candidates_v_list = generates_candidates(grip_point_side , obj_pos_1, "R", obj_state ,'object', step = 10, num_candidates = 5, type_obj = None)  

        # Segunda lista de candidatos******************************************************************************
        obj_pos_22 = Pose()
        obj_pos_22.position.x, obj_pos_1.position.y, obj_pos_1.position.z = 0, 0, 0
        obj_pos_22.orientation.x = 0.0
        obj_pos_22.orientation.y = 0.0
        obj_pos_22.orientation.z = 0.0
        obj_pos_22.orientation.w = 1.0
        grip_point_side_2 = [size.x/6, 0, size.z/2]
        candidates_v_list_2 = generates_candidates(grip_point_side_2 , obj_pos_22, "R", obj_state ,'object', step = -10, num_candidates = 5, type_obj = None)  

        # tercera lista de candidatos******************************************************************************
        obj_pos_3 = Pose()
        obj_pos_3.position.x, obj_pos_3.position.y, obj_pos_3.position.z = 0, 0, 0
        obj_pos_3.orientation.x = 0.0
        obj_pos_3.orientation.y = 0.0
        obj_pos_3.orientation.z = 0.0
        obj_pos_3.orientation.w = 1.0

        R, P, Y = tft.euler_from_quaternion([obj_pos_3.orientation.x ,  # pose expresada en RPY para realizar rotaciones
                                                    obj_pos_3.orientation.y ,
                                                    obj_pos_3.orientation.z, 
                                                    obj_pos_3.orientation.w])
        R = R + np.deg2rad(180) 
            
            
        q_grip = tft.quaternion_from_euler(R,P,Y,'sxyz')  # Pose en frame 'object' cuaterniones
        obj_pos_3.orientation.x = q_grip[0]
        obj_pos_3.orientation.y = q_grip[1]
        obj_pos_3.orientation.z = q_grip[2]
        obj_pos_3.orientation.w = q_grip[3]
        obj_pos_3.position.x = 0
        obj_pos_3.position.y = 0
        obj_pos_3.position.z = 0
        # genera punto de agarre lateral
        grip_point_side = [-size.x/6, 0, 0]
        marker_array_publish(grip_point_side , 'object', 59, 56)
            
        candidates_V_list_3 = generates_candidates(grip_point_side , obj_pos_3, "R", 'vertical' , 'C2', step = -16, num_candidates = 4, type_obj = None)

        # Cuarta lista de candidatos******************************************************************************
        obj_pos_4 = Pose()
        obj_pos_4.position.x, obj_pos_4.position.y, obj_pos_4.position.z = 0, 0, 0
        obj_pos_4.orientation.x = 0.0
        obj_pos_4.orientation.y = 0.0
        obj_pos_4.orientation.z = 0.0
        obj_pos_4.orientation.w = 1.0

        R, P, Y = tft.euler_from_quaternion([obj_pos_4.orientation.x ,  # pose expresada en RPY para realizar rotaciones
                                                    obj_pos_4.orientation.y ,
                                                    obj_pos_4.orientation.z, 
                                                    obj_pos_4.orientation.w])
        R = R + np.deg2rad(180) 
            
            
        q_grip = tft.quaternion_from_euler(R,P,Y,'sxyz')  # Pose en frame 'object' cuaterniones
        obj_pos_4.orientation.x = q_grip[0]
        obj_pos_4.orientation.y = q_grip[1]
        obj_pos_4.orientation.z = q_grip[2]
        obj_pos_4.orientation.w = q_grip[3]
        obj_pos_4.position.x = 0
        obj_pos_4.position.y = 0
        obj_pos_4.position.z = 0
        # genera punto de agarre lateral
        grip_point_side_4 = [-size.x/6, 0, 0]
        marker_array_publish(grip_point_side_4 , 'object', 50, 53)
            
        candidates_V_list_4 = generates_candidates(grip_point_side_4, obj_pos_4, "R", 'vertical' , 'C2', step = 16, num_candidates = 4, type_obj = None)


        return candidates_v_list + candidates_v_list_2 + candidates_V_list_3+candidates_V_list_4 # En frame 'object'




def prism(obj_pose, obj_state):
    """
    Construye los candidatos de agarre para un objeto prismatico
    """
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
        print("Best_Grasp_Node.-> Horizontal grip prism............")  # Agarre horizontal
        grip_point = [0,0,0]#[obj_pose.position.x , obj_pose.position.y, obj_pose.position.z] #debe estar en el frame 'base_link'
        return top_grip(grip_point)

    # VERTICAL ***************************************************************************************************    
    else:
        print("Best_Grasp_Node.-> Vertical grip.................")
        for i in range( num_points):   # generación de puntos alrededor del objeto
            point = np.asarray([ 0, epsilon*np.sin(theta), epsilon*np.cos(theta)  ])
            #point = points_actual_to_points_target(point, 'object', 'base_link')
            points.append(point)

            marker_array_publish(point, 'object', count, id)
            count += 1
            id += 1
            theta = theta - step_size 
        #print("Best_Grasp_Node.-> number of points for candidates", len(points))
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
                #print("vertical_prism"+str(j))
                broadcaster_frame_object('object', 'vertical_prism'+str(j) , candidate_grasp ) 
            j += 1
        print("Best_Grasp_Node.-> number of candidates vertical grip prism", len(grasp_candidates_quaternion))
        return grasp_candidates_quaternion



def spoon_grip():
    gripper_length = 0.1
    grip_point = [0, gripper_length, 0]     # se define el punto de agarre
    if debug:
        marker_array_publish(grip_point, 'object', 55, 56)
    
    return top_grip(grip_point)



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



def pose_for_ik_service(pose_in_frame_object):  
    """
    Cambia a los candidatos de agarre del sistema 'object' al sistema del brazo izquierdo
    y regresa un arreglo [x,y,z,R,P,Y], si no lo consiguió retorna el valor -1
    """
    print("In pose-for_ik_service")
    new_pose = pose_actual_to_pose_target(pose_in_frame_object, 'object' , 'shoulders_left_link') 

    if new_pose == -1:
        return -1, -1

    x , y, z = new_pose.position.x , new_pose.position.y , new_pose.position.z
    roll,pitch,yaw = tft.euler_from_quaternion( [new_pose.orientation.x , new_pose.orientation.y , 
                                          new_pose.orientation.z , new_pose.orientation.w ])
    cartesian_pose_shoulder = np.asarray([x ,y ,z , roll, pitch , yaw])
    candidate = cartesian_pose_shoulder

    return cartesian_pose_shoulder, new_pose



    
def evaluating_possibility_grip(candidate_quaternion_list, obj_state, category):
    """
        Evalua la existencia de los candidatos de agarre en el espacio articular y regresa una trayectoria
        desde la posicion actual del gripper hasta el origen del primer frame candidato aprobado 
    """
    global ik_srv
    ik_msg = InverseKinematicsPose2TrajRequest()
    ik_msg_3 = InverseKinematicsPose2TrajRequest()


    if(category == "BOWL") or (category == "CUBIC"):
        first_trajectory  = []
        second_trajectory = []
        first_trajectory  = candidate_quaternion_list
        second_trajectory = candidate_quaternion_list
        first_trajectory.reverse()
        candidate_quaternion_list = first_trajectory
    
        
    print("Best_Grasp_Node.-> evaluating_possibility_grip()")
    i = 0
    for pose1 in candidate_quaternion_list:  # rellena el mensaje para el servicio IK
        if obj_state == "vertical":
            pose_xyzrpy, new_pos = pose_for_ik_service(pose1)
            if new_pos == -1:
                #print("NEW POSE = ", new_pos)
                continue
            descarte =  descarte_forced_poses(new_pos)
            if descarte: continue

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
                print("Best_Grasp_Node.-> Suitable pose for vertical object found.....................")
                return resp_ik_srv.articular_trajectory , candidate_quaternion_list[i] , pose_xyzrpy, True
            except:
                i = i + 1 
                print("Best_Grasp_Node.-> Discarded candidate")  
                continue

        #***********************************************************************************************************************

        else:
            if(category == "BOWL") or (category == "CUBIC"):
                print("HORIZONTAL GRIP")
                pose1, new_pos = pose_for_ik_service(pose1)

                if new_pos == -1: continue

                ik_msg.x         = pose1[0] 
                ik_msg.y         = pose1[1]
                ik_msg.z         = pose1[2]
                ik_msg.roll      = pose1[3]     
                ik_msg.pitch     = pose1[4]
                ik_msg.yaw       = pose1[5]
                ik_msg.duration  = 7
                ik_msg.time_step = 0.07
                try:    # intenta obtener la primera trayectoria en el espacio articular
                    resp_ik_srv = ik_srv(ik_msg)    # Envia al servicio de IK
                    print("Best_Grasp_Node.-> Suitable pose 1 for horizontal object found.....................")
                    print("Generating second trajectory to take the object")

                    # el ultimo punto de la 1a trayectoria es el primero de la segunda
                    guess = [resp_ik_srv.articular_trajectory.points[-1].positions[0],
                            resp_ik_srv.articular_trajectory.points[-1].positions[1],
                            resp_ik_srv.articular_trajectory.points[-1].positions[2],
                            resp_ik_srv.articular_trajectory.points[-1].positions[3],
                            resp_ik_srv.articular_trajectory.points[-1].positions[4],
                            resp_ik_srv.articular_trajectory.points[-1].positions[5],
                            resp_ik_srv.articular_trajectory.points[-1].positions[6]]

                # SEGUNDA TRAYECTORIA..........................................................................
                    for pose_obj in second_trajectory:
                        cartesian_pose_shoulder, new_pose_shoulder = pose_for_ik_service(pose_obj)

                        if new_pose_shoulder == -1: # Si no se pudo realizar la conversion de sistema objec a shoulder
                            continue

                        ik_msg_3.x             = cartesian_pose_shoulder[0] 
                        ik_msg_3.y             = cartesian_pose_shoulder[1]
                        ik_msg_3.z             = cartesian_pose_shoulder[2] - Z_OFFSET
                        ik_msg_3.roll          = cartesian_pose_shoulder[3]      
                        ik_msg_3.pitch         = cartesian_pose_shoulder[4]
                        ik_msg_3.yaw           = cartesian_pose_shoulder[5]
                        ik_msg_3.duration      = 6
                        ik_msg_3.time_step     = 0.07
                
                        ik_msg_3.initial_guess = guess

                        try:
                            resp_3_ik_srv = ik_srv(ik_msg_3)    # Envia al servicio de IK
                            print("Best_Grasp_Node.-> Suitable pose 2 for horizontal object found.....................")
                            resp_ik_srv.articular_trajectory.points = resp_ik_srv.articular_trajectory.points + resp_3_ik_srv.articular_trajectory.points
                            return resp_ik_srv.articular_trajectory , pose_obj , new_pos, True
                        except:
                            print("Best_Grasp_Node.-> Candidato de la segunda trayectoria no aprobado")
                            continue
                


                except:
                    print("Best_Grasp_Node.-> candidato no aprobado")
                    continue

            else:   # Prismatico horizontal
                print("HORIZONTAL PRISM")
                pose1, new_pos = pose_for_ik_service(pose1)

                if new_pos == -1: continue

                ik_msg.x         = pose1[0] 
                ik_msg.y         = pose1[1]
                ik_msg.z         = pose1[2]
                ik_msg.roll      = pose1[3]     
                ik_msg.pitch     = pose1[4]
                ik_msg.yaw       = pose1[5]
                ik_msg.duration  = 7
                ik_msg.time_step = 0.07
                try:    # intenta obtener la primera trayectoria en el espacio articular
                    resp_ik_srv = ik_srv(ik_msg)    # Envia al servicio de IK
                    print("Best_Grasp_Node.-> Suitable pose 1 for horizontal object found.....................")
                    
                    print("Best_Grasp_Node.-> genera una segunda trayectoria para objeto horizontal")
                    # el ultimo punto de la 1a trayectoria es el primero de la segunda
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
                    ik_msg.z = pose1[2] - 0.11
                    ik_msg.roll = pose1[3]
                    ik_msg.pitch = pose1[4]
                    ik_msg.yaw = pose1[5]
                    ik_msg.duration = 1
                    ik_msg.time_step = 0.1
                    ik_msg.initial_guess = guess

                    try:    
                        resp_2_ik_srv = ik_srv(ik_msg)    # Envia al servicio de IK
                        print("Best_Grasp_Node.-> Second trajectory found.....................")
                        resp_ik_srv.articular_trajectory.points = resp_ik_srv.articular_trajectory.points + resp_2_ik_srv.articular_trajectory.points
                        
                        approved_pose = cartesian2pose_msg(pose1[0] ,pose1[1] ,pose1[2] -0.1 ,pose1[3] ,pose1[4] ,pose1[5]) 

                        return resp_ik_srv.articular_trajectory , approved_pose , new_pos, True
                    except: print("Best_Grasp_Node.-> Second trajectory no found")
   
                except:
                    i = i + 1 
                    print("Best_Grasp_Node.-> Pose no apta........")  
                    continue
            
    return None, None, None, False



def cartesian2pose_msg(x,y,z,R,P,Y):
    quaternion = tft.quaternion_from_euler(R,P,Y,'sxyz')  # Pose en frame 'object' cuaterniones
    pose_obj = Pose()
    pose_obj.orientation.x = quaternion[0]
    pose_obj.orientation.y = quaternion[1]
    pose_obj.orientation.z = quaternion[2]
    pose_obj.orientation.x = quaternion[3]
    pose_obj.position.x = x
    pose_obj.position.y = y
    pose_obj.position.z = z
    return pose_obj


def callback(req):
    global listener #, ik_srv
    resp = BestGraspTrajResponse()              
    obj_state = req.recog_object.object_state    

    print("Best_Grasp_Node.-> CENTROID:_____ ")
    print(req.recog_object.pose.position)
    print("Best_Grasp_Node.-> BB CATEGORY:_____ ")
    print(req.recog_object.category)  
    print("Best_Grasp_Node.-> STATE:_____ ", req.recog_object.object_state)
    print("Best_Grasp_Node.-> SIZE:_____ ") 
    print(req.recog_object.size)

    grip_point = [req.recog_object.pose.position.x, req.recog_object.pose.position.y, req.recog_object.pose.position.z]

    # Retorna lista de candidatos en frame 'object'
    pose_list_quaternion = grip_rules(req.recog_object.pose, req.recog_object.category, obj_state, req.recog_object.size , grip_point )
    
    if len( pose_list_quaternion) <= 0:
        print("Best_Grasp_Node.-> object is no graspable")
        return resp
    
    trajectory, pose, rpy_pose, graspable = evaluating_possibility_grip(pose_list_quaternion , obj_state, req.recog_object.category)
   
    if graspable:
        print("Best_Grasp_Node.-> SUITABLE POSE FOR OBJECT MANIPULATION......")
        broadcaster_frame_object('object', 'suitable_pose' , pose)
        resp.articular_trajectory = trajectory  # Retorna trayectoria en el espacio articular

        resp.graspable = True
        broadcaster_frame_object('shoulders_left_link' , 'suitable_pose' , pose)
        return resp

    else: 
        print("Best_Grasp_Node.-> No possible poses found :'(...................")
        resp.graspable = False
        return resp



def main():
    global listener , ik_srv, marker_pub, marker_array_pub, debug
    debug = True
    print("Node to grab objects based on their orientation by Iby..............ʕ•ᴥ•ʔ")
    rospy.init_node("gripper_orientation_for_grasping")
    rospy.Service("/manipulation/get_best_grasp_traj", BestGraspTraj, callback)

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


