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
Z_OFFSET = 0.13
    





def pose_actual_to_pose_target(pose, f_actual, f_target):
    global listener
    poseStamped_msg = PoseStamped()  
    poseStamped_msg.header.frame_id = "object"   # frame de origen
    poseStamped_msg.header.stamp = rospy.Time()  # la ultima transformacion
    poseStamped_msg.pose = pose

    try:
        listener.waitForTransform("object", "shoulders_right_link", rospy.Time(0), rospy.Duration(10.0))
        print("waitfor ..despues")
        new_poseStamped = listener.transformPose('shoulders_right_link', poseStamped_msg)
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
    new_pose = pose_actual_to_pose_target(pose_in_frame_object, 'object' , 'shoulders_right_link') 

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
 
    print("GripRa.-> evaluating_possibility_grip()")
    i = 0
    for pose1 in candidate_quaternion_list:  # rellena el mensaje para el servicio IK
        pose1, new_pos = pose_for_ik_service(pose1)

        if new_pos == -1: continue

        ik_msg.x         = pose1[0] 
        ik_msg.y         = pose1[1]
        ik_msg.z         = pose1[2]
        ik_msg.roll      = pose1[3]     
        ik_msg.pitch     = pose1[4]
        ik_msg.yaw       = pose1[5]
        ik_msg.duration  = 5
        ik_msg.time_step = 0.07
        try:    # intenta obtener la primera trayectoria en el espacio articular
            resp_ik_srv = ik_srv(ik_msg)    # Envia al servicio de IK
            print("Best_Grasp_Node.-> Suitable pose 1 for horizontal object found.....................")
            #resp_ik_srv.articular_trajectory.points = resp_ik_srv.articular_trajectory.points
            return resp_ik_srv.articular_trajectory , pose1 , new_pos, True

        except:
            print("Best_Grasp_Node.-> candidato no aprobado")
            continue

    return None, None, None, False



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
    pose_list_quaternion = cubic_and_bowl_obj(req.recog_object.pose , obj_state , grip_point, req.recog_object.size , req.recog_object.category)

    if len( pose_list_quaternion) <= 0:
        print("Best_Grasp_Node.-> object is no graspable")
        return resp
    
    trajectory, pose, rpy_pose, graspable = evaluating_possibility_grip(pose_list_quaternion , obj_state, req.recog_object.category)
   
    if graspable:
        print("Best_Grasp_Node.-> SUITABLE POSE FOR OBJECT MANIPULATION......")
        resp.articular_trajectory = trajectory  # Retorna trayectoria en el espacio articular

        resp.graspable = True
        #broadcaster_frame_object('shoulders_right_link' , 'suitable_pose' , pose)
        return resp

    else: 
        print("Best_Grasp_Node.-> No possible poses found :'(...................")
        resp.graspable = False
        return resp



def main():
    global listener , ik_srv, marker_pub, marker_array_pub, debug
    debug = True
    print("Node to grab objects based on their orientation by Iby..............ʕ•ᴥ•ʔ")
    rospy.init_node("grasp_object_ra")
    rospy.Service("/manipulation/grasp_object_ra", BestGraspTraj, callback)

    listener = tf.TransformListener()
    # se suscribe al servicio /manipulation/ik_trajectory
    rospy.wait_for_service( '/manipulation/ra_ik_trajectory' )
    ik_srv = rospy.ServiceProxy( '/manipulation/ra_ik_trajectory' , InverseKinematicsPose2Traj )
    marker_pub = rospy.Publisher("/vision/object_recognition/markers", Marker, queue_size = 10) 
    marker_array_pub = rospy.Publisher("/vision/obj_reco/marker_array", MarkerArray, queue_size = 10) 

    loop = rospy.Rate(30)
    while not rospy.is_shutdown():
        loop.sleep()

if __name__ == '__main__':
    main()


