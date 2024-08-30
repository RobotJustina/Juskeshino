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


MAXIMUM_GRIP_LENGTH = 0.16
MINIMUM_HEIGHT_PRISM = 0.17
MAXIMUM_CUBE_SIZE = 0.16
Z_OFFSET_CUB = 0.06# real 0.12
Z_OFFSET_CUB_2 = 0.01
Z_OFFSET_PRISM = 0.07#0.21
Z_OFFSET_PRISM_2 = 0.07
Z_OFFSET_BOWL  = 0.304#0.22#0.19
Z_OFFSET_BOWL_2  = 0.07#0.07


def generate_pose(point_xyz, pose_quaternion):
    msg = Pose()
    msg.orientation.x = pose_quaternion[0]
    msg.orientation.y = pose_quaternion[1]
    msg.orientation.z = pose_quaternion[2]
    msg.orientation.w = pose_quaternion[3]
    msg.position.x = point_xyz[0]
    msg.position.y = point_xyz[1]
    msg.position.z = point_xyz[2]
    return msg


def points_actual_to_points_target(point_in, f_actual, f_target):
    global listener
    point_msg                 = PointStamped()  
    point_msg.header.frame_id = f_actual   # frame de origen
    point_msg.header.stamp    = rospy.Time() # la ultima transformacion
    point_msg.point.x         = point_in[0]
    point_msg.point.y         = point_in[1]
    point_msg.point.z         = point_in[2]

    listener.waitForTransform(f_actual, f_target, rospy.Time(), rospy.Duration())
    point_target_frame        = listener.transformPoint(f_target, point_msg)
    new_point                 = point_target_frame.point
    return [ new_point.x , new_point.y , new_point.z ]



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
    return cartesian_pose_shoulder, new_pose



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




































































def generates_candidates(obj_pose, name_frame, step, offset, rotation_axis):    
    global debug, num_candidates
    j = 0
    if debug: marker_array_publish([obj_pose.position.x, obj_pose.position.y, obj_pose.position.z], 'object', 59, 56)
    grasp_candidates_quaternion = []
    rotation = np.asanyarray([ offset[0],offset[1], offset[2]])
    for j in range(num_candidates):      # genera candidatos
        q_gripper = tft.quaternion_from_euler(np.deg2rad(rotation[0]), np.deg2rad(rotation[1]), np.deg2rad(rotation[2]),'sxyz')  # Orientación de la tf de entrada
        candidate = generate_pose([obj_pose.position.x, obj_pose.position.y, obj_pose.position.z] , q_gripper)
        rotation += step*rotation_axis
        print("muñti", rotation)
        print("ROTATION", np.rad2deg(rotation_axis[0]), np.rad2deg(rotation_axis[1]), np.rad2deg(rotation_axis[2]))
        if debug:
            print("Best_Grasp_Node.-> emitiendo pose........." + name_frame+str(j), candidate.position)
            broadcaster_frame_object('object', name_frame+str(j), candidate )
        grasp_candidates_quaternion.append(candidate )     # guarda el candidato en frame bl
        
    return grasp_candidates_quaternion  # REGRESA A LOS CANDIDATOS EN FRAME OBJECT




def grasp_bowl():
    global size, num_candidates
    # Primera trayectoria**********************************************
    grip_point_bl = points_actual_to_points_target([-0.02 , size.z/2 , 0] , 'object', 'base_link')
    grip_point_bl[2] = grip_point_bl[2] + Z_OFFSET_BOWL
    first_point = points_actual_to_points_target(grip_point_bl, 'base_link', 'object')
    first_tf = Pose()
    first_pose = generates_candidates(first_tf, first_point)
    candidate_list = generates_candidates(first_pose,  "BOWL_1", 10, -90, np.asarray([0,1,0]))
    first_trajectory, c_ft, graspable =  evaluating_possibility_grip(candidate_list,)
    # Segunda trayectoria
    grip_pose_2 = generate_pose([0,0,0], [0,0,0,1])
    candidate_list = generates_candidates(grip_pose_2 , "BOWL_2", np.deg2rad(-10), offset = [0,0,0])
    second_trajectory, c_ft_2, graspable_2 =  evaluating_possibility_grip(candidate_list, )
    first_trajectory.points = first_trajectory.points + second_trajectory.points
    return first_trajectory , c_ft, graspable




def grasp_prism_horizontal():
    global size, num_candidates
    # Primera trayectoria*********************************************************************************************
    first_point_bl = points_actual_to_points_target([0,0,0], 'object', 'base_link')
    first_point_bl[2] = first_point_bl[2] + Z_OFFSET_PRISM
    first_point = points_actual_to_points_target(first_point_bl , 'base_link', 'object')
    first_pose = generate_pose[first_point, [0,0,0,1]]
    candidate_list = generates_candidates(first_pose,  "P_H_1", np.deg2rad(10), [0, np.deg2rad(-90) , 0], np.asarray([0,1,0]))
    first_trajectory, c_ft, graspable =  evaluating_possibility_grip(candidate_list , )

    # Segunda trayectoria*********************************************************************************************
    #    el ultimo punto de la 1a trayectoria es el primero de la segunda
    guess = [first_trajectory.points[-1].positions[0],
                first_trajectory.points[-1].positions[1],
                first_trajectory.points[-1].positions[2],
                first_trajectory.points[-1].positions[3],
                first_trajectory.points[-1].positions[4],
                first_trajectory.points[-1].positions[5],
                first_trajectory.points[-1].positions[6]]

    second_pose = generate_pose([0,0,0], [0,0,0,1])
    candidate_list = generates_candidates( second_pose,  "P_H_2", np.deg2rad(-10), [0,0,0], np.asarray([0,1,0]))
    second_trajectory, c_ft_2, graspable_2 =  evaluating_possibility_grip(candidate_list, guess)
    first_trajectory.points = first_trajectory.points + second_trajectory.points
    return first_trajectory , c_ft, graspable



def grasp_prism_vertical():
    global  num_candidates, size
    candidate_list = generates_candidates( generate_pose([0,0,0] ,[0,0,0,1]) , "V_P", -5, [np.deg2rad(0), np.deg2rad(-5), 0],  np.asarray([0,1,0]))
    return evaluating_possibility_grip(candidate_list ,)
     


def grasp_cubic():
    global num_candidates, size
    grip_point_bl = points_actual_to_points_target([0,0,0], 'object', 'base_link')
    grip_point_bl[2] = grip_point_bl[2] + Z_OFFSET_CUB
    grip_point = points_actual_to_points_target(grip_point_bl, 'base_link', 'object')
    candidate_list = generates_candidates(generate_pose(grip_point, [0,0,0,1]) , "CUBIC", -5, [0, 0, 0], np.asarray([0,1,0]))
    return evaluating_possibility_grip(candidate_list, )



def grasp_box():
    # Primera trayectoria*********************************************************************************************
    global size
    first_point_bl = points_actual_to_points_target([0 , size.z/2 , 0], 'object', 'base_link')
    first_point_bl[0] = first_point_bl[0] - 0.05#3Z_OFFSET_PRISM
    first_point = points_actual_to_points_target(first_point_bl , 'base_link', 'object')
    candidate_list = generates_candidates(generate_pose(first_point, [0,0,0,1]) , "BOX_1", -5,[0, 0, 0] ,np.asarray([0,1,0]))
    first_trajectory, c_ft, graspable =  evaluating_possibility_grip(candidate_list, )
    # Segunda trayectoria*********************************************************************************************
    """
    #    el ultimo punto de la 1a trayectoria es el primero de la segunda
    guess = [first_trajectory.points[-1].positions[0],
                first_trajectory.points[-1].positions[1],
                first_trajectory.points[-1].positions[2],
                first_trajectory.points[-1].positions[3],
                first_trajectory.points[-1].positions[4],
                first_trajectory.points[-1].positions[5],
                first_trajectory.points[-1].positions[6]]
    second_pose = generate_pose([0,0,0], [0,0,0,1])
    candidate_list = generates_candidates( second_pose,  "BOX_2", -10, num_candidates, [0,0,0])
    second_trajectory, c_ft_2, graspable_2 =  evaluating_possibility_grip(candidate_list , type_obj, guess)
    first_trajectory.points = first_trajectory.points + second_trajectory.points
    """
    return first_trajectory , c_ft, graspable



def evaluating_possibility_grip(candidate_q_list, guess = None ):
    global ik_srv
    ik_msg = InverseKinematicsPose2TrajRequest()
    print("Best_Grasp_Node.-> evaluating_possibility_grip()")
    for candidate in candidate_q_list:  # rellena el mensaje para el servicio IK
        candidate_ik_msg, candidate_tf = pose_for_ik_service(candidate)
        ik_msg.x         = candidate_ik_msg[0] 
        ik_msg.y         = candidate_ik_msg[1]
        ik_msg.z         = candidate_ik_msg[2]
        ik_msg.roll      = candidate_ik_msg[3]     
        ik_msg.pitch     = candidate_ik_msg[4]
        ik_msg.yaw       = candidate_ik_msg[5]
        ik_msg.duration  = 0
        ik_msg.time_step = 0.05
        if guess != None:
            ik_msg.initial_guess = guess
        try:
            resp_ik_srv = ik_srv(ik_msg)    # Envia al servicio de IK
            print("Best_Grasp_Node.-> Suitable pose for object .....................")
            return resp_ik_srv.articular_trajectory , candidate_tf , True
        except:
            print("Best_Grasp_Node.-> Candidato no aprobado")
            continue
    return None, None, False



def callback(req):
    global listener, category, size
    resp = BestGraspTrajResponse() 
    category = req.recog_object.category
    size = req.recog_object.size
    graspping_function = {"BOWL": grasp_bowl, "CUBIC": grasp_cubic, "PRISM_HORIZONTAL": grasp_prism_horizontal, "PRISM_VERTICAL": grasp_prism_vertical, "BOX": grasp_box}
    trajectory_pose_graspable, pose, graspable = graspping_function[ req.recog_object.category]()
    if graspable:
        print("Best_Grasp_Node.-> SUITABLE POSE FOR OBJECT MANIPULATION......")
        broadcaster_frame_object('shoulders_left_link', 'suitable_pose' , pose)
        resp.articular_trajectory = trajectory_pose_graspable  # Retorna trayectoria en el espacio articular
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = 'shoulders_left_link'
        pose_stamped.pose = pose
        resp.pose_stamped = pose_stamped
        resp.graspable = True
        return resp
    else: 
        print("Best_Grasp_Node.-> No possible poses found :'(...................")
        resp.graspable = False
        return resp



def main():
    global listener , ik_srv, marker_pub, marker_array_pub, debug, num_candidates
    debug = True
    print("Node to grab objects based on their orientation by ITZEL..............ʕ•ᴥ•ʔ")
    rospy.init_node("grasp_object")
    rospy.Service("/manipulation/get_best_grasp_traj", BestGraspTraj, callback)
    rospy.wait_for_service( '/manipulation/la_ik_trajectory' )
    ik_srv           = rospy.ServiceProxy( '/manipulation/la_ik_trajectory' , InverseKinematicsPose2Traj )
    marker_pub       = rospy.Publisher("/vision/object_recognition/markers",  Marker, queue_size = 10) 
    marker_array_pub = rospy.Publisher("/vision/obj_reco/marker_array",       MarkerArray, queue_size = 10) 

    listener = tf.TransformListener()
    num_candidates = rospy.get_param('~num_candidates', 3)

    loop = rospy.Rate(30)
    while not rospy.is_shutdown():
        loop.sleep()

if __name__ == '__main__':
    main()