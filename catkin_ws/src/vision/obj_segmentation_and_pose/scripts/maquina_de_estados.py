#!/usr/bin/env python3

import rospy
import numpy as np
import tf.transformations as tft
import tf
import tf2_ros
from sensor_msgs.msg import PointCloud2, Image
from geometry_msgs.msg import PointStamped, PoseStamped, Point, Pose
from vision_msgs.srv import *
from manip_msgs.srv import *
from std_msgs.msg import String
import time
import urdf_parser_py.urdf
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String, Float64MultiArray, Float32, Float64,Bool, Float32MultiArray
from tf.transformations import euler_from_quaternion
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

pubLaGoalPose = None


"""
    Nodo de prueba que envia una nube de puntos y recibe msg vision obj
    Tambien se debe crear nodo (servicio) que recibe el vision obj y regresa un pose 
    para la cinematica inversa.
"""

def frame_actual_to_frame_target(x, y, z, R, P, Y, f_actual, f_target):
    global pose_in, listener
    q1, q2, q3, q4 = tft.quaternion_from_euler(R, P, Y) # conversion a quaternion

    # Empaqueta msg, convierte orientacion de frame 'realsense_link' a frame 'base_link'
    poseStamped_msg = PoseStamped()  
    poseStamped_msg.header.frame_id = f_actual   # frame de origen
    poseStamped_msg.header.stamp = rospy.Time(0)  # la ultima transformacion
    pose_in = Pose()
    pose_in.position.x = x
    pose_in.position.y = y
    pose_in.position.z = z
    pose_in.orientation.x = q1
    pose_in.orientation.y = q2
    pose_in.orientation.z = q3
    pose_in.orientation.w = q4

    poseStamped_msg.pose = pose_in
    pose_frame_base_link = listener.transformPose(f_target, poseStamped_msg)
    new_pose = pose_frame_base_link.pose

    return new_pose
    




def la_trajectory_tracking(joint_trajectory):

    global pubLaGoalPose
    print("Starting left arm trajectory tracking with " + str(len(joint_trajectory.points) ) + "points")
    i = 0

    ts = 0.05
    #while not rospy.is_shutdown():
    for point in joint_trajectory.points:
        q1, q2, q3, q4, q5, q6, q7 = point.positions
        msg = Float64MultiArray()
        msg.data.append(q1)
        msg.data.append(q2)
        msg.data.append(q3)
        msg.data.append(q4)
        msg.data.append(q5)
        msg.data.append(q6)
        msg.data.append(q7)
        pubLaGoalPose.publish(msg)
        time.sleep(ts)  # Wait ts seconds while moving the arm to the desired position
        i += 1



def point_actual_to_point_target(position_xyz, f_actual, f_target):
    global listener
    p_k = PointStamped()# Creamos el mensaje para el metodo transformPoint
    p_k.header.frame_id = f_actual   # Sistema coordenado de los puntos 
    p_k.header.stamp = rospy.Time(0)  # Queremos la ultima transformacion
    p_k.point.x, p_k.point.y, p_k.point.z = position_xyz[0], position_xyz[1], position_xyz[2]
    
    point_in_frame_target = listener.transformPoint(f_target, p_k)

    return point_in_frame_target



def position():
    global recog_obj_srv, pubLaGoalPose

    RecognizeObject_msg = RecognizeObjectRequest()
    RecognizeObject_msg.point_cloud = rospy.wait_for_message("/camera/depth_registered/points" , PointCloud2)

    result_recog_obj = recog_obj_srv(RecognizeObject_msg )
    
    # Desempaqueta datos
    if result_recog_obj.recog_object.graspable:
        # Orientacion y posicion del objeto detectado:
        x = result_recog_obj.recog_object.pose.position.x
        y = result_recog_obj.recog_object.pose.position.y
        z = result_recog_obj.recog_object.pose.position.z
        X = result_recog_obj.recog_object.pose.orientation.x
        Y = result_recog_obj.recog_object.pose.orientation.y
        Z = result_recog_obj.recog_object.pose.orientation.z
        W = result_recog_obj.recog_object.pose.orientation.w
 
        print("Position")
        print(x,y,z)

        return [x,y,z]
    return [0,0,0]



def move_left_gripper(q):
    global pubLaGoalGrip
    pubLaGoalGrip.publish(q)
    time.sleep(1.0)
           




def main():
    global pose_obj_frame_base, recog_obj_srv, best_grip_srv ,listener, traj_tracking_srv, ik_srv
    global pubLaGoalPose, pubRaGoalPose, pubLaGoalGrip
    print("test node... ʕ•ᴥ•ʔ")
    rospy.init_node("nodo_test")
    #rospy.Subscriber("/camera/depth_registered/points", PointCloud2, callback) 
    rospy.wait_for_service("/vision/obj_reco/recognize_object")
    recog_obj_srv = rospy.ServiceProxy("/vision/obj_reco/recognize_object", RecognizeObject)
    rospy.wait_for_service( '/manipulation/la_ik_trajectory' )
    ik_srv = rospy.ServiceProxy( '/manipulation/la_ik_trajectory' , InverseKinematicsPose2Traj )
    pubLaGoalPose = rospy.Publisher("/hardware/left_arm/goal_pose" , Float64MultiArray, queue_size=10)
    pubLaGoalGrip = rospy.Publisher("/hardware/left_arm/goal_gripper" , Float64, queue_size=10)
    listener = tf.TransformListener()


    loop = rospy.Rate(30)
    while not rospy.is_shutdown():
        
    # E:posicion de objetos respecto a 'base_link'
        position_xyz = position()
        while (position_xyz[0] == 0) or (position_xyz[1] == 0) or (position_xyz[2] < 0 ): 
            print("Recalculando la posicion....................")
            position_xyz = position()

        print("pose del objeto", )
    
    """
    
    # E:conversion de puntos a frame 'shoulder_left'
        f_target = "shoulders_left_link"
        f_actual = 'base_link'
        p_shoulder_left = point_actual_to_point_target(position_xyz, f_actual, f_target)
        
        position_xyz[0] = p_shoulder_left.point.x
        position_xyz[1] = p_shoulder_left.point.y
        position_xyz[2] = p_shoulder_left.point.z

        print("Se realizo la conversion de centroide a frame shoulder left", position_xyz)

        loop.sleep()
    

    # E:calculo de la IK
        print("Calculando la ik para gripper.....")
        Pose2traj_msg = InverseKinematicsPose2TrajRequest()
        # Rellenar msg pose to pose
        Pose2traj_msg.x = position_xyz[0]
        Pose2traj_msg.y = position_xyz[1]
        Pose2traj_msg.z = position_xyz[2]
        Pose2traj_msg.roll = 0 
        Pose2traj_msg.pitch = np.deg2rad(0)
        Pose2traj_msg.yaw = 0 
        articular_trajectory = ik_srv( Pose2traj_msg )

        print("TRAYECTORIA ", articular_trajectory)

    
        # E:prepare 
        print("mov arm")
        msg = Float64MultiArray()
        msg.data.append(-0.7)
        msg.data.append(0)
        msg.data.append(0)
        msg.data.append(1)
        msg.data.append(0)
        msg.data.append(1)
        msg.data.append(0)
        pubLaGoalPose.publish(msg)
    

    # E:abre gripper
        move_left_gripper(0.5)

    # E:seguimiento de la trayectoria
        print("Realizando el seguimiento de trayectoria.....")
        #la_trajectory_tracking( articular_trajectory.articular_trajectory )
    """ 
        

if __name__ == '__main__':
    main()


