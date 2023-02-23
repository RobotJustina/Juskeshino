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



def callback(msg):
    global recog_obj_srv, best_grip_srv

    RecognizeObject_msg = RecognizeObjectRequest()
    RecognizeObject_msg.point_cloud = msg

    # servicio reconocimiento de objetos ***************************************
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
        # tipo de objeto
        type_obj = result_recog_obj.recog_object.category

        R,P,Y = tft.euler_from_quaternion([X, Y, Z, W])

        Pose2Pose_msg = InverseKinematicsPose2PoseRequest()
        # Rellenar msg pose to pose
        Pose2Pose_msg.x = x
        Pose2Pose_msg.y = y
        Pose2Pose_msg.z = z
        Pose2Pose_msg.roll = R
        Pose2Pose_msg.pitch = P
        Pose2Pose_msg.yaw = Y
        Pose2Pose_msg.initial_guess = [float(type_obj)]

    # *****************************************************************************
        # Retorna x,y,z,r,p,y en el sistema base_link, se debe transformar a sistema shoulder_left_link
        result_best_grip = best_grip_srv(Pose2Pose_msg )
        """
        xd = result_best_grip.q[0]
        yd = result_best_grip.q[1]
        zd = result_best_grip.q[2]
        rd = result_best_grip.q[3]
        pd = result_best_grip.q[4]
        yd = result_best_grip.q[5]

        
        pose_stamp_left_sh = frame_actual_to_frame_target(x_bl , y_bl , z_bl , r_bl, p_bl , yaw_bl, 
                                     'base_link' , 'shoulders_left_link' )
        R_l_sh, P_l_sh, Y_l_sh =tft.euler_from_quaternion([ pose_stamp_left_sh.orientation.x,
                                                            pose_stamp_left_sh.orientation.y, 
                                                            pose_stamp_left_sh.orientation.z, 
                                                            pose_stamp_left_sh.orientation.w])
        


        print("Pose de gripper para toma de objetos en frame left shoulder"),
        print("position")
        print(xd, yd, zd)
        print( "orientation RPY rad")
        print(rd, pd, yd)
        print("")
        print("")
        print( "orientation RPY grados")
        print(np.rad2deg(rd) , np.rad2deg(pd) ,np.rad2deg(yd))
        """
        # con el resultado ya se puede orientar gripper para tomar un objeto

        





def main():
    global pose_obj_frame_base, recog_obj_srv, best_grip_srv ,listener
    print("test node... ʕ•ᴥ•ʔ")
    rospy.init_node("nodo_test")

    rospy.Subscriber("/hardware/realsense/points", PointCloud2, callback) 

    rospy.wait_for_service("/vision/obj_reco/recognize_object")
    recog_obj_srv = rospy.ServiceProxy("/vision/obj_reco/recognize_object", RecognizeObject)

    rospy.wait_for_service("/vision/gripper_orientation_grasping")
    best_grip_srv = rospy.ServiceProxy("/vision/gripper_orientation_grasping", InverseKinematicsPose2Pose)
    
    listener = tf.TransformListener()

    loop = rospy.Rate(30)
    while not rospy.is_shutdown():
        
        loop.sleep()

if __name__ == '__main__':
    main()


