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

from tf.transformations import euler_from_quaternion
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint



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
    #ts = joint_trajectory.points[1].time_from_start
    #print("ts=",joint_trajectory.points[1].time_from_start.nsecs)

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




def callback(msg):
    global recog_obj_srv, best_grip_srv

    RecognizeObject_msg = RecognizeObjectRequest()
    RecognizeObject_msg.point_cloud = msg

    # servicio reconocimiento de objetos ***************************************
    print("Reconocimiento del objeto.....")
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

    # servicio de agarre de objetos *********************************************
        print("Calculando la mejor orientacion del gripper.....")
        Pose2traj_msg = InverseKinematicsPose2TrajRequest()
        # Rellenar msg pose to pose
        Pose2traj_msg.x = x
        Pose2traj_msg.y = y
        Pose2traj_msg.z = z
        Pose2traj_msg.roll = R
        Pose2traj_msg.pitch = P
        Pose2traj_msg.yaw = Y
        Pose2traj_msg.initial_guess = [float(type_obj)]
        # Retorna trayectoria en el espacio articular
        articular_trajectory = best_grip_srv( Pose2traj_msg )

    # seguimiento de trayectoria ***********************************************
        print("Realizando el seguimiento de trayectoria.....")
        la_trajectory_tracking( articular_trajectory )

           



def main():
    global pose_obj_frame_base, recog_obj_srv, best_grip_srv ,listener, traj_tracking_srv
    global pubLaGoalPose, pubRaGoalPose
    print("test node... ʕ•ᴥ•ʔ")
    rospy.init_node("nodo_test")

    rospy.Subscriber("/hardware/realsense/points", PointCloud2, callback) 

    rospy.wait_for_service("/vision/obj_reco/recognize_object")
    recog_obj_srv = rospy.ServiceProxy("/vision/obj_reco/recognize_object", RecognizeObject)

    rospy.wait_for_service("/vision/gripper_orientation_grasping")
    best_grip_srv = rospy.ServiceProxy("/vision/gripper_orientation_grasping", InverseKinematicsPose2Traj )

    rospy.wait_for_service("/manipulation/trajectory_tracking" )
    traj_tracking_srv = rospy.ServiceProxy("/manipulation/trajectory_tracking" , InverseKinematicsPose2Traj )

    pubLaGoalPose = rospy.Publisher("/hardware/left_arm/goal_pose" , Float64MultiArray, queue_size=10);
    pubRaGoalPose = rospy.Publisher("/hardware/right_arm/goal_pose", Float64MultiArray, queue_size=10);

    listener = tf.TransformListener()

    loop = rospy.Rate(30)
    while not rospy.is_shutdown():
        
        loop.sleep()

if __name__ == '__main__':
    main()


