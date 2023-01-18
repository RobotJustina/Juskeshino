#!/usr/bin/env python
import math
import rospy
import tf
import tf.transformations as tft
import numpy
import urdf_parser_py.urdf
from geometry_msgs.msg import PointStamped, Pose
from manip_msgs.srv import *
from tf.transformations import euler_from_quaternion
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import matplotlib.pyplot as plt
from pylab import *


def main():
    print ("Nodo cliente de IK")
    # Inicialice un nodo ROS con el nombre service_client
    rospy.init_node('srv_test_client')
    # Espere a que se ejecute el servicio qu esta solicitando
    rospy.wait_for_service('/manipulation/LA_inverse_kinematics')
    rospy.wait_for_service('/manipulation/la_direct_kinematics')
    rospy.wait_for_service('/manipulation/la_inverse_kinematics')
    # Crea la conexion al servicio y un objeto 
    ik_srv_client  = rospy.ServiceProxy('/manipulation/LA_inverse_kinematics', InverseKinematics)
    ladk_srv_client  = rospy.ServiceProxy('/manipulation/la_direct_kinematics', ForwardKinematics) 
    msg_dk = ForwardKinematicsRequest()
    #estim = -0.5, 0.6, 0.3, 2.0, 0.3, 0.2, 0.3
    msg_dk.q1 = -0.5
    msg_dk.q2 = 0.6
    msg_dk.q3 = 0.3
    msg_dk.q4 = 2.0
    msg_dk.q5 = 0.3
    msg_dk.q6 = 0.2
    msg_dk.q7 = 0.5
    r_dk = ladk_srv_client(msg_dk)
    print("DK:***************",r_dk)


    laik_srv_client  = rospy.ServiceProxy('/manipulation/la_inverse_kinematics', InverseKinematicsForPose)    
    msg = InverseKinematicsRequest()
    laik_msg = InverseKinematicsForPoseRequest()
    laik_msg.x = 0.2
    laik_msg.y = 0.23
    laik_msg.z = -0.3
    laik_msg.roll = -2
    laik_msg.pitch = -1
    laik_msg.yaw = 2.2
    resultado1 = laik_srv_client(laik_msg)
    print("Serv ik_la:",resultado1)


    
    # PROBANDO IK BRAZO IZQUIERDO
    # Punto final   GUI: Th1
    # - Translation: [0.514, 0.135, -0.359]
    # - Rotation: in RPY (radian) [0.678, -1.325, -0.723]
    # - Position q: [0.43, 0.54, -0.4, 1.02, 0.39, 0.044, 0.1]
    msg.x, msg.y, msg.z = 0.422, 0.195, -0.316#0.514, 0.135, 0.1 
    #msg.roll , msg.pitch, msg.yaw = 0.461, -1.31, -0.714#1.378, -1.525, -1.723 
    result = ik_srv_client(msg) 
    
    #print("Resultado: ", result)
    # PUNTOS PROBADOS
    # 0.514, 0.135, -0.359 0.678, -1.325, -0.723
    # 0.514, 0.135, 0.1, 1.378, -1.525, -1.723
    # 0.514, 0.135, 0.2 1.378, -1.525, -1.723  <--Fuera de limites articulares (Z)
    # 0.614, 0.135, 0 1.378, -1.525, -1.723
    # 0.65, 0.135, 0 1.378, -1.525, -1.723
    # 0.714, 0.135, 0 1.378, -1.525, -1.723  <-- No lo resolvio
    # 0.1, 0.1, 0 1.378, -1.525, -1.723  <-- Fuera de rango
    # 0.0, -0.1, -0.3, 1.378, -1.525, -1.723
    # 0.0, -0.2, -0.3, 1.378, -1.525, -1.723
    # 0.0, -0.3, -0.3, 1.378, -1.525, -1.723
    # 0.0, -0.4, -0.3, 1.378, -1.525, -1.723   <-- Fuera de rango
    # 0.514, 0.135, 0.1, 1.378, -1.1(desde y hasta +), -1.723  <-- Fuera de rango 
    # 0.514, 0.135, 0.1, 1.378, -1.2 (hasta -1.5), -1.723



if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    
