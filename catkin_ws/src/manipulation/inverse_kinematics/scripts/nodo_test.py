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
    Nodo para probar la cinematica inversa
"""

        
def main():
    global  ik_pose_srv, ik_traj_srv
    
    print("test node... ʕ•ᴥ•ʔ")
    rospy.init_node("nodo_test")

    # se suscribe al servicio de cinematica directa
    rospy.wait_for_service( '/manipulation/takeshi_forward_kinematics' )
    fk_srv = rospy.ServiceProxy( '/manipulation/takeshi_forward_kinematics' , ForwardKinematics )

    # se suscribe al servicio /manipulation/takeshi_ik_pose
    rospy.wait_for_service( '/manipulation/takeshi_ik_pose' )
    ik_pose_srv = rospy.ServiceProxy( '/manipulation/takeshi_ik_pose' , InverseKinematicsPose2Pose )

    # se suscribe al servicio /manipulation/takeshi_ik_trajectory
    rospy.wait_for_service( '/manipulation/takeshi_ik_trajectory' )
    ik_traj_srv = rospy.ServiceProxy( '/manipulation/takeshi_ik_trajectory' , InverseKinematicsPose2Traj )



    loop = rospy.Rate(30)

    while not rospy.is_shutdown():

        # Mensaje para servicio de cinematica directa********************
        fk_msg = ForwardKinematicsRequest()
        fk_msg.q = [0,0,-1.56,-1.57,0]  # articular
        cartesian_pose = fk_srv(fk_msg) # cartesiano

        print("cinematica directa")
        print(cartesian_pose)
        """
        # Mensaje para servicio ik pose**********************************
        ik_pp_msg = InverseKinematicsPose2PoseRequest()
        # Rellenar msg pose to pose
        ik_pp_msg.x =  cartesian_pose.x
        ik_pp_msg.y = cartesian_pose.y
        ik_pp_msg.z = cartesian_pose.z
        ik_pp_msg.roll = cartesian_pose.roll
        ik_pp_msg.pitch = cartesian_pose.pitch
        ik_pp_msg.yaw = cartesian_pose.yaw
        ik_pp_msg.initial_guess = fk_msg.q
        # Retorna arreglo de q's
        q_array = ik_pose_srv(ik_pp_msg)
        # retorno de msg ***********************************************
        print("Cinematica inversa pose2pose .......................")
        print(q_array)
        """
        # Mensaje para servicio ik traj**********************************
        ik_pt_msg = InverseKinematicsPose2TrajRequest()
        # Rellenar msg pose to traj
        ik_pt_msg.x =  cartesian_pose.x 
        ik_pt_msg.y = cartesian_pose.y 
        ik_pt_msg.z = cartesian_pose.z 
        ik_pt_msg.roll = cartesian_pose.roll + np.deg2rad(10)
        ik_pt_msg.pitch = cartesian_pose.pitch
        ik_pt_msg.yaw = cartesian_pose.yaw + np.deg2rad(10)
        ik_pt_msg.initial_guess = fk_msg.q
        # Retorna arreglo de q's

        q_traj = ik_traj_srv(ik_pt_msg)

        # retorno de msg ***********************************************
        #print(q_traj)
        #print(type( np.asarray(q_traj.articular_trajectory.points[-1].positions)))
        #print(q_traj.articular_trajectory.points[-1].positions)
        
        fk_msg2 = ForwardKinematicsRequest()
        q1 = q_traj.articular_trajectory.points[-1].positions[0]
        q2 = q_traj.articular_trajectory.points[-1].positions[1]
        q3 = q_traj.articular_trajectory.points[-1].positions[2]
        q4 = q_traj.articular_trajectory.points[-1].positions[3]
        q5 = q_traj.articular_trajectory.points[-1].positions[4]
    

        fk_msg2.q = [q1,q2,q3,q4,q5] 
        cartesian_pose2 = fk_srv(fk_msg2) # cartesiano
        print("cinematica directa despues****************")
        print(cartesian_pose2)
        
        
            
        loop.sleep()

if __name__ == '__main__':
    main()


