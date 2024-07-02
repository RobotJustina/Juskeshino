#!/usr/bin/env python3
import math
import rospy
import time
import numpy as np
from std_msgs.msg import Float64
from vision_msgs.srv import *
from manip_msgs.srv import *
from juskeshino_tools.JuskeshinoHardware import JuskeshinoHardware

class JuskeshinoManipulation:
    def setNodeHandle():
    	# Se subcribe a los servicios necesarios para manipulacion de objetos
        rospy.wait_for_service("/manipulation/get_best_grasp_traj")
        JuskeshinoManipulation.cltGripLa = rospy.ServiceProxy("/manipulation/get_best_grasp_traj", BestGraspTraj )

        rospy.wait_for_service("/manipulation/grasp_object_ra")
        JuskeshinoManipulation.cltGripRa = rospy.ServiceProxy("/manipulation/grasp_object_ra", BestGraspTraj )
        
        rospy.wait_for_service("/manipulation/la_ik_trajectory")
        JuskeshinoManipulation.cltIkLaPose = rospy.ServiceProxy("/manipulation/la_ik_trajectory", InverseKinematicsPose2Traj )

        rospy.wait_for_service("/manipulation/ra_ik_trajectory")
        JuskeshinoManipulation.cltIkRaPose = rospy.ServiceProxy("/manipulation/ra_ik_trajectory", InverseKinematicsPose2Traj )
        
        rospy.wait_for_service("/manipulation/ra_forward_kinematics")
        JuskeshinoManipulation.cltFkRaPose = rospy.ServiceProxy("/manipulation/ra_forward_kinematics", ForwardKinematics )
        
        rospy.wait_for_service("/manipulation/la_forward_kinematics")
        JuskeshinoManipulation.cltFkLaPose = rospy.ServiceProxy("/manipulation/la_forward_kinematics", ForwardKinematics )

        loop = rospy.Rate(10)
        counter = 3
        while not rospy.is_shutdown() and counter > 0:
            counter-=1
            loop.sleep()
        return True
        
    
    def dynamic_grasp_left_arm(is_thin: bool = False):
        #Open entirely the gripper
        msg = Float64()
        msg.data = 1.0
        JuskeshinoHardware.pubLaGoalGrip.publish(msg)
        rospy.sleep(1)
        msg = Float64()
        for i in np.linspace(start=1.0, stop=-0.1, num=20):
            rospy.sleep(0.05)
            msg.data = i
            JuskeshinoHardware.pubLaGoalGrip.publish(msg)
            current = np.asarray(rospy.wait_for_message("/hardware/left_arm/current_gripper", Float64, timeout=1.0).data)
            if abs(current-i) > 0.2:
                msg.data = current - 0.05
                JuskeshinoHardware.pubLaGoalGrip.publish(msg)
                return True
        msg.data = 0.0
        JuskeshinoHardware.pubLaGoalGrip.publish(msg)
        return is_thin

    def GripLa(vision_obj):
        req = BestGraspTrajRequest()
        req.recog_object = vision_obj
        resp = JuskeshinoManipulation.cltGripLa(req)      # Pasa la peticion al servicio de manipulacion y retorna la respuesta
        if resp.graspable:
            return [resp, True]
        else:
            return [resp, False]
        

    def GripRa(vision_obj):
        req = BestGraspTrajRequest()
        req.recog_object = vision_obj   
        resp = JuskeshinoManipulation.cltGripRa(req)      # Pasa la peticion al servicio de manipulacion y retorna la respuesta
        if resp.graspable:
            return [resp, True]
        else:
            return [resp, False]
    
    def raFk(q_array):
        req = ForwardKinematicsRequest()    
        req.q = q_array
        resp = JuskeshinoManipulation.cltFkRaPose(req)
        return resp

    
    def raIk(cartesian_pose):
        req = InverseKinematicsPose2TrajRequest()
        req.x = cartesian_pose[0]
        req.y = cartesian_pose[1]
        req.z = cartesian_pose[2]
        req.roll = cartesian_pose[3]
        req.pitch = cartesian_pose[4]
        req.yaw = cartesian_pose[5]
        req.duration = 7
        req.time_step = 0.2
        resp = JuskeshinoManipulation.cltIkRaPose(req)      # Pasa la peticion al servicio de manipulacion y retorna la respuesta
        return resp#.points[-1]
    
    def laIk(cartesian_pose):
        req = InverseKinematicsPose2TrajRequest()
        req.x = cartesian_pose[0]
        req.y = cartesian_pose[1]
        req.z = cartesian_pose[2]
        req.roll = cartesian_pose[3]
        req.pitch = cartesian_pose[4]
        req.yaw = cartesian_pose[5]
        req.duration = 7
        req.time_step = 0.2
        resp = JuskeshinoManipulation.cltIkLaPose(req)      # Pasa la peticion al servicio de manipulacion y retorna la respuesta
        return resp#.points[-1]
    

    
