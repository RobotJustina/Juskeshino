import math
import rospy
from vision_msgs.srv import *
from manip_msgs.srv import *

class JuskeshinoManipulation:
    def setNodeHandle():
    	# Se subcribe a los servicios necesarios para manipulacion de objetos
        rospy.wait_for_service("/manipulation/get_best_grasp_traj")
        JuskeshinoManipulation.cltBestGrip = rospy.ServiceProxy("/manipulation/get_best_grasp_traj", BestGraspTraj )
        rospy.wait_for_service("/manipulation/la_ik_trajectory")
        JuskeshinoManipulation.cltIkPose = rospy.ServiceProxy("/manipulation/la_ik_trajectory", InverseKinematicsPose2Traj )

        loop = rospy.Rate(10)
        counter = 3
        while not rospy.is_shutdown() and counter > 0:
            counter-=1
            loop.sleep()
        return True
    
    def planBestGraspingConfiguration(vision_obj):
        req = BestGraspTrajRequest()
        req.recog_object = vision_obj
        resp = JuskeshinoManipulation.cltBestGrip(req)      # Pasa la peticion al servicio de manipulacion y retorna la respuesta
        return resp
    
    
    def cartesian_to_articular_pose(cartesian_pose):
        req = InverseKinematicsPose2TrajRequest()
        req.x = cartesian_pose[0]
        req.y = cartesian_pose[1]
        req.z = cartesian_pose[2]
        req.roll = cartesian_pose[3]
        req.pitch = cartesian_pose[4]
        req.yaw = cartesian_pose[5]
        req.duration = 7
        req.time_step = 0.2
        resp = JuskeshinoManipulation.cltIkPose(req)      # Pasa la peticion al servicio de manipulacion y retorna la respuesta
        return resp.points[-1]
    

    

    
