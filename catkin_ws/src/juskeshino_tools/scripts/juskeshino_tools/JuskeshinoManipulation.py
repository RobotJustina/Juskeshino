import math
import rospy
from vision_msgs.srv import *
from manip_msgs.srv import *

class JuskeshinoManipulation:
    def setNodeHandle():
    	# Se subcribe a los servicios necesarios para manipulacion de objetos
        rospy.wait_for_service("/vision/get_best_grasp_traj")
        JuskeshinoManipulation.cltBestGrip = rospy.ServiceProxy("/vision/get_best_grasp_traj", BestGraspTraj )
	
        loop = rospy.Rate(10)
        counter = 3
        while not rospy.is_shutdown() and counter > 0:
            counter-=1
            loop.sleep()
        return True
    
    def planBestGraspingConfiguration(vision_obj):
        req = BestGraspTrajRequest()
        req.recog_object = vision_obj
        resp = JuskeshinoManipulation.cltBestGrip(req)
        return resp

    
