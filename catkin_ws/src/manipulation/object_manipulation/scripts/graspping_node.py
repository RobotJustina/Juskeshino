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



def main():
    global listener , ik_srv, marker_pub, marker_array_pub, debug
    debug = True
    print("Node to grab objects based on their orientation by Iby..............ʕ•ᴥ•ʔ")
    rospy.init_node("gripper_orientation_for_grasping")
    rospy.Service("/manipulation/get_best_grasp_traj", BestGraspTraj, callback)

    listener = tf.TransformListener()
    # se suscribe al servicio /manipulation/ik_trajectory
    rospy.wait_for_service( '/manipulation/la_ik_trajectory' )
    ik_srv = rospy.ServiceProxy( '/manipulation/la_ik_trajectory' , InverseKinematicsPose2Traj )
    marker_pub = rospy.Publisher("/vision/object_recognition/markers", Marker, queue_size = 10) 
    marker_array_pub = rospy.Publisher("/vision/obj_reco/marker_array", MarkerArray, queue_size = 10) 

    loop = rospy.Rate(30)
    while not rospy.is_shutdown():
        loop.sleep()

if __name__ == '__main__':
    main()