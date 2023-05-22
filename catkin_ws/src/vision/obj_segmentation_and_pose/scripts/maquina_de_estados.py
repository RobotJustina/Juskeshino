#!/usr/bin/env python3

import rospy
import numpy as np
import tf.transformations as tft
import tf
from sensor_msgs.msg import PointCloud2, Image
from geometry_msgs.msg import PointStamped, PoseStamped, Point, Pose
from vision_msgs.srv import *
from manip_msgs.srv import *
import time
from std_msgs.msg import String, Float64MultiArray, Float32, Float64,Bool, Float32MultiArray
from tf.transformations import euler_from_quaternion
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


SM_INIT = 0
SM_WAITING_NEW_COMMAND = 10
SM_GET_OBJ_POSE = 50
SM_MOVE_HEAD = 60
SM_WAIT_FOR_HEAD = 70
SM_MOVE_LEFT_ARM = 100 


def move_left_arm(goal_pose, pub_traj, clt_ik):
    print("Moving left arm to " + str(goal_pose))
    srv = InverseKinematicsPose2TrajRequest()
    srv.x = goal_pose[0]
    srv.y = goal_pose[1]
    srv.z = goal_pose[2]
    srv.roll = goal_pose[3]
    srv.pitch = goal_pose[4]
    srv.yaw = goal_pose[5]

    try:
        resp = clt_ik(srv)
    except:
        return None
    
    pub_traj.publish(resp.articular_trajectory)
    return


def get_obj_pose(clt_recog_obj):
    recognize_object_req = RecognizeObjectRequest()

    try:
        recognize_object_req.point_cloud = rospy.wait_for_message("/camera/depth_registered/points" , PointCloud2, timeout=2)
    except:
        return None
    resp_recog_obj = clt_recog_obj(recognize_object_req)
    print(resp_recog_obj.recog_object.pose)
    p = resp_recog_obj.recog_object.pose
    return [p.position.x, p.position.y, p.position.z]



def move_head(pub_hd_goal_pose, pan, tilt):
    msg = Float64MultiArray()
    msg.data.append(pan)
    msg.data.append(tilt)
    pub_hd_goal_pose.publish(msg)
    rospy.sleep(2.0)


def transform_to_la(x,y,z):

    return [x, y-0.24, z-1.22]

def main():

    print("state machine....................... ʕ•ᴥ•ʔ")
    rospy.init_node("act_pln") 
    rospy.wait_for_service("/vision/obj_segmentation/get_obj_pose")
    clt_pose_obj = rospy.ServiceProxy("/vision/obj_segmentation/get_obj_pose", RecognizeObject)
    rospy.wait_for_service("/vision/get_best_grasp_traj")
    clt_best_grip = rospy.ServiceProxy("/vision/get_best_grasp_traj", InverseKinematicsPose2Traj )
    rospy.wait_for_service( '/manipulation/la_ik_trajectory' )
    clt_ik = rospy.ServiceProxy( '/manipulation/la_ik_trajectory' , InverseKinematicsPose2Traj )


    pub_la_goal_traj = rospy.Publisher("/manipulation/la_q_trajectory" , JointTrajectory, queue_size=10)
    pub_la_goal_grip = rospy.Publisher("/hardware/left_arm/goal_gripper" , Float64, queue_size=10)
    pub_hd_goal_pose = rospy.Publisher("/hardware/head/goal_pose"     , Float64MultiArray, queue_size=10)
    

    new_command = False
    executing_command = False
    state = SM_INIT

    loop = rospy.Rate(30)
    while not rospy.is_shutdown():

        if state == SM_INIT:
            print("Starting State Machine..............")
            state = SM_WAITING_NEW_COMMAND

        elif state == SM_WAITING_NEW_COMMAND:
            state = SM_MOVE_HEAD

        elif state == SM_MOVE_HEAD:
            move_head(pub_hd_goal_pose ,0, -0.9)
            print("state == SM_MOVE_HEAD")
            state = SM_WAIT_FOR_HEAD

        elif state == SM_WAIT_FOR_HEAD:
            move_head(pub_hd_goal_pose ,0, -0.9)
            state = SM_GET_OBJ_POSE


        elif state == SM_GET_OBJ_POSE:
            print("state == SM_GET_OBJ  *")
            x,y,z = get_obj_pose(clt_pose_obj)
            x,y,z = transform_to_la(x,y,z)
            state = SM_MOVE_LEFT_ARM
            

        elif state == SM_MOVE_LEFT_ARM:
            move_left_arm( [x,y,z,0,-1.5,-0.3] , pub_la_goal_traj, clt_ik)
            print("state == SM_MOVE_ARM")
            state = -1

        else:
            break

        loop.sleep()
        
        

if __name__ == '__main__':
    main()


