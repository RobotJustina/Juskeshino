#!/usr/bin/env python3

import rospy
import numpy as np
import tf
import math
import cv2
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PointStamped, PoseStamped, Point, Pose, Twist
from vision_msgs.srv import *
from manip_msgs.srv import *
from hri_msgs.msg import *
import time
from std_msgs.msg import String, Float64MultiArray, Float32, Float64,Bool, Float32MultiArray
from actionlib_msgs.msg import GoalStatus
from tf.transformations import euler_from_quaternion
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from cv_bridge import CvBridge


SM_INIT = 0
SM_WAIT_FOR_NAVIGATE = 1
SM_NAVIGATE = 5
SM_WAITING_NEW_COMMAND = 10
SM_RECO_OBJ = 12
SM_GET_OBJ_POSE = 50
SM_MOVE_HEAD = 60
SM_WAIT_FOR_HEAD = 70
SM_PREPARE_ARM = 80
SM_MOVE_LEFT_ARM = 100 
VIRTUAL_LOCATION = [5.8, 4.36, 0.0]
LEFT_TABLE_NEAR = [5.45, 2.45, np.deg2rad(90)]
LEFT_TABLE_FAR = [6.95, 2.3, 0.0]
RIGTH_TABLE = [5.7, 0.28, np.deg2rad(-90)]
LOCAL_TARGET = LEFT_TABLE_NEAR
PREPARE = [-1.27, 0.4, 0.0, 1.9, 0.01, 0.69, -0.01]#[-1.2, 0.1, 0.0, 1.6, 0.0, 1.1, 0.0]
HOME = [0,0,0,0,0,0]


def callback_goal_reached(msg):
    global goal_reached
    goal_reached = msg.status == 3


def callback_hd_goal_reached(msg):
    global goal_hd_reached
    goal_hd_reached = msg.data



def callback_la_goal_reached(msg):
    global goal_la_reached
    
    goal_la_reached = msg.data


def go_to_goal_pose(pub_goal_pose, local_target):
    loop = rospy.Rate(10)
    goal_pose = PoseStamped()
    goal_pose.pose.orientation.w = math.cos(local_target[2] /2)
    goal_pose.pose.orientation.z = math.sin(local_target[2] /2)
    goal_pose.pose.position.x = local_target[0]
    goal_pose.pose.position.y = local_target[1]
    print("sending goal pose", goal_pose)
    counter = 3
    while not rospy.is_shutdown() and counter > 0:
        pub_goal_pose.publish(goal_pose)
        counter -=1
        loop.sleep()




def move_base(pub_cmd_vel, linear, angular):
    cmd = Twist()
    cmd.linear.x = linear
    cmd.angular.z = angular
    pub_cmd_vel.publish(cmd)
    rospy.sleep(2)
    pub_cmd_vel.publish(Twist())



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



def q2q_traj(p_final, clt_traj_planner, pub_traj):
    initial_pose = rospy.wait_for_message("/hardware/left_arm/current_pose", Float64MultiArray, 5.0)
    initial_pose = initial_pose.data
    request = GetPolynomialTrajectoryRequest()
    request.p2 = p_final
    request.p1 = initial_pose
    request.duration = 5
    request.time_step = 0.02
    resp_traj = clt_traj_planner(request)
    resp_traj.trajectory
    pub_traj.publish(resp_traj.trajectory)
 


def move_left_gripper(q, pubLaGoalGrip):
    pubLaGoalGrip.publish(q)
    time.sleep(1.0)



def get_obj_pose(clt_pose_obj):
    recognize_object_req = RecognizeObjectRequest()
    try:
        recognize_object_req.point_cloud = rospy.wait_for_message("/hardware/realsense/points" , PointCloud2, timeout=2)
        #recognize_object_req.point_cloud = rospy.wait_for_message("/camera/depth_registered/points" , PointCloud2, timeout=2)
    except:
        return None
    return clt_pose_obj(recognize_object_req)





def move_head(pub_hd_goal_pose, pan, tilt):
    msg = Float64MultiArray()
    msg.data.append(pan)
    msg.data.append(tilt)
    pub_hd_goal_pose.publish(msg)
    rospy.sleep(3.0)


    #return [x, y-0.24, z-1.22]
def transform_to_la(x,y,z, f_actual, f_target,listener):
    point_msg = PointStamped()  
    point_msg.header.frame_id = f_actual   # frame de origen
    point_msg.header.stamp = rospy.Time(0)  # la ultima transformacion
    point_msg.point.x = x
    point_msg.point.y = y
    point_msg.point.z = z
    point_target_frame = listener.transformPoint(f_target, point_msg)
    new_point = point_target_frame.point
    return [ new_point.x , new_point.y , new_point.z ]



# Para conocer la pose del robot
def get_robot_pose(listener):
    try:
        ([x, y, z], rot) = listener.lookupTransform('map', 'base_link', rospy.Time(0))
        a = 2*math.atan2(rot[2], rot[3])
        a = a - 2*math.pi if a > math.pi else a
        return [x, y, a]
    except:
        pass
    return [0,0,0]



def callback_recognized_speech(msg):
    global recognized_speech, new_task, executing_task
    # Si se esta ejecuntando una tarea deja de almacenar comandos de voz entrantes y de imprimir en pantalla
    if executing_task:  
        return
    new_task = True
    recognized_speech = msg.hypothesis
    print(recognized_speech)



def main():

    print("state machine....................... ʕ•ᴥ•ʔ")
    rospy.init_node("act_pln") 
    global goal_reached, goal_hd_reached, goal_la_reached

    rospy.wait_for_service("/vision/obj_segmentation/get_obj_pose")
    clt_pose_obj = rospy.ServiceProxy("/vision/obj_segmentation/get_obj_pose", RecognizeObject)
    rospy.wait_for_service("/vision/obj_reco/recognize_object")
    clt_recognize_object = rospy.ServiceProxy("/vision/obj_reco/recognize_object", RecognizeObjects)
    rospy.wait_for_service("/vision/get_best_grasp_traj")
    clt_best_grip = rospy.ServiceProxy("/vision/get_best_grasp_traj", BestGraspTraj )
    rospy.wait_for_service( '/manipulation/la_ik_trajectory' )
    clt_ik = rospy.ServiceProxy( '/manipulation/la_ik_trajectory' , InverseKinematicsPose2Traj )
    rospy.wait_for_service( '/manipulation/polynomial_trajectory')
    clt_traj_planner = rospy.ServiceProxy( '/manipulation/polynomial_trajectory' , GetPolynomialTrajectory )

    pub_la_goal_traj = rospy.Publisher("/manipulation/la_q_trajectory" , JointTrajectory, queue_size=10)
    pub_la_goal_grip = rospy.Publisher("/hardware/left_arm/goal_gripper" , Float64, queue_size=10)
    pub_la_goal_q    = rospy.Publisher("/hardware/left_arm/goal_pose" , Float64MultiArray, queue_size=10)
    pub_hd_goal_pose = rospy.Publisher("/hardware/head/goal_pose"     , Float64MultiArray, queue_size=10)
    pub_goal_pose    = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
    pub_cmd_vel      = rospy.Publisher('/hardware/mobile_base/cmd_vel', Twist, queue_size=10)
    #pubSay           = rospy.Publisher('/hri/speech_generator', SoundRequest, queue_size=10)
    rospy.Subscriber('/navigation/status',Bool ,callback_hd_goal_reached)
    rospy.Subscriber('/manipulation/head/goal_reached',Bool ,callback_hd_goal_reached)
    rospy.Subscriber('/manipulation/left_arm/goal_reached',Bool , callback_la_goal_reached)
    rospy.Subscriber('/hri/sp_rec/recognized', RecognizedSpeech, callback_recognized_speech)
    
    listener = tf.TransformListener()

    new_command = False
    executing_command = False
    recognized_speech = ""
    goal_reached = False
    state = SM_INIT
    loop = rospy.Rate(30)

    
    while not rospy.is_shutdown():
        
        if state == SM_INIT:
            print("Starting State Machine..............")
            state = SM_WAITING_NEW_COMMAND

        elif state == SM_WAITING_NEW_COMMAND:
            print("state == SM_WAITING_NEW_COMMAND")
            rospy.sleep(1.0)
            request = recognized_speech
            print(recognized_speech)
            rospy.sleep(3.0)

            if "ROBOT" in request:
                print("Se ha hecho una peticion")
                #parse_command(recognized_speech)
                state = -1

            else:                
                print("No se han recibido nuevas peticiones")
                print("Regresando a E1...")
                state = SM_INIT
            #state = SM_NAVIGATE
            #state = -1#SM_MOVE_HEAD

        elif state == SM_NAVIGATE:
            #global goal_reached
            print("going to the target place....")
            go_to_goal_pose(pub_goal_pose,LOCAL_TARGET)
            #goal_reached = 1
            state =  SM_WAIT_FOR_NAVIGATE

        elif state == SM_WAIT_FOR_NAVIGATE:
            if goal_reached:
                x_p, y_p, a = get_robot_pose(listener)
                print("Se llego al lugar solicitado con Pose: ", x_p,y_p,a )
                #rospy.sleep(1.0)
                state = SM_MOVE_HEAD

        elif state == SM_MOVE_HEAD:
            move_head(pub_hd_goal_pose ,0, -1.0)
            move_head(pub_hd_goal_pose ,0, -1.0)
            print("state == SM_MOVE_HEAD")
            #rospy.sleep(1.0)
            state = SM_WAIT_FOR_HEAD

        elif state == SM_WAIT_FOR_HEAD:
            print("SM_WAIT_FOR_HEAD")
            """
            if goal_hd_reached:
                print("succesfull move head ")
                rospy.sleep(1.0)
            """
            #rospy.sleep(1.0)
            state = SM_GET_OBJ_POSE


        elif state == SM_GET_OBJ_POSE:
            print("state == SM_GET_OBJ..........")
            resp_pose_obj = get_obj_pose(clt_pose_obj)
            x, y ,z = resp_pose_obj.recog_object.pose.position.x, resp_pose_obj.recog_object.pose.position.y, resp_pose_obj.recog_object.pose.position.z
            i = 0
            z_temp = z*10
            z_temp2 = str(z)
            while  z_temp <= 0 or z_temp2 == 'nan': 
                resp_pose_obj = get_obj_pose(clt_pose_obj)
                x, y ,z = resp_pose_obj.recog_object.pose.position.x, resp_pose_obj.recog_object.pose.position.y, resp_pose_obj.recog_object.pose.position.z
                z_temp = z*10
                z_temp2 = str(z)
                print("                     ")
                print("Intento:", i)
                print("                     ")
                i = i + 1
                if i > 15:
                    break
            print("position object ", x,y,z)    
            state = SM_RECO_OBJ

        

        elif state == SM_RECO_OBJ:
            print("state == SM_RECO_OBJ")
            reco_obj_req = RecognizeObjectsRequest()
            # LLenar msg
            #reco_obj_req.point_cloud = 
            ros_image_obj = resp_pose_obj.recog_object.image        #imagen del objeto
            ros_image_mask = resp_pose_obj.image
            bridge = CvBridge()
            cv_image_obj = bridge.imgmsg_to_cv2(ros_image_obj, "bgr8")
            #cv2.imshow("imagen reconstruida", cv_image_obj) #*****************
            #cv2.waitKey(0)

            reco_obj_req.image = ros_image_obj
            reco_obj_req.mask = ros_image_mask
            reco_obj_resp = clt_recognize_object(reco_obj_req)

            print("Objeto reconocido:_________")
            print(reco_obj_resp.recog_objects[0].id)
            #reco_obj_resp.image
            state = SM_PREPARE_ARM


        elif state == SM_PREPARE_ARM:
            print("state == SM_PREPARE_ARM")
            p_final = PREPARE
            q2q_traj(p_final, clt_traj_planner, pub_la_goal_traj)
            """
            print(goal_la_reached)
            if goal_la_reached:
                print("succesfull move arm ")
                time.sleep(1)
                state = SM_GET_OBJ_POSE
            """
            state = SM_MOVE_LEFT_ARM
            

        elif state == SM_MOVE_LEFT_ARM:
            req_best_grip = BestGraspTrajRequest()
            req_best_grip.recog_object = resp_pose_obj.recog_object
            resp_best_grip = clt_best_grip(req_best_grip)
            print("state == SM_MOVE_ARM")
            move_left_gripper(0.9, pub_la_goal_grip)
            if resp_best_grip.graspable:
                #pub_la_goal_traj.publish(resp_best_grip.articular_trajectory)
                print("moviendo brazo izquierdo...................")
                rospy.sleep(7.0)
            else:
                print("No se encontraron poses posibles...................")
            
            move_left_gripper(0.1, pub_la_goal_grip)
            rospy.sleep(1.0)
            """
            move_left_gripper(0.5, pub_la_goal_grip)
            rospy.sleep(1.0)
            move_left_gripper(0.1, pub_la_goal_grip)
            goal_pose = []
            goal_pose.append(resp_best_grip.x)
            goal_pose.append(resp_best_grip.y)
            goal_pose.append(resp_best_grip.z + 0.20)
            goal_pose.append(resp_best_grip.roll)
            goal_pose.append(resp_best_grip.pitch)
            goal_pose.append(resp_best_grip.yaw)
            #move_left_arm(goal_pose,  pub_la_goal_traj, clt_ik)
            """
            

            state = -1

        else:
            break

        loop.sleep()
        
        

if __name__ == '__main__':
    main()

