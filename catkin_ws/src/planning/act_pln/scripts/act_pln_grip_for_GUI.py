#!/usr/bin/env python3

import rospy
import numpy as np
import tf
import math
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PointStamped, PoseStamped, Point, Pose, Twist
from vision_msgs.srv import *
from manip_msgs.srv import *
from hri_msgs.msg import *
from sound_play.msg import SoundRequest
import time
from std_msgs.msg import String, Float64MultiArray, Float32, Float64,Bool, Float32MultiArray
from actionlib_msgs.msg import GoalStatus
from tf.transformations import euler_from_quaternion
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from cv_bridge import CvBridge
from actionlib_msgs.msg import GoalStatus



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
SM_RETURN_LOCATION = 11
SM_GRASP_OBJECT = 13
SM_PICK_UP_OBJECT = 14
SM_LIFT_OBJECT = 15
SM_PREPARE = 111
SM_CLEAN_VARIABLES = 2

# left arm poses
PREPARE_TOP_GRIP = [-0.68, 0.38, -0.01, 1.84, 0, 1.06, -0.01]#[-0.9, 0.4, 0.0, 1.9, 0.01, 1, -0.01]  #funciona para pringles horizontal (prisma horizontal)
PREPARE_LATERAL_GRIP = [-0.69, 0.2, 0, 1.55, 0, 1.16,0] #[-1.2, 0.2, 0  , 1.6, 0   , 1,     0] #Prepare original:funciona bien para pringles vertical (prisma vertical) 
TAKEN_OBJECT_VERTICAL = [0.46, 0.87, -0.4, 1.99, -0.99, 0.4, 1.6]
TAKEN_OBJECT_HORIZONTAL = [0.46, 0.87, -0.4, 1.99, -0.99, 0.4, 0.41]#[1, -0.08, -0.029, 0.44, 0, 0.66, -1.39]
HOME = [0,0,0,0,0,0]
LIFT_OBJECT = [0.11, 0.2, 0.0, 1.75, 0.0, 1.36, 0.0]
PREPARE     = [-0.7, 0.2, 1.55, 0.0, 1.16, 0.0, 0.0]
PREPARE_HIGH      = [-0.5, 0, 0, 2.4, 0, 0.5,0]

GRIPPER_OPENING = 0.9   # Apertura de gripper


def callback_goal_reached(msg): #¿?
    global goal_reached
    print("STATUS", msg.status)
    if msg.status == 3: 
        goal_reached = 1


def callback_la_goal_reached(msg):  # left arm
    global goal_la_reached
    goal_la_reached = msg.data


def go_to_goal_pose(pub_goal_pose, local_target):   # movil base
    loop = rospy.Rate(10)
    goal_pose = PoseStamped()
    goal_pose.pose.orientation.w = math.cos(local_target[2] /2)
    goal_pose.pose.orientation.z = math.sin(local_target[2] /2)
    goal_pose.pose.position.x = local_target[0]
    goal_pose.pose.position.y = local_target[1]
    
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



def q2q_traj(p_final, clt_traj_planner, pub_traj):
    initial_pose = rospy.wait_for_message("/hardware/left_arm/current_pose", Float64MultiArray, 5.0)
    initial_pose = initial_pose.data
    request = GetPolynomialTrajectoryRequest()
    request.p2 = p_final
    request.p1 = initial_pose
    request.duration = 3
    request.time_step = 0.02
    resp_traj = clt_traj_planner(request)
    resp_traj.trajectory
    pub_traj.publish(resp_traj.trajectory)
 


def move_left_gripper(q, pubLaGoalGrip):
    pubLaGoalGrip.publish(q)
    time.sleep(1.0)



def get_obj_pose(clt_pose_obj, obj_pc2, image):
    recognize_object_req = RecognizeObjectRequest()
    recognize_object_req.point_cloud = obj_pc2
    recognize_object_req.image = image
    return clt_pose_obj(recognize_object_req)



def move_head(pub_hd_goal_pose, pan, tilt):
    msg = Float64MultiArray()
    msg.data.append(pan)
    msg.data.append(tilt)
    pub_hd_goal_pose.publish(msg)
    rospy.sleep(3.0)



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



def say(pub_say, text):
    msg = SoundRequest()
    msg.sound   = -3
    msg.command = 1
    msg.volume  = 1.0
    msg.arg2    = "voice_kal_diphone"
    msg.arg = text
    pub_say.publish(msg)
    time.sleep(1.5)



def callback_take_object(msg):
    global object_name
    print("MENSAJE PUBLICADO EN TOPICO /plannning/take_object....")
    object_name = msg.data
    print("objeto solicitado:_____:", object_name)
    
    
    
def pub_status_msg_response(status, pub_object_status):
    resp = GoalStatus()
    if status == 3:     # SUCCEEDED
        resp.status = 3
    else:
        resp.status = 4 # ABORTED
    
    pub_object_status.publish(resp)



def main():

    print("STATE MACHINE.......................")
    rospy.init_node("act_pln_grip") 
    global goal_reached, goal_hd_reached, goal_la_reached, object_name
    global executing_command, new_command, recognized_speech

    rospy.wait_for_service("/vision/obj_segmentation/get_obj_pose") # Servicio que da la pose del objeto a tomar
    clt_pose_obj          = rospy.ServiceProxy("/vision/obj_segmentation/get_obj_pose", RecognizeObject)
    rospy.wait_for_service("/vision/obj_reco/detect_and_recognize_objects") # Servicio que reconoce el objeto
    rospy.wait_for_service("/manipulation/get_best_grasp_traj")   # Servicio que da la mejor pose de agarre
    rospy.wait_for_service( '/manipulation/polynomial_trajectory')  # Servicio para generar trayectorias para el brazo
    
    clt_recognize_objects = rospy.ServiceProxy("/vision/obj_reco/detect_and_recognize_objects", RecognizeObjects)
    clt_best_grip         = rospy.ServiceProxy("/manipulation/get_best_grasp_traj" , BestGraspTraj )
    clt_traj_planner      = rospy.ServiceProxy( '/manipulation/polynomial_trajectory' , GetPolynomialTrajectory )

    pub_la_goal_traj  = rospy.Publisher("/manipulation/la_q_trajectory" , JointTrajectory, queue_size=10)
    pub_la_goal_grip  = rospy.Publisher("/hardware/left_arm/goal_gripper" , Float64, queue_size=10)
    pub_la_goal_q     = rospy.Publisher("/hardware/left_arm/goal_pose" , Float64MultiArray, queue_size=10)
    pub_hd_goal_pose  = rospy.Publisher("/hardware/head/goal_pose"     , Float64MultiArray, queue_size=10)
    pub_goal_pose     = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
    pub_cmd_vel       = rospy.Publisher('/hardware/mobile_base/cmd_vel', Twist, queue_size=10)
    pub_object_status = rospy.Publisher('/plannning/simple_task/object_status' , GoalStatus, queue_size=1 )

    rospy.Subscriber('/navigation/status', GoalStatus ,callback_goal_reached)
    rospy.Subscriber('/manipulation/left_arm/goal_reached',Bool , callback_la_goal_reached)
    rospy.Subscriber('/plannning/simple_task/take_object' ,String ,callback_take_object )

    
    listener = tf.TransformListener()
    new_command = False
    executing_command = False
    recognized_speech = ""
    goal_reached = False
    state = SM_INIT
    loop = rospy.Rate(30)
    goal_la_reached = False
    
    while not rospy.is_shutdown():
        
        if state == SM_INIT:
            #print("Starting State Machine by Iby.................ʕ•ᴥ•ʔ")
            print("waiting for an object to be requested.............................ʕ•ᴥ•ʔ")
            object_name_msg = rospy.wait_for_message('/plannning/simple_task/take_object' , String)
            obj_target = object_name_msg.data
            print("OBJECT TARGET:____", obj_target)
            x_p, y_p, a = get_robot_pose(listener)
            STARTING_PLACE = [x_p, y_p, a]
            state = SM_MOVE_HEAD 


        elif state == SM_MOVE_HEAD:
            move_head(pub_hd_goal_pose ,0, -1.0)
            move_head(pub_hd_goal_pose ,0, -1.0)
            print("state == SM_MOVE_HEAD")
            state = SM_RECO_OBJ



        elif state == SM_RECO_OBJ:
            print("state == SM_RECO_OBJ")
            reco_objs_req = RecognizeObjectsRequest()
            # LLenar msg
            
            reco_objs_req.point_cloud = rospy.wait_for_message("/camera/depth_registered/points" , PointCloud2, timeout=2)
            
            reco_objs_resp = clt_recognize_objects(reco_objs_req)
            recog_objects = reco_objs_resp.recog_objects    # Accede a la lista de VisionObjects
            print("Objetos reconocidos:_________")

            for obj in recog_objects:   # Para cada objeto de la lista de VisionObjects
                print("* " + obj.id)
                if obj_target == obj.id:
                    print("Se encontró el objeto pedido.................")
                    # msg para el servicio \vision\obj_pose
                    recognize_object_req = RecognizeObjectRequest()
                    recognize_object_req.point_cloud = obj.point_cloud
                    recognize_object_req.image = obj.image
                    recognize_object_req.obj_mask = obj.obj_mask

                    state = SM_GET_OBJ_POSE
                    break
                else:
                    state = SM_INIT
                    print("try to grab the object again")
                    resp = pub_status_msg_response(4, pub_object_status)  # ABORTED

                


        elif state == SM_GET_OBJ_POSE:
            print("state == SM_GET_OBJ..........")
            resp_pose_obj = clt_pose_obj(recognize_object_req)
            x, y ,z = resp_pose_obj.recog_object.pose.position.x, resp_pose_obj.recog_object.pose.position.y, resp_pose_obj.recog_object.pose.position.z
            print("graspable:_____ ", resp_pose_obj.recog_object.graspable )
            shape_obj = resp_pose_obj.recog_object.size
            print("SHAPE OBJ:___", shape_obj)
            if not resp_pose_obj.recog_object.graspable:
                print("Request an object again............")

            print("position object ", x,y,z)    
            state = SM_PREPARE_ARM
            
            
        elif state == SM_PREPARE_ARM:
            print("state == SM_PREPARE_ARM")
            if (resp_pose_obj.recog_object.size.x < 0.11) or resp_pose_obj.recog_object.object_state == "horizontal":
                q2q_traj(PREPARE_TOP_GRIP , clt_traj_planner, pub_la_goal_traj)
            else:
                q2q_traj(PREPARE_LATERAL_GRIP , clt_traj_planner, pub_la_goal_traj)

            
            while (not goal_la_reached) or not rospy.is_shutdown:
                print("status: moving arm....", goal_la_reached)                
                time.sleep(1)
            
            goal_la_reached =  False       
            print("goal_la_reached STATUS", goal_la_reached)  
            
            q2q_traj(PREPARE_HIGH, clt_traj_planner, pub_la_goal_traj)
            while (not goal_la_reached) or not rospy.is_shutdown:
                print("status: moving arm....", goal_la_reached)                
                time.sleep(1)
            
            goal_la_reached =  False       
            print("goal_la_reached STATUS", goal_la_reached) 
          
            state = SM_GRASP_OBJECT


            
        elif state == SM_GRASP_OBJECT:
            print("state == SM_GRASP_OBJECT")
            req_best_grip = BestGraspTrajRequest()
            req_best_grip.recog_object = resp_pose_obj.recog_object
            resp_best_grip = clt_best_grip(req_best_grip)
            goal_la_reached =  False       
            print("goal_la_reached STATUS", goal_la_reached)
            #state = SM_INIT
            
            move_left_gripper(0.9, pub_la_goal_grip)
            
            if resp_best_grip.graspable:
                move_left_gripper(GRIPPER_OPENING, pub_la_goal_grip)
                print("publicando trayectoria en q para brazo izquierdo...................")
                state = SM_INIT
            
                
                pub_la_goal_traj.publish(resp_best_grip.articular_trajectory)
                goal_la_reached =  False
                print("goal_la_reached STATUS", goal_la_reached)
                while (not goal_la_reached) or not rospy.is_shutdown:

                    print("goal_la_reached status: moving arm....", goal_la_reached)
                    time.sleep(1)
                    
                if goal_la_reached:
                    print("succesfull move arm...")
                    
                    print("goal_la_reached STATUS", goal_la_reached)
                    state =  SM_PICK_UP_OBJECT
                
                
            else:
                print("No possible poses found...................")
                resp = pub_status_msg_response(4, pub_object_status)  # ABORTED
                state = SM_INIT
            
            
            
            
        elif state == SM_PICK_UP_OBJECT:
            print("state == SM_PICK_UP_OBJECT")
            goal_la_reached =  False
            print("goal_la_reached STATUS", goal_la_reached)
            time.sleep(1.5)
            move_left_gripper(-0.3 , pub_la_goal_grip)
            resp = pub_status_msg_response(3, pub_object_status)  # SUCCEEDED
            state = SM_INIT#SM_LIFT_OBJECT
   


        elif state == SM_LIFT_OBJECT:
            print("Cambiando posicion de brazo a prepare....")
            goal_la_reached =  False       
            print("goal_la_reached STATUS", goal_la_reached)
            q2q_traj(LIFT_OBJECT , clt_traj_planner, pub_la_goal_traj)

            while (not goal_la_reached) or not rospy.is_shutdown:
                print("status: moving arm....")
                time.sleep(1)

            goal_la_reached = False
            state = SM_INIT
            
            # MOVER BASE HACIA ATRAS
            

        elif state == SM_PREPARE:
            print("Cambiando posicion de brazo a prepare....")
            goal_la_reached =  False       
            print("goal_la_reached STATUS", goal_la_reached)
            q2q_traj(PREPARE , clt_traj_planner, pub_la_goal_traj)

            while (not goal_la_reached) or not rospy.is_shutdown:
                print("status: moving arm....")
                time.sleep(1)

            goal_la_reached = False
            state = SM_INIT

            
        elif state == SM_CLEAN_VARIABLES:
            new_command = False
            executing_command = False
            recognized_speech = ""
            goal_reached = False
            goal_la_reached = False
            state = SM_INIT
            s



        else:
            break

        loop.sleep()
        
        

if __name__ == '__main__':
    main()

