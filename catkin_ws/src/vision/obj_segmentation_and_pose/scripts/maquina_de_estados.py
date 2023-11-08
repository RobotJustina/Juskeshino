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

# ROBOT REAL LOCATION
LEFT_TABLE_NEAR = [5.45, 2.45, np.deg2rad(90)]
LIVINGROOM = [5.2, 2.33, np.deg2rad(90)]
KITCHEN = [5.35, 2.33, np.deg2rad(90)]
STARTING_PLACE= [0,0,0]
# ROBOT VIRTUAL LOCATION
V_LIVINGROOM = [5.2, 2.33, np.deg2rad(90)]
V_KITCHEN = [3.3, 5.56 , np.deg2rad(-90)]
V_STARTING_PLACE= [5.6 , 4.5, 0]

# left arm poses
PREPARE_TOP_GRIP = [-0.9, 0.4, 0.0, 1.9, 0.01, 1, -0.01]  #funciona para pringles horizontal (prisma horizontal)
PREPARE_LATERAL_GRIP =  [-1.2, 0.2, 0  , 1.6, 0   , 1,     0] #Prepare original:funciona bien para pringles vertical (prisma vertical) 
TAKEN_OBJECT_VERTICAL = [0.98, 0.22, -1.47, 1.76, 0.15, 0.5, 0.5]  #[0.05, 0.95, -0.35, 1.87, -1, 0.3, -0.9]
TAKEN_OBJECT_HORIZONTAL = []
HOME = [0,0,0,0,0,0]
GRIPPER_OPENING = 0.9   # Apertura de gripper

simulate = False


def callback_goal_reached(msg): #¿?
    global goal_reached
    print("STATUS", msg.status)
    if msg.status == 3: 
        goal_reached = 1


def callback_hd_goal_reached(msg):  # head
    global goal_hd_reached
    goal_hd_reached = msg.data


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
    request.duration = 5
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



def parse_command(cmd):
    obj = "pringles" if "PRINGLES" in cmd else "drink"
    location = V_KITCHEN if "KITCHEN" in cmd else LIVINGROOM  # Ubicacion de LIVINGROOM y kitchen
    return location, obj


def say(pub_say, text):
    msg = SoundRequest()
    msg.sound   = -3
    msg.command = 1
    msg.volume  = 1.0
    msg.arg2    = "voice_kal_diphone"
    msg.arg = text
    pub_say.publish(msg)
    time.sleep(1.5)


def callback_recognized_speech(msg):
    global recognized_speech, new_command, executing_command
    print("msg______", msg)
    if not executing_command:  
        new_command = True
        recognized_speech = msg.hypothesis[0]



def move_arm():
    print("llalala")



def main():

    print("STATE MACHINE.......................")
    rospy.init_node("act_pln") 
    global goal_reached, goal_hd_reached, goal_la_reached
    global executing_command, new_command, recognized_speech

    rospy.wait_for_service("/vision/obj_segmentation/get_obj_pose")
    clt_pose_obj = rospy.ServiceProxy("/vision/obj_segmentation/get_obj_pose", RecognizeObject)
    """
    rospy.wait_for_service("/vision/obj_reco/recognize_object")
    clt_recognize_object = rospy.ServiceProxy("/vision/obj_reco/recognize_object", RecognizeObjects)
    """
    rospy.wait_for_service("/vision/obj_reco/detect_and_recognize_objects")
    clt_recognize_objects = rospy.ServiceProxy("/vision/obj_reco/detect_and_recognize_objects", RecognizeObjects)

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
    pub_say           = rospy.Publisher('/hri/speech_generator', SoundRequest, queue_size=10)
    rospy.Subscriber('/navigation/status', GoalStatus ,callback_goal_reached)
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
    goal_la_reached = False
    
    while not rospy.is_shutdown():
        
        if state == SM_INIT:
            print("Starting State Machine by Iby.................ʕ•ᴥ•ʔ")
            obj_target = "pringles"
            print("OBJECT TARGET:____", obj_target)
            x_p, y_p, a = get_robot_pose(listener)
            STARTING_PLACE = [x_p, y_p, a]
            state = SM_MOVE_HEAD #SM_WAITING_NEW_COMMAND

        elif state == SM_WAITING_NEW_COMMAND:
            print("state == SM_WAITING_NEW_COMMAND") 
            #say(pub_say , "Hello, I am waiting for new task")
            rospy.sleep(3.0)
            request = recognized_speech
            print(request)
        
            if "ROBOT" in request:
                print("Se ha hecho una peticion")
                #say(pub_say , "has been requested"+recognized_speech)
                local_target, obj_target = parse_command(request)
                obj_target = "apple"
                state = SM_MOVE_HEAD#SM_NAVIGATE
            else:                
                print("No se han recibido nuevas peticiones")
                print("Regresando a E1...")
                state = SM_INIT

        elif state == SM_NAVIGATE:
            print("going to the target place....")
            go_to_goal_pose(pub_goal_pose, local_target)
            goal_reached = False
            while not goal_reached and (not rospy.is_shutdown()):
                time.sleep(1)

            print("Se llego al lugar solicitado")
            state =  SM_WAIT_FOR_NAVIGATE

        elif state == SM_WAIT_FOR_NAVIGATE:
            if goal_reached:
                x_p, y_p, a = get_robot_pose(listener)
                print("Se llego al lugar solicitado con Pose: ", x_p,y_p,a )
                rospy.sleep(1.0)
                state = SM_MOVE_HEAD

        elif state == SM_MOVE_HEAD:
            move_head(pub_hd_goal_pose ,0, -1.0)
            move_head(pub_hd_goal_pose ,0, -1.0)
            print("state == SM_MOVE_HEAD")
            state = SM_WAIT_FOR_HEAD

        elif state == SM_WAIT_FOR_HEAD:
            print("SM_WAIT_FOR_HEAD")
            """
            if goal_hd_reached:
                print("succesfull move head ")
                rospy.sleep(1.0)
            """
            state = SM_RECO_OBJ



        elif state == SM_RECO_OBJ:
            print("state == SM_RECO_OBJ")
            reco_objs_req = RecognizeObjectsRequest()
            # LLenar msg
            
            reco_objs_req.point_cloud = rospy.wait_for_message("/hardware/realsense/points" , PointCloud2, timeout=2)
            #reco_objs_req.point_cloud = rospy.wait_for_message("/camera/depth_registered/points" , PointCloud2, timeout=2)
            
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
                else:state = -1


        elif state == SM_GET_OBJ_POSE:
            print("state == SM_GET_OBJ..........")
            resp_pose_obj = clt_pose_obj(recognize_object_req)
            x, y ,z = resp_pose_obj.recog_object.pose.position.x, resp_pose_obj.recog_object.pose.position.y, resp_pose_obj.recog_object.pose.position.z
            print("graspable:_____ ", resp_pose_obj.recog_object.graspable )
            shape_obj = resp_pose_obj.recog_object.size
            print("SHAPE OBJ:___", shape_obj)
            if not resp_pose_obj.recog_object.graspable:
                print("request an object again............")
                state = SM_WAITING_NEW_COMMAND

            print("position object ", x,y,z)    
            state = SM_PREPARE_ARM
            
            
        elif state == SM_PREPARE_ARM:
            print("state == SM_PREPARE_ARM")
            if (resp_pose_obj.recog_object.size.x < 0.11) or resp_pose_obj.recog_object.object_state == "horizontal":
                p_final = PREPARE_TOP_GRIP
                print("PREPARE_TOP_GRIP..........................")
            else:
                p_final = PREPARE_LATERAL_GRIP
                print("PREPARE_LATERAL_GRIP..........................")


            q2q_traj(p_final, clt_traj_planner, pub_la_goal_traj)
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
            move_left_gripper(0.9, pub_la_goal_grip)
            
            if resp_best_grip.graspable:
                move_left_gripper(GRIPPER_OPENING, pub_la_goal_grip)
                print("publicando trayectoria en q para brazo izquierdo...................")
                pub_la_goal_traj.publish(resp_best_grip.articular_trajectory)
                goal_la_reached =  False
                print("goal_la_reached STATUS", goal_la_reached)
                while (not goal_la_reached) or not rospy.is_shutdown:

                    print("goal_la_reached status: moving arm....", goal_la_reached)
                    time.sleep(1)
                    
                if goal_la_reached:
                    print("succesfull move arm...")
                    
                    print("goal_la_reached STATUS", goal_la_reached)
                    state = -1#SM_PICK_UP_OBJECT
                
            else:
                print("No se encontraron poses posibles...................")
                state = -1
            
            
            
        elif state == SM_PICK_UP_OBJECT:
            print("state == SM_PICK_UP_OBJECT")
            decrement = GRIPPER_OPENING 
            goal_la_reached =  False
            print("goal_la_reached STATUS", goal_la_reached)

            while (decrement > -0.3):
                move_left_gripper(decrement , pub_la_goal_grip)
                # time.sleep(0.001)
                decrement = decrement - 0.25
                print("DECREMENT:_____", decrement)
            state = SM_LIFT_OBJECT
   


        elif state == SM_LIFT_OBJECT:
            print("Cambiando posicion de brazo ....")
            goal_la_reached =  False       
            print("goal_la_reached STATUS", goal_la_reached)
            q2q_traj(TAKEN_OBJECT_VERTICAL , clt_traj_planner, pub_la_goal_traj)
            while (not goal_la_reached) or not rospy.is_shutdown:
                print("status: moving arm....")
                time.sleep(1)

            goal_la_reached = False
            state = -1


        elif state == SM_RETURN_LOCATION:
            #global goal_reached
            local_target = STARTING_PLACE
            print("Returning to the starting place...."+ str(local_target))
            go_to_goal_pose(pub_goal_pose, local_target)
            goal_reached = 1
            state =  SM_WAIT_FOR_NAVIGATE


        elif state == SM_WAIT_FOR_NAVIGATE:
            if goal_reached:
                x_p, y_p, a = get_robot_pose(listener)
                print("Se llego al lugar solicitado con Pose: ", x_p,y_p,a )
                rospy.sleep(1.0)
                state = -1
            

        else:
            break

        loop.sleep()
        
        

if __name__ == '__main__':
    main()

