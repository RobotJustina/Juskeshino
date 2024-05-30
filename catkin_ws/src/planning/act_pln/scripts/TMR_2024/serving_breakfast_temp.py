#!/usr/bin/env python3
import rospy
import rospkg
import time
import numpy as np
import math
import yaml
from juskeshino_tools.JuskeshinoNavigation import JuskeshinoNavigation
from juskeshino_tools.JuskeshinoVision import JuskeshinoVision
from juskeshino_tools.JuskeshinoHardware import JuskeshinoHardware
from juskeshino_tools.JuskeshinoSimpleTasks import JuskeshinoSimpleTasks
from juskeshino_tools.JuskeshinoHRI import JuskeshinoHRI
from juskeshino_tools.JuskeshinoManipulation import JuskeshinoManipulation
from juskeshino_tools.JuskeshinoKnowledge import JuskeshinoKnowledge


# Left arm
HOME              = [0,0,0,0,0,0,0]
PREPARE           = [-0.8, 0.2, 0.0, 1.55, 0.0, 1.24, 0.0]#[-0.69, 0.2, 0.0, 1.55, 0.0, 1.16, 0.0]
PREPARE_TOP_GRIP  = [-1.25, 0.3, 0, 2.4, 0, 0.7,0]
PREPARE_SERVING   = [0.91, 0.4, -0.5, 1.45, 0, 0.16, 0.5]
SERVING           = [0.91, 0.4, -0.5, 1.45, 0, 0.16, -1.6]
LEAVE_CEREAL      = [0.54, 0.28, -0.13, 1.45, 0, 0, 0]
LEAVE_MILK        = [0.44, 0.18, -0.03, 1.45, 0, 0, 0]
LEAVE_BOWL        = [0.6,  0.6, -0.8, 1.7, 0, 0.2, 0]

# RIght arm
PREPARE_RA    = [-0.8, -0.1, 0.0, 1.3, 1.3,0, 0.0]


OBJECTS_TABLE = "desk_justina"
EAT_TABLE     = "desk_takeshi" 

# Gripper_aperture
GRIP_MILK   = 0.3
GRIP_BOWL   = -0.1
GRIP_CEREAL = 0.0



def serving_breakfast(object):
    print("PREPARE TOP")
    JuskeshinoHRI.say("Prepare arm")
    time.sleep(0.5)
    JuskeshinoHardware.moveLeftArmWithTrajectory(PREPARE_TOP_GRIP, 10)
    JuskeshinoHRI.say("Prepare serving")
    time.sleep(0.5)
    JuskeshinoHardware.moveLeftArmWithTrajectory(PREPARE_SERVING, 10)
    
    #JuskeshinoNavigation.moveDist(0.3, 7)      # mueve la base adelante con el brazo levantado y extendido
    if object =="milk": 
        JuskeshinoHRI.say("Serving milk")
    else: 
        JuskeshinoHRI.say("Serving cereal")
    time.sleep(0.5)
    JuskeshinoHardware.moveLeftArmWithTrajectory(SERVING, 10)
    JuskeshinoHRI.say("Prepare serving")
    time.sleep(0.5)
    JuskeshinoHardware.moveLeftArmWithTrajectory(PREPARE_SERVING, 10)
    time.sleep(0.5)

    if object =="milk":
        JuskeshinoHRI.say("leave milk")
        time.sleep(0.5)
        JuskeshinoHardware.moveLeftArmWithTrajectory(LEAVE_MILK, 10)
        #JuskeshinoNavigation.moveDist(0.3, 7)
        JuskeshinoHardware.moveLeftGripper(0.7, 2.0)
        
    else:
        JuskeshinoHRI.say("move right")
        time.sleep(0.5)
        JuskeshinoNavigation.moveLateral(0.22, 10)
        JuskeshinoHRI.say("leave cereal")
        time.sleep(0.5)
        JuskeshinoHardware.moveLeftArmWithTrajectory(LEAVE_CEREAL, 10)
        
        #JuskeshinoNavigation.moveDist(0.3, 7)
        #time.sleep(0.2)#
        JuskeshinoHardware.moveLeftGripper(0.7, 2.0)



def approach_to_table():
    JuskeshinoHRI.say("I will try to align with table")
    i = 0
    while i <4:
        if (not JuskeshinoSimpleTasks.alignWithTable()):
            JuskeshinoHRI.say("Cannot align with table")
            time.sleep(0.3)
            JuskeshinoNavigation.moveDist(0.10, 10)
        
        else:
            JuskeshinoHRI.say("Align with table")
            break
        i = i+1



def broadcaster_frame_object(frame, child_frame, pose):  
    #br = tf2_ros.TransformBroadcaster()
    br =  tf2_ros.StaticTransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()
    t.header.frame_id = frame
    t.child_frame_id = child_frame 
    t.header.stamp = rospy.Time.now()
    t.transform.translation.x = pose.position.x
    t.transform.translation.y = pose.position.y
    t.transform.translation.z = pose.position.z
    t.transform.rotation.x = pose.orientation.x
    t.transform.rotation.y = pose.orientation.y
    t.transform.rotation.z = pose.orientation.z
    t.transform.rotation.w = pose.orientation.w
    br.sendTransform(t)



def pose_actual_to_pose_target(pose, f_actual, f_target):
    global listener
    poseStamped_msg = PoseStamped()  
    poseStamped_msg.header.frame_id = f_actual   # frame de origen
    poseStamped_msg.header.stamp = rospy.Time()  # la ultima transformacion
    poseStamped_msg.pose = pose

    try:
        listener.waitForTransform(f_actual , target_frame, rospy.Time(0), rospy.Duration(10.0))
        print("waitfor ..despues")
        new_poseStamped = listener.transformPose(f_target , poseStamped_msg)
        new_pose = new_poseStamped.pose
        return new_pose
    
    except:
        print("Best_Grasp_Node.-> Could not get the pose in the desired frame")
        return -1

 

def points_actual_to_points_target(point_in, f_actual, f_target):
    global listener
    point_msg = PointStamped()  
    point_msg.header.frame_id = f_actual   # frame de origen
    point_msg.header.stamp = rospy.Time() # la ultima transformacion
    point_msg.point.x = point_in[0]
    point_msg.point.y = point_in[1]
    point_msg.point.z = point_in[2]

    listener.waitForTransform(f_actual, f_target, rospy.Time(), rospy.Duration())
    point_target_frame = listener.transformPoint(f_target, point_msg)
    new_point = point_target_frame.point
    return [ new_point.x , new_point.y , new_point.z ]

def modify_location(pose_obj, location):  # position_obj is in frame 'base_link'
    pose_obj = pose_actual_to_pose_target(pose_obj , "base_link" , "map")
    pos_obj.position.x



def categorize_object(obj_id):      # Recibe un string que indica el nombre del objeto encontrado
    with open('Juskeshino/catkin_ws/src/config_files/groceries_classification_rm.yaml', 'r') as file:
    print("In categories")
    try:
        f = open(obj_yaml,'r')
        data = yaml.safe_load(f)
    except:
        data = {}
        print("KnownLocations.->Cannot load locations from file ")

    for category, items in data.items():
        if obj_id.lower() in [item.lower() for item in items]:
            rospy.loginfo(f"--->>> Detected {obj_id} matches an item in the '{category}' category.")
            # Do something with the matched object
            return category  
    return False



def main():
    print("INITIALIZING SERVE BREAKFAST 2024 TEST BY ITZEL..............ヾ(๑╹◡╹)ﾉ")
    rospy.init_node("serve_breakfast_test")
    rate = rospy.Rate(10)
    #pubLaGoalGrip = rospy.Publisher("/hardware/left_arm/goal_gripper", Float64, queue_size=1)

    # Se subcribe a los servicios necesarios para manipulacion, navegacion,vision, etc...
    JuskeshinoManipulation.setNodeHandle()
    JuskeshinoNavigation.setNodeHandle()
    JuskeshinoVision.setNodeHandle()
    JuskeshinoHardware.setNodeHandle()
    JuskeshinoSimpleTasks.setNodeHandle()
    JuskeshinoHRI.setNodeHandle()
    JuskeshinoKnowledge.setNodeHandle()


    pila = ["banana","bowl", "milk", "small_cereal"]                             
    count = 0
    j = 0

    # START************************************************************************************
    JuskeshinoHRI.say("I'm ready for the test")

    # Waiting for the door to be open 
    JuskeshinoHRI.say("I'm waiting for the door to be open")
    if not JuskeshinoSimpleTasks.waitForTheDoorToBeOpen(1000):
        print("ACT-PLN.->Door never opened")
        return
    else: JuskeshinoHRI.say("I can see now that the door is open")
     

    # transport object
    while count < 3: 
        actual_obj = pila[count]
        # Ir a locacion de ingredientes
        JuskeshinoHRI.say("I'm going to the "+ OBJECTS_TABLE)
        
        if not JuskeshinoNavigation.getClose(OBJECTS_TABLE , 120): 
            JuskeshinoHRI.say("SB-PLN.->Cannot get close to the "+ OBJECTS_TABLE +" position")
        time.sleep(0.5)

        # Apunta camara a la mesa
        if not JuskeshinoHardware.moveHead(0,-1, 5):
            try:
                JuskeshinoHRI.say("SB-PLN.->Cannot move head")
                time.sleep(0.2)
                JuskeshinoHardware.moveHead(0,-1, 5)
            except:
                JuskeshinoHRI.say("Cannot move head")

        time.sleep(0.3)
        approach_to_table()
            
        j=0 # atemps
        while j < 2:    
            # Busqueda y reconocimiento del objeto 2 veces
            JuskeshinoHRI.say("Trying to detect the object")
            
            [obj, img] = JuskeshinoVision.detectAndRecognizeObjectWithoutOrientation(actual_obj)   
                #JuskeshinoHRI.say("I found" + actual_obj)
            if (obj == None):
                JuskeshinoHRI.say("I couldn't find the object")
                
                [obj, img] = JuskeshinoVision.detectAndRecognizeObjectWithoutOrientation(actual_obj)   
                #print("SB-PLN.->Detected object : " ,obj.id , obj.category, obj.object_state, obj.pose.position)
                #JuskeshinoHRI.say("I found" + actual_obj)
                #categorize_object(obj.id)

                if (obj == None):
                    JuskeshinoHRI.say("I couldn't find the object")

            print("SB-PLN.->Detected object : " ,obj.id , obj.category, obj.object_state, obj.pose.position)
            categorize_object(obj.id)
            print("SB-PLN.-> RECOGNITION RESULT POSITION OBJECT : " ,obj.pose.position)

                

            # El robot se mueve a la mejor mejor ubicacion de agarre ,Toma objeto con brazo izquierdo
            mov = JuskeshinoSimpleTasks.handling_location_la(obj.pose.position)

            # Ajusta altura de torso para mejor agarre
            if (obj.pose.position.z > 0.70) and (actual_obj == "bowl"):
                try:
                    JuskeshinoHardware.moveTorso(np.linalg.norm(obj.pose.position.z - 0.70) , 5.0)
                    time.sleep(1.3)
                except:
                    JuskeshinoHRI.say("Cannot move torso")
            if (actual_obj == "milk"):
                try:
                    JuskeshinoHardware.moveTorso(0.05 , 5.0)
                    time.sleep(1.3)
                except:
                    JuskeshinoHRI.say("Cannot move torso")
            if ((actual_obj == "small_cereal")):
                try:
                    JuskeshinoHardware.moveTorso(0.06 , 5.0)
                    time.sleep(1.3)
                except:
                    JuskeshinoHRI.say("Cannot move torso")




            # En la mejor ubicacion de agarre realiza de nuevo un reconocimiento del objeto***********************
            JuskeshinoHRI.say("Trying to detect the object")
            try:
                [obj, img] = JuskeshinoVision.detectAndRecognizeObject(actual_obj)   
                print("SB-PLN.->RECOGNITION WITH POSE : " + str([obj.id, obj.category, obj.object_state, obj.pose.position]))
                JuskeshinoHRI.say("I found" + actual_obj)
                category = categorize_object(obj.id)
                if category:
                    prompt = "I found" + obj.id + " which is part of the category " + category
                    JuskeshinoHRI.say(prompt)
                    print("---------------------I found:", obj.id, "which is part of the category: ", category)

            except:
                JuskeshinoHRI.say("I couldn't find the object")                        
            

            
            
            # Tomar objeto   
            if (actual_obj == "bowl") or (actual_obj == "small_cereal"): 
                JuskeshinoHardware.moveLeftArmWithTrajectory(PREPARE_TOP_GRIP, 10)  # prepare 
                JuskeshinoHRI.say("Open gripper")
                JuskeshinoHardware.moveLeftGripper(0.4 , 2.0)
            else: 
                JuskeshinoHardware.moveLeftArmWithTrajectory(PREPARE, 10)  # prepare 
                JuskeshinoHRI.say("Open gripper")
                JuskeshinoHardware.moveLeftGripper(0.9, 1.0)

            JuskeshinoHRI.say("looking for proper grip")
            [resp, graspable] = JuskeshinoManipulation.GripLa(obj)
            if graspable:
                JuskeshinoHRI.say("graspable")
                print("SB-PLN.->object position", obj.pose.position)
                JuskeshinoHardware.moveLeftArmWithTrajectory(resp.articular_trajectory,15)
                JuskeshinoHRI.say("Closing gripper")
                time.sleep(0.2)
                
                if (actual_obj == "bowl"):
                    if(JuskeshinoHardware.moveLeftGripper(-0.1 , 3.0) ):
                        JuskeshinoHRI.say("gripper close")
                    else: 
                        JuskeshinoHRI.say("Cannot close gripper")
                        JuskeshinoHardware.moveLeftGripper(0.1 , 1.0)

                if (actual_obj == "milk"):
                    time.sleep(0.2)
                    JuskeshinoHardware.moveLeftGripper(GRIP_MILK , 3.0) 
                if (actual_obj == "small_cereal"):
                    time.sleep(0.2)
                    JuskeshinoHardware.moveLeftGripper(GRIP_CEREAL , 3.0) 

                time.sleep(0.5)
                print("ACT-PLN.->Moving arm to prepare***")
                JuskeshinoHardware.moveLeftArmWithTrajectory(PREPARE_TOP_GRIP, 10)  # prepare    
                break
            else:
                JuskeshinoHRI.say("No possible poses found")
                print("SB-PLN.->No possible poses found")

            j = j+1
        
        # Despues de que agarro el objeto va a la mesa del desayuno***********************************************************************
        try:
            JuskeshinoHardware.moveTorso(0.02 , 5.0)
            time.sleep(1.3)
        except:
                print("Cannot move torso")
        print("SB-PLN.->Moving base backwards")     # Se mueve hacia atras para planear la ruta a la mesa del desayuno
        #JuskeshinoHRI.say("I'have grasped the object")
        JuskeshinoNavigation.moveDist(-0.3, 10)
        time.sleep(0.5)

        #.........................................................................................................................
        # Ir a la mesa del desayuno...............................................................................................
        #.........................................................................................................................
        
        # Navega hacia la segunda mesa
        print("SB-PLN.->Getting close to " + EAT_TABLE + "location")
        JuskeshinoHRI.say("I'm going to the" + EAT_TABLE + "location")

        if not JuskeshinoNavigation.getClose(EAT_TABLE, 100):  
            print("SB-PLN.->Cannot get close to " + EAT_TABLE +" position")


        JuskeshinoHRI.say("I have arrived at the" + EAT_TABLE + " location")
        time.sleep(0.5)
        
        print("SB-PLN.->move head")
        if not JuskeshinoHardware.moveHead(0,-1, 5):
            print("SB-PLN.->Cannot move head")
            time.sleep(0.5)
            JuskeshinoHardware.moveHead(0,-1, 5)
        time.sleep(1)
        # Se alinea con la mesa
        approach_to_table()
        time.sleep(0.3)
    
        # Coloca objeto sobre la mesa
        print("SB-PLN.->Moving left arm to deliver position")

        if (actual_obj == "milk") or (actual_obj == "small_cereal"):
            time.sleep(1)
            serving_breakfast(actual_obj) 
        if actual_obj == "bowl":
            JuskeshinoHRI.say("Leave bowl")
            JuskeshinoHardware.moveLeftArmWithTrajectory(LEAVE_BOWL , 12)
            time.sleep(0.6)
            print("SB-PLN.->Open gripper")
            JuskeshinoHardware.moveLeftGripper(0.7, 2.0)
            time.sleep(1)            # Soltar el objeto

        # Moverse para atras
        print("SB-PLN.->Moving base backwards")
        JuskeshinoNavigation.moveDist(-0.36, 7)
        time.sleep(0.5)
        JuskeshinoHardware.moveLeftArmWithTrajectory(HOME , 10)

        count = count + 1

            
    # El desayuno esta servido 
    JuskeshinoHRI.say("Breakfast is served. I finish the test")
    
    return 
    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == "__main__":
    main()