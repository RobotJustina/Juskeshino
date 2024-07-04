#!/usr/bin/env python3
import rospy
import rospkg
import time
import numpy as np
import math
import tf.transformations as tft
import tf2_ros
import tf
import numpy as np
import math
from geometry_msgs.msg import PointStamped, Point
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
LEAVE_BOWL        = [0.6,  0.6, -0.8, 1.7, 0, -0.1, 0]

POST_GRIP         = [0.38, 0.19, -0.01, 1.57, 0 , 0.35, 0.0 ]

# RIght arm
PREPARE_RA    = [-0.8, -0.1, 0.0, 1.3, 1.3,0, 0.0]

# Locations
OBJECTS_TABLE = "objects_table_robocup"#"desk_justina"
EAT_TABLE     = "breakfast_table_robocup"#"desk_takeshi" 
OBJECTS_TABLE_THETA = [5.44 ,2.15, 1.5]


# Objects
BOWL   = "red_bowl"
MILK   = "jelly"
CEREAL = "mango_juice"

# Gripper_aperture
GRIP_MILK   = 0.4
GRIP_BOWL   = -0.05
GRIP_CEREAL = 0.1


# ESTADOS
INITIAL = 1
CONFIG  = 2
START = 3
MOVE_TO_LOCATION = 4    # Ir a locacion de ingredientes
MOVE_HEAD = 5
DETECT_OBJECT = 6
APROACH_TO_TABLE = 7
HANDLING_LOCATION = 8   # El robot se mueve a la mejor mejor ubicacion de agarre ,Toma objeto con brazo izquierdo
PREPARE_ARM = 9
LOOKING_GRIP = 10
TAKE_OBJECT = 11
POST_GRASP = 12 
GO_TO_KITCHEN = 13
PUT_OBJECT = 14
APROACH_TO_TABLE_2 = 15
CYCLE_END = 16
CONFIG_BY_CYCLE = 17
END = 18
DETECT_OBJECT_ORIENTATION = 19
BEGIN_CYCLE = 20
MOVE_TO_LOCATION_OBJECTS = 21
VERIFY_SUITABLE_LOCATION_GRASP = 22




# Categorias 
def categorize_objs(name):
    # 'fork', 'knife', 'mug', 'sponge', 'b_cups', 'c_cups', 'e_cups', 'f_cups', 'dice', 'marker', 'rubiks_cube'
    kitchen = ['red_bowl', 'fork', 'knife', 'mug', 'plate', 'spatula', 'sponge', 'spoon', 'b_cups', 'c_cups', 'e_cups', 'f_cups']
    # 'extra_large_clamp', 'large_clamp', 'small_clamp', 'medium_clamp'
    tools = ['adjustable_wrench', 'flat_screwdriver', 'phillips_screwdriver', 'wood_block']
    # 'softball', 'a_mini_soccer_ball', 'racquetball', 'golf_ball',
    balls = ['tennis_ball', 'baseball']
    fruits = ['apple', 'banana', 'lemon', 'pear']  # 'plum', 'orange'
    food = ['chips_can', 'mango_juice','mustard_bottle', 'potted_meat_can','jelly', 'tomato_soup_can', 'tuna_fish_can', 'pudding_box', 'cracker_box']  # 'master_chef_can', 'sugar_box'
    other = ['dice', 'marker', 'rubiks_cube']
    if name in kitchen:
        return 'kitchen'
    elif name in tools:
        return 'tools'
    elif name in balls:
        return 'balls'
    elif name in fruits:
        return 'fruits'
    elif name in food:
        return 'food'
    elif name in other:
        return 'other'
    return 'unknown'

def serving_breakfast(object):
    print("PREPARE TOP")
    #JuskeshinoHRI.say("Prepare arm")
    #time.sleep(0.5)
    #JuskeshinoHardware.moveLeftArmWithTrajectory(PREPARE_TOP_GRIP, 10)
    JuskeshinoHRI.say("Prepare serving")
    time.sleep(0.5)
    JuskeshinoHardware.moveLeftArmWithTrajectory(PREPARE_SERVING, 10)
    
    #JuskeshinoNavigation.moveDist(0.3, 7)      # mueve la base adelante con el brazo levantado y extendido
    if object ==MILK: 
        JuskeshinoHRI.say("Serving milk")
    else: 
        JuskeshinoHRI.say("Serving cereal")
    time.sleep(0.5)
    JuskeshinoHardware.moveLeftArmWithTrajectory(SERVING, 10)
    JuskeshinoHRI.say("Prepare serving")
    time.sleep(0.5)
    JuskeshinoHardware.moveLeftArmWithTrajectory(PREPARE_SERVING, 10)
    time.sleep(0.5)

    if object ==MILK:
        JuskeshinoHRI.say("leave milk")
        time.sleep(0.5)
        JuskeshinoHardware.moveLeftArmWithTrajectory(LEAVE_MILK, 10)
        #JuskeshinoNavigation.moveDist(0.3, 7)
        JuskeshinoHardware.moveLeftGripper(0.7, 2.0) 
    else:
        #JuskeshinoHRI.say("move right")
        #time.sleep(0.5)
        JuskeshinoNavigation.moveLateral(0.15, 10)
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




def detect_obj_function(actual_obj):
    print("in function")
    count = 0
    while(count < 13):
        time.sleep(0.2)
        try:
            [obj, img] = JuskeshinoVision.detectAndRecognizeObject()
            print("(*B*)")
            if obj.id == actual_obj:
                print("ID:___", obj.od)
                return obj
            else:
                continue
        except:
            print("I cannot find the objects")
            JuskeshinoHRI.say("I cannot find the objects")
            count = count + 1
    return None



def main():
    rospy.init_node("serve_breakfast_test")
    rate = rospy.Rate(10)
    global listener, simu

    listener = tf.TransformListener()
    simu = False
    torso = True
    actual_value = 0
    ALTURA_TORSO = 0.11

    current_state = INITIAL
    while not rospy.is_shutdown():

        if(current_state == INITIAL):
            print("ESTADO:___INITIAL.................")
            print("INITIALIZING SERVE BREAKFAST 2024 TEST BY ITZEL..............ヾ(๑╹◡╹)ﾉ")
            current_state = CONFIG




        elif(current_state == CONFIG):
            print("ESTADO:___CONFIG.................")
            # Se subcribe a los servicios necesarios para manipulacion, navegacion,vision, etc...
            JuskeshinoManipulation.setNodeHandle()
            JuskeshinoNavigation.setNodeHandle()
            JuskeshinoVision.setNodeHandle()
            JuskeshinoHardware.setNodeHandle()
            JuskeshinoSimpleTasks.setNodeHandle()
            JuskeshinoHRI.setNodeHandle()
            JuskeshinoKnowledge.setNodeHandle()
            
            pila = [BOWL, CEREAL, MILK]
            count = 0
            j = 0
            actual_obj = pila[j]
            tries = 0
            grip_attempts = 0
            cycle = 0
            print("cycle:___", cycle)
            mesa_alta = True
            align_with_table = True
            current_state = START




        elif(current_state == START):
            print("ESTADO:___START..................")
            JuskeshinoHRI.say("I'm ready for the test")
            #JuskeshinoHRI.say("I'm waiting for the door to be open")
            if not JuskeshinoSimpleTasks.waitForTheDoorToBeOpen(10):
                print("SERVING BREAKFAST-> Puerta cerrada")
                current_state = START
            else: 
                print("SERVING BREAKFAST-> Puerta abierta")
                JuskeshinoHRI.say("The door is open")
                location_actual = OBJECTS_TABLE
                current_state = BEGIN_CYCLE





        elif(current_state == BEGIN_CYCLE):
            print("ESTADO:__________BEGIN CYCLE.............")
            print("mesa alta", mesa_alta)
            print("cycle:___", cycle)
            print("j:___", j)
            print("Actual object", pila[j])

            current_state = MOVE_TO_LOCATION





        elif(current_state == MOVE_TO_LOCATION):
            print("ESTADO:___MOVE_TO_LOCATION..................")
            #JuskeshinoHRI.say("I'm going to the "+ location_actual)
            
            if not JuskeshinoNavigation.getClose(location_actual , 300): 
                JuskeshinoHRI.say("Cannot get close to the "+ location_actual +" position")
                print("SB-PLN.->Cannot get close to the "+ location_actual +" position")
                current_state =  MOVE_HEAD

            else:
                print("SERVING BREAKFAST-> Se llego con exito a la locacion solicitada")
                current_state = MOVE_HEAD
            time.sleep(0.5)




     
        elif(current_state == MOVE_HEAD):
            print("ESTADO:___MOVE_HEAD..................")
            try:
                JuskeshinoHardware.moveHead(0,-1, 5)
                current_state = DETECT_OBJECT
            except:
                JuskeshinoHRI.say("Cannot move head")
                JuskeshinoHardware.moveHead(0,-1, 5)
                current_state = DETECT_OBJECT

            


        elif(current_state == DETECT_OBJECT):
            print("ESTADO:____DETECT_OBJECT..................")
            try:
                [obj, img] = JuskeshinoSimpleTasks.object_search(actual_obj)
                """
                time.sleep(0.2)
                [obj, img] = JuskeshinoVision.detectAndRecognizeObjectWithoutOrientation(actual_obj)
                """   
                time.sleep(0.2)
                print("SB-PLN.->Detected object : " ,obj.id , obj.category, obj.object_state, obj.pose.position)
                JuskeshinoHRI.say("I found" + actual_obj.replace("_", " ") + "of the category" + categorize_objs(obj.id))
                
                current_state = HANDLING_LOCATION

            except:
                JuskeshinoHRI.say("I couldn't find the object")
                tries = tries + 1
                current_state = DETECT_OBJECT
                if tries > 3:
                    JuskeshinoHRI.say("I didn't find the object")
                    print("SB-PLN.-> Se excedió el numero de intentos")
                    current_state = DETECT_OBJECT_ORIENTATION
 



        elif(current_state == HANDLING_LOCATION):
            print("ESTADO:____HANDLING_LOCATION..................")
            pos_obj_bl = [obj.pose.position.x, obj.pose.position.y, obj.pose.position.z]
            mov_flag, dist = JuskeshinoNavigation.getCloseSuitableGripPositionLa(OBJECTS_TABLE , pos_obj_bl, 100.0)
            JuskeshinoHardware.moveHead(0,-1, 5)
            time.sleep(0.2)
            if mov_flag:
                JuskeshinoNavigation.moveDist(0.18, 7.0)
            # Ajusta altura de torso para mejor agarre
            if (not simu) or (torso):
                if(mesa_alta):
                    JuskeshinoHardware.moveTorso(ALTURA_TORSO , 5.0)

            tries = 0
            """
            if(mov_flag):
                current_state = DETECT_OBJECT
            else:
            """
            current_state = DETECT_OBJECT_ORIENTATION

            


        elif(current_state == DETECT_OBJECT_ORIENTATION):
            print("ESTADO:____DETECT_OBJECT_ORIENTATION..................")
            
            JuskeshinoHRI.say("Trying to detect the object")
            print("Trying to detect the object:__", actual_obj)

            time.sleep(0.2)
            try:
                [obj, img] = JuskeshinoSimpleTasks.object_search_orientation(actual_obj)
                #[obj, img] = JuskeshinoVision.detectAndRecognizeObject(actual_obj)   
                print("SB-PLN.->Detected object : " ,obj.id , obj.category, obj.object_state, obj.pose.position)
                JuskeshinoHRI.say("I found" + actual_obj.replace("_", " "))
                current_state = PREPARE_ARM
            except:
                JuskeshinoHRI.say("I couldn't find the object")
                try:
                    [obj, img] = JuskeshinoSimpleTasks.object_search_orientation(actual_obj)
                    #[obj, img] = JuskeshinoVision.detectAndRecognizeObjectWithoutOrientation(actual_obj)
                    time.sleep(0.2)
                    print("SB-PLN.->Detected object : " ,obj.id , obj.category, obj.object_state, obj.pose.position)
                    JuskeshinoHRI.say("I found" +actual_obj.replace("_", " "))
                    
                    current_state = PREPARE_ARM

                except:
                    JuskeshinoHRI.say("I couldn't find")

                    print("tries", tries)
                    tries = tries + 1
                    current_state = DETECT_OBJECT_ORIENTATION
                    if tries > 2:
                        JuskeshinoHRI.say("oooo")
                        print("SB-PLN.-> Se excedió el numero de intentos")
                        current_state = PREPARE_ARM
            
            print("SB-PLN.->Detected object : " ,obj.id , obj.category, obj.object_state, obj.pose.position)
                      
            


            
        elif(current_state == PREPARE_ARM):
            print("ESTADO:___PREPARE_ARM..................")
            if((obj.category == "CUBIC") or (obj.category == "BOWL") or (obj.category == "BOX") or (obj.object_state == "horizontal")):
                JuskeshinoHardware.moveLeftArmWithTrajectory(PREPARE_TOP_GRIP, 10)  # prepare 
                if(obj.category == "BOWL"):
                    JuskeshinoHRI.say("Open gripper")
                    APERTURE = 0.4
                    JuskeshinoHardware.moveLeftGripper(APERTURE , 5.0)
                else:
                    APERTURE = 0.9
                    JuskeshinoHardware.moveLeftGripper(APERTURE , 5.0)
            else: 
                JuskeshinoHardware.moveLeftArmWithTrajectory(PREPARE, 10)  # prepare 
                JuskeshinoHRI.say("Open gripper")
                APERTURE = 1.0
                JuskeshinoHardware.moveLeftGripper(APERTURE , 5.0)

            current_state = LOOKING_GRIP





        elif(current_state == LOOKING_GRIP):
            print("ESTADO:___LOOKING_GRIP..................")
            print("Grip attempts:______", grip_attempts)
            JuskeshinoHRI.say("looking for proper grip")

            [resp, graspable] = JuskeshinoManipulation.GripLa(obj)
            if graspable:
                JuskeshinoHRI.say("graspable")
                print("SB-PLN.->object position", obj.pose.position)
                resp = resp.articular_trajectory
                current_state = TAKE_OBJECT
            else:
                grip_attempts = grip_attempts + 1
                if grip_attempts < 3:

                    current_state = DETECT_OBJECT_ORIENTATION

                else:
                    JuskeshinoHRI.say("No possible poses found")
                    print("SB-PLN.->No possible poses found")
                    resp = resp.q
                
                    current_state = TAKE_OBJECT

                


        
        
        elif(current_state == TAKE_OBJECT):
            print("ESTADO:___TAKE_OBJECT..................")
            JuskeshinoHardware.moveLeftArmWithTrajectory(resp ,15)
            time.sleep(0.5)
            JuskeshinoHRI.say("Closing gripper")
            count_grip = 0.1
            actual_value = APERTURE 
                
            JuskeshinoManipulation.dynamic_grasp_left_arm(is_thin = True)
            """
            if (actual_obj == BOWL):
                while (actual_value >= GRIP_BOWL):
                    actual_value = APERTURE -count_grip
                    print("Actual value:___", actual_value)
                    JuskeshinoHardware.moveLeftGripper(actual_value , 5.0)
                    count_grip = count_grip + 0.1

            if (actual_obj == MILK):
                while (actual_value >= GRIP_MILK):
                    actual_value = APERTURE -count_grip
                    print("Actual value:___", actual_value)
                    JuskeshinoHardware.moveLeftGripper(actual_value , 5.0)
                    count_grip = count_grip + 1

            if (actual_obj == CEREAL):
                while (actual_value >= GRIP_CEREAL):
                    actual_value = APERTURE -count_grip
                    print("Actual value:___", actual_value)
                    JuskeshinoHardware.moveLeftGripper(actual_value , 5.0)
                    count_grip = count_grip + 1
            """

            current_state = POST_GRASP





        elif(current_state == POST_GRASP):
            print("ESTADO:___POST_GRASP..................")
            print("SB-PLN.->Moving base backwards")    
            JuskeshinoNavigation.moveDist(-0.33, 10)
            if (not simu) or (torso):
                try:
                    JuskeshinoHardware.moveTorso(0.02 , 5.0)
                    time.sleep(1)
                except:
                    print("Cannot move torso")
            JuskeshinoHardware.moveLeftArmWithTrajectory(PREPARE_TOP_GRIP, 10)
            
            current_state = GO_TO_KITCHEN




        elif(current_state == GO_TO_KITCHEN):
            print("ESTADO:____GO_TO_KITCHEN..................")
            JuskeshinoHRI.say("I'm going to " + EAT_TABLE)
            if not JuskeshinoNavigation.getClose(EAT_TABLE, 300):  
                print("SB-PLN.->Cannot get close to " + EAT_TABLE +" position")
                JuskeshinoHRI.say("Cannot get close to " + EAT_TABLE)
                
            current_state = APROACH_TO_TABLE_2




        elif(current_state == APROACH_TO_TABLE_2):
            print("ESTADO:____APROACH_TO_TABLE_2..................")
            print("SB-PLN.->move head")
            if not JuskeshinoHardware.moveHead(0,-1, 5):
                print("SB-PLN.->Cannot move head")
                time.sleep(0.5)
                JuskeshinoHardware.moveHead(0,-1, 5)
            time.sleep(0.5)

            #approach_to_table()
            time.sleep(0.3)
            current_state = PUT_OBJECT




    
        elif(current_state == PUT_OBJECT):
            print("ESTADO:____PUT_OBJECT...........................")
            if (actual_obj == MILK) or (actual_obj == CEREAL):
                time.sleep(1)
                serving_breakfast(actual_obj) 
            if actual_obj == BOWL:
                JuskeshinoHRI.say("Leave bowl")
                JuskeshinoHardware.moveLeftArmWithTrajectory(LEAVE_BOWL , 12)
                time.sleep(0.6)
                print("SB-PLN.->Open gripper")
                JuskeshinoHardware.moveLeftGripper(0.7, 2.0)
                time.sleep(0.5)            # Soltar el objeto

            current_state = CYCLE_END



        

        elif(current_state == CYCLE_END):
            print("ESTADO:____CYCLE_END")
            JuskeshinoNavigation.moveDist(-0.50, 7)
            time.sleep(0.5)
            JuskeshinoHardware.moveLeftArmWithTrajectory(HOME , 10)
            time.sleep(0.2)
            JuskeshinoHardware.moveLeftGripper(0.0, 2.0)

            current_state = CONFIG_BY_CYCLE





        elif(current_state == CONFIG_BY_CYCLE):
            print("ESTADO:___CONFIG_BY_CYCLE")
            cycle = cycle + 1
            print("CYCLE:_____", cycle)
            if cycle > 2:
                JuskeshinoHRI.say("Breakfast is served. I finish the test")
                current_state = END
            else:
                j = j + 1
                tries = 0
                grip_attempts = 0
                actual_obj = pila[j]
                print("ACTUAL OBJECT:_____", actual_obj)

                current_state = MOVE_TO_LOCATION    


        


        elif(current_state == END):     # El desayuno esta servido 
            print("ESTADO:____END")
            JuskeshinoHRI.say("Breakfast is served. I finish the test")
            current_state = -1


    
    
    return 
    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == "__main__":
    main()