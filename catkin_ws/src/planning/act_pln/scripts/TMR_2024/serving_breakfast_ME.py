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
from std_msgs.msg import Float64MultiArray
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
PREPARE_SERVING   = [0.91, 0.4, -0.5, 1.15, 0, 0.16, 0.5]
SERVING           = [0.91, 0.4, -0.5, 1.15, 0, 0.16, -1.6]
LEAVE_CEREAL      = [0.54, 0.28, -0.13, 1.25, 0, 0, 0]
LEAVE_MILK        = [0.44, 0.18, 0.37, 1.15, 0, 0, 0]
LEAVE_BOWL        = [0.6,  0.6, -0.8, 1.7, 0, -0.1, 0]
LEAVE_BOWL_2        = [0.6,  0.6, -0.1, 1.7, 0.0,-0.1, 0]
CARRY_BOWL        = [-0.9, 0.2, 0.0, 2.05, 0.0, -0.64, 0.0]

POST_GRIP         = [0.38, 0.19, -0.01, 1.57, 0 , 0.35, 0.0 ]

# RIght arm
PREPARE_RA    = [-0.8, -0.1, 0.0, 1.3, 1.3,0, 0.0]

# Locations
OBJECTS_TABLE = "location_desk_justina"#
EAT_TABLE     = "desk_takeshi" 
LOCATIONS     = ["location_desk_justina" , "location_desk_justina", "location_desk_justina"]


# Objects
BOWL   = "apple"#"pudding_box"#"potted_meat_can"#"apple"#"sausages"#"bowl"
MILK   = "chips_can"
CEREAL = "rubiks_cube"

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
HANDLING_LOCATION_2 = 23




# Categorias 
def categorize_objs(name):
    cleaning_supplies = ['dishwasher_tab', 'soap', 'sponges', 'washcloth']
    decorations = ['candle']
    dishes = ['bowl','cup','fork','knife','plate','spoon']
    drinks = ['big_coke','cola','dubbelfris', 'fanta', 'ice_tea', 'milk','water']
    food = ['cornflakes','curry','hagelslag','mayonaise','pancake_mix', 'pea_soup', 'sausages']
    fruits = ['apple','banana','lemon','orange','peach','pear','plum','strawberry']
    snacks = ['candy','crisps','liquorice','pringles','stroopwafel','tictac']
    if name in cleaning_supplies:
        return 'cleaning supplies'
    elif name in decorations:
        return 'decorations'
    elif name in dishes:
        return 'dishes'
    elif name in drinks:
        return 'drinks'
    elif name in food:
        return 'food'
    elif name in fruits:
        return 'fruits'
    elif name in snacks:
        return 'snacks'
    
    return 'unknown'

def serving_breakfast(object):
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
    simu = True
    torso = False
    actual_value = 0
    ALTURA_TORSO = 0.20
    ALTURA_TORSO_BOWL = 0.2#0.22


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
            
            JuskeshinoHardware.moveTorso(0.7 , 10.0)
            
            pila = [BOWL, CEREAL, MILK]
            count = 0
            j = 0
            actual_obj = pila[j]
            location_actual = LOCATIONS[j]
            tries = 0
            grip_attempts = 0
            cycle = 0
            print("cycle:___", cycle)
        
            
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
            print("cycle:___", cycle)
            print("j:___", j)
            print("Actual object", pila[j])
            print("Actual locations", location_actual )
            #JuskeshinoNavigation.moveDist(1.5,10
            #JuskeshinoHRI.say("Could you please remove all the chairs on the table?")
            current_state = MOVE_TO_LOCATION





        elif(current_state == MOVE_TO_LOCATION):
            print("ESTADO:___MOVE_TO_LOCATION..................")
            JuskeshinoHRI.say("I'm going to the "+ location_actual)
            location_atual = OBJECTS_TABLE
            if not JuskeshinoNavigation.getClose(OBJECTS_TABLE , 300): 
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
                JuskeshinoHardware.moveHead(0,-0.9, 5)
                current_state = DETECT_OBJECT
            except:
                JuskeshinoHRI.say("Cannot move head")
                JuskeshinoHardware.moveHead(0,-0.9, 5)
                current_state = DETECT_OBJECT

            


        elif(current_state == DETECT_OBJECT):
            print("ESTADO:____DETECT_OBJECT..................")
            try:
                [obj, img] = JuskeshinoSimpleTasks.object_search(actual_obj, -1) #*************************************************
                """
                time.sleep(0.2)
                [obj, img] = JuskeshinoVision.detectAndRecognizeObjectWithoutOrientation(actual_obj)
                """   
                time.sleep(0.2)
                #print("SB-PLN.->Detected object : " ,obj.id , obj.category, obj.object_state, obj.pose.position)
                JuskeshinoHRI.say("I found" + actual_obj.replace("_", " ") + "of the category" + categorize_objs(obj.id))
                current_state = HANDLING_LOCATION

            except:
                JuskeshinoHRI.say("I couldn't find the object")
                tries = tries + 1
                current_state = DETECT_OBJECT
                if tries > 300:
                    JuskeshinoHRI.say("I didn't find the object")
                    print("SB-PLN.-> Se excedió el numero de intentos")
                    current_state = DETECT_OBJECT_ORIENTATION
 



        elif(current_state == HANDLING_LOCATION):
            print("ESTADO:____HANDLING_LOCATION..................")
            pos_obj_bl = [obj.pose.position.x, obj.pose.position.y, obj.pose.position.z]
            mov_flag, dist = JuskeshinoNavigation.getCloseSuitableGripPositionLa( pos_obj_bl, 100.0)
            JuskeshinoHardware.moveHead(0,-1, 5)
            time.sleep(0.2)

            tries = 0

            pos_obj_bl = [obj.pose.position.x, obj.pose.position.y, obj.pose.position.z]
            print("position obj:____", obj.pose.position.x)
            if(obj.pose.position.x > 0.52):
                move_front = obj.pose.position.x -0.54
                print("MOVE FRONT:__", move_front)
                JuskeshinoNavigation.moveDist(move_front , 5.0)


            current_state = DETECT_OBJECT_ORIENTATION#HANDLING_LOCATION_2#DETECT_OBJECT_ORIENTATION





        elif(current_state == DETECT_OBJECT_ORIENTATION):
            print("ESTADO:____DETECT_OBJECT_ORIENTATION..................")
            
            JuskeshinoHRI.say("Trying to detect the object")
            print("Trying to detect the object:__", actual_obj)

            time.sleep(0.2)
            try:
              
                [obj, img] = JuskeshinoSimpleTasks.object_search_orientation(actual_obj, -1)
                #[obj, img] = JuskeshinoVision.detectAndRecognizeObject(actual_obj)   
                #print("SB-PLN.->Detected object : " ,obj.id , obj.category, obj.object_state, obj.pose.position)
                JuskeshinoHRI.say("I found" + actual_obj.replace("_", " "))


                if(obj.pose.position.x > 0.65):
                    JuskeshinoHRI.say("I cannot take the object")
                    JuskeshinoNavigation.moveDist(-0.3,10)
                    current_state = CONFIG_BY_CYCLE
                else:
                    
                    current_state = PREPARE_ARM
            except:
                JuskeshinoHRI.say("I couldn't find the object")
                current_state = DETECT_OBJECT_ORIENTATION
                print("REGRESA A DETECTAR EL OBJETO DE NUEVO.................")
                """
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
                """
            
            #print("SB-PLN.->Detected object : " ,obj.id , obj.category, obj.object_state, obj.pose.position)
                      
            


            
        elif(current_state == PREPARE_ARM):
            print("ESTADO:___PREPARE_ARM..................")
            #if((obj.category == "CUBIC") or (obj.category == "BOWL") or (obj.category == "BOX") or (obj.object_state == "horizontal")):
            #JuskeshinoHardware.moveLeftArmWithTrajectory(PREPARE_TOP_GRIP, 10)  # prepare 
            JuskeshinoHardware.moveLeftArmWithTrajectory([-0.9, 0.3, 0.0 ,1.55, 0.0 , 1.34, 0.0], 10)
            if(obj.category == "BOWL"):
                JuskeshinoHRI.say("Open gripper")
                APERTURE = 0.6
                JuskeshinoHardware.moveLeftGripper(APERTURE , 5.0)
            else:
                APERTURE = 0.9
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
                if grip_attempts < 4:

                    current_state = DETECT_OBJECT_ORIENTATION

                else:
                    JuskeshinoHRI.say("No possible poses found")
                    print("SB-PLN.->No possible poses found")
                    #resp = resp.q
                
                    current_state = -1

                


        
        
        elif(current_state == TAKE_OBJECT):
            print("ESTADO:___TAKE_OBJECT..................")
            JuskeshinoHardware.moveLeftArmWithTrajectory(resp ,15)
            time.sleep(0.5)
            
            if (actual_obj == BOWL):
                JuskeshinoManipulation.dynamic_grasp_left_arm(is_thin = True)
            else:
                JuskeshinoManipulation.dynamic_grasp_left_arm()
            
            if(actual_obj != BOWL):
                """
                actual_position = rospy.wait_for_message("/hardware/left_arm/current_pose", Float64MultiArray, 5.0)
                actual_position_arm = list(actual_position.data)
                actual_position_arm[5] = actual_position_arm[5] + 1.57 
                JuskeshinoHardware.moveLeftArmWithTrajectory(actual_position_arm , 10) 
                """
            

            current_state = POST_GRASP





        elif(current_state == POST_GRASP):
            print("ESTADO:___POST_GRASP..................")
            print("SB-PLN.->Moving base backwards")    
            #JuskeshinoNavigation.moveDist(-0.33, 10)

            """
            if (not simu) or (torso):
                if(actual_obj == BOWL):
                    try:
                        JuskeshinoHardware.moveTorso( 0.24, 10.0)
                        #time.sleep(1)
                    except:
                        print("Cannot move torso")
            """
            JuskeshinoNavigation.moveDist(-0.33, 10)
            JuskeshinoHardware.moveLeftArmWithTrajectory(HOME , 10)
            """
            if(actual_obj != BOWL):
                JuskeshinoHardware.moveLeftArmWithTrajectory(PREPARE_TOP_GRIP, 10)
            """
            current_state = GO_TO_KITCHEN




        elif(current_state == GO_TO_KITCHEN):
            print("ESTADO:____GO_TO_KITCHEN..................")
            #JuskeshinoHRI.say("I'm going to " + EAT_TABLE)
            if not JuskeshinoNavigation.getClose(EAT_TABLE, 300):  
                print("SB-PLN.->Cannot get close to " + EAT_TABLE +" position")
                JuskeshinoHRI.say("Cannot get close to " + EAT_TABLE)
                
            current_state = PUT_OBJECT



    
        elif(current_state == PUT_OBJECT):
            print("ESTADO:____PUT_OBJECT...........................")
            if (actual_obj == MILK) or (actual_obj == CEREAL):
                #time.sleep(1)
                serving_breakfast(actual_obj) 
            if actual_obj == BOWL:
                JuskeshinoNavigation.moveDist(0.08, 7)
                #JuskeshinoNavigation.moveLateral(-0.05 , 5.0)
                JuskeshinoHRI.say("Leave bowl")
                JuskeshinoHardware.moveLeftArmWithTrajectory(LEAVE_BOWL, 10)
                print("SB-PLN.->Open gripper")
                JuskeshinoHardware.moveLeftGripper(1.0, 5.0)
                time.sleep(0.5)            # Soltar el objeto
            
            JuskeshinoHRI.say("cicle end")

            current_state = CYCLE_END



        

        elif(current_state == CYCLE_END):
            print("ESTADO:____CYCLE_END")
            JuskeshinoNavigation.moveDist(-0.55, 7)
            time.sleep(0.5)
            JuskeshinoHardware.moveLeftArmWithTrajectory(HOME , 10)
            time.sleep(0.2)
            JuskeshinoHardware.moveLeftGripper(0.0, 2.0)
            try:
                JuskeshinoHardware.moveTorso(0.05 , 10.0)
                #time.sleep(1)
            except:
                print("Cannot move torso")

            current_state = CONFIG_BY_CYCLE





        elif(current_state == CONFIG_BY_CYCLE):
            print("ESTADO:___CONFIG_BY_CYCLE")
            cycle = cycle + 1
            print("CYCLE:_____", cycle)
            if cycle > 2:
                #JuskeshinoHRI.say("Breakfast is served. I finish the test")
                current_state = END
            else:
                j = j + 1
                tries = 0
                grip_attempts = 0
                actual_obj = pila[j]
                location_actual = LOCATIONS[j]
                print("ACTUAL OBJECT:_____", actual_obj)

                current_state = BEGIN_CYCLE


        


        elif(current_state == END):     # El desayuno esta servido 
            print("ESTADO:____END")
            JuskeshinoHRI.say("Breakfast is served. I finish the test")
            current_state = -1


    
    
    return 
    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == "__main__":
    main()