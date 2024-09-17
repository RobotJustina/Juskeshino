#!/usr/bin/env python3
import rospy
import rospkg
import time
import numpy as np
import math
import tf.transformations as tft
from std_msgs.msg import String
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


POST_GRIP         = [0.38, 0.19, -0.01, 1.57, 0 , 0.35, 0.0 ]

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


def callback_take_object(msg):
    global object_name
    print("MENSAJE PUBLICADO EN TOPICO /plannning/take_object....")
    object_name = msg.data


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



def main():
    print("INITIALIZING GRIP TEST BY ITZEL..............ヾ(๑╹◡╹)ﾉ")
    rospy.init_node("grip_test")
    rate = rospy.Rate(10)
    global object_name
    # Se subcribe a los servicios necesarios para manipulacion, navegacion,vision, etc...
    rospy.Subscriber('/plannning/simple_task/take_object' ,String ,callback_take_object )
    global listener
    listener = tf.TransformListener()

    while not rospy.is_shutdown():
        print("waiting for an object to be requested.............................ʕ•ᴥ•ʔ")
        object_name_msg = rospy.wait_for_message('/plannning/simple_task/take_object' , String)
        obj_target = object_name_msg.data
        print("OBJECT TARGET:____", obj_target)
        actual_obj = obj_target
        current_state = CONFIG

        while not rospy.is_shutdown():
            if(current_state == CONFIG):
                print("ESTADO:___CONFIG.................")
                # Se subcribe a los servicios necesarios para manipulacion, navegacion,vision, etc...
                JuskeshinoManipulation.setNodeHandle()
                JuskeshinoNavigation.setNodeHandle()
                JuskeshinoVision.setNodeHandle()
                JuskeshinoHardware.setNodeHandle()
                JuskeshinoSimpleTasks.setNodeHandle()
                JuskeshinoHRI.setNodeHandle()
                JuskeshinoKnowledge.setNodeHandle()
                j = 0
                tries = 0
                grip_attempts = 0
                current_state = MOVE_HEAD


        
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
                    tilt = -1.0
                    [obj, img] = JuskeshinoSimpleTasks.object_search(actual_obj, tilt) #************************
                    JuskeshinoHRI.say("I found" + actual_obj.replace("_", " ") + "of the category" + categorize_objs(obj.id))
                    current_state = HANDLING_LOCATION
                except:
                    JuskeshinoHRI.say("I couldn't find the object")
                    print("I couldn't find the object")
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
                if mov_flag:
                    JuskeshinoNavigation.moveDist(0.18, 7.0)
                tries = 0
                pos_obj_bl = [obj.pose.position.x, obj.pose.position.y, obj.pose.position.z]
                print("position obj:____", obj.pose.position.x)
                if(obj.pose.position.x > 0.48):
                    move_front = obj.pose.position.x -0.48
                    print("MOVE FRONT:__", move_front)
                    JuskeshinoNavigation.moveDist(move_front , 10.0)
                print("position obj:____", obj.pose.position.x)
                if(obj.pose.position.x < 0.42):
                    move_back =  0.42 - obj.pose.position.x
                    print("MOVE BACK:__", move_back)
                    JuskeshinoNavigation.moveDist(-move_back , 10.0)
                current_state = DETECT_OBJECT_ORIENTATION




            elif(current_state == DETECT_OBJECT_ORIENTATION):
                print("ESTADO:____DETECT_OBJECT_ORIENTATION..................")
                print("Trying to detect the object:__", actual_obj)
                time.sleep(0.2)
                try:
                    tilt = -1.0
                    [obj, img] = JuskeshinoSimpleTasks.object_search_orientation(actual_obj, tilt)
                    JuskeshinoHRI.say("I found" + actual_obj.replace("_", " "))
                    
                    if(obj.pose.position.x > 0.65):
                        JuskeshinoHRI.say("I cannot take the object")
                        JuskeshinoNavigation.moveDist(-0.3,10)
                        current_state = CONFIG_BY_CYCLE
                    else:
                        
                        current_state = PREPARE_ARM
                except:
                    JuskeshinoHRI.say("New attempt at object recognition")
                    tries = tries + 1
                    if tries > 8:
                        JuskeshinoHRI.say("I couldn't find the object, try again")
                        break
                    else: 
                        current_state = DETECT_OBJECT_ORIENTATION
                    print("REGRESA A DETECTAR EL OBJETO DE NUEVO.................")       
                


            elif(current_state == PREPARE_ARM):
                print("ESTADO:___PREPARE_ARM..................")
                JuskeshinoHardware.moveLeftArmWithTrajectory([-0.9, 0.3, 0.0 ,1.55, 0.0 , 1.34, 0.0], 10)
                print(obj.point_cloud)
                if(obj.category == "BOWL"):
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
                    print("SB-PLN.->object position", obj.pose.position)
                    resp = resp.articular_trajectory
                    current_state = TAKE_OBJECT
                else:
                    grip_attempts = grip_attempts + 1
                    if grip_attempts < 16:
                        current_state = DETECT_OBJECT_ORIENTATION
                    else:
                        JuskeshinoHRI.say("No possible poses found")
                        print("SB-PLN.->No possible poses found")
                        current_state = TAKE_OBJECT

                    

            elif(current_state == TAKE_OBJECT):
                print("ESTADO:___TAKE_OBJECT..................")
                JuskeshinoHardware.moveLeftArmWithTrajectory(resp ,15)
                time.sleep(0.5)
                if (actual_obj == "BOWL"):
                    JuskeshinoManipulation.dynamic_grasp_left_arm(is_thin = True)
                else:
                    JuskeshinoManipulation.dynamic_grasp_left_arm()
                actual_obj = None
                current_state = -1
                break
    return 

if __name__ == "__main__":
    main()