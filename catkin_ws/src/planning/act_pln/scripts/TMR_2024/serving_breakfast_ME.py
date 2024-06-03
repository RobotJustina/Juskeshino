#!/usr/bin/env python3
import rospy
import rospkg
import time
import numpy as np
import math
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

POST_GRIP         = [-1.25, 0.3, 0, 2.4, 0, 0.7,0]

# RIght arm
PREPARE_RA    = [-0.8, -0.1, 0.0, 1.3, 1.3,0, 0.0]


OBJECTS_TABLE = "desk_justina"
EAT_TABLE     = "desk_takeshi" 

OBJECTS_TABLE_X_Y_THETA = [5.44 ,2.15, 1.5]

# Gripper_aperture
GRIP_MILK   = 0.3
GRIP_BOWL   = -0.1
GRIP_CEREAL = 0.0


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



def location_obj(table_loc):
    objs, img = JuskeshinoVision.detectAndRecognizeObjects()
    loc_objs = list([])
        
    for obj in objs:
        if (obj.id == "bowl"):
            JuskeshinoHRI.say("I found: " + obj.id )
            print(obj.pose.position)
            x_bowl, y_bowl, z_bowl = obj.pose.position.x, obj.pose.position.y, obj.pose.position.z
        if (obj.id == "milk"):
            JuskeshinoHRI.say("I found: " + obj.id )
            print(obj.pose.position)
            x_milk, y_milk, z_milk = obj.pose.position.x, obj.pose.position.y, obj.pose.position.z
        if (obj.id == "cereal"):
            JuskeshinoHRI.say("I found: " + obj.id )
            print(obj.pose.position)
            x_cereal, y_cereal, z_cereal = obj.pose.position.x, obj.pose.position.y , obj.pose.position.z

        if objs == None:
            print('Cannot detect objects, I will try again...')
            JuskeshinoHRI.say('Cannot detect objects, I will try again...')
            return None
        if len(loc_objs) > 3:
            break

    xb, yb, zb = points_actual_to_points_target( [x_bowl, y_bowl, z_bowl] , 'base_link' , 'map' )
    xm, ym, zm = points_actual_to_points_target( [x_milk, y_milk, z_milk] , 'base_link' , 'map' )
    xc, yc, zc = points_actual_to_points_target( [ x_cereal, y_cereal, z_cereal] , 'base_link' , 'map' )
    
    loc_bowl = [x_bowl, y_bowl, theta]
    loc_milk = [x_bowl, y_bowl, theta]
    loc_bowl = [x_bowl, y_bowl, theta]





def main():
    rospy.init_node("serve_breakfast_test")
    rate = rospy.Rate(10)

    current_state = INITIAL

    while not rospy.is_shutdown():

        if(current_state == INITIAL):
            print("ESTADO:___INITIAL.................")
            print("INITIALIZING SERVE BREAKFAST 2024 TEST BY ITZEL..............ヾ(๑╹◡╹)ﾉ")
            
            current_state = CONFIG

        elif(current_state == CONFIG):
            # Se subcribe a los servicios necesarios para manipulacion, navegacion,vision, etc...
            JuskeshinoManipulation.setNodeHandle()
            JuskeshinoNavigation.setNodeHandle()
            JuskeshinoVision.setNodeHandle()
            JuskeshinoHardware.setNodeHandle()
            JuskeshinoSimpleTasks.setNodeHandle()
            JuskeshinoHRI.setNodeHandle()
            JuskeshinoKnowledge.setNodeHandle()
            pila = ["bowl", "milk", "small_cereal"]
            count = 0
            j = 0
            actual_obj = pila[j]
            first_detection = False
            second_detection = False
            tries = 0
            grip_attempts = 0
            cycle = 0

            current_state = START



        elif(current_state == CONFIG_BY_CYCLE):
            current_state = -1



        elif(current_state == START):
            print("ESTADO:___CONFIG..................")
            JuskeshinoHRI.say("I'm ready for the test")
            JuskeshinoHRI.say("I'm waiting for the door to be open")
            if not JuskeshinoSimpleTasks.waitForTheDoorToBeOpen(10):
                print("SERVING BREAKFAST-> Puerta cerrada")


                current_state = START
            else: 
                print("SERVING BREAKFAST-> Puerta abierta")
                JuskeshinoHRI.say("SERVING BREAKFAST->I can see now that the door is open")
                location_actual = OBJECTS_TABLE

                current_state = MOVE_TO_LOCATION





        elif(current_state == MOVE_TO_LOCATION):
            print("ESTADO:___MOVE_TO_LOCATION..................")
            JuskeshinoHRI.say("I'm going to the "+ location_actual)
            
            if not JuskeshinoNavigation.getClose(location_actual , 120): 
                JuskeshinoHRI.say("SB-PLN.->Cannot get close to the "+ location_actual +" position")
                tries = tries + 1
                print("Intentos:___", tries)
                current_state = MOVE_TO_LOCATION

            else:
                tries = 0
                print("SERVING BREAKFAST-> Se llego con exito a la locacion solicitada")
                current_state = MOVE_HEAD
            time.sleep(0.5)




     
        elif(current_state == MOVE_HEAD):
            print("ESTADO:___MOVE_HEAD..................")
            tries = 0
            try:
                JuskeshinoHardware.moveHead(0,-1, 5)
                current_state = APROACH_TO_TABLE
            except:
                JuskeshinoHRI.say("Cannot move head")
                tries = tries + 1
                current_state = MOVE_HEAD
            if tries > 3:
                JuskeshinoHRI.say("Cannot move head")
                current_state = APROACH_TO_TABLE
            time.sleep(0.3)





        elif(current_state == APROACH_TO_TABLE):
            print("ESTADO:___APROACH_TO_TABLE..................")
            approach_to_table()
            time.sleep(0.3)
            current_state = DETECT_OBJECT
            




        elif(current_state == DETECT_OBJECT):
            print("ESTADO:___DETECT_OBJECT..................")
            tries = 0
            JuskeshinoHRI.say("Trying to detect the object")
            try:
                [obj, img] = JuskeshinoVision.detectAndRecognizeObjectWithoutOrientation(actual_obj)   
                print("SB-PLN.->Detected object : " ,obj.id , obj.category, obj.object_state, obj.pose.position)
                JuskeshinoHRI.say("I found" + actual_obj)
                first_detection = True
                if (second_detection == False) and (first_detection == False):
                    current_state = HANDLING_LOCATION
                else:
                    current_state = PREPARE_ARM
            except:
                JuskeshinoHRI.say("I couldn't find the object")
                tries = tries + 1
                current_state = DETECT_OBJECT
                if tries > 3:
                    JuskeshinoHRI.say("Cannot move head")
                    print("SB-PLN.-> Se excedió el numero de intentos")
                    current_state = HANDLING_LOCATION

                



        elif(current_state == HANDLING_LOCATION):
            print("ESTADO:___HANDLING_LOCATION..................")
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
            first_detection = True
            current_state = DETECT_OBJECT
                      
            


            
        elif(current_state == PREPARE_ARM):
            print("ESTADO:___PREPARE_ARM..................")
            if (actual_obj == "bowl") or (actual_obj == "small_cereal"): 
                JuskeshinoHardware.moveLeftArmWithTrajectory(PREPARE_TOP_GRIP, 10)  # prepare 
                JuskeshinoHRI.say("Open gripper")
                JuskeshinoHardware.moveLeftGripper(0.4 , 2.0)
            else: 
                JuskeshinoHardware.moveLeftArmWithTrajectory(PREPARE, 10)  # prepare 
                JuskeshinoHRI.say("Open gripper")
                JuskeshinoHardware.moveLeftGripper(0.9, 1.0)
            current_state = LOOKING_GRIP





        elif(current_state == LOOKING_GRIP):
            print("ESTADO:___LOOKING_GRIP..................")
            JuskeshinoHRI.say("looking for proper grip")
            first_detection = False
            second_detection = False
            [resp, graspable] = JuskeshinoManipulation.GripLa(obj)
            if graspable:
                JuskeshinoHRI.say("graspable")
                print("SB-PLN.->object position", obj.pose.position)
                current_state = TAKE_OBJECT
            else:
                grip_attempts = grip_attempts + 1
                if grip_attempts < 3:
                    current_state = DETECT_OBJECT
                else:
                    JuskeshinoHRI.say("No possible poses found")
                    print("SB-PLN.->No possible poses found")
                    current_state = -1

        
        
        
        elif(current_state == TAKE_OBJECT):
            print("ESTADO:___TAKE_OBJECT..................")
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
            current_state = POST_GRASP





        elif(current_state == POST_GRASP):
            print("ESTADO:___POST_GRASP..................")
            print("ACT-PLN.->Moving arm to prepare***")
            JuskeshinoHardware.moveLeftArmWithTrajectory(POST_GRIP, 10)  # prepare    
            time.sleep(0.5)
            #JuskeshinoHardware.moveLeftArmWithTrajectory(PREPARE_TOP_GRIP, 10)
            try:
                JuskeshinoHardware.moveTorso(0.02 , 5.0)
                time.sleep(1.3)
            except:
                    print("Cannot move torso")
            print("SB-PLN.->Moving base backwards")    
            JuskeshinoNavigation.moveDist(-0.5, 10)
            time.sleep(0.5)
            JuskeshinoHardware.moveLeftArmWithTrajectory(HOME, 10)  # prepare    
            time.sleep(0.5)





        elif(current_state == GO_TO_KITCHEN):
            print("ESTADO:____GO_TO_KITCHEN..................")
            tries = 0
            JuskeshinoHRI.say("I'm going to the" + EAT_TABLE + "location")
            if not JuskeshinoNavigation.getClose(EAT_TABLE, 100):  
                print("SB-PLN.->Cannot get close to " + EAT_TABLE +" position")
                tries = tries + 1
                current_state = GO_TO_KITCHEN

            else:
                JuskeshinoHRI.say("I have arrived at the" + EAT_TABLE + " location")
                time.sleep(0.5)
                current_state = PUT_OBJECT





        elif(current_state == APROACH_TO_TABLE_2):
            print("ESTADO:____APROACH_TO_TABLE_2..................")
            print("SB-PLN.->move head")
            if not JuskeshinoHardware.moveHead(0,-1, 5):
                print("SB-PLN.->Cannot move head")
                time.sleep(0.5)
                JuskeshinoHardware.moveHead(0,-1, 5)
            time.sleep(1)
            approach_to_table()
            time.sleep(0.3)
            current_state = PUT_OBJECT



    
        elif(current_state == PUT_OBJECT):
            print("ESTADO:____PUT_OBJECT...........................")
            if (actual_obj == "milk") or (actual_obj == "small_cereal"):
                time.sleep(1)
                serving_breakfast(actual_obj) 
            if actual_obj == "bowl":
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
            cycle = cycle + 1
            if cycle > 2:
                current_state = END
            else:
                current_state = CONFIG_BY_CYCLE


        


        elif(current_state == END):     # El desayuno esta servido 
            print("ESTADO:____END")
            JuskeshinoHRI.say("Breakfast is served. I finish the test")
            current_state = -1


    
    
    return 
    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == "__main__":
    main()