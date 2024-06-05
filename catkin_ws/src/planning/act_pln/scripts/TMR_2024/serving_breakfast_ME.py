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
LEAVE_BOWL        = [0.6,  0.6, -0.8, 1.7, 0, 0.2, 0]

POST_GRIP         = [0.38, 0.19, -0.01, 1.57, 0 , 0.25, 0.0 ]

# RIght arm
PREPARE_RA    = [-0.8, -0.1, 0.0, 1.3, 1.3,0, 0.0]

# Locations
OBJECTS_TABLE = "desk_justina"
EAT_TABLE     = "desk_takeshi" 
OBJECTS_TABLE_X_Y_THETA = [5.44 ,2.15, 1.5]

# Locations_simu
OBJECTS_TABLE_SIMU = "desk_justina"
EAT_TABLE_SIMU     = "desk_takeshi" 
OBJECTS_TABLE_X_Y_THETA_SIMU = [5.44 ,2.15, 1.5]


# Objects
BOWL   = "red_bowl"
MILK   = "yogurt"
CEREAL = "mango_juice"


# Gripper_aperture
GRIP_MILK   = 0.4
GRIP_BOWL   = 0.1
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
DETECT_OBJECT_ORIENTATION = 18
BEGIN_CYCLE = 19
MOVE_TO_LOCATION_OBJECTS = 20




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




def location_obj():
    global listener
    table_loc = OBJECTS_TABLE_X_Y_THETA
    print("In function")
    detect = 0
    dist_obj = list([])
    JuskeshinoHardware.moveTorso(0.02 , 5.0)

    count = 0
    while(count < 13):
        #time.sleep(0.2)
        try:
            objs, img = JuskeshinoVision.detectAndRecognizeObjects()
            print("(*B*)")
            if len(objs) < 3:
                print("len menor a 3")
                count = count + 1
                continue
            else:
                print("len(objs):___", len(objs))
                break
        except:
            print("I cannot find the objects")
            JuskeshinoHRI.say("I cannot find the objects")
            count = count + 1

    print("tamano lista objectos:___", len(objs))
    x_bowl, y_bowl, z_bowl = 0,0,0
    x_milk, y_milk, z_milk = 0,0,0
    x_cereal, y_cereal, z_cereal = 0,0,0
    detected = 0

    if len(objs) < 3:
        print('Cannot detect objects, I will try again...')
        JuskeshinoHRI.say('Cannot detect objects, I will try again...')
        return [],[],[]
        
    for obj in objs:
        print("Objeto:___", obj.id)
        print("Position:___", obj.pose.position.x, obj.pose.position.y, obj.pose.position.z)
        if (obj.id == BOWL ):
            #JuskeshinoHRI.say("I found: " + obj.id )
            #print("Objeto:___", obj.id)
            #print(obj.pose.position)
            x_bowl, y_bowl, z_bowl = obj.pose.position.x, obj.pose.position.y, obj.pose.position.z
            #print("x_bowl, y_bowl, z_bowl",x_bowl, y_bowl, z_bowl)
            detected = detect + 1
        if (obj.id == MILK ):
            #JuskeshinoHRI.say("I found: " + obj.id )
            #print(obj.pose.position)
            x_milk, y_milk, z_milk = obj.pose.position.x, obj.pose.position.y, obj.pose.position.z
            detected = detect + 1
        if (obj.id == CEREAL ):
            #JuskeshinoHRI.say("I found: " + obj.id )
            #print(obj.pose.position)
            x_cereal, y_cereal, z_cereal = obj.pose.position.x, obj.pose.position.y , obj.pose.position.z
            detected = detect + 1
    
    xb, yb, zb = points_actual_to_points_target( [x_bowl, y_bowl, z_bowl] , 'base_link' , 'map' )
    print("new point bowl", xb, yb, zb)
    xm, ym, zm = points_actual_to_points_target( [x_milk, y_milk, z_milk] , 'base_link' , 'map' )
    print("new point milk", xm, ym, zm)
    xc, yc, zc = points_actual_to_points_target( [x_cereal, y_cereal, z_cereal] , 'base_link' , 'map' )
    print("new point cereal", xc, yc, zc)
    loc_bowl = [xb , table_loc[1], table_loc[2]]
    loc_milk = [xm , table_loc[1], table_loc[2]]
    loc_cereal = [xc , table_loc[1], table_loc[2]]
    print("LOC BOWL:______", loc_bowl)
    print("LOC MILK:______", loc_milk)
    print("LOC CEREAL:____", loc_cereal)
    print("LOC TABLE INGREDIENTS:___", OBJECTS_TABLE_X_Y_THETA)

    return loc_bowl, loc_milk, loc_cereal




def main():
    rospy.init_node("serve_breakfast_test")
    rate = rospy.Rate(10)
    global listener, simu
    listener = tf.TransformListener()
    simu = False

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
            pila = [BOWL, MILK, CEREAL]
            count = 0
            j = 0
            actual_obj = pila[j]
            first_detection = False
            second_detection = False
            loc_objects = False
            tries = 0
            grip_attempts = 0
            cycle = 0
            loc_bowl, loc_milk, loc_cereal = [],[],[]
            flag_loc_objs = True
            print("cycle:___", cycle)
            current_state = START




        elif(current_state == START):
            print("ESTADO:___CONFIG..................")
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

            current_state = MOVE_TO_LOCATION

            """
            if (len(loc_bowl) < 1 and (len(loc_milk) < 1) and (len(loc_cereal) < 1)):
                current_state = MOVE_TO_LOCATION
            else:
                current_state = MOVE_TO_LOCATION_OBJECTS
            """





        elif(current_state == MOVE_TO_LOCATION):
            print("ESTADO:___MOVE_TO_LOCATION..................")
            #JuskeshinoHRI.say("I'm going to the "+ location_actual)
            
            if not JuskeshinoNavigation.getClose(location_actual , 300): 
                JuskeshinoHRI.say("SB-PLN.->Cannot get close to the "+ location_actual +" position")

                current_state =  MOVE_HEAD

            else:
                tries = 0
                print("SERVING BREAKFAST-> Se llego con exito a la locacion solicitada")
                current_state = MOVE_HEAD
            time.sleep(0.5)





     
        elif(current_state == MOVE_HEAD):
            print("ESTADO:___MOVE_HEAD..................")
            try:
                JuskeshinoHardware.moveHead(0,-1, 5)
                current_state = APROACH_TO_TABLE
            except:
                JuskeshinoHRI.say("Cannot move head")
                JuskeshinoHardware.moveHead(0,-1, 5)
                current_state = APROACH_TO_TABLE
                #tries = tries + 1
                #current_state = MOVE_HEAD

            """   
            if tries > 3:
                JuskeshinoHRI.say("Cannot move head")
                current_state = APROACH_TO_TABLE
            time.sleep(0.3)
            """




        elif(current_state == APROACH_TO_TABLE):
            print("ESTADO:___APROACH_TO_TABLE..................")
            approach_to_table()
            time.sleep(0.3)
            tries = 0
            
            current_state = DETECT_OBJECT
            




        elif(current_state == DETECT_OBJECT):
            print("ESTADO:___DETECT_OBJECT..................")
        
            """
            JuskeshinoHRI.say("Trying to detect the object")
            if (cycle < 1) and (flag_loc_objs == True):
                loc_bowl, loc_milk, loc_cereal = location_obj()
                if ((len(loc_bowl) > 0) and (len(loc_milk) > 0) and (len(loc_cereal) > 0)):
                    JuskeshinoNavigation.moveDist(-0.33, 10)
                    current_state = MOVE_TO_LOCATION_OBJECTS
                else:
                    flag_loc_objs = False
                    current_state = DETECT_OBJECT
            """
            try:
                time.sleep(0.3)
                [obj, img] = JuskeshinoVision.detectAndRecognizeObjectWithoutOrientation(actual_obj)   
                time.sleep(0.2)
                print("SB-PLN.->Detected object : " ,obj.id , obj.category, obj.object_state, obj.pose.position)
                    #JuskeshinoHRI.say("I found" + actual_obj)
                current_state = HANDLING_LOCATION

            except:
                JuskeshinoHRI.say("I couldn't find the object")
                tries = tries + 1
                current_state = DETECT_OBJECT
                if tries > 3:
                    JuskeshinoHRI.say("Cannot move head")
                    print("SB-PLN.-> Se excedió el numero de intentos")
                    current_state = HANDLING_LOCATION
            #cycle = cycle + 1

                



        elif(current_state == HANDLING_LOCATION):
            print("ESTADO:___HANDLING_LOCATION..................")
            mov = JuskeshinoSimpleTasks.handling_location_la(obj.pose.position)
            JuskeshinoHRI.say("Aligne with table")
            JuskeshinoSimpleTasks.alignWithTable()
            
            # Ajusta altura de torso para mejor agarre
            if (actual_obj == BOWL):
                JuskeshinoHardware.moveTorso(0.07 , 5.0)

            if (actual_obj == MILK):
                    JuskeshinoHardware.moveTorso(0.07 , 5.0) 
            if (actual_obj == CEREAL):
                JuskeshinoHardware.moveTorso(0.08 , 5.0)

            tries = 0
            current_state = DETECT_OBJECT_ORIENTATION
            




        elif(current_state == DETECT_OBJECT_ORIENTATION):
            print("ESTADO:___DETECT_OBJECT_ORIENTATION..................")
            
            JuskeshinoHRI.say("Trying to detect the object")
            print("Trying to detect the object:__", actual_obj)

            """
            obj = detect_obj_function(actual_obj)

            if obj == None:
                current_state = DETECT_OBJECT_ORIENTATION
            else:
                print("SB-PLN.->Detected object : " ,obj.id , obj.category, obj.object_state, obj.pose.position)
                JuskeshinoHRI.say("I found" + actual_obj)
                current_state = PREPARE_ARM
            """

            time.sleep(0.2)
            try:
                [obj, img] = JuskeshinoVision.detectAndRecognizeObject(actual_obj)   
                print("SB-PLN.->Detected object : " ,obj.id , obj.category, obj.object_state, obj.pose.position)
                JuskeshinoHRI.say("I found" + actual_obj)
                current_state = PREPARE_ARM

            except:
                JuskeshinoHRI.say("I couldn't find the object")
                print("tries", tries)
                tries = tries + 1
                current_state = DETECT_OBJECT_ORIENTATION
                if tries > 2:
                    JuskeshinoHRI.say("Cannot move head")
                    print("SB-PLN.-> Se excedió el numero de intentos")
                    current_state = PREPARE_ARM
            #print("SB-PLN.->Detected object : " ,obj.id , obj.category, obj.object_state, obj.pose.position)
                      
            


            
        elif(current_state == PREPARE_ARM):
            print("ESTADO:___PREPARE_ARM..................")
            if (actual_obj == BOWL) or (actual_obj == CEREAL): 
                JuskeshinoHardware.moveLeftArmWithTrajectory(PREPARE_TOP_GRIP, 10)  # prepare 
                JuskeshinoHRI.say("Open gripper")
                JuskeshinoHardware.moveLeftGripper(0.4 , 2.0)
            else: 
                JuskeshinoHardware.moveLeftArmWithTrajectory(PREPARE, 10)  # prepare 
                JuskeshinoHRI.say("Open gripper")
                JuskeshinoHardware.moveLeftGripper(1.0, 1.0)
            current_state = LOOKING_GRIP





        elif(current_state == LOOKING_GRIP):
            print("ESTADO:___LOOKING_GRIP..................")
            JuskeshinoHRI.say("looking for proper grip")

            [resp, graspable] = JuskeshinoManipulation.GripLa(obj)
            if graspable:
                JuskeshinoHRI.say("graspable")
                print("SB-PLN.->object position", obj.pose.position)
                resp = resp.articular_trajectory
                current_state = TAKE_OBJECT
            else:
                resp = resp.q
                current_state = TAKE_OBJECT
                """
                grip_attempts = grip_attempts + 1
                if grip_attempts < 3:
                    current_state = DETECT_OBJECT
                else:0
                    JuskeshinoHRI.say("No possible poses found")
                    print("SB-PLN.->No possible poses found")
                    current_state = -1
                """

        

        
        
        elif(current_state == TAKE_OBJECT):
            print("ESTADO:___TAKE_OBJECT..................")
            JuskeshinoHardware.moveLeftArmWithTrajectory(resp ,15)
            time.sleep(0.5)
            JuskeshinoHRI.say("Closing gripper")
                
            if (actual_obj == BOWL):
                JuskeshinoHardware.moveLeftGripper(0.3 , 3.0)
                JuskeshinoHardware.moveLeftGripper(0.2 , 3.0)
                JuskeshinoHardware.moveLeftGripper(0.1 , 3.0)
                JuskeshinoHardware.moveLeftGripper(0.0 , 3.0)
                """
                if(JuskeshinoHardware.moveLeftGripper(GRIP_BOWL , 3.0) ):
                    JuskeshinoHRI.say("gripper close")
                else: 
                    JuskeshinoHRI.say("Cannot close gripper")
                    JuskeshinoHardware.moveLeftGripper(0.1 , 1.0)
                """

            if (actual_obj == MILK):
                JuskeshinoHardware.moveLeftGripper(0.7 , 3.0)
                JuskeshinoHardware.moveLeftGripper(0.6 , 3.0)
                JuskeshinoHardware.moveLeftGripper(0.5 , 3.0)
                JuskeshinoHardware.moveLeftGripper(0.4 , 3.0)
            if (actual_obj == CEREAL):
                JuskeshinoHardware.moveLeftGripper(0.7 , 3.0)
                JuskeshinoHardware.moveLeftGripper(0.5 , 3.0)
                JuskeshinoHardware.moveLeftGripper(0.3 , 3.0)
                JuskeshinoHardware.moveLeftGripper(0.1 , 3.0)

            current_state = POST_GRASP





        elif(current_state == POST_GRASP):
            print("ESTADO:___POST_GRASP..................")
            print("ACT-PLN.->Moving arm to prepare***")
            time.sleep(0.6)
            JuskeshinoHardware.moveTorso(0.13 , 5.0)
            #JuskeshinoHardware.moveLeftArmWithTrajectory(POST_GRIP, 10)  # prepare    
            time.sleep(0.3)
            JuskeshinoHRI.say("lift object")
            JuskeshinoHardware.moveLeftArmWithTrajectory(PREPARE_TOP_GRIP, 10)
            try:
                JuskeshinoHardware.moveTorso(0.02 , 5.0)
                time.sleep(1)
            except:
                print("Cannot move torso")

            print("SB-PLN.->Moving base backwards")    
            JuskeshinoNavigation.moveDist(-0.33, 10)
            tries = 0  
            time.sleep(0.3)

            current_state = GO_TO_KITCHEN





        elif(current_state == GO_TO_KITCHEN):
            print("ESTADO:____GO_TO_KITCHEN..................")
            JuskeshinoHRI.say("I'm going to the" + EAT_TABLE + "location")
            if not JuskeshinoNavigation.getClose(EAT_TABLE, 100):  
                print("SB-PLN.->Cannot get close to " + EAT_TABLE +" position")
                #tries = tries + 1
                #current_state = GO_TO_KITCHEN
            else:
                JuskeshinoHRI.say("I have arrived at the" + EAT_TABLE + " location")
                time.sleep(0.5)
                current_state = APROACH_TO_TABLE_2





        elif(current_state == APROACH_TO_TABLE_2):
            print("ESTADO:____APROACH_TO_TABLE_2..................")
            print("SB-PLN.->move head")
            if not JuskeshinoHardware.moveHead(0,-1, 5):
                sprint("SB-PLN.->Cannot move head")
                time.sleep(0.5)
                JuskeshinoHardware.moveHead(0,-1, 5)
            time.sleep(1)
            approach_to_table()
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

            current_state = CONFIG_BY_CYCLE





        elif(current_state == CONFIG_BY_CYCLE):
            print("ESTADO:___CONFIG_BY_CYCLE")
            cycle = cycle + 1
            print("CYCLE:_____", cycle)
            if cycle > 2:
                current_state = END
            else:
                j = j + 1
                first_detection = False
                second_detection = False
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