#!/usr/bin/env python3
import rospy
import rospkg
import time
import numpy as np
from juskeshino_tools.JuskeshinoNavigation import JuskeshinoNavigation
from juskeshino_tools.JuskeshinoVision import JuskeshinoVision
from juskeshino_tools.JuskeshinoHardware import JuskeshinoHardware
from juskeshino_tools.JuskeshinoSimpleTasks import JuskeshinoSimpleTasks
from juskeshino_tools.JuskeshinoHRI import JuskeshinoHRI
from juskeshino_tools.JuskeshinoManipulation import JuskeshinoManipulation
from juskeshino_tools.JuskeshinoKnowledge import JuskeshinoKnowledge

HOME              = [0,0,0,0,0,0,0]
PREPARE           = [-0.69, 0.2, 0.0, 1.55, 0.0, 1.16, 0.0]
PREPARE_LATERAL_GRIP = [-0.69, 0.2, 0, 1.55, 0, 1.16,0] #[-1.2, 0.2, 0  , 1.6, 0   , 1,     0] #Prepare original:funciona bien para pringles vertical (prisma vertical) 

PREPARE_TEST      = [-1.25, 0.3, 0, 2.4, 0, 0.7,0]
PREPARE_TOP_GRIP  = [-1.25, 0.3, 0, 2.4, 0, 0.7,0]
PREPARE_SERVING   = [0.91, 0.4, -0.5, 1.45, 0, 0.16, 0.5]
SERVING           = [0.91, 0.4, -0.5, 1.45, 0, 0.16, -1.6]
LEAVE_CEREAL      = [0.54, 0.28, -0.13, 1.45, 0, 0, 0]
LEAVE_MILK        = [0.44, 0.18, -0.03, 1.45, 0, 0, 0]
LEAVE_BOWL        = [0.6,  0.6, -0.8, 1.7, 0, 0.2, 0]
LEAVE_NEAR_CEREAL = [0.39, 0.18, -0.03, 1.45, 0, 0, 0]
EMPTYING_POSE     = [0.37, 0.57, -0.11, 1.68, -0.73, 0.76, -0.90]
LEAVE_NEAR_MILK   = [0.39, 0.18, -0.01, 1.42, 0, 0.37, 0]

GRIPER_MILK   = 0.0
GRIPER_CEREAL = 0.0
GRIPER_BOWL   = 0.0

MESA_INGREDIENTES ="ingredients_table" 
MESA_COMER        = "eat_table"

def serving_breakfast(object):
    print("PREPARE TOP")
    JuskeshinoHardware.moveLeftArmWithTrajectory(PREPARE_TOP_GRIP, 10)
    time.sleep(0.5)
    JuskeshinoHardware.moveLeftArmWithTrajectory(PREPARE_SERVING, 10)
    time.sleep(0.5)
    #JuskeshinoNavigation.moveDist(0.3, 7)      # mueve la base adelante con el brazo levantado y extendido
    if object =="milk": JuskeshinoHRI.say("Serving milk")
    else: JuskeshinoHRI.say("Serving cereal")
    JuskeshinoHardware.moveLeftArmWithTrajectory(SERVING, 10)
    time.sleep(2)
    JuskeshinoHardware.moveLeftArmWithTrajectory(PREPARE_SERVING, 10)
    time.sleep(0.5)
    if object =="milk":
        JuskeshinoHardware.moveLeftArmWithTrajectory(LEAVE_MILK, 10)
        time.sleep(0.5)
        #JuskeshinoNavigation.moveDist(0.3, 7)
        JuskeshinoHardware.moveLeftGripper(0.7, 2.0)
        
    else:
        JuskeshinoNavigation.moveLateral(0.22, 10)
        JuskeshinoHardware.moveLeftArmWithTrajectory(LEAVE_CEREAL, 10)
        time.sleep(0.5)
        #JuskeshinoNavigation.moveDist(0.3, 7)
        #time.sleep(0.2)
        JuskeshinoHardware.moveLeftGripper(0.7, 2.0)



def main():
    print("INITIALIZING SERVE BREAKFAST 2024 TEST BY ITZEL..............ヾ(๑╹◡╹)ﾉ")
    rospy.init_node("serve_breakfast_test")
    rate = rospy.Rate(10)

    rospack = rospkg.RosPack()
    locations_default = rospack.get_path("config_files") + "/known_locations_objects.yaml"

    print(locations_default)
    #locations_default = rospack.get_path("config_files") + "/known_locations_simul.yaml"
    locations_file = rospy.get_param("~locations", locations_default)

    # Se subcribe a los servicios necesarios para manipulacion, navegacion,vision, etc...
    JuskeshinoNavigation.setNodeHandle()
    JuskeshinoVision.setNodeHandle()
    JuskeshinoHardware.setNodeHandle()
    JuskeshinoSimpleTasks.setNodeHandle()
    JuskeshinoHRI.setNodeHandle()
    JuskeshinoManipulation.setNodeHandle()
    JuskeshinoKnowledge.setNodeHandle()
    JuskeshinoKnowledge.loadLocations(locations_file)

    
    #JuskeshinoHRI.say("I'm ready for the serving breakfast test")
    # Esperar a que se abra la puerta
    JuskeshinoHRI.say("I'm waiting for door open")
    print(("I'm waiting for door open"))
    
    #if not JuskeshinoSimpleTasks.waitForTheDoorToBeOpen(300):
    #    print("ACT-PLN.->Door never opened")
    #    return
    JuskeshinoHRI.say("door open")
    # Ir a la cocina
    
    
    
    pila = ["bowl", "cereal", "milk"] 
    count = 0
    while count < 3: # Revisa pila
    
        print("OBJECT", pila[count])
        actual_obj = pila[count]
        # Ir a locacion de ingredientes
        JuskeshinoHRI.say("I'm going to kitchen")
        print("I'm going to the "+MESA_INGREDIENTES+" position.")

        
        #if not JuskeshinoNavigation.getClose(MESA_INGREDIENTES, 120):  #*******************
        #    print("SB-PLN.->Cannot get close to the "+MESA_INGREDIENTES+" position")
        
            
        time.sleep(1.3)
        # Alinearse con mueble
        print("SB-PLN.->move head")
        if not JuskeshinoHardware.moveHead(0,-1, 5):
            print("SB-PLN.->Cannot move head")
            time.sleep(1.0)
            JuskeshinoHardware.moveHead(0,-1, 5)
            time.sleep(1.0)
            JuskeshinoHardware.moveHead(0,-1, 5)
            
        
        JuskeshinoHardware.moveHead(0,-1, 5)


        time.sleep(1)
        if not JuskeshinoSimpleTasks.alignWithTable():
            print("SB-PLN.->Cannot align with table")
            

        # Busqueda y reconocimiento del objeto

        print("SB-PLN.->Trying to detect object: ")
        JuskeshinoHRI.say("I'm trying to detect the object")
            
        [obj, img] = JuskeshinoSimpleTasks.object_search(actual_obj)   #**************************
        if obj == None: 
            print("SB-PLN.->Object  no found...........")
            JuskeshinoHRI.say("I couldn't find the object")
            

        else:   # Objeto detectado.............................................
            print("SB-PLN.->Detected object : " + str([obj.id, obj.category, obj.object_state, obj.pose.position]))
            JuskeshinoHRI.say("I found" + actual_obj)
            print("SB-PLN.->handling location ")
            [dist, mov] = JuskeshinoSimpleTasks.handling_location(obj)

        j=0
        while j < 3:
            [obj, img] = JuskeshinoVision.detectAndRecognizeObject(actual_obj) #**************************
            #print("SB-PLN.->Detected object : " + str([obj.id, obj.category, obj.object_state, obj.pose.position]))
                        

            # Tomar objeto                                              
            print("SB-PLN.->Sending goal traj to prepare")
            print("PREPARE HIGHT")
            """
            if actual_obj=='cereal':
                JuskeshinoHardware.moveLeftArmWithTrajectory(PREPARE_LATERAL_GRIP, 10)  # prepare 
            else:    
                JuskeshinoHardware.moveLeftArmWithTrajectory(PREPARE_TOP_GRIP, 10)  # prepare 
            """
            JuskeshinoHardware.moveLeftArmWithTrajectory(PREPARE_TOP_GRIP, 10)  # prepare
            time.sleep(0.1)
            print("SB-PLN.->Open gripper")
            if actual_obj == "bowl": JuskeshinoHardware.moveLeftGripper(0.4 , 3.0)
            else: JuskeshinoHardware.moveLeftGripper(0.9, 3.0)
                
            print("SB-PLN.-> Call Best Grasping Configuration Service")
                
            [resp, graspable] = JuskeshinoManipulation.planBestGraspingConfiguration(obj)
            if graspable:
                JuskeshinoHRI.say("take object")
                print("SB-PLN.->object position", obj.pose.position)
                print("SB-PLN.->Sending best gripper configuration")
                time.sleep(1)
                
                JuskeshinoHardware.moveLeftArmWithTrajectory(resp.articular_trajectory,10)
                print("SB-PLN.->Closing gripper")
                
                if actual_obj == "bowl":
                    JuskeshinoHardware.moveLeftGripper(GRIPER_BOWL, 2.0) 
                if actual_obj == "cereal":JuskeshinoHardware.moveLeftGripper(GRIPER_CEREAL, 2.0) 
                if actual_obj == "milk":JuskeshinoHardware.moveLeftGripper(GRIPER_MILK, 2.0)
                
                time.sleep(0.5)
                print("ACT-PLN.->Moving arm to prepare***")
                if actual_obj=='cereal':
                    JuskeshinoHardware.moveLeftArmWithTrajectory(PREPARE_LATERAL_GRIP, 10)  # prepare 
                else:
                    JuskeshinoHardware.moveLeftArmWithTrajectory(PREPARE_TOP_GRIP, 10)  # prepare 
                   
                break
            else:
                JuskeshinoHRI.say("No possible poses found")
                print("SB-PLN.->No possible poses found")
                j = j+1

                time.sleep(1)
        
        JuskeshinoHardware.moveLeftGripper(GRIPER_BOWL, 2.0)
          
        print("SB-PLN.->Moving base backwards")
        #JuskeshinoHRI.say("I'have grasped the object")
        JuskeshinoNavigation.moveDist(-0.3, 10)
        time.sleep(0.5)
        print("ACT-PLN.->Moving arm to prepare***")
        JuskeshinoHardware.moveLeftArmWithTrajectory(PREPARE, 10)  # prepare
    
        actual_obj = pila[count]
        count = count + 1
        print("Actual object: ", actual_obj)

        #.........................................................................................................................
        # Ir a la mesa del desayuno...............................................................................................
        #.........................................................................................................................
        
        
        print("SB-PLN.->Getting close to " + MESA_COMER + "location")
        JuskeshinoHRI.say("I'm going to the location")

        if not JuskeshinoNavigation.getClose(MESA_COMER, 100):  #**********************************
        #if not JuskeshinoNavigation.getClose("cupboard", 100):
            print("SB-PLN.->Cannot get close to " + MESA_COMER +" position")
    
        JuskeshinoHRI.say("I have arrived at the" + MESA_COMER + " location")
        time.sleep(1)
        print("SB-PLN.->move head")
        if not JuskeshinoHardware.moveHead(0,-1, 5):
            print("SB-PLN.->Cannot move head")
            time.sleep(0.5)
            JuskeshinoHardware.moveHead(0,-1, 5)
        
        time.sleep(1)
        # Se alinea con la mesa
        JuskeshinoHRI.say("I'M GOING TO LINE UP WITH THE TABLE")
        print("SB-PLN.->Aligning with table")
        JuskeshinoSimpleTasks.alignWithTable()
    


        # lleva  brazo a posicion por encima de la mesa
        print("SB-PLN.->Moving left arm to deliver position")
        JuskeshinoHRI.say("Leave object")

        if (actual_obj == "milk") or (actual_obj == "cereal"):
            time.sleep(1)
            print("SB-PLN.->Moving base backwards")
            #JuskeshinoNavigation.moveDist(-0.3, 10)    #..........se hecha para atras para extender brazo
            serving_breakfast(actual_obj) 

        if actual_obj == "bowl":
            JuskeshinoHRI.say("prepare")
            JuskeshinoHardware.moveLeftArmWithTrajectory(PREPARE_TOP_GRIP , 12) 
            time.sleep(0.5)
            JuskeshinoHRI.say("bowl")
            JuskeshinoHardware.moveLeftArmWithTrajectory(LEAVE_BOWL , 12)
            time.sleep(1)
            # se mueve hacia delante 0.3 m
            print("SB-PLN.->Moving base backwards")
            JuskeshinoHRI.say("I'm going to place the object on the table")
            #JuskeshinoNavigation.moveDist(0.3, 7)
            # Soltar el objeto
            time.sleep(0.5)
            print("SB-PLN.->Open gripper")
            JuskeshinoHardware.moveLeftGripper(0.7, 2.0)
            time.sleep(1)            # Soltar el objeto
            

        # Moverse para atras
        print("SB-PLN.->Moving base backwards")
        JuskeshinoNavigation.moveDist(-0.3, 7)
        time.sleep(0.5)
        print("SB-PLN.->Moving left arm to prepare")
        JuskeshinoHardware.moveLeftArmWithTrajectory(PREPARE , 10)
        time.sleep(0.5)
        JuskeshinoHardware.moveLeftArmWithTrajectory(HOME , 10)
        
        
    # El desayuno esta servido 
    JuskeshinoHRI.say("Breakfast is served. I finish the test")
    


    return 
    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == "__main__":
    main()


	
