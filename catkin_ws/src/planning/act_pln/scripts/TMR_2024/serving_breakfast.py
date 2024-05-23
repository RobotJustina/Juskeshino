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


# Left arm
HOME              = [0,0,0,0,0,0,0]
PREPARE           = [-0.69, 0.2, 0.0, 1.55, 0.0, 1.16, 0.0]
PREPARE_TOP_GRIP  = [-1.25, 0.3, 0, 2.4, 0, 0.7,0]
PREPARE_SERVING   = [0.91, 0.4, -0.5, 1.45, 0, 0.16, 0.5]
SERVING           = [0.91, 0.4, -0.5, 1.45, 0, 0.16, -1.6]
LEAVE_CEREAL      = [0.54, 0.28, -0.13, 1.45, 0, 0, 0]
LEAVE_MILK        = [0.44, 0.18, -0.03, 1.45, 0, 0, 0]
LEAVE_BOWL        = [0.6,  0.6, -0.8, 1.7, 0, 0.2, 0]

# RIght arm
PREPARE_RA    = [-0.8, -0.1, 0.0, 1.3, 1.3,0, 0.0]


MESA_INGREDIENTES = "desk_justina" 
MESA_COMER        = "kitchen"   #desk_takeshi" #"eat_table"

# Gripper_aperture
GRIP_MILK   = 0.3
GRIP_BOWL   = 0.0
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
        #time.sleep(0.2)
        JuskeshinoHardware.moveLeftGripper(0.7, 2.0)



def main():
    print("INITIALIZING SERVE BREAKFAST 2024 TEST BY ITZEL..............ヾ(๑╹◡╹)ﾉ")
    rospy.init_node("serve_breakfast_test")
    rate = rospy.Rate(10)

    # Se subcribe a los servicios necesarios para manipulacion, navegacion,vision, etc...
    JuskeshinoNavigation.setNodeHandle()
    JuskeshinoVision.setNodeHandle()
    JuskeshinoHardware.setNodeHandle()
    JuskeshinoSimpleTasks.setNodeHandle()
    JuskeshinoHRI.setNodeHandle()
    JuskeshinoManipulation.setNodeHandle()
    JuskeshinoKnowledge.setNodeHandle()

    
    JuskeshinoHRI.say("I'm ready for the test")
    # Esperar a que se abra la puerta
    JuskeshinoHRI.say("I'm waiting for the door to be open")
    print(("I'm waiting for door open"))
    
    if not JuskeshinoSimpleTasks.waitForTheDoorToBeOpen(300):
        print("ACT-PLN.->Door never opened")
        return
    
    pila = ["bowl", "milk", "cereal"] 

    #Inician acciones
    count = 0
    j = 0
    while count < 3: # Revisa pila
        print("OBJECT ACTUAL", pila[count])
        actual_obj = pila[count]
        # Ir a locacion de ingredientes
        
        JuskeshinoHRI.say("I'm going to the "+MESA_INGREDIENTES+" position.")
        print("I'm going to the "+MESA_INGREDIENTES+" position.")

        if not JuskeshinoNavigation.getClose(MESA_INGREDIENTES, 120):  #*******************
            print("SB-PLN.->Cannot get close to the "+MESA_INGREDIENTES+" position")
        time.sleep(1.3)

        # Apunta camara a la mesa
        print("SB-PLN.->move head")
        if not JuskeshinoHardware.moveHead(0,-1, 5):
            print("SB-PLN.->Cannot move head")
            time.sleep(1.0)
            JuskeshinoHardware.moveHead(0,-1, 5)
            time.sleep(1.0)
            JuskeshinoHardware.moveHead(0,-1, 5)
        JuskeshinoHRI.say("Align With the Table")
        time.sleep(1)
        # Alinearse con la mesa
        if not JuskeshinoSimpleTasks.alignWithTable():
            print("SB-PLN.->Cannot align with table")
            
        j=0 
        while j < 2:    
            # Busqueda y reconocimiento del objeto
            JuskeshinoHRI.say("Trying to detect the object")
            [obj, img] = JuskeshinoVision.detectAndRecognizeObjectWithoutOrientation(actual_obj)   #**************************
            if obj == None: 
                JuskeshinoHRI.say("I couldn't find the object")
            else:   # Objeto detectado..................................................................................
                print("SB-PLN.->Detected object : " + str([obj.id, obj.category, obj.object_state, obj.pose.position]))
                JuskeshinoHRI.say("I found" + actual_obj)

                
            # El robot se mueve a la mejor mejor ubicacion de agarre ,Toma objeto con brazo izquierdo
            mov = JuskeshinoSimpleTasks.handling_location(obj, "la")

            if (obj.pose.position.z > 0.72) and (actual_obj == "bowl"):
                try:
                    JuskeshinoHardware.moveTorso(np.linalg.norm(obj.pose.position.z - 0.70) , 5.0)
                    time.sleep(0.5)
                except:
                    print("Cannot move torso")

            # En la mejor ubicacion de agarre realiza de nuevo un reconocimiento del objeto***********************
            [obj, img] = JuskeshinoVision.detectAndRecognizeObject(actual_obj)   #**************************
            if obj == None: 
                JuskeshinoHRI.say("I couldn't find the object")

            print("SB-PLN.->Detected object : " + str([obj.id, obj.category, obj.object_state, obj.pose.position]))                        
            # Tomar objeto                                             
            print("SB-PLN.->Sending goal traj to prepare")

            JuskeshinoHardware.moveLeftArmWithTrajectory(PREPARE_TOP_GRIP, 10)  # prepare 
            time.sleep(0.2)
            print("SB-PLN.->Open gripper")
            if actual_obj == "bowl": JuskeshinoHardware.moveLeftGripper(0.4 , 3.0)
            else: JuskeshinoHardware.moveLeftGripper(0.9, 3.0)

            print("SB-PLN.-> Call Best Grasping Configuration Service")
            [resp, graspable] = JuskeshinoManipulation.GripLa(obj)
            if graspable:
                JuskeshinoHRI.say("graspable object")
                print("SB-PLN.->object position", obj.pose.position)
                JuskeshinoHardware.moveLeftArmWithTrajectory(resp.articular_trajectory,15)
                print("SB-PLN.->Closing gripper")
                if (actual_obj == "bowl"):
                    JuskeshinoHardware.moveLeftGripper(GRIP_BOWL , 3.0) 
                if (actual_obj == "milk"):
                    JuskeshinoHardware.moveLeftGripper(GRIP_MILK , 3.0) 
                if (actual_obj == "cereal"):
                    JuskeshinoHardware.moveLeftGripper(GRIP_CEREAL , 3.0) 

                time.sleep(0.5)
                print("ACT-PLN.->Moving arm to prepare***")
                JuskeshinoHardware.moveLeftArmWithTrajectory(PREPARE_TOP_GRIP, 10)  # prepare    
                break
            else:
                JuskeshinoHRI.say("No possible poses found")
                print("SB-PLN.->No possible poses found")

            j = j+1
        
        # Despues de que agarro el objeto va a la mesa del desayuno
        try:
            JuskeshinoHardware.moveTorso(0.02 , 5.0)
            time.sleep(1)
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
        print("SB-PLN.->Getting close to " + MESA_COMER + "location")
        JuskeshinoHRI.say("I'm going to the" + MESA_COMER + "location")
        if not JuskeshinoNavigation.getClose(MESA_COMER, 100):  
            print("SB-PLN.->Cannot get close to " + MESA_COMER +" position")

        time.sleep(1)
        JuskeshinoNavigation.moveDist(0.38, 5.0)
        JuskeshinoHRI.say("I have arrived at the" + MESA_COMER + " location")
        time.sleep(1)
        # Se alinea con la mesa
        print("SB-PLN.->move head")
        if not JuskeshinoHardware.moveHead(0,-1, 5):
            print("SB-PLN.->Cannot move head")
            time.sleep(0.5)
            JuskeshinoHardware.moveHead(0,-1, 5)
        time.sleep(1)
        #JuskeshinoHRI.say("Aligning with table")
        #print("SB-PLN.->Aligning with table")
        #JuskeshinoSimpleTasks.alignWithTable()
    
        # Coloca objeto sobre la mesa
        print("SB-PLN.->Moving left arm to deliver position")
        JuskeshinoHRI.say("Leave object")
        if (actual_obj == "milk") or (actual_obj == "cereal"):
            time.sleep(1)
            serving_breakfast(actual_obj) 
        if actual_obj == "bowl":
            JuskeshinoHRI.say("prepare")
            #JuskeshinoHardware.moveLeftArmWithTrajectory(PREPARE_TOP_GRIP , 12) 
            time.sleep(0.5)
            JuskeshinoHRI.say("bowl")
            JuskeshinoHardware.moveLeftArmWithTrajectory(LEAVE_BOWL , 12)
            time.sleep(1)
            # se mueve hacia delante 0.3 m**********************************pendiente
            print("SB-PLN.->Moving base backwards")
            JuskeshinoHRI.say("I'm going to place the object on the table")
            #JuskeshinoNavigation.moveDist(0.15, 7)
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

        count = count + 1

            
    # El desayuno esta servido 
    JuskeshinoHRI.say("Breakfast is served. I finish the test")
    
    return 
    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == "__main__":
    main()