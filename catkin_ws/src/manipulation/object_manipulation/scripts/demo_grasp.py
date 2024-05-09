#!/usr/bin/env python3
import rospy
import rospkg
import time
import numpy as np

from std_msgs.msg import String
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

PREPARE_RA_PRISM       = [-1, -0.2, 0.0, 1.3, 1,0, 0.0]
PREPARE_RA_CUB    = [-0.8, -0.1, 0.0, 1.3, 1.3,0, 0.0]



def callback_take_object(msg):
    global object_name
    print("MENSAJE PUBLICADO EN TOPICO /plannning/take_object....")
    object_name = msg.data
    print("objeto solicitado:_____:", object_name)



def main():
    print("INITIALIZING GRIP TEST BY ITZEL..............ヾ(๑╹◡╹)ﾉ")
    rospy.init_node("grip_test")
    rate = rospy.Rate(10)
    global object_name

    # Se subcribe a los servicios necesarios para manipulacion, navegacion,vision, etc...
    rospy.Subscriber('/plannning/simple_task/take_object' ,String ,callback_take_object )
    JuskeshinoNavigation.setNodeHandle()
    JuskeshinoVision.setNodeHandle()
    JuskeshinoHardware.setNodeHandle()
    JuskeshinoSimpleTasks.setNodeHandle()
    JuskeshinoHRI.setNodeHandle()
    JuskeshinoManipulation.setNodeHandle()
    JuskeshinoKnowledge.setNodeHandle()

    

    #actual_obj = "pringles" #objeto que se desa tomar

    while not rospy.is_shutdown():


        print("waiting for an object to be requested.............................ʕ•ᴥ•ʔ")
        object_name_msg = rospy.wait_for_message('/plannning/simple_task/take_object' , String)
        obj_target = object_name_msg.data
        print("OBJECT TARGET:____", obj_target)
        actual_obj = obj_target

        print("Grip test.->move head")
        if not JuskeshinoHardware.moveHead(0,-1, 5):
            print("SB-PLN.->Cannot move head")
            time.sleep(1.0)
            JuskeshinoHardware.moveHead(0,-1, 5)
            time.sleep(1)
            if not JuskeshinoSimpleTasks.alignWithTable():
                print("Grip test.->Cannot align with table")


        # Busqueda y reconocimiento del objeto
        print("SB-PLN.->Trying to detect object: ")
        JuskeshinoHRI.say("Trying to detect the object")

        j=0 # Intentos de agarre de objetos
        while j < 2:#realiza de nuevo un reconocimiento del objeto
            try:
                [obj, img] = JuskeshinoVision.detectAndRecognizeObject(actual_obj) #**************************
                print("SB-PLN.->Detected object : " + str([obj.id, obj.category, obj.object_state, obj.pose.position]))
                if obj == None: 
                    print("SB-PLN.->Object  no found...........")
                    JuskeshinoHRI.say("I couldn't find the object")
                    j = j + 1
                    break
            except:
                break

            if (obj.pose.position.y >= 0): 
                la = True
                print("the object is taken with the left arm")
            else: 
                la = False
                print("the object is taken with the right arm")
                            
            
            # El robot se mueve a la mejor mejor ubicacion de agarre
            [dist, mov] = JuskeshinoSimpleTasks.handling_location(obj)

            if mov:
                try:
                    [obj, img] = JuskeshinoVision.detectAndRecognizeObject(actual_obj) #**************************
                    print("SB-PLN.->Detected object : " + str([obj.id, obj.category, obj.object_state, obj.pose.position]))
                    if obj == None: 
                        print("SB-PLN.->Object  no found...........")
                        JuskeshinoHRI.say("I couldn't find the object")
                        j = j + 1
                        break
                except:
                    break


            # Tomar objeto                                              
            print("SB-PLN.->Sending goal traj to prepare")    
            if la:
                JuskeshinoHardware.moveLeftArmWithTrajectory(PREPARE_TOP_GRIP, 10)
            else:
                JuskeshinoHardware.moveRightArmWithTrajectory(PREPARE_RA_CUB, 10)  # prepare 
            

            time.sleep(0.2)
            print("SB-PLN.->Open gripper")
            if la:
                JuskeshinoHardware.moveLeftGripper(0.7, 3.0)
            else:
                JuskeshinoHardware.moveRightGripper(0.7, 3.0)

            print("SB-PLN.-> Call Best Grasping Configuration Service")
            if la:
                [resp, graspable] = JuskeshinoManipulation.GripLa(obj)
            else:
                [resp, graspable] = JuskeshinoManipulation.GripRa(obj)
            if graspable:
                JuskeshinoHRI.say("graspable object")
                print("SB-PLN.->object position", obj.pose.position)
                print("SB-PLN.->Sending best gripper configuration")
                time.sleep(0.5)
                if la:
                    JuskeshinoHardware.moveLeftArmWithTrajectory(resp.articular_trajectory,10)
                    print("SB-PLN.->Closing gripper")
                    #JuskeshinoHardware.moveLeftGripper(0, 2.0) 
                else:
                    JuskeshinoHardware.moveRightArmWithTrajectory(resp.articular_trajectory,10)
                    print("SB-PLN.->Closing gripper")
                    time.sleep(6)
                    JuskeshinoHardware.moveRightGripper(0, 2.0)
                time.sleep(0.5)
                print("ACT-PLN.->Moving right arm to prepare***")
                if la:
                    JuskeshinoHardware.moveLeftArmWithTrajectory(PREPARE_TOP_GRIP, 10)  # prepare 
                    break
                else:
                    JuskeshinoHardware.moveRightArmWithTrajectory(PREPARE_RA_PRISM, 10)  # prepare 
                    break
                
            else:
                JuskeshinoHRI.say("No possible poses found")
                print("SB-PLN.->No possible poses found")
                j = j+1 # Actualiza intentos de agarre
            

        rate.sleep()

    return 



if __name__ == "__main__":
    main()


	
