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

PREPARE_RA_PRISM       = [-1, -0.2, 0.0, 1.3, 1,0, 0.0]
PREPARE_RA_CUB    = [-0.8, -0.1, 0.0, 1.3, 1.3,0, 0.0]



def main():
    print("INITIALIZING GRIP TEST BY ITZEL..............ヾ(๑╹◡╹)ﾉ")
    rospy.init_node("grip_test")
    rate = rospy.Rate(10)

    # Se subcribe a los servicios necesarios para manipulacion, navegacion,vision, etc...
    JuskeshinoNavigation.setNodeHandle()
    JuskeshinoVision.setNodeHandle()
    JuskeshinoHardware.setNodeHandle()
    JuskeshinoSimpleTasks.setNodeHandle()
    JuskeshinoHRI.setNodeHandle()
    JuskeshinoManipulation.setNodeHandle()
    JuskeshinoKnowledge.setNodeHandle()

    
    la = False
    actual_obj = "pringles" #objeto que se desa tomar

    print("Grip test.->move head")
    if not JuskeshinoHardware.moveHead(0,-1, 5):
        print("SB-PLN.->Cannot move head")
        time.sleep(1.0)
        JuskeshinoHardware.moveHead(0,-1, 5)
        time.sleep(1)
        if not JuskeshinoSimpleTasks.alignWithTable():
            print("Grip test.->Cannot align with table")

    """ 
    q_array = [0.4, 0 , -0.1 ,0.7, 0 , 0 , -0.1]
    print(q_array)
    fk = JuskeshinoManipulation.raFk(q_array)
    print("FK:________________________")
    print(fk)

    xyz_array = [fk.x, fk.y, fk.z, fk.roll, fk.pitch, fk.yaw]
    print("xyz_array____________")
    print(xyz_array)
    ik = JuskeshinoManipulation.raIk(xyz_array)
    print("IK:________________________")
    print(ik)
    """
    # Busqueda y reconocimiento del objeto
    print("SB-PLN.->Trying to detect object: ")
    JuskeshinoHRI.say("Trying to detect the object")

    j=0 # Intentos de agarre de objetos
    while j < 2:
        # En la mejor ubicacion de agarre realiza de nuevo un reconocimiento del objeto
        [obj, img] = JuskeshinoVision.detectAndRecognizeObject(actual_obj) #**************************
        print("SB-PLN.->Detected object : " + str([obj.id, obj.category, obj.object_state, obj.pose.position]))
                        
            # Tomar objeto                                              
        print("SB-PLN.->Sending goal traj to prepare")    
    
        
        if la:
            JuskeshinoHardware.moveLeftArmWithTrajectory(PREPARE_TOP_GRIP, 10)
        else:
            JuskeshinoHardware.moveRightArmWithTrajectory(PREPARE_RA_PRISM, 10)  # prepare 
        

        time.sleep(0.1)
        print("SB-PLN.->Open gripper")
        if la:
            JuskeshinoHardware.moveLeftGripper(0.9, 3.0)
        else:
            JuskeshinoHardware.moveRightGripper(0.9, 3.0)

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
                JuskeshinoHardware.moveLeftGripper(0, 2.0) 
            else:
                JuskeshinoHardware.moveRightArmWithTrajectory(resp.articular_trajectory,10)
                print("SB-PLN.->Closing gripper")
            time.sleep(0.5)
            print("ACT-PLN.->Moving right arm to prepare***")
            if la:
                JuskeshinoHardware.moveLeftArmWithTrajectory(PREPARE_TOP_GRIP, 10)  # prepare 
                break
            else:
                JuskeshinoHardware.moveRightArmWithTrajectory(PREPARE_RA_CUB, 10)  # prepare 
                break
            
        else:
            JuskeshinoHRI.say("No possible poses found")
            print("SB-PLN.->No possible poses found")
            j = j+1 # Actualiza intentos de agarre
        


    return 
    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == "__main__":
    main()


	
