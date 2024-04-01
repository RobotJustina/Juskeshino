#!/usr/bin/env python3
import rospy
import time
import numpy as np

from juskeshino_tools.JuskeshinoNavigation import JuskeshinoNavigation
from juskeshino_tools.JuskeshinoVision import JuskeshinoVision
from juskeshino_tools.JuskeshinoHardware import JuskeshinoHardware
from juskeshino_tools.JuskeshinoSimpleTasks import JuskeshinoSimpleTasks
from juskeshino_tools.JuskeshinoHRI import JuskeshinoHRI
from juskeshino_tools.JuskeshinoManipulation import JuskeshinoManipulation
from juskeshino_tools.JuskeshinoKnowledge import JuskeshinoKnowledge

HOME        = [0 ,0 ,0 ,0 ,0 ,0 ,0 ]
PREPARE     = [-0.69, 0.2, 0.0, 1.55, 0.0, 1.16, 0.0]
TAKE_OBJECT = [0.26, 0.22, -0.56, 1.2, 0, 0.44, 0.3 ]


def main():
    print("INITIALIZING CARRY MY LUGGAGE 2024 TEST NODE BY ITZEL.......ᐠ( ᐢ ᵕ ᐢ )ᐟ")
    rospy.init_node("carry_my_luggage_test")
    rate = rospy.Rate(10)

    # Se subcribe a los servicios y topicos necesarios para manipulacion, navegacion,vision, etc...
    JuskeshinoNavigation.setNodeHandle()
    JuskeshinoVision.setNodeHandle()
    JuskeshinoHardware.setNodeHandle()
    JuskeshinoSimpleTasks.setNodeHandle()
    JuskeshinoHRI.setNodeHandle()
    JuskeshinoManipulation.setNodeHandle()
    JuskeshinoKnowledge.setNodeHandle()

    
    # Inicio de la prueba
    print("CML-PLN.-> Carry_my_luggage_test 2024")
    JuskeshinoHRI.say("I am ready for the carry my luggage test")
    time.sleep(1)
    # Pedir al operador que apunte a la bolsa
    voice = ""
    JuskeshinoHRI.say("Please point at the bag that you want me to carry.")
    while not rospy.is_shutdown():
        JuskeshinoHRI.say("Tell me, Justina yes, when you are pointing at the bag")
        time.sleep(1)
        voice = JuskeshinoHRI.waitForNewSentence(10)
        print("voice: ", voice)
        if "YES" in voice: break

    #_Detectar persona apuntando bolsa
    print("CML-PLN.-> Find person, enableHumanPose was enabled")
    JuskeshinoVision.enableHumanPose(True)
    time.sleep(2)
    print("CML-PLN.->Pointing hand")
    p_h = JuskeshinoVision.pointingHand()
    p_h_side = "left"
    print("CML-PLN.-> enableHumanPose was disabled")
    JuskeshinoVision.enableHumanPose(True)
    # Detectar en que lado del operador esta la bolsa
    if p_h:
        if p_h == "left":
            print("CML-PLN.-> left hand points to the bag")
            JuskeshinoHRI.say("left hand points to the bag")
            # Acercarse a la bolsa y extender brazo
            JuskeshinoNavigation.moveDistAngle(0.1 , np.radians(-45) , 5)
            JuskeshinoHardware.moveLeftArmWithTrajectory(TAKE_OBJECT , 10)  

        else:
            print("CML-PLN.-> right hand points to the bag")
            JuskeshinoHRI.say("right hand points to the bag")
            # Acercarse a la bolsa y extender brazo
            JuskeshinoNavigation.moveDistAngle(0.1 , np.radians(-45) , 5)
            JuskeshinoHardware.moveLeftArmWithTrajectory(TAKE_OBJECT , 10) 
        
    # Pedir al operador la bolsa
    JuskeshinoHRI.say("Please put the bag on the lefth gripper")
    time.sleep(1)
    JuskeshinoHardware.moveLeftArmWithTrajectory(TAKE_OBJECT , 10)  
    time.sleep(2)
    JuskeshinoHardware.moveLeftArmWithTrajectory(PREPARE , 10) 
    # Dar indicaciones al operador
    while not rospy.is_shutdown():
        JuskeshinoHRI.say("Tell me, Justina yes, when you put the bag")
        time.sleep(1)
        voice = JuskeshinoHRI.waitForNewSentence(10)
        print("voice: ", voice)
        if "YES" in voice: break
    
	# buscar pies del operador
    human_detector = False
    while not rospy.is_shutdown():
        JuskeshinoHRI.say("please stand in front of me")
        print("CML-PLN.-> Find person, enableHumanPose was enabled")
        JuskeshinoVision.enableHumanPose(True)
        time.sleep(3)
        print("CML-PLN.-> calling human detector")
        human_detector = JuskeshinoVision.humanDetector()
        print("CML-PLN.-> human detector is :__", human_detector)
        if human_detector: break
        else: JuskeshinoHRI.say("I can't found you, please stand in front of me")

    print("CML-PLN.-> Find legs , enableLegFinder was enabled")
    JuskeshinoHRI.enableLegFinder(True)
    print("CML-PLN.-> find legs in front")
    
    while not rospy.is_shutdown():
        JuskeshinoHRI.say("Say follow me when you are ready")
        time.sleep(1)
        voice = JuskeshinoHRI.waitForNewSentence(10)
        print("voice: ", voice)
        if "FOLLOW" in voice: break
    voice = " "

    # Seguir al operador    
    JuskeshinoHRI.say("I'm going to follow you, please say here is the car if we reached the final location")

    while not rospy.is_shutdown():
        legs_found = JuskeshinoHRI.frontalLegsFound()
        print("CML-PLN.-> Legs in front?___", legs_found.data)
        time.sleep(4)

        if(legs_found):
            print("ACT-PLN.-> found legs")
            print("ACT-PLN.-> HumanFollower enable")
            JuskeshinoHRI.enableHumanFollower(True)
        else:
            print("ACT-PLN.-> Not found legs")
            JuskeshinoHRI.say("I can't found you, please stand in front of me")
            JuskeshinoHRI.enableHumanFollower(False)
        
        oice = JuskeshinoHRI.waitForNewSentence(10)
        print("voice: ", voice)
        if "CAR" in voice: break

    # Legar a la locacion objetivo
    print("ACT-PLN.-> HumanFollower disable")
    JuskeshinoHRI.enableHumanFollower(False)
    print("CML-PLN.-> Find legs , enableLegFinder was disabled")
    JuskeshinoHRI.enableLegFinder(False)
    
    # entregar bolsa
    JuskeshinoHRI.say("Please take the bag")
    time.sleep(1)
    JuskeshinoHRI.say("Tell me, Justina yes, when you take the bag")

    while not rospy.is_shutdown():
        voice = JuskeshinoHRI.waitForNewSentence(10)
        print("voice: ", voice)
        if "YES" in voice: break

    JuskeshinoHardware.moveLeftArmWithTrajectory(HOME , 10) 
	# Buscar la cola
    JuskeshinoNavigation.moveDistAngle(0 , np.radians(180) , 5)
    print("CML-PLN.-> Find person, enableHumanPose was enabled")
    JuskeshinoVision.enableHumanPose(True)
    time.sleep(2)
    print("CML-PLN.-> Find legs , enableLegFinder was enabled")
    JuskeshinoHRI.enableLegFinder(True)
    person = 0
    if person > 1:
        print("encontro la fila")
        # Acercarse a la ultima persona en la fila
        #usar Lidar
        
    # Seguir la cola
    legs_found = JuskeshinoHRI.frontalLegsFound()
    print("CML-PLN.-> Legs in front?___", legs_found.data)
    time.sleep(2)

    if(legs_found):
        print("ACT-PLN.-> found legs")
        print("ACT-PLN.-> HumanFollower enable")
        JuskeshinoHRI.enableHumanFollower(True)
        time.sleep(100)

	# Estado final
    print("ACT-PLN.-> HumanFollower disable")
    JuskeshinoHRI.enableHumanFollower(False)
    print("CML-PLN.-> Find legs , enableLegFinder was disabled")
    JuskeshinoHRI.enableLegFinder(False)
    JuskeshinoHRI.say("I finished the test")



    return 
    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == "__main__":
    main()