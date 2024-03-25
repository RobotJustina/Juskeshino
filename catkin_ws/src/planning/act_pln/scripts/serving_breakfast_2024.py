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


PREPARE           = [-0.69, 0.2, 0.0, 1.55, 0.0, 1.16, 0.0]
LEAVE_CEREAL      = [0.2, 0.18, -0.03, 1.45, 0, 0, 0]
LEAVE_MILK        = [0.2, -0.12, -0.03, 1.45, 0, 0, 0]
LEAVE_BOWL        = [0.2, -0.6, -0.03, 1.45, 0, 0, 0]
LEAVE_NEAR_CEREAL = [0.39, 0.18, -0.03, 1.45, 0, 0, 0]
EMPTYING_POSE     = [0.37, 0.57, -0.11, 1.68, -0.73, 0.76, -0.90]
LEAVE_NEAR_MILK   = [0.39, 0.18, -0.01, 1.42, 0, 0.37, 0]


def search_bb_obj(objects, bb_target):
    i = 0
    for obj in objects:   # Para cada objeto de la lista de VisionObjects
        print("********************" + obj.category)
        if bb_target == obj.category:
            print("Se encontró el objeto pedido.................")
            return i
        i = i + 1
    return None




def main():
    print("INITIALIZING SERVE BREAKFAST 2024 TEST BY IBY..............ヾ(๑╹◡╹)ﾉ")
    rospy.init_node("serve_breakfast_test")
    rate = rospy.Rate(10)

    rospack = rospkg.RosPack()
    locations_default = rospack.get_path("config_files") + "/known_locations_objects.yaml"
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


    # Esperar a que se abra la puerta
    """
    JuskeshinoHRI.say("I'm waiting for the door to be open")
    
    if not JuskeshinoSimpleTasks.waitForTheDoorToBeOpen(300):
        print("ACT-PLN.->Door never opened")
        return
    JuskeshinoHRI.say("I can see now that the door is open")
    # Ir a la cocina
    JuskeshinoHRI.say("I'm going to the start position.")

    if not JuskeshinoNavigation.getClose("entrance", 30):
        print("ACT-PLN.->Cannot get close to start position")

    JuskeshinoHRI.say("I arrived to the entrance")
    

    # Llenar pila con objetos: cereal, leche, tazon, cuchara
    # pila = ["bowl", "cereal", "cereal"]
    
    count = 0
    while count < 3: # Revisa pila

        # Ir a locacion de ultimo objeto en la pila
        JuskeshinoHRI.say("I'm going to the kitchen position.")

        if not JuskeshinoNavigation.getClose("kitchen", 100):
            print("ACT-PLN.->Cannot get close to the kitchen position")

        # Alinearse con mueble
        if not JuskeshinoHardware.moveHead(0,-1, 5):
            print("ACT-PLN.->Cannot move head")
        if not JuskeshinoSimpleTasks.alignWithTable():
            print("ACT-PLN.->Cannot align with table")

        # Busqueda y reconocimiento del objeto
        print("ACT-PLN.->Trying to detect object: ")
        JuskeshinoHRI.say("I'm trying to detect the object")
            
        [obj, img] = JuskeshinoSimpleTasks.object_search("milk")
        if obj == None: 
            print("Object not found...........")

        else:
            print("ACT-PLN.->Detected object : " + str([obj.id, obj.category, obj.object_state, obj.pose.position]))
            JuskeshinoHRI.say("I found the object, I'm going to try to grasp them")
            # Colocandose en posicion de agarre
            JuskeshinoSimpleTasks.handling_location(obj)
            time.sleep(2)
            [obj, img] = JuskeshinoSimpleTasks.object_search("milk")
            print("POSITION OBJECT......................", obj.pose.position)
        
            # Tomar objeto
            print("ACT-PLN.->Sending goal traj to prepare")
            JuskeshinoHardware.moveLeftArmWithTrajectory([-0.66, 0.36, -0.04, 1.79, 0, 1.02, 0], 10)
            print("ACT-PLN.->Open gripper")
            JuskeshinoHardware.moveLeftGripper(0.7, 2.0)

            resp = JuskeshinoManipulation.planBestGraspingConfiguration(obj)
            print("object position", obj.pose.position)
            print("ACT-PLN.->Sending best gripper configuration")
            JuskeshinoHardware.moveLeftArmWithTrajectory(resp.articular_trajectory,10)

            print("ACT-PLN.->Closing gripper first attempt")
            JuskeshinoHardware.moveLeftGripper(-0.5, 2.0)
            print("ACT-PLN.->Closing gripper second attempt")
            JuskeshinoHardware.moveLeftGripper(-0.5, 2.0)
            print("ACT-PLN.->Closing gripper third attempt")               
            JuskeshinoHardware.moveLeftGripper(-0.5, 2.0)                  
    
            #print("ACT-PLN.->Moving arm to post grasping position")
            #JuskeshinoHardware.moveLeftArmWithTrajectory([0.46, 0.87, -0.4, 1.99, -0.99, 0.4,  1.6], 10)   # Objeto recogido
    
            print("ACT-PLN.->Moving base backwards")
            JuskeshinoHRI.say("I'have grasped the object")
            JuskeshinoNavigation.moveDist(-0.3, 10)

            print("ACT-PLN.->Moving arm to prepare")
            JuskeshinoHardware.moveLeftArmWithTrajectory([-0.69, 0.2, 0, 1.55, 0, 1.16, 0], 10)  # prepare

        """
    
    # Ir a la mesa
    actual_obj = "BOX" 

    print("ACT-PLN.->Getting close to breakfast table")
    JuskeshinoHRI.say("I'm going to the breakfast table")

    if not JuskeshinoNavigation.getClose("breakfast_table", 100):
        print("ACT-PLN.->Cannot get close to desk position")
    
    if not JuskeshinoHardware.moveHead(0,-1, 5):
        print("ACT-PLN.->Cannot move head")

    print("ACT-PLN.->Aligning with table")
    JuskeshinoSimpleTasks.alignWithTable()
    
    # Colocar el objeto sobre la mesa
    print("ACT-PLN.->Moving base backwards")
    JuskeshinoHRI.say("I'have grasped the object")
    JuskeshinoNavigation.moveDist(-0.3, 10)

    print("ACT-PLN.->Moving left arm to deliver position")
    JuskeshinoHRI.say("I'm going to leave the object")
    if actual_obj == "BOX":
        JuskeshinoHardware.moveLeftArmWithTrajectory(LEAVE_CEREAL , 10)

    if actual_obj == "PRISM":
        JuskeshinoHardware.moveLeftArmWithTrajectory(LEAVE_MILK , 10)  

    if actual_obj == "BOWL":
        JuskeshinoHardware.moveLeftArmWithTrajectory(LEAVE_BOWL , 10) 

    print("ACT-PLN.->Open gripper")
    JuskeshinoHardware.moveLeftGripper(0.7, 2.0)

    print("ACT-PLN.->Moving base backwards")
    JuskeshinoHRI.say("I'have grasped the object")
    JuskeshinoNavigation.moveDist(0.3, 7)

    # Moverse para atras
    print("ACT-PLN.->Moving base backwards")
    JuskeshinoNavigation.moveDist(-0.3, 7)
    time.sleep(1)
    print("ACT-PLN.->Moving left arm to prepare")
    JuskeshinoHardware.moveLeftArmWithTrajectory(PREPARE , 10)
    
    """
    # Ir a la mesa
    print("ACT-PLN.->Getting close to breakfast table")
    JuskeshinoHRI.say("I'm going to the breakfast table")

    if not JuskeshinoNavigation.getClose("breakfast_table", 100):
        print("ACT-PLN.->Cannot get close to desk position")
    
    if not JuskeshinoHardware.moveHead(0,-1, 5):
        print("ACT-PLN.->Cannot move head")
    print("ACT-PLN.->Aligning with table")
    JuskeshinoSimpleTasks.alignWithTable()

    # Busqueda y reconocimiento de cereal********************************
    [objects, image] = JuskeshinoVision.detectAndRecognizeObjects()
    index_objs = search_bb_obj(objects, "BOX")
    print("INDICE DESEADO", index_objs)

    # Tomar objeto
    resp = JuskeshinoManipulation.planBestGraspingConfiguration(objects[index_objs])
    print("ACT-PLN.->Sending best gripper configuration")
    JuskeshinoHardware.moveLeftArmWithTrajectory(resp.articular_trajectory,10)
    
    # Llevar a la pose de vaciado por 5 s (0.08 m por encima del tazon)
    JuskeshinoHardware.moveLeftArmWithTrajectory(EMPTYING_POSE, 10)

    # Colocar de nuevo en la mesa cuidadosamente (a la derecha del tazon)
    print("ACT-PLN.->Moving left arm to deliver position")
    JuskeshinoHRI.say("I'm going to leave the object")
    JuskeshinoHardware.moveLeftArmWithTrajectory(LEAVE_NEAR_CEREAL , 10)

    # Busqueda y reconocimiento de leche*********************************
    [objects, image] = JuskeshinoVision.detectAndRecognizeObjects()
    index_objs = search_bb_obj(objects, "PRISM")
    print("INDICE DESEADO", index_objs)

    # Tomar objeto
    resp = JuskeshinoManipulation.planBestGraspingConfiguration(objects[index_objs])
    print("ACT-PLN.->Sending best gripper configuration")
    JuskeshinoHardware.moveLeftArmWithTrajectory(resp.articular_trajectory,10)
    
    # Llevar a la pose de vaciado por 5 s (0.08 m por encima del tazon)
    JuskeshinoHardware.moveLeftArmWithTrajectory(EMPTYING_POSE, 10)

    # Colocar de nuevo en la mesa cuidadosamente (a la derecha del tazon)
    print("ACT-PLN.->Moving left arm to deliver position")
    JuskeshinoHRI.say("I'm going to leave the object")
    JuskeshinoHardware.moveLeftArmWithTrajectory(LEAVE_NEAR_MILK , 10)

    # El desayuno esta servido 
    JuskeshinoHRI.say("I finished the test")
    """


    return 
    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == "__main__":
    main()
