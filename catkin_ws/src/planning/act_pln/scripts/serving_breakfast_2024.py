#!/usr/bin/env python3
import rospy
import rospkg
import geometry_msgs
import vision_msgs
from juskeshino_tools.JuskeshinoNavigation import JuskeshinoNavigation
from juskeshino_tools.JuskeshinoVision import JuskeshinoVision
from juskeshino_tools.JuskeshinoHardware import JuskeshinoHardware
from juskeshino_tools.JuskeshinoSimpleTasks import JuskeshinoSimpleTasks
from juskeshino_tools.JuskeshinoHRI import JuskeshinoHRI
from juskeshino_tools.JuskeshinoManipulation import JuskeshinoManipulation
from juskeshino_tools.JuskeshinoKnowledge import JuskeshinoKnowledge

def handling_location(position_obj ):
    left_threshold_manipulation = 0.26

    if position_obj.y > 0.26:
        dist_move = position_obj.y - left_threshold_manipulation
        print("dist move izq", dist_move)
        JuskeshinoNavigation.moveLateral(dist_move, 7)

    if position_obj.y < 0.05:
        dist_move = position_obj.y - 0.05
        print("dist move der", dist_move)
        JuskeshinoNavigation.moveLateral(dist_move, 7)



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

    """
    # Esperar a que se abra la puerta
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
    pila = ["bowl", "cereal", "milk"]
    print("pila: ", pila)
    
    

    while len(pila) > 0: # Revisa pila
    
        i = len(pila)
        
        print("len pila while 1", i)
        indx_obj = i-1
        actual_obj = pila[indx_obj]
    
        print("actual_obj: ", actual_obj)
        # Ir a locacion de ultimo objeto en la pila
        JuskeshinoHRI.say("I'm going to the" + actual_obj + "position.")

        if not JuskeshinoNavigation.getClose("kitchen", 100):
            print("ACT-PLN.->Cannot get close to the desk position")

        # Alinearse con mueble
        if not JuskeshinoHardware.moveHead(0,-1, 5):
            print("ACT-PLN.->Cannot move head")
        if not JuskeshinoSimpleTasks.alignWithTable():
            print("ACT-PLN.->Cannot align with table")

        # Busqueda y reconocimiento del objeto
        while indx_obj >= 0:
            print("ACT-PLN.->Trying to detect object: ", actual_obj)
            JuskeshinoHRI.say("I'm trying to detect the " + actual_obj)
            
            [obj, img] = JuskeshinoVision.detectAndRecognizeObject(actual_obj)
            if obj != None: # si reconocio el objeto
                print("ACT-PLN.->Detected object: " + str([obj.id, obj.category, obj.object_state, obj.pose.position]))
                position_obj = obj.pose.position
                handling_location(position_obj )
                print("ACT-PLN.->Trying to detect object 2: ", actual_obj)
                [obj, img] = JuskeshinoVision.detectAndRecognizeObject(actual_obj)
                print("ACT-PLN.->Detected object 2: " + str([obj.id, obj.category, obj.object_state, obj.pose.position]))
                
                print("ACT-PLN.->Planning best gripper configuration")
                JuskeshinoHRI.say("I found the " + pila[-1] + ", I'm going to try to grasp them")
                pila.pop(indx_obj)
                print("pila actualizada: ", pila)
                break
            else:
                indx_obj = indx_obj -1
                actual_obj = pila[indx_obj]
                print("ELSE actual obj: ", actual_obj, "indx obj: ", indx_obj)

        # Tomar objeto

        print("ACT-PLN.->Sending goal traj to prepare")
        JuskeshinoHardware.moveLeftArmWithTrajectory([-0.66, 0.36, -0.04, 1.79, 0, 1.02, 0], 10)
        print("ACT-PLN.->Open gripper")
        JuskeshinoHardware.moveLeftGripper(0.7, 2.0)

        resp = JuskeshinoManipulation.planBestGraspingConfiguration(obj)
        print("ACT-PLN.->Sending best gripper configuration")
        JuskeshinoHardware.moveLeftArmWithTrajectory(resp.articular_trajectory,10)

        print("ACT-PLN.->Closing gripper first attempt")
        JuskeshinoHardware.moveLeftGripper(-0.3, 2.0)
        print("ACT-PLN.->Closing gripper second attempt")
        JuskeshinoHardware.moveLeftGripper(-0.3, 2.0)
        print("ACT-PLN.->Closing gripper third attempt")
        JuskeshinoHardware.moveLeftGripper(-0.3, 2.0)
        print("ACT-PLN.->Moving arm to post grasping position")
        JuskeshinoHardware.moveLeftArmWithTrajectory([0.46, 0.87, -0.4, 1.99, -0.99, 0.4,  1.6], 10)   # Objeto recogido
        print("ACT-PLN.->Mofing arm to prepare")
        JuskeshinoHardware.moveLeftArmWithTrajectory([-0.69, 0.2, 0, 1.55, 0, 1.16, 0], 10)  # prepare
 
        print("ACT-PLN.->Moving base backwards")
        JuskeshinoHRI.say("I'have grasped the" + pila[-1])
        JuskeshinoNavigation.moveDist(-0.3, 10)
        """
    
    # Ir a la mesa
    print("ACT-PLN.->Getting close to breakfast table")
    JuskeshinoHRI.say("I'm going to the breakfast table")
    JuskeshinoNavigation.getClose("breakfast_table", 40)
    
    
    print("ACT-PLN.->Aligning with table")
    JuskeshinoSimpleTasks.alignWithTable()
    
    # Colocar el objeto sobre la mesa
    JuskeshinoSimpleTasks.placing_object_on_table()

    """
        # Depositar objeto delicadamente
        print("ACT-PLN.->Moving left arm to deliver position")
        JuskeshinoHRI.say("I'm going to leave the pringles")
        JuskeshinoHardware.moveLeftArmWithTrajectory([0.46, 0.87, -0.4, 1.99, -0.99, 0.4,  1.6], 10)
        print("ACT-PLN.->Open gripper")
        JuskeshinoHardware.moveLeftGripper(0.7, 2.0)
        print("ACT-PLN.->Moving left arm to prepare")
        JuskeshinoHardware.moveLeftArmWithTrajectory([-0.69, 0.2, 0, 1.55, 0, 1.16, 0], 10)  # prepare
        JuskeshinoHardware.moveLeftArmWithTrajectory([ 0.00, 0.0, 0, 0.00, 0, 0.00, 0], 10)  # prepare

        # Moverse para atras
        print("ACT-PLN.->Moving base backwards")
        JuskeshinoNavigation.moveDist(-0.3, 10)


        # Actualiza pila
        print(pila.pop())
        print("pila: ", pila)
        


    

    # Cuando la pila esta vacia ir a la locacion de preparacion del desayuno
    print("Pila vacia")

    # Busqueda y reconocimiento de cereal

    # Tomar y llevar a la pose de vaciado por 5 s (0.08 m por encima del tazon)

    # Colocar de nuevo en la mesa cuidadosamente (a la derecha del tazon)

    # Busqueda y reconocimiento de leche

    # Tomar y llevar a la pose de vaciado por 5 s (0.08 m por encima del tazon)

    # Colocar de nuevo en la mesa cuidadosamente (a la izquierda del tazon)

    # El desayuno esta servido 

    
    
    print("ACT-PLN.->Going back to entrance")
    JuskeshinoHRI.say("I'm going back to the entrance")
    JuskeshinoNavigation.getClose("entrance", 60)
    JuskeshinoHRI.say("I'm ready for a new command")

    
    
    print("ACT-PLN.->Waiting for command")
    cmd = JuskeshinoSimpleTasks.waitForSentenceUntilConfirmed(10)
    print("ACT-PLN.->Trying to find a human and approach to him")
    JuskeshinoHRI.say("I'm trying to find william")
    JuskeshinoSimpleTasks.findHumanAndApproach(20)
    print("ACT-PLN.->Meeting william")
    JuskeshinoHRI.say("Are you William? Please answer hoostina yes or hoostina no")
    print("ACT-PLN.->Waiting for confirmation")
    JuskeshinoSimpleTasks.waitForConfirmation(10)
    print("ACT-PLN.->Starting following")
    JuskeshinoHRI.say("Ok. Hello william, I am going to follow you. Please stand in front of me")
    JuskeshinoHRI.enableHumanFollowing(True)
    JuskeshinoHRI.say("Please say stop following me, to stop following you")
    rospy.sleep(10)
    JuskeshinoHRI.enableHumanFollowing(False)
    print("ACT-PLN.->Stopping following")
    JuskeshinoHRI.say("Ok. I'm going to stop following you")
    print("ACT-PLN.->Going back to entrance")
    JuskeshinoHRI.say("I'm going back to the start position")
    JuskeshinoNavigation.getClose("entrance", 30)
    JuskeshinoHRI.say("I finished the test")
    """

    return 
    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == "__main__":
    main()
