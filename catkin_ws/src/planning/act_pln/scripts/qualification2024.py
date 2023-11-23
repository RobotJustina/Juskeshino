#!/usr/bin/env python
import rospy
import geometry_msgs
import vision_msgs
from juskeshino_tools.JuskeshinoNavigation import JuskeshinoNavigation
from juskeshino_tools.JuskeshinoVision import JuskeshinoVision
from juskeshino_tools.JuskeshinoHardware import JuskeshinoHardware
from juskeshino_tools.JuskeshinoSimpleTasks import JuskeshinoSimpleTasks
from juskeshino_tools.JuskeshinoHRI import JuskeshinoHRI
from juskeshino_tools.JuskeshinoManipulation import JuskeshinoManipulation
from juskeshino_tools.JuskeshinoKnowledge import JuskeshinoKnowledge

def main():
    print("INITIALIZING QUALIFICATION 2024 TEST BY MARCOSOFT...")
    rospy.init_node("act_pln")
    rate = rospy.Rate(10)

    locations_file = rospy.get_param("~locations", "known_locations.yaml")

    JuskeshinoNavigation.setNodeHandle()
    JuskeshinoVision.setNodeHandle()
    JuskeshinoHardware.setNodeHandle()
    JuskeshinoSimpleTasks.setNodeHandle()
    JuskeshinoHRI.setNodeHandle()
    JuskeshinoManipulation.setNodeHandle()
    JuskeshinoKnowledge.setNodeHandle()
    JuskeshinoKnowledge.loadLocations(locations_file)
    """
    JuskeshinoHardware.moveLeftArmWithTrajectory([-1.2, 0.2, 0, 1.6, 0, 1.1, 0], 10)
    JuskeshinoHardware.moveLeftGripper(0.7, 1.0)
    JuskeshinoHardware.moveLeftGripper(-0.3, 1.0)
    JuskeshinoHardware.moveLeftGripper(0.7, 1.0)
    JuskeshinoHardware.moveLeftGripper(-0.3, 1.0)

    return
    """
    JuskeshinoHRI.say("I'm waiting for the door to be open")
    if not JuskeshinoSimpleTasks.waitForTheDoorToBeOpen(100):
        print("Door never opened")
        return

    JuskeshinoHRI.say("I'm goint to the start position.")
    if not JuskeshinoNavigation.getClose("entrance", 30):
        print("Cannot get close to start position")
    JuskeshinoHRI.say("I arrived to start position")

    print("Waiting for command")
    cmd = JuskeshinoSimpleTasks.waitForSentenceUntilConfirmed(10)
    print("Command confirmed: " + cmd)

    if not JuskeshinoNavigation.getClose("desk", 30):
        print("Cannot get close to goal position: desk")
    if not JuskeshinoHardware.moveHead(0,-1, 5):
        print("Cannot move head")
    if not JuskeshinoSimpleTasks.alignWithTable():
        print("Cannot align with table")

    
    print("Sending goal traj")
    JuskeshinoHardware.moveLeftArmWithTrajectory([-0.66, 0.36, -0.04, 1.79, 0, 1.02, 0], 10)
    JuskeshinoHardware.moveLeftGripper(0.7, 2.0)
    print("Goal traj reached")
    [obj, img] = JuskeshinoVision.detectAndRecognizeObject("pringles")
    resp = JuskeshinoManipulation.planBestGraspingConfiguration(obj)
    JuskeshinoHardware.moveLeftArmWithTrajectory(resp.articular_trajectory,10)
    JuskeshinoHardware.moveLeftGripper(-0.3, 2.0)
    JuskeshinoHardware.moveLeftGripper(-0.3, 2.0)
    JuskeshinoHardware.moveLeftGripper(-0.3, 2.0)   
    JuskeshinoHardware.moveLeftArmWithTrajectory([0.46, 0.87, -0.4, 1.99, -0.99, 0.4,  1.6], 10)   # Objeto recogido
    JuskeshinoHardware.moveLeftArmWithTrajectory([-0.69, 0.2, 0, 1.55, 0, 1.16, 0], 10)  # prepare
    JuskeshinoNavigation.moveDist(-0.3, 10)
    JuskeshinoNavigation.getClose("cupboard", 30)
    JuskeshinoSimpleTasks.alignWithTable()
    JuskeshinoHardware.moveLeftArmWithTrajectory([0.46, 0.87, -0.4, 1.99, -0.99, 0.4,  1.6], 10)
    JuskeshinoHardware.moveLeftGripper(0.7, 2.0)
    JuskeshinoHardware.moveLeftArmWithTrajectory([-0.69, 0.2, 0, 1.55, 0, 1.16, 0], 10)  # prepare
    JuskeshinoNavigation.moveDist(-0.3, 10)
    JuskeshinoNavigation.getClose("entrance", 30)






    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == "__main__":
    main()
