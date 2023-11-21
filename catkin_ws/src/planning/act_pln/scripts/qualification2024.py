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

def main():
    print("INITIALIZING QUALIFICATION 2024 TEST BY MARCOSOFT...")
    rospy.init_node("act_pln")
    rate = rospy.Rate(10)

    JuskeshinoNavigation.setNodeHandle()
    JuskeshinoVision.setNodeHandle()
    JuskeshinoHardware.setNodeHandle()
    JuskeshinoSimpleTasks.setNodeHandle()
    JuskeshinoHRI.setNodeHandle()
    JuskeshinoManipulation.setNodeHandle()

    JuskeshinoHRI.say("I'm waiting for the door to be open")
    if not JuskeshinoSimpleTasks.waitForTheDoorToBeOpen(100):
        print("Door never opened")
        return

    JuskeshinoHRI.say("I'm goint to the start position.")
    if not JuskeshinoNavigation.getCloseXYA(2.0, 0, 0, 30):
        print("Cannot get close to start position")
    JuskeshinoHRI.say("I arrived to start position")

    print("Waiting for command")
    cmd = JuskeshinoSimpleTasks.waitForSentenceUntilConfirmed(10)
    print("Command confirmed: " + cmd)
    
    if not JuskeshinoNavigation.getCloseXYA(3.2, 5.6, -1.57, 30):
        print("Cannot get close to goal position")
    if not JuskeshinoHardware.moveHead(0,-1, 5):
        print("Cannot move head")
    if not JuskeshinoSimpleTasks.alignWithTable():
        print("Cannot align with table")

    
    print("Sending goal traj")
    JuskeshinoHardware.moveLeftArmWithTrajectory([-1.2, 0.2, 0, 1.6, 0, 1.1, 0], 10)
    JuskeshinoHardware.moveLeftGripper(1.0, 1.0)
    print("Goal traj reached")
    [obj, img] = JuskeshinoVision.detectAndRecognizeObject("pringles")
    resp = JuskeshinoManipulation.planBestGraspingConfiguration(obj)
    JuskeshinoHardware.moveLeftArmWithTrajectory(resp.articular_trajectory,10)
    JuskeshinoHardware.moveLeftGripper(0.0, 1.0)
    JuskeshinoHardware.moveLeftArmWithTrajectory([-1.2, 0.2, 0, 1.6, 0, 1.1, 0], 10)
    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == "__main__":
    main()
