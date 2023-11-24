#!/usr/bin/env python
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

def main():
    print("INITIALIZING QUALIFICATION 2024 TEST BY MARCOSOFT...")
    rospy.init_node("act_pln")
    rate = rospy.Rate(10)

    rospack = rospkg.RosPack()
    locations_default = rospack.get_path("config_files") + "/known_locations.yaml"
    locations_file = rospy.get_param("~locations", locations_default)

    JuskeshinoNavigation.setNodeHandle()
    JuskeshinoVision.setNodeHandle()
    JuskeshinoHardware.setNodeHandle()
    JuskeshinoSimpleTasks.setNodeHandle()
    JuskeshinoHRI.setNodeHandle()
    JuskeshinoManipulation.setNodeHandle()
    JuskeshinoKnowledge.setNodeHandle()
    JuskeshinoKnowledge.loadLocations(locations_file)

    JuskeshinoHRI.say("I'm waiting for the door to be open")
    if not JuskeshinoSimpleTasks.waitForTheDoorToBeOpen(100):
        print("ACT-PLN.->Door never opened")
        return
    JuskeshinoHRI.say("I can see now that the door is open")
    JuskeshinoHRI.say("I'm going to the start position.")
    if not JuskeshinoNavigation.getClose("entrance", 30):
        print("ACT-PLN.->Cannot get close to start position")
    JuskeshinoHRI.say("I arrived to start position")

    
    print("ACT-PLN.->Waiting for command")
    cmd = JuskeshinoSimpleTasks.waitForSentenceUntilConfirmed(10)
    print("ACT-PLN.->Command confirmed: " + cmd)
    if not JuskeshinoNavigation.getClose("desk", 30):
        print("ACT-PLN.->Cannot get close to goal position: desk")
    if not JuskeshinoHardware.moveHead(0,-1, 5):
        print("ACT-PLN.->Cannot move head")
    if not JuskeshinoSimpleTasks.alignWithTable():
        print("ACT-PLN.->Cannot align with table")
    print("ACT-PLN.->Sending goal traj to prepare")
    JuskeshinoHardware.moveLeftArmWithTrajectory([-0.66, 0.36, -0.04, 1.79, 0, 1.02, 0], 10)
    print("ACT-PLN.->Open gripper")
    JuskeshinoHardware.moveLeftGripper(0.7, 2.0)
    print("ACT-PLN.->Trying to detect object")
    JuskeshinoHRI.say("I'm trying to detect the pringles")
    [obj, img] = JuskeshinoVision.detectAndRecognizeObject("pringles")
    print("ACT-PLN.->Detected object: " + str([obj.id, obj.category, obj.object_state]))
    print("ACT-PLN.->Planning best gripper configuration")
    JuskeshinoHRI.say("I found the pringles, I'm going to try to grasp them")
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
    JuskeshinoHRI.say("I'have grasped the pringles")
    JuskeshinoNavigation.moveDist(-0.3, 10)
    print("ACT-PLN.->Getting close to cupboard")
    JuskeshinoHRI.say("I'm going to the cupboard")
    JuskeshinoNavigation.getClose("cupboard", 30)
    print("ACT-PLN.->Aligning with table")
    JuskeshinoSimpleTasks.alignWithTable()
    print("ACT-PLN.->Moving left arm to deliver position")
    JuskeshinoHRI.say("I'm going to leave the pringles")
    JuskeshinoHardware.moveLeftArmWithTrajectory([0.46, 0.87, -0.4, 1.99, -0.99, 0.4,  1.6], 10)
    print("ACT-PLN.->Open gripper")
    JuskeshinoHardware.moveLeftGripper(0.7, 2.0)
    print("ACT-PLN.->Moving left arm to prepare")
    JuskeshinoHardware.moveLeftArmWithTrajectory([-0.69, 0.2, 0, 1.55, 0, 1.16, 0], 10)  # prepare
    JuskeshinoHardware.moveLeftArmWithTrajectory([ 0.00, 0.0, 0, 0.00, 0, 0.00, 0], 10)  # prepare
    print("ACT-PLN.->Moving base backwards")
    JuskeshinoNavigation.moveDist(-0.3, 10)
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

    return 
    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == "__main__":
    main()
