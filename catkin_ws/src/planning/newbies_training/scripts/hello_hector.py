#!/usr/bin/env python3
import rospy
import rospkg
import time
from juskeshino_tools.JuskeshinoNavigation import JuskeshinoNavigation
from juskeshino_tools.JuskeshinoVision import JuskeshinoVision
from juskeshino_tools.JuskeshinoHardware import JuskeshinoHardware
from juskeshino_tools.JuskeshinoSimpleTasks import JuskeshinoSimpleTasks
from juskeshino_tools.JuskeshinoHRI import JuskeshinoHRI
from juskeshino_tools.JuskeshinoManipulation import JuskeshinoManipulation
from juskeshino_tools.JuskeshinoKnowledge import JuskeshinoKnowledge

def main():
    rospy.init_node("hello_hector")
    rate = rospy.Rate(10)
    rospack = rospkg.RosPack()
    JuskeshinoNavigation.setNodeHandle()
    JuskeshinoVision.setNodeHandle()
    JuskeshinoHardware.setNodeHandle()
    JuskeshinoSimpleTasks.setNodeHandle()
    JuskeshinoHRI.setNodeHandle()
    JuskeshinoManipulation.setNodeHandle()
    JuskeshinoKnowledge.setNodeHandle()

    JuskeshinoHRI.say("hello")
    JuskeshinoNavigation.moveDist(0.5, 7)
    first = [2, 0, 0, 0, 0, 0, 0]
    JuskeshinoHardware.moveLeftArmWithTrajectory(first, 10)
    time.sleep(2)
    first = [0, 0, 0, 0, 0, 0, 0]
    JuskeshinoHardware.moveLeftArmWithTrajectory(first, 10)
    JuskeshinoHRI.say("bye")
    """
    data1 = [0, 0, 0, 0, 0, 0, 0]
    data2 = [0, 0, 0, 0, 0, 0, 0]
    for i in range(7):
        data1 = [0, 0, 0, 0, 0, 0, 0]
        data2 = [0, 0, 0, 0, 0, 0, 0]
        data1[i] = 1
        data2[-i] = 1
        JuskeshinoHardware.moveLeftArmWithTrajectory(data1, 10)
        JuskeshinoHardware.moveRightArmWithTrajectory(data2, 10)
        time.sleep(1)
    """

if __name__ == "__main__":
    main()
