import numpy
import rospy
from std_msgs.msg import Float64, Float64MultiArray
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import PointCloud2

HEAD_TOLERANCE = 0.1

class JuskeshinoHardware:
    def setNodeHandle():
        print("JuskeshinoHardware.->Setting ros node...")
        JuskeshinoHardware.pubCmdVel     = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        JuskeshinoHardware.pubTorso      = rospy.Publisher("/torso_controller/command", Float64, queue_size=1)
        JuskeshinoHardware.pubLaGoalQ    = rospy.Publisher("/hardware/left_arm/goal_pose", Float64MultiArray, queue_size=1)
        JuskeshinoHardware.pubRaGoalQ    = rospy.Publisher("/hardware/right_arm/goal_pose", Float64MultiArray, queue_size=1)
        JuskeshinoHardware.pubLaGoalTraj = rospy.Publisher("/manipulation/la_q_trajectory", JointTrajectory, queue_size=1)
        JuskeshinoHardware.pubRaGoalTraj = rospy.Publisher("/manipulation/ra_q_trajectory", JointTrajectory, queue_size=1)
        JuskeshinoHardware.pubHdGoalQ    = rospy.Publisher("/hardware/head/goal_pose", Float64MultiArray, queue_size=1)
        JuskeshinoHardware.pubLaGoalGrip = rospy.Publisher("/hardware/left_arm/goal_gripper", Float64, queue_size=1)
        JuskeshinoHardware.pubRaGoalGrip = rospy.Publisher("/hardware/right_arm/goal_gripper", Float64, queue_size=1)
        
        loop = rospy.Rate(10)
        counter = 3
        while not rospy.is_shutdown() and counter > 0:
            counter-=1
            loop.sleep()
        return True

    def startMoveHead(pan, tilt):
        msg = Float64MultiArray()
        msg.data.append(pan)
        msg.data.append(tilt)
        JuskeshinoHardware.pubHdGoalQ.publish(msg)
        JuskeshinoHardware.hdGoalPose = numpy.asarray([pan,tilt])
        return

    def waitForHeadGoalReached(timeOut_ms):
        current = numpy.asarray(rospy.wait_for_message("/hardware/head/current_pose", Float64MultiArray, timeout=1.0).data)
        e = numpy.linalg.norm(current - JuskeshinoHardware.hdGoalPose)
        attempts = int(timeOut_ms/100)
        loop = rospy.Rate(10)
        while (not rospy.is_shutdown() and e > HEAD_TOLERANCE and attempts > 0):
            loop.sleep()
            current = numpy.asarray(rospy.wait_for_message("/hardware/head/current_pose", Float64MultiArray, timeout=1.0).data)
            e = numpy.linalg.norm(current - JuskeshinoHardware.hdGoalPose)
            attempts -= 1
        return e <= HEAD_TOLERANCE

    def moveHead(pan, tilt, timeOut_ms):
        JuskeshinoHardware.startMoveHead(pan, tilt)
        return JuskeshinoHardware.waitForHeadGoalReached(timeOut_ms)
        
