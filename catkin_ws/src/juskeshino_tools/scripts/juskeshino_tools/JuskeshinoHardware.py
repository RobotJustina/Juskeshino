import numpy
import rospy
from std_msgs.msg import Float64, Float32, Float64MultiArray
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import PointCloud2
from manip_msgs.srv import *

HEAD_TOLERANCE = 0.1
LEFT_ARM_TOLERANCE  = 0.2
RIGHT_ARM_TOLERANCE = 0.2
TORSO_TOLERANCE = 0.1

class JuskeshinoHardware:
    def setNodeHandle():
        print("JuskeshinoHardware.->Setting ros node...")
        JuskeshinoHardware.pubCmdVel     = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        JuskeshinoHardware.pubTorso      = rospy.Publisher("/hardware/torso/goal_pose", Float64, queue_size=1)
        JuskeshinoHardware.pubLaGoalQ    = rospy.Publisher("/hardware/left_arm/goal_pose", Float64MultiArray, queue_size=10)
        JuskeshinoHardware.pubRaGoalQ    = rospy.Publisher("/hardware/right_arm/goal_pose", Float64MultiArray, queue_size=10)
        JuskeshinoHardware.pubLaGoalTraj = rospy.Publisher("/manipulation/la_q_trajectory", JointTrajectory, queue_size=10)
        JuskeshinoHardware.pubRaGoalTraj = rospy.Publisher("/manipulation/ra_q_trajectory", JointTrajectory, queue_size=10)
        JuskeshinoHardware.pubHdGoalQ    = rospy.Publisher("/hardware/head/goal_pose", Float64MultiArray, queue_size=10)
        JuskeshinoHardware.pubLaGoalGrip = rospy.Publisher("/hardware/left_arm/goal_gripper", Float64, queue_size=10)
        JuskeshinoHardware.pubRaGoalGrip = rospy.Publisher("/hardware/right_arm/goal_gripper", Float64, queue_size=10)
        JuskeshinoHardware.cltPolyTraj   = rospy.ServiceProxy("/manipulation/polynomial_trajectory", GetPolynomialTrajectory)
        loop = rospy.Rate(10)
        counter = 3
        while not rospy.is_shutdown() and counter > 0:
            counter-=1
            loop.sleep()
        return True

    def getPolynomialTrajectory(p1,p2):
        req = GetPolynomialTrajectoryRequest()
        req.p1 = p1
        req.p2 = p2
        req.time_step  = 0.05
        max_delta = -1;
        for i in range(len(p1)):
            if abs(p1[i] - p2[i]) > max_delta:
                max_delta = abs(p1[i] - p2[i])
        req.duration = max_delta / 0.7 + 0.5
        try:
            resp = JuskeshinoHardware.cltPolyTraj(req)
            return resp.trajectory
        except:
            return None

    def startMoveHead(pan, tilt):
        msg = Float64MultiArray()
        msg.data.append(pan)
        msg.data.append(tilt)
        JuskeshinoHardware.pubHdGoalQ.publish(msg)
        JuskeshinoHardware.hdGoalPose = numpy.asarray([pan,tilt])
        return

    def startMoveLeftArm(q):
        msg = Float64MultiArray()
        msg.data = q
        JuskeshinoHardware.pubLaGoalQ.publish(msg)
        JuskeshinoHardware.laGoalPose = numpy.asarray(q)
        return

    def startMoveRightArm(q):
        msg = Float64MultiArray()
        msg.data = q
        JuskeshinoHardware.pubRaGoalQ.publish(msg)
        JuskeshinoHardware.raGoalPose = numpy.asarray(q)
        return

    def startMoveTorso(dist):
        msg = Float64()
        msg.data = dist
        JuskeshinoHardware.pubTorso.publish(msg)
        JuskeshinoHardware.torsoGoalPose = dist
        return
        

    def startMoveLeftArmWithTrajectory(q):
        if not isinstance(q, JointTrajectory):
            q1 = numpy.asarray(rospy.wait_for_message("/hardware/left_arm/current_pose", Float64MultiArray, timeout=1.0).data)
            traj = JuskeshinoHardware.getPolynomialTrajectory(q1, q)
        else:
            traj = q
        JuskeshinoHardware.pubLaGoalTraj.publish(traj)
        JuskeshinoHardware.laGoalPose = numpy.asarray(traj.points[-1].positions)
        return

    def startMoveRightArmWithTrajectory(q):
        if not isinstance(q, JointTrajectory):
            q1 = numpy.asarray(rospy.wait_for_message("/hardware/right_arm/current_pose", Float64MultiArray, timeout=1.0).data)
            traj = JuskeshinoHardware.getPolynomialTrajectory(q1, q)
        else:
            traj = q
        JuskeshinoHardware.pubRaGoalTraj.publish(traj)
        JuskeshinoHardware.raGoalPose = numpy.asarray(traj.points[-1].positions)
        return

    def moveHead(pan, tilt, timeout):
        JuskeshinoHardware.startMoveHead(pan, tilt)
        return JuskeshinoHardware.waitForHdGoalReached(timeout)
    
    def moveTorso(dist, timeout):
        JuskeshinoHardware.startMoveTorso(dist)
        return JuskeshinoHardware.waitForTorsoGoalReached(timeout)

    def moveLeftArm(q, timeout):
        JuskeshinoHardware.startMoveLeftArm(q)
        return JuskeshinoHardware.waitForLaGoalReached(timeout)

    def moveRightArm(q, timeout):
        JuskeshinoHardware.startMoveRightArm(q)
        return JuskeshinoHardware.waitForLaGoalReached(timeout)

    def moveLeftArmWithTrajectory(q, timeout):
        JuskeshinoHardware.startMoveLeftArmWithTrajectory(q)
        return JuskeshinoHardware.waitForLaGoalReached(timeout)

    def moveRightArmWithTrajectory(q, timeout):
        JuskeshinoHardware.startMoveRightArmWithTrajectory(q)
        return JuskeshinoHardware.waitForRaGoalReached(timeout)

    def moveLeftGripper(q, timeout):
        msg = Float64()
        msg.data = q
        JuskeshinoHardware.pubLaGoalGrip.publish(msg)
        JuskeshinoHardware.pubLaGoalGrip.publish(msg)
        rospy.sleep(timeout)

    def moveRightGripper(q, timeout):
        msg = Float64()
        msg.data = q
        JuskeshinoHardware.pubRaGoalGrip.publish(msg)
        JuskeshinoHardware.pubRaGoalGrip.publish(msg)
        rospy.sleep(timeout)

    def waitForHdGoalReached(timeout):
        current = numpy.asarray(rospy.wait_for_message("/hardware/head/current_pose", Float64MultiArray, timeout=1.0).data)
        e = numpy.linalg.norm(current - JuskeshinoHardware.hdGoalPose)
        attempts = int(timeout/0.1)
        loop = rospy.Rate(10)
        while (not rospy.is_shutdown() and e > HEAD_TOLERANCE and attempts > 0):
            loop.sleep()
            current = numpy.asarray(rospy.wait_for_message("/hardware/head/current_pose", Float64MultiArray, timeout=1.0).data)
            e = numpy.linalg.norm(current - JuskeshinoHardware.hdGoalPose)
            attempts -= 1
        return e <= HEAD_TOLERANCE
    
    def waitForTorsoGoalReached(timeout):
        current = rospy.wait_for_message("/hardware/torso/current_pose", Float64, timeout=1.0).data
        e = numpy.linalg.norm(current - JuskeshinoHardware.laGoalPose)
        attempts = int(timeout/0.1)
        loop = rospy.Rate(10)
        while (not rospy.is_shutdown() and e > TORSO_TOLERANCE and attempts > 0):
            loop.sleep()
            current = numpy.asarray(rospy.wait_for_message("/hardware/head/current_pose", Float64MultiArray, timeout=1.0).data)
            e = numpy.linalg.norm(current - JuskeshinoHardware.torsoGoalPose)
            attempts -= 1
        return e <= TORSO_TOLERANCE



    def waitForLaGoalReached(timeout):
        current = numpy.asarray(rospy.wait_for_message("/hardware/left_arm/current_pose", Float64MultiArray, timeout=1.0).data)
        e = numpy.linalg.norm(current - JuskeshinoHardware.laGoalPose)
        attempts = int(timeout/0.1)
        loop = rospy.Rate(10)
        while (not rospy.is_shutdown() and e > LEFT_ARM_TOLERANCE and attempts > 0):
            loop.sleep()
            current = numpy.asarray(rospy.wait_for_message("/hardware/left_arm/current_pose", Float64MultiArray, timeout=1.0).data)
            e = numpy.linalg.norm(current - JuskeshinoHardware.laGoalPose)
            attempts -= 1
        return e <= LEFT_ARM_TOLERANCE
    

    def waitForRaGoalReached(timeout):
        current = numpy.asarray(rospy.wait_for_message("/hardware/right_arm/current_pose", Float64MultiArray, timeout=1.0).data)
        e = numpy.linalg.norm(current - JuskeshinoHardware.raGoalPose)
        attempts = int(timeout/0.1)
        loop = rospy.Rate(10)
        while (not rospy.is_shutdown() and e > RIGHT_ARM_TOLERANCE and attempts > 0):
            loop.sleep()
            current = numpy.asarray(rospy.wait_for_message("/hardware/right_arm/current_pose", Float64MultiArray, timeout=1.0).data)
            e = numpy.linalg.norm(current - JuskeshinoHardware.raGoalPose)
            attempts -= 1
        return e <= RIGHT_ARM_TOLERANCE
