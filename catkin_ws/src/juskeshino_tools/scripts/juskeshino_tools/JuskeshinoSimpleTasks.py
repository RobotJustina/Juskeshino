import math
import rospy
import tf
import numpy as np
import time
from vision_msgs.msg import *
from geometry_msgs.msg import *
from juskeshino_tools.JuskeshinoHardware import JuskeshinoHardware
from juskeshino_tools.JuskeshinoNavigation import JuskeshinoNavigation
from juskeshino_tools.JuskeshinoVision import JuskeshinoVision
from juskeshino_tools.JuskeshinoHRI import JuskeshinoHRI
from juskeshino_tools.JuskeshinoManipulation import JuskeshinoManipulation


class JuskeshinoSimpleTasks:
    def setNodeHandle():
        print("JuskeshinoSimpleTasks.->Setting ROS node...")
        return True

    def waitForTheDoorToBeOpen(timeout):
        attempts = int(timeout/0.1)
        loop = rospy.Rate(10)
        door_closed = JuskeshinoNavigation.isThereObstacleInFront()
        while(not rospy.is_shutdown() and (door_closed is None or door_closed) and attempts >= 0):
            door_closed = JuskeshinoNavigation.isThereObstacleInFront()
            loop.sleep()
            attempts -=1
        return (door_closed is not None and not door_closed)

    def waitForConfirmation(timeout):
        answer = None
        while not rospy.is_shutdown() and answer is None:
            answer = JuskeshinoHRI.waitForNewSentence(timeout)
            if answer is None:
                JuskeshinoHRI.say("I cannot hear you. Please answer hoostina yes or hoostina no")
            if answer.lower() != "justina yes" and answer.lower() != "justina no":
                answer = None
        return answer.lower() == "justina yes"

    def waitForSentenceAndConfirm(timeout):
        cmd = None
        while not rospy.is_shutdown() and cmd is None:
            cmd = JuskeshinoHRI.waitForNewSentence(timeout)
            if cmd is None or cmd.lower() == "justina yes" or cmd.lower() == "justina no":
                JuskeshinoHRI.say("I cannot hear you. Can you repeat your command?")
                cmd = None
        JuskeshinoHRI.say("Do you want me to " + cmd.lower() + "?. Please answer hoostina yes or hoostina no")
        answer = None
        while not rospy.is_shutdown() and answer is None:
            answer = JuskeshinoHRI.waitForNewSentence(timeout)
            if answer is None:
                JuskeshinoHRI.say("I cannot hear you. Please answer hoostina yes or hoostina no")
            if answer.lower() != "justina yes" and answer.lower() != "justina no":
                answer = None
        return [answer.lower() == "justina yes", cmd]

    def waitForSentenceUntilConfirmed(timeout):
        JuskeshinoHRI.say("Please tell me what do you want me to do.")
        [answer, cmd] = JuskeshinoSimpleTasks.waitForSentenceAndConfirm(timeout)
        while not answer:
            JuskeshinoHRI.say("Ok. Please tell me again what do you want me to do.")
            [answer, cmd] = JuskeshinoSimpleTasks.waitForSentenceAndConfirm(timeout)
        JuskeshinoHRI.say("Ok. I'm going to " +  cmd)
        return cmd
            
    def alignWithTable():
        if not JuskeshinoHardware.moveHead(0,-1,3):
            if not JuskeshinoHardware.moveHead(0,-1,3):
                print("JuskeshinoSimpleTasks.->Cannot move head")
                return False
        edge = JuskeshinoVision.findTableEdge()
        if edge is None:
            edge = JuskeshinoVision.findTableEdge()
            if edge is None:
                print("JuskeshinoSimpleTasks.->Cannot find table edge")
                return False
        A = edge[0].y - edge[1].y
        B = edge[1].x - edge[0].x
        error_a = math.pi/2 - math.atan2(A,B)%math.pi
        error_a = 0 if abs(error_a) < 0.03 else error_a
        error_d = abs(A*edge[0].x + B*edge[0].y)/math.sqrt(A**2 + B**2) - 0.25
        error_d = 0 if error_d < 0.07 else error_d
        timeout = ((abs(error_a) + abs(error_d))/0.5 + 1)
        print("JuskeshinoSimpleTasks.->Moving to align with table d="+str(error_d) + " and theta="+str(error_a))
        return JuskeshinoNavigation.moveDistAngle(error_d, error_a, timeout)

    def findHumanAndApproach(timeout):
        head_poses = [[0.0, 0.0], [0.3, 0], [-0.3,0], [0.6, 0], [-0.6,0], [0.9,0], [-0.9, 0], [1.2,0], [-1.2,0], [1.5,0], [-1.5,0]]
        JuskeshinoVision.enableHumanPoseDetection(True)

        for [pan, tilt] in head_poses:
            if not JuskeshinoHardware.moveHead(pan,tilt,2.0):
                JuskeshinoHardware.moveHead(pan,tilt,2.0)
            rospy.sleep(0.5)
            human_poses = rospy.wait_for_message("/vision/human_pose/human_pose_array", HumanCoordinatesArray, timeout=1.0)
            src_frame_id = human_poses.header.frame_id
            human_poses = human_poses.coordinates_array
            if len(human_poses) > 0:
                break
        if len(human_poses) < 1:
            JuskeshinoVision.enableHumanPoseDetection(False)
            return False
        
        nearest_x, nearest_y, nearest_z = 0,0,0
        nearest_dist = float("inf")
        for p in human_poses:
            x,y,z = 0,0,0
            for k in p.keypoints_array:
                x += k.keypoint_coordinates.position.x
                y += k.keypoint_coordinates.position.y
                z += k.keypoint_coordinates.position.z
            x,y,z = x/len(p.keypoints_array), y/len(p.keypoints_array), z/len(p.keypoints_array)
            [x, y, z] = JuskeshinoSimpleTasks.transformPoint(x, y, z, "base_link", src_frame_id)
            dist = math.sqrt(x**2 + y**2)
            if dist < nearest_dist:
                nearest_x = x
                nearest_y = y
                nearest_z = z
                nearest_dist = dist
        JuskeshinoVision.enableHumanPoseDetection(False)
        print("JuskeshinoSimpleTask.->Nearest human pose detected at: " + str([nearest_x, nearest_y, nearest_z]))
        nearest_dist -= 1.0 #Robot will get close at 1 meter from human
        nearest_theta = math.atan2(nearest_y, nearest_x)
        nearest_x = nearest_dist*math.cos(nearest_theta)
        nearest_y = nearest_dist*math.sin(nearest_theta)
        [robot_x, robot_y, robot_a] = JuskeshinoNavigation.getRobotPoseWrtMap()
        goal_x = robot_x + nearest_x*math.cos(robot_a) - nearest_y*math.sin(robot_a)
        goal_y = robot_y + nearest_x*math.sin(robot_a) + nearest_y*math.cos(robot_a)
        goal_a = (robot_a + nearest_theta + math.pi)%(2*math.pi) - math.pi
        if not JuskeshinoNavigation.getCloseXYA(goal_x, goal_y, goal_a, 20):
            if not JuskeshinoNavigation.getCloseXYA(goal_x, goal_y, goal_a, 20):
                return False
        return JuskeshinoHardware.moveHead(0,0,2.0)
                
    def transformPoint(x,y,z, target_frame, source_frame):
        listener = tf.TransformListener()
        listener.waitForTransform(target_frame, source_frame, rospy.Time(), rospy.Duration(4.0))
        obj_p = PointStamped()
        obj_p.header.frame_id = source_frame
        obj_p.header.stamp = rospy.Time(0)
        obj_p.point.x, obj_p.point.y, obj_p.point.z = x,y,z
        obj_p = listener.transformPoint(target_frame, obj_p)
        return [obj_p.point.x, obj_p.point.y, obj_p.point.z]
    

    def handling_location(vision_obj, arm ):
        position_obj = vision_obj.pose.position
        l_threshold_la       = 0.25
        r_threshold_la       = 0.11
        l_threshold_ra       = -0.14
        r_threshold_ra       = -0.32
        suitable_grip_height = 0.70

        if position_obj.z > suitable_grip_height:
            try:
                JuskeshinoHardware.moveTorso(np.linalg.norm(position_obj.z - suitable_grip_height) , 5.0)
                time.sleep(0.5)
            except:
                print("Cannot move torso")
        if "ra":
            if position_obj.y > l_threshold_ra:
                JuskeshinoNavigation.moveLateral( l_threshold_ra - position_obj.y , 5.0)
                return True
            if position_obj.y < r_threshold_ra:
                JuskeshinoNavigation.moveLateral(r_threshold_ra - position_obj.y , 5.0)
                return True
            return False
        else:     # Lado izq del robot
            if position_obj.y > l_threshold_la:
                JuskeshinoNavigation.moveLateral(position_obj.y - l_threshold_la , 5.0)
                return True
            if position_obj.y < r_threshold_la:
                JuskeshinoNavigation.moveLateral(position_obj.y - r_threshold_la , 5.0)
                return True
            return False



    def object_search(name_obj):
        JuskeshinoHardware.moveHead(0,-1, 5)
        [obj, img] = JuskeshinoVision.detectAndRecognizeObject(name_obj)

        if obj == None: # si no reconocio el objeto
            JuskeshinoHardware.moveHead(-0.5,-1, 5) #move head to the right 
            time.sleep(1.5)
            [obj, img] = JuskeshinoVision.detectAndRecognizeObject(name_obj)
            print("JuskeshinoSimpleTask.->Primer intento")

            if obj == None: # si no reconocio el objeto
                JuskeshinoHardware.moveHead(0.5,-1, 5) #move head to the left
                time.sleep(1.5)
                [obj, img] = JuskeshinoVision.detectAndRecognizeObject(name_obj)
                print("JuskeshinoSimpleTask.->Segundo intento")

                if obj == None: # si no reconocio el objeto
                    print("JuskeshinoSimpleTask.->NO se encontro el objeto")
                    return None
                
            return [obj, img]

        else:
            print("JuskeshinoSimpleTask.->Objeto detectado ")
            return [obj, img]





        

