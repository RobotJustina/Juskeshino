#!/usr/bin/env python3
import rospy
import tf
import numpy as np
import time
import math
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
        error_d = abs(A*edge[0].x + B*edge[0].y)/math.sqrt(A**2 + B**2) - 0.3
        error_d = 0 if error_d < 0.07 else error_d
        timeout = ((abs(error_a) + abs(error_d))/0.5 + 1)
        print("JuskeshinoSimpleTasks.->Moving to align with table d="+str(error_d) + " and theta="+str(error_a))
        return JuskeshinoNavigation.moveDistAngle(error_d, error_a, timeout)
    

    def GetClosestHumanPose(timeout):
        JuskeshinoVision.enableHumanPose(True)
        human_posess = rospy.wait_for_message("/vision/human_pose/human_pose_array", HumanCoordinatesArray, timeout=3.0)
        human_poses = human_posess.coordinates_array
        min_dist = 500
        if human_poses is None:
            JuskeshinoVision.enableHumanPose(False)
            return [None, None]

        for person in human_poses:         
            for k in person.keypoints_array:
                if "nose" in k.keypoint_name:
                    [x, y, z] = JuskeshinoSimpleTasks.transformPoint( k.keypoint_coordinates.position.x ,
                                                                      k.keypoint_coordinates.position.y, 
                                                                      k.keypoint_coordinates.position.z, "base_link", human_posess.header.frame_id )
                    dist = math.sqrt(x**2 + y**2)
                    if dist < min_dist:
                        min_dist = dist
                        closest_human_id = person.person_id
                        closest_human_pose = person

        time.sleep(3)
        JuskeshinoVision.enableHumanPose(False)
        print("JuskeshinoSimpleTask.GetHumanPoseArray->Nearest human id:____ ", closest_human_id)
        return closest_human_pose


                
    def transformPoint(x,y,z, target_frame, source_frame):
        listener = tf.TransformListener()
        listener.waitForTransform(target_frame, source_frame, rospy.Time(), rospy.Duration(4.0))
        obj_p = PointStamped()
        obj_p.header.frame_id = source_frame
        obj_p.header.stamp = rospy.Time(0)
        obj_p.point.x, obj_p.point.y, obj_p.point.z = x,y,z
        obj_p = listener.transformPoint(target_frame, obj_p)
        return [obj_p.point.x, obj_p.point.y, obj_p.point.z]
    
    
    def handling_location_la(position_obj):     # Recibe un  objeto de tipo position extraido de un mensaje pose de ROS
        l_threshold_la       = 0.26
        r_threshold_la       = 0.15
        
        if position_obj.y > l_threshold_la:     # Objeto a la izquierda
            mov_izq = position_obj.y - l_threshold_la
            print("mov izq", mov_izq)
            JuskeshinoNavigation.moveLateral(mov_izq , 5.0)
            return 

        if position_obj.y < r_threshold_la:     # Objeto a la derecha
            mov_der = position_obj.y - r_threshold_la
            print("mov der", mov_der)
            JuskeshinoNavigation.moveLateral(mov_der , 5.0)
            time.sleep(0.2)
            return 
            
    


    def object_search(name_obj, tilt):
        JuskeshinoHardware.moveHead(0,tilt, 5)
        print("JuskeshinoSimpleTask.->Primer intento")
        [obj, img] = JuskeshinoVision.detectAndRecognizeObjectWithoutOrientation(name_obj)

        if obj == None: # si no reconocio el objeto
            JuskeshinoHardware.moveHead(-0.5,tilt, 5) #move head to the right 
            time.sleep(1)
            [obj, img] = JuskeshinoVision.detectAndRecognizeObjectWithoutOrientation(name_obj)
            print("JuskeshinoSimpleTask.->Segundo intento")

            if obj == None: # si no reconocio el objeto
                JuskeshinoHardware.moveHead(0.5,tilt, 5) #move head to the left
                time.sleep(1)
                [obj, img] = JuskeshinoVision.detectAndRecognizeObjectWithoutOrientation(name_obj)
                print("JuskeshinoSimpleTask.->tercer intento")

                if obj == None: # si no reconocio el objeto
                    print("JuskeshinoSimpleTask.->NO se encontro el objeto")
                    return None
                
            return [obj, img]

        else:
            print("JuskeshinoSimpleTask.->Objeto detectado ")
            return [obj, img]
        


        
    def object_search_orientation(name_obj, tilt):
        JuskeshinoHardware.moveHead(0,tilt, 5)
        print("JuskeshinoSimpleTask.->Primer intento")
        [obj, img] = JuskeshinoVision.detectAndRecognizeObject(name_obj)

        if obj == None: # si no reconocio el objeto
            JuskeshinoHardware.moveHead(-0.5,tilt, 5) #move head to the right 
            time.sleep(1)
            [obj, img] = JuskeshinoVision.detectAndRecognizeObject(name_obj)
            print("JuskeshinoSimpleTask.->Segundo intento")

            if obj == None: # si no reconocio el objeto
                JuskeshinoHardware.moveHead(0.5,tilt, 5) #move head to the left
                time.sleep(1)
                [obj, img] = JuskeshinoVision.detectAndRecognizeObject(name_obj)
                print("JuskeshinoSimpleTask.->tercer intento")

                if obj == None: # si no reconocio el objeto
                    print("JuskeshinoSimpleTask.->NO se encontro el objeto")
                    return None
                
            return [obj, img]

        else:
            print("JuskeshinoSimpleTask.->Objeto detectado ")
            return [obj, img]




