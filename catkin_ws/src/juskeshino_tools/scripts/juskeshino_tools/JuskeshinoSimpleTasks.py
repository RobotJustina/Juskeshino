#!/usr/bin/env python3
import rospy
import tf
import numpy as np
import time
import math
import cv2
from vision_msgs.msg import *
from vision_msgs.srv import *
from geometry_msgs.msg import *
from sensor_msgs.msg import PointCloud2
from juskeshino_tools.JuskeshinoHardware import JuskeshinoHardware
from juskeshino_tools.JuskeshinoNavigation import JuskeshinoNavigation
from juskeshino_tools.JuskeshinoVision import JuskeshinoVision
from juskeshino_tools.JuskeshinoHRI import JuskeshinoHRI
from juskeshino_tools.JuskeshinoManipulation import JuskeshinoManipulation


class JuskeshinoSimpleTasks:
    def setNodeHandle():
        print("JuskeshinoSimpleTasks.->Setting ROS node...")
        return True

    #
    # Tasks related to starting sequences for robocup tests
    #
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

    #
    # Tasks related to object recognition
    #
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
            
    def transformPoint(pointxyz, target_frame, source_frame):
        listener = tf.TransformListener()
        listener.waitForTransform(target_frame, source_frame, rospy.Time(), rospy.Duration(4.0))
        obj_p = PointStamped()
        obj_p.header.frame_id = source_frame
        obj_p.header.stamp = rospy.Time(0)
        obj_p.point = pointxyz
        obj_p =  listener.transformPoint(target_frame, obj_p)
        return obj_p.point
        
        
    #
    # Tasks related to finding and recognizing people
    #
    def getClosestPerson(gesture=None, body_pose=None):
        JuskeshinoVision.enableHumanPose(True)
        msg_persons = rospy.wait_for_message("/vision/human_pose/human_pose_array", Persons, timeout=7.0)
        msg_persons = rospy.wait_for_message("/vision/human_pose/human_pose_array", Persons, timeout=7.0)
        #msg_persons = rospy.wait_for_message("/vision/human_pose/human_pose_array", Persons, timeout=7.0)
        print("JuskeshinoSimpleTasks.->Got " + str(len(msg_persons.persons)) + " persons")
        JuskeshinoVision.enableHumanPose(False)
        if msg_persons is None or len(msg_persons.persons) < 1:
            return None 
        min_dist = 1000
        closest_person = None
        for person in msg_persons.persons:
            if gesture is not None and person.gesture.id != gesture:
                continue
            if body_pose is not None and person.body_pose.id != body_pose:
                continue
            p = Point(x=person.pose.position.x, y=person.pose.position.y, z=person.pose.position.z)
            p = JuskeshinoSimpleTasks.transformPoint( p, "base_link", msg_persons.header.frame_id )
            d = math.sqrt(p.x**2 + p.y**2 + p.z**2)
            if(d < min_dist):
                min_dist = d
                closest_person = person
        return closest_person
                
    def findHumanAndApproach(timeout, gesture=None, body_pose=None):
        head_poses = [[0,-0.1], [0.5, -0.1], [-0.5, -0.1], [1,-0.1], [-1,-0.1], [1.5,-0.1], [-1.5, -0.1]]
        for [pan,tilt] in head_poses:
            JuskeshinoHardware.moveHead(pan, tilt, 3)
            timeout -= 3
            person = JuskeshinoSimpleTasks.getClosestPerson(gesture, body_pose)
            if person is not None:
                break
        if person is None:
            print("JuskeshinoSimpleTasks.->Cannot find any person.")
            return False, None
        print("JuskeshinoSimpleTasks.->Found closest person at " + str([person.pose.position.x, person.pose.position.y, person.pose.position.z]))
        p = Point(x=person.pose.position.x, y=person.pose.position.y, z=person.pose.position.z) #Person point
        p = JuskeshinoSimpleTasks.transformPoint( p, "map", person.header.frame_id ) #wrt map
        xg, yg, ag = JuskeshinoNavigation.findSuitableNearPointInFreeSpace(p.x, p.y)
        if xg is None or yg is None or ag is None:
            print("JuskeshinoSimpleTasks.->Cannot approach to person. ")
            return False, person
        if not JuskeshinoNavigation.getCloseXYA(xg, yg, ag, timeout):
            print("JuskeshinoSimpleTasks.->Cannot approach to person.")
            return False, person
        JuskeshinoHardware.moveHead(0,0,3.0)
        return True, person

    def findHumanBodyPose(body_pose):
        print("JuskeshinoSimpleTasks:.->Trying to find human body pose: ", body_pose)
        if "standing" in body_pose.lower():
            body_pose = vision_msgs.Gesture.STANDING
        elif "sitting" in body_pose.lower():
            body_pose = vision_msgs.Gesture.SITTING
        else:
            body_pose = vision_msgs.Gesture.LYING
        JuskeshinoVision.enableHumanPose(True)
        msg_persons = rospy.wait_for_message("/vision/human_pose/human_pose_array", Persons, timeout=7.0)
        print("JuskeshinoSimpleTasks.->Got " + str(len(msg_persons.persons)) + " persons")
        if msg_persons is None or len(msg_persons.persons) < 1:
            return None
        for person in msg_persons.person:
            if person.body_pose == body_pose:
                return person
        return None

    def findHumanWithBodyPoseAndApproach(body_pose, timeout):
        head_poses = [[0,-0.1], [0.5, -0.1], [-0.5, -0.1], [1,-0.1], [-1,-0.1], [1.5,-0.1], [-1.5, -0.1]]
        for [pan,tilt] in head_poses:
            JuskeshinoHardware.moveHead(pan, tilt, 3)
            timeout -= 3
            person = JuskeshinoSimpleTasks.findHumanBodyPose(body_pose)
            if person is not None:
                break
        if person is None:
            print("JuskeshinoSimpleTasks.->Cannot find a " + body_pose + " person")
            return False
        print("JuskeshinoSimpleTasks.->Found person with body pose at " + str([person.pose.position.x, person.pose.position.y, person.pose.position.z]))
        p = Point(x=person.pose.position.x, y=person.pose.position.y, z=person.pose.position.z) #Person point
        p = JuskeshinoSimpleTasks.transformPoint( p, "map", person.header.frame_id ) #wrt map
        xg, yg, ag = JuskeshinoNavigation.findSuitableNearPointInFreeSpace(p.x, p.y)
        if xg is None or yg is None or ag is None:
            print("JuskeshinoSimpleTasks.->Cannot approach to person. ")
            return False
        if not JuskeshinoNavigation.getCloseXYA(xg, yg, ag, timeout):
            print("JuskeshinoSimpleTasks.->Cannot approach to person.")
            return False
        JuskeshinoHardware.moveHead(0,0,3.0)
        return True
        
    
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
        JuskeshinoHardware.moveHead(0,tilt, 10)
        print("JuskeshinoSimpleTask.->Primer intento")
        [obj, img] = JuskeshinoVision.detectAndRecognizeObjectWithoutOrientation(name_obj)

        if obj == None: # si no reconocio el objeto
            print("No se encontró el objeto")
            JuskeshinoHardware.moveHead(-0.5,tilt, 10) #move head to the right 
            time.sleep(1.5)
            [obj, img] = JuskeshinoVision.detectAndRecognizeObjectWithoutOrientation(name_obj)
            print("JuskeshinoSimpleTask.->Segundo intento")

            if obj == None: # si no reconocio el objeto
                JuskeshinoHardware.moveHead(0.5,tilt, 10) #move head to the left
                time.sleep(1.5)
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
        
