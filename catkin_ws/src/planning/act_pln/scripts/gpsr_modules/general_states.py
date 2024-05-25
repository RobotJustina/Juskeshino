#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import smach
import smach_ros
import tf2_ros
import math
import numpy as np
import random
import traceback

import tf
import tf2_ros

import logging

#import hsrb_interface

#from common import speech

#default_tts = speech.DefaultTTS()
#console = speech.Console()
#SAY = default_tts.say

class MyObject:
    def __init__(self, name='', synonyms=[], info = '', location = '', possession = '', num = 1, pose = [0.0, 0.0, 0.0]):
        self.name = name
        self.synonyms = synonyms
        self.info = info
        self.location = location
        self.possession = possession
        self.num = num
        self.pose = pose
    
    def __getattr__(self, name):
        return None


def location_obj(loc_name, room_info):
    
    loc_obj = None
    try:
        flag = False
        #search for exact match
        for _r in room_info:
            for name in _r.synonyms:
                if loc_name == name:
                    loc_obj = _r
                    flag = True
                    break
            if (flag):
                break
        
        #search for closest match
        if not flag:
            for _r in room_info:
                for name in _r.synonyms:
                    if loc_name in name:
                        loc_obj = _r
                        flag = True
                        break
                if (flag):
                    break
    except:
        rospy.logerr(traceback.print_exc())
    
    return loc_obj


def object_obj(obj_name, objects_info):
    
    obj_key = None
    try:
        flag = False
        #search for exact match
        for _obj in objects_info:
            for name in _obj.synonyms:
                if obj_name == name:
                    obj_key = _obj
                    flag = True
                    break
            if (flag):
                break
        
        #search for closest match
        if not flag:
            for _obj in objects_info:
                for name in _obj.synonyms:
                    if obj_name in name:
                        obj_key = _obj
                        flag = True
                        break
                if (flag):
                    break
        #print(obj_key)
    except:
        rospy.logerr(traceback.print_exc())
    
    return obj_key


class FakeState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success'])
    def execute(self, userdata):
            return 'success'


class WaitCommand(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'failure'])
    
    def execute(self, userdata):
        try:
            print("------------------------------------")
            print('WAITING FOR A NEW COMMAND')
            print("------------------------------------")
            
            return 'success'
        except:
            rospy.logerr(traceback.print_exc())
            return 'failure'


class SayString(smach.State):
    def __init__(self, text = 'Hello', wait = True):
        smach.State.__init__(self, outcomes=['success', 'failure'])
        
        self.text = text
        self.wait = wait
    
    def execute(self, userdata):
        try:
            _txt = self.text
            text = self.text.lower()
            is_question = False
            if ("answer" in text and "question" in text):
                _res = ""
                MAX_ERR = 4
                err_count = 0
                _msg = []
                rospy.sleep(2)
                while (not is_question and err_count < MAX_ERR):
                    _msg = rospy.wait_for_message("/speech_recognition/final_result", std_msgs.msg.String, 10)
                    _msg = _msg.data
                    print("Speech message: %s"%_msg)

                    if _msg:
                        is_question = True
                        text = _res.lower()
                    
                    err_count+=1

            if ("name" in text):
                _txt = "Hello, my name is robot."
                
            elif ("highest" in text and "mountain" in text):
                _txt = "The highest mountain in Japan is Mount Fuji"

            elif ("largest" in text and "lake" in text):
                _txt = "The largest lake in Japan is Lake Biwa"

            elif ("coming" in text and "from" in text):
                _txt = "We are from Tamagawa University in Japan"

            elif ("enjoying" in text and "competition" in text):
                _txt = "Yes, we are enjoying RoboCup a lot."

            elif ("favorite" in text and "drink" in text):
                _txt = "My favorite drink is battery."

            elif ("how" in text and "today" in text):
                _txt = "I am feeling great."

            elif ("ride" in text and "elephant" in text):
                _txt = "No, they ride a dinosaur."

            elif ("country" in text and "won" in text):
                _txt = "Japan won the WBC this year."

            elif ("joints" in text and "arm" in text):
                _txt = "My arm has 5 joints."

            elif ("times" in text and "elephant" in text):
                _txt = "Three times five is fifteen."

            elif(is_question):
                _txt = "Sorry. I don't know."
            
            SAY(_txt, self.wait)
            return 'success'
        except:
            rospy.logerr(traceback.print_exc())
            return 'failure'


class MoveRobot(smach.State):
    def __init__(self, robot, omni_base, loc_name = "", tf_buffer=None, timeout=None):
        smach.State.__init__(self, outcomes=['success', 'failure', 'timeout'],
                                   input_keys=['start_time', 'stop_time', 'room_info'])#, 'places_info', 'robot_info'])
        self.robot = robot
        self.omni_base = omni_base
        
        self.loc_name = loc_name
        
        if tf_buffer:
            self.tf_buffer = tf_buffer
        else:
            self.tf_buffer = tf2_ros.Buffer(rospy.Duration(5.))
            tf2_ros.TransformListener(self.tf_buffer)
        
        self.collision_world = robot.get('global_collision_world')
        self.whole_body = self.robot.get('whole_body')
        #self.omni_base = self.robot.get("omni_base")
        self.gripper = self.robot.get('gripper')
        
    def execute(self, userdata):
        try:
            if (not self.loc_name):
                return 'failure'
            
            #loc_key = location_keys(self.loc_name, userdata.places_info)
            _loc = location_obj(self.loc_name, userdata.room_info)
            _pos = _loc.pose
            
            rospy.loginfo("Move to {} at {}, {}, {}".format(
                _loc.name, _pos[0], _pos[1], _pos[3]))
            self.whole_body.move_to_go()
            
            try:
                self.omni_base.go_abs(_pos[0], _pos[1], _pos[3], _NAVTIMEOUT)
            except:
                self.omni_base.go_abs(_pos[0], _pos[1], _pos[3], 0, "pumas")
            
            if userdata.start_time and userdata.stop_time:
                interval = time.time() - userdata.start_time
                if interval > userdata.stop_time:
                    return 'timeout'
            
            return 'success'
            
        except:
            import traceback
            rospy.logerr(traceback.format_exc())
            return 'failure'


class CheckGrasping(smach.State):
    def __init__(self, robot, grasp_thresh = 0.0):
        smach.State.__init__(self, outcomes=['success', 'drop', 'failure'])
        
        self.robot = robot
        self.whole_body = self.robot.get('whole_body')
        self.gripper = self.robot.get('gripper')
        
        self.grasp_thresh = grasp_thresh
        
    def execute(self, userdata):
        try:
            self.whole_body.move_to_go()
            self.gripper.apply_force(1.0)
            
            hand_joint = self.whole_body.joint_positions['hand_motor_joint']
            rospy.loginfo('hand_motor_joint:={}'.format(hand_joint))
            
            if (hand_joint > self.grasp_thresh):
                return 'success'
            
            else:
                return 'drop'
        except:
            rospy.logerr(traceback.print_exc())
            return 'failure'


class CheckPerson(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'failure'],
                                   input_keys=['object_coordinates'])
    
    def execute(self, userdata):
        try:
            
            if userdata.object_coordinates is None:
                return 'failure'
            else:
            	return 'success'
            
        except:
            rospy.logerr(traceback.print_exc())
            return 'failure'


class RotateRobot(smach.State):
    def __init__(self, a_robot, tf_buffer=None, timeout=None):
        smach.State.__init__(self, outcomes=['success', 'failure', 'timeout'],
                                   input_keys=['start_time', 'stop_time', 'rotate_angle'],
                                   output_keys=['rotate_angle'])
        self.a_robot = a_robot
        
        if tf_buffer:
            self.tf_buffer = tf_buffer
        else:
            self.tf_buffer = tf2_ros.Buffer(rospy.Duration(5.))
            tf2_ros.TransformListener(self.tf_buffer)
        
    def execute(self, userdata):
        try:
            if userdata.rotate_angle == 0.5:
                self.a_robot.move_head(userdata.rotate_angle,
                                  0, wait_result=True)
                userdata.rotate_angle = -0.5
            else:
                self.a_robot.move_head(userdata.rotate_angle,
                                  0, wait_result=True)
                userdata.rotate_angle = 0.5
            
            if userdata.start_time and userdata.stop_time:
                interval = time.time() - userdata.start_time
                if interval > userdata.stop_time:
                    return 'timeout'
            
            return 'success'
            
        except:
            import traceback
            rospy.logerr(traceback.format_exc())
            return 'failure'


class Get2Object(smach.State):
    def __init__(self, robot, omni_base, obj_name = "unknown", tf_buffer=None, timeout=None):
        smach.State.__init__(self, outcomes=['success', 'timeout', 'failure'],
                                   input_keys=['start_time', 'stop_time', 'fun_params', 'objects_info', 'room_info'], 
                                   output_keys=['target_object', 'fun_params'])
        
        self.robot = robot
        self.omni_base = omni_base
        
        if tf_buffer:
            self.tf_buffer = tf_buffer
        else:
            self.tf_buffer = tf2_ros.Buffer(rospy.Duration(5.))
            tf2_ros.TransformListener(self.tf_buffer)
        
        self.collision_world = robot.get('global_collision_world')
        self.whole_body = self.robot.get('whole_body')
        #self.omni_base = self.robot.get("omni_base")
        self.gripper = self.robot.get('gripper')
        
        self.obj_name = obj_name
        
    def execute(self, userdata):
        try:
            if (not self.obj_name):
                return 'failure'
            
            _obj = object_obj(self.obj_name, userdata.objects_info)
            if (not _obj):
                return 'failure'
            userdata.target_object = _obj.name
            
            _loc = location_obj(_obj.location, userdata.room_info)
            _pos = _loc.pose
            
            userdata.fun_params.robot_pose.delay = 1.0
            userdata.fun_params.robot_pose.tilt = -0.76
            userdata.fun_params.robot_pose.distance = 0.80
            userdata.fun_params.robot_pose.height_diff = 0.25
            userdata.fun_params.robot_pose.wrist_roll = 0.0
            userdata.fun_params.robot_pose.in_use = True

            userdata.fun_params.edges_bb.position = 0.80
            userdata.fun_params.edges_bb.orientation = 0.0001
            userdata.fun_params.edges_bb.maxx = 0.40
            userdata.fun_params.edges_bb.miny = _pos[2] - 0.10 #0.10 #_pos.z - 0.10
            userdata.fun_params.edges_bb.maxy = _pos[2] + 0.10 #1.00 #_pos.z + 0.10
            userdata.fun_params.edges_bb.maxz = 1.20
            userdata.fun_params.edges_bb.in_use = True

            userdata.fun_params.objects_bb.depth_min = 0.40
            userdata.fun_params.objects_bb.depth_max = 1.20
            userdata.fun_params.objects_bb.width = 0.80
            userdata.fun_params.objects_bb.height_min = _pos[2] #0.10 #_pos.z
            userdata.fun_params.objects_bb.height_max = _pos[2] + 0.40 #1.00 #_pos.z + 0.40
            userdata.fun_params.objects_bb.in_use = True
            
            if "floor" in _obj.location:
                userdata.fun_params.robot_pose.tilt = -0.96

                userdata.fun_params.objects_bb.depth_max = 1.20
                userdata.fun_params.objects_bb.width = 0.80
                userdata.fun_params.objects_bb.height_min = 0.0001
                userdata.fun_params.objects_bb.height_max = 0.25
            
            rospy.loginfo("Move to {} at {}, {}, {}".format(
                _obj.location, _pos[0], _pos[1], _pos[3]))
            
            self.whole_body.move_to_go()
            
            try:
                self.omni_base.go_abs(_pos[0], _pos[1], _pos[3], _NAVTIMEOUT)
            except:
                self.omni_base.go_abs(_pos[0], _pos[1], _pos[3], 0, "pumas")
            
            if userdata.start_time and userdata.stop_time:
                interval = time.time() - userdata.start_time
                if interval > userdata.stop_time:
                    return 'timeout'
            
            return 'success'
            
        except:
            import traceback
            rospy.logerr(traceback.format_exc())
            return 'failure'


class Get2Deposit(smach.State):
    def __init__(self, robot, omni_base, place_name = "unknown", tf_buffer=None, timeout=None):
        smach.State.__init__(self, outcomes=['success', 'drawer', 'container', 'tray', 'basket', 'timeout', 'failure'],
                                   input_keys=['start_time', 'stop_time', 'fun_params', 'room_info'], 
                                   output_keys=['fun_params'])
        
        self.robot = robot
        self.omni_base = omni_base
        
        if tf_buffer:
            self.tf_buffer = tf_buffer
        else:
            self.tf_buffer = tf2_ros.Buffer(rospy.Duration(5.))
            tf2_ros.TransformListener(self.tf_buffer)
        
        self.collision_world = robot.get('global_collision_world')
        self.whole_body = self.robot.get('whole_body')
        #self.omni_base = self.robot.get("omni_base")
        self.gripper = self.robot.get('gripper')
        
        self.place_name = place_name
        
    def execute(self, userdata):
        try:
            if (not self.place_name):
                return 'failure'
            
            _loc = location_obj(self.place_name, userdata.room_info)
            _pos = _loc.pose
            
            userdata.fun_params.robot_pose.delay = 1.0
            userdata.fun_params.robot_pose.tilt = -0.76
            userdata.fun_params.robot_pose.distance = 0.80
            userdata.fun_params.robot_pose.height_diff = 0.25
            userdata.fun_params.robot_pose.wrist_roll = 0.0
            userdata.fun_params.robot_pose.in_use = True

            userdata.fun_params.edges_bb.position = 0.80
            userdata.fun_params.edges_bb.orientation = 0.0001
            userdata.fun_params.edges_bb.maxx = 0.40
            userdata.fun_params.edges_bb.miny = _pos[2] - 0.10
            userdata.fun_params.edges_bb.maxy = _pos[2] + 0.10
            userdata.fun_params.edges_bb.maxz = 1.20
            userdata.fun_params.edges_bb.in_use = True

            userdata.fun_params.points_bb.maxx = 0.40
            userdata.fun_params.points_bb.miny = _pos[2] - 0.10
            userdata.fun_params.points_bb.maxy = _pos[2] + 0.10
            userdata.fun_params.points_bb.maxz = 1.20
            userdata.fun_params.points_bb.in_use = True

            userdata.fun_params.objects_bb.depth_min = 0.40
            userdata.fun_params.objects_bb.depth_max = 1.20
            userdata.fun_params.objects_bb.width = 0.80
            userdata.fun_params.objects_bb.height_min = _pos[2]
            userdata.fun_params.objects_bb.height_max = _pos[2] + 0.40
            userdata.fun_params.objects_bb.in_use = True

            userdata.fun_params.space_info.dist2plane = 0.08
            userdata.fun_params.space_info.wside = 0.10
            userdata.fun_params.space_info.dside = 0.12
            userdata.fun_params.space_info.delta_lift = 0.10
            userdata.fun_params.space_info.side = 0
            userdata.fun_params.space_info.in_use = True

            location = _loc.name
            rospy.logerr(_loc.name)
            
            if any([x in _loc.name for x in ["drawer"]]):
                userdata.fun_params.robot_pose.tilt = -0.78
                userdata.fun_params.edges_bb.maxx = 0.40
                userdata.fun_params.edges_bb.miny = _pos[2] - 0.20
                userdata.fun_params.edges_bb.maxy = _pos[2]
                userdata.fun_params.points_bb.maxx = 0.40
                userdata.fun_params.points_bb.miny = _pos[2] - 0.20
                userdata.fun_params.points_bb.maxy = _pos[2]
                area = 'drawer'
            
            elif any([x in _loc.name for x in ["container"]]):
                userdata.fun_params.robot_pose.tilt = -0.78
                userdata.fun_params.edges_bb.orientation = []
                userdata.fun_params.edges_bb.maxx = 0.15
                userdata.fun_params.edges_bb.miny = _pos[2] - 0.05
                userdata.fun_params.edges_bb.maxy = _pos[2]
                userdata.fun_params.points_bb.maxx = 0.15
                userdata.fun_params.points_bb.miny = _pos[2] - 0.05
                userdata.fun_params.points_bb.maxy = _pos[2]
                area = 'container'
            
            elif any([x in _loc.name for x in ["tray", "table"]]):
                userdata.fun_params.robot_pose.tilt = -0.78
                userdata.fun_params.edges_bb.maxx = 0.15
                userdata.fun_params.edges_bb.miny = _pos[2] - 0.10
                userdata.fun_params.edges_bb.maxy = _pos[2] + 10
                userdata.fun_params.objects_bb.width = 0.40
                userdata.fun_params.objects_bb.height_max = 0.60
                area = 'tray'

            elif any([x in _loc.name for x in ["basket"]]):
                userdata.fun_params.robot_pose.tilt = -0.78
                userdata.fun_params.edges_bb.orientation = []
                userdata.fun_params.edges_bb.maxx = 0.15
                userdata.fun_params.edges_bb.miny = _pos[2] - 0.05
                userdata.fun_params.edges_bb.maxy = _pos[2] + 0.05
                userdata.fun_params.points_bb.maxx = 0.15
                userdata.fun_params.points_bb.miny = _pos[2] - 0.05
                userdata.fun_params.points_bb.maxy = _pos[2] + 0.05
                area = 'basket'

            else:
                _loc = location_obj("deposit_unknown", userdata.room_info)
                _pos = _loc.pose
                
                userdata.fun_params.robot_pose.tilt = -0.78
                userdata.fun_params.edges_bb.orientation = []
                userdata.fun_params.edges_bb.maxx = 0.15
                userdata.fun_params.edges_bb.miny = _pos[2] - 0.05
                userdata.fun_params.edges_bb.maxy = _pos[2] + 0.05
                userdata.fun_params.points_bb.maxx = 0.15
                userdata.fun_params.points_bb.miny = _pos[2] - 0.05
                userdata.fun_params.points_bb.maxy = _pos[2] + 0.05
                userdata.hand_pose = bin_b_pose
                area = 'basket'
            
            rospy.loginfo("Move to {} at {}, {}, {}".format(
                _loc.name, _pos[0], _pos[1], _pos[3]))
            
            self.whole_body.move_to_go()
            
            try:
                self.omni_base.go_abs(_pos[0], _pos[1], _pos[3], _NAVTIMEOUT)
            except:
                self.omni_base.go_abs(_pos[0], _pos[1], _pos[3], 0, "pumas")
            
            if userdata.start_time and userdata.stop_time:
                interval = time.time() - userdata.start_time
                if interval > userdata.stop_time:
                    return 'timeout'
            
            return area
            
        except:
            import traceback
            rospy.logerr(traceback.format_exc())
            return 'failure'

