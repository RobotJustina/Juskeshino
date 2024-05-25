#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import tf2_ros
import numpy as np
import traceback

import std_msgs.msg

from ros_clips.msg import StringArray
from ros_clips.srv import RunPlanning, GetPrintMessage

from gpsr_modules.general_states import MyObject

def get_clips_plan():
    
    #Start Client
    planning_service_name = "/ros_clips/run_planning"
    rospy.wait_for_service(planning_service_name)
    planning_service = rospy.ServiceProxy(planning_service_name, RunPlanning)
    
    _plan = []    
    try:
        steps = 0
        _plan = planning_service(steps).plan.data
    except:
        _plan = []

    return _plan


def load_clips_file(filepath):
    
    pub = rospy.Publisher("/ros_clips/clipspy_load_file", std_msgs.msg.String, queue_size=10)
    rospy.sleep(1)
    
    pub.publish(filepath)
    rospy.Rate(10).sleep()


def load_state_fact(_state):
    
    pub_fact = rospy.Publisher("/ros_clips/clipspy_load_fact", std_msgs.msg.String, queue_size=10)
    rospy.sleep(1)
    
    pub_fact.publish(_state)
    rospy.Rate(10).sleep()


def load_condep_facts(cdp_facts):
    
    pub_fact = rospy.Publisher("/ros_clips/clipspy_load_fact", std_msgs.msg.String, queue_size=10)
    rospy.sleep(1)
    
    n = len(cdp_facts)
    if n > 0:
        for _c in cdp_facts:
            pub_fact.publish(_c)
            rospy.Rate(10).sleep()
        
        _n = "(num-sentences %d)"%n
        pub_fact.publish(_n)
        rospy.Rate(10).sleep()


def retract_condep_facts(cdps):
    
    rospy.wait_for_service("/ros_clips/get_facts")
    get_facts = rospy.ServiceProxy("/ros_clips/get_facts", GetPrintMessage)
    current_facts = get_facts().data.data
    
    pub_retract = rospy.Publisher("/ros_clips/clipspy_retract_fact", std_msgs.msg.Int32, queue_size=10)
    rospy.sleep(1)
    
    if type(cdps) not in (tuple, list):
        cdps = cdps.split()
    
    for cd in current_facts:
        name = cd[:-1].split("(",1)[1].split(None, 1)[0].lower()
        if (name in [x.lower() for x in cdps]):
            idx = int(cd.split()[0])
            pub_retract.publish(idx)
            rospy.Rate(10).sleep()


#( room (type Room) (name nil) (room nil) (zone nil) (center 0 0 0 0) (num 1) )
def load_room_facts(room_info):
    
    rospy.wait_for_service("/ros_clips/get_facts")
    get_facts = rospy.ServiceProxy("/ros_clips/get_facts", GetPrintMessage)
    current_facts = get_facts().data.data
    
    pub_fact = rospy.Publisher("/ros_clips/clipspy_load_fact", std_msgs.msg.String, queue_size=10)
    pub_retract = rospy.Publisher("/ros_clips/clipspy_retract_fact", std_msgs.msg.Int32, queue_size=10)
    rospy.sleep(1)
    
    for _rm in room_info:
        for f in current_facts:
            n = "name %s"%_rm.name
            r = "zone %s"%_rm.location.replace(" ", "_")
            if (n in f) and (r in f):
                idx = int(f.split()[0])
                pub_retract.publish(idx)
                rospy.Rate(10).sleep()
        
        fact = "( room (type Room) (name %s) (room %s) (zone %s) (center %f %f %f %f) )"%( _rm.name,  
                                                                                           _rm.location.replace(" ", "_"),
                                                                                           _rm.name,
                                                                                           _rm.pose[0], _rm.pose[1], _rm.pose[2], _rm.pose[3])
        
        pub_fact.publish(fact)
        rospy.Rate(10).sleep()


#( item (type Objects) (name nil) (room nil) (zone nil) (attributes nil) (lower base) (upper nothing) (possession nobody) (num 1) )
def load_objects_facts(objects_info, room_info):
    
    rospy.wait_for_service("/ros_clips/get_facts")
    get_facts = rospy.ServiceProxy("/ros_clips/get_facts", GetPrintMessage)
    current_facts = get_facts().data.data
    
    pub_fact = rospy.Publisher("/ros_clips/clipspy_load_fact", std_msgs.msg.String, queue_size=10)
    pub_retract = rospy.Publisher("/ros_clips/clipspy_retract_fact", std_msgs.msg.Int32, queue_size=10)
    rospy.sleep(1)
    
    for _o in objects_info:
        for f in current_facts:
            n = "name %s"%_o.name
            m = "num %d"%_o.num
            if (n in f) and (m in f):
                idx = int(f.split()[0])
                pub_retract.publish(idx)
                rospy.Rate(10).sleep()
        
        _loc = None
        loc_name = _o.location.replace(" ", "_")
        for _r in room_info:
            if loc_name == _r.name.replace(" ", "_"):
                _loc = _r
                break
        
        if _loc:
            fact = "( item (type Objects) (name %s) (room %s) (zone %s) (attributes %s) (num %d) )"%( _o.name, 
                                                                                                      _loc.location.replace(" ", "_"),
                                                                                                      _loc.name.replace(" ", "_"),
                                                                                                      _o.info.replace(" ", "_"),
                                                                                                      _o.num)
        else:
            _nil = 'nil'
            fact = "( item (type Objects) (name %s) (room %s) (zone %s) (attributes %s) (num %d) )"%( _o.name, 
                                                                                                      _nil,
                                                                                                      _nil,
                                                                                                      _o.info.replace(" ", "_"),
                                                                                                      _o.num)
        pub_fact.publish(fact)
        rospy.Rate(10).sleep()



#( item (type Furniture) (name nil) (room nil) (zone nil) (attributes nil) (pose 0 0 0) (num 1) )
def load_furniture_facts(furniture_info, room_info):
    
    rospy.wait_for_service("/ros_clips/get_facts")
    get_facts = rospy.ServiceProxy("/ros_clips/get_facts", GetPrintMessage)
    current_facts = get_facts().data.data
    
    pub_fact = rospy.Publisher("/ros_clips/clipspy_load_fact", std_msgs.msg.String, queue_size=10)
    pub_retract = rospy.Publisher("/ros_clips/clipspy_retract_fact", std_msgs.msg.Int32, queue_size=10)
    rospy.sleep(1)
    
    for _fr in furniture_info:
        for f in current_facts:
            n = "name %s"%_fr.name
            m = "num %d"%_fr.num
            if (n in f) and (m in f):
                idx = int(f.split()[0])
                pub_retract.publish(idx)
                rospy.Rate(10).sleep()
        
        _loc = None
        loc_name = _o.location.replace(" ", "_")
        for _r in room_info:
            if loc_name == _fr.name.replace(" ", "_"):
                _loc = _r
                break
        
        if _loc:
            fact = "( item (type Furniture) (name %s) (room %s) (zone %s) (attributes %s) (pose %f %f %f) (num %d) )"%( _fr.name, 
                                                                                                                        _loc.location.replace(" ", "_"),
                                                                                                                        _loc.name.replace(" ", "_"),
                                                                                                                        _fr.info.replace(" ", "_"),
                                                                                                                        _fr.pose[0], _fr.pose[1], _fr.pose[2],
                                                                                                                        _fr.num)
        else:
            _nil = 'nil'
            fact = "( item (type Furniture) (name %s) (room %s) (zone %s) (attributes %s) (pose %f %f %f) (num %d) )"%( _fr.name, 
                                                                                                                        _nil,
                                                                                                                        _nil,
                                                                                                                        _fr.info.replace(" ", "_"),
                                                                                                                        _fr.pose[0], _fr.pose[1], _fr.pose[2],
                                                                                                                        _fr.num)
        pub_fact.publish(fact)
        rospy.Rate(10).sleep()



#( room (type Door) (name nil) (room nil) (zone nil) (center 0 0 0 0) (status nil) (num 1) )
def load_door_facts(door_info, room_info):
    
    rospy.wait_for_service("/ros_clips/get_facts")
    get_facts = rospy.ServiceProxy("/ros_clips/get_facts", GetPrintMessage)
    current_facts = get_facts().data.data
    
    pub_fact = rospy.Publisher("/ros_clips/clipspy_load_fact", std_msgs.msg.String, queue_size=10)
    pub_retract = rospy.Publisher("/ros_clips/clipspy_retract_fact", std_msgs.msg.Int32, queue_size=10)
    rospy.sleep(1)
    
    for _d in door_info:
        for f in current_facts:
            n = "name %s"%_d.name
            m = "num %d"%_d.num
            if (n in f) and (m in f):
                idx = int(f.split()[0])
                pub_retract.publish(idx)
                rospy.Rate(10).sleep()
        
        fact = "( room (type Door) (name %s) (room %s) (zone %s) (center %f %f %f %f) )"%( _rm.name,  
                                                                                           _rm.location.replace(" ", "_"),
                                                                                           _rm.name,
                                                                                           _rm.pose[0], _rm.pose[1], _rm.pose[2], _rm.pose[3])
        
        pub_fact.publish(fact)
        rospy.Rate(10).sleep()



#( item (type Human) (name nil) (room nil) (zone nil) (attributes nil) (objs nil) (num 1) )
def load_human_facts(human_info, room_info):
    
    rospy.wait_for_service("/ros_clips/get_facts")
    get_facts = rospy.ServiceProxy("/ros_clips/get_facts", GetPrintMessage)
    current_facts = get_facts().data.data
    
    pub_fact = rospy.Publisher("/ros_clips/clipspy_load_fact", std_msgs.msg.String, queue_size=10)
    pub_retract = rospy.Publisher("/ros_clips/clipspy_retract_fact", std_msgs.msg.Int32, queue_size=10)
    rospy.sleep(1)
    
    for _h in human_info:
        for f in current_facts:
            n = "name %s"%_h.name
            m = "num %d"%_h.num
            if (n in f) and (m in f):
                idx = int(f.split()[0])
                pub_retract.publish(idx)
                rospy.Rate(10).sleep()
        
        _loc = None
        loc_name = _h.location.replace(" ", "_")
        for _r in room_info:
            if loc_name == _r.name.replace(" ", "_"):
                _loc = _r
                break
        
        if _loc:
            fact = "( item (type Human) (name %s) (room %s) (zone %s) (num %d) )"%( _h.name, 
                                                                                    _loc.location.replace(" ", "_"),
                                                                                    _loc.name.replace(" ", "_"),
                                                                                    _h.num)
        else:
            _nil = 'nil'
            fact = "( item (type Human) (name %s) (room %s) (zone %s) (num %d) )"%( _h.name, 
                                                                                    _nil,
                                                                                    _nil,
                                                                                    _h.num)
        
        pub_fact.publish(fact)
        rospy.Rate(10).sleep()



#( item (type Robot) (name nil) (zone nil) (pose 0 0 0) (grasp nil) (objs nil) (num 1) )
def load_robot_facts(robot_info):
    
    rospy.wait_for_service("/ros_clips/get_facts")
    get_facts = rospy.ServiceProxy("/ros_clips/get_facts", GetPrintMessage)
    current_facts = get_facts().data.data
    
    pub_fact = rospy.Publisher("/ros_clips/clipspy_load_fact", std_msgs.msg.String, queue_size=10)
    pub_retract = rospy.Publisher("/ros_clips/clipspy_retract_fact", std_msgs.msg.Int32, queue_size=10)
    rospy.sleep(1)
    
    for _rb in robot_info:
        for f in current_facts:
            n = "name %s"%_rb.name
            m = "num %d"%_rb.num
            if (n in f) and (m in f):
                idx = int(f.split()[0])
                pub_retract.publish(idx)
                rospy.Rate(10).sleep()
        
        fact = "( item (type Robot) (name %s) (zone %s) (pose %f %f %f) )"%( _rb.name, 
                                                                             _rb.location.replace(" ", "_"), 
                                                                             _rb.pose[0], _rb.pose[1], _rb.pose[2])
        
        pub_fact.publish(fact)
        rospy.Rate(10).sleep()



#(arm (type Arm) (name nil)  (grasp nil) (objs nil) (status nil) (num 1) )
def load_arm_facts(arm_info):
    
    rospy.wait_for_service("/ros_clips/get_facts")
    get_facts = rospy.ServiceProxy("/ros_clips/get_facts", GetPrintMessage)
    current_facts = get_facts().data.data
    
    pub_fact = rospy.Publisher("/ros_clips/clipspy_load_fact", std_msgs.msg.String, queue_size=10)
    pub_retract = rospy.Publisher("/ros_clips/clipspy_retract_fact", std_msgs.msg.Int32, queue_size=10)
    rospy.sleep(1)
    
    for _a in arm_info:
        for f in current_facts:
            n = "name %s"%_a.name
            m = "num %d"%_a.num
            if (n in f) and (m in f):
                idx = int(f.split()[0])
                pub_retract.publish(idx)
                rospy.Rate(10).sleep()
        
        fact = "( arm (type Arm) (name %s) (num %d) )"%( _a.name, 
                                                         _a.num)
        
        pub_fact.publish(fact)
        rospy.Rate(10).sleep()
