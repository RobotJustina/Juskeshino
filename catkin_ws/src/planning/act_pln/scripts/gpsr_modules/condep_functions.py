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
import re

import speech_recognition as sr

import cv2
from cv_bridge import CvBridge, CvBridgeError
bridge = CvBridge()

import shlex
from psutil import Popen

import tf
import tf2_ros

import logging

import hsrb_interface
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Bool

from common import speech

#eRasers states
from common.hand_object import HandObject
from common.wait_hand_pushed import wait_hand_pushed
#from human_position import DetectPerson

#CONCEPTUAL DEPENDENCIES
from conceptual_deps.msg import StringArray
from ros_whisper_vosk.srv import GetSpeech
from conceptual_deps.srv import GetConDep, GetTextConDep
from gpsr_modules.clips_functions import get_clips_plan, load_state_fact, load_condep_facts, retract_condep_facts

import std_msgs.msg, sensor_msgs.msg

default_tts = speech.DefaultTTS()
console = speech.Console()
SAY = default_tts.say


_OPENIA = True
_INSTRUCTIONPOINT = "instruction_point"

####YES-NO RECOGNISER
global response
def speech_callback(msg):
    global response
    print ("Callback: %s"%msg.data)
    response = msg.data

global qr_response
def qr_callback(msg):
    global qr_response
    
    try:
        cv_image = bridge.imgmsg_to_cv2(msg, "passthrough")
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))
    
    qcd = cv2.QRCodeDetector()
    ret_qr, decoded_info, points, _ = qcd.detectAndDecodeMulti(cv_image)
    
    if ret_qr:
        qr_response = decoded_info[0]
    else:
        qr_response = []

def recognize_speech_from_mic():

    global response    
    response = []
    rospy.Subscriber("/speech_recognition/final_result", std_msgs.msg.String, speech_callback)
    response = []
    while (not response):
        rospy.Rate(10).sleep()
    
    return response

def recognize_speech_from_qr():

    global qr_response    
    qr_response = []
    rospy.Subscriber("/hsrb/head_rgbd_sensor/rgb/image_raw", sensor_msgs.msg.Image, qr_callback)
    
    while (not qr_response):
        rospy.Rate(10).sleep()
    
    return qr_response

def yes_no_question():
    _res = ""
    MAX_ERR = 4
    err_count = 0
    ask_yesno = True
    answer_yes = ['yes', 'yup', 'right', 'alright', 'exactly', 'ok', 'okay', 'sure', 'of course', 'definitely', 'jess', 'just']
    answer_no = ['no', 'nope', 'not', 'never', 'nix', 'nay', 'nah', 'negative', 'veto']
    
    _s = []
    rospy.sleep(2)
    while (ask_yesno and err_count < MAX_ERR):
        #_s = recognize_speech_from_mic()
        _s = rospy.wait_for_message("/speech_recognition/final_result", std_msgs.msg.String, 10)
        _s = _s.data
        print("Speech message: %s"%_s)
        _s = _s.lower().split()
        for _a in _s:
            if (_a in answer_yes):
                _res = "yes"
                ask_yesno = False
                break
            elif (_a in answer_no):
                _res = "no"
                ask_yesno = False
                break
        err_count+=1
    
    return _res

def open_question():
    _res = ""
    MAX_ERR = 4
    err_count = 0
    is_answer = False
    _res = []
    rospy.sleep(2)
    while (not is_answer and err_count < MAX_ERR):
        #_res = recognize_speech_from_mic()
        _res = rospy.wait_for_message("/speech_recognition/final_result", std_msgs.msg.String, 10)
        _res = _res.data
        print("Speech message: %s"%_res)

        if _res:
            is_answer = True
            _res = _res.lower()
        
        err_count+=1
    
    return _res

####SPEECH TO CONCEPTUAL DEPENDENCY
def info_keyname(name, _info):
    
    keyname = None
    try:
        flag = False
        #search for exact match
        for _obj in _info:
            for _n in _obj.synonyms:
                if name == _n:
                    keyname = _obj.name
                    flag = True
                    break
            if (flag):
                break
        
        #find closest word
        if not flag:
            for _obj in _info:
                for _n in _obj.synonyms:
                    if name in _n:
                        keyname = _obj.name
                        flag = True
                        break
                if (flag):
                    break
    
    except:
        rospy.logerr(traceback.print_exc())
    
    return keyname

def find_word_in_string(_str, _info):
    
    _word = None
    try:
        flag = False
        for _obj in _info:
            for _n in _obj.synonyms:
                if _str in _n or _n in _str:
                    _word = _obj.name
                    flag = True
                    break
            if (flag):
                break
    
    except:
        rospy.logerr(traceback.print_exc())
    
    return _word

def get_synonym(name, _info):
    
    _syn = None
    try:
        flag = False
        for _obj in _info:
            if name == _obj.name:
                if len(_obj.synonyms) > 0:
                    _syn = _obj.synonyms[1]
                else:
                    _syn = _obj.synonyms[0]
                flag = True
                break
        
        if not flag:
            _syn = name
    
    except:
        rospy.logerr(traceback.print_exc())
    
    return _syn

def get_commands(max_retrial=3):

    print("------------------------------------")
    print('GETTING COMMANDS FROM SPEECH')
    print("------------------------------------")
    
    #Start Client
    node_process = Popen( shlex.split('rosrun ros_whisper_vosk whisper_service.py --english') )
    rospy.sleep(10)
        
    speech_text = []
    if (max_retrial > 0):
        SAY("Tell me your order. Please wait four seconds to speak.")
        #rospy.sleep(1)

        speech_service_name = "speech_recognition/whisper_service"
        rospy.wait_for_service(speech_service_name)
        speech_service = rospy.ServiceProxy(speech_service_name, GetSpeech)
        
        #condep_service_name = "conceptual_dependencies/condep_service"
        #rospy.wait_for_service(condep_service_name)
        #condep_service = rospy.ServiceProxy(condep_service_name, GetConDep)
        
        counter = 0
        #open_mic_windows_pub = rospy.Publisher('open_mic_window', Bool, queue_size=1)
        
        rospy.sleep(3)
        while counter < max_retrial:
            #condeps = []
            #open_mic_windows_pub.publish(Bool(True))
            try:
                #condeps = speech_condep_service().cds.data
                speech_text = speech_service().data
                SAY(speech_text)
            except:
                speech_text = [] #condeps = []
            
            SAY("Push my hand if this is correct.")
            if wait_hand_pushed(_timeout=5):
                node_process.terminate()
                break #return condeps
            else:
                SAY("Sorry. Can you repeat your order?")
            
            speech_text = []
            counter += 1
        
        node_process.terminate()
        rospy.sleep(1)
    
    if (not speech_text):
        is_qr = False
        speech_text = []
        SAY("Can you give me a QR code? Please, show me the code in front of my top camera, where I can see it.")
        while not is_qr:
            speech_text = recognize_speech_from_qr()
            if(speech_text):
                is_qr = True
    
        SAY(speech_text)
    
    SAY("OK. I will process your command. Please wait.", wait_result=False)

    text_result = speech_text

    is_rephrase = True
    res = text_result.split()
    if res[0].lower() in ["what", "where", "who"]:
        is_rephrase = False
    
    if(_OPENIA and is_rephrase):
        import openai
        openai.api_key = "TODO"
        
        try:
            response = openai.ChatCompletion.create(
                model="gpt-3.5-turbo",
                messages=[
                        {"role": "system", "content": "You are a chatbot"},
                        {"role": "user", "content": "Suppose you have a robot that can only understand the following commands: 'go', 'guide', 'give', 'deliver', 'take', 'find', 'meet', 'deposit', 'open', 'close', 'say', 'tell', 'remind', 'follow', 'who', 'where', and 'what', and follows the structure of 'go to [location]', 'guide [person] from [origin] to [destination]', 'give [object] to [person]', 'deliver [object] to [location]', 'find [object] at [location]', 'meet [person] at [location]', 'deposit [object] to [place]', 'open [target]', 'close [target]', 'say [message]' to [person], 'tell [message] to [person]', 'remind [message] from [source] to [target]', 'follow [person] from [origin] to [destination]'. For exmple, 'Find Luis and ask him what time is it' can be rephrased as 'Find Luis and say what time is it'. The command 'answer' should be rephrased as 'say' and repeat the question without actually answering it, for example, 'Find Luis and answer his question' can be rephrased as 'Find Luis and say I will answer your question' or 'Answer what time is it?' becomes 'Say what time is it'. One more example is receiving 'Follow Max' then, the output should be 'Follow Max' or 'Find Paola and give her an apple' you should reply 'Find Paola and give an apple to Paola'. The nouns 'anyone' or 'person' become a proper noun, for example 'Find a person and follow him' becomes 'Find person and follow person'. Similarly, generic nouns like 'boy' or 'girl' becomes 'person', for example 'Find a boy' becomes 'Find a person'. Now, suppose that a user provides the following instruction '%s'. Rephrase that instruction using only the commands that the robot can understand. Do not use any verb that the robot can not understand. Use the simplest expression without ambiguity or redundant instructions. Discard the information that does not fit in the structures. Replace personal pronouns with the correct proper noun. Please, reply with only one single sentence without the reasoning or any additional information so the robot can read it directly."%(speech_text)},
            ]
            )
        
            text_result = ''
            for choice in response.choices:
                text_result += choice.message.content
        
            print(text_result)
        except:
            text_result = speech_text

    #Start Conceptual Dependencies Client
    text_condep_service_name = "conceptual_dependencies/text_condep_service"
    rospy.wait_for_service(text_condep_service_name)
    text_condep_service = rospy.ServiceProxy(text_condep_service_name, GetTextConDep)
    
    condeps = []    
    try:
        condeps = text_condep_service(text_result).cds.data
    except:
        condeps = []
    
    rospy.sleep(1)
    return condeps


def get_text_commands(text):
    
    SAY(text)
    node_process = Popen( shlex.split('rosrun ros_whisper_vosk whisper_service.py --english') )
    rospy.sleep(5)
    
    print("------------------------------------")
    print('GETTING COMMANDS FROM SPEECH')
    print("------------------------------------")
    
    #Start Client
    text_service_name = "conceptual_dependencies/text_condep_service"
    rospy.wait_for_service(text_service_name)
    text_condep_service = rospy.ServiceProxy(text_service_name, GetTextConDep)
    
    condeps = []    
    try:
        condeps = text_condep_service(text).cds.data
    except:
        condeps = []
    
    node_process.terminate()
    rospy.sleep(1)
    
    return condeps


def get_plan(_condeps, _info = []):
    
    #Create con_deps templates
    cdp_facts = []
    for s in _condeps:
        _s = s[:-1].split("(",1)
        res = re.findall(r'\(.*?\)', _s[1])
        
        for i in range(len(res)):
            res[i] = re.sub( r'[()]', '', res[i] )       
        
        if _s[0].lower() in ['qtrans', 'ptrans', 'attend', 'atrans', 'grab', 'release', 'speak', 'mtrans', 'propel', 'ftrans']:
            _cdp = "(%s "%_s[0].lower()
            
            for i in range(len(res)):
                _field = res[i].split(None, 1)[0].lower()
                
                #([multifield] answer nil))
                #([multifield] msg nil)
                if _field not in ['question', 'answer', 'msg']:
                    _value = res[i].split(None, 1)[1].lower()
                    _value = _value.replace('a ', '')
                    _value = _value.replace('an ', '')
                    _value = _value.replace('the ', '')
                    _value = _value.replace('some ', '')
                    if _info:
                        _v = info_keyname(_value, _info)
                        if _v:
                            _value = _v
                        else:
                            _value = 'nil' #_value.replace(" ", "_")
                    else:
                        _value = _value.replace(" ", "_")

                else:
                    _value = res[i].split(None, 1)[1].lower()
                
                _c = "(%s %s)"%(_field, _value)
                _cdp = _cdp + _c
            _cdp = _cdp + ")"
        else:
            break
        
        cdp_facts.append(_cdp)
    
    load_condep_facts(cdp_facts)
    _p = get_clips_plan()
    
    return _p


def get_full_plan(condeps, _info = []):
    
    _condeps = condeps
    p = []
    is_plan = True
    while (_condeps):
        for i in range(len(_condeps), 0, -1):
            #Retract current conceptual dependencies facts
            cp_names = ['qtrans', 'ptrans', 'attend', 'atrans', 'grab', 'release', 'speak', 'mtrans', 'propel', 'ftrans']
            retract_condep_facts(cp_names)
            
            sub_cdp = _condeps[:i]
            tmp_cdp = _condeps[i:]
            
            _p = get_plan(sub_cdp, _info)
            
            if (_p):
                _condeps = tmp_cdp
                break
        
        if not _p:
            is_plan = False
            break
        p = p + _p
    
    if is_plan:
        return p
    else:
        print("No plan found!")
        p = []
        return(p)


def get_partial_plan(condeps, _info = []):
    
    _condeps = condeps
    p = []
    is_plan = True
    while (_condeps):
        for i in range(len(_condeps), 0, -1):
            #Retract current conceptual dependencies facts
            cp_names = ['qtrans', 'ptrans', 'attend', 'atrans', 'grab', 'release', 'speak', 'mtrans', 'propel', 'ftrans']
            retract_condep_facts(cp_names)
            
            sub_cdp = _condeps[:i]
            tmp_cdp = _condeps[i:]
            
            _p = get_plan(sub_cdp, _info)
            
            if (_p):
                _condeps = tmp_cdp
                break
        
        if _p:
            p = p + _p
        else:
            _condeps = _condeps[1:]

    
    if p:
        return p
    else:
        print("No plan found!")
        p = []
        return(p)

def generate_smach_states(_plan = []):
    #BUILD STATES
    idx = 0
    _cmd = ""
    _cmd_list = []
    for p in _plan:
        _p = p[:-1].split("(",1)
        res = re.findall(r'\(.*?\)', _p[1])
        
        for i in range(len(res)):
            res[i] = re.sub( r'[()]', '', res[i] )       
        
        action = None
        for i in range(len(res)):
            _a = res[i].split(None, 1)[0]
            if _a.lower() == 'actions':
                action = res[i].split(None, 1)[1]
                break
        if action:
            _a = action.split() 
            next_state = "STATE_%d"% (idx+1)
            #if(idx+1 == len(_plan)):
            #    next_state = "success"
            
            if _a[0] == 'goto':
                #str_plan = str_plan + " I will go to " + _a[2] + " in " + _a[1] + "."
                
                loc_name = _a[2]
                loc_name = loc_name.lower().replace('the ', '')
                cmd_state = """
    smach.StateMachine.add('STATE_%d', FakeState(),
                           transitions={'success': 'STATE_%d_MOVEROBOT'})
    
    smach.StateMachine.add('STATE_%d_MOVEROBOT', MoveRobot(robot, omni_base, loc_name="%s"),
                           transitions={'success': '%s',
                                        'timeout': 'STATE_%d_MOVEROBOT',
                                        'failure': 'failure'})
""" % (idx, idx,
       idx, loc_name, next_state, idx)

            elif _a[0] == 'guide-human':
                #str_plan = str_plan + " I will guide you to " + _a[2] + "."
                
                loc_name = _a[2]
                loc_name = loc_name.lower().replace('the ', '')
                cmd_state = """
    smach.StateMachine.add('STATE_%d', FakeState(),
                           transitions={'success': 'STATE_%d_SAYSTRING'})
    
    smach.StateMachine.add('STATE_%d_SAYSTRING', SayString(text="I will take you to %s. Please follow my lead."),
                           transitions={'success': 'STATE_%d_FAKE_GUIDE',
                                        'failure': 'STATE_%d_SAYSTRING'})

    smach.StateMachine.add('STATE_%d_FAKE_GUIDE', MoveRobot(robot, omni_base, loc_name="%s"),
                           transitions={'success': '%s',
                                        'timeout': 'STATE_%d_FAKE_GUIDE',
                                        'failure': 'failure'})
""" % (idx, idx,
       idx, loc_name, idx, idx,
       idx, loc_name, next_state, idx)

            elif _a[0] == 'find-object':
                #str_plan = str_plan + " I will find " + _a[1] + "."
                _target = _a[1]
                cmd_state = """
    smach.StateMachine.add('STATE_%d', FakeState(),
                           transitions={'success': 'STATE_%d_GET2OBJECT'})
    
    smach.StateMachine.add('STATE_%d_GET2OBJECT', Get2Object(robot, omni_base, obj_name = "%s"),
                           transitions={'success': 'STATE_%d_SAYSTRING',
                                        'timeout': 'STATE_%d_GET2OBJECT',
                                        'failure': 'failure'})
    
    smach.StateMachine.add('STATE_%d_SAYSTRING', SayString(text="I will find %s."),
                           transitions={'success': 'STATE_%d_DETECTOBJECT',
                                        'failure': 'STATE_%d_SAYSTRING'})

    smach.StateMachine.add('STATE_%d_DETECTOBJECT', GraspObjectTF(robot, "tip-adapter", neutral=True, find_obj=True, take_obj=False, back=True),
                           transitions={'success': '%s',
                                        'unknown': 'STATE_%d_GET2OBJECT',
                                        'timeout': 'STATE_%d_GET2OBJECT',
                                        'failure': 'failure'})
""" % (idx, idx,
       idx, _target, idx, idx,
       idx, _target, idx, idx,
       idx, next_state, idx, idx)
            
            elif _a[0] == 'take-object':
                #str_plan = str_plan + " I will take " + _a[1] + "."
                _target = _a[1]
                cmd_state = """
    smach.StateMachine.add('STATE_%d', FakeState(),
                           transitions={'success': 'STATE_%d_GET2OBJECT'})
    
    smach.StateMachine.add('STATE_%d_GET2OBJECT', Get2Object(robot, omni_base, obj_name = "%s"),
                           transitions={'success': 'STATE_%d_DETECTOBJECT',
                                        'timeout': 'STATE_%d_GET2OBJECT',
                                        'failure': 'failure'})
    
    smach.StateMachine.add('STATE_%d_DETECTOBJECT', GraspObjectTF(robot, "tip-adapter", neutral=True, find_obj=True, back=True),
                           transitions={'success': 'STATE_%d_CHECKGRASPING',
                                        'unknown': 'STATE_%d_CHECKGRASPING',
                                        'timeout': 'STATE_%d_CHECKGRASPING',
                                        'failure': 'STATE_%d_CHECKGRASPING'})
    
    smach.StateMachine.add('STATE_%d_CHECKGRASPING', CheckGrasping(robot, grasp_thresh=GRASP_THRESHOLD),
                       transitions={'success': '%s',
                                    'drop': 'STATE_%d',
                                    'failure': 'failure'})
""" % (idx, idx,
       idx, _target, idx, idx,
       idx, idx, idx, idx, idx,
       idx, next_state, idx)

            
            elif _a[0] == 'find-human':
                #str_plan = str_plan + " I will find " + _a[1] + "."
                cmd_state = """
    smach.StateMachine.add('STATE_%d', FakeState(),
                           transitions={'success': 'STATE_%d_KILLYOLO'})
    
    smach.StateMachine.add('STATE_%d_KILLYOLO', KillYOLO(is_yolo=_STARTYOLO),
                           transitions={'success': 'STATE_%d_SAYSTRING',
                                        'failure': 'STATE_%d_SAYSTRING'})
    
    smach.StateMachine.add('STATE_%d_SAYSTRING', SayString(text="Hello %s, please raise your hand."),
                           transitions={'success': 'STATE_%d_DETECTPERSON',
                                        'failure': 'STATE_%d_SAYSTRING'})
    
    smach.StateMachine.add('STATE_%d_DETECTPERSON', DetectPerson(robot),
                           transitions={'success': 'STATE_%d_SAYSTRING2',
                                        'timeout': 'STATE_%d_ROTATEROBOT',
                                        'failure': 'STATE_%d_DETECTPERSON'})
    
    smach.StateMachine.add('STATE_%d_ROTATEROBOT', RotateRobot(a_robot),
                           transitions={'success': 'STATE_%d_DETECTPERSON',
                                        'timeout': 'failure',
                                        'failure': 'STATE_%d_DETECTPERSON'})
    
    smach.StateMachine.add('STATE_%d_SAYSTRING2', SayString(text="I found you, %s!"),
                           transitions={'success': 'STATE_%d_APPROACHPERSON',
                                        'failure': 'STATE_%d_SAYSTRING2'})
    
    smach.StateMachine.add('STATE_%d_APPROACHPERSON', ApproachPerson(robot, say_fn=SAY,
                                                       tf_buffer=robot._get_tf2_buffer()),
                           transitions={'success': 'STATE_%d_RESTARTYOLO',
                                        'failure': 'STATE_%d_RESTARTYOLO'},
                           remapping={'person': 'human_coordinate'})
    
    smach.StateMachine.add('STATE_%d_RESTARTYOLO', RestartYOLO(is_yolo=_STARTYOLO),
                           transitions={'success': '%s',
                                        'failure': 'failure'})
""" % (idx, idx,
       idx, idx, idx, 
       idx, _a[1], idx, idx,
       idx, idx, idx, idx,
       idx, idx, idx,
       idx, _a[1], idx, idx,
       idx, idx, idx,
       idx, next_state)
            
            elif _a[0] == 'deliver-object':
                #str_plan = str_plan + " I will deliver the object to " + _a[1] + "."
                cmd_state = """
    smach.StateMachine.add('STATE_%d', FakeState(),
                           transitions={'success': 'STATE_%d_CHECKPERSON'})
    
    smach.StateMachine.add('STATE_%d_CHECKPERSON', CheckPerson(),
                           transitions={'success': 'STATE_%d_HANDOBJECT',
                                        'failure': 'STATE_%d_KILLYOLO'})
    
    smach.StateMachine.add('STATE_%d_KILLYOLO', KillYOLO(is_yolo=_STARTYOLO),
                           transitions={'success': 'STATE_%d_SAYSTRING',
                                        'failure': 'STATE_%d_SAYSTRING'})
    
    smach.StateMachine.add('STATE_%d_SAYSTRING', SayString(text="Hello %s, please raise your hand."),
                           transitions={'success': 'STATE_%d_DETECTPERSON',
                                        'failure': 'STATE_%d_SAYSTRING'})
    
    smach.StateMachine.add('STATE_%d_DETECTPERSON', DetectPerson(robot),
                           transitions={'success': 'STATE_%d_HANDOBJECT',
                                        'timeout': 'STATE_%d_ROTATEROBOT',
                                        'failure': 'STATE_%d_DETECTPERSON'})
    
    smach.StateMachine.add('STATE_%d_ROTATEROBOT', RotateRobot(a_robot),
                           transitions={'success': 'STATE_%d_DETECTPERSON',
                                        'timeout': 'failure',
                                        'failure': 'STATE_%d_DETECTPERSON'})
    
    smach.StateMachine.add('STATE_%d_HANDOBJECT', HandObject(robot, say_fn=SAY,
                                                   start_msg="Here you are. Pull the object, please.",
                                                   tf_buffer=robot._get_tf2_buffer()),
                           transitions={'success': 'STATE_%d_RESTARTYOLO',
                                        'timeout': 'STATE_%d_HANDOBJECT',
                                        'failure': 'STATE_%d_HANDOBJECT'},
                           remapping={'person': 'human_coordinate'})

    smach.StateMachine.add('STATE_%d_RESTARTYOLO', RestartYOLO(is_yolo=_STARTYOLO),
                           transitions={'success': '%s',
                                        'failure': 'failure'})
""" % (idx, idx,
       idx, idx, idx,
       idx, idx, idx,
       idx, _a[1], idx, idx,
       idx, idx, idx, idx,
       idx, idx, idx,
       idx, idx, idx, idx,
       idx, next_state)
            
            elif _a[0] == 'find-space':
                #str_plan = str_plan + " I will deposit the object in " + _a[1] + "."
                #['success', 'drawer', 'container', 'tray', 'basket', 'timeout', 'failure']
                cmd_state = """
    smach.StateMachine.add('STATE_%d', FakeState(),
                           transitions={'success': 'STATE_%d_GET2DEPOSIT'})
    
    smach.StateMachine.add('STATE_%d_GET2DEPOSIT', Get2Deposit(robot, omni_base, place_name = "%s"),
                           transitions={'drawer':    'STATE_%d_DEPOSIT2DRAWER',
                                        'container': 'STATE_%d_DEPOSIT2CONTAINER',
                                        'tray':      'STATE_%d_DEPOSIT2TRAY',
                                        'basket':    'STATE_%d_DEPOSIT2BASKET',
                                        'timeout':   'failure',
                                        'failure':   'failure'})
    
    smach.StateMachine.add('STATE_%d_DEPOSIT2DRAWER', FindPointTF(robot, drift_x=0.15, drift_y = 0.05, drift_z=0.175, delta_flex=-0.78),
                           transitions={'success': '%s',
                                        'unknown': 'STATE_%d_OBJECTINHAND',
                                        'timeout': 'failure',
                                        'failure': 'STATE_%d_OBJECTINHAND'})
    
    smach.StateMachine.add('STATE_%d_DEPOSIT2CONTAINER', FindPointTF(robot, drift_x=0.10, drift_z=0.10, delta_flex=-0.78),
                           transitions={'success': '%s',
                                        'unknown': 'STATE_%d_OBJECTINHAND',
                                        'timeout': 'failure',
                                        'failure': 'STATE_%d_OBJECTINHAND'})
    
    smach.StateMachine.add('STATE_%d_DEPOSIT2TRAY', DetectSpaceOnPlaneTF(robot, drift_x=-0.05, drift_z=0.075, back=True),
                           transitions={'success': '%s',
                                        'unknown': 'STATE_%d_OBJECTINHAND',
                                        'timeout': 'failure',
                                        'failure': 'STATE_%d_OBJECTINHAND'})
    
    smach.StateMachine.add('STATE_%d_DEPOSIT2BASKET', FindPointTF(robot, drift_x=0.15, drift_z=0.10, delta_flex=-0.78),
                           transitions={'success': '%s',
                                        'unknown': 'STATE_%d_OBJECTINHAND',
                                        'timeout': 'failure',
                                        'failure': 'STATE_%d_OBJECTINHAND'})
    
    smach.StateMachine.add('STATE_%d_OBJECTINHAND', CheckGrasping(robot, grasp_thresh=GRASP_THRESHOLD),
                       transitions={'success': 'STATE_%d_GET2DEPOSIT',
                                    'drop': '%s',
                                    'failure': 'failure'})
""" % (idx, idx,
       idx, _a[1], idx, idx, idx, idx,
       idx, next_state, idx, idx,
       idx, next_state, idx, idx,
       idx, next_state, idx, idx,
       idx, next_state, idx, idx,
       idx, idx, next_state)
            
            elif _a[0] == 'say-string':
                text = re.findall('\[(.+)\]', action)
                #str_plan = str_plan + " I will say your sentence."
                cmd_state = """
    smach.StateMachine.add('STATE_%d', FakeState(),
                           transitions={'success': 'STATE_%d_SAYSTRING'})
    
    smach.StateMachine.add('STATE_%d_SAYSTRING', SayString(text="Hello. I have a message for you. %s."),
                           transitions={'success': '%s',
                                        'failure': 'STATE_%d_SAYSTRING'})
""" % (idx, idx,
       idx, text[0], next_state, idx)
            
            elif _a[0] == 'propel':
                act = _a[1]
                if act == 'nil':
                    act = "do nothing to"
                #str_plan = str_plan + " I will " + act + " " + _a[2] + "."
                cmd_state = """
    smach.StateMachine.add('STATE_%d', FakeState(),
                           transitions={'success': 'STATE_%d_SAYSTRING'})
    
    smach.StateMachine.add('STATE_%d_SAYSTRING', SayString(text="I can't perform this part of the plan. Sorry."),
                           transitions={'success': '%s',
                                        'failure': 'STATE_%d_SAYSTRING'})
""" % (idx, idx,
       idx, next_state, idx)
            
            elif _a[0] == 'remind':
                text = re.findall('\[(.+)\]', action)
                _source = re.search('(?<=from )(\w+)',action).group()
                _target = re.search('(?<=to )(\w+)',action).group()
                _num = re.search('(?<=num )(\w+)',action).group()
                #str_plan = str_plan + " I will remind the sentence with id " + _num + " from " + _source + " to " + _target + "."
                _s = " I will remind your sentence with id " + _num + " from " + _source + " to " + _target + "."
                cmd_state = """
    smach.StateMachine.add('STATE_%d', FakeState(),
                           transitions={'success': 'STATE_%d_SAYSTRING'})
    
    smach.StateMachine.add('STATE_%d_SAYSTRING', SayString(text="%s"),
                           transitions={'success': 'STATE_%d_SAYSTRING2',
                                        'failure': 'STATE_%d_SAYSTRING'})
    
    smach.StateMachine.add('STATE_%d_SAYSTRING2', SayString(text="Your sentence is: %s."),
                           transitions={'success': '%s',
                                        'failure': 'STATE_%d_SAYSTRING2'})
""" % (idx, idx,
       idx, _s, idx, idx,
       idx, text, next_state, idx)

            elif _a[0] == 'follow':
                obj = _a[1]
                #str_plan = str_plan + " I will follow you."
                cmd_state = """
    smach.StateMachine.add('STATE_%d', FakeState(),
                           transitions={'success': 'STATE_%d_FOLLOWPERSON'})
    
    smach.StateMachine.add('STATE_%d_FOLLOWPERSON', FollowPerson(robot),
                           transitions = {'success': '%s',
                                          'failure': 'failure',
                                          'timeout': 'failure'})
""" % (idx, idx,
       idx, next_state)

        else:
            break
        
        _cmd_list.append(cmd_state)
        idx += 1
    
    cmd_state = """
    smach.StateMachine.add('STATE_%d', SayString(text="I finish the task. I will go back."),
                           transitions={'success': 'STATE_%d_MOVEBACK',
                                        'failure': 'STATE_%d'})
    
    smach.StateMachine.add('STATE_%d_MOVEBACK', MoveRobot(robot, omni_base, loc_name="%s"),
                           transitions={'success': 'success',
                                        'timeout': 'STATE_%d_MOVEBACK',
                                        'failure': 'failure'})
""" % (idx, idx, idx,
       idx, _INSTRUCTIONPOINT, idx)
    
    _cmd_list.append(cmd_state)
    
    return _cmd_list

####CONCEPTUAL DEPENDENCY TO ROBOT PLAN
def get_robot_plan(condeps, _info = [], tell_plan = False, partial_plan = False, debug = False):
    
    #(actions ask room ?obj)
    #(actions ask human recipient)
    #(actions answer where ?obj ?room-object ?zone-object)
    #(actions answer where ?human ?room-human ?zone-human)
    #(actions answer who ?object ?human ?zone-human)
    #(actions goto ?room-target ?zone-target)
    #(actions guide-human ?room-target ?zone-target)
    #(actions find-object ?object)
    #(actions take-object ?object)
    #(actions find-space ?place)
    #(actions find-human ?human)
    #(actions deliver-object ?destination)
    #(actions say-string [?text])
    #(actions remind from ?source to ?target num ?id [?text])
    #(actions propel ?action ?target)
    #(actions follow ?human)
    
    ##GET PLAN
    #fill incomplete commands
    cmd_list = []
    MAX_ERR = 5
    err_count = 0
    is_ask = True
    while(is_ask and err_count < MAX_ERR):

        if partial_plan:
            _plan = get_partial_plan(condeps, _info)
        else:
            _plan = get_full_plan(condeps, _info)
        
        is_ask = False
        for p in _plan:
            _p = p[:-1].split("(",1)
            res = re.findall(r'\(.*?\)', _p[1])
            
            for i in range(len(res)):
                res[i] = re.sub( r'[()]', '', res[i] )  
            
            action = None
            for i in range(len(res)):
                _a = res[i].split(None, 1)[0]
                if _a.lower() == 'actions':
                    action = res[i].split(None, 1)[1]
                    break
            
            if action:
                _a = action.split()
                if _a[0] == 'ask':
                    #(ask room " ?obj ")
                    is_ask = True
                    if _a[1] == 'object':
                        if _a[2] == 'location':
                            text = get_synonym(_a[3], _info)
                            _str = " Where is " + text + "?"
                            SAY(_str)
                            
                            _res = open_question()
                            print("You said: %s"%_res)
                            _w = find_word_in_string(_res, _info)
                            if _w:
                                #(state (attribute location)(obj ?object)(value ?place))
                                _st = "(state(attribute location)(obj %s)(value %s))"%(_a[3], _w)
                                load_state_fact(_st)
                            else:
                                SAY("Sorry, I didn't get it.")
                    
                    #(ask human recipient)
                    elif _a[1] == 'human':
                        if _a[2] == 'recipient':
                            _str = "To whom should I deliver the object? Give me a name, please."
                            SAY(_str)
                            
                            _res = open_question()
                            print("You said: %s"%_res)
                            _w = find_word_in_string(_res, _info)
                            if _w:
                                #(state (attribute recipient)(value ?human)
                                _st = "(state (attribute recipient)(value %s))"%(_w)
                                load_state_fact(_st)
                            else:
                                SAY("Sorry, I didn't get it.")
        
        if is_ask:
            err_count+=1
    
    if(err_count == MAX_ERR):
        SAY('Sorry, I cannot execute the command. I am learning.')
        cmd_list = []
        return cmd_list

    print("#############")
    print(_plan)
    print("#############")
    
    #Answer questions:
    is_answer = False
    for p in _plan:
        _p = p[:-1].split("(",1)
        res = re.findall(r'\(.*?\)', _p[1])
        
        for i in range(len(res)):
            res[i] = re.sub( r'[()]', '', res[i] )  
        
        action = None
        for i in range(len(res)):
            _a = res[i].split(None, 1)[0]
            if _a.lower() == 'actions':
                action = res[i].split(None, 1)[1]
                break
        
        if action:
            _a = action.split()
            if _a[0] == 'answer':
                if _a[1] == 'where':
                    is_answer = True
                    #(answer where object ?obj ?room-object ?zone-object)
                    #(answer where human ?human ?room-human ?zone-human)
                    _str = ""
                    if _a[2] == 'object':
                        _str = "The "
                    text_2 = get_synonym(_a[3], _info)
                    text_3 = get_synonym(_a[4], _info)
                    text_4 = get_synonym(_a[5], _info)
                    if (text_3 != 'nil' and text_4 != 'nil'):
                        _str = _str + text_2 + " is in " + text_3 +  " at " + text_4 + "."
                    else:
                        _str = "I don't know."
                    SAY(_str)
                    _plan.remove(p)
                elif _a[1] == 'who':
                    is_answer = True
                    #(answer who ?obj ?human ?zone-object)
                    text_2 = get_synonym(_a[2], _info)
                    text_3 = get_synonym(_a[3], _info)
                    if (text_3 != 'nil'):
                        _str = text_3 + " has " + text_2 + "." 
                    else:
                        _str = "I don't know"
                    SAY(_str)
                    _plan.remove(p)

    if (is_answer):
        cmd_list = []
        return cmd_list
    
    if (len(_plan) == 0 and not is_answer):
        SAY("Sorry, I didn't get it.")
        cmd_list = []
        return cmd_list
    
    #Say commands
    if (len(condeps) == 0):
        commands = "There is no command."
    elif (len(condeps) == 1):
        commands = "There is one command."
    else:
        commands = "There are "+str(len(condeps))+" commands."
    SAY(commands)
    
    ##GET PLAN
    is_question = False
    str_plan = "The plan is: "
    for p in _plan:
        _p = p[:-1].split("(",1)
        res = re.findall(r'\(.*?\)', _p[1])
        
        for i in range(len(res)):
            res[i] = re.sub( r'[()]', '', res[i] )  
        
        action = None
        for i in range(len(res)):
            _a = res[i].split(None, 1)[0]
            if _a.lower() == 'actions':
                action = res[i].split(None, 1)[1]
                break
        
        if action:
            _a = action.split()
            if _a[0] == 'goto':
                text_1 = get_synonym(_a[1], _info)
                text_2 = get_synonym(_a[2], _info)
                str_plan = str_plan + " I will go to " + text_2 + " in " + text_1 + "."
            
            if _a[0] == 'guide-human':
                text_1 = get_synonym(_a[1], _info)
                text_2 = get_synonym(_a[2], _info)
                str_plan = str_plan + " I will lead towards " + text_2 + " in " + text_1 + "."
            
            elif _a[0] == 'find-object':
                text = get_synonym(_a[1], _info)
                str_plan = str_plan + " I will find " + text + "."
            
            elif _a[0] == 'take-object':
                text = get_synonym(_a[1], _info)
                str_plan = str_plan + " I will take " + text + "."

            elif _a[0] == 'find-space':
                text = get_synonym(_a[1], _info)
                str_plan = str_plan + " I will deposit the object in " + text + "."
            
            elif _a[0] == 'find-human':
                text = get_synonym(_a[1], _info)
                str_plan = str_plan + " I will find " + text + "."
            
            elif _a[0] == 'deliver-object':
                text = get_synonym(_a[1], _info)
                str_plan = str_plan + " I will deliver the object to " + text + "."
            
            elif _a[0] == 'say-string':
                text = re.findall('\[(.+)\]', action)
                str_plan = str_plan + " I will say your sentence."
            
            elif _a[0] == 'remind':
                text = re.findall('\[(.+)\]', action)
                _source = re.search('(?<=from )(\w+)',action).group()
                _target = re.search('(?<=to )(\w+)',action).group()
                _num = re.search('(?<=num )(\w+)',action).group()
                #str_plan = str_plan + " I will remind the sentence with id " + _num + " from " + _source + " to " + _target + "."
                str_plan = str_plan + " I will remind your sentence."
            
            elif _a[0] == 'propel':
                act = _a[1]
                if act == 'nil':
                    act = "do nothing to"
                str_plan = str_plan + " I will " + act + " " + _a[2] + "."

            elif _a[0] == 'follow':
                obj = _a[1]
                str_plan = str_plan + " I will follow " + _a[1] + "."

    if len(_plan) > 0 and tell_plan:
        if not is_question:
            SAY('Do you want me to tell the plan? Please wait a couple of seconds to answer.')
            rospy.sleep(1)
            
            res = yes_no_question()
            #res = "yes" #HARDCODE!!!! COMMENT THIS!!
            if res == 'yes':
                SAY(str_plan)
                print("############")
                print(str_plan)
                print("############")
                rospy.sleep(5)
            
            SAY('Do you want me to perform the plan? Please wait a couple of seconds to answer.')
            rospy.sleep(2)
            
            res = yes_no_question()
            #res = "yes" #HARDCODE!!!! COMMENT THIS!!
            if res == 'no':
                cmd_list = []
                return cmd_list
        
    SAY("OK. I will perform your command.")
    
    print("############")
    print(_plan)
    print("############")
    
    if (debug):
        cmd_list = []
        return cmd_list
    
    cmd_list = generate_smach_states(_plan)
        
    return cmd_list
