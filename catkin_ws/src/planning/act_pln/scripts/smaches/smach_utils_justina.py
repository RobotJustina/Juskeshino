#!/usr/bin/env python3

import smach
import smach_ros
import cv2
import tf as tf
import tf2_ros as tf2
from cv_bridge import CvBridge, CvBridgeError
import rospy
from rospy.exceptions import ROSException
import numpy as np
import ros_numpy
from sensor_msgs.msg import Image as ImageMsg, PointCloud2 , LaserScan
from geometry_msgs.msg import Twist, PointStamped, PoseStamped, Quaternion, Point
from std_msgs.msg import String
import actionlib
from actionlib_msgs.msg import GoalStatus
from hri_msgs.msg import RecognizedSpeech

from ros_whisper_vosk.srv import GetSpeech , SetGrammarVosk

from face_recog.msg import *
from face_recog.srv import *





from hmm_navigation.msg import NavigateAction, NavigateActionGoal, NavigateActionFeedback, NavigateActionResult

class OMNIBASE():
    def __init__(self):
        
        self.timeout = 0.5
        
        self.navclient = actionlib.SimpleActionClient(
           '/navigate', NavigateAction)  # PUMAS NAV ACTION LIB
        
        self.nav_goal_pub = rospy.Publisher(
            '/move_base_simple/goal', PoseStamped, queue_size=10)

    

    def move_base(self, goal_x=0.0, goal_y=0.0, goal_yaw=0.0, time_out=60, known_location='None'):
        # Create and fill Navigate Action Goal message
        nav_goal = NavigateActionGoal()
        nav_goal.goal.x = goal_x
        nav_goal.goal.y = goal_y
        nav_goal.goal.yaw = goal_yaw
        nav_goal.goal.timeout = time_out
        nav_goal.goal.known_location = known_location
        print(nav_goal)

        # send message to the action server
        self.navclient.send_goal(nav_goal.goal)

        # wait for the action server to complete the order
        self.navclient.wait_for_result(timeout=rospy.Duration(time_out))
        # Results of navigation
        # PENDING         = 0
        # ACTIVE          = 1
        # PREEMPTED       = 2
        # SUCCEEDED       = 3
        # ABORTED         = 4
        # REJECTED        = 5
        # PREEMPTING      = 6
        # RECALLING       = 7
        # RECALLED        = 8
        # LOST            = 9
        action_state = self.navclient.get_state()
        return action_state


class RGB():
    def __init__(self):
        self._cloud_sub = rospy.Subscriber("/usb_cam/image_raw",ImageMsg, self._cloud_cb) ## USB CAM ROSPKG
        #self._cloud_sub = rospy.Subscriber("/camera/depth_registered/rgb/image_raw",ImageMsg, self._cloud_cb)## JUSTINA KINECT
        
        self._image_data = None
    def _cloud_cb(self, msg):
        self._image_data = bridge.imgmsg_to_cv2(msg)
    def get_image(self):
        return self._image_data


class RGBD():
    def __init__(self):
        
        self._cloud_sub = rospy.Subscriber(
            "/camera/depth_registered/points",
            PointCloud2, self._cloud_cb)
        self._points_data = None
        self._image_data = None
        

    def _cloud_cb(self, msg):
        self._points_data = ros_numpy.numpify(msg)
        self._image_data = self._points_data['rgb'].view((np.uint8, 4))[..., [2, 1, 0]]
        

    def get_image(self):
        return self._image_data

    def get_points(self):
        return self._points_data

#########################################
def get_keywords_speech(timeout=5):
    """ Function to get key words from ros VOSK service
        speech recognition (/speech_ recog)"""
    pub = rospy.Publisher('/talk_now', String, queue_size=10)
    rospy.sleep(0.8)
    msg = String()
    msg.data='start'
    pub.publish(msg)
    try:
        msg = rospy.wait_for_message('/speech_recognition/final_result', String, timeout)
        result = msg.data
        pub.publish(String())
        rospy.sleep(1.0)
        return result
            
    except ROSException:
        rospy.loginfo('timeout')
        pub.publish(String())
        return 'timeout'
#------------------------------------------------------
def match_speech(speech, to_match):
    for element in to_match:
        if element in speech:
            return True
    return False
#------------------------------------------------------
def train_face(image, name):
    """writes request message and requests trainface service
            /face_recog pkg"""
    req=RecognizeFaceRequest()
    strings=Strings()
    string_msg= String()
    string_msg.data=name
    req.Ids.ids.append(string_msg)

    img_msg=bridge.cv2_to_imgmsg(image)
    req.in_.image_msgs.append(img_msg)
    res=train_new_face(req)
    
    return res.Ids.ids[0].data.split(' ')[0] == 'trained'


#########################################

def wait_for_face(timeout=10 , name=''):
    """Wait for timeout seconds until a face is found.
        if name is provided then only a face with id== name will return True
    """
    start_time = rospy.get_time()
    strings=Strings()
    string_msg= String()
    string_msg.data='Anyone'
    rospy.sleep(0.3)
    while (rospy.get_time() - start_time) < timeout:
        img=rgb.get_image()
        req=RecognizeFaceRequest()
        print ('Got  image with shape',img.shape)
        req.Ids.ids.append(string_msg)
        img_msg=bridge.cv2_to_imgmsg(img)
        req.in_.image_msgs.append(img_msg)
        res= recognize_face(req)
        #NO FACE FOUND
        if res.Ids.ids[0].data == 'NO_FACE':
            print (f'time {(rospy.get_time() - start_time)} elapsed')
        #AT LEAST ONE FACE FOUND
        else:
            print('at least one face found')
            ds_to_faces=[]
            for i , idface in enumerate(res.Ids.ids):
                print (i,idface.data)
                ds_to_faces.append(res.Ds.data[i])    ##
                if (idface.data)==name :
                    new_res= RecognizeFaceResponse()
                    new_res.Ds.data= res.Ds.data[i]
                    new_res.Angs.data= res.Angs.data[i:i+4]
                    new_res.Ids.ids=res.Ids.ids[i].data
                    print('return res,img',new_res)
                    print ('hit',idface.data, 'at' , res.Ds.data[i]  , 'meters')
                    ds_to_faces=[]
                    return new_res , img

            if len (ds_to_faces)!=0:
                i=np.argmin(ds_to_faces)
                new_res= RecognizeFaceResponse()
                new_res.Ds.data= res.Ds.data[i]
                new_res.Angs.data= res.Angs.data[i:i+4]
                new_res.Ids.ids=res.Ids.ids[i].data
                print('return res,img',new_res)
                ds_to_faces=[]
                return new_res , img







global omni_base, rgb, rgbd , bridge
rospy.init_node('smach')


bridge = CvBridge()
rgbd=RGBD()
rgb=RGB()
omni_base=OMNIBASE()




speech_recog_server = rospy.ServiceProxy('/speech_recognition/vosk_service' ,GetSpeech)##############SPEECH VOSK RECOG FULL DICT
set_grammar = rospy.ServiceProxy('set_grammar_vosk', SetGrammarVosk)                   ###### Get speech vosk keywords from grammar (function get_keywords)         
recognize_face = rospy.ServiceProxy('recognize_face', RecognizeFace)                    #FACE RECOG
train_new_face = rospy.ServiceProxy('new_face', RecognizeFace)                          #FACE RECOG
