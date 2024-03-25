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
from sensor_msgs.msg import Image as ImageMsg, PointCloud2, LaserScan
from geometry_msgs.msg import Twist, PointStamped, PoseStamped, Quaternion, Point, TransformStamped

from std_msgs.msg import String
import actionlib
from actionlib_msgs.msg import GoalStatus
from hri_msgs.msg import RecognizedSpeech
import yaml
#
from ros_whisper_vosk.srv import GetSpeech, SetGrammarVosk
#
from segmentation.srv import *
#
from object_classification.srv import *
#
from face_recog.msg import *
from face_recog.srv import *
#
from human_detector.srv import Human_detector, Human_detectorResponse
from human_detector.srv import Point_detector, Point_detectorResponse
#
from hmm_navigation.msg import NavigateAction, NavigateActionGoal, NavigateActionFeedback, NavigateActionResult
#
from juskeshino_tools.JuskeshinoHRI import JuskeshinoHRI
from juskeshino_tools.JuskeshinoHardware import JuskeshinoHardware
import time


class Talker():
    def __init__(self):
        JuskeshinoHRI.setNodeHandle()

    def talk(self, sentence, timeout=0):
        st_time = time.time()
        now = time.time()
        JuskeshinoHRI.say(sentence)
        # while st_time - now < timeout:  # Uncomment if time out is needed
        #     now = time.time()


class Head:  # known as Gaze on Takeshi grasp_utils.py
    def __init__(self):
        # pan, tilt are in rads
        JuskeshinoHardware.setNodeHandle()
        self.pan_min_limit = -1.5708
        self.pan_max_limit = 1.5708
        self.tilt_min_limit = -1.5
        self.tilt_max_limit = 0.3

    def to_tf(tf):
        pass

    def set_named_target(self, pose_name='neutral'):
        if pose_name == 'neutral':
            self.set_joint_values(head_pose=[0.0, 0.0])
        if pose_name == 'down':
            self.set_joint_values(head_pose=[0.0, self.tilt_min_limit])
        if pose_name == 'face_to_face':
            self.set_joint_values(head_pose=[0.0, -0.1])

    def set_joint_values(self, head_pose=[0.0, 0.0]):
        if head_pose[0] < self.pan_min_limit or head_pose[0] > self.pan_max_limit:
            print("Error! pan value exceeds limits")
            print("limits are: [", self.pan_min_limit,
                  ',', self.pan_max_limit, ']')
        elif head_pose[1] < self.pan_min_limit or head_pose[1] > self.pan_max_limit:
            print("Error! tilt value exceeds limits")
            print("limits are: [", self.tilt_min_limit,
                  ',', self.tilt_max_limit, ']')
        else:  # in range
            JuskeshinoHardware.moveHead(head_pose[0], head_pose[1], 2)

    def turn_base_gaze(tf, to_gaze):  # Tuns head and base to find target
        pass


######################################################
def seg_res_tf(res):

    # brazo.set_named_target('go')
    if len(res.poses.data) == 0:
        print('no objs')
        return False
    else:

        poses = np.asarray(res.poses.data)
        quats = np.asarray(res.quats.data)
        poses = poses.reshape((int(len(poses)/3), 3))
        quats = quats.reshape((int(len(quats)/4), 4))
        num_objs = len(poses)
        print(f'{num_objs} found')

        for i, cent in enumerate(poses):
            x, y, z = cent
            axis = [0, 0, 1]
            angle = tf.transformations.euler_from_quaternion(quats[i])[0]
            rotation_quaternion = tf.transformations.quaternion_about_axis(
                angle, axis)
            point_name = f'object_{i}'
            print(f'{point_name} found at {cent}')
            # same rotation as map
            tf_man.pub_tf(pos=cent, rot=[0, 0, 0, 1],
                          point_name=point_name+'_norot', ref='map')
            succ = tf_man.pub_tf(
                pos=cent, rot=rotation_quaternion, point_name=point_name, ref='map')  # PCA
            # tf_man.pub_static_tf(pos=cent, rot =[0,0,0,1], point_name=point_name+'_norot', ref='map')##  static tf
            # tf_man.pub_static_tf(pos=cent, rot =rotation_quaternion, point_name=point_name, ref='map')## static tf

    return succ
# ------------------------------------------------------
# 3


class TF_MANAGER():
    def __init__(self):
        self._tfbuff = tf2.Buffer()
        self._lis = tf2.TransformListener(self._tfbuff)
        self._tf_static_broad = tf2.StaticTransformBroadcaster()
        self._broad = tf2.TransformBroadcaster()

    def _fillMsg(self, pos=[0, 0, 0], rot=[0, 0, 0, 1], point_name='', ref="map"):
        TS = TransformStamped()
        TS.header.stamp = rospy.Time.now()
        TS.header.frame_id = ref
        TS.child_frame_id = point_name
        TS.transform.translation = Point(*pos)
        TS.transform.rotation = Quaternion(*rot)
        return TS

    def pub_tf(self, pos=[0, 0, 0], rot=[0, 0, 0, 1], point_name='', ref="map"):
        dinamic_ts = self._fillMsg(pos, rot, point_name, ref)
        self._broad.sendTransform(dinamic_ts)

    def pub_static_tf(self, pos=[0, 0, 0], rot=[0, 0, 0, 1], point_name='', ref="map"):
        static_ts = self._fillMsg(pos, rot, point_name, ref)
        self._tf_static_broad.sendTransform(static_ts)

    def change_ref_frame_tf(self, point_name='', rotational=[0, 0, 0, 1], new_frame='map'):
        try:
            traf = self._tfbuff.lookup_transform(
                new_frame, point_name, rospy.Time(0))
            translation, _ = self.tf2_obj_2_arr(traf)
            self.pub_static_tf(pos=translation, rot=rotational,
                               point_name=point_name, ref=new_frame)
            return True
        except:
            return False

    def getTF(self, target_frame='', ref_frame='map'):
        try:
            tf = self._tfbuff.lookup_transform(
                ref_frame, target_frame, rospy.Time(0))
            return self.tf2_obj_2_arr(tf)
        except:
            return [False, False]

    def tf2_obj_2_arr(self, transf):
        pos = []
        pos.append(transf.transform.translation.x)
        pos.append(transf.transform.translation.y)
        pos.append(transf.transform.translation.z)

        rot = []
        rot.append(transf.transform.rotation.x)
        rot.append(transf.transform.rotation.y)
        rot.append(transf.transform.rotation.z)
        rot.append(transf.transform.rotation.w)

        return [pos, rot]


class TF():
    def __init__(self):
        self._listener = tf.TransformListener()
        self._br = tf.TransformBroadcaster()

    def getTF(self, target="", reference="map", duration=2.0):
        # True / False is a success flag
        now = rospy.Time(0)
        try:
            self._listener.waitForTransform(
                reference, target, now, rospy.Duration(duration))
            (trans, rot) = self._listener.lookupTransform(
                reference, target, now)
            return (True, trans, rot)
        except:
            return False, 0.0, 0.0

    def pubTF(self, pos=[0, 0, 0], rot=[0, 0, 0, 1], TF_name="", reference="map"):
        try:
            self._br.sendTransform(
                pos, rot, rospy.Time.now(), TF_name, reference)
            return True
        except:
            return False

        # Maybe could not be used
    '''def pubStaticTF(self, pos = [0,0,0], rot = [0,0,0,1], TF_name ="", reference="map"):
        static_ts = self._fillMsg(pos, rot, point_name, ref)
        self._tf_static_broad.sendTransform(static_ts)'''

    def changeRefFrameTF(self, TF_name='', rot=[0, 0, 0, 1], new_ref="map"):
        try:
            t, r = self._listener.lookupTransform(
                new_ref, TF_name, rospy.Time(0))
            self._br.sendTransform(t, r, rospy.Time.now(), TF_name, new_ref)
            return True
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return False

    @staticmethod
    def quat_to_eu(quat=[0, 0, 0, ]):
        roll, pitch, yaw = tf.transformations.euler_from_quaternion(quat)
        return [roll, pitch, yaw]

    @staticmethod
    def eu_to_quat(eu=[0, 0, 0]):
        q1, q2, q3, q4 = tf.transformations.quaternion_from_euler(*eu)
        return [q1, q2, q3, q4]


# 3
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
        self._cloud_sub = rospy.Subscriber(
            "/usb_cam/image_raw", ImageMsg, self._cloud_cb)  # USB CAM ROSPKG
        # self._cloud_sub = rospy.Subscriber("/camera/depth_registered/rgb/image_raw",ImageMsg, self._cloud_cb)## JUSTINA KINECT

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
        self._image_data = self._points_data['rgb'].view(
            (np.uint8, 4))[..., [2, 1, 0]]

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
    msg.data = 'start'
    pub.publish(msg)
    try:
        msg = rospy.wait_for_message(
            '/speech_recognition/final_result', String, timeout)
        result = msg.data
        pub.publish(String())
        rospy.sleep(1.0)
        return result

    except ROSException:
        rospy.loginfo('timeout')
        pub.publish(String())
        return 'timeout'
# ------------------------------------------------------


def match_speech(speech, to_match):
    for element in to_match:
        if element in speech:
            return True
    return False
# ------------------------------------------------------


def train_face(image, name):
    """writes request message and requests trainface service
            /face_recog pkg"""
    req = RecognizeFaceRequest()
    strings = Strings()
    string_msg = String()
    string_msg.data = name
    req.Ids.ids.append(string_msg)

    img_msg = bridge.cv2_to_imgmsg(image)
    req.in_.image_msgs.append(img_msg)
    res = train_new_face(req)

    return res.Ids.ids[0].data.split(' ')[0] == 'trained'


#########################################

def wait_for_face(timeout=10, name=''):
    """Wait for timeout seconds until a face is found.
        if name is provided then only a face with id== name will return True
    """
    start_time = rospy.get_time()
    strings = Strings()
    string_msg = String()
    string_msg.data = 'Anyone'
    rospy.sleep(0.3)
    while (rospy.get_time() - start_time) < timeout:
        img = rgb.get_image()
        req = RecognizeFaceRequest()
        print('Got  image with shape', img.shape)
        req.Ids.ids.append(string_msg)
        img_msg = bridge.cv2_to_imgmsg(img)
        req.in_.image_msgs.append(img_msg)
        res = recognize_face(req)
        # NO FACE FOUND
        if res.Ids.ids[0].data == 'NO_FACE':
            print(f'time {(rospy.get_time() - start_time)} elapsed')
        # AT LEAST ONE FACE FOUND
        else:
            print('at least one face found')
            ds_to_faces = []
            for i, idface in enumerate(res.Ids.ids):
                print(i, idface.data)
                ds_to_faces.append(res.Ds.data[i])
                if (idface.data) == name:
                    new_res = RecognizeFaceResponse()
                    new_res.Ds.data = res.Ds.data[i]
                    new_res.Angs.data = res.Angs.data[i:i+4]
                    new_res.Ids.ids = res.Ids.ids[i].data
                    print('return res,img', new_res)
                    print('hit', idface.data, 'at', res.Ds.data[i], 'meters')
                    ds_to_faces = []
                    return new_res, img

            if len(ds_to_faces) != 0:
                i = np.argmin(ds_to_faces)
                new_res = RecognizeFaceResponse()
                new_res.Ds.data = res.Ds.data[i]
                new_res.Angs.data = res.Angs.data[i:i+4]
                new_res.Ids.ids = res.Ids.ids[i].data
                print('return res,img', new_res)
                ds_to_faces = []
                return new_res, img


global omni_base, rgb, rgbd, bridge, pointing_detect_server, classify_client, segmentation_server, tf_man
rospy.init_node('smach_justina_tune_vision')


bridge = CvBridge()
rgbd = RGBD()
rgb = RGB()  # WEB CAM DEBUG
omni_base = OMNIBASE()
tf_man = TF_MANAGER()
voice = Talker()
head = Head()


speech_recog_server = rospy.ServiceProxy(
    '/speech_recognition/vosk_service', GetSpeech)  # SPEECH VOSK RECOG FULL DICT
# Get speech vosk keywords from grammar (function get_keywords)
set_grammar = rospy.ServiceProxy('set_grammar_vosk', SetGrammarVosk)
recognize_face = rospy.ServiceProxy(
    'recognize_face', RecognizeFace)  # FACE RECOG
train_new_face = rospy.ServiceProxy('new_face', RecognizeFace)  # FACE RECOG
pointing_detect_server = rospy.ServiceProxy('/detect_pointing', Point_detector)
classify_client = rospy.ServiceProxy('/classify', Classify)
segmentation_server = rospy.ServiceProxy('/segment', Segmentation)
