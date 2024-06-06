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
from geometry_msgs.msg import Twist, PoseStamped, Quaternion, Point, TransformStamped # PointStamped
from tf2_geometry_msgs import PointStamped

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
from human_detector.srv import Human_detector, Human_detectorResponse, Human_detectorRequest 
from human_detector.srv import Point_detector, Point_detectorResponse
#
from hmm_navigation.msg import NavigateAction, NavigateActionGoal, NavigateActionFeedback, NavigateActionResult
#
from juskeshino_tools.JuskeshinoHRI import JuskeshinoHRI
from juskeshino_tools.JuskeshinoHardware import JuskeshinoHardware
import time
from receptionist_knowledge import *
import trajectory_msgs.msg  # HAND point
import pandas as pd



class Talker():
    def __init__(self):
        JuskeshinoHRI.setNodeHandle()

    def talk(self, sentence, timeout=0):
        st_time = time.time()
        
        JuskeshinoHRI.say(sentence)
        now = time.time()
        # while st_time - now < timeout:  # Uncomment if time out is needed
        #     now = time.time()


class Head:  # known as Gaze on Takeshi grasp_utils.py
    def __init__(self, head_controller_topic = '/head_trajectory_controller/command'):
        # pan, tilt are in rads
        JuskeshinoHardware.setNodeHandle()
        self.pan_min_limit = -1.5708
        self.pan_max_limit = 1.5708
        self.tilt_min_limit = -1.5
        self.tilt_max_limit = 0.3

        ##########
        self._reference = 'map'
        self._cam = 'camera_rgb_optical_frame'  #realsense_link, take:'head_rgbd_sensor_link'
        self._base = 'base_link'
        self._hand = 'left_arm_grip_center' #'hand_palm_link' # TODO: verify link
        self._tf_man = TF_MANAGER()
        self._pub = rospy.Publisher( head_controller_topic,
            trajectory_msgs.msg.JointTrajectory, queue_size=10)

    def _gaze_point(self):
    ###Moves head to make center point of rgbd image to coordinates w.r.t.map
        trans,_ = self._tf_man.getTF(ref_frame=self._reference,target_frame=self._cam)
        rospy.sleep(0.3)
        _,rot = self._tf_man.getTF(ref_frame=self._reference, target_frame=self._base)
        _,_, th_rob = tf.transformations.euler_from_quaternion(rot)
        
        x_rob, y_rob, z_rob = trans[0], trans[1], trans[2]
        D_x = x_rob - self._x
        D_y = y_rob - self._y
        D_z = z_rob - self._z
        D_th = np.arctan2(D_y,D_x)
        #pan: horizontal angle, tilt: vertical angle
        #keep pan angle between -2*pi and 2*pi
        pan_correct = (- th_rob + D_th + np.pi) % (2*np.pi)
        #pan mapping from pi and -pi
        if pan_correct > np.pi:
            pan_correct -= 2 * np.pi
        elif pan_correct < -np.pi:
            pan_correct += 2 * np.pi
        tilt_correct = -np.arctan2(D_z,np.linalg.norm((D_x,D_y)))
        #pan exorcist alert 
        print("pan, tilt:", pan_correct, tilt_correct)

        #if abs(pan_correct) > 0.5 * np.pi:
        if abs(pan_correct) > self.pan_max_limit:
            print ('Exorcist alert')
            # pan_correct=0.5*np.pi
            self.set_joint_values([0.0, tilt_correct])
            self.turn_base_gaze()
            return [0.0, tilt_correct]
            # return self._gaze_point()
        else:    
            head_pose = [pan_correct,  tilt_correct]
            return head_pose

    def _gaze_abs_rel(self, x, y, z):
        self._x = x
        self._y = y
        self._z = z
        head_pose =  self._gaze_point()
        self.set_joint_values(head_pose)
        return head_pose

    def absolute(self, x: float, y:float, z: float):
        #Head gaze to a x, y, z point relative to map
        self._reference = 'map'
        return self._gaze_abs_rel(x,y,z)

    def set_named_target(self, pose_name='neutral'):
        if pose_name == 'neutral':
            self.set_joint_values(head_pose=[0.0, 0.0])
        elif pose_name == 'down':
            self.set_joint_values(head_pose=[0.0, self.tilt_min_limit])
        elif pose_name == 'right':
            self.set_joint_values(head_pose=[-0.7, 0.0])
        elif pose_name == 'left':
            self.set_joint_values(head_pose=[0.7, 0.0])
        elif pose_name == 'face_to_face':
            self.set_joint_values(head_pose=[0.0, -0.1])

    #def get_joint_values(self): 

    def set_joint_values(self, head_pose=[0.0, 0.0]):  # JUSTINA version
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

    def to_tf(self, target_frame='None'):
        print("target_frame", target_frame)
        if target_frame != 'None':
            rospy.sleep(0.5)
            xyz,_ = self._tf_man.getTF(target_frame=target_frame)
            rospy.sleep(0.8)
            tries = 1
            
            print("---------- xyz:", xyz)
            #type(xyz) is bool --> tf no encontrada
            while (type(xyz) is bool):
                num_loc = (int) (target_frame.replace('Place_face', ''))
                locs = [[9.65,-2.02,0.0], [9.72,-2.1,0.0], [9.67,-3.06,0.0]]
                loc =locs[num_loc-1]
                xyz = [loc[0], loc[1], 0.85]
            if type(xyz) is not bool:
               self.absolute(*xyz)

    def turn_base_gaze(self, tf='None', to_gaze ='base_link'):
        base = OMNIBASE()
        succ = False
        THRESHOLD = 0.05
        tries = 0
        if tf != 'None':
            target_frame = tf
        else:
            target_frame = 'gaze'
            self._tf_man.pub_static_tf(pos=[self._x,self._y,self._z], point_name='gaze')
            rospy.sleep(0.1)
        while (not succ) and (tries <=10):
            tries += 1
            rospy.sleep(0.2)
            xyz,_=self._tf_man.getTF(target_frame=target_frame, ref_frame=to_gaze)
            eT = 0
            if type(xyz) is not bool:
                eT = np.arctan2(xyz[1],xyz[0])
                succ = abs(eT) < THRESHOLD 
            if succ:
                eT = 0
            base.tiny_move(velT = eT, MAX_VEL_THETA=1.1)
        return True
    
    def publish_tfs(self):
        file_name = rospy.get_param("file_name", "/known_locations.yaml")
        file_path = rospkg.RosPack().get_path('config_files')  + file_name
        print(file_path)
        with open(file_path, 'r') as file:
            con = yaml.safe_load(file)

        values=[]
        locations=[]
        for c in con:
            locations.append(c)

            for i in range(len(con[c])):
                values.append(list(con[c][i].values())[0])

        data = np.asarray(values).reshape((int(len(values)/7),7))    #x , y ,theta  ,quat   since z always 0
        df = pd.DataFrame(data)
        df.columns=['x', 'y', 'th', 'qx', 'qy', 'qz', 'qw']
        df['child_id_frame'] = locations
        print (f'known locs server available using {file_name}')
        print(locations)

        if len(df)!=0:
            for i in range(len(df)):
                name = locations[i]
                trans = df[['x','y','th']].iloc[i].values
                quat = df[['qx','qy','qz','qw']].iloc[i].values
                t = self.write_tf(trans, quat, name)

                #self._tf_man.pub_static_tf(pos=[x, y, z], point_name='gaze')
                self._tf_man._tf_static_broad.sendTransform(t)


    def write_tf(self, pose, q, child_frame , parent_frame='map'):
        if child_frame == "kitchen_table_search":
            z = 1.0
        else:
            z = 0

        t = TransformStamped()
        t.header.stamp = rospy.Time(0)
        t.header.frame_id = parent_frame
        t.child_frame_id = child_frame
        t.transform.translation.x = pose[0]
        t.transform.translation.y = pose[1]
        t.transform.translation.z = z
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        return t


def detect_object_yolo(object_name, res, long_names=True):
    # find object_name in the response message from object_classification service (Yolo)
    objs=[]
    if long_names:  # removes nnn_ from name object
        for i, name in enumerate(res.names):
            objs.append(name.data[4:])
            if name.data[4:]==object_name:
                return res.poses[i]
        if object_name=='all': 
            return objs
    else:
        for i, name in enumerate(res.names):
            objs.append(name)
            if name==object_name:
                return res.poses[i]
        if object_name=='all': 
            return objs
    return []

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

    def getTF(self, target_frame='', ref_frame='map', times=0):
        print(">>>>>>>>>>>>>", target_frame)
        print(type(target_frame))
        try:
            tf = self._tfbuff.lookup_transform(
                ref_frame, target_frame, rospy.Time(0), rospy.Duration(1))
            return self.tf2_obj_2_arr(tf)
        except:
            times += 1
            rospy.logerr("lookup_transform Failed, trying again")
            if times > 3:
                return [False, False]
            else:
                self.getTF(target_frame=target_frame, ref_frame=ref_frame, times=times)

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
    def __init__(self, cmd_vel_topic = '/hardware/mobile_base/cmd_vel'):  #'/hsrb/command_velocity'):    old->cmd_vel

        # if robot_real:
        #     cmd_vel_topic = '/hardware/mobile_base/cmd_vel'

        self.timeout = 0.5
        self._base_vel_pub = rospy.Publisher(
            cmd_vel_topic , Twist, queue_size=10)
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

    def _move_base_vel(self):
        twist = Twist()
        twist.linear.x = self.velX
        twist.linear.y = self.velY
        twist.angular.z = self.velT
        self._base_vel_pub.publish(twist)

    def _move_base_time(self):
        start_time = rospy.Time.now().to_sec()
        while rospy.Time.now().to_sec() - start_time < self.timeout:
            self._move_base_vel()

    def tiny_move(self, velX=0, velY=0, velT=0, std_time=0.5, MAX_VEL=0.03, MAX_VEL_THETA=0.5):
        self.MAX_VEL = MAX_VEL
        self.MAX_VEL_THETA = MAX_VEL_THETA
        self.timeout = std_time
        if abs(velX) > MAX_VEL:
            self.velX = MAX_VEL * (velX / abs(velX))
        else:
            self.velX = velX
        if abs(velY) > MAX_VEL:
            self.velY = MAX_VEL * (velY / abs(velY))
        else:
            self.velY = velY
        if abs(velT) > MAX_VEL_THETA:
            self.velT = MAX_VEL_THETA * (velT / abs(velT))
        else:
            self.velT = velT
        self._move_base_time()


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


#------------------------------------------------------
def detect_human_to_tf(dist = 6):
    req = Human_detectorRequest()
    req.dist = dist
    humanpose=human_detect_server(req)
    #print ("humanpose",humanpose)
    if (np.asarray((humanpose.x,humanpose.y,humanpose.z)).all()== np.zeros(3).all()):
        #print ("ASARRAY",np.asarray((humanpose.x,humanpose.y,humanpose.z)))
        return False
    else:
        tf_man.pub_static_tf(np.asarray((humanpose.x,humanpose.x,humanpose.z)),point_name='human', ref='camera_rgb_optical_frame')
        rospy.sleep(0.5)
        succ=tf_man.change_ref_frame_tf('human')
        rospy.sleep(0.5)
        #print("SUCC?", succ)
        return succ



# ------------------------------------------------------
def match_speech(speech, to_match):
    for element in to_match:
        if element in speech:
            return True
    return False


# ------------------------------------------------------
# Functions
def places_2_tf():
    places, locs = party.get_places_location()
    for place, loc in zip(places, locs):
        print(place, loc)
        pos = [loc[0], loc[1], 0.0] # TODO: Z was 0.85 why?
        rot = tf.transformations.quaternion_from_euler(0.0, 0.0, loc[2])
        tf_man.pub_static_tf(pos=pos, rot=rot, point_name=place)
        rospy.sleep(0.6)
        tf_face = place.replace('_', '_face')
        tf_man.pub_static_tf(pos=[0.6, 0, 0.85], rot=rot, point_name=tf_face, ref=place)
        rospy.sleep(0.6)
#------------------------------------------------------
        
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

def wait_for_face(timeout=10 , name='', lap_camera=False):
    
    rospy.sleep(0.3)
    
    start_time = rospy.get_time()
    strings=Strings()
    string_msg= String()
    string_msg.data='Anyone'
    while rospy.get_time() - start_time < timeout:
        if lap_camera:
            print("USB camera")
            img=rgb.get_image()
        else:
            img=rgbd.get_image()

        img  
        req=RecognizeFaceRequest()
        print ('Got  image with shape',img.shape)
        req.Ids.ids.append(string_msg)
        img_msg=bridge.cv2_to_imgmsg(img[:,150:-150])
        req.in_.image_msgs.append(img_msg)

        res= recognize_face(req)


        #NO FACE FOUND
        if res.Ids.ids[0].data == 'NO_FACE':
            print ('No face FOund Keep scanning')
            
            return None, None
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




global omni_base, rgb, rgbd, bridge, pointing_detect_server, classify_client
global segmentation_server, tf_man, voice, head, party, tfBuffer, listener
global robot_real, human_detect_server
rospy.init_node('smach_justina_tune_vision')

robot_real = True
vosk_enable = True
bridge = CvBridge()
rgbd = RGBD()
rgb = RGB()  # WEB CAM DEBUG
omni_base = OMNIBASE()
tf_man = TF_MANAGER()
voice = Talker()
head = Head()
party = RECEPTIONIST()

tfBuffer = tf2.Buffer()  # tf2_ros.Buffer()
listener = tf2.TransformListener(tfBuffer) # tf2_ros.TransformListener(tfBuffer)

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


human_detect_server = rospy.ServiceProxy('/detect_human' , Human_detector)  ####HUMAN FINDER OPPOSEBASED