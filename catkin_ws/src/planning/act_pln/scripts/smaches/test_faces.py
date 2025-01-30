#!/usr/bin/env python3
import rospy
import ros_numpy
import numpy as np
from std_msgs.msg import String
from cv_bridge import CvBridge
from sensor_msgs.msg import Image as ImageMsg, PointCloud2
from ros_whisper_vosk.srv import GetSpeech, SetGrammarVosk
from juskeshino_tools.JuskeshinoHRI import JuskeshinoHRI
from juskeshino_tools.JuskeshinoHardware import JuskeshinoHardware
from rospy.exceptions import ROSException
import smach
import smach_ros
from face_recog.msg import *
from face_recog.srv import *
import time

class Talker():
    def __init__(self):
        JuskeshinoHRI.setNodeHandle()

    def talk(self, sentence, timeout=0):
        JuskeshinoHRI.say(sentence)


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


def match_speech(speech, to_match):
    for element in to_match:
        if element in speech:
            return True
    return False


def wait_for_face(timeout=10, name='', lap_camera=False):

    rospy.sleep(0.3)

    start_time = rospy.get_time()
    strings = Strings()
    string_msg = String()
    string_msg.data = 'Anyone'
    while rospy.get_time() - start_time < timeout:
        if lap_camera:
            print("USB camera")
            img = rgb.get_image()
        else:
            img = rgbd.get_image()

        img
        req = RecognizeFaceRequest()
        print('Got  image with shape', img.shape)
        req.Ids.ids.append(string_msg)
        img_msg = bridge.cv2_to_imgmsg(img[:, 150:-150])
        req.in_.image_msgs.append(img_msg)

        res = recognize_face(req)

        # NO FACE FOUND
        if res.Ids.ids[0].data == 'NO_FACE':
            print('No face Found Keep scanning')

            return None, None
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


def analyze_face_background(img, name=" "):
    name_pub = rospy.Publisher('/name_face', String, queue_size=10)
    img_pub = rospy.Publisher('/image_to_analyze', ImageMsg, queue_size=10)
    str_msg = String()
    str_msg.data = name
    rospy.sleep(0.5)
    name_pub.publish(str_msg)
    img_msg = bridge.cv2_to_imgmsg(img)
    img_pub.publish(img_msg)


rospy.init_node('smach_justina_tune_vision')

robot_real = True
vosk_enable = True
rgbd = RGBD()
rgb = RGB()  # WEB CAM DEBUG
voice = Talker()
bridge = CvBridge()
recognize_face = rospy.ServiceProxy(
    'recognize_face', RecognizeFace)  # FACE RECOG
train_new_face = rospy.ServiceProxy('new_face', RecognizeFace)  # FACE RECOG
set_grammar = rospy.ServiceProxy('set_grammar_vosk', SetGrammarVosk)
speech_recog_server = rospy.ServiceProxy(
    '/speech_recognition/vosk_service', GetSpeech)  # SPEECH VOSK RECOG FULL DICT



##############
class Initial(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed'],
                             input_keys=['l_arm_home', 'confirm_list',
                                         'negation_list', 'speech_time'],
                             output_keys=['l_arm_home', 'confirm_list', 'negation_list', 'speech_time'])
        self.tries = 0
        global camera_enable
        if robot_real:
            print("Cam enable")
            camera_enable = False
        else:
            print("Cam disable")
            camera_enable = True

    def execute(self, userdata):
        print('\n> STATE <: INITIAL')
        global vosk_enable
        if vosk_enable:
            print("VOSK ENABLE -->")
            userdata.confirm_list = [
                'yes', 'robot yes', 'jack', 'juice', 'justina yes', 'yeah', 'correct', 'affirmative']
            userdata.negation_list = [
                'no', 'robot no', 'not', 'now', 'nope', 'justina no', 'incorrect', 'negative']
            userdata.speech_time = 6
            print("speech timeout: ", userdata.speech_time)
        else:
            userdata.confirm_list = ["YES", "YEAH",
                                     "ROBOT YES", "JUSTINA YES", "JUICE"]
            userdata.negation_list = ['NO', 'ROBOT NO', 'NOPE', 'JUSTINA NO']

        self.tries += 1
        print(f'Try {self.tries}')
        userdata.l_arm_home = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        # party.clean_knowledge(host_name="Jack", host_location="Place_2")
        # places_2_tf()

        # Set neutral pose
        # JuskeshinoHardware.moveLeftArmWithTrajectory(userdata.l_arm_home, 6)

        # -----INIT GRAMMAR FOR VOSK
        # -----Use with get_keywords_speech()
        # -----------SPEECH REC
        drinks = ['coke', 'juice', 'milk', 'water', 'soda', 'wine',
                  'I want a', 'I would like a', 'tea', 'iced tea', 'cola', 'red wine', 'orange juice', 'tropical juice']
        # drinks = ['cola', 'ice_tea', 'water', 'milk',
        #           'big_coke', 'fanta', 'dubbelfris']

        names = ['Sophie', 'Julia', 'Emma', 'Sara', 'Laura', 'Hayley', 'Susan', 'Fleur', 'Gabriëlle', 'Robin', 'John', 'Liam',
                 'Lucas', 'William', 'Kevin', 'Jesse', 'Noah', 'Harrie', 'Peter', 'Robin']
        # # 'adel', 'angel', 'axel', 'charlie', 'jane', 'john', 'jules', 'morgan', 'paris', 'robin', 'simone', 'jack']
        gram = drinks + names + userdata.confirm_list + userdata.negation_list

        # print("** confirmation list: ")
        # print(userdata.confirm_list)
        # print("** negation list: ")
        # print(userdata.negation_list)

        # head.publish_tfs()
        if self.tries == 1:
            set_grammar(gram)
            print("drinks:")
            print(drinks)
            print("-->")
            self.tries = 0
            return 'succ'

        elif self.tries == 3:
            rospy.logerr('Failed! some error ocurred')
            return 'failed'


# --------------------------------------------------
class Scan_face(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['succ', 'failed'],
                             output_keys=['name', 'face_img'])
        self.tries = 0

    def execute(self, userdata):
        global camera_enable
        self.tries += 1
        if self.tries == 1:
            print("\n")
            rospy.logwarn('--> STATE <: Scan face')

        JuskeshinoHardware.moveHead(0.0, -0.1, 2)
        # head.set_joint_values([0.0, -0.1])  # Ensure head is up before scan
        rospy.sleep(0.3)
        JuskeshinoHardware.moveHead(0.0, -0.1, 2)
        # head.set_joint_values([0.0, -0.1])
        print('Scanning for faces')
        voice.talk('Scanning for faces')

        # For simulation use camera_enable = True
        res, userdata.face_img = wait_for_face(
            lap_camera=camera_enable)  # default 10 secs
        if res != None:
            userdata.name = res.Ids.ids
            return 'succ'
        else:
            return 'failed'


# Decide face STATE: Decide if the new guest is known, unknown or can't be decided
class Decide_face(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['succ', 'failed', 'unknown'],
                             input_keys=['name', 'face_img',
                                         'confirm_list', 'speech_time'],
                             output_keys=['name', 'face_img'])
        self.tries = 0

    def execute(self, userdata):
        print("\n")
        rospy.logwarn('--> STATE <: Decide face')

        if userdata.name == 'NO_FACE':
            print('I did not see you, trying again')
            voice.talk('I did not see you, trying try again')
            return 'failed'

        elif userdata.name == 'unknown':
            print('I find you.')
            voice.talk('I find you.')
            return 'unknown'

        else:
            print(f'I found you, I Think you are, {userdata.name}.')
            print('Is it correct?')
            voice.talk(f'I found you, I Think you are, {userdata.name}.')
            voice.talk('Is it correct?')

            # TODO: Validate sentence
            print(f'Please answer, Robot yes or Robot no.')
            voice.talk(f'Please answer,. Robot yes... or. Robot no.')

            if vosk_enable:
                rospy.logwarn('Listening now (Cc')
                confirmation = get_keywords_speech(userdata.speech_time)

            else:
                JuskeshinoHRI.getLastRecognizedSentence()
                rospy.sleep(0.3)
                confirmation = JuskeshinoHRI.waitForNewSentence(
                    userdata.speech_time)  # 10 is to much?

            print(confirmation)
            if confirmation == "robot yes":
                # TODO: Save Name <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
                #party.add_guest(userdata.name)
                print(f'Nice.')
                voice.talk(f'Nice.')

                return 'succ'
            elif confirmation == "robot no":
                return 'unknown'
            else:
                print('I could not hear you, lets try again, please speak louder.')
                voice.talk(
                    'I could not hear you, lets try again, please speak louder.')
                return "failed"

            # if confirmation not in userdata.confirm_list:
            #     return 'unknown'
            # elif confirmation == "timeout":
            #     print('I could not hear you, lets try again, please speak louder.')
            #     voice.talk('I could not hear you, lets try again, please speak louder.')
            #     return "failed"
            # else:
            #     party.add_guest(userdata.name)
            #     return 'succ'


# --------------------------------------------------
class New_face(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['succ', 'failed'],
                             input_keys=['name', 'face_img',
                                         'confirm_list', 'speech_time'],
                             output_keys=['name', 'face_img'])
        self.tries = 0

    def execute(self, userdata):
        self.tries += 1
        if self.tries == 1:
            print("\n")
            rospy.logwarn('--> STATE <: new face')

        # If name is not recognized 3 times, guest will be registered as a "someone"
        if self.tries == 3:
            print("I didn't undestand your name, lets continue")
            voice.talk('I didnt undestand your name, lets continue')
            userdata.name = 'someone'
            train_face(userdata.face_img, userdata.name)
            # TODO: Save Name <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
            #party.add_guest(userdata.name)
            self.tries = 0
            return 'succ'

        # Asking for name
        print('Please, tell me your name')
        voice.talk('Please, tell me your name')

        if vosk_enable:
            rospy.logwarn('Listening now (Cc')
            speech = get_keywords_speech(userdata.speech_time)
        else:
            JuskeshinoHRI.getLastRecognizedSentence()
            rospy.sleep(0.3)
            speech = JuskeshinoHRI.waitForNewSentence(userdata.speech_time)
            speech.lower()

        # in case thinks like I am , my name is . etc
        if len(speech.split(' ')) > 1:
            name = (speech.split(' ')[-1])
        else:
            name = speech

        if userdata.name == 'timeout':
            print('Please repeat it and speak louder.')
            voice.talk('Please repeat it and speak louder.')
            return 'failed'

        if '[unk]' in name:
            name = 'william'

        print(f'Is {name} your name?')
        voice.talk(f'Is {name} your name?')

        if vosk_enable:
            rospy.logwarn('Listening now (Cc')
            confirmation = get_keywords_speech(userdata.speech_time)
        else:
            JuskeshinoHRI.getLastRecognizedSentence()
            rospy.sleep(0.3)
            confirmation = JuskeshinoHRI.waitForNewSentence(
                userdata.speech_time)

        print(confirmation)
        confirmation = confirmation.split(' ')
        # userdata.confirm_list
        confirm = match_speech(confirmation, "robot yes")
        if confirm:
            userdata.name = name
            print(f'Nice to Meet You {userdata.name}')
            voice.talk(f'Nice to Meet You {userdata.name}')
            # TODO: Save Name <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
            #party.add_guest(userdata.name)
            train_face(userdata.face_img, userdata.name)
            self.tries = 0
            return 'succ'
        else:
            print('lets try again')
            voice.talk('lets try again')
            return 'failed'


# --------------------------------------------------
# Entry point
if __name__ == '__main__':
    print("Justina STATE MACHINE...")
    # State machine, final state "END"
    sm = smach.StateMachine(outcomes=['END'])
    # sm.userdata.clear = False
    sis = smach_ros.IntrospectionServer(
        'SMACH_VIEW_SERVER', sm, '/SM_RECEPTIONIST')
    sis.start()

    with sm:
        # State machine for Receptionist task
        # Initial states routine
        smach.StateMachine.add("INITIAL", Initial(),
                               transitions={'failed': 'INITIAL', 'succ': 'SCAN_FACE'})

        # Guest recognition states
        smach.StateMachine.add("SCAN_FACE", Scan_face(),
                               transitions={'failed': 'SCAN_FACE', 'succ': 'DECIDE_FACE'})

        smach.StateMachine.add("DECIDE_FACE", Decide_face(),
                               transitions={'failed': 'SCAN_FACE', 'succ': 'END', 'unknown': 'NEW_FACE'})

        smach.StateMachine.add("NEW_FACE", New_face(),
                               transitions={'failed': 'NEW_FACE', 'succ': 'END'})

    outcome = sm.execute()
