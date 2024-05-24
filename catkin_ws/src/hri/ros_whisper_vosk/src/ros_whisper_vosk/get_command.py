#!/usr/bin/env python

import rospy
import smach
import traceback
import cv2

from common import speech
from common.utils import TemporarySubscriber

class GetCommand(smach.State):
    def __init__(self, listen_fn, parse_fn,
                 use_bypass=True, timeout=30., retry=0,
                 say_fn=None,
                 prompt_msg="Please give me a command.",
                 miss_msg="Sorry. I don't understand.",
                 bypass_msg="Could you give it to me by QR code?",
                 success_msg="I got it.",
                 qr_image_topic='/hsrb/head_rgbd_sensor/rgb/image_rect_color'):
        
        from cv_bridge import CvBridge

        smach.State.__init__(self, outcomes=['success', 'bypass', 'failure', 'timeout'],
                             output_keys=['command'])
        self.timeout = timeout
        self.retry = retry
        self.listen_fn = listen_fn
        self.parse_fn = parse_fn
        self.use_bypass = use_bypass
        self.bridge = CvBridge()
        self.qr_reader = cv2.QRCodeDetector()
        self.qr_data = []
        self.prompt_msg = prompt_msg
        self.miss_msg = miss_msg
        self.bypass_msg = bypass_msg
        self.success_msg = success_msg
        self.qr_image_topic = qr_image_topic
        self.say_fn = say_fn if say_fn else speech.DefaultTTS().say

    def execute(self, userdata):
        try:
            self.qr_data = []
            from sensor_msgs.msg import Image
            with TemporarySubscriber(self.qr_image_topic, Image, self.image_cb):
                count = 0
                while count < self.retry:
                    if self.use_bypass and self.qr_data:
                        parse = filter(self.parse_fn, self.qr_data)
                        if parse:
                            if self.say_fn and self.success_msg:
                                self.say_fn(self.success_msg)
                            userdata.command = next(parse, None) #parse[0]
                            print(userdata.command)
                            return 'success'
                    if self.say_fn and self.prompt_msg:
                        self.say_fn(self.prompt_msg)
                    try:
                        text = self.listen_fn(self.timeout)
                    except:
                        rospy.logerr(traceback.format_exc())
                        if self.preempt_requested():
                            return 'timeout'
                        if not self.use_bypass:
                            return 'timeout'
                        break
                    if self.parse_fn(text):
                        if self.say_fn and self.success_msg:
                            self.say_fn(self.success_msg)
                        userdata.command = text
                        return 'success'
                    if self.say_fn and self.miss_msg:
                        self.say_fn(self.miss_msg)
                    rospy.sleep(2)
                    count += 1

                if not self.use_bypass:
                    return 'timeout'

                if self.say_fn and self.bypass_msg:
                    self.say_fn(self.bypass_msg)

                elapsed = 0.
                while not rospy.is_shutdown() and (not self.timeout or elapsed < self.timeout):
                    parse = list(filter(self.parse_fn, self.qr_data))
                    #parse = filter(self.parse_fn, self.qr_data)
                    if parse:
                        if self.say_fn and self.success_msg:
                            self.say_fn(self.success_msg)
                        userdata.command = next(parse, None) #parse[0]orNone
                        return 'success'
                    rospy.sleep(.2)
                    elapsed += .2
            return 'timeout'
        except:
            rospy.logerr(traceback.format_exc())
            return 'failure'

    def image_cb(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        retval, decoded_info, points, straight_qrcode = self.qr_reader.detectAndDecodeMulti(img)

        if retval:
            for dec_inf, point in zip(decoded_info, points):
                if dec_inf == '':
                    continue
                self.qr_data.append(dec_inf)


from ros_whisper_vosk.msg import speech_recognition
from ros_whisper_vosk.srv import SetGrammarVosk
from std_msgs.msg import String
from rospy.exceptions import ROSException

class SpeechRecognitionVoskClient():
    def __init__(self, speech_topic='/speech_recognition/final_result',
                 set_grammar_srv='set_grammar_vosk'):

        self.speech_topic = speech_topic

        rospy.loginfo("waiting for service -> " + set_grammar_srv)
        rospy.wait_for_service(set_grammar_srv)
        self.set_grammar = rospy.ServiceProxy(set_grammar_srv, SetGrammarVosk)

        # self.yes_list = ['Yes', 'Yes it is', 'Yes I do', 'Yes I am', 'correct']
        # self.no_list = ['No', "No it's not", "No I don't", "No I'm not"]

        self.yes_list = ['robot yes']
        self.no_list = ['robot no']

    def get_next_speech(self, timeout=None):
        result = None
        try:
            msg = rospy.wait_for_message(self.speech_topic, String, timeout)
            result = msg.data
            
        except ROSException:
            rospy.loginfo('timeout')

        return result

    def get_next_yes_or_no(self, timeout=None):
        self.set_grammar(self.yes_list + self.no_list)
        rospy.sleep(2)
        result = self.get_next_speech(timeout)
        print(result)
        if result in self.yes_list:
            return 'yes'
        elif result in self.no_list:
            return 'no'
        else:
            return None

        
if __name__ == "__main__":
    rospy.init_node("test_command")

    default_tts = speech.DefaultTTS()
    console = speech.Console()
    SAY = default_tts.say
    SAY("HELLO")
    vosk_interface = SpeechRecognitionVoskClient()

    OBJ_LIST = ["apple", "banana", "kiwi", "peach", "prum", "pear", "pringles"]

    def listen_obj_fn(timeout):
        myobjlist = OBJ_LIST
        sentence = ["bring me the {}", "bring me {}", "{}", "I need {}"]
        grammar = []
        for obj in myobjlist:
            for sent in sentence:
                grammar.append(sent.format(obj))

        vosk_interface.set_grammar(grammar)
        rospy.sleep(2) # temporary inset sleep, need change to start recognize ros service
        return vosk_interface.get_next_speech(timeout)

    def parse_obj_fn(text):
        if not text:
            return None
        for obj_item in OBJ_LIST:
            if obj_item in text:
                return obj_item
        return None

    sm = smach.StateMachine(outcomes=['success', 'failure'])
    with sm:
        smach.StateMachine.add('LISTENORDER', GetCommand(listen_obj_fn,
                                                         parse_obj_fn,
                                                         timeout=5.,
                                                         say_fn=SAY,
						         retry=1),
                               transitions = {'success': 'CONFIRM',
                                              'bypass': 'failure',
                                              'timeout': 'failure',
                                              'failure': 'failure'})

        @smach.cb_interface(outcomes=['success','failure'],
                            input_keys=['command'])
        def confirm_cb(userdata):
            try:
                print('------------------------------')
                print(userdata.command)
                order_item = parse_obj_fn(userdata.command)
                SAY("the order is {}".format(order_item))
                print('------------------------------')
                return 'success'
            except:
                traceback.print_exc()
                return "failure"

        smach.StateMachine.add('CONFIRM', smach.CBState(confirm_cb),
                               transitions={'success':'YESNO',
                                            'failure': 'failure'})

        @smach.cb_interface(outcomes=['yes','no', 'failure', 'none'],
                            input_keys=['command'])        
        def yes_no_cb(userdata):
            try:
                SAY("Please say yes or no")
                result = vosk_interface.get_next_yes_or_no()
                rospy.sleep(2.0)
                if result == "yes":
                    print("yes")
                    return "yes"
                elif result == "no":
                    print("yes")                    
                    return "no"
                else:
                    print("none")
                    return "none"

            except:
                traceback.print_exc()
                return "failure"

        smach.StateMachine.add('YESNO', smach.CBState(yes_no_cb),
                               transitions={'yes':'success',
                                            'no':'LISTENORDER',
                                            'none': 'YESNO',
                                            'failure':'failure'})
    
    sm.execute()
