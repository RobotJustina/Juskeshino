import rospy
from std_msgs.msg import *
from hri_msgs.msg import *
from sound_play.msg import SoundRequest

class JuskeshinoHRI:
    def setNodeHandle():
        print("JuskeshinoHRI.->Setting ros node...")
        rospy.Subscriber("/hri/sp_rec/recognized", RecognizedSpeech, JuskeshinoHRI.callbackRecognizedSpeech)
        rospy.Subscriber("/hri/leg_finder/legs_found" , Bool , JuskeshinoHRI.callbackLegsFound)

        JuskeshinoHRI.pubSoundRequest      = rospy.Publisher("/hri/speech_generator", SoundRequest, queue_size=10)
        JuskeshinoHRI.pubLegFinderEnable   = rospy.Publisher("/hri/leg_finder/enable", Bool, queue_size=10)
        JuskeshinoHRI.pubHFollowEnable     = rospy.Publisher("/hri/human_following/enable", Bool, queue_size=10)
        JuskeshinoHRI.pubHumanFollowerStop = rospy.Publisher("/stop", Empty, queue_size=1)

        JuskeshinoHRI.recognizedSpeech = RecognizedSpeech()
        JuskeshinoHRI.legsFound = Bool()

        loop = rospy.Rate(10)
        counter = 3
        while not rospy.is_shutdown() and counter > 0:
            counter-=1
            loop.sleep()
        return True
    
    def enableLegFinder(enable):
        if(not enable):
            print("JuskeshinoHRI.->Leg_finder disabled. ")
        else:
            print("JuskeshinoHRI.->Leg_finder enabled.")

        msg = Bool()
        msg.data = enable
        JuskeshinoHRI.pubLegFinderEnable.publish(msg)

    def frontalLegsFound():
        return JuskeshinoHRI.legsFound

    def waitForFrontalLegsFound(timeout):
        JuskeshinoHRI.legsFound = False
        JuskeshinoHRI.enableLegFinder(True)
        attempts = int(timeout/0.1);
        loop = rospy.Rate(10)
        while not rospy.is_shutdown() and not JuskeshinoHRI.legsFound:
            loop.sleep()
            attempts -= 1
        return JuskeshinoHRI.legsFound

    def callbackLegsFound(msg):
        #print("JuskeshinoHRI.->Legs found signal received!")
        JuskeshinoHRI.legsFound = msg

    def callbackRecognizedSpeech(msg):
        JuskeshinoHRI.recognizedSpeech = msg

    def getLastRecognizedSentence():
        if len(JuskeshinoHRI.recognizedSpeech.hypothesis) < 1:
            return None
        rec = JuskeshinoHRI.recognizedSpeech.hypothesis[0]
        JuskeshinoHRI.recognizedSpeech.hypothesis = tuple([])
        JuskeshinoHRI.recognizedSpeech.confidences = tuple([])
        return rec

    def clearRecognizedSentences():
        JuskeshinoHRI.recognizedSpeech.hypothesis = tuple([])
        JuskeshinoHRI.recognizedSpeech.confidences = tuple([])

    def waitForNewSentence(timeout):
        scaled = int(timeout/0.1)
        attempts = 0
        loop = rospy.Rate(10)
        JuskeshinoHRI.recognizedSpeech.hypothesis = tuple([])
        JuskeshinoHRI.recognizedSpeech.confidences = tuple([])
        while (not rospy.is_shutdown() and attempts < scaled):
            rec = JuskeshinoHRI.getLastRecognizedSentence()
            # print("****", rec)
            if rec is not None:
                return rec
            attempts += 1
            loop.sleep()
        return ''

    def startSay(text, voice="voice_cmu_us_slt_arctic_hts"):
        msg = SoundRequest()
        msg.sound   = -3
        msg.command = 1
        msg.volume  = 1.0
        msg.arg2    = voice
        msg.arg     = text
        JuskeshinoHRI.pubSoundRequest.publish(msg)

    def say(text, voice="voice_cmu_us_slt_arctic_hts"):
        JuskeshinoHRI.startSay(text, voice)
        rospy.sleep(0.085*len(text))

    def enableHumanFollower(enable):
        if(not enable):
            print("JuskeshinoHRI.->Human_follower disabled. ")
        else:
            print("JuskeshinoHRI.->Human_follower enabled.")

        msg = Bool()
        msg.data = enable
        JuskeshinoHRI.pubHFollowEnable.publish(msg)

    def stopHumanFollower():
        msg = Empty()
        JuskeshinoHRI.pubHumanFollowerStop.publish(msg)

    



        
