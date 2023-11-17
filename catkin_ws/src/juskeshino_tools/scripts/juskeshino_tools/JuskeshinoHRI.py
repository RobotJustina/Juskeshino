import rospy
from hri_msgs.msg import *
from sound_play.msg import SoundRequest

class JuskeshinoHRI:
    def setNodeHandle():
        print("JuskeshinoHRI.->Setting ros node...")
        rospy.Subscriber("/hri/sp_rec/recognized", RecognizedSpeech, JuskeshinoHRI.callbackRecognizedSpeech)
        JuskeshinoHRI.pubSoundRequest = rospy.Publisher("/hri/speech_generator", SoundRequest, queue_size=10)

        JuskeshinoHRI.recognizedSpeech = RecognizedSpeech()
        loop = rospy.Rate(10)
        counter = 3;
        while not rospy.is_shutdown() and counter > 0:
            counter-=1
            loop.sleep()
        return True

    def callbackRecognizedSpeech(msg):
        JuskeshinoHRI.recognizedSpeech = msg

    def getLastRecognizedSentence():
        if len(JuskeshinoHRI.recognizedSpeech.hypothesis) < 1:
            return None
        return JuskeshinoHRI.recognizedSpeech.hypothesis[0]

    def say(text, voice="voice_kal_diphone"):
        msg = SoundRequest()
        msg.sound   = -3
        msg.command = 1
        msg.volume  = 1.0
        msg.arg2    = voice
        msg.arg     = text
        JuskeshinoHRI.pubSoundRequest.publish(msg)
