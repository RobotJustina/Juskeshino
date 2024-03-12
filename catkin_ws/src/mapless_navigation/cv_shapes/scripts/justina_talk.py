#!/usr/bin/env python
import rospy
import rospkg
import sensor_msgs.msg
from sound_play.msg import SoundRequest



def robot_say(text, voice="voice_cmu_us_slt_arctic_hts", publisher=None):
    msg = SoundRequest()
    msg.sound   = -3
    msg.command = 1
    msg.volume  = 1.0
    msg.arg     = text
    msg.arg2    = voice
    publisher.publish(msg)
    rospy.sleep(0.09*len(text))

def main():
    rospy.init_node('justina_talk')
    rospy.loginfo("justina_talk_node")

    message_str = "Hi there, I am Justina"

    pub_voice = rospy.Publisher("/hri/speech_generator", SoundRequest, queue_size=10)

    rate = rospy.Rate(.5)  # Hz

    
    while not rospy.is_shutdown():
        
        rospy.loginfo(message_str)
        robot_say(message_str, publisher=pub_voice)

        rate.sleep()



if __name__ == "__main__":
    main()
