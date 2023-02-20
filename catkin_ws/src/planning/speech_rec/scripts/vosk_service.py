#!/usr/bin/env python

import os
import sys
import json
import queue
import vosk
import sounddevice as sd
from mmap import MAP_SHARED

#ROS
import rospy
import rospkg
import std_msgs.msg
from ros_whisper_vosk.srv import GetSpeech, GetFakeSpeech

#VOSK
import vosk_ros_model_downloader as downloader

#Load model
rospack = rospkg.RosPack()
rospack.list()
package_path = rospack.get_path('ros_whisper_vosk')

model_path = '/models/'
model_dir = package_path + model_path
model = "vosk-model-small-en-us-0.15" #change the name of the model to match the downloaded model's name

if not os.path.exists(model_dir+model):
    print ("No model found! Please use the GUI to download a model...")
    model_downloader = downloader.model_downloader()
    model_downloader.execute()
    model = model_downloader.model_to_download

model_name = rospy.get_param('vosk/model',model)
if not rospy.has_param('vosk/model'):
    rospy.set_param('vosk/model', model_name)

input_dev_num = sd.query_hostapis()[0]['default_input_device']
if input_dev_num == -1:
    rospy.logfatal('No input device found')
    raise ValueError('No input device found, device number == -1')

device_info = sd.query_devices(input_dev_num, 'input')
# soundfile expects an int, sounddevice provides a float:

samplerate = int(device_info['default_samplerate'])
rospy.set_param('vosk/sample_rate', samplerate)

model = vosk.Model(model_dir+model_name)

#Start ROS Publishers
pub_final = rospy.Publisher('speech_recognition/final_result',std_msgs.msg.String, queue_size=10)

q = queue.Queue()  

#Callbacks
def stream_callback(indata, frames, time, status):
    #"""This is called (from a separate thread) for each audio block."""
    q.put(bytes(indata))
        
def callbackVoskService(req):

    predicted_text = ""
    with sd.RawInputStream(samplerate=samplerate, blocksize=16000, device=input_dev_num, dtype='int16',
                       channels=1, callback=stream_callback):
        rospy.logdebug('Started recording')
        
        rec = vosk.KaldiRecognizer(model, samplerate)
        print("Vosk is ready to listen!")
        isRecognized = False
        isRecognized_partially = False
              
        while not isRecognized:
            data = q.get()
            if rec.AcceptWaveform(data):

                # In case of final result
                result = rec.FinalResult()

                diction = json.loads(result)
                lentext = len(diction["text"])

                if lentext > 2:
                    predicted_text = diction["text"].lstrip()
                    print("You said: " + predicted_text)
                    isRecognized = True
                else:
                    isRecognized = False
                # Resets current results so the recognition can continue from scratch
                rec.Reset()
            else:
                # In case of partial result
                result_partial = rec.PartialResult()
                if (len(result_partial) > 20):

                    isRecognized_partially = True
                    partial_dict = json.loads(result_partial)
                    partial = partial_dict["partial"]

    pub_final.publish(predicted_text)
    return predicted_text

def callbackFakeVoskService(req):

    predicted_text = req.text
    predicted_text = predicted_text.lstrip()
    print("You said: " + predicted_text)

    pub_final.publish(predicted_text)
    return predicted_text

#Main
def main():
    rospy.init_node('vosk_service')
  
    #Start ROS Services
    rospy.Service("speech_recognition/vosk_service",GetSpeech, callbackVoskService)
    rospy.Service("speech_recognition/fake_vosk_service", GetFakeSpeech, callbackFakeVoskService)
    
    rospy.loginfo("Vosk Service Initialized")
    
    #Infinite loop
    rospy.spin()

if __name__ == "__main__":
    main()
