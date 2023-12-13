#!/usr/bin/env python3



from cv_bridge import CvBridge
#from object_classification.srv import Classify,ClassifyResponse, ClassifyRequest
from face_recog .msg import *
from face_recog .srv import *
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import String
import rospy
import numpy as np
import tf
import tf2_ros
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
#import face_recognition
from deepface import DeepFace
import cv2
import os



from rospkg import RosPack

rp = RosPack()
path_for_faces = rp.get_path('config_files')+'/faces_for_recognition/'
#path_for_faces='/home/takeshi/Pictures/faces_for_recognition/'
#path_for_faces='/home/roboworks/Pictures/faces_for_recognition/'



def callback_2(req):
    global path_for_faces , encodings , ids
    
    
    print ('got ',len(req.in_.image_msgs),'images Train New ID ') 
    print ('got ', len (req.Ids.ids),'names') 
    



    """images=[]
    new_names=[]
    for i in range(len(req.in_.image_msgs)):
        images.append(bridge.imgmsg_to_cv2(req.in_.image_msgs[i]))
        print (req.Ids.ids[i].data)
        if req.Ids.ids[i].data in os.listdir(path_for_faces):print('ID ALREADY ASSIGNED , please choose another name')
        else:
            os.mkdir(path_for_faces+'/'+req.Ids.ids[i].data)
            image= cv2.cvtColor(bridge.imgmsg_to_cv2(req.in_.image_msgs[i]), cv2.COLOR_BGR2RGB)
            cv2.imwrite(path_for_faces+'/'+req.Ids.ids[i].data+'/'+req.Ids.ids[i].data+'.jpg',image)
            


    """
        


    images=[]
    new_names=[]
    for i in range(len(req.in_.image_msgs)):
        images.append(bridge.imgmsg_to_cv2(req.in_.image_msgs[i]))
        print (req.Ids.ids[i].data)
        image= cv2.cvtColor(bridge.imgmsg_to_cv2(req.in_.image_msgs[i]), cv2.COLOR_BGR2RGB)
        if req.Ids.ids[i].data in os.listdir(path_for_faces):
            print('ID ALREADY ASSIGNED , adding image caution advised',str(len(os.listdir(path_for_faces+'/'+req.Ids.ids[i].data))))
            cv2.imwrite(path_for_faces+'/'+req.Ids.ids[i].data+'/'+req.Ids.ids[i].data+str(len(os.listdir(path_for_faces+'/'+req.Ids.ids[i].data)))+'.jpg',image)
        else:
            os.mkdir(path_for_faces+'/'+req.Ids.ids[i].data)
            cv2.imwrite(path_for_faces+'/'+req.Ids.ids[i].data+'/'+req.Ids.ids[i].data+'.jpg',image)
            print('New ID added ')





    ############Write Response message
    Ds, Rots=Floats(),Floats()
    strings=Strings()

    string_msg= String()
    string_msg.data='trained new id'+'_'+req.Ids.ids[i].data
    strings.ids.append(string_msg)
    Ds.data=[0.0,0.0]
    Rots.data=[1,1]
    #print ('Predictions (top 3 for each class)',flo.data)

    return RecognizeFaceResponse(Ds,Rots,strings)          

def callback(req):
    global path_for_faces , encodings , ids
    
    
    print ('got ',len(req.in_.image_msgs),'images')    
    images=[]
    distances = []
    Dstoface=[]
    Angs=[]
    names=[]


    for i in range(len(req.in_.image_msgs)):
        images.append(bridge.imgmsg_to_cv2(req.in_.image_msgs[i]))
    for image in images:
        print (image.shape)
        try:
            res=DeepFace.extract_faces(image )
            print ('face found h')
            dfs = DeepFace.find(image,path_for_faces)
            print('id->',dfs[0]['identity'].iloc[1].split('/')[-2])
            name=dfs[0]['identity'].iloc[1].split('/')[-2]
            names.append(name)

    


        except(ValueError): 
            print('No Face h')
            Ds, Rots=Floats(),Floats()  
                                    ###DEFINITION RESPONSE
            strings=Strings()
            string_msg= String()
            string_msg.data= 'NO_FACE'
            strings.ids.append(string_msg)
            Dstoface.append(0.0)
            Ds.data= Dstoface
            Angs.append(0.0)
            Rots.data= Angs
            return RecognizeFaceResponse(Ds,Rots,strings)        

    
##########################################################Wierd order because of dlib server compatibility
        Angs.append( res[0]['facial_area']['y'] )
        Angs.append( res[0]['facial_area']['h'] )
        Angs.append( res[0]['facial_area']['w'] )
        Angs.append( res[0]['facial_area']['x'] )
        
        ############Write Response message
        Ds, Rots=Floats(),Floats()
        strings=Strings()
        for name in names:    
            string_msg= String()
            string_msg.data=name
            strings.ids.append(string_msg)
        Ds.data=Dstoface
        Rots.data=Angs
        return RecognizeFaceResponse(Ds,Rots,strings)        
   
def callback_3(req):
    global path_for_faces , encodings , ids
    images=[]
    for i in range(len(req.in_.image_msgs)):
        images.append(bridge.imgmsg_to_cv2(req.in_.image_msgs[i]))
    for image in images:
        print (image.shape)
        try:
            res=DeepFace.extract_faces(image)
            objs = DeepFace.analyze(image, actions = ['age', 'gender', 'race', 'emotion'] ) 

            ############Write Response message
            Ds, Rots=Floats(),Floats()
            strings=Strings()
            Angs=[]
            Dstoface=[]

            for st in (objs[0]['dominant_gender'],objs[0]['dominant_race'],objs[0]['dominant_emotion'],str(objs[0]['age'])):
                print (st)
                string_msg= String()
                string_msg.data=st
                strings.ids.append(string_msg)

            #strings.ids.append(str( objs[0]['age']) )

            ##########################################################Wierd order because of dlib server compatibility
            Angs.append( res[0]['facial_area']['y'] )
            Angs.append( res[0]['facial_area']['h'] )
            Angs.append( res[0]['facial_area']['w'] )
            Angs.append( res[0]['facial_area']['x'] )


            Rots.data=Angs
            return RecognizeFaceResponse(Ds,Rots,strings)        

        except(ValueError): 
            print('No Face')
            Ds, Rots=Floats(),Floats()                          ###DEFINITION RESPONSE
            strings=Strings()
            Dstoface=[]
            Angs=[]
            string_msg= String()
            string_msg.data= 'NO_FACE'
            strings.ids.append(string_msg)
            Dstoface.append(0.0)
            Ds.data= Dstoface
            Angs.append(0.0)
            Rots.data= Angs
            return RecognizeFaceResponse(Ds,Rots,strings)        


def classify_server():
    global listener, bridge
    rospy.init_node('face_analysis_deep_available')

    bridge = CvBridge()
    listener = tf.TransformListener()
    broadcaster= tf.TransformBroadcaster()
    tf_static_broadcaster= tf2_ros.StaticTransformBroadcaster()
    rospy.loginfo("Face Recognition service available")                    # initialize a ROS node
    #s = rospy.Service('recognize_face', RecognizeFace, callback) 
    #s2 = rospy.Service('new_face', RecognizeFace, callback_2) 
    s3 = rospy.Service('analyze_face', RecognizeFace, callback_3) 
    
   

    rospy.spin()

if __name__ == "__main__":
    classify_server()
