#!/usr/bin/env python3



from cv_bridge import CvBridge
#from object_classification.srv import Classify,ClassifyResponse, ClassifyRequest
from face_recog.msg import *
from face_recog.srv import *
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import String
import rospy
import numpy as np
import tf
import tf2_ros
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
import face_recognition
import cv2
import os



from rospkg import RosPack

rp = RosPack()
path_for_faces = rp.get_path('config_files')+'/faces_for_recognition/'


#path_for_faces='/home/robocup/Pictures/faces_for_recognition/'
#path_for_faces='/home/roboworks/Pictures/faces_for_recognition/'

ids=[]
first= True

for person in os.listdir(path_for_faces):    
    for example in os.listdir(path_for_faces+person):
        if first:
            first= False
            dataset_pic=face_recognition.load_image_file(path_for_faces+person+'/'+example)
            encodings = face_recognition.face_encodings(dataset_pic)
            
        else:
            dataset_pic = face_recognition.load_image_file(path_for_faces+person+'/'+example)
            encodings.append(face_recognition.face_encodings(dataset_pic)[0])
        ids.append(person)
ids=np.asarray(ids)

#############################################################################

objpoints = np.array([
    (0.0, 0.0, 0.0),             # Nose tip
    (0.0, -80.0, -30),        # Chin
    (-40.0, 40.0, -30),     # Left eye left corner
                            (40.0, 40.0, -30),      # Right eye right corneq
                            (-30.0, -30.0, -30.0),    # Left Mouth corner
                            (30.0, -30.0, -30.0)      # Right mouth corner
])


def rect2sph(rectcoords):

    x=rectcoords[0]
    y=rectcoords[1]
    z=rectcoords[2]
    r= (x**2+y**2+z**2)**.5

    th=np.arctan(y/(x+.0000000001))
    phi= (np.arctan((x**2+y**2)**.5 )/ ( z+.000000001))
    sphrcoords=np.array([r,th,phi])
    return sphrcoords


def sph2rect(sphrcoords):
    r, th, phi = sphrcoords[0], sphrcoords[1], sphrcoords[2]
    x = r * np.sin(phi) * np.cos(th)
    y = r * np.sin(phi) * np.sin(th)
    z = r * np.cos(phi)

    rectcoords = np.array([x, y, z])
    return rectcoords


def callback_2(req):
    global path_for_faces , encodings , ids
    
    
    print ('got ',len(req.in_.image_msgs),'images Train New ID ') 
    print ('got ', len (req.Ids.ids),'names') 
    



    images=[]
    new_names=[]
    for i in range(len(req.in_.image_msgs)):
        images.append(bridge.imgmsg_to_cv2(req.in_.image_msgs[i]))
        print (req.Ids.ids[i].data)
        if req.Ids.ids[i].data in os.listdir(path_for_faces):print('ID ALREADY ASSIGNED , please choose another name')
        else:
            os.mkdir(path_for_faces+'/'+req.Ids.ids[i].data)
            image= cv2.cvtColor(bridge.imgmsg_to_cv2(req.in_.image_msgs[i]), cv2.COLOR_BGR2RGB)
            cv2.imwrite(path_for_faces+'/'+req.Ids.ids[i].data+'/'+req.Ids.ids[i].data+'.jpg',image)



        


    




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
    for i in range(len(req.in_.image_msgs)):
        images.append(bridge.imgmsg_to_cv2(req.in_.image_msgs[i]))
    for image in images:
        print (image.shape)
        face_locations = face_recognition.face_locations(image)
        distances = []
        Dstoface=[]
        Angs=[]
        names=[]
        if len (face_locations)==0:


            Ds, Rots=Floats(),Floats()                          ###DEFINITION RESPONSE
            strings=Strings()
            string_msg= String()
            string_msg.data= 'NO_FACE'
            strings.ids.append(string_msg)
            Dstoface.append(0.0)
            Ds.data= Dstoface
            Angs.append(0.0)
            Rots.data= Angs
            return RecognizeFaceResponse(Ds,Rots,strings)        


        if len (face_locations)>0:
            face_encodings = face_recognition.face_encodings(image, face_locations)
            face_landmarks = face_recognition.face_landmarks(image)
#####################################################################
            
            # FACEWRLDCOORDS######################################################3
            size = image.shape
            focal_length = size[1]
            center = (size[1] / 2, size[0] / 2)
            camera_matrix = np.array(
                [[focal_length, 0, center[0]],
                 [0, focal_length, center[1]],
                 [0, 0, 1]], dtype="double"
            )
            dist_coeffs = np.zeros((4, 1))  # Assuming no lens distortion
            print ("there are",len(face_landmarks))
            for face_landmark in face_landmarks:

                imgpoints = np.array([
                        face_landmark['nose_tip'][2],
                        face_landmark['chin'][8],
                        face_landmark['left_eye'][0],
                        face_landmark['right_eye'][3],
                        face_landmark['top_lip'][0],
                        face_landmark['top_lip'][6]
                    ], dtype="double")
                _, rotation_vector, translation_vector = cv2.solvePnP(
                objpoints, imgpoints, camera_matrix, dist_coeffs)
                z = focal_length
                x, y = imgpoints[0]
                xx = (x - camera_matrix[0, 2])
                yy = -(y - camera_matrix[1, 2])
                Ang = np.arctan(xx / focal_length)
                Crect = np.array([xx, yy, z])
                _, th, phi = rect2sph(Crect)
                if np.linalg.norm(rotation_vector) < 5:
                    Dtoface = 40 * focal_length / \
                        (np.abs(imgpoints[2][0] - imgpoints[3][0])*1000 / 2)
                    Dstoface.append(Dtoface)
                    print ("##############################")
                    print ("Dss", Dtoface)
                    
                else:
                    Dtoface = 40 * focal_length / \
                        (np.linalg.norm(imgpoints[2] - imgpoints[3])*1000 / 2)
                    Dstoface.append(Dtoface)
                    print ("$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$")
                    print ("Dss", Dtoface)
                Cpol = np.array([Dtoface, th, phi])
                Crect = sph2rect(Cpol)
                Ang = np.arctan(xx / focal_length)
                phi= np.arctan(yy/focal_length)
                Angs.append(Ang)
                
            print (Dstoface)
            print (Angs)
            names=[]
            for face_encoding in face_encodings:
                results = face_recognition.compare_faces(encodings, face_encoding)
                print ('results',results)
                if any(results) !=True: names.append('unknown')
                else:names.append(np.unique(ids[results])[0])
            print (names)
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



                
                
                
                






    #print ('Predictions (top 3 for each class)',flo.data)

    
  
    
    
    
    


def classify_server():
    global listener, bridge
    rospy.init_node('face_recognition_server')

    bridge = CvBridge()
    listener = tf.TransformListener()
    broadcaster= tf.TransformBroadcaster()
    tf_static_broadcaster= tf2_ros.StaticTransformBroadcaster()
    rospy.loginfo("Face Recognition service available")                    # initialize a ROS node
    s = rospy.Service('recognize_face', RecognizeFace, callback) 
    s2 = rospy.Service('new_face', RecognizeFace, callback_2) 
    
   

    rospy.spin()

if __name__ == "__main__":
    classify_server()
