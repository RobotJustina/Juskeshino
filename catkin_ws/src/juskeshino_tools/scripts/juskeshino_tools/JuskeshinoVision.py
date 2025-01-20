import rospy
import numpy as np
import ros_numpy
from cv_bridge import CvBridge
from std_msgs.msg import *
from sensor_msgs.msg import PointCloud2, Image as ImageMsg
from vision_msgs.srv import *
from vision_msgs.msg import *
from face_recog.msg import *
from face_recog.srv import *

class JuskeshinoVision:
    def setNodeHandle():
        print("JuskeshinoVision.->Setting ros node...")

        JuskeshinoVision.recognize_face             = rospy.ServiceProxy('recognize_face', RecognizeFace) 
        JuskeshinoVision.train_new_face             = rospy.ServiceProxy('new_face', RecognizeFace) 
        #JuskeshinoVision.cltFindLines               = rospy.ServiceProxy("/vision/line_finder/find_table_edge",             FindLines           )
        #JuskeshinoVision.cltFindHoriPlanes          = rospy.ServiceProxy("/vision/line_finder/find_horizontal_plane_ransac",FindPlanes          )
        #JuskeshinoVision.cltTrainObject             = rospy.ServiceProxy("/vision/obj_reco/detect_and_train_object",        TrainObject         )
        JuskeshinoVision.cltDetectRecogObjects      = rospy.ServiceProxy("/vision/obj_reco/detect_and_recognize_objects",   RecognizeObjects    )
        JuskeshinoVision.cltDetectRecogObject       = rospy.ServiceProxy("/vision/obj_reco/detect_and_recognize_object",    RecognizeObject     )
        JuskeshinoVision.cltGetObjectPose           = rospy.ServiceProxy("/vision/obj_segmentation/get_obj_pose",  RecognizeObject     ) 
        #JuskeshinoVision.cltGetPointsAbovePlane     = rospy.ServiceProxy("/vision/get_points_above_plane",                  PreprocessPointCloud)
        JuskeshinoVision.cltFindPersons             = rospy.ServiceProxy('/vision/face_reco_pkg/recognize_face/names',      FaceRecog           )
        JuskeshinoVision.cltTrainPersons            = rospy.ServiceProxy("/vision/face_reco_pkg/training_face/name",        FaceTrain           )
        JuskeshinoVision.cltClothesColor            = rospy.ServiceProxy("/vision/clothes_color",                           FindPerson          )

        JuskeshinoVision.pubHumanPoseEnable         = rospy.Publisher("/vision/human_pose/enable", Bool, queue_size=1)
        rospy.Subscriber("/vision/human_pose_estimation/human_detector_bool", Bool ,JuskeshinoVision.callbackHumanBool)
        rospy.Subscriber("/vision/human_pose_estimation/pointing_hand/status", Bool ,JuskeshinoVision.callbackPointingHand)
        
        JuskeshinoVision.pointing_hand  = Bool()
        JuskeshinoVision.human_detector = Bool()
        JuskeshinoVision.bridge = CvBridge()

        loop = rospy.Rate(10)
        counter = 3
        while not rospy.is_shutdown() and counter > 0:
            counter-=1
            loop.sleep()
        return True
    

    def callbackPointingHand(msg):
        JuskeshinoVision.pointing_hand = msg

    def pointingHand():
        return JuskeshinoVision.pointing_hand

    def callbackHumanBool(msg):
        JuskeshinoVision.human_detector = msg

    def humanDetector():
        return JuskeshinoVision.human_detector

    def enableHumanPose(enable):
        msg = Bool()
        msg.data = enable
        JuskeshinoVision.pubHumanPoseEnable.publish(msg)
        return

    def findTableEdge():
        req = FindLinesRequest()
        req.point_cloud = rospy.wait_for_message("/camera/depth_registered/points", PointCloud2, timeout=1.0)
        try:
            resp = JuskeshinoVision.cltFindLines(req)
            return resp.lines
        except:
            print("JuskeshinoVision.->Cannot find table edge :'(")
            return None
    
    def clothes_color():
        req = FindPerson()
        req.cloud = rospy.wait_for_message("/camera/depth_registered/points", PointCloud2, timeout=1.0)
        try:
            resp = JuskeshinoVision.cltClothesColor(req)
            return [resp.person.shirt, resp.person.pants]
        except:
            print("JuskeshinoVision.->Cannot detect and recognize clothes color")
            return [None, None]


    

    def detectAndRecognizeObjects():
        req = RecognizeObjectsRequest()
        req.point_cloud = rospy.wait_for_message("/camera/depth_registered/points", PointCloud2, timeout=1.0)
        try:
            resp = JuskeshinoVision.cltDetectRecogObjects(req)
            return [resp.recog_objects, resp.image]
        except:
            print("JuskeshinoVision.->Cannot detect and recognize objects :'(")
            return [None, None]

    def detectAndRecognizeObject(name):
        req = RecognizeObjectRequest()
        req.point_cloud = rospy.wait_for_message("/camera/depth_registered/points", PointCloud2, timeout=1.0)
        req.name = name
        try:
            resp = JuskeshinoVision.cltDetectRecogObject(req)
            #print(resp.recog_object.point_cloud)
            reqObjPose = RecognizeObjectRequest()
            reqObjPose.point_cloud = resp.recog_object.point_cloud
            reqObjPose.image       = resp.recog_object.image
            reqObjPose.obj_mask    = resp.recog_object.obj_mask
            reqObjPose.name        = resp.recog_object.id
            respObjPose = JuskeshinoVision.cltGetObjectPose(reqObjPose)
            return [respObjPose.recog_object, resp.image]
        except:
            print("JuskeshinoVision.->Cannot detect and recognize object '" + name + "'")
            return [None, None]

    def getObjectOrientation(obj): #A vision object is expected
        reqObjPose = RecognizeObjectRequest()
        reqObjPose.point_cloud = obj.point_cloud
        reqObjPose.image       = obj.image
        reqObjPose.obj_mask    = obj.obj_mask
        respObjPose = JuskeshinoVision.cltGetObjectPose(reqObjPose)
        newObj = obj
        newObj.object_state = respObjPose.recog_object.object_state
        newObj.category = respObjPose.recog_object.category
        newObj.graspable = respObjPose.recog_object.graspable
        newObj.size = respObjPose.recog_object.size
        newObj.pose = respObjPose.recog_object.pose
        return newObj
        

    def detectAndRecognizeObjectWithoutOrientation(name):
        req = RecognizeObjectRequest()
        req.point_cloud = rospy.wait_for_message("/camera/depth_registered/points", PointCloud2, timeout=1.0)
        req.name = name
        try:
            resp = JuskeshinoVision.cltDetectRecogObject(req)
            return [resp.recog_object, resp.image]
        except:
            print("JuskeshinoVision.->Cannot detect and recognize object '" + name + "'")
            return [None, None]


    def enableRecogFacesName(flag):
        req = FaceRecogRequest()
        req.is_face_recognition_enabled = flag
        resp = JuskeshinoVision.cltFindPersons.call(req)

        if (resp):
            for name in resp.names:
                print(name)

            name_recog =resp.names
            return name_recog
    
        else:
            print("JuskeshinoVision.->vacio")
            vector_vacio = resp.names
            return vector_vacio
    
    
    def trainingPerson(person):
        print("JuskeshinoVision.->Train person: ", person)
        req = FaceTrainRequest()
        req.name.data = person
        resp = JuskeshinoVision.cltTrainPersons.call(req)
        if (resp):
            print("Success ", resp.success)
            print(resp.message)

    #****Reconocimiento de rostros usando codigo de los takeshis

    def train_face(image, name):
        """writes request message and requests trainface service
                /face_recog pkg"""
        rospy.sleep(3)
        req = RecognizeFaceRequest()
        strings = Strings()
        string_msg = String()
        string_msg.data = name
        req.Ids.ids.append(string_msg)

        img_msg = image#JuskeshinoVision.bridge.cv2_to_imgmsg(image)
        req.in_.image_msgs.append(img_msg)
        res = JuskeshinoVision.train_new_face(req)

        return res.Ids.ids[0].data.split(' ')[0] == 'trained'

    def wait_for_face(timeout=10 , name='', lap_camera=False):
        
        rospy.sleep(3)
        
        start_time = rospy.get_time()
        strings=Strings()
        string_msg= String()
        string_msg.data='Anyone'
        while rospy.get_time() - start_time < timeout:
            cloud = rospy.wait_for_message("/camera/depth_registered/points", PointCloud2, timeout=7.0)
            array = ros_numpy.numpify(cloud)
            img = array['rgb'].view((np.uint8, 4))[..., [2, 1, 0]]

            if lap_camera:print("USB camera")

            req=RecognizeFaceRequest()
            print ('Got  image with shape',img.shape)
            req.Ids.ids.append(string_msg)
            img_msg=JuskeshinoVision.bridge.cv2_to_imgmsg(img[:,150:-150])
            req.in_.image_msgs.append(img_msg)

            res= JuskeshinoVision.recognize_face(req)

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


    def analyze_face_background(img, name=" "):
        name_pub = rospy.Publisher('/name_face', String, queue_size=10)
        img_pub = rospy.Publisher('/image_to_analyze', ImageMsg, queue_size=10)
        str_msg = String()
        str_msg.data = name
        rospy.sleep(0.5)
        name_pub.publish(str_msg)
        img_msg = JuskeshinoVision.bridge.cv2_to_imgmsg(img)
        img_pub.publish(img_msg)
