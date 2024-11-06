import rospy
from std_msgs.msg import *
from sensor_msgs.msg import PointCloud2
from vision_msgs.srv import *
from vision_msgs.msg import *

class JuskeshinoVision:
    def setNodeHandle():
        print("JuskeshinoVision.->Setting ros node...")
        JuskeshinoVision.cltFindLines               = rospy.ServiceProxy("/vision/line_finder/find_table_edge",             FindLines           )
        JuskeshinoVision.cltFindHoriPlanes          = rospy.ServiceProxy("/vision/line_finder/find_horizontal_plane_ransac",FindPlanes          )
        JuskeshinoVision.cltTrainObject             = rospy.ServiceProxy("/vision/obj_reco/detect_and_train_object",        TrainObject         )
        JuskeshinoVision.cltDetectRecogObjects      = rospy.ServiceProxy("/vision/obj_reco/detect_and_recognize_objects",   RecognizeObjects    )
        JuskeshinoVision.cltDetectRecogObject       = rospy.ServiceProxy("/vision/obj_reco/detect_and_recognize_object",    RecognizeObject     )
        JuskeshinoVision.cltGetObjectPose           = rospy.ServiceProxy("/vision/obj_segmentation/get_obj_pose",  RecognizeObject     ) 
        JuskeshinoVision.cltGetPointsAbovePlane     = rospy.ServiceProxy("/vision/get_points_above_plane",                  PreprocessPointCloud)
        JuskeshinoVision.cltFindPersons             = rospy.ServiceProxy('/vision/face_reco_pkg/recognize_face/names',      FaceRecog           )
        JuskeshinoVision.cltTrainPersons            = rospy.ServiceProxy("/vision/face_reco_pkg/training_face/name",        FaceTrain           )

        JuskeshinoVision.pubHumanPoseEnable         = rospy.Publisher("/vision/human_pose/enable", Bool, queue_size=1)

        rospy.Subscriber("/vision/human_pose_estimation/human_detector_bool", Bool ,JuskeshinoVision.callbackHumanBool)
        rospy.Subscriber("/vision/human_pose_estimation/pointing_hand/status", Bool ,JuskeshinoVision.callbackPointingHand)
        
        JuskeshinoVision.pointing_hand  = Bool()
        JuskeshinoVision.human_detector = Bool()

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
        newObj = obj;
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

