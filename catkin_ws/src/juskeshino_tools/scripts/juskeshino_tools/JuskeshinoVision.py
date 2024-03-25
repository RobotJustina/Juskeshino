import rospy
from std_msgs.msg import *
from sensor_msgs.msg import PointCloud2
from vision_msgs.srv import *
from vision_msgs.msg import *

class JuskeshinoVision:
    def setNodeHandle():
        print("JuskeshinoVision.->Setting ros node...")
        JuskeshinoVision.cltFindLines           = rospy.ServiceProxy("/vision/line_finder/find_table_edge",             FindLines           )
        JuskeshinoVision.cltFindHoriPlanes      = rospy.ServiceProxy("/vision/line_finder/find_horizontal_plane_ransac",FindPlanes          )
        JuskeshinoVision.cltTrainObject         = rospy.ServiceProxy("/vision/obj_reco/detect_and_train_object",        TrainObject         )
        JuskeshinoVision.cltDetectRecogObjects  = rospy.ServiceProxy("/vision/obj_reco/detect_and_recognize_objects",   RecognizeObjects    )
        JuskeshinoVision.cltDetectRecogObject   = rospy.ServiceProxy("/vision/obj_reco/detect_and_recognize_object",    RecognizeObject     )
        JuskeshinoVision.cltGetObjectPose       = rospy.ServiceProxy("/vision/obj_segmentation/get_obj_pose",           RecognizeObject     ) 
        JuskeshinoVision.cltGetPointsAbovePlane = rospy.ServiceProxy("/vision/get_points_above_plane",                  PreprocessPointCloud)
        JuskeshinoVision.pubHumanPoseEnable     = rospy.Publisher("/vision/human_pose/enable", Bool, queue_size=1)
        JuskeshinoVision.pubHumanDetectorEnable     = rospy.Publisher("/vision/human_pose/enable", Bool, queue_size=1)

        #ros::Publisher  pub = n.advertise<std_msgs::Bool>("human_detector_bool", 1);
        rospy.Subscriber('/vision/pointing_hand/status', Bool ,JuskeshinoVision.callbackPointingHand)
        JuskeshinoVision.pointing_hand = Bool()

        loop = rospy.Rate(10)
        counter = 3
        while not rospy.is_shutdown() and counter > 0:
            counter-=1
            loop.sleep()
        return True
    

    def pointingHand():
        return JuskeshinoVision.pointing_hand


    def callbackPointingHand(msg):
        JuskeshinoVision.pointing_hand = msg

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
            reqObjPose = RecognizeObjectRequest()
            reqObjPose.point_cloud = resp.recog_object.point_cloud
            reqObjPose.image       = resp.recog_object.image
            reqObjPose.obj_mask    = resp.recog_object.obj_mask
            respObjPose = JuskeshinoVision.cltGetObjectPose(reqObjPose)
            return [respObjPose.recog_object, resp.image]
        except:
            print("JuskeshinoVision.->Cannot detect and recognize object '" + name + "'")
            return [None, None]

    def enableHumanPoseDetection(enable):
        msg = Bool()
        msg.data = enable
        JuskeshinoVision.pubHumanPoseEnable.publish(msg)
