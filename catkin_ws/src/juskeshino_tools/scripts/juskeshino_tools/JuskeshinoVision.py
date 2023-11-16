import rospy
from sensor_msgs.msg import PointCloud2
from vision_msgs.srv import *
from vision_msgs.msg import *

class JuskeshinoVision:
    def setNodeHandle():
        print("JuskeshinoVision.->Setting ros node...")
        JuskeshinoVision.cltFindLines           = rospy.ServiceProxy("/vision/line_finder/find_table_edge",             FindLines           )
        JuskeshinoVision.cltFindHoriPlanes      = rospy.ServiceProxy("/vision/line_finder/find_horizontal_plane_ransac",FindPlanes          )
        JuskeshinoVision.cltTrainObject         = rospy.ServiceProxy("/vision/obj_reco/detect_and_train_object",        TrainObject         )
        JuskeshinoVision.cltRecogObjects        = rospy.ServiceProxy("/vision/obj_reco/detect_and_recognize_objects",   RecognizeObjects    )
        JuskeshinoVision.cltRecogObject         = rospy.ServiceProxy("/vision/obj_reco/detect_and_recognize_object",    RecognizeObject     )
        JuskeshinoVision.cltGetPointsAbovePlane = rospy.ServiceProxy("/vision/get_points_above_plane",                  PreprocessPointCloud)
        
        loop = rospy.Rate(10)
        counter = 3;
        while not rospy.is_shutdown() and counter > 0:
            counter-=1
            loop.sleep()
        return True

    def findTableEdge():
        req = FindLinesRequest()
        req.point_cloud = rospy.wait_for_message("/camera/depth_registered/points", PointCloud2, timeout=1.0)
        try:
            resp = JuskeshinoVision.cltFindLines(req)
            return resp.lines
        except:
            print("JuskeshinoVision.->Cannot find table edge :'(")
            return None

    
