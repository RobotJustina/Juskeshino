#! /usr/bin/env python3

                                                 
from utils import *



def trigger_response(request):
    
    print ('Segmenting')
    points_msg=rospy.wait_for_message("/hsrb/head_rgbd_sensor/depth_registered/rectified_points",PointCloud2,timeout=5)
    # <<<<<<<<<<<<<<<<<<<<<<ANTERIOR SEGMENTADOR>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    # points_data = ros_numpy.numpify(points_msg)    
    # image_data = points_data['rgb'].view((np.uint8, 4))[..., [2, 1, 0]]   
    # image=cv2.cvtColor(image_data, cv2.COLOR_BGR2RGB)
    # print (image.shape)
    # cents,xyz, images, img = plane_seg( points_msg,lower=10    , hi


    res= detect_human(points_msg)
    # IN PROGRESS
    #res = detect_all(points_msg)
    print (res)
    return res
    

rospy.loginfo("human detection service available")                    # initialize a ROS node
my_service = rospy.Service(                        # create a service, specifying its name,
    '/detect_human', Human_detector, trigger_response         # type, and callback
)
rospy.spin()   
