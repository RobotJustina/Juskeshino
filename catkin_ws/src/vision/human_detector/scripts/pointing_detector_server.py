#! /usr/bin/env python3

                                                 
from utils_pointing import *



def trigger_response(request):    
    print ('Segmenting')
    #points_msg=rospy.wait_for_message("/hsrb/head_rgbd_sensor/depth_registered/rectified_points",PointCloud2,timeout=5)#TAKESHI
    points_msg=rospy.wait_for_message("/camera/depth_registered/points",PointCloud2,timeout=5)
    res= detect_pointing(points_msg)
    return res
def callback(request):
	print ('Segmenting')  
	points_msg=rospy.wait_for_message("/camera/depth_registered/points",PointCloud2,timeout=5)
	#points_msg=rospy.wait_for_message("/hsrb/head_rgbd_sensor/depth_registered/rectified_points",PointCloud2,timeout=5)#TAKESHI
	res= detect_human(points_msg)	
	print (res)
	return res    

rospy.loginfo("human pose detection service available")                    # initialize a ROS node
my_service = rospy.Service(                        # create a service, specifying its name,
    '/detect_pointing', Point_detector, trigger_response         # type, and callback
)
service2 = rospy.Service('/detect_human', Human_detector, callback ) 

rospy.spin()   
