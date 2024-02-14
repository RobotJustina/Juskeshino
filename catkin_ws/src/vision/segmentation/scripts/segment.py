#! /usr/bin/env python3

                                                 
from utils import *



def trigger_response(request):
    ''' 
    Callback function used by the service server to process
    requests from clients. It returns a TriggerResponse
    '''
    print ('Segmenting')
    points_msg=rospy.wait_for_message("/hsrb/head_rgbd_sensor/depth_registered/rectified_points",PointCloud2,timeout=5)
    
    cents,xyz, images, img = plane_seg2(points_msg)

    print(len(cents))

    #cv2.imshow("Segmented Image",img)
    #cv2.waitKey(0)
    #cv2.destroyAllWindows()
    for i,cent in enumerate(cents):
        print (cent)
        x,y,z=cent
        if np.isnan(x) or np.isnan(y) or np.isnan(z):
            print('nan')
        else:
            t=write_tf(    (x,y,z),(0,0,0,1), 'Object'+str(i), "head_rgbd_sensor_rgb_frame"     )
            broadcaster.sendTransform(t)
           
    pose, quat=Floats(),Floats()
    res= SegmentationResponse()
    pose.data=np.asarray(cents).ravel()
    res.poses=pose
    
    return res
    

rospy.loginfo("segmentation service available")                    # initialize a ROS node
my_service = rospy.Service(                        # create a service, specifying its name,
    '/segment', Segmentation, trigger_response         # type, and callback
)
rospy.spin()   
