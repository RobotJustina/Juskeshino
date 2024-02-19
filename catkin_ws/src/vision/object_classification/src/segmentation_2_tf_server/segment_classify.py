#! /usr/bin/env python3
import rospy  
import tf2_ros                                    # the main module for ROS-python programs
from std_srvs.srv import Trigger, TriggerResponse # we are creating a 'Trigger service'...
                                                  # ...Other types are available, and you can create
                                                  # custom types
from utils import *

def images_to_request(images):
    req=classify_client.request_class()
    for image in images:
        img_msg=bridge.cv2_to_imgmsg(image)
        req.in_.image_msgs.append(img_msg)
    
    
    return req


def trigger_response(request):
    ''' 
    Callback function used by the service server to process
    requests from clients. It returns a TriggerResponse
    '''
    print ('Segmenting')
    points_msg=rospy.wait_for_message("/hsrb/head_rgbd_sensor/depth_registered/rectified_points",PointCloud2,timeout=5)
    points_data = ros_numpy.numpify(points_msg)    
    image_data = points_data['rgb'].view((np.uint8, 4))[..., [2, 1, 0]]   
    image=cv2.cvtColor(image_data, cv2.COLOR_BGR2RGB)
    print (image.shape)
    
    
    cents,xyz, images, img = plane_seg( points_msg,lower=100    , higher=4000,reg_hy=350)
    req = images_to_request(images)
    print ('request',req)


    print(len(cents))
    for i,cent in enumerate(cents):
                print (cent)
                x,y,z=cent
                if np.isnan(x) or np.isnan(y) or np.isnan(z):
                    print('nan')
                else:
                    broadcaster.sendTransform((x,y,z),(0,0,0,1), rospy.Time.now(), 'Object'+str(i),"head_rgbd_sensor_rgb_frame")
            
    

    

	    


    return TriggerResponse(
        success=True,
        message= 'ALARM '
    )

global classify_client
rospy.loginfo("segmentation service available")                    # initialize a ROS node
classify_client = rospy.ServiceProxy('/classify', Classify)
my_service = rospy.Service(                        # create a service, specifying its name,
    '/segment_classify', Trigger, trigger_response         # type, and callback
)
rospy.spin()   
