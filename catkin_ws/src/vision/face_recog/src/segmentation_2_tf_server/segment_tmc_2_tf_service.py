#! /usr/bin/env python
import rospy                                      # the main module for ROS-python programs
from std_srvs.srv import Trigger, TriggerResponse # we are creating a 'Trigger service'...
from tmc_tabletop_segmentator.srv import TabletopSegmentation
from tmc_tabletop_segmentator.srv import TabletopSegmentationRequest
from utils_srv import *

                                                  # ...Other types are available, and you can create
                                                  # custom types
def trigger_response(request):
    ''' 
    Callback function used by the service server to process
    requests from clients. It returns a TriggerResponse
    '''
###########################################################





#define a tabletop segmentation request.
# Play with these parameters

    listener = tf.TransformListener()
    broadcaster= tf.TransformBroadcaster()
    tf_static_broadcaster= tf2_ros.StaticTransformBroadcaster()
    bridge = CvBridge()

    service_client = rospy.ServiceProxy('/tabletop_segmentator_node/execute', TabletopSegmentation)
    service_client.wait_for_service(timeout=1.0)

    req = TabletopSegmentationRequest()
    req.crop_enabled = True  # limit the processing area
    req.crop_x_max = 0.7     # X coordinate maximum value in the area [m]
    req.crop_x_min = -0.7    # X coordinate minimum value in the area [m]
    req.crop_y_max = 1.0     # Y coordinate maximum value in the area [m]
    req.crop_y_min = -1.0    # Y coordinate minimum value in the area [m]
    req.crop_z_max = 1.1     # Z coordinate maximum value in the area [m]
    req.crop_z_min = 0.0     # Z coordinate minimum value in the area [m]
    req.cluster_z_max = 1.0  # maximum height value of cluster on table [m]
    req.cluster_z_min = 0.0  # minimum height value of cluster on table [m]
    req.remove_bg = True    # remove the background of the segment image

    res = service_client(req)

    rospy.loginfo('Number of detected objects={0}'.format(
        len(   res.segmented_objects_array.table_objects_array)))
    rospy.loginfo('Number of detected planes={0}'.format(
        len(res.table_array.tables)))
    objs_depth_centroids=[]
    for i in range (len(res.segmented_objects_array.table_objects_array )):
        print ( 'Plane',i,'has', len(res.segmented_objects_array.table_objects_array[i].depth_image_array), 'objects')
        for j in range (len(res.segmented_objects_array.table_objects_array[i].points_array)):
            cv2_img_depth = bridge.imgmsg_to_cv2(res.segmented_objects_array.table_objects_array[i].depth_image_array[0] )
            cv2_img = bridge.imgmsg_to_cv2(res.segmented_objects_array.table_objects_array[i].rgb_image_array[0] )
            pc= ros_numpy.numpify (res.segmented_objects_array.table_objects_array[i].points_array[j])
            points=np.zeros((pc.shape[0],3))
            points[:,0]=pc['x']
            points[:,1]=pc['y']
            points[:,2]=pc['z']
            objs_depth_centroids.append(np.mean(points,axis=0))
    objs_depth_centroids= sort_list (objs_depth_centroids)
    for i in range(len(objs_depth_centroids)):
        #Table is a plane at z=.8 So we consider false positives all the centroids outside the region on axis z ( .79 , .9)
        if objs_depth_centroids[i][2] > 0 and objs_depth_centroids[i][2] <44.9: 
            static_transformStamped = geometry_msgs.msg.TransformStamped()
            broadcaster.sendTransform((objs_depth_centroids[i]),(0,0,0,1), rospy.Time.now(), 'Object'+str(i),"head_rgbd_sensor_link")


    return TriggerResponse(
        success=True,
        message="Segmentation requested"
    )

rospy.init_node('segment_2_tf_service')                     # initialize a ROS node
segment_tmc_tabletop = rospy.Service(                        # create a service, specifying its name,
    '/segment_2_tf', Trigger, trigger_response         # type, and callback
)
rospy.loginfo("segmentation service available")
rospy.spin()   
