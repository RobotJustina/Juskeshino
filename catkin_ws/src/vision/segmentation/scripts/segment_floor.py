#! /usr/bin/env python3

                                                 
from utils import *



def trigger_response(request):
    ''' 
    Callback function used by the service server to process
    requests from clients. It returns a TriggerResponse
    '''
    print ('Segmenting')
    points_msg=rospy.wait_for_message("/hsrb/head_rgbd_sensor/depth_registered/rectified_points",PointCloud2,timeout=5)
    # <<<<<<<<<<<<<<<<<<<<<<ANTERIOR SEGMENTADOR>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    # points_data = ros_numpy.numpify(points_msg)    
    # image_data = points_data['rgb'].view((np.uint8, 4))[..., [2, 1, 0]]   
    # image=cv2.cvtColor(image_data, cv2.COLOR_BGR2RGB)
    # print (image.shape)
    # cents,xyz, images, img = plane_seg( points_msg,lower=10    , higher=4000,reg_hy=350)
    # <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    plot_im=False
    params=read_yaml('/segmentation_params.yaml')

    cents,xyz, images, img = plane_seg2(points_msg,hg=0.95,lg=0.001,lower=100, higher=500000,reg_ly= 100,reg_hy=450,plot=plot_im)
    


    print(len(cents))

    for i,cent in enumerate(cents):
        print (cent)
        x,y,z=cent
        if np.isnan(x) or np.isnan(y) or np.isnan(z):
            print('nan')
        else:
            print ('Estimated Height of the object ',max(xyz[i][:,2])-min(xyz[i][:,2]))
            
            print ('Estimated Width',max(xyz[i][:,1]) -min(xyz[i][:,1])               )
            
            print ('Estimated Depth',max(xyz[i][:,0])-min(xyz[i][:,0]))
            
            np.save( "/home/roboworks/Documents/points", xyz[i]   )
            
            t=write_tf(    (x,y,z),(0,0,0,1), 'Object'+str(i), "head_rgbd_sensor_rgb_frame"     )
            broadcaster.sendTransform(t)
            """#broadcaster.sendTransform((x,y,z),(0,0,0,1), rospy.Time.now(), 'Object'+str(i),"head_rgbd_sensor_rgb_frame")
                        
                #trans,rot=tf_listener.lookupTransform('map', 'Object'+str(i), rospy.Time(0))"""
    """for i,cent in enumerate(cents):
                    try:
                        trans = tfBuffer.lookup_transform('map', 'Object'+str(i), rospy.Time(), rospy.Duration(5.0))
                        print ("############tf2",trans,rot)
                        cents_map.append(trans)
                        print ("############_APP",trans,rot,cents_map)
                        trans,rot=read_tf(trans)
                    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                            print ( 'No  object TF FOUND')"""
                #print ('#############',cents_map)
                #ccs_map=np.asarray(cents_map)
    if plot_im:
        cv2.imshow("SEG",img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    pose, quat=Floats(),Floats()
    res= SegmentationResponse()
    #pose.data=cents_map
    img_msg=bridge.cv2_to_imgmsg(img)
    if len(res.im_out.image_msgs)==0:
        res.im_out.image_msgs.append(img_msg)
    pose.data=np.asarray(cents).ravel()
    #print ('##POSE',pose,trans,ccs_map,cents_map    )
    res.poses=pose
    # res.quats=pose
    return res
    

rospy.loginfo("segmentation service available")                    # initialize a ROS node
my_service = rospy.Service(                        # create a service, specifying its name,
    '/segment', Segmentation, trigger_response         # type, and callback
)
rospy.spin()   
