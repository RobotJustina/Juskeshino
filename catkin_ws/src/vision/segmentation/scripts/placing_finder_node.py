#! /usr/bin/env python3

                                                 
from utils_placing import *



def trigger_response(request):
    ''' 
    Trigger service ( a null request performs segmentation)
    '''
    print (f'Segmenting at {request.height.data: .2f} ')
    points_msg=rospy.wait_for_message("/hsrb/head_rgbd_sensor/depth_registered/rectified_points",PointCloud2,timeout=5)
    points_data = ros_numpy.numpify(points_msg)    
    image_data = points_data['rgb'].view((np.uint8, 4))[..., [2, 1, 0]]   
    image=cv2.cvtColor(image_data, cv2.COLOR_BGR2RGB)
    image = points_data['rgb'].view((np.uint8, 4))[..., [2, 1, 0]]
    rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)    
    #
    ################
    #CORRECT POINTS###################
    ################
    try:
            trans = tfBuffer.lookup_transform('map', 'head_rgbd_sensor_link', rospy.Time())
                        
            trans,rot=read_tf(trans)
            #print ("############head",trans,rot)
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print ( 'No head TF FOUND')
    t= write_tf(trans,rot)
    cloud_out = do_transform_cloud(points_msg, t)
    np_corrected=ros_numpy.numpify(cloud_out)
    corrected=np_corrected.reshape(points_data.shape)
    zs_no_nans=corrected['z'][~np.isnan(corrected['z'])]
    

###########################################
    high_x = (corrected['x'] > 7.8)
    low_x = (corrected['x'] < 8.25)###X COORDS
###############################################
    #AUTO DETECT PLANE?
    if request.height.data==-1:
        counts, bins =(np.histogram(zs_no_nans, bins=100))
        inds=np.where(counts>10000)
        planes_heights=bins[np.add(inds, 1)].flatten()
        print (f'Number of planes found {len(inds[0])} at z=[{bins[ np.add(inds, 1)]}]#############3')
        print (f'Plane heights detected {planes_heights} maximum  z=[{planes_heights.max()}]#############3')    
    else:planes_heights=[request.height.data]
    for plane_height in planes_heights:
        low_plane = (corrected['z'] > (plane_height-0.02)) #plane height
        high_plane = (corrected['z'] < (plane_height+0.02))#plane height + obj height
        orig_image= rgb_image.copy()
        mask= np.zeros(corrected['z'].shape)#mask
        z_lims=np.logical_and(low_plane, high_plane)
        x_lims=np.logical_and(low_x, high_x)
        result_indices = np.where(z_lims)#np.logical_and(z_lims, x_lims))
        mask[result_indices]=200
        _, binary_image = cv2.threshold(mask, 20, 255, cv2.THRESH_BINARY)
        eroded_image=cv2.erode(binary_image,kernel = np.ones((40, 40), np.uint8))
        deb_image=np.copy(orig_image)
        print ('quick estimation',np.nanmean(corrected['x'][np.where(eroded_image==255)]),np.nanmean(corrected['y'][np.where(eroded_image==255)]),np.nanmean(corrected['z'][np.where(eroded_image==255)]))
        contours, hierarchy = cv2.findContours(eroded_image.astype(np.uint8) ,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        cc=[]
        for contour in contours:
            M = cv2.moments(contour)
            area = cv2.contourArea(contour)
            if area > 200:          
                print (area)      
            # calculate x,y coordinate of center
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                cc.append((cX,cY))
                deb_image=cv2.circle(orig_image, (cX, cY), 5, (200, 200, 0), -1)
            else:print ('area too small') 
        poses=[]
        for (cX,cY) in cc:
            x,y = cX,cY
            print (x,y,'x,y')
            pose=np.asarray((np.mean(corrected['x'][y:y+45,x-23:x+22]),
                             np.mean(corrected['y'][y:y+45,x-23:x+22]),
                             np.mean(corrected['z'][y:y+45,x-23:x+22]) + 0.05))    
            deb_image[y:y+45,x-23:x+22,:]=255
            poses.append(pose)
            print('estimation', pose)
    ###################################3
    pose, quats=Floats(),Floats()
    heights_res, widths_res=Floats(),Floats()
    pose_c= Floats()
    heights, widths =Floats(),Floats()
    res= SegmentationResponse()
    #############################################
    img_msg=bridge.cv2_to_imgmsg(deb_image)
    #plt.imshow(img)
    #plt.imshow (image_with_contours)
    res.im_out.image_msgs.append(img_msg)
    pose.data=np.asarray(poses).ravel()
    #quats.data=np.asarray(quats_pca).ravel()
    res.poses=pose
    #res.quats=quats    
    #quats.data=np.asarray(quats_pca).ravel()
    #widths_res=np.asarray(widths).ravel()
    #res.widths=widths_res
    #heights_res=np.asarray(heights).ravel()
    #res.heights=heights_res
    return res        

#rospy.init_node('placing_finder') 
rospy.loginfo("Placing Finder service available")                    # initialize a ROS node
my_service = rospy.Service(                        # create a service, specifying its name,
    '/placing_finder', Segmentation, trigger_response         # type, and callback
)
rospy.spin()   
