#! /usr/bin/env python3

                                                 
from utils_segment import *



def trigger_response(request):
    ''' 
    Trigger service ( a null request performs segmentation)
    '''
    print (f'Segmenting at {request.height.data: .2f} ')
    #points_msg=rospy.wait_for_message("/hsrb/head_rgbd_sensor/depth_registered/rectified_points",PointCloud2,timeout=5)  # TAKESHI
    points_msg=rospy.wait_for_message("/camera/depth_registered/points",PointCloud2,timeout=5)
    df=read_yaml('/segmentation_params.yaml')
    higher_v=df['higher']  #Higher area limit 
    lower_v=df['lower']    #Lower Area Limit
    reg_hy_v=df['reg_hy']  #higher limit of pix y ( bottom part of img) 
    reg_ly_v=df['reg_ly']  #lower limit of pix y ( high part of img) 
    #print ( 'segmentation params ',df)
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
            #trans = tfBuffer.lookup_transform('map', 'head_rgbd_sensor_link', rospy.Time())
            trans = tfBuffer.lookup_transform('map', 'camera_rgb_optical_frame', rospy.Time())

                        
            trans,rot=read_tf(trans)
            #print ("############head",trans,rot)
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print ( 'No head TF FOUND')
    t= write_tf(trans,rot)
    cloud_out = do_transform_cloud(points_msg, t)
    np_corrected=ros_numpy.numpify(cloud_out)
    corrected=np_corrected.reshape(points_data.shape)
    orig_image= rgb_image.copy()
    mask= np.zeros(corrected['z'].shape)#mask    
    ###################################3
    pose, quats=Floats(),Floats()
    pose_c= Floats()
    heights, widths =Floats(),Floats()
    res= SegmentationResponse()
    #############################################
    if request.height.data==-1:
        zs_no_nans=corrected['z'][~np.isnan(corrected['z'])]
        counts, bins =(np.histogram(zs_no_nans, bins=100))
        inds=np.where(counts>5000)
        low_planes_height=bins[np.add(inds, 1)].flatten()
        print (f'Number of planes found {len(inds[0])} at z=[{bins[ np.add(inds, 1)]}]')
        if (low_planes_height[0] > -0.05) and (low_planes_height[0] < 0.05): low_planes_height=low_planes_height[1:]
    else:
        low_plane = (corrected['z'] > request.height.data)      # HEIGHT REQUESTED OR OBTAINED FROM HISTOGRAM
        low_planes_height=[]
        low_planes_height.append(request.height.data)
    cents=[]
    quats_pca=[]
    for low_planes_h in low_planes_height:
        print(f'segmenting at {low_planes_h} ')        
        low_plane = (corrected['z'] > low_planes_h)      # HEIGHT REQUESTED OR OBTAINED FROM HISTOGRAM
        high_plane = (corrected['z'] < low_planes_h+.4)        
        result_indices = np.where(np.logical_and(low_plane, high_plane))
        mask[result_indices]=200
        _, binary_image = cv2.threshold(mask, 20, 255, cv2.THRESH_BINARY)
        ###############FOR DEBUG IMAGE
        cv2_image = cv2.cvtColor(binary_image.astype(np.uint8), cv2.COLOR_GRAY2BGR) 
        img=cv2.bitwise_and(orig_image, cv2_image)
        image_with_contours = img.copy()
        #######################################
        contours, hierarchy = cv2.findContours(binary_image.astype(np.uint8) ,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)        
        if len (contours)==0:print(f'no objs at z={request.height.data} plane')
        for contour in contours:
            area = cv2.contourArea(contour)    
            if area > lower_v and area < higher_v :  #### AREA IN PIXELS ( USEFUL TO AVOID WALLS OR OTHER BIG CLUSTRS)  ( VALUES READ FROM YAML USE SEGMENTATOR TUNER)
            # Draw contours on the image
                M = cv2.moments(contour)                
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])                
                if (cY > reg_ly_v and cY < reg_hy_v  ):    ### REJECT CENTROID OUTSIDE OF THIS RANGE: READ FROM YAML                
                    boundRect = cv2.boundingRect(contour)
                    mask = np.zeros_like(binary_image) 
                    mask=cv2.rectangle(mask,(boundRect[0], boundRect[1]),(boundRect[0]+boundRect[2], boundRect[1]+boundRect[3]), (255,255,255), -1)
                    individual_mask=(mask*binary_image).astype(np.uint8)        
                    cent=np.asarray(   ((  np.nanmean(corrected['x'][np.where(individual_mask==1)]) ,np.nanmean(corrected['y'][np.where(individual_mask==1)]),np.nanmean(corrected['z'][np.where(individual_mask==1)])       ))      )
                    cents.append(cent)        
                    ################################PCA
                    points_c=np.asarray((corrected['x'][np.where(individual_mask==1)],corrected['y'][np.where(individual_mask==1)],corrected['z'][np.where(individual_mask==1)]))
                    E_R=points_to_PCA(points_c.transpose())
                    e_ER=tf.transformations.euler_from_matrix(E_R)
                    quat= tf. transformations.quaternion_from_euler(e_ER[0],e_ER[1],e_ER[2])
                    quats_pca.append(quat)
                    print(np.rad2deg(tf.transformations.euler_from_matrix(E_R)))
                    #######FOR DEBUG IMAGE
                    cv2.drawContours(image_with_contours, contour, -1, (0, 255, 0), 2)  # -1 draws all contours
                    rgb_image=cv2.rectangle(image_with_contours,(boundRect[0], boundRect[1]),(boundRect[0]+boundRect[2], boundRect[1]+boundRect[3]), (255,255,0), 2)
                else: print(f'Centroid_y out of range {cY} ,{reg_ly_v},{reg_hy_v}')
            else: print(f'Area of contour outside of range {area} ,{lower_v},{higher_v}')
        
    if len(cents)==0:
        img_msg=bridge.cv2_to_imgmsg(rgb_image)
        res.im_out.image_msgs.append(img_msg)
        return res
    img_msg=bridge.cv2_to_imgmsg(image_with_contours)
    #plt.imshow(img)
    #plt.imshow (image_with_contours)
    res.im_out.image_msgs.append(img_msg)
    pose.data=np.asarray(cents).ravel()
    quats.data=np.asarray(quats_pca).ravel()
    res.poses=pose
    res.quats=quats    
    return res        

rospy.loginfo("segmentation service available")                    # initialize a ROS node
my_service = rospy.Service(                        # create a service, specifying its name,
    '/segment', Segmentation, trigger_response         # type, and callback
)
rospy.spin()   
