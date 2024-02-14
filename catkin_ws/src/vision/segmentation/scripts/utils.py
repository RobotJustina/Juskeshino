#! /usr/bin/env python3
import tf
import cv2
import rospy  
import tf2_ros                                    
from segmentation.srv import Segmentation, SegmentationResponse 
from cv_bridge import CvBridge
from object_classification.srv import *
import tf2_ros    
from segmentation.msg import *
import numpy as np
import ros_numpy
import os
import matplotlib.pyplot as plt
import rospkg
import yaml
from sensor_msgs.msg import Image , LaserScan , PointCloud2
from geometry_msgs.msg import TransformStamped, Pose
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from mpl_toolkits.axes_grid1 import make_axes_locatable
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import seaborn as sns
import pandas as pd
from sklearn.decomposition import PCA
#-----------------------------------------------------------------
global tf_listener, ptcld_lis, broadcaster , bridge , rospack , Pca

rospack = rospkg.RosPack()
rospy.init_node('plane_segmentation') 
tfBuffer = tf2_ros.Buffer()
tfBuffer = tf2_ros.Buffer()
listener2 = tf2_ros.TransformListener(tfBuffer)
listener = tf.TransformListener()

broadcaster = tf2_ros.TransformBroadcaster()
tf_static_broadcaster = tf2_ros.StaticTransformBroadcaster()

#tf_listener = tf.TransformListener()
#broadcaster= tf.TransformBroadcaster()
#tf_static_broadcaster= tf2_ros.StaticTransformBroadcaster()
#pub = rospy.Publisher('/segmented_images', Image, queue_size=1)
bridge=CvBridge()

Pca=PCA()

#-----------------------------------------------------------------
def points_to_PCA(points):
    df=pd.DataFrame(points)
    df.columns=[['x','y','z']]
    threshold= df['z'].min().values[0]*0.998
    print (threshold)
    rslt_df = df.loc[df[df['z'] > threshold].index]
    points=rslt_df[['x','y','z']].dropna().values
    Pca=PCA(n_components=3)
    Pca.fit(points)
    print('Pca.explained_variance_',Pca.explained_variance_)
    ref=np.eye(3)
    pcas=Pca.components_
    R=[]
    R.append(np.dot(pcas[0],ref))
    R.append(np.dot(pcas[1],ref))
    R.append(np.dot(pcas[2],ref))
    R=np.asarray(R)
    ## HOMOGENEUS
    E_R= np.zeros((4,4))
    E_R[:3,:3]+=R
    E_R[-1,-1]=1
    return     E_R


#-----------------------------------------------------------------
def write_tf(pose, q, child_frame="" , parent_frame='map'):
    #  pose = trans  q = quaternion  , childframe =""
    # format  write the transformstampled message
    t= TransformStamped()
    t.header.stamp = rospy.Time.now()
    #t.header.stamp = rospy.Time(0)
    t.header.frame_id =parent_frame
    t.child_frame_id =  child_frame
    t.transform.translation.x = pose[0]
    t.transform.translation.y = pose[1]
    t.transform.translation.z = pose[2]
    #q = tf.transformations.quaternion_from_euler(eu[0], eu[1], eu[2])
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]
    return t
    
#-----------------------------------------------------------------
def read_tf(t):
    # trasnform message to np arrays
    pose=np.asarray((
        t.transform.translation.x,
        t.transform.translation.y,
        t.transform.translation.z
        ))
    quat=np.asarray((
        t.transform.rotation.x,
        t.transform.rotation.y,
        t.transform.rotation.z,
        t.transform.rotation.w
        ))
    
    return pose, quat

#-----------------------------------------------------------------
def correct_points(points_msg):

    # Function that transforms Point Cloud reference frame from  head, to map. (i.e. sensor coords to map coords )
    # img= correct_points() (Returns rgbd depth corrected image)    
    #points msg in 
    #data = rospy.wait_for_message('/hsrb/head_rgbd_sensor/depth_registered/rectified_points', PointCloud2)
    

    np_data=ros_numpy.numpify(points_msg)

    try:
        trans = tfBuffer.lookup_transform('map', 'head_rgbd_sensor_link', rospy.Time())
                    
        trans,rot=read_tf(trans)
        #print ("############head",trans,rot)
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        print ( 'No head TF FOUND')

    #trans,rot=tf_listener.lookupTransform('/map', '/head_rgbd_sensor_rgb_frame', rospy.Time(0))
    #print ("############TF1",trans,rot)
    eu=np.asarray(tf.transformations.euler_from_quaternion(rot))
    t=TransformStamped()
    rot=tf.transformations.quaternion_from_euler(eu[0],eu[1],eu[2])
    #rot=tf.transformations.quaternion_from_euler(-eu[1],0,0)
    t.header.stamp = points_msg.header.stamp
    t.transform.rotation.x = rot[0]
    t.transform.rotation.y = rot[1]
    t.transform.rotation.z = rot[2]
    t.transform.rotation.w = rot[3]
    cloud_out = do_transform_cloud(points_msg, t)
    np_corrected=ros_numpy.numpify(cloud_out)
    corrected=np_corrected.reshape(np_data.shape)
    img= np.copy(-corrected['z'])
    img[np.isnan(img)]=2
    img_corrected = np.where((img<trans[2]*0.96) ,img,5)
    #Returns img with xyz info  and points datastructure ( see numpify a ptcloud2)
    return img_corrected , corrected


#-----------------------------------------------------------------    
def read_yaml(yaml_file = '/known_locations.yaml'):
    
    file_path = rospack.get_path('segmentation')+'/config_files'  + yaml_file

    with open(file_path, 'r') as file:
        content = yaml.safe_load(file)
    return content

#-----------------------------------------------------------------    

def plane_seg(points_msg,hg=0.85,lg=1.5,th_v=0.03,lower=1000 ,higher=50000,reg_ly= 30,reg_hy=600 ,plane_height=-1 ):

    ###
    points_data = ros_numpy.numpify(points_msg)    
    image_data = points_data['rgb'].view((np.uint8, 4))[..., [2, 1, 0]]   
    image=cv2.cvtColor(image_data, cv2.COLOR_BGR2RGB)
    image = points_data['rgb'].view((np.uint8, 4))[..., [2, 1, 0]]
    rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)    
    im_corrected,corrected=correct_points(points_msg)  ### aplica la transformada de la camara para corregir la perspectiva
    zs_no_nans=np.where(~np.isnan(corrected['z']),corrected['z'],1)   
    t = tfBuffer.lookup_transform('map', 'head_rgbd_sensor_link', rospy.Time())
    trans=t.transform.translation.z
    if plane_height == -1:
        histogram, bin_edges =(np.histogram(zs_no_nans, bins=100))
        plane_height= (trans)+bin_edges[histogram[:-1].argmax()]
    print(plane_height, 'plane_height')
  
    #thres_floor=0.13
    thres_floor = 0.01
  
    im_corrected = np.where((zs_no_nans >-trans+thres_floor+plane_height),zs_no_nans,1)
    contours, hierarchy = cv2.findContours(im_corrected.astype('uint8'),cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    i=0
    cents=[]
    cents_corr=[]
    points=[]
    points_c=[]
    images=[]
    for i, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        if area > lower and area < higher :  #### AREA IN PIXELS ( USEFUL TO AVOID WALLS OR OTHER BIG CLUSTRS)
            M = cv2.moments(contour)
            # calculate x,y coordinate of center
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            if (cY > reg_ly and cY < reg_hy  ):         #between pxy  region low y region high y
                boundRect = cv2.boundingRect(contour)
                image_aux= image[boundRect[1]:boundRect[1]+boundRect[3],boundRect[0]:boundRect[1]+boundRect[2]]
                images.append(image_aux)   ### image of every bbox between pxy  region low y region high y
                image_aux= im_corrected[boundRect[1]:boundRect[1]+boundRect[3],boundRect[0]:boundRect[0]+boundRect[2]]
                mask=np.where(image_aux!=5)
                npmask=np.asarray(mask).T
                rgb_image=cv2.rectangle(rgb_image,(boundRect[0], boundRect[1]),(boundRect[0]+boundRect[2], boundRect[1]+boundRect[3]), (255,255,0), 2)
                cv2.circle(rgb_image, (cX, cY), 5, (255, 255, 255), -1)
                cv2.putText(rgb_image, "centroid_"+str(i+1)+"_"+str(cX)+','+str(cY)    ,    (cX - 25, cY - 25)   ,cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 0), 2)
                xyz=[]
                xyz_c=[]
                if len (npmask)>0:    ## MASK containing the segmented object, read every xyz world coordinate related to pixels in the mask.
                    for a in npmask:
                        ix,iy=a[0],a[1]
                        aux2=(np.asarray((corrected['x'][boundRect[1]+ix,boundRect[0]+iy],corrected['y'][boundRect[1]+ix,boundRect[0]+iy],corrected['z'][boundRect[1]+ix,boundRect[0]+iy])))
                        aux=(np.asarray((points_data['x'][boundRect[1]+ix,boundRect[0]+iy],points_data['y'][boundRect[1]+ix,boundRect[0]+iy],points_data['z'][boundRect[1]+ix,boundRect[0]+iy])))
                        
                        if np.isnan(aux[0]) or np.isnan(aux[1]) or np.isnan(aux[2]):
                                'reject point'
                        else:
                            xyz.append(aux)
                        if np.isnan(aux2[0]) or np.isnan(aux2[1]) or np.isnan(aux2[2]):
                                'reject point'
                        else:                            
                            xyz_c.append(aux2)
                xyz=np.asarray(xyz)
                xyz_c=np.asarray(xyz_c)
                
                cent=xyz.mean(axis=0)
                cent_corr=xyz_c.mean(axis=0)  #centroids of each object ( mean of xyz pts.)
                cents.append(cent)
                cents_corr.append(cent_corr)
                points.append(xyz)
                points_c.append(xyz_c)
            else:   
                print ('cent out of region... rejected')
    # RETURNS  CENTS= centroids of each segmented object
    #          points xyz of each object (for pca)
    #          images array of each image segmented (for image training)
    #          rgb_image ( labeled bboxed image , useful for display and debug (beta))
    #          points_c points of each segmented object , but now from the corrected cloud-(beta) usefull for estimating objs dimensions 
    return cents,np.asarray(points), images,rgb_image,np.asarray(points_c) ,cents_corr
    #return cents,np.asarray(points), images,im_corrected,np.asarray(points_c) 

