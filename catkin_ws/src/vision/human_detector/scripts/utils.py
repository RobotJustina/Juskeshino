#! /usr/bin/env python3
import tf
import cv2
import rospy  
import tf2_ros                                    
from human_detector.srv import Human_detector ,Human_detectorResponse 
from cv_bridge import CvBridge
from object_classification.srv import *
import tf2_ros    
from segmentation.msg import *

import numpy as np
import ros_numpy
import os
import matplotlib.pyplot as plt
import cv2 
from sensor_msgs.msg import Image , LaserScan , PointCloud2
from geometry_msgs.msg import TransformStamped, Pose
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from utils.misc_utils import *

#-----------------------------------------------------------------
global tf_listener, ptcld_lis, broadcaster , bridge , net


rospy.init_node('human_detector') 
tfBuffer = tf2_ros.Buffer()
tfBuffer = tf2_ros.Buffer()
listener2 = tf2_ros.TransformListener(tfBuffer)
listener = tf.TransformListener()

broadcaster = tf2_ros.TransformBroadcaster()
tf_static_broadcaster = tf2_ros.StaticTransformBroadcaster()
usr_url=os.path.expanduser( '~' )
protoFile = usr_url+"/openpose/models/pose/body_25/pose_deploy.prototxt"
weightsFile = usr_url+"/openpose/models/pose/body_25/pose_iter_584000.caffemodel"
net = cv2.dnn.readNetFromCaffe(protoFile, weightsFile)
#tf_listener = tf.TransformListener()
#broadcaster= tf.TransformBroadcaster()
#tf_static_broadcaster= tf2_ros.StaticTransformBroadcaster()
#pub = rospy.Publisher('/segmented_images', Image, queue_size=1)
bridge=CvBridge()

#-----------------------------------------------------------------
def write_tf(pose, q, child_frame , parent_frame='map'):
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


def probmap_to_3d_mean(points_data,probMap, thres_prob=0.3):
    ##Prob map to 3d point

    mask=np.where(probMap>thres_prob)
    npmask=np.asarray(mask).T

    npmask.shape
    xyz=[]
    if len (npmask)>1:
        for a in npmask:
            ix,iy=a[0],a[1]
            aux=(np.asarray((points_data['x'][ix,iy],points_data['y'][ix,iy],points_data['z'][ix,iy])))
            #print (aux)
            if np.isnan(aux[0]) or np.isnan(aux[1]) or np.isnan(aux[2]):
                    'reject point'
            else:
                xyz.append(aux)

    xyz=np.asarray(xyz)
    if (len(xyz)!=0):
        cent=xyz.mean(axis=0)
    else:
        cent=np.zeros(3)
    return cent


def detect_human(points_msg):
    points_data = ros_numpy.numpify(points_msg)    
    image_data = points_data['rgb'].view((np.uint8, 4))[..., [2, 1, 0]]   
    image=cv2.cvtColor(image_data, cv2.COLOR_BGR2RGB)
    #image = points_data['rgb'].view((np.uint8, 4))[..., [2, 1, 0]]
    #rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    print (image.shape)
    frame=image
    inHeight = frame.shape[0]
    inWidth = frame.shape[1]


    # Prepare the frame to be fed to the network
    inpBlob = cv2.dnn.blobFromImage(frame, 1.0 / 255, (inWidth, inHeight), (0, 0, 0), swapRB=False, crop=False)

    # Set the prepared object as the input blob of the network
    net.setInput(inpBlob)

    output = net.forward()
    i = 0 #Face
    #i = 1# Neck
    probMap = output[0, i, :, :]
    probMap = cv2.resize(probMap, (inWidth, inHeight))
    cent= probmap_to_3d_mean(points_data,probMap)
    print (cent)
    if np.isnan(cent.any()):return Human_detectorResponse()
    print (cent)
    if np.isnan(cent.any()):cent=np.zeros(3)
    res=Human_detectorResponse()
    res.x= cent[0]
    res.y= cent[1]
    res.z= cent[2]
    
    return res    


def detect_pointing(points_msg):
    points_data = ros_numpy.numpify(points_msg)    
    image_data = points_data['rgb'].view((np.uint8, 4))[..., [2, 1, 0]]   
    image=cv2.cvtColor(image_data, cv2.COLOR_BGR2RGB)
    
    print (image.shape)
    frame=image
    inHeight = frame.shape[0]
    inWidth = frame.shape[1]


    # Prepare the frame to be fed to the network
    inpBlob = cv2.dnn.blobFromImage(frame, 1.0 / 255, (inWidth, inHeight), (0, 0, 0), swapRB=False, crop=False)

    # Set the prepared object as the input blob of the network
    net.setInput(inpBlob)
    output = net.forward()

    poses=[]
    res=img[:,:,1]
    for i in np.asarray((3,4,6,7)):
        
        probMap = output[0, i, :, :]
        probMap = cv2.resize(probMap, (inWidth, inHeight))
        
        pose=[np.nanmean(pts['x'][np.where(probMap>=0.3)]),
                np.nanmean(pts['y'][np.where(probMap>=0.3)]),
                np.nanmean(pts['z'][np.where(probMap>=0.3)])]
        
        poses.append(pose)
        res= 500*probMap+ res  #DEBUG IMAGE
    #plt.imshow(res)
    #right elbow       ####
    # right wrist      ####
    # left elbow
    # left wrist
    res=Human_detectorResponse()
    ##############################
    tf_man.pub_static_tf(pos=poses[0],point_name='right_elbow', ref='head_rgbd_sensor_rgb_frame')
    tf_man.change_ref_frame_tf(point_name='right_elbow')
    tf_man.pub_static_tf(pos=poses[1],point_name='right_wrist', ref='head_rgbd_sensor_rgb_frame')
    tf_man.change_ref_frame_tf(point_name='right_wrist')
    tf_man.pub_static_tf(pos=poses[2],point_name='left_elbow', ref='head_rgbd_sensor_rgb_frame')
    tf_man.change_ref_frame_tf(point_name='left_elbow')
    tf_man.pub_static_tf(pos=poses[3],point_name='left_wrist', ref='head_rgbd_sensor_rgb_frame')
    tf_man.change_ref_frame_tf(point_name='left_wrist')
    wrist_xyz=tf_man.getTF(target_frame='right_wrist')
    elbow_xyz=tf_man.getTF(target_frame='right_elbow')
    v= np.asarray(wrist_xyz[0])-np.asarray(elbow_xyz[0])
    t=elbow_xyz[0][2]-v[2]
    x= elbow_xyz[0][0]+ t*v[0]
    y= elbow_xyz[0][1]+ t*v[1]
    tf_man.pub_static_tf(pos=[x,y,0],point_name='point_right')
    res.x_r= x
    res.y_r= y
    res.z_r= 0
    

    wrist_xyz=tf_man.getTF(target_frame='left_wrist')
    elbow_xyz=tf_man.getTF(target_frame='left_elbow')
    v= np.asarray(wrist_xyz[0])-np.asarray(elbow_xyz[0])
    t=elbow_xyz[0][2]-v[2]
    x= elbow_xyz[0][0]+ t*v[0]
    y= elbow_xyz[0][1]+ t*v[1]
    res.x_l= x
    res.y_l= y
    res.z_l= 0
    
    tf_man.pub_static_tf(pos=[x,y,0],point_name='point_left')

    return res    




#><>>>>>>>>>>>>>><<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<


    i = 0 #Face
    #i = 1# Neck
    probMap = output[0, i, :, :]
    probMap = cv2.resize(probMap, (inWidth, inHeight))
    cent= probmap_to_3d_mean(points_data,probMap)
    print (cent)
    if np.isnan(cent.any()):return Human_detectorResponse()
    print (cent)
    if np.isnan(cent.any()):cent=np.zeros(3)
    res=Human_detectorResponse()
    res.x= cent[0]
    res.y= cent[1]
    res.z= cent[2]
    
    return res    


def get_points(frame,inHeight,inWidth,output,threshold=0.1):
    
    h=output.shape[2]
    w=output.shape[3]
    points=[]

    for i in range(25):
        probMap = output[0,i,:,:]
        minVal,prob,minLoc,point = cv2.minMaxLoc(probMap)
        print("P: ",point)
        x = (inWidth * point[0]) / w
        y = (inHeight * point[1]) / h

        if prob > threshold : 
            cv2.circle(frame,(int(x),int(y)),5,(0,255,255),-1)
            cv2.putText(frame,str(i),(int(x),int(y)),
                        cv2.FONT_HERSHEY_SIMPLEX,0.6,(255,165,5),1,
                        lineType=cv2.LINE_AA)

            points.append((x,y))
        else:
            points.append(None)

    return points


# IN PROGRESS
def detect_all(points_msg):
    direct=os.getcwd()
    im=cv2.imread(direct+"/Documents/Tests/persons2.jpg")
    print(im.shape)
    points_data = ros_numpy.numpify(points_msg)    
    image_data = points_data['rgb'].view((np.uint8, 4))[..., [2, 1, 0]]   
    image=cv2.cvtColor(image_data, cv2.COLOR_BGR2RGB)
    #image = points_data['rgb'].view((np.uint8, 4))[..., [2, 1, 0]]
    #rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    print (image.shape)
    frame=im
    inHeight = frame.shape[0]
    inWidth = frame.shape[1]

    keypoints=[]
    # Prepare the frame to be fed to the network
    inpBlob = cv2.dnn.blobFromImage(frame, 1.0 / 255, (inWidth, inHeight), (0, 0, 0), swapRB=False, crop=False)

    # Set the prepared object as the input blob of the network
    net.setInput(inpBlob)

    output = net.forward()

    points = get_points(frame,inHeight,inWidth,output)
    print("POINTSSSS",len(points),points)
    cv2.imshow("DRAW",frame)
    cv2.waitKey(0)

    """
    i = 0 #Face
    #i = 1# Neck
    print("SHAPEs",output.shape)
    probMap = output[0, i, :, :]
    print(probMap.shape)
    probMap = cv2.resize(probMap, (inWidth, inHeight))
    minVal, prob, minLoc, point = cv2.minMaxLoc(probMap)
 
        #imr=cv2.bitwise_or(im, probMap)
    cv2.imshow("A",cv2.cvtColor(im, cv2.COLOR_BGR2RGB))
    for i in range(output.shape[1]):
        probM=output[0,i,:,:]
        probM=cv2.resize(probM, (inWidth, inHeight))

        cv2.imshow("B",probM)
        cv2.waitKey(0)

    """
    cv2.destroyAllWindows()

    return Human_detectorResponse() 