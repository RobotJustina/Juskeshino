# -*- coding: utf-8 -*-


#!/usr/bin/env python
    
import numpy as np
import rospy
import ros_numpy
import tf2_ros
import tf
import os
import message_filters
from sensor_msgs.msg import Image , LaserScan , PointCloud2
from geometry_msgs.msg import TransformStamped
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud


from object_classification.srv import *
from utils_srv import RGBD


from std_msgs.msg import String


import cv2
from cv_bridge import CvBridge, CvBridgeError
bridge = CvBridge()
class_names=['002masterchefcan', '003crackerbox', '004sugarbox', '005tomatosoupcan', '006mustardbottle', '007tunafishcan', '008puddingbox', '009gelatinbox', '010pottedmeatcan', '011banana', '012strawberry', '013apple', '014lemon', '015peach', '016pear', '017orange', '018plum', '019pitcherbase', '021bleachcleanser', '022windexbottle', '024bowl', '025mug', '027skillet', '028skilletlid', '029plate', '030fork', '031spoon', '032knife', '033spatula', '035powerdrill', '036woodblock', '037scissors', '038padlock', '040largemarker', '042adjustablewrench', '043phillipsscrewdriver', '044flatscrewdriver', '048hammer', '050mediumclamp', '051largeclamp', '052extralargeclamp', '053minisoccerball', '054softball', '055baseball', '056tennisball', '057racquetball', '058golfball', '059chain', '061foambrick', '062dice', '063-amarbles', '063-bmarbles', '065-acups', '065-bcups', '065-ccups', '065-dcups', '065-ecups', '065-fcups', '065-gcups', '065-hcups', '065-icups', '065-jcups', '070-acoloredwoodblocks', '070-bcoloredwoodblocks', '071nineholepegtest', '072-atoyairplane', '073-alegoduplo', '073-blegoduplo', '073-clegoduplo', '073-dlegoduplo', '073-elegoduplo', '073-flegoduplo', '073-glegoduplo']

def correct_points(data, tans, rot, low=0.27,high=1000):

    # Function that transforms Point Cloud reference frame from  head, to map. (i.e. sensor coords to map coords )
    # low  high params Choose corrected plane to segment  w.r.t. head link 
    # img= correct_points() (Returns rgbd depth corrected image)

    #data = rospy.wait_for_message('/hsrb/head_rgbd_sensor/depth_registered/rectified_points', PointCloud2)
    np_data=ros_numpy.numpify(data)
    
    trans,rot=tf_listener.lookupTransform('/map', '/head_rgbd_sensor_gazebo_frame', rospy.Time(0)) 
    
    eu=np.asarray(tf.transformations.euler_from_quaternion(rot))
    t=TransformStamped()
    rot=tf.transformations.quaternion_from_euler(-eu[1],0,0)
    t.header.stamp = data.header.stamp
    
    t.transform.rotation.x = rot[0]
    t.transform.rotation.y = rot[1]
    t.transform.rotation.z = rot[2]
    t.transform.rotation.w = rot[3]

    cloud_out = do_transform_cloud(data, t)
    np_corrected=ros_numpy.numpify(cloud_out)
    corrected=np_corrected.reshape(np_data.shape)

    img= np.copy(corrected['y'])

    img[np.isnan(img)]=2
    img3 = np.where((img>low)&(img< 0.99*(trans[2])),img,255)
    return img3
def plane_seg_square_imgs(image,iimmg,points_data,data,trans,rot,lower=500 ,higher=50000,reg_ly= 30,reg_hy=600,plt_images=True):
    
    
    
    #image= rgbd.get_h_image()
    #iimmg= rgbd.get_image()
    #points_data= rgbd.get_points()
    img=np.copy(image)
    print('here')
    img3= correct_points(data,trans,rot)
    print(' no here')

    _,contours, hierarchy = cv2.findContours(img3.astype('uint8'),cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    i=0
    cents=[]
    points=[]
    images=[]
    for i, contour in enumerate(contours):

        area = cv2.contourArea(contour)

        if area > lower and area < higher :
            M = cv2.moments(contour)
            # calculate x,y coordinate of center
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])


            boundRect = cv2.boundingRect(contour)
            #just for drawing rect, dont waste too much time on this
            image_aux= iimmg[boundRect[1]:boundRect[1]+max(boundRect[2],boundRect[3]),boundRect[0]:boundRect[0]+max(boundRect[2],boundRect[3])]
            images.append(image_aux)
            img=cv2.rectangle(img,(boundRect[0], boundRect[1]),(boundRect[0]+boundRect[2], boundRect[1]+boundRect[3]), (0,0,0), 2)
            # calculate moments for each contour
            if (cY > reg_ly and cY < reg_hy  ):

                cv2.circle(img, (cX, cY), 5, (255, 255, 255), -1)
                cv2.putText(img, "centroid_"+str(i)+"_"+str(cX)+','+str(cY)    ,    (cX - 25, cY - 25)   ,cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 0), 2)
                print ('cX,cY',cX,cY)
                xyz=[]


                for jy in range (boundRect[0], boundRect[0]+boundRect[2]):
                    for ix in range(boundRect[1], boundRect[1]+boundRect[3]):
                        aux=(np.asarray((points_data['x'][ix,jy],points_data['y'][ix,jy],points_data['z'][ix,jy])))
                        if np.isnan(aux[0]) or np.isnan(aux[1]) or np.isnan(aux[2]):
                            'reject point'
                        else:
                            xyz.append(aux)

                xyz=np.asarray(xyz)
                cent=xyz.mean(axis=0)
                cents.append(cent)
                print (cent)
                points.append(xyz)
            else:
                print ('cent out of region... rejected')
    """sub_plt=0
                if plt_images:
                    for image in images:
            
                        sub_plt+=1
                        ax = plt.subplot(5, 5, sub_plt )
            
                        plt.imshow(image)
                        plt.axis("off")"""
    
    cents=np.asarray(cents)
    ### returns centroids found and a group of 3d coordinates that conform the centroid
    return(cents,np.asarray(points), images,img)
def seg_square_imgs(image, iimmg,points_data,lower=2000,higher=50000,reg_ly=0,reg_hy=1000,reg_lx=0,reg_hx=1000,plt_images=False): 

    # Segment image using color and K means  ()
   

    img= iimmg[:,:,0]

    values=image.reshape((-1,3))
    values= np.float32(values)
    criteria= (  cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER  ,1000,0.1)
    k=6
    _ , labels , cc =cv2.kmeans(values , k ,None,criteria,30,cv2.KMEANS_RANDOM_CENTERS)
    cc=np.uint8(cc)
    segmented_image= cc[labels.flatten()]
    segmented_image=segmented_image.reshape(image.shape)
    th3 = cv2.adaptiveThreshold(segmented_image,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY,11,2)
    kernel = np.ones((5,5),np.uint8)
    im4=cv2.erode(th3,kernel,iterations=4)
    plane_mask=points_data['z']
    cv2_img=plane_mask.astype('uint8')
    

    _,contours, hierarchy = cv2.findContours(im4.astype('uint8'),cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    i=0
    cents=[]
    points=[]
    images=[]
    for i, contour in enumerate(contours):

        area = cv2.contourArea(contour)

        if area > lower and area < higher :
            M = cv2.moments(contour)
            # calculate x,y coordinate of center
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])


            boundRect = cv2.boundingRect(contour)
            #just for drawing rect, dont waste too much time on this
            image_aux= iimmg[boundRect[1]:boundRect[1]+max(boundRect[3],boundRect[2]),boundRect[0]:boundRect[0]+max(boundRect[3],boundRect[2])]
            images.append(image_aux)
            img=cv2.rectangle(img,(boundRect[0], boundRect[1]),(boundRect[0]+boundRect[2], boundRect[1]+boundRect[3]), (0,0,0), 2)
            #img=cv2.rectangle(img,(boundRect[0], boundRect[1]),(boundRect[0]+max(boundRect[2],boundRect[3]), boundRect[1]+max(boundRect[2],boundRect[3])), (0,0,0), 2)
            # calculate moments for each contour
            if (cY > reg_ly and cY < reg_hy and  cX > reg_lx and cX < reg_hx   ):

                cv2.circle(img, (cX, cY), 5, (255, 255, 255), -1)
                cv2.putText(img, "centroid_"+str(i)+"_"+str(cX)+','+str(cY)    ,    (cX - 25, cY - 25)   ,cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 0), 2)
                print ('cX,cY',cX,cY)
                xyz=[]


                for jy in range (boundRect[0], boundRect[0]+boundRect[2]):
                    for ix in range(boundRect[1], boundRect[1]+boundRect[3]):
                        aux=(np.asarray((points_data['x'][ix,jy],points_data['y'][ix,jy],points_data['z'][ix,jy])))
                        if np.isnan(aux[0]) or np.isnan(aux[1]) or np.isnan(aux[2]):
                            'reject point'
                        else:
                            xyz.append(aux)

                xyz=np.asarray(xyz)
                cent=xyz.mean(axis=0)
                cents.append(cent)
                print (cent)
                points.append(xyz)
            else:
                print ('cent out of region... rejected')
                images.pop()
   
    cents=np.asarray(cents)
    
    return(cents,np.asarray(points), images,img)


##############################################################################################################################################################################################################################################################################################################################
def callback(img_msg,points_data):


    cv2_img = bridge.imgmsg_to_cv2(img_msg)#, "bgr8")
    #np_data = ros_numpy.numpify(points_data)
    
    
    cv2.imshow('class_server'	, cv2_img)
    # Process any keyboard commands
    #keystroke = cv2.waitkey(5)
  
     #print req

    keystroke = cv2.waitKey(0)
    if 32 <= keystroke and keystroke < 128:
        key = chr(keystroke).lower()
        print (key)
        if key =='p':
            print('Segment plane and classify')
            image= rgbd.get_h_image()
            iimmg= rgbd.get_image()
            points= rgbd.get_points()
           
            
            trans,rot= tf_listener.lookupTransform('/map', '/head_rgbd_sensor_gazebo_frame', rospy.Time(0)) 
            img3= correct_points(points_data,trans,rot ,.17,100 )
            cents,xyz, images, img= plane_seg_square_imgs(image,iimmg,points, points_data,trans,rot,lower=10, higher=5000)
            cv2.imshow('segmented image' ,img.astype('uint8'))
            req=classify.request_class()


            for image in images:
                img_msg=bridge.cv2_to_imgmsg(image)
                req.in_.image_msgs.append(img_msg)


            resp1 = classify(req)
            print (len(resp1.out.data))
            class_resp= np.asarray(resp1.out.data)
            cont3=0
            class_labels=[]
            for cla in class_resp:
                
                if cont3==3:
                    print ('-----------------')
                    cont3=0
                print (class_names [(int)(cla)])
                class_labels.append(class_names [(int)(cla)])
                cont3+=1
            #print (len(images))

            

            
            #print (class_labels)
            
          
            
            #cv2.imshow('segmented image' ,img3.astype('uint8'))
            
            #print(res)

        if key =='s':
            print('Segment color and classify')
            image= rgbd.get_h_image()
            iimmg= rgbd.get_image()
            points= rgbd.get_points()

            cents,xyz, images, img= seg_square_imgs(image,iimmg,points)
            

            print(len (images))
            req=classify.request_class()


            for image in images:
                img_msg=bridge.cv2_to_imgmsg(image)
                req.in_.image_msgs.append(img_msg)


            resp1 = classify(req)
            print (len(resp1.out.data))
            class_resp= np.asarray(resp1.out.data)
            cont3=0
            class_labels=[]
            for cla in class_resp:
                
                if cont3==3:
                    print ('-----------------')
                    cont3=0
                print (class_names [(int)(cla)])
                class_labels.append(class_names [(int)(cla)])
                cont3+=1


            
            #print (class_labels)
            
          
            
            cv2.imshow('classified_image' ,img.astype('uint8'))
        
            #print(res)
        
        
        if key=='q':
            rospy.signal_shutdown("User hit q key to quit.")



#cv2_img_depth = bridge.imgmsg_to_cv2(res.segmented_objects_array.table_objects_array[i].depth_image_array[j] )
#cv2_img = bridge.imgmsg_to_cv2(res.segmented_objects_array.table_objects_array[i].rgb_image_array[j] )


def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a uniques
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    global tf_listener,req,service_client , res , classify ,rgbd
    rospy.init_node('listener', anonymous=True)
    rgbd= RGBD()

    rospy.wait_for_service('classify')
    try:
        classify = rospy.ServiceProxy('/classify', Classify)    
    
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
    
    #rospy.Subscriber("/hsrb/head_rgbd_sensor/rgb/image_rect_color", Image, callback)
    #rospy.Subscriber("/hsrb/head_rgbd_sensor/depth_registered/image", Image, callback)
    tf_listener = tf.TransformListener()
    #images= message_filters.Subscriber("/hsrb/head_rgbd_sensor/rgb/image_rect_color",Image)
    images= message_filters.Subscriber("/usb_cam/image_raw",Image)
    points= message_filters.Subscriber("/hsrb/head_rgbd_sensor/depth_registered/rectified_points",PointCloud2)
    #message_filters.Subscriber("/hsrb/head_rgbd_sensor/depth_registered/image"     ,Image)
    #ats= message_filters.ApproximateTimeSynchronizer([symbol,odom,twist],queue_size=5,slop=.1,allow_headerless=True)
    ats= message_filters.ApproximateTimeSynchronizer([images,points],queue_size=5,slop=.1,allow_headerless=True)
    ats.registerCallback(callback)
        
    #rospy.Subscriber("/hsrb/base_scan", LaserScan, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()

