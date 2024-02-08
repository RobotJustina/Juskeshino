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
import rospkg
from smach_utils_justina import *
#from tuner_justina_utils import *
from object_classification.srv import *
#from utils_srv import RGBD


from std_msgs.msg import String
first= True
rospack = rospkg.RosPack()
yaml_file = rospy.get_param("segmentation_params", "/segmentation_params.yaml")
file_path = rospack.get_path('segmentation')+'/config_files'  + yaml_file





def read_segmentation_yaml(yaml_file = "/segmentation_params.yaml"):
    
    file_path = rospack.get_path('segmentation')+'/config_files'  + yaml_file

    with open(file_path, 'r') as file:
        content = yaml.safe_load(file)
    return content

#------------------------------------------------------
def nothing(x):
    pass

##############################################################################################################################################################################################################################################################################################################################
def callback(points_msg):

    global first , rospack , file_path
 


    #print('got imgs msgs')
    points_data = ros_numpy.numpify(points_msg)    
    image_data = points_data['rgb'].view((np.uint8, 4))[..., [2, 1, 0]]   #JUST TO MANTAIN DISPLAY
    image=cv2.cvtColor(image_data, cv2.COLOR_BGR2RGB)




    img=rgbd.get_image()
    img=cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
   

    #cv2.imshow('xtion rgb'	, image)

     
    if first:
        print (first)
        cv2.imshow('class rgbd'  , img)
        df = read_segmentation_yaml()

        cv2.createTrackbar('Max Area', 'class rgbd', 0, 240*320, nothing)   ### AREA MAX IS half THE WHOLE IMAGE 
        cv2.createTrackbar('Min Area', 'class rgbd', 0, 2000, nothing)  
        cv2.createTrackbar('Hi limit pix y', 'class rgbd',240,480,nothing)
        cv2.createTrackbar('Lo limit pix y', 'class rgbd',0,240,nothing)
        cv2.createTrackbar('Plane height (cms)', 'class rgbd', -10, 100, nothing)   ### AREA MAX IS half THE WHOLE IMAGE 
        cv2.setTrackbarPos('Max Area', 'class rgbd',df['higher']) 
        cv2.setTrackbarPos('Min Area', 'class rgbd',df['lower']) 
        cv2.setTrackbarPos('Hi limit pix y','class rgbd',df['reg_hy']) 
        cv2.setTrackbarPos('Lo limit pix y','class rgbd',df['reg_ly']) 
        first=False
    cv2.imshow('class rgbd'  , img)
    #print (r)

    # Process any keyboard commands
    keystroke = cv2.waitKey(1)
    
   
    
    

    
    if 32 <= keystroke and keystroke < 128:
        key = chr(keystroke).lower()
        print (key)
    #    
    #    
    #   
        
        if key=='u': 

            print('cha')
            ptcld_lis.unregister()
            
            
        if key=='f': 
            
            
            print('file_path',file_path)
            df = read_segmentation_yaml()
            print('df',df)
            r = cv2.getTrackbarPos('Max Area', 'class rgbd')
            df['higher']=r
            r = cv2.getTrackbarPos('Min Area', 'class rgbd')
            df['lower']=r
            r = cv2.getTrackbarPos('Hi limit pix y', 'class rgbd')
            df['reg_hy']=r
            r = cv2.getTrackbarPos('Lo limit pix y', 'class rgbd')
            df['reg_ly']=r


            with open(file_path, 'w') as file:
                documents = yaml.dump(df, file, default_flow_style=False)
            return True

        if key=='y':
            print ('#############YOLO SERVICE YCB REQUESTED')
            img_msg  = bridge.cv2_to_imgmsg(image)
            req      = classify_client.request_class()
            req.in_.image_msgs.append(img_msg)
            res      = classify_client(req)
            debug_image=bridge.imgmsg_to_cv2(res.debug_image.image_msgs[0])
            cv2.imshow('our of res'  , debug_image)

        if key=='p':
            print ('#############Pointing  SERVICE openPose REQUESTED')
            
            res=pointing_detect_server.call()
            print (f'xr{res.x_r} yr{res.y_r} xl{res.x_l} yl{res.y_l} ')

           
            if (res.x_r+res.y_r)!=0:
                print('right')
                tf_man.pub_static_tf(pos=[res.x_r, res.y_r,0], rot =[0,0,0,1], point_name='pointing_right')
            if (res.x_l+res.y_l)!=0:
                print('left')
                tf_man.pub_static_tf(pos=[res.x_l, res.y_l,0], rot =[0,0,0,1], point_name='pointing_left')
            debug_image=bridge.imgmsg_to_cv2(res.debug_image[0])
            cv2.imshow('our of res'  , debug_image)

        if key=='s': 

            request= segmentation_server.request_class() 
            r = cv2.getTrackbarPos('Plane height (cms)', 'class rgbd')
            request.height.data=r * 0.01
            if r == 100:request.height.data=-1.0
            print ('#############Segmenting at plane####################',request.height.data)
            
            #head.set_joint_values([ 0.1, -0.5])
            res=segmentation_server.call(request)
            succ=seg_res_tf(res)
           
            img=bridge.imgmsg_to_cv2(res.im_out.image_msgs[0])
            cv2.imshow('our of res'  , img)
      
        if key=='q':
            rospy.signal_shutdown("User hit q key to quit.")



def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a uniques
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    global tf_listener, ptcld_lis 
    
    #rospy.init_node('image_tag_rgbd', anonymous=True)
    
    tf_listener = tf.TransformListener() 
    ptcld_lis=rospy.Subscriber("/camera/depth_registered/points",PointCloud2, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()



    


if __name__ == '__main__':
    
    
    
    listener()

