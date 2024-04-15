#!/usr/bin/env python3
from cv_bridge import CvBridge
from object_classification.srv import Classify,ClassifyResponse, ClassifyRequest
from object_classification.msg import Floats  , Ints
import cv2
import numpy as np
####################
import torch
import torch.backends.cudnn as cudnn
from yolov5.models.experimental import attempt_load
from yolov5.utils.general import *
from yolov5.utils.torch_utils import select_device #select_device, load_classifier, time_sync
######################################################################################
from utils_srv import *
#############################################################################




################################################################################
def callback(req):
    print ('got ',len(req.in_.image_msgs),'images')    
    res=ClassifyResponse()
    images=[]
    for i in range(len(req.in_.image_msgs)):
        images.append(cv2.cvtColor(bridge.imgmsg_to_cv2(req.in_.image_msgs[i]),cv2.COLOR_BGR2RGB))

    for test_img in images:
        img = torch.from_numpy(test_img).to(device) # RGB IMAGE TENSOR (TORCH)
        img = img / 255.0                              #NORMALIZE
        img=img.unsqueeze(0)                        # ADD DIMENSION FOR TENSOR ( BATCH)
        img=torch.moveaxis(img,3,1)                  #Channel order for YOLO
        pred = model(img, augment=False)[0]
        pred = non_max_suppression(pred)  # IOU 
        debug_img=np.copy(test_img)
        num_preds=0

        #points_msg = rospy.wait_for_message('/hsrb/head_rgbd_sensor/depth_registered/rectified_points', PointCloud2) #Takeshi
        points_msg = rospy.wait_for_message('/camera/depth_registered/points', PointCloud2)
        points= ros_numpy.numpify(points_msg)
        for  det in pred:
            for *xyxy, conf, cls in (det):# Model Result is bounding box  confidence  and class
                if conf.cpu().tolist() > 0.5:
                    num_preds+=1
                    pt_min=[int(xyxy[0].cpu().tolist()),int(xyxy[1].cpu().tolist())]
                    pt_max=[int(xyxy[2].cpu().tolist()),int(xyxy[3].cpu().tolist())]
                    pose=Pose()
                    cc=[np.nanmean(  points['x'][pt_min[1]:pt_max[1],pt_min[0]:pt_max[0]]),
                        np.nanmean(  points['y'][pt_min[1]:pt_max[1],pt_min[0]:pt_max[0]]),
                        np.nanmean(  points['z'][pt_min[1]:pt_max[1],pt_min[0]:pt_max[0]]) ]
                    
                    if np.isnan(cc[0]) or np.isnan(cc[1]) or  np.isnan(cc[2]):
                        print ('no points')
                        pose.position.x=cc[0]
                        pose.position.y=cc[1]
                        pose.position.z=cc[2]
                        pose.orientation.w=1
                        res.poses.append(pose)
                    else:

                        pose.position.x=cc[0]
                        pose.position.y=cc[1]
                        pose.position.z=cc[2]
                        pose.orientation.w=1
                        res.poses.append(pose)
                        
                        #t=write_tf(    cc , (0,0,0,1), model.names[int(cls.cpu().tolist())], "head_rgbd_sensor_rgb_frame"   ) #Takeshi
                        t=write_tf(    cc , (0,0,0,1), model.names[int(cls.cpu().tolist())], "camera_rgb_optical_frame"   ) #Justina realsense_link
                        
                        broadcaster.sendTransform(t)
                    debug_img=cv2.rectangle(debug_img ,pt_min,pt_max,  (0, 255, 0), 2   )
                    debug_img= cv2.putText(debug_img ,model.names[int(cls.cpu().tolist())],
                                pt_min, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)                
                    print (num_preds,pt_min, pt_max,conf.cpu().tolist(),model.names[int(cls.cpu().tolist())], cc )
                    for coord in pt_min:    res.pt_max.data.append(coord)
                    for coord in pt_max:    res.pt_min.data.append(coord)
                    res.confidence.data.append(conf)     
                    string_msg= String()
                    string_msg.data=model.names[int(cls.cpu().tolist())]
                    res.names.append(string_msg)               
        
        print(f'### number of detections -> {num_preds}')

    rgb_debug_img = cv2.cvtColor(debug_img, cv2.COLOR_BGR2RGB)    
    res.debug_image.image_msgs.append(bridge.cv2_to_imgmsg(rgb_debug_img))
    ##### TFS
    return res 


def classify_server(model_name='ycb.pt'):
    global listener,rgbd, bridge , model , device , tfBuffer, broadcaster
    rospy.init_node('classification_server')
    rgbd= RGBD()
    bridge = CvBridge()

    tfBuffer = tf2_ros.Buffer()
    tfBuffer = tf2_ros.Buffer()
    listener2 = tf2_ros.TransformListener(tfBuffer)
    listener = tf.TransformListener()

    broadcaster = tf2_ros.TransformBroadcaster()
    tf_static_broadcaster = tf2_ros.StaticTransformBroadcaster()

    device = select_device('')
    rospack = rospkg.RosPack()
    file_path = rospack.get_path('object_classification')
    ycb_yolo_path = file_path + '/src/weights/' + model_name
    rospy.logwarn("model path: " + ycb_yolo_path)
    rospy.logwarn("Loaded model: " + model_name)

    model = attempt_load(ycb_yolo_path, device)
    #rospy.loginfo("calssification_ YOLOV5 service available")                    # initialize a ROS node
    s = rospy.Service('classify', Classify, callback) 
    # print("Classification service available")
    rospy.spin()


if __name__ == "__main__":
    model_name = "TaRJust_ycb.pt"
    classify_server()
