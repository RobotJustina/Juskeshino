#!/usr/bin/env python
import rospy
import cv2
import numpy
import ros_numpy
from vision_msgs.srv import *
from vision_msgs.msg import VisionObject
from sensor_msgs.msg import PointCloud2
from cv_bridge import CvBridge
from yolov5.utils.torch_utils import select_device
from yolov5.models.experimental import attempt_load
from yolov5.utils.general import *
import torch

remap_obj_names = {'potted_meat_can':'pringles',
                   'tomato_soup_can':'tuna'}

def get_vision_object(img, label, confidence, frame_id, x0, y0, x1, y1, cloud):
    global remove_background, threshold1, threshold2
    mask = numpy.zeros((img.shape[0], img.shape[1]), dtype=numpy.uint8)
    mask[y0:y1, x0:x1] = 255

    if remove_background:
        print("Not removing bg")
        gray  = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        canny = cv2.Canny(gray, threshold1, threshold2)
        canny = cv2.bitwise_and(canny,mask)
        kernel = np.ones((3, 3), np.uint8)
        dilated = cv2.dilate(canny,kernel, iterations=7)
        eroded = cv2.erode(dilated,kernel, iterations=5)
        contours, hierarchy = cv2.findContours(eroded, cv2.RETR_EXTERNAL , cv2.CHAIN_APPROX_SIMPLE)
        mask = numpy.zeros(eroded.shape, dtype=numpy.uint8)
        mask = cv2.bitwise_or(eroded, mask)
        for i in range(len(contours)):
            cv2.drawContours(mask, contours, i, 255, -1)

        mask = cv2.erode(mask, kernel )

    display_img = cv2.bitwise_and(img, cv2.cvtColor(mask,cv2.COLOR_GRAY2BGR))

    bridge = CvBridge()
    msg = VisionObject()
    msg.header.frame_id = frame_id
    msg.header.stamp = rospy.Time.now()
    msg.id = label
    msg.confidence = confidence
    msg.image = bridge.cv2_to_imgmsg(cv2.bitwise_and(img,cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)), encoding="passthrough")
    msg.obj_mask = bridge.cv2_to_imgmsg(mask, encoding="passthrough")
    obj_cloud = numpy.copy(cloud)
    obj_cloud[mask == 0] = 0
    centroid_x = numpy.nanmean(obj_cloud['x'][mask != 0])
    centroid_y = numpy.nanmean(obj_cloud['y'][mask != 0])
    centroid_z = numpy.nanmean(obj_cloud['z'][mask != 0])
    min_x = numpy.nanmin(obj_cloud['x'][mask != 0])
    min_y = numpy.nanmin(obj_cloud['y'][mask != 0])
    min_z = numpy.nanmin(obj_cloud['z'][mask != 0])
    max_x = numpy.nanmax(obj_cloud['x'][mask != 0])
    max_y = numpy.nanmax(obj_cloud['y'][mask != 0])
    max_z = numpy.nanmax(obj_cloud['z'][mask != 0])
    size_x = max_x - min_x
    size_y = max_y - min_y
    size_z = max_z - min_z
    obj_cloud = ros_numpy.point_cloud2.merge_rgb_fields(obj_cloud)
    msg.point_cloud = ros_numpy.point_cloud2.array_to_pointcloud2(obj_cloud, stamp=rospy.Time.now(), frame_id=frame_id)
    msg.size.x = size_x
    msg.size.y = size_y
    msg.size.z = size_z
    msg.pose.position.x = centroid_x
    msg.pose.position.y = centroid_y
    msg.pose.position.z = centroid_z
    msg.pose.orientation.w = 1.0
    msg.x = min(x0, x1)
    msg.y = min(y0, y1)
    msg.width = abs(x1 - x0)
    msg.height = abs(y1 - x1)
    return msg, display_img

def callback_recognize_object(req):
    global device, model, min_confidence, result_img, pub_obj
    print("ObjRecoYolo.->Requested recognize object: " + req.id)
    clt_transform = rospy.ServiceProxy("/vision/point_cloud_to_base_link", PreprocessPointCloud)
    req_trans = PreprocessPointCloudRequest()
    req_trans.input_cloud = req.point_cloud
    resp_trans = clt_transform(req_trans)
    cloud = resp_trans.output_cloud
    cloud = ros_numpy.point_cloud2.pointcloud2_to_array(cloud)
    cloud = ros_numpy.point_cloud2.split_rgb_field(cloud)
    yolo_img = cv2.merge((cloud['r'], cloud['g'], cloud['b']))
    cv_img = cv2.merge((cloud['b'], cloud['g'], cloud['r']))
    
    yolo_img = torch.from_numpy(yolo_img).to(device) # RGB IMAGE TENSOR (TORCH)
    yolo_img = yolo_img / 255.0                      # NORMALIZE
    yolo_img = yolo_img.unsqueeze(0)                 # ADD DIMENSION FOR TENSOR ( BATCH)
    yolo_img = torch.moveaxis(yolo_img,3,1)          # Channel order for YOLO
    pred = model(yolo_img, augment=False)[0]
    pred = non_max_suppression(pred)                 # IOU

    bridge = CvBridge()
    resp = RecognizeObjectResponse()
    for det in pred:
        for x0,y0,x1,y1, conf, cls in (det):         # Model Result is bounding box  confidence  and class
            confidence = conf.cpu().tolist()
            if confidence > min_confidence:
                x0 = int(x0.cpu().tolist())
                y0 = int(y0.cpu().tolist())
                x1 = int(x1.cpu().tolist())
                y1 = int(y1.cpu().tolist())
                name = model.names[int(cls.cpu().tolist())]
                if name in remap_obj_names.keys():
                    name = remap_obj_names[name]
                if req.id == name:
                    msg, obj_img = get_vision_object(cv_img, name, confidence, "base_link", x0, y0, x1, y1, cloud)
                    result_img = obj_img
                    result_img = cv2.putText(result_img, name+" "+str(confidence)[:4],(x0,y0-2),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,255,0),1)
                    resp.recog_object = msg
                    resp.image = bridge.cv2_to_imgmsg(obj_img, encoding="passthrough")
                    pub_obj.publish(msg.point_cloud)
                    return resp
    return False

def callback_recognize_objects(req):
    global device, model, min_confidence, result_img
    print("ObjRecoYolo.->Requested recognize objects...")
    clt_transform = rospy.ServiceProxy("/vision/point_cloud_to_base_link", PreprocessPointCloud)
    req_trans = PreprocessPointCloudRequest()
    req_trans.input_cloud = req.point_cloud
    resp_trans = clt_transform(req_trans)
    cloud = resp_trans.output_cloud
    cloud = ros_numpy.point_cloud2.pointcloud2_to_array(cloud)
    cloud = ros_numpy.point_cloud2.split_rgb_field(cloud)
    yolo_img = cv2.merge((cloud['r'], cloud['g'], cloud['b']))
    cv_img = cv2.merge((cloud['b'], cloud['g'], cloud['r']))
    result_img   = numpy.zeros(cv_img.shape, dtype=numpy.uint8)
    
    yolo_img = torch.from_numpy(yolo_img).to(device) # RGB IMAGE TENSOR (TORCH)
    yolo_img = yolo_img / 255.0                      # NORMALIZE
    yolo_img = yolo_img.unsqueeze(0)                 # ADD DIMENSION FOR TENSOR ( BATCH)
    yolo_img = torch.moveaxis(yolo_img,3,1)          # Channel order for YOLO
    pred = model(yolo_img, augment=False)[0]
    pred = non_max_suppression(pred)                 # IOU

    resp = RecognizeObjectsResponse()
    for det in pred:
        for x0,y0,x1,y1, conf, cls in (det):         # Model Result is bounding box  confidence  and class
            confidence = conf.cpu().tolist()
            if confidence > min_confidence:
                x0 = int(x0.cpu().tolist())
                y0 = int(y0.cpu().tolist())
                x1 = int(x1.cpu().tolist())
                y1 = int(y1.cpu().tolist())
                name = model.names[int(cls.cpu().tolist())]
                if name in remap_obj_names.keys():
                    name = remap_obj_names[name]
                msg, obj_img = get_vision_object(cv_img, name, confidence, "base_link",x0, y0, x1, y1, cloud)
                result_img = cv2.bitwise_or(obj_img, result_img)
                result_img = cv2.putText(result_img, name+" "+str(confidence)[:4],(x0,y0-2),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,255,0),1)
                resp.recog_objects.append(msg)
                print("Cloud size: ", [msg.point_cloud.width, msg.point_cloud.height])
    return resp

def main():
    global device, model, min_confidence, result_img, pub_obj, remove_background, threshold1, threshold2
    print("INITIALIZING OBJECT CLASSIFICATION in an Oscarly manner...")
    rospy.init_node("object_classification")
    model_name = rospy.get_param("~model", "TaRJustv3_ycb.pt")
    min_confidence = rospy.get_param("~min_confidence", 0.5)
    remove_background = rospy.get_param("~rm_bg", True)
    threshold1 = rospy.get_param('~threshold1', 50)
    threshold2 = rospy.get_param('~threshold2', 180)
    loop = rospy.Rate(10)
    print("ObjClassification.->Try to remove background: ", remove_background, " canny th1: ", threshold1, "  cannyh th2: ", threshold2)
    device = select_device('')
    print("ObjClassification.->Loading model: " + model_name)
    model  = attempt_load(model_name, device)
    print("ObjClassification.->Loaded model with labels: ", model.names)
    result_img = numpy.zeros((512, 512, 3), numpy.uint8)
    
    rospy.Service("/vision/obj_reco/detect_and_recognize_objects", RecognizeObjects, callback_recognize_objects)
    rospy.Service("/vision/obj_reco/detect_and_recognize_object" , RecognizeObject , callback_recognize_object)
    pub_obj = rospy.Publisher("/vision/obj_reco/resulting_cloud", PointCloud2, queue_size=30)
    while not rospy.is_shutdown():
        cv2.imshow("YOLO - Recognition Result", result_img)
        cv2.waitKey(10)
        loop.sleep()

if __name__ == "__main__":
    main()
