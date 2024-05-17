#!/usr/bin/env python
import rospy
import cv2
import numpy
from vision_msgs.srv import RecognizeObject, RecognizeObjects, RecognizeObjectsResponse
from vision_msgs.msg import VisionObject
from cv_bridge import CvBridge
from yolov5.utils.torch_utils import select_device
from yolov5.models.experimental import attempt_load
from yolov5.utils.general import *
import torch

def callback_recognize_objects(req):
    global device, model, min_confidence, result_img
    print("Received request img encodeing " + req.image.encoding)
    bridge = CvBridge()
    img = bridge.imgmsg_to_cv2(req.image)
    result_img = numpy.copy(img)
    img = cv2.cvtColor(img,cv2.COLOR_BGR2RGB)
    img = torch.from_numpy(img).to(device) # RGB IMAGE TENSOR (TORCH)
    img = img / 255.0                              #NORMALIZE
    img = img.unsqueeze(0)                        # ADD DIMENSION FOR TENSOR ( BATCH)
    img = torch.moveaxis(img,3,1)                  #Channel order for YOLO
    pred = model(img, augment=False)[0]
    pred = non_max_suppression(pred)  # IOU

    resp = RecognizeObjectsResponse()
    for det in pred:
        for x0,y0,x1,y1, conf, cls in (det):# Model Result is bounding box  confidence  and class
            confidence = conf.cpu().tolist()
            if confidence > min_confidence:
                x0 = int(x0.cpu().tolist())
                y0 = int(y0.cpu().tolist())
                x1 = int(x1.cpu().tolist())
                y1 = int(y1.cpu().tolist())
                name = model.names[int(cls.cpu().tolist())] 
                result_img=cv2.rectangle(result_img ,(x0,y0), (x1,y1),  (0, 255, 0), 2)
                result_img= cv2.putText(result_img, name+" "+str(confidence),(x0,y0-2),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,255,0),1)
                visobj = VisionObject()
                visobj.id = name
                visobj.confidence = confidence
                resp.recog_objects.append(visobj)
    return resp
    

def main():
    global device, model, min_confidence, result_img
    print("INITIALIZING OBJECT CLASSIFICATION in an Oscarly manner...")
    rospy.init_node("object_classification")
    model_name = rospy.get_param("~model", "ycb.pt")
    min_confidence = rospy.get_param("~min_confidence", 0.5)
    loop = rospy.Rate(10)

    device = select_device('')
    print("ObjClassification.->Loading model: " + model_name)
    model  = attempt_load(model_name, device)
    print("ObjClassification.->Loaded model")
    result_img = numpy.zeros((512, 512, 3), numpy.uint8)

    rospy.Service("/vision/obj_reco/recognize_objects_yolo", RecognizeObjects, callback_recognize_objects)
    while not rospy.is_shutdown():
        cv2.imshow("YOLO - Recognition Result", result_img)
        cv2.waitKey(10)
        loop.sleep()

if __name__ == "__main__":
    main()
