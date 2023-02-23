#! /usr/bin/env python3
import rospy  
import tf2_ros                                    # the main module for ROS-python programs
from std_srvs.srv import Trigger, TriggerResponse # we are creating a 'Trigger service'...
                                                  # ...Other types are available, and you can create
                                                  # custom types


from utils_notebooks import *



protoFile = "/home/roboworks/openpose/models/pose/body_25/pose_deploy.prototxt"
weightsFile = "/home/roboworks/openpose/models/pose/body_25/pose_iter_584000.caffemodel"
#protoFile = "/home/biorob/openpose/models/pose/body_25/pose_deploy.prototxt"
#weightsFile = "/home/biorob/openpose/models/pose/body_25/pose_iter_584000.caffemodel"
net = cv2.dnn.readNetFromCaffe(protoFile, weightsFile)

#############################################################################
def predict_waving(frame):


    # Specify the input image dimensions
    inHeight = frame.shape[0]
    inWidth = frame.shape[1]


    # Prepare the frame to be fed to the network
    inpBlob = cv2.dnn.blobFromImage(frame, 1.0 / 255, (inWidth, inHeight), (0, 0, 0), swapRB=False, crop=False)

    # Set the prepared object as the input blob of the network
    net.setInput(inpBlob)

    output = net.forward()

    H = output.shape[2]
    W = output.shape[3]
    threshold=0.7
    # Empty list to store the detected keypoints
    points = []
    invalid_joints = []

    for i in range(12):
        # confidence map of corresponding body's part.
        probMap = output[0, i, :, :]

        # Find global maxima of the probMap.
        _, prob,_, point = cv2.minMaxLoc(probMap)
        #print (prob,point)

        # Scale the point to fit on the original image
        x = (inWidth * point[0]) / W
        y = (inHeight * point[1]) / H

        if prob > threshold :
        # Add the point to the list if the probability is greater than the threshold
            points.append((int(x), int(y)))
        else :
            points.append(None)
            invalid_joints.append(i)
        
    print(points)
        
    # Logic

    joints1 = [2,3,5,6] # elbows and shoulders
    joints2 =  [3,4,6,7] #

    if any(x in joints1 for x in invalid_joints): #if doesn't find the elbow or shoulder go out
        print("Out")
        return False , (0,0)
    else:
        if points[3][1]<points[2][1] or points[6][1]<points[5][1]: # ask about elbow over shoulder
            print('elbow up') 
            return True, points[0][:]
        else:
            print('elbow down, ask for wrist')
            if any(x in joints2 for x in invalid_joints): # if doesn't find the wrist go out 
                print("Out")
                return False, points[0][:]
            else:
                if points[4][1]<points[3][1] or points[7][1]<points[6][1]: # ask about wrist over elbow
                    print('Hand up')
                    return True, points[0][:]
                else:
                    print('Hand down, go Out')
                    return False ,  points[0][:]
           
############################################



def trigger_response(request):
    ''' 
    Callback function used by the service server to process
    requests from clients. It returns a TriggerResponse
    '''
    
    results = []
    #for i in range(5):
    rgbd= RGBD()
    rospy.sleep(0.35)
    frame= rgbd.get_image()
    points= rgbd.get_points()
    rospy.sleep(0.35)
    red , face_coords =predict_waving(frame)
    print (points.shape)
    xyz_wrt_robot= points[ face_coords[1],face_coords[0] ]
    
    #results.append(red)

    #true_count = sum(results)

     # solo falta que decida por el promedio
             

    

    return TriggerResponse(
        success=red,
        message= str( xyz_wrt_robot)
    )


rospy.init_node('waving_server') 
listener = tf.TransformListener()
broadcaster= tf.TransformBroadcaster()
tf_static_broadcaster= tf2_ros.StaticTransformBroadcaster()
rospy.loginfo("waving detection service available")                    # initialize a ROS node
my_service = rospy.Service(                        # create a service, specifying its name,
    '/detect_waving', Trigger, trigger_response         # type, and callback
)
rospy.spin()   
