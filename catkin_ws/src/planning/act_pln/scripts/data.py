#!/usr/bin/env python

import rospy
import ros_numpy
import numpy as np
import yaml
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float64MultiArray
from vision_msgs.srv import RecognizeObject, RecognizeObjectRequest
from manip_msgs.srv import BestGraspTraj, BestGraspTrajRequest


# Data collection
data ={
    "object_name": "",
    "object_category": "",
    "object_position": {
        "x": "",
        "y": "",
        "z": "",
    },

    "object_orientation": {
        "x": "",
        "y": "",
        "z": "",
        "w": "",
    },

    "object_point_cloud": "",

    "x_gripper": "",
    "y_gripper": "",
    "z_gripper": "",
    "roll_gripper": "",
    "yaw_gripper": "", 

    "head_pose_pan": "",
    "head_pose_tilt": "",
}

def detect_and_recognize_object(name):
    rospy.wait_for_service('/vision/obj_reco/detect_and_recognize_object')
    
    req = RecognizeObjectRequest()
    req.name = name
    req.point_cloud = rospy.wait_for_message("/camera/depth_registered/points", PointCloud2, timeout=1.0)
    
    try:    
        detectAndRecognizeObject = rospy.ServiceProxy('/vision/obj_reco/detect_and_recognize_object', RecognizeObject)
        print("hola")
        resp = detectAndRecognizeObject(req)
        print("despues hola")
        if resp is None:
            print("No se pudo reconocer el objesto")
            return
        else:
            return resp.recog_object, resp.image

        """
            resp.recog_object --> type VisionObject.msg
            resp.image --> type Image
        """
    except rospy.ServiceException as e:
        rospy.logwarn("Service failed: " + str(e))

"""
VisionObject.msg

std_msgs/Header header
string id                                    #name of identifying the object (milk, orange juice, etc) or color (green,blue, etc)
string category                              #object type (drink, snack, etc)
float32 confidence                           #value in [0,1] indicating the probability of a correct identification
                                             #in the case of color objects indiates the grasping priority
sensor_msgs/Image image			     #image in rgb of object;
sensor_msgs/Image obj_mask		      #binary image of object
sensor_msgs/PointCloud2 point_cloud          #Point cloud (probably, colored) of the object
geometry_msgs/Vector3 size                   #Size in meters: size in x, y and z
geometry_msgs/Pose pose                      #Centroid and orientation
geometry_msgs/Vector3[] bounding_box         #Two vectors indicating the bounding box
geometry_msgs/Vector3[] bounding_polygon     #N vectors 
int32 x					     #top left x
int32 y					     #top left y
int32 width				     #top widht
int32 height				     #top height
bool graspable                               #graspable by the gripper
std_msgs/ColorRGBA color_rgba	             #Mean object's color
float32 alfa				     #angle of the largest dimension of the object with respect to the surface
string object_state			     #inclination of the collinear axis to the longest dimension of the object
"""


"""
RecognizeObject.srv

int32 iterations
sensor_msgs/PointCloud2 point_cloud  #If recognition is made only with RGB image, this is empty
sensor_msgs/Image image              #If recognition is made with PointCloud, this is empty
string name                          #Requested object's name
sensor_msgs/Image obj_mask	      #binary image of object
---
vision_msgs/VisionObject recog_object
sensor_msgs/Image image              #Althoug every object has its own point_cloud and image, this field
                                     #is intended to show the recognized object in the original image. 
"""

def get_object_pose_with_orientation(object_resp): #vision object
    rospy.wait_for_service('/vision/obj_segmentation/get_obj_pose_with_orientation')
    
    req = RecognizeObjectRequest()
    req.name = object_resp.id
    req.point_cloud = object_resp.point_cloud
    req.obj_mask = object_resp.obj_mask

    
    try:    
        getObjectPoseWithOrientation = rospy.ServiceProxy('/vision/obj_segmentation/get_obj_pose_with_orientation', RecognizeObject)
        resp = getObjectPoseWithOrientation(req)
        return resp.recog_object, resp.image
        """
            resp.recog_object --> type VisionObject.msg
            resp.image --> type Image
        """
    except rospy.ServiceException as e:
        rospy.logwarn("Service failed: " + str(e))



def get_best_grasp_trajectory(name):
    rospy.wait_for_service('/manipulation/get_best_grasp_traj')

    req = BestGraspTrajRequest()
    req.recog_object = name

    try:
        getBestGraspTrayectory = rospy.ServiceProxy("/manipulation/get_best_grasp_traj", BestGraspTraj)
        resp = getBestGraspTrayectory(req)
        return resp
        """
            resp --> type BestGRaspTraj
        """

    except rospy.ServiceException as e:
        rospy.logwarn("Service failed: " + str(e))


"""
BestGRaspTraj.srv
(request)
vision_msgs/VisionObject recog_object                    
---
(response)
trajectory_msgs/JointTrajectory articular_trajectory
bool graspable                               #graspable by the gripper
float64 x_gripper			
float64 y_gripper
float64 z_gripper
float64 roll_gripper
float64 pitch_gripper
float64 yaw_gripper
float64[] q	# Send generic trajectory

"""


def get_head_current_pose():
    current_pose = rospy.wait_for_message("/hardware/head/current_pose", Float64MultiArray, timeout=1.0)
    return current_pose


def main():
    rospy.init_node("data")
    object_name = 'apple'
    
    #rate = rospy.Rate(10)
    #while not rospy.is_shutdown():

    # Detect and recognize object
    recog_object, image = detect_and_recognize_object(object_name)
    print(recog_object.category) #no aparece

    # Save data
    data['object_name'] = recog_object.id
    data['object_position']['x'] = recog_object.pose.position.x
    data['object_position']['y'] = recog_object.pose.position.y
    data['object_position']['z'] = recog_object.pose.position.z

    data['object_orientation']['x'] = recog_object.pose.orientation.x
    data['object_orientation']['y'] = recog_object.pose.orientation.y
    data['object_orientation']['z'] = recog_object.pose.orientation.z
    data['object_orientation']['w'] = recog_object.pose.orientation.z

    recog_object_orientated, image_orientated = get_object_pose_with_orientation(recog_object)
    print("category", recog_object_orientated.category)
    print("state", recog_object_orientated.object_state)
    
    data['object_category'] = recog_object_orientated.category
    # ----------------------------------- Save Point Cloud ---------------------------------
    objectPointCloud = recog_object.point_cloud
    numpy_arr_objectPointCloud = ros_numpy.point_cloud2.pointcloud2_to_array(objectPointCloud) 
    
    filename = object_name + '_' + 'point_cloud.npz'
    print(filename)

    # Save the arrays to a npz file
    np.savez(filename, numpy_arr_objectPointCloud)

    # Load the arrays from the file
    loaded_arrays = np.load(filename)
    print("Point cloud from npz file:", loaded_arrays['arr_0'])

    # Save local-path file
    data['object_point_cloud']="/" + filename 

    # -----------------------------------------------------------------------------------


    # Get current head pose
    current_head_pose = get_head_current_pose()
    """
    std_msgs/Float64MultiArray Message
        MultiArrayLayout  layout        # specification of data layout
        float64[]         data          # array of data

    """
    data['head_pose_pan'] =  current_head_pose.data[0]
    data['head_pose_tilt'] =  current_head_pose.data[1]



    # Generate best grasp trayectory
    #grasp = get_best_grasp_trajectory(recog_object)
    grasp = get_best_grasp_trajectory(recog_object_orientated)
    print("x gripper:", grasp.x_gripper)
    print("y gripper:", grasp.y_gripper)
    print("z gripper:", grasp.z_gripper)
    print("articular trajectory:", grasp.articular_trajectory.points)
    print("DATA LAST POSITION ARM")
    print("Posición de q's:", grasp.articular_trajectory.points[-1].positions)

    # Save data
    data['x_gripper'] = grasp.x_gripper
    data['y_gripper'] = grasp.y_gripper
    data['z_gripper'] = grasp.z_gripper
    data['roll_gripper'] = grasp.roll_gripper
    data['yaw_gripper'] = grasp.yaw_gripper

    print(data)

    # Store data
    with open('data.yaml', 'w') as file:
        yaml.dump(data, file)

    #    rate.sleep()
    
    
if __name__ == '__main__':
    main()