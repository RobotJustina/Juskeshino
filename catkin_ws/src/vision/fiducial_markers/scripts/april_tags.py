#!/usr/bin/env python
import apriltag
import rospy
import cv2
import numpy
import math
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

debug = True

def rotationMatrixToEulerAngles(R):
    sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
    singular = sy < 1e-6
    if  not singular :
        x = math.atan2(R[2,1] , R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
    else :
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0

    return numpy.array([x, y, z])

def get_dist_angle_to_marker(img, intrinsic_matrix):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    options = apriltag.DetectorOptions(families="tag16h5")
    detector = apriltag.Detector(options)
    results = detector.detect(gray)
    if len(results) < 1:
        return [None, None]
    r = results[0]
    if debug:
        (ptA, ptB, ptC, ptD) = r.corners
        ptB = (int(ptB[0]), int(ptB[1]))
        ptC = (int(ptC[0]), int(ptC[1]))
        ptD = (int(ptD[0]), int(ptD[1]))
        ptA = (int(ptA[0]), int(ptA[1]))
        # draw the bounding box of the AprilTag detection
        cv2.line(img, ptA, ptB, (0, 255, 0), 2)
        cv2.line(img, ptB, ptC, (0, 255, 0), 2)
        cv2.line(img, ptC, ptD, (0, 255, 0), 2)
        cv2.line(img, ptD, ptA, (0, 255, 0), 2)
        cv2.imshow("Detection", img)
        cv2.waitKey(10)
    P = intrinsic_matrix
    H = r.homography
    H = H.T
    h1 = H[0]
    h2 = H[1]
    h3 = H[2]
    P_inv = numpy.linalg.inv(P)
    L = 1/numpy.linalg.norm(numpy.dot(P_inv, h1))
    r1 = L * numpy.dot(P_inv, h1)
    r2 = L * numpy.dot(P_inv, h2)
    r3 = numpy.cross(r1, r2)
    T = L * (P_inv @ h3.reshape(3, 1))/4.3
    R = numpy.array([[r1], [r2], [r3]])
    R = numpy.reshape(R, (3, 3))
    euler = rotationMatrixToEulerAngles(R)
    theta = -euler[1]
    return numpy.linalg.norm(T), theta

def callback_img(msg):
    bridge = CvBridge()
    img  = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
    dist,theta = get_dist_angle_to_marker(img, camera_calibration_matrix)
    dist_angle_array = Float32MultiArray()
    dist_angle_array.data = [dist, theta]
    pub_dist_angle.publish(dist_angle_array)
    print([dist, theta])


def main():
    global camera_calibration_matrix, pub_dist_angle, debug
    print("INITIALIZING APRIL TAG DETECTOR")
    rospy.init_node("april_tags")
    debug = rospy.get_param("~debug", True)
    rospy.Subscriber("/camera/depth_registered/rgb/image_raw", Image, callback_img)
    pub_dist_angle = rospy.Publisher("vision/fiducial_markers/april_tag_dist_angle", Float32MultiArray, queue_size = 1)

    camera_info = rospy.wait_for_message("/camera/depth_registered/rgb/camera_info", CameraInfo)
    cp = camera_info.P
    camera_calibration_matrix = numpy.asarray([[cp[0], cp[1], cp[2]], [cp[4], cp[5], cp[6]], [cp[8], cp[9], cp[10]]])
    loop = rospy.Rate(10)
    while not rospy.is_shutdown():
        loop.sleep()

if __name__ == "__main__":
    main()
