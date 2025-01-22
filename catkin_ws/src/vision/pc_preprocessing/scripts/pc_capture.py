#!/usr/bin/env python
import cv2
import numpy
import rospy
import ros_numpy
from sensor_msgs.msg import PointCloud2

def main():
    print("Initializing cloud capturer by Marcosoft...")
    rospy.init_node("cloud_capturer")
    cloud = rospy.wait_for_message("/camera/depth_registered/points", PointCloud2)
    print("Received cloud with " + str(cloud.width) + "x" + str(cloud.height) + " points")
    xyz = ros_numpy.point_cloud2.pointcloud2_to_array(cloud)
    rgb_arr = xyz['rgb'].copy()
    rgb_arr.dtype = numpy.uint32
    r = numpy.asarray(((rgb_arr >> 16) & 255), dtype='uint8')
    g = numpy.asarray(((rgb_arr >>  8) & 255), dtype='uint8')
    b = numpy.asarray(((rgb_arr      ) & 255), dtype='uint8')
    bgr = cv2.merge((b,g,r))

    xyz = xyz.reshape((cloud.width*cloud.height,1))
    bgr = bgr.reshape((cloud.width*cloud.height,1,3))
    data = numpy.zeros((cloud.width*cloud.height, 6))
    for i in range(cloud.width*cloud.height):
        data[i,0] = xyz[i]["x"]
        data[i,1] = xyz[i]["y"]
        data[i,2] = xyz[i]["z"]
        data[i,3] = bgr[i,0,2]/255.0
        data[i,4] = bgr[i,0,1]/255.0
        data[i,5] = bgr[i,0,0]/255.0
    numpy.savetxt("cloud.txt", data, delimiter=",")
        

if __name__ == "__main__":
    main()
