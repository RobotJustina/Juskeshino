#! /usr/bin/env python3


from utils_pointing import *


def trigger_response(request):
    print('Segmenting')
    # points_msg=rospy.wait_for_message("/hsrb/head_rgbd_sensor/depth_registered/rectified_points",PointCloud2,timeout=5)#TAKESHI
    points_msg = rospy.wait_for_message(
        "/camera/depth_registered/points", PointCloud2, timeout=5)

    dist = 6 if request.dist == 0 else request.dist
    # print("\n\nDISTANCIA",dist,"\n\n")
    res = detect_pointing(points_msg, dist)

    return res


def callback(request):
    print('Segmenting')
    print("\n\nDISTANCIA", request.dist, "\n\n")
    #points_msg = rospy.wait_for_message("/hsrb/head_rgbd_sensor/depth_registered/rectified_points", PointCloud2, timeout=5)
    points_msg=rospy.wait_for_message("/camera/depth_registered/points",PointCloud2,timeout=5)

    dist = 6 if request.dist == 0 else request.dist
    # print("\n\nDISTANCIA",dist,"\n\n")
    res = detect_human(points_msg, dist)
    print(res)
    return res


# initialize a ROS node
rospy.loginfo("human pose detection service available")
my_service = rospy.Service(                        # create a service, specifying its name,
    '/detect_pointing', Point_detector, trigger_response) # type, and callback

service2 = rospy.Service('/detect_human', Human_detector, callback)

rospy.spin()
