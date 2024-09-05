#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker, MarkerArray

def publish_marker_array():
    # Initialize the ROS node
    rospy.init_node('markers')

    # Create a Publisher for the MarkerArray
    marker_array_publisher = rospy.Publisher('marker_array', MarkerArray, queue_size=10)

    # Create a MarkerArray message
    marker_array = MarkerArray()

   # listx=[6.14,5.65,9.71,9.68,7.32,5.49,8,9.75,7.9,9,8,7.2,
   # 4.5,4.8,4.8,9.2,4.7,6.6,6.6, 6.4, 5.8, 5,6,6.3, 6.0,5.3,
   # 6.7,5.6,5.8,9.5,6.7,9.6,8,7.3,8,9,5.9,8.8,7.2,6.1,5.6,5.3,5.2,
   # 5.2,5,7.9]

    listx=[0.80, 1.39, 2.00, 2.47, 4.38, 4.76, 0.76, 1.53, 2.63, 3.51, 4.65, 0.51, 1.53, 2.58, 3.92, 1.14, 1.01, 2.03, 3.07, 3.68, 4.13, 4.62, 0.69, 2.04, 3.89, 4.69, 0.54, 2.04, 2.55, 3.93,
    0.63, 2.33, 3.35, 0.60, 1.52, 2.55, 3.88, 0.50, 2.85, 2.55, 3.88, 0.71, 3.43,
    3.95, 3.96,0.53,4.17,1.53,2.53,3.6]
    print(len(listx))

    #listy=[9,5.18,7.75,4.94,1.77,1.68,7.88,7,6,0.5,0.5,0.5,
    #1.5,2.3,0.7,3.6,5.8,6.4,7.2,5.3,7.6,7,3.9,1.25,0.7,3.2,
    #3,6.4,7,5.9,2.3,4.1,5.3,1.3,1.3,1.7,4.7,7.8,7.8,5.8,2.4,4.5,3.9,
    #5.8,7.5,6.9]
    listy=[9.7,8.7,9,5.45,7.6,7,8.35,0.9,1.4,0.85,6.5,8.3,8.9,9,7.7,1,7.75,6.8,
    6,5.3,5.4,6.5,5.5,5.6,1,1.35,0.83]
    listy=[9.16, 9.38, 9.28, 9.29, 9.34, 9.28, 8.53, 8.83, 8.72, 9.00, 8.69, 8.17, 8.15, 8.37, 8.34, 7.69, 7.14, 7.19, 7.16, 7.23, 7.11, 7.40, 6.64, 6.63, 6.17, 6.04, 5.62, 5.53, 5.23, 5.53, 
    5.15, 4.90, 4.92, 4.12, 4.40, 4.48, 4.09, 3.56, 6.08, 2.73, 2.45, 2.65, 1.1,
    1.6, 4.89,1.58,0.83,1.04,1.01,3.28]

    print(len(listy))

    r=[0 for _ in range(len(listy))]
    #index=[0,2,6,7,9,10,17,18,20,28,29,34,35,37,38,44,45]
    #index=[3,9,10,18,34,35]
    index=[7,11]
    for i in index:
        r[i]=1

    # Populate the MarkerArray with individual markers
    for i in range(len(listx)):  # You can adjust the number of markers as needed
        marker = Marker()
        marker.header.frame_id = "odom"  # Adjust the frame_id as needed
        marker.header.stamp = rospy.Time.now()
        marker.id = i
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = listx[i]  # Adjust the position as needed
        marker.pose.position.y = listy[i]
        marker.pose.position.z = 0.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        marker.color.a = 1.0
        marker.color.r = r[i]
        marker.color.g = 1-r[i]
        marker.color.b = 0.0

        marker_array.markers.append(marker)

    # Publish the MarkerArray
    rate = rospy.Rate(1)  # Adjust the rate as needed
    while not rospy.is_shutdown():
        marker_array_publisher.publish(marker_array)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_marker_array()
    except rospy.ROSInterruptException:
        pass
