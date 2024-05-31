#!/usr/bin/env python3
import rospy
import rospkg
import time
from juskeshino_tools.JuskeshinoNavigation import JuskeshinoNavigation
from juskeshino_tools.JuskeshinoVision import JuskeshinoVision
from juskeshino_tools.JuskeshinoHardware import JuskeshinoHardware
from juskeshino_tools.JuskeshinoSimpleTasks import JuskeshinoSimpleTasks
from juskeshino_tools.JuskeshinoHRI import JuskeshinoHRI
from juskeshino_tools.JuskeshinoManipulation import JuskeshinoManipulation
from juskeshino_tools.JuskeshinoKnowledge import JuskeshinoKnowledge
from std_msgs.msg import Bool, Float32, Header
from vision_msgs.msg import HumanCoordinatesArray
from geometry_msgs.msg import Point, PointStamped
import math 

detected = False

def get_by_name(data, goal):
    for human in data.coordinates_array:
        counter = 0
        names = []
        for kpoint in human.keypoints_array:
            names.append(kpoint.keypoint_name)
            if kpoint.keypoint_name == goal:
                return [
                        kpoint.keypoint_coordinates.position.x,
                        kpoint.keypoint_coordinates.position.y,
                        kpoint.keypoint_coordinates.position.z
                ]
            counter += 1
    print("Error, goal doesnt exist, these're the possible names:")
    for name in names:
        print(name)
    return [None, None, None]

def intersect_points_ground(p1, p2):
    line = [
            p2[0]-p1[0],
            p2[1]-p1[1],
            p2[2]-p1[2],
            ]
    t = -p1[2]/line[2]
    intersection = [
            p1[0] + t * line[0],
            p1[1] + t * line[1]
            ,
            0
            ]
    return intersection

def human_callback(data):
    global point_publisher, detected
    if detected:
        return data
    r_elb = get_by_name(data, "r_elb")
    r_sho = get_by_name(data, "r_sho")
    r_wri = get_by_name(data, "r_wri")
    r_hip = get_by_name(data, "r_hip")
    l_elb = get_by_name(data, "l_elb")
    l_sho = get_by_name(data, "l_sho")
    l_wri = get_by_name(data, "l_wri")
    l_hip = get_by_name(data, "l_hip")
    neck = get_by_name(data, "neck")
    """
    if(
            None in r_elb or
            None in r_sho or
            None in r_wri or
            None in r_hip or
    """
    try:
        x_mean = (neck[2] + r_hip[2] + l_hip[2]) / 3
    except:
        print("NO HUMAN POSES DETECTED")
        return
    delta_r = r_wri[0] - x_mean
    delta_l = l_wri[0] - x_mean
    print(x_mean)
    side = 'c'
    if delta_r > delta_l:
        print("Right wrist's farest")
        intersect = intersect_points_ground(
                r_sho,
                r_wri
                )
        print(intersect)
        point = Point()
        pointStamped = PointStamped()
        point.x = intersect[0]
        point.y = intersect[1]
        point.z = intersect[2]
        pointStamped.point = point
        h = Header()
        h.stamp = rospy.Time.now()
        h.frame_id = 'camera_rgb_optical_frame'
        pointStamped.header = h
        point_publisher.publish(pointStamped)
        print(point)
        #JuskeshinoHardware.moveHead(-0.5,0, 5)
        JuskeshinoHardware.moveHead(math.tan(r_wri[1]/r_wri[2]),math.tan(r_wri[0]/r_wri[2]), 5)
        detected = True
        side = 'r'
        JuskeshinoHRI.say("Coul you put the right bag on mi arm please?")
        first = [0, 0, 0, 0, 2, 0, 1]
        JuskeshinoHardware.moveLeftArmWithTrajectory(first, 10)
    elif delta_l > delta_r:
        print("Left wrist's farest")
        intersect = intersect_points_ground(
                l_sho,
                l_wri
                )
        point = Point()
        pointStamped = PointStamped()
        point.x = intersect[0]
        point.y = intersect[1]
        point.z = intersect[2]
        pointStamped.point = point
        h = Header()
        h.stamp = rospy.Time.now()
        h.frame_id = 'camera_rgb_optical_frame'
        pointStamped.header = h
        point_publisher.publish(pointStamped)
        print(point)
        #JuskeshinoHardware.moveHead(0.5,0, 5)
        JuskeshinoHardware.moveHead(math.tan(l_wri[1]/l_wri[2]),math.tan(l_wri[0]/l_wri[2]), 5)
        detected = True
        JuskeshinoHRI.say("Coul you put the left bag on my arm please?")
    else:
        print("IDK")

def main():
    global point_publisher
    rospy.init_node("carry_my_luggage")
    rate = rospy.Rate(10)
    rospack = rospkg.RosPack()
    JuskeshinoNavigation.setNodeHandle()
    JuskeshinoVision.setNodeHandle()
    JuskeshinoHardware.setNodeHandle()
    JuskeshinoSimpleTasks.setNodeHandle()
    JuskeshinoHRI.setNodeHandle()
    JuskeshinoManipulation.setNodeHandle()
    JuskeshinoKnowledge.setNodeHandle()
    rospy.Subscriber(
            "/vision/human_pose/human_pose_array",
            HumanCoordinatesArray,
            human_callback
            )
    point_publisher = rospy.Publisher(
            "/newbies/pointing_site",
            PointStamped,
            queue_size = 1
            )
    rospy.spin()

if __name__ == "__main__":
    main()
