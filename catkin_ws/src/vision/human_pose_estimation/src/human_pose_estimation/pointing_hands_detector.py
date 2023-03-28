#!/usr/bin/env python3
# coding:UTF-8

import math
import numpy as np

import rospy
import smach
import tf
import tf2_ros
import tf2_geometry_msgs

import traceback

# import my custom msg
from geometry_msgs.msg import Pose,PoseArray,PoseStamped
from vision_msgs.msg import Keypoint 
from vision_msgs.msg import HumanCoordinates 
from vision_msgs.msg import HumanCoordinatesArray

class PointingHandsDetector(smach.State):
    
    def __init__(self,timeout=60.,tfBuffer=None):
        smach.State.__init__(self,
                             outcomes=['success','failure','timeout'],
                             output_keys=['human_coordinate'])

        self.human_coordinate = None

        self.timeout = timeout

        self.trace_keypoint = 0
        self.trace_counter = 0
        self.timeout = timeout
        self.frame = 0

        self.xtion_hight = 1.17
        self.xtion_max_depth = 5.00
        self.xtion_min_depth = 0.30
        self.xtion_r_width = 1.5
        self.xtion_l_width = -1.5

        if tfBuffer is None:
            self.tfBuffer = tf2_ros.Buffer()
            self.listener = tf2_ros.TransformListener(self.tfBuffer)
        else:
            self.tfBuffer = tfBuffer
        self.br = tf.TransformBroadcaster()


    def callback(self, data):

        people = data.coordinates_array
        #print("how many people = ", data.number_of_people)

        if people is None:
            return

        try:

            for person in people:
                person_id = person.person_id
                keypoints = {kpt.keypoint_name:kpt for kpt in person.keypoints_array if kpt.keypoint_name in ['neck', 'r_elb', 'r_wri', 'l_elb', 'l_wri', 'r_hip', 'l_hip']}

                neck  = [keypoints["neck"].keypoint_coordinates.position.x,
                         keypoints["neck"].keypoint_coordinates.position.y, 
                         keypoints["neck"].keypoint_coordinates.position.z] 

                r_wri = [keypoints["r_wri"].keypoint_coordinates.position.x,
                         keypoints["r_wri"].keypoint_coordinates.position.y, 
                         keypoints["r_wri"].keypoint_coordinates.position.z]
   
                l_wri = [keypoints["l_wri"].keypoint_coordinates.position.x,
                         keypoints["l_wri"].keypoint_coordinates.position.y,
                         keypoints["l_wri"].keypoint_coordinates.position.z]

                r_elb = [keypoints["r_elb"].keypoint_coordinates.position.x,
                         keypoints["r_elb"].keypoint_coordinates.position.y, 
                         keypoints["r_elb"].keypoint_coordinates.position.z]
   
                l_elb = [keypoints["l_elb"].keypoint_coordinates.position.x,
                         keypoints["l_elb"].keypoint_coordinates.position.y,
                         keypoints["l_elb"].keypoint_coordinates.position.z]

                r_hip = [keypoints["r_hip"].keypoint_coordinates.position.x,
                         keypoints["r_hip"].keypoint_coordinates.position.y,
                         keypoints["r_hip"].keypoint_coordinates.position.z]

                l_hip = [keypoints["l_hip"].keypoint_coordinates.position.x,
                         keypoints["l_hip"].keypoint_coordinates.position.y,
                         keypoints["l_hip"].keypoint_coordinates.position.z]

                # add c_hip
                c_hip_x = (r_hip[0] + l_hip[0]) / 2
                c_hip_y = (r_hip[1] + l_hip[1]) / 2
                c_hip_z = (r_hip[2] + l_hip[2]) / 2

                # vectors for pointing gesture detection
                r_elbow2wrist = np.array([r_wri[0] - r_elb[0], \
                                         r_wri[1] - r_elb[1], \
                                         r_wri[2] - r_elb[2]])

                l_elbow2wrist = np.array([l_wri[0] - l_elb[0], \
                                         l_wri[1] - l_elb[1], \
                                         l_wri[2] - l_elb[2]])

                r_camera2elbow = np.array([r_wri[0] - 0, \
                                           r_wri[1] - 0, \
                                           r_wri[2] - 0])

                l_camera2elbow = np.array([l_wri[0] - 0, \
                                           l_wri[1] - 0, \
                                           l_wri[2] - 0])

                r_pose_array = PoseArray()
                r_pose_array.header.stamp = rospy.Time.now()
                r_pose_array.header.frame_id = "head_rgbd_sensor_rgb_frame"

                l_pose_array = PoseArray()
                l_pose_array.header.stamp = rospy.Time.now()
                l_pose_array.header.frame_id = "head_rgbd_sensor_rgb_frame"
                                              
                rt = 1
                lt = 1

                while not rospy.is_shutdown():

                    #Intersection of a line and a plane
                    r_camera2wrist = r_camera2elbow + rt * r_elbow2wrist
                    r_distance = math.sqrt((r_camera2elbow[0] - r_camera2wrist[0])**2 + (r_camera2elbow[1] - r_camera2wrist[1])**2 + (r_camera2elbow[2] - r_camera2wrist[2])**2)

                    l_camera2wrist = l_camera2elbow + lt * l_elbow2wrist
                    l_distance = math.sqrt((l_camera2elbow[0] - l_camera2wrist[0])**2 + (l_camera2elbow[1] - l_camera2wrist[1])**2 + (l_camera2elbow[2] - l_camera2wrist[2])**2)

                    if (r_distance and l_distance) < self.xtion_max_depth:
                        if self.xtion_hight > (r_camera2wrist[1] and l_camera2wrist[1]):

                             r_pose = Pose()
                             r_pose.position.x = r_camera2wrist[0]
                             r_pose.position.y = r_camera2wrist[1]
                             r_pose.position.z = r_camera2wrist[2]
                             r_pose.orientation.w = 1.0
                             r_pose_array.poses.append(r_pose)
                             rt += 1

                             l_pose = Pose()
                             l_pose.position.x = l_camera2wrist[0]
                             l_pose.position.y = l_camera2wrist[1]
                             l_pose.position.z = l_camera2wrist[2]
                             l_pose.orientation.w = 1.0
                             l_pose_array.poses.append(l_pose)
                             lt += 1
                        else:
                            break
                    else:
                        break

                    ### xtion frame to basefoot frame ###
                    r_pose_stamped = PoseStamped()
                    r_pose_stamped.header.frame_id = data.header.frame_id 
                    r_pose_stamped.header.stamp = rospy.Time.now()
                    r_pose_stamped.pose.position.x = r_pose_array.poses[-1].position.x
                    r_pose_stamped.pose.position.y = r_pose_array.poses[-1].position.y
                    r_pose_stamped.pose.position.z = r_pose_array.poses[-1].position.z
                    r_pose_stamped.pose.orientation.w = 1
                    #print(r_pose_stamped)

                    l_pose_stamped = PoseStamped()
                    l_pose_stamped.header.frame_id = data.header.frame_id 
                    l_pose_stamped.header.stamp = rospy.Time.now()
                    l_pose_stamped.pose.position.x = l_pose_array.poses[-1].position.x
                    l_pose_stamped.pose.position.y = l_pose_array.poses[-1].position.y
                    l_pose_stamped.pose.position.z = l_pose_array.poses[-1].position.z
                    l_pose_stamped.pose.orientation.w = 1
                    #print(l_pose_stamped)

                    c_pose_stamped = PoseStamped()
                    c_pose_stamped.header.frame_id = data.header.frame_id 
                    c_pose_stamped.header.stamp = rospy.Time.now()
                    c_pose_stamped.pose.position.x = c_hip_x
                    c_pose_stamped.pose.position.y = c_hip_y
                    c_pose_stamped.pose.position.z = c_hip_z
                    c_pose_stamped.pose.orientation.w = 1
                    #print(c_pose_stamped)

                    r_pose2map = self.tfBuffer.transform(r_pose_stamped, 'map', timeout=rospy.Duration(1))
                    l_pose2map = self.tfBuffer.transform(l_pose_stamped, 'map', timeout=rospy.Duration(1))
                    c_pose2map = self.tfBuffer.transform(c_pose_stamped, 'map', timeout=rospy.Duration(1))

                    origin2rx = r_pose_stamped.pose.position.x - c_pose_stamped.pose.position.x
                    origin2ry = r_pose_stamped.pose.position.y - c_pose_stamped.pose.position.y
                    origin2lx = l_pose_stamped.pose.position.x - c_pose_stamped.pose.position.x
                    origin2ly = l_pose_stamped.pose.position.y - c_pose_stamped.pose.position.y

                    origin2rxy = origin2rx**2 + origin2ry**2
                    origin2lxy = origin2lx**2 + origin2lx**2

                    sqrt_r = math.sqrt(origin2rxy)
                    sqrt_l = math.sqrt(origin2lxy)

                    if sqrt_r > sqrt_l:
                        print("Pointing by R hand")
                    if sqrt_r < sqrt_l:
                        print("Pointing by L hand")

        except KeyError:
            pass

        except IndexError:
            pass

    def execute(self,userdata):
        try:
            sub = rospy.Subscriber('/human_coordinates_array', HumanCoordinatesArray, self.callback) 
            start_time = rospy.Time.now()
            while not rospy.is_shutdown():
                if self.human_coordinate:
    
                    userdata.human_coordinate = self.human_coordinate
                    print("to userdata variable ->> ",self.human_coordinate)
                    sub.unregister()
                    return 'success'
    
                if (rospy.Time.now() - start_time).to_sec() > self.timeout:
                    sub.unregister()
                    return 'timeout'
        except:
            rospy.logger(traceback.format_exc())
            sub.unregister()
            return 'failure'

if __name__ == '__main__':
        rospy.init_node('pointing_hand_detection_state')
        sm = smach.StateMachine(outcomes=['success','failure'])

        with sm:
            smach.StateMachine.add('DEBUG', PointingHandsDetector(),
                                   transitions={'success': 'success',
                                                'timeout': 'failure',
                                                'failure': 'failure'})
        sm.execute()
