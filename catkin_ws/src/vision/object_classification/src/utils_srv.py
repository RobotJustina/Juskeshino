# -*- coding: utf-8 -*-
import tf as tf
import tf2_ros
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist, PointStamped, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
import pandas as pd
import ros_numpy
from gazebo_ros import gazebo_interface
from sklearn.decomposition import PCA
import math as m
# import moveit_commander
# import moveit_msgs.msg
from cv_bridge import CvBridge, CvBridgeError
import rospkg

import actionlib
import subprocess

from geometry_msgs.msg import PoseStamped, Quaternion, TransformStamped, Twist, Pose
import geometry_msgs.msg
from IPython.display import Image
# from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal #TODO: ModuleNotFoundError: No module named 'move_base_msgs'
from sensor_msgs.msg import LaserScan, PointCloud2
import sys
# sys.path.remove('/opt/ros/melodic/lib/python2.7/dist-packages')
import cv2
global listener2, tfBuffer, broadcaster,  tf_static_broadcaster


# -----------------------------------------------------------------
def read_tf(t):
    # trasnform message to np arrays
    pose = np.asarray((
        t.transform.translation.x,
        t.transform.translation.y,
        t.transform.translation.z
    ))
    quat = np.asarray((
        t.transform.rotation.x,
        t.transform.rotation.y,
        t.transform.rotation.z,
        t.transform.rotation.w
    ))

    return pose, quat

# -----------------------------------------------------------------


def write_tf(pose, q, child_frame="", parent_frame='map'):
    #  pose = trans  q = quaternion  , childframe =""
    # format  write the transformstampled message
    t = TransformStamped()
    t.header.stamp = rospy.Time.now()
    # t.header.stamp = rospy.Time(0)
    t.header.frame_id = parent_frame
    t.child_frame_id = child_frame
    t.transform.translation.x = pose[0]
    t.transform.translation.y = pose[1]
    t.transform.translation.z = pose[2]
    # q = tf.transformations.quaternion_from_euler(eu[0], eu[1], eu[2])
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]
    return t
# -----------------------------------------------------------------


class RGBD():
    u"""RGB-Dデータを扱うクラス"""

    def __init__(self):
        self._br = tf.TransformBroadcaster()
        # ポイントクラウドのサブスクライバのコールバックに_cloud_cbメソッドを登録
        self._cloud_sub = rospy.Subscriber(
            "/camera/depth_registered/points",
            PointCloud2, self._cloud_cb)
        self._points_data = None
        self._image_data = None
        self._h_image = None
        self._region = None
        self._h_min = 0
        self._h_max = 0
        self._xyz = [0, 0, 0]
        self._frame_name = None

    def _cloud_cb(self, msg):
        # ポイントクラウドを取得する
        self._points_data = ros_numpy.numpify(msg)
        # 画像を取得する
        self._image_data = \
            self._points_data['rgb'].view((np.uint8, 4))[..., [2, 1, 0]]

    def get_image(self):
        u"""画像を取得する関数"""
        return self._image_data

    def get_points(self):
        u"""ポイントクラウドを取得する関数"""
        return self._points_data
