#! /usr/bin/env python3

import tf
import tf2_ros as tf2
import numpy as np
import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
from hmm_navigation.msg import NavigateAction, NavigateActionGoal, NavigateActionFeedback, NavigateActionResult
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PointStamped, PoseStamped, Quaternion, Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Empty, String

#from utils.grasp_utils import *
import rospkg
import yaml

class TF_MANAGER():
    def __init__(self):
        self._tfbuff = tf2.Buffer()
        self._lis = tf2.TransformListener(self._tfbuff)
        self._tf_static_broad = tf2.StaticTransformBroadcaster()
        self._broad = tf2.TransformBroadcaster()

    def _fillMsg(self, pos=[0, 0, 0], rot=[0, 0, 0, 1], point_name='', ref="map"):
        TS = TransformStamped()
        TS.header.stamp = rospy.Time.now()
        TS.header.frame_id = ref
        TS.child_frame_id = point_name
        TS.transform.translation = Point(*pos)
        TS.transform.rotation = Quaternion(*rot)
        return TS

    def pub_tf(self, pos=[0, 0, 0], rot=[0, 0, 0, 1], point_name='', ref="map"):
        dinamic_ts = self._fillMsg(pos, rot, point_name, ref)
        self._broad.sendTransform(dinamic_ts)

    def pub_static_tf(self, pos=[0, 0, 0], rot=[0, 0, 0, 1], point_name='', ref="map"):
        static_ts = self._fillMsg(pos, rot, point_name, ref)
        self._tf_static_broad.sendTransform(static_ts)

    def change_ref_frame_tf(self, point_name='', rotational=[0, 0, 0, 1], new_frame='map'):
        try:
            traf = self._tfbuff.lookup_transform(
                new_frame, point_name, rospy.Time(0))
            translation, _ = self.tf2_obj_2_arr(traf)
            self.pub_static_tf(pos=translation, rot=rotational,
                               point_name=point_name, ref=new_frame)
            return True
        except:
            return False

    def getTF(self, target_frame='', ref_frame='map'):
        try:
            tf = self._tfbuff.lookup_transform(
                ref_frame, target_frame, rospy.Time(0))
            return self.tf2_obj_2_arr(tf)
        except:
            return [False, False]

    def tf2_obj_2_arr(self, transf):
        pos = []
        pos.append(transf.transform.translation.x)
        pos.append(transf.transform.translation.y)
        pos.append(transf.transform.translation.z)

        rot = []
        rot.append(transf.transform.rotation.x)
        rot.append(transf.transform.rotation.y)
        rot.append(transf.transform.rotation.z)
        rot.append(transf.transform.rotation.w)

        return [pos, rot]
##################################################
# Por probar
class nav_status():
    def __init__(self):
        self.suscriber = rospy.Subscriber(
            '/navigation/status',
            GoalStatus, self._callback)
        self._status = None

    def _callback(self, goal_status):
        self._status = goal_status.status

    def get_status(self):
        return self._status

##################################################
# YAML reader

## PARAM  FILE
def read_yaml(known_locations_file='/known_locations.yaml'):
    rospack = rospkg.RosPack()
    file_path = rospack.get_path('config_files') + known_locations_file

    with open(file_path, 'r') as file:
        content = yaml.safe_load(file)
    return content


def match_location(f_name, location):
    content = read_yaml(f_name)
    try:
        return True, content[location]
    except:
        return False, 'No location found'
##################################################
# Navigation functions


def pose2feedback(pose_robot, quat_robot, timeleft, euclD , anglD):
    feed = NavigateActionFeedback()
    feed.feedback.x_robot = pose_robot[0]
    feed.feedback.y_robot = pose_robot[1]
    _, _, yaw = tf.transformations.euler_from_quaternion(quat_robot)
    feed.feedback.yaw_robot = yaw
    feed.feedback.timeleft = timeleft
    feed.feedback.euclD = euclD
    feed.feedback.anglD = anglD
    return feed


def goal_police(goal):
    # goal corrector
    goal_corrected = goal

    return goal_corrected

##################################################


class PumasNavServer():
    def __init__(self):
        self.pumas_nav_server = actionlib.SimpleActionServer(
            "navigate", NavigateAction,
            execute_cb=self.execute_cb,
            auto_start=False)
        self.pumas_nav_server.start()

    def execute_cb(self, goal):
        success = False
        #arm.set_named_target('go')
        # Matching known location if given
        x, y, yaw = goal.x, goal.y, goal.yaw
        known_loc = goal.known_location.casefold()
        if known_loc != 'None':
            succ, loc = match_location(file_name, known_loc)
            if succ:
                XYT = loc[:3]
                x, y, yaw = XYT[0]['x'], XYT[1]['y'], XYT[2]['theta']
                goal.x, goal.y, goal.yaw = x, y, yaw

        #publish head and arm movements instead of making them here
        #head.set_joint_values(head_pose=[0.0, -0.5])
        # Fill result message (could be modified)
        result = NavigateActionResult()
        rate = rospy.Rate(10)
        timeout = rospy.Time.now().to_sec() + goal.timeout
        rot = tf.transformations.quaternion_from_euler(0, 0, yaw)
        goal_pose = PoseStamped()
        goal_pose.pose.position = Point(x, y, 0)
        goal_pose.pose.orientation = Quaternion(*rot)
        goal_nav_publish.publish(goal_pose)

        # Feedback publisher and timeout checker
        while (timeout >= rospy.Time.now().to_sec()) and not success:
            # Goal distance and orientation calculation
            c_pose, q_rob = tf_man.getTF(target_frame='base_link')
            _, _, rob_yaw = tf.transformations.euler_from_quaternion(q_rob)
            #rob_yaw = rob_yaw % (2 * np.pi)
            #anglD = (yaw - rob_yaw)
            anglD = (yaw - rob_yaw)

                #UGLY
            if anglD < -2*np.pi :
                anglD= -1*anglD%(2*np.pi)
            elif anglD > 2*np.pi :
                anglD= anglD%(2*np.pi)


            if anglD < -np.pi :
                anglD= (anglD%np.pi)
            elif anglD > np.pi:
                anglD=(anglD%np.pi)*-1

            euclD = np.linalg.norm(np.asarray((x, y)) - c_pose[:2])
            print (anglD)
            timeleft = timeout - rospy.Time.now().to_sec()
            if timeleft > 0:
                print(timeleft, 'timeleft')
                feed = pose2feedback(c_pose, q_rob, timeleft, euclD,anglD)
                self.pumas_nav_server.publish_feedback(feed.feedback)

            # state = NS.get_status()

            # success = state == 3 and euclD < 0.05 and anglD < 0.3
            success = euclD < 0.45 and abs(anglD) < 0.1

        # state = NS.get_status()
        #head.set_joint_values(head_pose = [0.0, 0.0])
        rospy.sleep(0.8)
        pub_stop.publish()
        if not success:
            self.pumas_nav_server.set_aborted()
        else:
            self.pumas_nav_server.set_succeeded()


if __name__ == "__main__":
    global goal_nav_publish, pub_stop, tf_man,  file_name
    rospy.init_node('pumas_navigation_actionlib_server')

  

    #head = GAZE()
    #arm = ARM()

    tf_man = TF_MANAGER()
    goal_nav_publish = rospy.Publisher(
        '/move_base_simple/goal', PoseStamped, queue_size=1)
    file_name = rospy.get_param("~file_name", "/known_locations.yaml")
    #tf_man = TF_MANAGER()
    pub_stop = rospy.Publisher('/navigation/stop', Empty, queue_size=10)
    print('pumas nav action server available')
    # NS = nav_status()
    s = PumasNavServer()
    rospy.spin()
