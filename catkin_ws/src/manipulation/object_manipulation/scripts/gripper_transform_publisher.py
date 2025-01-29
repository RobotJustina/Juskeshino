#!/usr/bin/env python

import roslib

import rospy
import tf
import tf.msg
import geometry_msgs.msg
from gazebo_msgs.srv import GetModelState


class DynamicTFBroadcaster:

    def __init__(self):
        self.pub_tf = rospy.Publisher("/tf", tf.msg.tfMessage, queue_size=5)

        change = 0.0
        while not rospy.is_shutdown():
            # Run this loop at about 10Hz
            rospy.sleep(0.1)

            pose = get_object_relative_pose("justina_gripper","justina::camera_link").pose
            t = geometry_msgs.msg.TransformStamped()
            t.header.frame_id = "camera_link"
            t.header.stamp = rospy.Time.now()
            t.child_frame_id = "gm/gr_left_arm_link7"
            t.transform.translation.x = pose.position.x
            t.transform.translation.y = pose.position.y
            t.transform.translation.z = pose.position.z
            t.transform.rotation.x = pose.orientation.x
            t.transform.rotation.y = pose.orientation.y
            t.transform.rotation.z = pose.orientation.z
            t.transform.rotation.w = pose.orientation.w

            tfm = tf.msg.tfMessage([t])
            self.pub_tf.publish(tfm)


if __name__ == '__main__':
    global get_object_relative_pose
    rospy.init_node('gripper_tf_broadcaster')
    get_object_relative_pose = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    tfb = DynamicTFBroadcaster()

    rospy.spin()