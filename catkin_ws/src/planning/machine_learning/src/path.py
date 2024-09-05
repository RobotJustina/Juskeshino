#! /usr/bin/env python3
import rospy
import ros_numpy
import tf
from nav_msgs.msg import Path
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose

def get_pose(listener):
	pose_stamped=PoseStamped()
	try:
		([x, y, z], rot) = listener.lookupTransform('odom', 'base_link', rospy.Time(0))
		pose_stamped.pose.position.x=x
		pose_stamped.pose.position.y=y
	except:
		print("No se tiene información")
	return pose_stamped


def main():
	global listener
	rospy.init_node("path")
	pub_path = rospy.Publisher("/Path_robot", Path, queue_size=10)
	loop = rospy.Rate(5)
	listener = tf.TransformListener()
	path=Path()
	path.header = Header(frame_id='odom')
	for i in range(5):
		loop.sleep()
	while(not rospy.is_shutdown()):
		pose_stamped=get_pose(listener)
		path.poses.append(pose_stamped)
		pub_path.publish(path)
		loop.sleep()

if __name__ == '__main__':
	try:
		while not rospy.is_shutdown():
			main()
	except rospy.ROSInterruptException:
		pass

