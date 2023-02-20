#! /usr/bin/env python
import rospy
import tf2_ros
from std_msgs.msg import String
import rospkg
import yaml
from geometry_msgs.msg import PoseStamped
import tf as tf

def read_yaml(known_locations_file = '/known_locations.yaml'):
	rospack = rospkg.RosPack()
	file_path = rospack.get_path('config_files') + known_locations_file

	with open(file_path, 'r') as file:
		content = yaml.safe_load(file)
	return content
def match_location(location):
	content = read_yaml()
	try:
		return True, content[location]
	except:
		return False, 'No location found'

#Get location service
class location_parser():
	def __init__(self):
		self.goal = PoseStamped()
		self.goal.header.frame_id = 'map'
		self.goal.pose.position.z = 0

		self.positionXYT = [0.0, 0.0, 0.0]
		self._location_sub = rospy.Subscriber(
			'/goal_location', String, self._callback)
		self._goal_pub = rospy.Publisher('/known_location/goal',PoseStamped, queue_size = 10)
	def _callback(self, msg):
		# print(msg)
		goal_msg = msg.data.casefold()
		succ,loc = match_location(goal_msg)
		if succ:
			self.positionXYT = loc[:3]
			self._fill_msg()
		else:
			print(loc)
	def _fill_msg(self):
		self.goal.header.stamp = rospy.Time.now()
		self.goal.pose.position.x = self.positionXYT[0]['x']
		self.goal.pose.position.y = self.positionXYT[1]['y']
		
		quat = tf.transformations.quaternion_from_euler(0, 0, self.positionXYT[2]['theta'])

		self.goal.pose.orientation.x = quat[0]
		self.goal.pose.orientation.y = quat[1]
		self.goal.pose.orientation.z = quat[2]
		self.goal.pose.orientation.w = quat[3]
		self._goal_pub.publish(self.goal)

if __name__ == '__main__':
	rospy.init_node('known_locations_parser')
	parser = location_parser()
	rospy.spin()
