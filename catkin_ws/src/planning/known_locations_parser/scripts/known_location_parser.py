#! /usr/bin/env python3
import rospy  
import tf2_ros    
# import numpy as np
from std_msgs.msg import String
# from known_locations_tf_server.srv import *
import rospkg
import yaml

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
def get_location_cb(req):
	succ,loc = match_location(req)
	if succ:
		return [loc[:3]]
	else:
		return loc


if __name__ == '__main__':
	# loc = input()
	# loc = loc.casefold() 
	# print(read_yaml())
	# print(match_location(loc))
	rospy.init_node('known_locations_parser_server')

	s = rospy.Service('get_location',,get_location_cb)