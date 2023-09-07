#! /usr/bin/env python3

import rospy
import numpy as np
import cv2
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PointStamped

#list with each concatenated data => [grid+goal+cmd]
info=[]
##temporary variables for saving data
last_cmd=[]
last_goal=[]
save_bool=False

def callback_grid(msg):
	grid=msg.data
	if(save_bool):
		temp=grid+last_cmd+last_goal
		info.append(temp)
		print(f"current info cmd:{last_cmd}, goal:{last_goal} ")

def callback_goal(msg):
	last_goal=msg.data

def callback_point(msg):
	save_bool=True

def callback_cmd(msg)
	last_cmd=[msg.linear.x, msg.angular.z]

def main():
	rospy.init_node("sync_msg")
	rospy.Subscriber("/local_occ_grid_array", Float32MultiArray, callback_grid)
	rospy.Subscriber("/NN_goal", Float32MultiArray, callback_goal)
	rospy.Subscriber("/clicked_point", PointStamped, callback_point)
	rospy.Subscriber("/hardware/mobile_base/cmd_vel", Twist , callback_cmd)
	print("sync_msg has been started")
	rospy.spin()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
