#! /usr/bin/env python3

import rospy
import numpy as np
import rospkg
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PointStamped

#list with each concatenated data => [grid+goal+cmd]
info=[]
##temporary variables for saving data
last_goal=[0,0]
grid=[]
rospack = rospkg.RosPack()
dataset_folder = rospack.get_path("machine_learning")

def callback_grid(msg):
	global grid
	grid=list(msg.data)
	#print(f"current info cmd:{last_cmd}, goal:{last_goal} ")

def callback_goal(msg):
	global last_goal
	last_goal=list(msg.data)

def callback_point(msg):
	print("New goal")

def callback_cmd(msg):
	global last_goal, grid, info
	cmd=[msg.linear.x, msg.angular.z]
	temp=grid+cmd+last_goal
    info.append(temp)

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
		while not rospy.is_shutdown():
			main()
		print("Saving data")
		#print(data)
		data=np.asarray(info)
		print(data)
		np.savez(dataset_folder + "/src/data_train",data=data)
	except rospy.ROSInterruptException:
		pass
