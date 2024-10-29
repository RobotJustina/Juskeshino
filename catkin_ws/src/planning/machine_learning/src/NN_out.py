#! /usr/bin/env python3
import rospy
import numpy as np
import torch as th
from torch import nn
import rospkg
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PointStamped
from Redes import architecture

##temporary variables for saving data
last_goal=[0,0]
rospack = rospkg.RosPack()

##Get NN Model
model_folder = rospack.get_path("machine_learning")
# #mired = architecture.Red_lin()
# mired = architecture.Reg()
# mired.load_state_dict(th.load(model_folder+"/src/Data_justina/modelo.pth"))

mired = architecture.CNN_B()
mired.load_state_dict(th.load(model_folder+"/src/Data_gazebo/CNN_B.pth"))

disp = 'cuda' if th.cuda.is_available() else 'cpu'
mired.to(disp)
##Get cmd_vel command from LBG algorithm
C=np.genfromtxt(model_folder+"/src/Data_justina/Centroid.csv", delimiter=",", dtype=float)

linx=0.0
angz=0.0

def callback_grid_reg(msg):
	global mired, last_goal, disp, linx, angz, C
	grid=list(msg.data)
	entrada=grid+last_goal
	entrada = np.asarray(entrada)
	entrada = np.expand_dims(entrada, axis=0)
	x_ent=th.tensor(entrada)
	x_ent=x_ent.to(th.device(disp), th.float32)
	if(abs(last_goal[0])>0.2):
		with th.no_grad():
			y_pred = mired(x_ent)
		y_pred =y_pred.cpu().numpy()
		linx=y_pred[0,0]
		angz=y_pred[0,1]
	else:
		linx=0.0
		angz=0.0

def callback_grid(msg):
	global mired, last_goal, disp, linx, angz, C
	grid=list(msg.data)
	entrada=grid+last_goal
	entrada = np.asarray(entrada)
	entrada = np.expand_dims(entrada, axis=0)
	x_ent=th.tensor(entrada)
	x_ent=x_ent.to(th.device(disp), th.float32)
	if(abs(last_goal[0])>0.2):
		with th.no_grad():
			y_pred = mired(x_ent)
		y_pred =y_pred.cpu().numpy()
		index=int(np.argmax(y_pred))
		linx=C[index,0]
		angz=C[index,1]
	else:
		linx=0.0
		angz=0.0

def callback_goal(msg):
	global last_goal
	last_goal=list(msg.data)

def callback_point(msg):
	print("New goal")

def main():
	global linx, angz
	rospy.init_node("NN_out")
	rospy.Subscriber("/local_occ_grid_array", Float32MultiArray, callback_grid_reg)
	rospy.Subscriber("/NN_goal", Float32MultiArray, callback_goal)
	rospy.Subscriber("/clicked_point", PointStamped, callback_point)
	#pub_cmd = rospy.Publisher("/hardware/mobile_base/cmd_vel", Twist  , queue_size=10)
	pub_cmd = rospy.Publisher("/cmd_vel", Twist  , queue_size=10)
	print("NN_out has been started")
	loop = rospy.Rate(20)
	msg=Twist()
	while not rospy.is_shutdown():
		pub_cmd.publish(msg)
		msg.linear.x=linx
		msg.angular.z=angz
		pub_cmd.publish(msg)
		loop.sleep

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
