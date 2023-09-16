#! /usr/bin/env python3
import rospy
import numpy as np
import torch as th
from torch import nn
import rospkg
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PointStamped

class Red3(nn.Module):
	def __init__(self, capa_1, capa_2, capa_3, capa_4, capa_5):
		super(Red3, self).__init__()
		self.c1 = nn.Linear(6402, capa_1) #Capa densa
		self.b1 = nn.LayerNorm(capa_1) #Capa de normalización
		self.d1 = nn.Dropout(p=0.5) #Capa de normalización
		self.c2 = nn.Linear(capa_1, capa_2) #Capa densa
		self.b2 = nn.LayerNorm(capa_2) #Capa de normalización
		self.d2 = nn.Dropout(p=0.5) #Capa de normalización
		self.c3 = nn.Linear(capa_2, capa_3) #Capa densa
		self.b3 = nn.LayerNorm(capa_3) #Capa de normalización
		self.d3 = nn.Dropout(p=0.5) #Capa de normalización
		self.c4 = nn.Linear(capa_3, capa_4) #Capa densa
		self.b4 = nn.LayerNorm(capa_4) #Capa de normalización
		self.d4 = nn.Dropout(p=0.5) #Capa de normalización
		self.c5 = nn.Linear(capa_4, capa_5) #Capa densa
		self.b5 = nn.LayerNorm(capa_5) #Capa de normalización
		self.d5 = nn.Dropout(p=0.5) #Capa de normalización
		self.salida = nn.Linear(capa_5, 16)

	def forward(self, x):
		x = nn.functional.relu(self.c1(x)) #Activaciones tanh
		x = self.b1(x)
		x = self.d1(x)
		x = nn.functional.relu(self.c2(x))
		x = self.b2(x)
		x = self.d2(x)
		x = nn.functional.relu(self.c3(x))
		x = self.b3(x)
		x = self.d3(x)
		x = nn.functional.relu(self.c4(x))
		x = self.b4(x)
		x = self.d4(x)
		x = nn.functional.relu(self.c5(x))
		x = self.b5(x)
		x = self.d5(x)
		x = self.salida(x)
		return nn.functional.softmax(x,dim=1)

##temporary variables for saving data
last_goal=[0,0]
rospack = rospkg.RosPack()

##Get NN Model
model_folder = rospack.get_path("machine_learning")
mired = Red3(300, 300, 200, 200, 100)
mired.load_state_dict(th.load(model_folder+"/src/modelo.pth"))
disp = 'cuda' if th.cuda.is_available() else 'cpu'
mired.to(disp)
##Get cmd_vel command from LBG algorithm
C=np.genfromtxt(model_folder+"/src/Centroid.csv", delimiter=",", dtype=float)

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
	rospy.Subscriber("/local_occ_grid_array", Float32MultiArray, callback_grid)
	rospy.Subscriber("/NN_goal", Float32MultiArray, callback_goal)
	rospy.Subscriber("/clicked_point", PointStamped, callback_point)
	#pub_cmd = rospy.Publisher("/hardware/mobile_base/cmd_vel", Twist  , queue_size=10)
	pub_cmd = rospy.Publisher("/cmd_vel", Twist  , queue_size=10)
	print("NN_out has been started")
	loop = rospy.Rate(20)
	msg=Twist()
	while not rospy.is_shutdown():
		pub_cmd.publish(msg)
		msg.linear.x=linx/2
		msg.angular.z=angz
		pub_cmd.publish(msg)
		loop.sleep

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
