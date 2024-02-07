#! /usr/bin/env python3
import rospy
import numpy as np
import rospkg
import torch as th
import random
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
from collections import deque
from Redes import architecture
import torch.optim as optim
import torch.nn as nn
##r_laser is the distance from robot to goal
##r_obstacle is the punishment if the robot crash with an obstacle
##r_goal is the reward if the robot get the goal
rospack = rospkg.RosPack()
model_folder = rospack.get_path("machine_learning")
r_laser, r_obstacle, r_goal=0.0,0.0,0.0
deque_len=10000
replay_buffer = deque(maxlen=deque_len)
dist=0.0
linx=0.0
angz=0.0
last_goal=[0,0]
first=True
#mired = architecture.Red_conv(3)
#mired.load_state_dict(th.load(model_folder+"/src/Data_gazebo/modelo_gazebo.pth"))
policy_net = architecture.DQN_1(3)
target_net = architecture.DQN_1(3)
disp = 'cuda' if th.cuda.is_available() else 'cpu'
policy_net.to(disp)
target_net.to(disp)
action_bef=0
grid_bef, grid_act=0,0
C=np.asarray([[0.3, 0.0], [0.0, 0.5],[0.0, -0.5]])
steps=0
LR = policy_net.lr
optimizer = optim.AdamW(policy_net.parameters(), lr=LR, amsgrad=True)
r_total=0
done=True

def callback_scan(msg):
	global r_obstacle
	d_min=min(msg.ranges)
	if(d_min < 0.21):
		r_obstacle=-200-(0.21-d_min)*50
	else:
		r_obstacle=0

def callback_goal(msg):
	global r_laser, r_goal, last_goal,done
	dist=msg.data[0]
	if(dist<0.2):
		r_goal=200
		done=True
	else:
		r_goal=0
		done=False
	r_laser=-dist
	last_goal=list(msg.data)

def train(replay_buffer):
	global policy_net,target_net, disp, optimizer
	gamma=0.9
	batch_size=128
	batch = random.sample(replay_buffer, batch_size)
	buffer_content = list(batch)
	state = th.Tensor(np.array([transition[0] for transition in buffer_content]))
	reward= th.Tensor(np.array([transition[1] for transition in buffer_content]))
	actions = th.Tensor(np.array([transition[2] for transition in buffer_content]))
	next_state = th.Tensor(np.array([transition[3] for transition in buffer_content]))
	actions = actions.to(th.device(disp))
	actions = actions.long()
	actions = actions.unsqueeze(1)
	state=state.to(th.device(disp), th.float32)
	state_action_values = policy_net(state).gather(1, actions)
	next_state=next_state.to(th.device(disp), th.float32)
	target_net.to(disp)
	reward=reward.to(disp)
	gamma = th.tensor(gamma)
	gamma = gamma.to(disp)
	with th.no_grad():
		next_state_values = target_net(next_state).max(1).values
	expected_state_action_values = (next_state_values * gamma) + reward
	criterion = nn.SmoothL1Loss()
	loss = criterion(state_action_values, expected_state_action_values.unsqueeze(1))
	# Optimize the model
	optimizer.zero_grad()
	loss.backward()
	# In-place gradient clipping
	th.nn.utils.clip_grad_value_(policy_net.parameters(), 100)
	optimizer.step()

def callback_grid(msg):
	global grid_bef,grid_act,last_goal, first, linx,angz, r_obstacle, r_laser,r_goal, replay_buffer, policy_net, target_net, steps, done, r_total
	epsilon=0.8
	grid_bef=grid_act
	grid_act=list(msg.data)+last_goal
	random_number = np.random.rand()
	if not done:
		if(random_number<=epsilon):
			entrada = np.asarray(grid_act)
			entrada = np.expand_dims(entrada, axis=0)
			x_ent=th.tensor(entrada)
			x_ent=x_ent.to(th.device(disp), th.float32)
			#print(x_ent.shape)
			with th.no_grad():
				y_pred = policy_net(x_ent)
			y_pred =y_pred.cpu().numpy()
			#print(y_pred, steps, len(replay_buffer))
			index=int(np.argmax(y_pred))
		else:
			index_values = [0, 1, 2]
			index=np.random.choice(index_values)
		linx=C[index,0]
		angz=C[index,1]
		#print(linx,angz)
		r=r_obstacle+r_laser+r_goal
		if(first==False):
			replay_buffer.append((np.asarray(grid_bef),r, index, np.asarray(grid_act)))
		first=False
		if(len(replay_buffer)>1000):
			steps+=1
			r_total+=r
			train(replay_buffer)
			##Los pesos de policy_net pasan a target_net después de 100 pasos
			if(steps%50==0):
				print(r_total)
				r_total=0
				target_net.load_state_dict(policy_net.state_dict())
		th.cuda.empty_cache()
	else:
		linx=0.0
		angz=0.0

def saving_function():
    print("Data is saving...")

def main():
	global r_laser, r_obstacle, r_goal, grid_act, linx, angz, replay_buffer
	rospy.init_node("NN_out")
	rospy.on_shutdown(saving_function)
	rospy.Subscriber("/laser_mod", LaserScan, callback_scan)
	rospy.Subscriber("/NN_goal", Float32MultiArray, callback_goal)
	rospy.Subscriber("/local_occ_grid_array", Float32MultiArray, callback_grid)
	pub_cmd = rospy.Publisher("/cmd_vel", Twist  , queue_size=10)
	print("REWARDS")
	loop = rospy.Rate(10)
	msg = Twist()
	while not rospy.is_shutdown():
		msg.linear.x=linx
		msg.angular.z=angz
		pub_cmd.publish(msg)
		loop.sleep()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass

