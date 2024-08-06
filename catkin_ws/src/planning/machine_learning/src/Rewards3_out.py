#! /usr/bin/env python3
import rospy
import numpy as np
import rospkg
import torch as th
import random
import tf
import time
import math
import os
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
from collections import deque
from Redes import architecture
import torch.optim as optim
import torch.nn as nn
from geometry_msgs.msg import PointStamped
##r_laser is the distance from robot to goal
##r_obstacle is the punishment if the robot crash with an obstacle
##r_goal is the reward if the robot get the goal
rospack = rospkg.RosPack()
data_folder = rospack.get_path("machine_learning") + "/src/DRL_files"
r_laser, r_obstacle, r_goal=0.0,0.0,0.0
deque_len=5000
replay_buffer = deque(maxlen=deque_len)
last_goal=[0.0,0.0]
steps=0
grid,grid_bef, grid_act, action_bef=None,None,None,None
C=np.asarray([[0.3, 0.0], [0.0, 0.5],[0.0, -0.5]])
r_total=0
done=False
stop=False
last_distance=None
init_time=-1.0

th.manual_seed(42)
th.cuda.manual_seed(42)
th.backends.cudnn.deterministic = True

policy_net = architecture.DQN_5(3)
target_net = architecture.DQN_5(3)
target_net.eval()
if os.path.exists(data_folder+"/DRL_gazebo_policy.pth"):
	policy_net.load_state_dict(th.load(data_folder+"/DRL_gazebo_policy.pth",map_location=th.device('cpu')))
	target_net.load_state_dict(th.load(data_folder+"/DRL_gazebo_target.pth",map_location=th.device('cpu')))
else:
	target_net_state_dict = target_net.state_dict()
	policy_net_state_dict = policy_net.state_dict()
	for key in policy_net_state_dict:
		target_net_state_dict[key]=policy_net_state_dict[key]
		target_net.load_state_dict(target_net_state_dict)

if os.path.exists(data_folder+"/replay_buffer.npy"):
	array_load = np.load(data_folder + "/replay_buffer.npy", allow_pickle=True)
	replay_buffer=deque(map(list, array_load), maxlen=deque_len)
	print(len(replay_buffer))

if os.path.exists( data_folder+'/pasos.npy'):
	total_steps = np.load(data_folder+'/pasos.npy')
else:
	total_steps=0
print(total_steps)

if os.path.exists( data_folder+'/r_episode.npy'):
	r_list= list(np.load(data_folder+'/r_episode.npy'))
else:
	r_list=[]
print(r_list)

def callback_point(msg):
	global init_time
	init_time=rospy.get_time()
	#print(f"Init time: {init_time}")
	print(f"New goal: {msg.point.x, msg.point.y}") 

def callback_scan(msg):
	global r_obstacle, stop, steps
	d_min=min(msg.ranges)
	if(d_min < 0.18 or stop):
		r_obstacle=-200
		stop=True
		print("obstaculo cerca")
	else:
		stop=False
		r_obstacle=0

def callback_goal(msg):
	global r_goal, last_goal,done
	dist=msg.data[0]
	if(dist<0.22):
		r_goal=200
		done=True
	else:
		r_goal=0
		done=False
	last_goal=list(msg.data)

def train(replay_buffer):
	global policy_net,target_net,total_steps
	LR = 2.5e-4
	optimizer = optim.RMSprop(policy_net.parameters(), lr=LR)
	disp = 'cuda' if th.cuda.is_available() else 'cpu'
	gamma=0.9
	batch_size=128
	batch = random.sample(replay_buffer, batch_size)
	buffer_content = list(batch)

	state = th.Tensor(np.array([transition[0] for transition in buffer_content]))
	reward= th.Tensor(np.array([transition[1] for transition in buffer_content]))
	actions = th.Tensor(np.array([float(transition[2]) for transition in buffer_content]))
	next_state = th.Tensor(np.array([transition[3] for transition in buffer_content if transition[4]]))
	non_final_mask = th.Tensor(np.array([transition[4] for transition in buffer_content]))

	target_net.to(disp)
	non_final_mask=non_final_mask.to(th.device(disp), th.bool)
	next_state=next_state.to(th.device(disp), th.float32)
	next_state_values = th.zeros(batch_size, device=disp)
	with th.no_grad():
		next_state_values[non_final_mask] = target_net(next_state).max(1).values
	next_state=next_state.to('cpu')
	non_final_mask=non_final_mask.to('cpu')
	target_net.to('cpu')

	actions = actions.to(th.device(disp)).long()
	state=state.to(th.device(disp), th.float32)

	policy_net.to(disp)
	state_action_values = policy_net(state).gather(1, actions.unsqueeze(1))
	state=state.to('cpu')
	actions=actions.to('cpu')

	reward=reward.to(disp)
	gamma = th.tensor(gamma).to(disp)

	print(reward.shape)
	expected_state_action_values = (next_state_values * gamma)+reward
	reward=reward.to('cpu')
	gamma=gamma.to('cpu')
	next_state_values=next_state_values.to('cpu')

	#criterion = nn.SmoothL1Loss()
	criterion = nn.MSELoss()
	#print(state_action_values.shape, (expected_state_action_values.unsqueeze(1)).shape)
	loss = criterion(state_action_values, expected_state_action_values.unsqueeze(1))
	print(f'Perdida {loss}')
	# Optimize the model
	optimizer.zero_grad()
	loss.backward()
	# In-place gradient clipping
	th.nn.utils.clip_grad_value_(policy_net.parameters(), 100)
	optimizer.step()
	policy_net.to('cpu')
	TAU = 0.005
	if(total_steps%8==0):
		target_net_state_dict = target_net.state_dict()
		policy_net_state_dict = policy_net.state_dict()
		for key in policy_net_state_dict:
		#target_net_state_dict[key] = policy_net_state_dict[key]
			target_net_state_dict[key]=policy_net_state_dict[key]*TAU + target_net_state_dict[key]*(1-TAU)
		target_net.load_state_dict(target_net_state_dict)
	state_action_values=state_action_values.to('cpu')
	expected_state_action_values=expected_state_action_values.to('cpu')
	th.cuda.empty_cache()

def polar2vec(last_goal):
	p0=(last_goal[0]/10)*99
	p1=((last_goal[1]/(math.pi))+1)/2 * 99
	l0=[0 for i in range(100)]
	if(round(p1)>=99):
		l0[99]=1
	else:
		l0[round(p1)]=1
	l1=[0 for i in range(100)]
	if(round(p0)>=99):
		l1[99]=1
	else:
		l1[round(p0)]=1
	l=l0+l1
	return l

def select_action(policy_net,steps, grid_act):
	disp = 'cuda' if th.cuda.is_available() else 'cpu'
	temp_act=grid_act[:6400]+polar2vec(grid_act[6400:])
	entrada = np.expand_dims(np.asarray(temp_act), axis=0)
	x_ent = th.tensor(entrada).to(th.device(disp), th.float32)
	#policy_net.to(disp)
	policy_net.eval()
	with th.no_grad():
		y_pred = policy_net(x_ent)
	y_pred.numpy()
	index=int(np.argmax(y_pred))
	#print(f'Red {index} con {y_pred}, 0:ir de frente')
	th.cuda.empty_cache()
	return index

def callback_grid(msg):
	global grid, last_goal
	grid=list(msg.data)+last_goal

def saving_function():
	global target_net, policy_net, data_folder, replay_buffer, steps, r_list,total_steps,done
	replay_numpy = list(replay_buffer)
	disp = 'cuda' if th.cuda.is_available() else 'cpu'
	policy_net.to(disp)
	target_net.to(disp)
	np.save(data_folder + "/replay_buffer",replay_numpy)
	th.save(target_net.state_dict(), data_folder+"/DRL_gazebo_target.pth")
	th.save(policy_net.state_dict(), data_folder+"/DRL_gazebo_policy.pth")
	# Guardamos la constante en un archivo npy
	np.save(data_folder+'/pasos.npy', total_steps)
	np.save(data_folder+'/r_episode.npy', r_list)
	pub_cmd = rospy.Publisher("/cmd_vel", Twist  , queue_size=1)
	msg=Twist()
	pub_cmd.publish(msg)

def tiempo_total():
	global init_time
	tiempo=rospy.get_time()
	print(f"inicio: {init_time}, {tiempo}")
	tiempo=tiempo-init_time
	print(f"Time: {tiempo}")

def main():
	global loop, grid, grid_act, grid_bef, policy_net,C, total_steps, action_bef, stop, replay_buffer, done, r_obstacle, r_dist, r_goal
	pub_cmd = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
	rospy.init_node("DRL_train")
	#rospy.on_shutdown(saving_function)
	rospy.on_shutdown(tiempo_total)
	rospy.Subscriber("/laser_mod", LaserScan, callback_scan, queue_size=1)
	rospy.Subscriber("/NN_goal", Float32MultiArray, callback_goal, queue_size=1)
	rospy.Subscriber("/local_occ_grid_array", Float32MultiArray, callback_grid, queue_size=1)
	rospy.Subscriber("/clicked_point", PointStamped, callback_point)
	#pub_cmd = rospy.Publisher("/cmd_vel", Twist  , queue_size=1)
	print("REWARDS")
	loop = rospy.Rate(2)
	while grid is None:
		pass
	while not rospy.is_shutdown() and not done:
		grid_act=grid
		msg = Twist()
		if(grid_act[6400]>10):
			grid_act[6400]=10
		#temp_act=grid_act[:6400]+polar2vec(grid_act[6400:])
		action_bef=select_action(policy_net, total_steps, grid_act)
		msg.linear.x=C[action_bef,0]
		msg.angular.z=C[action_bef,1]
		pub_cmd.publish(msg)
		loop.sleep()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
