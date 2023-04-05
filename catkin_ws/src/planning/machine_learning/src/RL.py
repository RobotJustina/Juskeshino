#! /usr/bin/env python3
#
### mover izquiera o derecha simple_move/goal_dist_lateral
### mover al frente simple_move/goal_dist
import rospy
import numpy as np
import ros_numpy
import math
import glob
import os
from pathlib import Path
from std_msgs.msg import Float32
from std_msgs.msg import Empty
from actionlib_msgs.msg import GoalStatus
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Float32MultiArray

def Last_pub():
    pub_stop.publish(Empty())
    print("Se cierra el nodo")
    return

def callback_goal(msg):
    global next
    if(msg.status==3):
        next=True
    return

def callback_votes(msg):
    global edo
    R1=msg.data[0]
    R2=msg.data[2]
    R3=msg.data[3]
    R4=msg.data[4]
    R5=msg.data[5]
    edo=int(16*R1+8*R2+4*R3+2*R4+R5)
    return

def do_action(act):
    goal_lateral=Float32()
    goal=Float32()
    goal_ang=Float32MultiArray()
    if (act==0): #ir hacia el frente
         goal.data=0.1
         pub_fro.publish(goal)
    elif(act==1): #ir a la izquierda
         goal_lateral.data=0.1
         pub_lat.publish(goal_lateral)
    elif(act==2): #ir  a la derecha
         goal_lateral=-0.1
         pub_lat.publish(goal_lateral)
    elif(act==3): ###giro a la izquierda
         goal_ang.data=[0.0, 0.5]
         pub_ang.publish(goal_ang)
    else: ##giro a la derecha
         goal_ang.data=[0.0, -0.5]
         pub_ang.publish(goal_ang)
    return

rospy.init_node("RL")
#rospy.Subscriber("/hardware/scan", LaserScan, callback_laser_scan)
rospy.Subscriber("/votes", Int32MultiArray , callback_votes)
rospy.Subscriber("/ready", GoalStatus, callback_goal)
pub_lat = rospy.Publisher("/simple_move/goal_dist_lateral", Float32, queue_size=10)
pub_fro = rospy.Publisher("/simple_move/goal_dist", Float32, queue_size=10)
pub_stop = rospy.Publisher("/simple_move/stop", Empty, queue_size=10)
pub_ang =rospy.Publisher("/simple_move/goal_dist_angle", Float32MultiArray, queue_size=10)
loop = rospy.Rate(0.5)
rospy.on_shutdown(Last_pub)

def R_values():
    R=np.zeros((32,5))
    R[0,:]=[10,-1,-1,-10,-10] #0
    R[1,:]=[10,-1,-10,-10,-10] #1
    R[2,:]=[-10,10,10,10,10] #2
    R[3,:]=[-10,10,-10,10,-10] #3
    R[4,:]=[10,-10,-1,-10,-10] #4
    R[5,:]=[10,-10,-10,-10,-10] #5
    R[6,:]=[-10,-10,10,-10,-10] #6
    R[7,:]=[-10,10,10,10,10] #7
    R[8,:]=[10,-1,-10,-10,-10] #8
    R[9,:]=[10,-1,-10,-10,-10] #9
    R[10,:]=[-10,10,-10,-10,-10] #10
    R[11,:]=[-10,10,-10, 10,-10] #11
    R[12,:]=[10,-10,-10,-10,-10] #12
    R[13,:]=[10,-10,-10,-10,-10] #13
    R[14,:]=[-10,10,-10,-10, 10] #14

    R[15,:]=[-10,10,-10,10,-10] #15

    R[16,:]=[10,-10,-1,-10,-10] #16

    R[17,:]=[10,-10,-10,-10,-10] #17

    R[18,:]=[-10,-10,10,10,-10] #18
    R[19,:]=[-10,-10,10,-10,-10] #19
    R[20,:]=[10,-10,-1,-10,-10] #20
    R[21,:]=[10,-10,-10,-10,-10] #21
    R[22,:]=[-10,-10,10,-10,10] #22
    R[23,:]=[-10,-10,10,-10,10] #23
    R[24,:]=[10,-10,-10,-10,-10] #24
    R[25,:]=[10,-10,-10,-10,-10] #25
    R[26,:]=[-10,-10,-10,10,10] #26
    R[27,:]=[-10,-10,-10,10,-10] #27
    R[28,:]=[10,-10,-10,-10,-10] #28
    R[29,:]=[10,-10,-10,-10,-10] #29
    R[20,:]=[-10,-10,-10,-10,10] #30
    R[31,:]=[-10,-10,-10,-10,10] #31
    return R

def choose_action(state):
    global epsilon
    global Q
    action=0
    if np.random.uniform(0, 1)<epsilon or(Q[state,0]==Q[state,1] and Q[state,1]==Q[state,2] and Q[state,2]==Q[state,3] and Q[state,3]==Q[state,4]) :
        action=np.random.randint(0,5)
        #print("random action")
    else:
        action = np.argmax(Q[state, :])
    return action

def main():
    global edo
    global epsilon
    global Q
    global next
    global obs_aux
    obs_aux=False
    edo=0
    next=False
    #Q=Q_values()
    R=R_values()
    epsilon = 0.25
    total_episodes = 10000
    max_steps = 50
    alpha = 0.85
    gamma = 0.95
    steps=0

    path=str(Path(__file__).resolve().parent)+"/*.npz"
    #print(Path(__file__).resolve())  # /home/skovorodkin/stack/scripts/1.py

    outfile=str(Path(__file__).resolve().parent)+"/Entrenamiento.npz"
    file_list=glob.glob(path)
    #file_list=glob.glob('./*.npz')
    if len(file_list)>=1:
        print("Se encontraron datos de entrenamiento")
        npzfile = np.load(outfile)
        first_episode=npzfile['x']
        Q=npzfile['Q']
        print("Episodio "+str(first_episode))
        first_episode+=1
        print(Q)
    else:
        print("No hay datos de entrenamiento")
        Q = np.zeros((32,5))
        first_episode=0
    loop.sleep()
    loop.sleep()

    for x in range(first_episode, first_episode+total_episodes):
        if(rospy.is_shutdown()):
            break
        G=0 ##Ganancia acumulada
        act_ant=choose_action(edo)
        edo_ant=edo
        next=False
        steps=0 ##Conteo de pasos
        while steps<max_steps and not(rospy.is_shutdown()):
            #Se realiza la accion anterior y se escoge una nueva dependiendo del estado
            do_action(act_ant)
            while(not(next) and not(rospy.is_shutdown())):
                pass
            act=choose_action(edo)
            next=False
            Q[edo_ant,act_ant]=Q[edo_ant,act_ant]+alpha*(R[edo_ant,act_ant]+gamma*Q[edo,act]-Q[edo_ant,act_ant])
            G=G+R[edo_ant,act_ant]
            edo_ant, act_ant = edo, act
            steps=steps+1
        if not(rospy.is_shutdown()):
            np.savez(outfile, x=x, Q=Q)
            print("Episodio "+str(x)+" Ganancia total "+str(G))
            print(Q)

if __name__ == '__main__':
    try:
        while not rospy.is_shutdown():
            main()
    except rospy.ROSInterruptException:
        pass
