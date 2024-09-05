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
from std_msgs.msg import Int32
from std_msgs.msg import Float32MultiArray

last_goal=[0,0]

def callback_goal_point(msg):
    global last_goal
    last_goal=list(msg.data)

def Last_pub():
    pub_stop.publish(Empty())
    print("Se cierra el nodo")
    return

def callback_offset(msg):
    global offset
    global wait
    offset=msg.data
    if offset>=500:
        offset=offset-500
        wait=True
    else:
        wait=False
    return

def callback_goal(msg):
    global next
    if(msg.status==3):
        next=True
    return

def callback_votes(msg):
    global edo
    global offset
    R1=msg.data[0]
    R2=msg.data[2]
    R3=msg.data[3]
    R4=msg.data[4]
    R5=msg.data[5]
    edo=int(16*R1+8*R2+4*R3+2*R4+R5+offset)
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
rospy.Subscriber("/offset", Int32, callback_offset)
rospy.Subscriber("/NN_goal", Float32MultiArray, callback_goal_point)
pub_lat = rospy.Publisher("/simple_move/goal_dist_lateral", Float32, queue_size=10)
pub_fro = rospy.Publisher("/simple_move/goal_dist", Float32, queue_size=10)
pub_stop = rospy.Publisher("/simple_move/stop", Empty, queue_size=10)
pub_ang =rospy.Publisher("/simple_move/goal_dist_angle", Float32MultiArray, queue_size=10)
loop = rospy.Rate(0.5)
rospy.on_shutdown(Last_pub)

def R_values():
    R=np.zeros((128,5)) ##Arriba
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
    ## Izquierda
    R[32,:]=[-10,10,-10,-1,-10] #0
    R[33,:]=[-10,10,-10,-1,-10] #1
    R[34,:]=[-10,10,-10,-1,-10] #2
    R[35,:]=[-10,10,-10,-1,-10] # #3
    R[36,:]=[-10,10,-10,-1,-10] #4
    R[37,:]=[-10,10,-10,-1,-10] #5
    R[38,:]=[-10,10,-10,-1,-10] #6
    R[39,:]=[-10,10,-10,-1,-10] #7
    R[40,:]=[10,-10,-10,-1,-10] #8
    R[41,:]=[10,-10,-10,-1,-10] #9
    R[42,:]=[-10,-10,-1,10,-10] #10
    R[43,:]=[-10,-10,-1,10,10] #11
    R[44,:]=[10,-10,-10,-1,-10] #12
    R[45,:]=[10,-10,-10,-1,-10] #13
    R[46,:]=[-10,-10,-1,10,-1] #14
    R[47,:]=[-10,-10,10,-10,10] #15
    R[48,:]=[-10,10,-10,-1,-10] #16
    R[49,:]=[-10,10,-10,-1,-10] #17
    R[50,:]=[-10,10,-10,-1,-10] #18
    R[51,:]=[-10,10,-10,-1,-10] #19
    R[52,:]=[-10,10,-10,-1,-10] #20
    R[53,:]=[-10,10,-10,-1,-10] #21
    R[54,:]=[-10,10,-10,-1,-10] #22
    R[55,:]=[-10,10,-10,-1,-10] #23
    R[56,:]=[10,-10,-10,-1,-1] #24
    R[57,:]=[10,-10,-10,-1,-1] #25
    R[58,:]=[-10,-10,-10,10,10] #26
    R[59,:]=[-10,-10,-10,10,10] #27
    R[60,:]=[10,-10,-10,-1,-1] #28
    R[61,:]=[10,-10,-10,-1,-1] #29
    R[62,:]=[-10,-10,-10,10,-10] #30
    R[63,:]=[-10,-10,-10,10,-10] #31
    ## Atras
    R[64,:]=[-10,-10,-10,-10,10] #0
    R[65,:]=[-10,-10,-10,-10,10] #1
    R[66,:]=[-10,-10,-10,-10,10] #2
    R[67,:]=[-10,-10,-10,-10,10] # #3
    R[68,:]=[-10,-10,-10,-10,10] #4
    R[69,:]=[-10,-10,-10,-10,10] #5
    R[70,:]=[-10,-10,-10,-10,10] #6
    R[71,:]=[-10,-10,-10,-10,10] #7
    R[72,:]=[-10,-10,-10,-10,10] #8
    R[73,:]=[-10,-10,-10,-10,10] #9
    R[74,:]=[-10,-10,-10,-10,10] #10
    R[75,:]=[-10,-10,-10,-10,10] #11
    R[76,:]=[-10,-10,-10,-10,10] #12
    R[77,:]=[-10,-10,-10,-10,10] #13
    R[78,:]=[-10,-10,-10,-10,10] #14
    R[79,:]=[-10,-10,-10,-10,10] #15
    R[80,:]=[-10,-10,-10,-10,10] #16
    R[81,:]=[-10,-10,-10,-10,10] #17
    R[82,:]=[-10,-10,-10,-10,10] #18
    R[83,:]=[-10,-10,-10,-10,10] #19
    R[84,:]=[-10,-10,-10,-10,10] #20
    R[85,:]=[-10,-10,-10,-10,10] #21
    R[86,:]=[-10,-10,-10,-10,10] #22
    R[87,:]=[-10,-10,-10,-10,10] #23
    R[88,:]=[-10,-10,-10,-10,10] #24
    R[89,:]=[-10,-10,-10,-10,10] #25
    R[90,:]=[-10,-10,-10,-10,10] #26
    R[91,:]=[-10,-10,-10,-10,10] #27
    R[92,:]=[-10,-10,-10,-10,10] #28
    R[93,:]=[-10,-10,-10,-10,10] #29
    R[94,:]=[-10,-10,-10,-10,10] #30
    R[95,:]=[-10,-10,-10,-10,10] #31
    ## Derecha
    R[96,:]=[-10,-10,10,-10,-1] #0
    R[97,:]=[-10,-10,10,-10,-1] #1
    R[98,:]=[-10,-10,10,-10,-1] #2
    R[99,:]=[-10,-10,10,-10,-1] # #3
    R[100,:]=[-10,-10,10,-10,-1] #4
    R[101,:]=[-10,-10,10,-10,-1] #5
    R[102,:]=[-10,-10,10,-10,-1] #6
    R[103,:]=[-10,-10,10,-10,-1] #7
    R[104,:]=[10,-10,-10,-10,-1] #8
    R[105,:]=[10,-10,-10,-10,-1] #9
    R[106,:]=[-10,-1,-10,-10,10] #10
    R[107,:]=[-10,-1,-10,10,10] #11
    R[108,:]=[10,-10,-10,-10,-1] #12
    R[109,:]=[10,-10,-10,-10,-1] #13
    R[110,:]=[-10,-1,-10,-1,-10] #14
    R[111,:]=[-10,10,-10,10,-10] #15
    R[112,:]=[-10,-10,10,-10,-1] #16
    R[113,:]=[-10,-10,10,-10,-1] #17
    R[114,:]=[-10,-10,10,-10,-1] #18
    R[115,:]=[-10,-10,10,-10,-1] #19
    R[116,:]=[-10,-10,10,-10,-1] #20
    R[117,:]=[-10,-10,10,-10,-1] #21
    R[118,:]=[-10,-10,10,-10,-1] #22
    R[119,:]=[-10,-10,10,-10,-1] #23
    R[120,:]=[10,-10,-10,-1,-1] #24
    R[121,:]=[10,-10,-10,-1,-1] #25
    R[122,:]=[-10,-10,-10,10,10] #26
    R[123,:]=[-10,-10,-10,10,10] #27
    R[124,:]=[10,-10,-10,-1,-1] #28
    R[125,:]=[10,-10,-10,-1,-1] #29
    R[126,:]=[-10,-10,-10,-10,10] #30
    R[127,:]=[-10,-10,-10,-10,10] #31

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
    global offset
    global wait
    global last_goal
    wait=True
    offset=0
    edo=0
    next=False
    R=R_values()
    epsilon = 0.25
    total_episodes = 10000
    max_steps = 50
    alpha = 0.4 ##Learning rate
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
        Q = np.zeros((128,5))
        first_episode=0
    loop.sleep()
    loop.sleep()
    while(wait and not(rospy.is_shutdown()) ):
        pass
    print("Siguiente")
    for x in range(first_episode, first_episode+total_episodes):
        if(rospy.is_shutdown()):
            break
        G=0 ##Ganancia acumulada
        act_ant=choose_action(edo)
        edo_ant=edo
        next=False
        steps=0 ##Conteo de pasos
        while steps<max_steps and not(rospy.is_shutdown()) and last_goal[0]>0.22:
            print(last_goal[0])
            #Se realiza la accion anterior y se escoge una nueva dependiendo del estado
            do_action(act_ant)
            while( (wait or not(next)) and not(rospy.is_shutdown())):
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
