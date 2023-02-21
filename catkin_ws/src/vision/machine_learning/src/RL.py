#! /usr/bin/env python3
#
### mover izquiera o derecha simple_move/goal_dist_lateral
### mover al frente simple_move/goal_dist
import rospy
import numpy as np
from sensor_msgs.msg   import LaserScan
from std_msgs.msg import Float32
from std_msgs.msg import Empty
from actionlib_msgs.msg import GoalStatus

def Last_pub():
    pub_stop.publish(Empty())
    print("Se cierra el nodo")
    return

def callback_goal(msg):
    print("Se llego a la meta")
    global next
    if(msg.status==3):
        next=True
    return

def callback_laser_scan(msg):
    global edo
    edo=0
    obstacle_left=False
    obstacle_right=False
    obstacle=False
    for i in range(154,184):
        temp=msg.ranges[i]<1.0
        obstacle_left=obstacle_left or temp
    for i in range(0,30):
       temp=msg.ranges[i]<1.0
       obstacle_right=obstacle_right or temp
    for i in range(77,107):
       temp=msg.ranges[i]<1.0
       obstacle=obstacle or temp
    if not(obstacle)  and not(obstacle_left) and not(obstacle_right):
        edo=0
    elif obstacle  and not(obstacle_left) and not(obstacle_right):
        edo=1
    elif not(obstacle)  and obstacle_left and not(obstacle_right):
        edo=2
    elif not(obstacle)  and not(obstacle_left) and obstacle_right:
        edo=3
    elif obstacle  and obstacle_left and not(obstacle_right):
        edo=4
    elif obstacle  and not(obstacle_left) and obstacle_right:
        edo=5
    elif not(obstacle)  and obstacle_left and obstacle_right:
        edo=6
    else:
        edo=7
    return

def do_action(act):
    goal_lateral=Float32()
    goal=Float32()
    if (act==0): #ir hacia el frente
         goal.data=0.2
         goal_lateral=0.0
         pub_lat.publish(goal_lateral)
         pub_fro.publish(goal)
    elif(act==1): #ir a la izquierda
         goal_lateral.data=0.2
         goal.data=0.0
         pub_fro.publish(goal)
         pub_lat.publish(goal_lateral)
    elif(act==2): #ir  a la derecha
         goal.data=0.0
         goal_lateral=-0.2
         pub_fro.publish(goal)
         pub_lat.publish(goal_lateral)
    else: ###ir hacia atrás
         goal.data=-0.2
         goal_lateral=0.0
         pub_lat.publish(goal_lateral)
         pub_fro.publish(goal)
    return

rospy.init_node("RL")
rospy.Subscriber("/hardware/scan", LaserScan, callback_laser_scan)
rospy.Subscriber("/simple_move/goal_reached", GoalStatus, callback_goal)
pub_lat = rospy.Publisher("/simple_move/goal_dist_lateral", Float32, queue_size=10)
pub_fro = rospy.Publisher("/simple_move/goal_dist", Float32, queue_size=10)
pub_stop = rospy.Publisher("/simple_move/stop", Empty, queue_size=10)
#loop = rospy.Rate(1)
rospy.on_shutdown(Last_pub)

def Q_values():
    Q=np.zeros((8,4))
    Q[0,0]=9.83783079
    Q[0,1]=-0.52186371
    Q[0,2]=-6.86721928
    Q[0,3]=-15.20510166

    Q[1,0]=13.54966589
    Q[1,1]=45.27875365
    Q[1,2]=45.58711282
    Q[1,3]=-8.99421424

    Q[2,0]=28.00631636
    Q[2,1]=-57.85837207
    Q[2,2]=-1.63933125
    Q[2,3]=-0.85

    Q[3,0]=11.11689248
    Q[3,1]=10.87116953
    Q[3,2]=-5.96812786
    Q[3,3]=-0.9570219

    Q[4,0]=-13.57794324
    Q[4,1]=-38.29580639
    Q[4,2]=-32.32615104
    Q[4,3]=-33.31341905

    Q[5,0]=-5.2259098
    Q[5,1]=42.6387361
    Q[5,2]=23.00825518
    Q[5,3]=15.42163419
    return Q

def R_values():
    R=np.zeros((8,4))
    R[0,0]=10
    R[0,1]=-1
    R[0,2]=-1
    R[0,3]=-10

    R[1,0]=-10
    R[1,1]=10
    R[1,2]=10
    R[1,3]=-1

    R[2,0]=10
    R[2,1]=-10
    R[2,2]=-1
    R[2,3]=-1

    R[3,0]=10
    R[3,1]=-1
    R[3,2]=-10
    R[3,3]=-1

    R[4,0]=-10
    R[4,1]=-10
    R[4,2]=10
    R[4,3]=-1

    R[5,0]=-10
    R[5,1]=10
    R[5,2]=-10
    R[5,3]=-1

    R[6,0]=10
    R[6,1]=-10
    R[6,2]=-10
    R[6,3]=-1

    R[7,0]=-10
    R[7,1]=-10
    R[7,2]=-10
    R[7,3]=10
    return R

def choose_action(state):
    global epsilon
    global Q
    action=0
    if np.random.uniform(0, 1)< epsilon or(Q[state,0]==Q[state,1] and Q[state,1]==Q[state,2] and Q[state,2]==Q[state,3]):
        action=np.random.randint(0,4)
    else:
        action = np.argmax(Q[state, :])
    return action

def main():
    global edo
    global epsilon
    global Q
    global next
    edo=0
    next=False
    Q = np.zeros((8,4))
    #Q=Q_values()
    R=R_values()
    epsilon = 0.9
    total_episodes = 10000
    max_steps = 100
    alpha = 0.85
    gamma = 0.95
    #do_action(act)
    for x in range(total_episodes):
        ##Inicializar
        G=0
        act_ant=choose_action(edo)
        edo_ant=edo
        steps=0
        while steps<max_steps:
            #Se realiza la accion anterior y se escoge una nueva dependiendo del estado
            do_action(act_ant)
            next=False
            #do_action(0)
            act=choose_action(edo)
            Q[edo_ant,act_ant]=Q[edo_ant,act_ant]+alpha*(R[edo_ant,act_ant]+gamma*Q[edo,act]-Q[edo_ant,act_ant])
            G=G+R[edo_ant,act_ant]
            edo_ant, act_ant = edo, act
            steps=steps+1
            #loop.sleep()
            while(not(next)):
                print(steps)
        print("Episodio "+str(x)+" Ganancia total "+str(G))
        print(Q)

if __name__ == '__main__':
    try:
        while not rospy.is_shutdown():
            main()
    except rospy.ROSInterruptException:
        pass
