#! /usr/bin/env python3
#
### mover izquiera o derecha simple_move/goal_dist_lateral
### mover al frente simple_move/goal_dist
import rospy
import numpy as np
import ros_numpy
import math
from sensor_msgs.msg   import LaserScan
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32
from std_msgs.msg import Empty
from actionlib_msgs.msg import GoalStatus
from std_msgs.msg import Float64MultiArray

def Last_pub():
    pub_stop.publish(Empty())
    print("Se cierra el nodo")
    return

def callback_goal(msg):
    global next
    if(msg.status==3):
        next=True
    return

def callback_pointcloud(msg):
    arr=ros_numpy.point_cloud2.pointcloud2_to_array(msg)
    m,n=arr.shape
    ang=-0.62
    contador=0
    for i in range(m//4,3*m//4):
        for j in range(n//2,n):
            x,y,z=arr[i,j][0], arr[i,j][1], arr[i,j][2] ##Datos como se entregan por la cámara
            if(math.isnan(x) or math.isnan(y) or math.isnan(z)):
                continue
            x=z*math.cos(ang)+y*math.sin(ang)
            y=-z*math.sin(ang)+y*math.cos(ang)
            z=-y
            x= z
            #print(x,z)
            if(x< 1 and z>-0.7):
                contador+=1
    print(contador)
    return

def callback_laser_scan(msg):
    global edo
    edo=0
    obstacle_left=False
    obstacle_right=False
    obstacle=False
    for i in range(152,184):
        temp=msg.ranges[i]<1.0
        obstacle_left=obstacle_left or temp
    for i in range(0,32):
       temp=msg.ranges[i]<1.0
       obstacle_right=obstacle_right or temp
    for i in range(74,110):
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
         goal.data=0.1
         pub_fro.publish(goal)
    elif(act==1): #ir a la izquierda
         goal_lateral.data=0.1
         pub_lat.publish(goal_lateral)
    elif(act==2): #ir  a la derecha
         goal_lateral=-0.1
         pub_lat.publish(goal_lateral)
    else: ###ir hacia atrás
         goal.data=-0.1
         pub_fro.publish(goal)
    return

rospy.init_node("RL")
rospy.Subscriber("/hardware/scan", LaserScan, callback_laser_scan)
rospy.Subscriber("/hardware/realsense/points",PointCloud2, callback_pointcloud)
rospy.Subscriber("/simple_move/goal_reached", GoalStatus, callback_goal)
pub_lat = rospy.Publisher("/simple_move/goal_dist_lateral", Float32, queue_size=10)
pub_fro = rospy.Publisher("/simple_move/goal_dist", Float32, queue_size=10)
pub_stop = rospy.Publisher("/simple_move/stop", Empty, queue_size=10)
pub_head =rospy.Publisher("/hardware/head/goal_pose", Float64MultiArray, queue_size=10)

loop = rospy.Rate(0.5)
rospy.on_shutdown(Last_pub)

def Q_values():
    Q=np.zeros((8,4))
    Q[0,0]=5.99900391
    Q[0,1]=-1.90608052
    Q[0,2]=1.32473345
    Q[0,3]=-1.82684756

    Q[1,0]=-5.92838938
    Q[1,1]=33.53236352
    Q[1,2]=15.69608459
    Q[1,3]=-3.81752493

    Q[2,0]=0
    Q[2,1]=0
    Q[2,2]=0
    Q[2,3]=0

    Q[3,0]=31.7809589
    Q[3,1]=4.30146037
    Q[3,2]=-5.5936299
    Q[3,3]=1.59059546

    Q[4,0]=0
    Q[4,1]=0
    Q[4,2]=0
    Q[4,3]=0

    Q[5,0]=-9.64333788
    Q[5,1]=8.28557938
    Q[5,2]=0
    Q[5,3]=10.11073587

    Q[7,0]=-22.8296999
    Q[7,1]=-10.69824534
    Q[7,2]=0
    Q[7,3]=0

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
    max_steps = 20
    alpha = 0.85
    gamma = 0.95
    #do_action(act)
    steps=-4
    while steps <0:
        steps=steps+1
        prueba=Float32()
        prueba.data=-0.03
        pub_fro.publish(prueba)
        loop.sleep()

    prueba=Float64MultiArray()
    prueba.data=[0.0, -0.62]
    pub_head.publish(prueba)

    for x in range(total_episodes):
        if(rospy.is_shutdown()):
            break
        G=0
        act_ant=choose_action(edo)
        edo_ant=edo
        next=False
        steps=0
        while steps<max_steps and not(rospy.is_shutdown()):
            #Se realiza la accion anterior y se escoge una nueva dependiendo del estado
            #do_action(act_ant)
            do_action(act_ant)
            while(not(next) and not(rospy.is_shutdown())):
                pass
                #do_action(act_ant)
                #loop.sleep()
            act=choose_action(edo)
            next=False
            Q[edo_ant,act_ant]=Q[edo_ant,act_ant]+alpha*(R[edo_ant,act_ant]+gamma*Q[edo,act]-Q[edo_ant,act_ant])
            G=G+R[edo_ant,act_ant]
            edo_ant, act_ant = edo, act
            steps=steps+1
        print("Episodio "+str(x)+" Ganancia total "+str(G))
        print(Q)

if __name__ == '__main__':
    try:
        while not rospy.is_shutdown():
            main()
    except rospy.ROSInterruptException:
        pass
