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

def callback_pointcloud(msg):
    global obs_aux
    obs_aux=False
    arr=ros_numpy.point_cloud2.pointcloud2_to_array(msg)
    m,n=arr.shape
    ang=-0.62
    contador=0
    min=10
    for i in range(m//8,7*m//8):
        for j in range(5*n//8,n):
            x,y,z=arr[i,j][0], arr[i,j][1], arr[i,j][2] ##Datos como se entregan por la cámara
            if(math.isnan(x) or math.isnan(y) or math.isnan(z)):
                continue
            z=z*math.cos(ang)+y*math.sin(ang)
            y=-z*math.sin(ang)+y*math.cos(ang)
            #temp=y
            x,y,z=z,-x,-y
            #x=z
            #y=-x
            #z=-temp
            if( (x < 1.2 and x>=0.1) and z>-1.2 and(y<=0.35 and y>=-0.35)):
                contador+=1
    if contador>=1000:
        obs_aux=True
        print("Cuidado al frente")
    return

def get_votes(msg):
    #Array=np.zeros((186,2))
    Array=np.zeros((3,3))
    xmin=-0.799 #distancia minima en x
    ymin=-1.799 #distancia minima en y
    d=1.2 # distancia entre los intervalos de las regiones
    #c=(1.8+d)//1.2
    for i in range(185):
        ang=i*math.pi/185
        y=-msg.ranges[i]*math.cos(ang)
        x=msg.ranges[i]*math.sin(ang)
        r=(x-xmin)//d
        c=(y-ymin)//d
        if r<=2 and r>=0 and c<=2 and c>=0:
            Array[int(r),int(2-c)]+=1
    #print(Array)
    return Array

def threshold_votes(Array, threshold):
    r,c=Array.shape
    for i in range(r):
        for j in range(c):
            if(Array[i,j]<=threshold):
                Array[i,j]=0
            else:
                Array[i,j]=1
    #print("----------")
    #print(Array)
    return Array

def callback_laser_scan(msg):
    global edo
    global obs_aux
    th=1
    A=get_votes(msg) ##Se obtiene la matriz devotos
    A=threshold_votes(A,th) ##Se le aplica un threshold a la matriz de votos
    #print(A)
    R1=A[0,0]
    R2=A[0,2]
    R3=A[1,0]
    R4=A[1,1]
    if R4==0 and A[0,1]==1 or obs_aux:
        R4=int(1)
    R5=A[1,2]
    edo=int(16*R1+8*R2+4*R3+2*R4+R5)
    #print(edo)
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
rospy.Subscriber("/hardware/scan", LaserScan, callback_laser_scan)
rospy.Subscriber("/hardware/realsense/points",PointCloud2, callback_pointcloud)
rospy.Subscriber("/simple_move/goal_reached", GoalStatus, callback_goal)
pub_lat = rospy.Publisher("/simple_move/goal_dist_lateral", Float32, queue_size=10)
pub_fro = rospy.Publisher("/simple_move/goal_dist", Float32, queue_size=10)
pub_stop = rospy.Publisher("/simple_move/stop", Empty, queue_size=10)
pub_head =rospy.Publisher("/hardware/head/goal_pose", Float64MultiArray, queue_size=10)
pub_ang =rospy.Publisher("/simple_move/goal_dist_angle", Float32MultiArray, queue_size=10)

loop = rospy.Rate(0.5)
rospy.on_shutdown(Last_pub)

def Q_values():
    Q=np.zeros((8,4))
    return Q

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
    Q = np.zeros((32,5))
    #Q=Q_values()
    R=R_values()
    epsilon = 0.9
    total_episodes = 10000
    max_steps = 50
    alpha = 0.85
    gamma = 0.95
    steps=-2
    while steps <0:
        steps=steps+1
        prueba=Float32()
        prueba.data=-0.03
        pub_fro.publish(prueba)
        loop.sleep()

    prueba=Float64MultiArray() #La cámara apunta al suelo
    prueba.data=[0.0, -0.62]
    pub_head.publish(prueba)

    for x in range(total_episodes):
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
