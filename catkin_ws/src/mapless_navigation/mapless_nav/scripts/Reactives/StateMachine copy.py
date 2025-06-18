#! /usr/bin/env python3

import rospy
import smach, smach_ros
from geometry_msgs.msg import Twist, PointStamped
from std_msgs.msg import Float32MultiArray, Float64MultiArray
from nav_msgs.msg import OccupancyGrid
import numpy as np
import matplotlib.pyplot as plt
import sys
from sensor_msgs.msg import LaserScan
import copy
import time

np.set_printoptions(threshold=sys.maxsize)

last_goal = [0, 0] # d, th
base = None 
feat = None
target_reached = True
obst_detected = [False, False, False] # L, R, C
obst_memory = [False, False, False]
speed_factor = 0.8

#_________
class SimpleBase():
    def __init__(self):
        self._base_vel_pub = rospy.Publisher('/hardware/mobile_base/cmd_vel' , Twist, queue_size=10)
        self.twist = Twist()

    def call_base(self):
        print("-BASE-")

    def move_vel(self, l_velX, l_velY, a_velZ):
        self.twist.linear.x = l_velX
        self.twist.linear.y = l_velY
        self.twist.angular.z = a_velZ
        self._base_vel_pub.publish(self.twist)


class Features():
    def __init__(self):
        self.left_obst = []
        self.right_obst = []
        self.cent_obst = []
        self.wall = []
        self.laser_obst_left = []
        self.laser_obst_right = []
        self.laser_obst_cent = []
        self.laser_wall = []

    def set_features(self, left, right, center, wall):
        self.left_obst = left
        self.right_obst = right
        self.cent_obst = center
        self.wall = wall
    
    def get_features(self):
        l = self.left_obst
        r = self.right_obst 
        c = self.cent_obst
        w = self.wall
        return l, r, c, w
    
    def set_laser(self, left, right, center, wall):
        self.laser_obst_left = left
        self.laser_obst_right = right
        self.laser_obst_cent = center
        self.laser_wall = wall
    
    def get_laser(self):
        l = self.laser_obst_left
        r = self.laser_obst_right 
        c = self.laser_obst_cent
        w = self.laser_wall
        return l, r, c, w

#_________

def callback_goal(msg):
    global last_goal, target_reached
    if target_reached:
        last_goal[0] = 0.0
        last_goal[1] = 0.0
    else:
        last_goal = list(msg.data)


def callback_point(msg):
    global target_reached
    clear_console()
    str = f"\nNew goal: ({msg.point.x:.3f}, {msg.point.y:.3f})" + " "*100
    sys.stdout.write(str)
    sys.stdout.flush()
    sys.stdout.write("\n")
    sys.stdout.flush()
    target_reached = False


def callback_laser(msg):
    global feat
    n = len(msg.ranges) # inx 185 (R to L)
    c = n//2 + 1
    #76, c -+ 17
    left = np.clip( msg.ranges[c+25:] , 0, 11)
    right = np.clip( msg.ranges[:c-25] , 0, 11)
    center = np.clip( msg.ranges[c-17:c+17] , 0, 11)
    wall = np.clip( msg.ranges[c-25:c+25] , 0, 11)
    feat.set_laser(left, right, center, wall)


def occGridCallback(msg):
    global feat

    data = np.asarray(msg.data)
    res = round(msg.info.resolution, 2)
    rows = msg.info.height
    data = np.reshape(data, (rows, rows))
    data = np.rot90(np.flip(data, axis=0))
    c = rows// 2
    near_obst_dist = int(1/res)

    data_L = data[c:, :c-6]
    data_L2 = data_L.sum(0)//100
    data_L2 = data_L2[len(data_L2)//2:]
    
    # print(data_L2)
    # print("Shape ",data_L2.shape)
    # show_image(data_L, "img")


    data_R = data[c:, c+6:]
    data_R2 = data_R.sum(0)//100# Y 0-34, X 1-40
    data_R2 = data_R2[:len(data_R2)//2]

    # print(data_R2)
    # print("Shape ",data_R2.shape)
    # show_image(data_R, "img")


    data_C = data[-near_obst_dist:, c-6:c+6]
    data_C2 = data_C.sum(0)//100

    # print(data_C2)
    # print("Shape ",data_C2.shape)
    # show_image(data_C, "img")
    
    #step = int(100*res)
    data_F = data[-near_obst_dist:-near_obst_dist+3, :]
    #print("data_F shape", data_F.shape)
    data_F2 = data_F.sum(0)//100

    # print(data_F2)
    # print("Shape ",data_F2.shape)
    # print("Shape ",data_F2.shape)
    # show_image(data_F, "img")

    
    feat.set_features([data_L, data_L2], [data_R, data_R2], 
                        [data_C, data_C2], [data_F, data_F2])

    # weight = 1
    # for i in range(len(data_R2)):
    #     data_R2[i] = data_R2[i] * weight
    #     weight = weight - 1/(len(data_R2))

#> Funct()
def show_image(img, name):
    plt.title(name)
    plt.imshow(img, cmap='gray') 
    plt.show()


def clear_console():
    blanks = " "*110
    sys.stdout.write("\033[A" + blanks + "\r")
    #print(" "*100, end='\r')
    sys.stdout.flush()


def shutdown_stop():
    global base
    print("Exit ...")
    base.move_vel(0.0, 0.0, 0.0)
    rospy.sleep(0.5)


def turn_direction():
    if last_goal[1] > 0.2:
        return 'L'
    if last_goal[1] < -0.2:
        return 'R'
    else:
        return 'F'

    
def free_side(laser_l, laser_r, obst_l, obst_r):
    print("free_side()")
    global obst_detected
    fl = len(laser_l) * 11
    fr = len(laser_r) * 11

    l_sum = laser_l.sum()
    r_sum = laser_r.sum()
    
    free_l = l_sum == fl or np.all(laser_l > 1.0)
    free_r = r_sum == fr or np.all(laser_r > 1.0)
    dense_l = obst_l.sum()
    dense_r = obst_r.sum()
    print("** Dense L", dense_l)
    print("** Dense R", dense_r)

    density = 40
    print(free_l, free_r)
    if free_l and dense_l < len(laser_l):
        print("-Free Left")
        obst_detected[0] = False
    else:
        obst_detected[0] = True
      
    if free_r and dense_r < len(laser_r):
        print("-Free Right")
        obst_detected[1] = False
    else:
        obst_detected[1] = True

    print()
    if free_l and free_r:
        if l_sum + dense_l >= r_sum + dense_r:
            best = 'R'
        else:
            best = 'L'
    elif free_l and dense_l < len(laser_r):
        best = 'L'
    elif free_r and dense_r < len(laser_r):
        best = 'R'
    else:
        best = 'B'

    return best


def laser_side():
    global feat
    print("laser_side()")
    laser_l, laser_r, laser_c, laser_f = feat.get_laser()
    #print('laser_l', laser_l)
    # print('laser_r', laser_r)
    #print('laser_c', laser_c)
    print('--')
    

    free_l = zero_runs(np.where(laser_l > 1, 0, 1))
    print(np.where(laser_l > 1, 0, 1))
    max_l_range = [0, 0]
    for range in free_l:
        n = range[1] - range[0]
        if n > len(laser_c):
            print("space", range)
            if n > max_l_range[1] - max_l_range[0]:
                max_l_range = range
    
    print("max L", max_l_range)
    l_range = max_l_range[1] - max_l_range[0]
    l_space = l_range > 0
    #if l_space and not obst_detected[0]:
    #    print('left is free :)')

    free_r = zero_runs(np.where(laser_r > 1, 0, 1))
    print(np.where(laser_r > 1, 0, 1))
    max_r_range = [0, 0]
    for range in free_r:
        n = range[1] - range[0]
        if n > len(laser_c):
            print("space", range)
            if n > max_r_range[1] - max_r_range[0]:
                max_r_range = range
    
    print("max R", max_r_range)
    r_range =  max_r_range[1] - max_r_range[0] 
    r_space =  r_range > 0

    if r_range == l_range == 0:
        print("no space")
        return 'B'
    
    if l_range >= r_range:
        return 'L'
    else:
        return 'R'
    

    



def zero_runs(a):
    # Create an array that is 1 where a is 0, and pad each end with an extra 0.
    iszero = np.concatenate(([0], np.equal(a, 0).view(np.int8), [0]))
    absdiff = np.abs(np.diff(iszero))
    # Runs start and end where absdiff is 1.
    ranges = np.where(absdiff == 1)[0].reshape(-1, 2)
    return ranges


def move_robot(vel, timer):
    global base
    t0 = time.time()
    
    while True:
        base.move_vel(vel[0], vel[1], vel[2])
        if time.time() - t0 >= timer:
            break
    base.move_vel(0.0, 0.0, 0.0)

#>>> STATE MACHINE -------------------------------------------------------
class Initial(smach.State):
    def __init__(self):
        smach.State.__init__(self , outcomes=['failed', 'tries', 'succ'])
        self.tries = 0

    def execute(self, userdata):
        global base, feat
        if self.tries == 0:
            rospy.logwarn('--> STATE <: Initial')
            base = SimpleBase()
            rospy.sleep(1.0)
        elif self.tries >0: 
            clear_console()

        self.tries += 1
        if target_reached == True: # No objective
            return 'tries'
        else:
            return 'succ'


class Evaluate(smach.State):
    def __init__(self):
        smach.State.__init__(self , outcomes=['failed', 'tries', 'succ', 'free', 'obstacle'], 
                                output_keys=['command_time', 'evasion'])
        self.tries = 0
        self.rnd_direction = 0

    def execute(self, userdata): 
        global last_goal
        global base, feat
        global obst_detected, obst_memory
        if self.tries == 0:
            rospy.logwarn('--> STATE <: Evaluate')
            rospy.sleep(0.2)
        elif self.tries >0:
            clear_console()
            clear_console()

        self.tries += 1
        
        obst_l, obst_r, obst_c, obst_f = feat.get_features()
        laser_l, laser_r, laser_c, laser_f= feat.get_laser()

        print()
        userdata.command_time = 300
        if obst_c[1].sum() == 0 and last_goal[0] > 0.5: # No obstacle front
            print("free ...")
            self.tries = 0
            
            userdata.evasion = '-'
            return 'free'
        elif obst_c[1].sum() > 0:
            decision = free_side(laser_l, laser_r, obst_l[1], obst_r[1])
            userdata.evasion = decision
            
            obst_detected[2] = obst_c[1].sum() != 0
            obst_memory = copy.deepcopy(obst_detected)
            print("---------------Wall---------------\n")
            print("---------------Wall---------------\n")
            print("decision", decision)
            #base.move_vel(0.0, 0.0, 0.0)
            self.tries = 0
            return 'obstacle' 
        else:
            base.move_vel(0.0, 0.0, 0.0)

        if last_goal[0] < 0.5:
            self.tries = 0
            return 'succ'


        return 'tries'


class EvadeObstacle(smach.State):
    def __init__(self):
        smach.State.__init__(self , outcomes=['failed', 'succ', 'free', 'tries'], output_keys=['command_time', 'evasion'], 
                            input_keys=['evasion'])
        self.rnd_direction = 0

    def execute(self, userdata): 
        global feat
        global obst_detected, obst_memory
        verbose = False
        rospy.logwarn('--> STATE <: EvadeObstacle')
        rospy.sleep(0.2)


        userdata.command_time = 180
        obst_l, obst_r, obst_c, obst_f = feat.get_features()
        laser_l, laser_r, laser_c, laser_f= feat.get_laser()
        obst_detected[2] = obst_c[1].sum() != 0
        free_side(laser_l, laser_r, obst_l[1], obst_r[1])
        if verbose:
            print("obst left", obst_l[1])
            print("obst right", obst_r[1])
            print("obst center", obst_c[1])
            print("obst front", obst_f[1])


        print("??Evasion", userdata.evasion)
        print("Memory", obst_memory)
        print("obst detected", obst_detected)
        print()

        lid_side = laser_side()
        print("laser side", lid_side)
        
        if userdata.evasion == 'B':
            print('evade back _')
            # move_robot([-0.3*speed_factor, 0.0, 0.0], 1.5)
            if lid_side  == 'B':  
            #     return 'succ'
                sign = np.random.randint(-1, 2)
                sign = -1 if sign < 0 else 1
                print('sign', sign)
                speed = sign *0.5
            
            if lid_side == 'L':
                speed = 0.5
            if lid_side == 'R':
                speed = -0.5
            base.move_vel(0.0, 0.0, speed * speed_factor)
            return 'succ'



        if userdata.evasion == 'L':
            print('evade left <')
            speed = 0.5
            # if laser_side == 'L':
            #     speed = 1
            index = 0

        #     move_robot([0.0, 0.0, 0.5*speed_factor], 1.5)
         
        if userdata.evasion == 'R':
            print('evade right >')
            speed = -0.5
            # if laser_side == 'R':
            #     speed = -1
            index = 1
        
        times = 0
        while obst_c[1].sum() > 0:
            times += 1
            obst_l, obst_r, obst_c, obst_f = feat.get_features()
            base.move_vel(0.0, 0.0, speed * speed_factor)
            
        print("FRONT --->")
        move_robot([0.5*speed_factor, 0.0, 0.0], 1.0)
        ##rospy.sleep(0.2)
        obst_l, obst_r, obst_c, obst_f = feat.get_features()
        laser_l, laser_r, laser_c, laser_f= feat.get_laser()            
        free_side(laser_l, laser_r, obst_l[1], obst_r[1])
        print("1111111111111111111111111111111111111111111111111111111111111111111111111111111")
        
        print( obst_c[1])
        print("obstacleeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee")

        if last_goal[0] < 1.2:
            print("O_O so close")
            self.tries = 0
            userdata.evasion = '-'
            return 'free'

        print("obst detected", obst_detected)
        if not obst_detected[index]:
            #if not obst_detected[2]:
            return 'succ'

        move_robot([-0.5*speed_factor, 0.0, 0.0], 0.5)
        return 'tries'





class TurnObjective(smach.State):
    def __init__(self):
        smach.State.__init__(self , outcomes=['failed','L', 'R', 'F'], output_keys=['command_time'])
        self.tries = 0

    def execute(self, userdata): 
        global last_goal
        global feat
        if self.tries == 0:
            rospy.logwarn('--> STATE <: TurnObjective\n')
            rospy.sleep(0.2)
        elif self.tries >0: 
            clear_console()
        self.tries += 1
        userdata.command_time = 400
        dir = turn_direction()

        return dir









class MoveLeft(smach.State):
    def __init__(self):
        smach.State.__init__(self , outcomes=['failed', 'tries', 'succ', 'F'],  output_keys=['command_time'], 
                             input_keys=['command_time', 'evasion'])
        self.tries = 0

    def execute(self, userdata):
        global base, feat
        if self.tries == 0:
            rospy.logwarn('--> STATE <: MoveLeft')
            rospy.sleep(0.2)
        elif self.tries >1: 
            clear_console()

        base.move_vel(0.0, 0.0, 0.5*speed_factor)
        self.tries += 1

        if self.tries > userdata.command_time:#500
            self.tries = 0
            if userdata.evasion != '-':
                userdata.command_time = 400
                return 'F'
            return 'succ'

        return 'tries'
    

class MoveRight(smach.State):
    def __init__(self):
        smach.State.__init__(self , outcomes=['failed', 'tries', 'succ', 'F'], output_keys=['command_time'], 
                             input_keys=['command_time', 'evasion'])
        self.tries = 0

    def execute(self, userdata):
        global base, feat
        if self.tries == 0:
            rospy.logwarn('--> STATE <: MoveRight')
            rospy.sleep(0.2)
        elif self.tries >1: 
            clear_console()

        base.move_vel(0.0, 0.0, -0.5*speed_factor)
        self.tries += 1

        if self.tries > userdata.command_time:#500
            self.tries = 0
            if userdata.evasion != '-':
                userdata.command_time = 400
                return 'F'
            return 'succ'

        return 'tries'


class MoveForward(smach.State):
    def __init__(self):
        smach.State.__init__(self , outcomes=['failed', 'tries', 'succ', 'obstacle'], input_keys=['command_time', 'evasion'], 
                             output_keys=['command_time'])
        self.tries = 0

    def execute(self, userdata):
        global base, feat
        if self.tries == 0:
            rospy.logwarn('--> STATE <: MoveForward')
            rospy.sleep(0.2)
        elif self.tries >1: 
            clear_console()

        base.move_vel(0.8*speed_factor, 0.0, 0.0)
        self.tries = self.tries + 1

        if self.tries > userdata.command_time:#500
            self.tries = 0
            if userdata.evasion != '-':
                userdata.command_time = 300
                return 'obstacle'
            return 'succ'

        return 'tries'


class MoveBackward(smach.State):
    def __init__(self):
        smach.State.__init__(self , outcomes=['failed', 'tries', 'succ', 'obstacle', 'L', 'R'], output_keys=['command_time'],
                                             input_keys=['command_time', 'evasion'])
        self.tries = 0

    def execute(self, userdata):
        global base, feat
        if self.tries == 0:
            rospy.logwarn('--> STATE <: MoveBackward')
            rospy.sleep(0.2)
        elif self.tries >1: 
            clear_console()

        base.move_vel(-0.3*speed_factor, 0.0, 0.0)
        self.tries = self.tries + 1

        if self.tries > userdata.command_time:#500
            self.tries = 0
            if userdata.evasion == 'B':
                return 'obstacle'
            elif userdata.evasion == 'L':
                userdata.command_time = 2000
                return 'L'
            elif userdata.evasion == 'R':
                userdata.command_time = 2000
                return 'R'

            return 'succ'

        return 'tries'



if __name__ == '__main__':
    print("State machine")
    rospy.init_node('smach_react_nav')
    
    rospy.Subscriber("/local_occ_grid", OccupancyGrid, occGridCallback)
    rospy.Subscriber("/hardware/scan", LaserScan, callback_laser)
    rospy.Subscriber("/clicked_point", PointStamped, callback_point)
    rospy.Subscriber("/NN_goal", Float32MultiArray, callback_goal)
    feat = Features()
    pubHeadPos = rospy.Publisher("/hardware/head/goal_pose", Float64MultiArray, queue_size=1)
    msgHeadPos = Float64MultiArray()
    msgHeadPos.data = [0.0, -0.4]
    pubHeadPos.publish(msgHeadPos)
    rospy.sleep(1)
    pubHeadPos.publish(msgHeadPos)


    sm = smach.StateMachine(outcomes=['END'])
    sis = smach_ros.IntrospectionServer('SMACH_VIEW_SERVER', sm, '/SM_REACTIVE_NAV')
    sis.start()

    with sm:
        smach.StateMachine.add("INITIAL", Initial(), transitions={'failed':'INITIAL', 'tries':'INITIAL', 'succ':'EVALUATE'})

        smach.StateMachine.add("EVALUATE", Evaluate(), transitions={'failed':'INITIAL', 'tries':'EVALUATE', 'succ':'END',
                                                                    'free':'TURN_OBJECTIVE', 'obstacle':'EVADE_OBSTACLE'})

        smach.StateMachine.add("EVADE_OBSTACLE", EvadeObstacle(), transitions={'failed':'INITIAL', 'succ':'EVALUATE', 'free':'MOVE_LEFT',
                                                                               'tries':'EVADE_OBSTACLE'})
        
        smach.StateMachine.add("TURN_OBJECTIVE", TurnObjective(), transitions={'failed':'INITIAL', 'L':'MOVE_LEFT',
                                                                               'R':'MOVE_RIGHT', 'F':'MOVE_FORWARD'})

        smach.StateMachine.add("MOVE_LEFT", MoveLeft(), transitions={'failed':'INITIAL', 'tries':'MOVE_LEFT', 'succ':'EVALUATE',
                                                                     'F':'MOVE_FORWARD'})

        smach.StateMachine.add("MOVE_RIGHT", MoveRight(), transitions={'failed':'INITIAL', 'tries':'MOVE_RIGHT', 'succ':'EVALUATE',
                                                                       'F':'MOVE_FORWARD'})

        smach.StateMachine.add("MOVE_FORWARD", MoveForward(), transitions={'failed':'INITIAL', 'tries':'MOVE_FORWARD', 'succ':'EVALUATE',
                                                                           'obstacle':'EVADE_OBSTACLE'})

        smach.StateMachine.add("MOVE_BACKWARD", MoveBackward(), transitions={'failed':'INITIAL', 'tries':'MOVE_BACKWARD', 'succ':'EVALUATE', 
                                                                        'obstacle':'EVADE_OBSTACLE', 'L':'MOVE_LEFT', 'R':'MOVE_RIGHT'})
        
    rospy.on_shutdown(shutdown_stop)
    outcome = sm.execute()
    rospy.signal_shutdown('')
