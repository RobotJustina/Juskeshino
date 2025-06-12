#! /usr/bin/env python3

import rospy
import smach, smach_ros
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
import numpy as np
import matplotlib.pyplot as plt
import cv2
base = None 

#_________
class SimpleBase():
    def __init__(self):
        self._base_vel_pub = rospy.Publisher('/hardware/mobile_base/cmd_vel' , Twist, queue_size=10)
    
    def call_base(self):
        print("-BASE-")

    def move_vel(self, l_velX, l_velY, a_velZ):
        twist = Twist()
        twist.linear.x = l_velX
        twist.linear.y = l_velY
        twist.angular.z = a_velZ
        self._base_vel_pub.publish(twist)


class Features():
    def __init__(self):
        self.left_obst = []
        self.right_obst = []
        self.cent_obst = []
        self.wall = []

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

#_________


def occGridCallback(msg):
    global feat

    data = np.asarray(msg.data)
    rows = msg.info.height
    data = np.reshape(data, (rows, rows))
    data = np.rot90(np.flip(data, axis=0))
    c = rows// 2


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


    data_C = data[c:, c-6:c+6]
    data_C2 = data_C.sum(0)//100

    # print(data_C2)
    # print("Shape ",data_C2.shape)
    # show_image(data_C, "img")
    

    data_F = data[:c, :]
    data_F2 = data_F.sum(0)//100

    # print(data_F2)
    # print("Shape ",data_F2.shape)
    # show_image(data_F, "img")

    
    feat.set_features(data_L2, data_R2, data_C2, data_F2)
    rospy.sleep(0.2)
    feat.set_features(data_L2, data_R2, data_C2, data_F2)
    # weight = 1
    # for i in range(len(data_R2)):
    #     data_R2[i] = data_R2[i] * weight
    #     weight = weight - 1/(len(data_R2))





def show_image(img, name):
    plt.title(name)
    plt.imshow(img, cmap='gray') 
    plt.show()


#>>> STATE MACHINE
class Initial(smach.State):
    def __init__(self):
        smach.State.__init__(self , outcomes=['succ', 'failed'])
        self.tries = 0

    def execute(self, userdata):
        rospy.logwarn('--> STATE <: Initial')
        global base, feat
        base = SimpleBase()
        rospy.sleep(1.0)
        return 'succ'


class Evaluate(smach.State):
    def __init__(self):
        smach.State.__init__(self , outcomes=['succ', 'failed'])

    def execute(self, userdata): 
        rospy.logwarn('--> STATE <: Evaluate')
        global feat
        obst_l, obst_r, obst_c, obst_f = feat.get_features()
        print("obst left", obst_l)
        print("obst right", obst_r)
        print("obst center", obst_c)
        print("obst front", obst_f)

        # logic

        return 'succ'
    

class NavForward(smach.State):
    def __init__(self):
        smach.State.__init__(self , outcomes=['tries', 'failed', 'succ'])
        self.tries = 0
        #self.base_vel_pub = rospy.Publisher('/hardware/mobile_base/cmd_vel' , Twist, queue_size=10)

    def execute(self, userdata):
        global base, feat
        if self.tries == 0:
            rospy.logwarn('--> STATE <: NavForward')


        base.call_base()
        base.move_vel(1.0, 0.0, 0.0)

        self.tries = self.tries + 1

        if self.tries > 500:
            return 'succ'

        return 'tries'


def shutdown_stop():
    global base
    print("Exit")
    base.call_base()
    base.move_vel(0.0, 0.0, 0.0)
    rospy.sleep(0.5)


if __name__ == '__main__':
    global feat
    print("State machine")
    rospy.init_node('smach_react_nav')
    
    rospy.Subscriber("/local_occ_grid", OccupancyGrid, occGridCallback)

    feat = Features()

    sm = smach.StateMachine(outcomes=['END'])
    sis = smach_ros.IntrospectionServer('SMACH_VIEW_SERVER', sm, '/SM_REACTIVE_NAV')
    sis.start()

    with sm:
        smach.StateMachine.add("INITIAL", Initial(), transitions={'failed':'INITIAL', 'succ':'EVALUATE'})

        smach.StateMachine.add("EVALUATE", Evaluate(), transitions={'failed':'INITIAL', 'succ':'NAV_FORWARD'})

        smach.StateMachine.add("NAV_FORWARD", NavForward(), transitions={'failed':'INITIAL', 'tries':'NAV_FORWARD', 'succ':'END'})

    rospy.on_shutdown(shutdown_stop)
    outcome = sm.execute()
    rospy.signal_shutdown('')
