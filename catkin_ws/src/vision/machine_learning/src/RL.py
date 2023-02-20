#! /usr/bin/env python3
#
### mover izquiera o derecha simple_move/goal_dist_lateral
### mover al frente simple_move/goal_dist
import rospy
import numpy as np
from sensor_msgs.msg   import LaserScan
from std_msgs.msg import Float32
from std_msgs.msg import Empty

def Last_pub():
    pub_stop.publish(Empty())
    print("Se cierra el nodo")

def callback_laser_scan(msg):
    global obstacle_left
    obstacle_left=False
    for i in range(154,184):
        temp=msg.ranges[i]<1.0
        obstacle_left=obstacle_left or temp
    return

rospy.init_node("RL")
rospy.Subscriber("/hardware/scan", LaserScan, callback_laser_scan)
pub_lat = rospy.Publisher("/simple_move/goal_dist_lateral", Float32, queue_size=10)
pub_fro = rospy.Publisher("/simple_move/goal_dist", Float32, queue_size=10)
pub_stop = rospy.Publisher("/simple_move/stop", Empty, queue_size=10)
loop = rospy.Rate(10)
rospy.on_shutdown(Last_pub)


def main():
    global obstacle_left
    obstacle_left=False
    goal_lateral=Float32()
    while True:
        if obstacle_left:
            pub_stop.publish(Empty())
        else:
            goal_lateral.data=0.2
            pub_lat.publish(goal_lateral)
        loop.sleep()

if __name__ == '__main__':
    try:
        while not rospy.is_shutdown():
            main()
    except rospy.ROSInterruptException:
        pass
