#! /usr/bin/env python3
import rospy
import tf
from nav_msgs.msg import OccupancyGrid
import numpy as np

recording = False
occ_grid = None

def occGridCallback(msg):
    global occ_grid
    occ_grid = msg


def main():
    global recording, occ_grid

    rospy.init_node("occ_grid_re_publisher")
    rospy.loginfo("INITIALIZING occ_grid_re_publisher")

    listener = tf.TransformListener()
    listener.waitForTransform("odom", "base_link", rospy.Time(), rospy.Duration(4.0))

    rospy.Subscriber("/local_occ_grid", OccupancyGrid, occGridCallback)
    pubOccGridMat = rospy.Publisher("/re_local_occ_grid", OccupancyGrid, queue_size=1)

    loop = rospy.Rate(5)
    while not rospy.is_shutdown():
        
        if occ_grid != None:
            print(type(occ_grid))
            pubOccGridMat.publish(occ_grid)
        
        loop.sleep()


if __name__ == "__main__":
     main()