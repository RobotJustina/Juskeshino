#! /usr/bin/env python3
import rospy
import rosbag

def rosbag_read():
    print("funciona")
    bag = rosbag.Bag('test.bag')
    for topic, msg, t in bag.read_messages(topics=['/cmd_vel']):
        print(msg)
    bag.close()


if __name__ == '__main__':
    try:
        rosbag_read()
    except rospy.ROSInterruptException:
        pass
