#!/usr/bin/env python3

#Basic binary face detector

import rospy
from sensor_msgs.msg import String

def main():
    rospy.init_node("cmd_object_publisher")
    pub_object = rospy.Publisher("/plannning/simple_task/take_object", String, queue_size=10)
    obj = rospy.get_param("~obj","apple")
    pub_object.publish(obj)
    loop = rospy.Rate(1)
    while not rospy.is_shutdown():
        loop.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass


