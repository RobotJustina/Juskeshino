#! /usr/bin/env python3

import rospy
import numpy as np
import cv2
from std_msgs.msg import Int32MultiArray

def callback_grid(msg):
    #print(msg.data[2:12])
    image=np.zeros((30,30), dtype="uint8")
    for i in range(0,30):
        image[i,:30]=msg.data[i*30:i*30+30]
    image=255*image
    resized_image = cv2.resize(image, (300, 300), interpolation = cv2.INTER_CUBIC)
    cv2.imshow("Obstacle image", resized_image)
    cv2.waitKey(1)

def main():
    rospy.init_node("show_grid")
    rospy.Subscriber("/grid", Int32MultiArray, callback_grid)
    print("grid shower has been started")
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
