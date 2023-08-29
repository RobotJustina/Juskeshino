#! /usr/bin/env python3

import rospy
import numpy as np
import cv2
from std_msgs.msg import Int32MultiArray

def callback_grid(msg):
    img=np.array(msg.data, dtype="uint8")
    img=np.reshape(img,(30,30), order="C")
    img[img==1]=255
    resized_image = cv2.resize(img, (400, 400), interpolation = cv2.INTER_CUBIC)
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
