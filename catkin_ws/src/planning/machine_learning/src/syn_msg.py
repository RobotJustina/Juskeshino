#! /usr/bin/env python3

import rospy
import numpy as np
import cv2
from std_msgs.msg import Float32MultiArray

info=[]

def callback_grid(msg):
	img=np.array(msg.data, dtype="uint8")
	img=np.reshape(img,(80,80), order="F")
	img[img==100]=255
	resized_image = cv2.resize(img, (400, 400), interpolation = cv2.INTER_CUBIC)
        cv2.imshow("Obstacle image", resized_image)
        cv2.waitKey(1)

def main():
	rospy.init_node("sync_msg")
	rospy.Subscriber("/local_occ_grid_array", Float32MultiArray, callback_grid)
	print("grid shower has been started")
	rospy.spin()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
