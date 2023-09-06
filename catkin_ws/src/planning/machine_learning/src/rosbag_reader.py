#! /usr/bin/env python3
import rospy
import rosbag
import numpy as np
import matplotlib.pyplot as plt

def rosbag_read():
    print("funciona")
    cmd_list=[]
    files=['cmd_vel_2.bag', 'cmd_vel.bag']
    for file in files:
        bag = rosbag.Bag(file)
        for topic, msg, t in bag.read_messages(topics=['/hardware/mobile_base/cmd_vel']):
            cmd_list.append([msg.linear.x, msg.angular.z])
        bag.close()
    cmd_numpy=np.array(cmd_list)

    np.savetxt('Velocidades.csv', cmd_numpy, delimiter=',')
    cmd_numpy=np.transpose(cmd_numpy)
    print(cmd_numpy.shape)

    #plt.scatter(cmd_numpy[0,:], cmd_numpy[1,:], label="Datos", color="blue")
    #plt.ylabel('Velocidad angular')
    #plt.xlabel('Velocidad lineal')
    #plt.legend()
    #plt.show()

if __name__ == '__main__':
    try:
        rosbag_read()
    except rospy.ROSInterruptException:
        pass
