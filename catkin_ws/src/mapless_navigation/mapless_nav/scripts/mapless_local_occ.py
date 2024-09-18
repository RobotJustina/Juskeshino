#! /usr/bin/env python3
import rospy
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import OccupancyGrid
import numpy as np
import rospkg
import torch

#from models import nn_models

# package_path = rospkg.RosPack().get_path("mapless_nav")
# model_path = package_path + "/models/modelo.pth"
# print(model_path)
# model = nn_models.SingleConvModel()
# model.load_state_dict(torch.load(model_path))

def occGridArrayCallback(msg):
    data = np.asarray(msg.data)

    print("occ_g_a:", data.shape)

# TODO: DELETE
# def occGridCallback(msg):
#     data = np.asarray(msg.data)

#     print("occ_g:", data.shape)


if __name__ == "__main__":
    rospy.init_node("mapless_local_occupancy")
    rospy.loginfo("INITIALIZING MAPLESS LOCAL OCCUPANCY")

    loc_occ_grid_sub = rospy.Subscriber("/local_occ_grid_array", Float32MultiArray, occGridArrayCallback)
    
    # TODO: DELETE
    #loc_occ_sub = rospy.Subscriber("/local_occ_grid", OccupancyGrid, occGridCallback)


    loop = rospy.Rate(1)
    while not rospy.is_shutdown():

        

        loop.sleep