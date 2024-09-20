#! /usr/bin/env python3
import rospy
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import OccupancyGrid
import numpy as np
import rospkg
import torch

from TorchModels import models

package_path = rospkg.RosPack().get_path("mapless_nav")
model_path = package_path + "/scripts/TorchModels/modelo.pth"
print("model path: ", model_path)
#model = models.SingleConvModel()
model = models.Reg()
model.load_state_dict(torch.load(model_path))
disp = 'cuda' if torch.cuda.is_available() else 'cpu'
model.to(disp)

def occGridArrayCallback(msg):
    global model
    """
    simple_data = msg.data  # Default is a tuple (6400)
    print(len(simple_data))
    print(type(simple_data))
    """
    # Transform to List and concatenate
    data_list = list(msg.data)
    data_list += [0, 0]  # TODO: Concatenate las goal

    print("occ_g_list:", len(data_list))
    print("data_list:", type(data_list))
    print("---")
    # Transform to numpy arrray
    data_np = np.asarray(data_list)

    print("data_np shape=", data_np.shape)
    print("data_np:", type(data_np))
    
    # Expand dims
    data_np = np.expand_dims(data_np, 0)
    print("data_np shape=", data_np.shape)
    print(">>")


# TODO: DELETE
# def occGridCallback(msg):SingleConvModel()
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