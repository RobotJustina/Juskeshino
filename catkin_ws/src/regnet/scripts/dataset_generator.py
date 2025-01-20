#!/home/robocup/regnet/REGNet_for_3D_Grasping/env/bin/python

import rospy
import numpy as np
import ros_numpy
import glob
import torch
from regnetmodule import RegnetModule, REGNET_PATH
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2utils
import numpy.lib.recfunctions as rf
import regnetmodule as rm

def main():
    print("Starting dataset node")
    rospy.init_node("dataset_node")
    loop = rospy.Rate(0.2)
    while not rospy.is_shutdown():
        rospy.sleep(1)
        read_file()
        loop.sleep()
    

def read_file():
    #refinedModule = RegnetModule()
    test_file_path = REGNET_PATH + '/test_file/virtual_data'
    open3d_data = True if 'real_data' in test_file_path else False
    print(open3d_data)
    if open3d_data:
        pc_paths = glob.glob(test_file_path+"/*.pcd",recursive=True)
    else:
        pc_paths = glob.glob(test_file_path+"/*.p",recursive=True)
    for pc_path in pc_paths:
        rm.file_type_config_eval(open3d_data, pc_path)

if __name__ == "__main__":
    main()
