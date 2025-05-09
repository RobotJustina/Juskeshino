#!/home/robocup/venvs/python3_9/bin/python
import rospy
import torch
import numpy as np
import copy
import numpy.lib.recfunctions as rf
import os
import matplotlib.pyplot as plt
import geomstats.backend as gs
import numpy.lib.recfunctions as rf
from ros_np_multiarray import ros_np_multiarray as rnm
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import Header
#from rospy.numpy_msg import numpy_msg
from geomstats.geometry.hypersphere import Hypersphere, HypersphereMetric
from manip_msgs.srv import Graspnet
from joint_var_network import Joint_Mixture_Density_Grasp_Network, quick_create_test, DEVICE

# def ros_pc2_to_npmatrix(pc):
#     data = ros_numpy.point_cloud2.pointcloud2_to_array(pc)
#     rgb = ros_numpy.point_cloud2.split_rgb_field(data)
#     dt2 = np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('r', '<f4'), ('g', '<f4'), ('b', '<f4')])
#     rgb = np.asarray(rgb).astype(dt2)
#     rgb['r'] = np.divide(rgb['r'], 255)
#     rgb['g'] = np.divide(rgb['g'], 255)
#     rgb['b'] = np.divide(rgb['b'], 255)
#     return rgb

def npmatrix_to_torch(points):
    #points = rf.structured_to_unstructured(pcd)
    points = torch.tensor(points[:,:,:3],dtype=torch.float32)
    points = torch.nan_to_num(points,nan=0.0)
    points = torch.permute(points,(2,1,0))
    points = points.unsqueeze(0).to(DEVICE)
    return points

def cut_pc(u,v,pc):
    l = 200
    cropped_pc = pc[(u - int(l/2)): (u + int(l/2)) , (v - int(l/2)) : (v + int(l/2))]
    return cropped_pc

def tensor_to_pose(tensor):
    nppose = tensor.detach().cpu().numpy()
    nppose = nppose[0]
    rpose = Pose()
    rpose.position.x = nppose[0]
    rpose.position.y = nppose[1]
    rpose.position.z = nppose[2]
    rpose.orientation.x = nppose[3]
    rpose.orientation.y = nppose[4]
    rpose.orientation.z = nppose[5]
    rpose.orientation.w = nppose[6]
    return rpose

def callback_graspnet(msg):
    global model
    pcd = msg.np_pcd
    #pcd = ros_pc2_to_npmatrix(pcd)
    #pcd = cut_pc(u,v,pcd)
    pcd = rnm.to_numpy_f32(pcd)
    #print(pcd)
    pcd = npmatrix_to_torch(pcd)
    print(pcd.shape)
    with torch.no_grad():
        pos, ori , _ = model(pcd)
        pose = torch.cat((pos,ori),dim=1)
    pose = tensor_to_pose(pose)
    return pose

def main():
    global model
    model = quick_create_test().to(DEVICE)
    model.eval()
    rospy.sleep(0.01)
    print('Graspnet service available')
    rospy.init_node('graspnet_node')
    rospy.Service('/manipulation/grasp/graspnet_request' ,Graspnet ,callback_graspnet)
    loop = rospy.Rate(2)
    while not rospy.is_shutdown():
        loop.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass


