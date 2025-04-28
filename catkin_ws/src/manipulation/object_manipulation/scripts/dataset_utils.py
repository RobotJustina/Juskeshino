#!/usr/bin/env python
import rospy
import rospkg 
import pickle
import ros_numpy
import numpy as np
import math
import os
import open3d
import copy
import numpy.lib.recfunctions as rf
import torch
import cv2
import tf2_ros
from sensor_msgs.msg import PointCloud2
from tf2_geometry_msgs import PointStamped, PoseStamped
from geometry_msgs.msg import Point, PointStamped, PoseStamped
from gazebo_msgs.srv import GetModelState
from visualization_msgs.msg import Marker
from scipy.spatial import cKDTree
from tempfile import TemporaryFile
from io import BytesIO
import sqlite3

MAX_POINTS = 25600
#DATASET_PATH = 'catkin_ws/src/graspnet/dataset/'
#DATASET_PATH = 'catkin_ws/src/graspnet/dataset_test/'
#DATASET_PATH = 'catkin_ws/src/graspnet/dataset_fake/'
#DATASET_PATH = 'catkin_ws/src/graspnet/dataset_base_link/'
#DATASET_PATH = 'catkin_ws/src/graspnet/dataset_cube_test/'
#DATASET_PATH = 'catkin_ws/src/graspnet/dataset_rest_test/'
#DATASET_PATH = 'catkin_ws/src/graspnet/training_dataset/'

rospack = rospkg.RosPack()
graspnet_path = rospack.get_path('graspnet')
print(graspnet_path)
DATASET_PATH = graspnet_path + "/training_dataset/"
#DATABASE_PATH = graspnet_path + '/grasp_database_test_siu.db'
DATABASE_PATH = graspnet_path + '/grasp_database_quaternion.db'
#DATASET_PATH = graspnet_path + "/validate_dataset/"

DEVICE = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

gpus = 1

def debug_type(obj, obj_name):
    print(obj_name + "Object characteristics:")
    print(obj)
    print(obj.shape)
    print(obj.dtype)
    print(type(obj))

def ros_pc2_to_nparray(pc):
    data = ros_numpy.point_cloud2.pointcloud2_to_array(pc)
    data = data.reshape(-1)
    rgb = ros_numpy.point_cloud2.split_rgb_field(data)
    dt2 = np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('r', '<f4'), ('g', '<f4'), ('b', '<f4')])
    rgb = np.asarray(rgb).astype(dt2)
    rgb['r'] = np.divide(rgb['r'], 255)
    rgb['g'] = np.divide(rgb['g'], 255)
    rgb['b'] = np.divide(rgb['b'], 255)
    rgb = rf.structured_to_unstructured(rgb)
    rgb = rgb[~np.isnan(rgb).any(axis=1)]
    return rgb

def ros_pc2_to_npmatrix(pc):
    print(pc.header.frame_id)
    data = ros_numpy.point_cloud2.pointcloud2_to_array(pc)
    #debug_type(data,"Split points")
    rgb = ros_numpy.point_cloud2.split_rgb_field(data)
    #debug_type(rgb,"Split points + RGB")
    dt2 = np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('r', '<f4'), ('g', '<f4'), ('b', '<f4')])
    rgb = np.asarray(rgb).astype(dt2)
    rgb['r'] = np.divide(rgb['r'], 255)
    rgb['g'] = np.divide(rgb['g'], 255)
    rgb['b'] = np.divide(rgb['b'], 255)
    #debug_type(rgb, "Split points + Normalized RGB")
    return rgb

def save_to_file(pcd, grasp, gr_pose, obj_relative_pos, head_pose_q, obj_type, score, file_path):
    output_dict = {
        #'points'             : pc, #np array of XYZ points of pointcloud w.r.p to camera_link
        #'colors'             : color, #np array of RGB values of pointcloud in [0,1] range (necessary for open3d and tensor)
        'pcd'                : pcd,
        'grasp'              : grasp, #[x, y, z, roll, pitch, yaw, w] array of final grip center pose w.r.p to camera_link
        'gripper_origin'     : gr_pose, #Pose of gr_left_arm_link7 w.r.p to camera_link
        'obj_relative_pos'   : obj_relative_pos, #Object relative position w.r.p to camera_link
        'head_pose_q'        : head_pose_q, #Array of head angles at capture time
        'obj_type'           : obj_type, #String of object type
        'scores'             : score, #Assigned score for grasp
    }
    print(file_path)
    if file_path:
        with open(file_path, 'wb') as file:
            pickle.dump(output_dict, file)

def update_grasp_to_db_by_id(grasp,obj_shape,obj_type,pcd_id, id):
    x,y,z,i,j,k,w = grasp
    conn = sqlite3.connect(DATABASE_PATH)
    cursor = conn.cursor()
    cursor.execute('''
    UPDATE grasps_table
    SET x = ?, y=?, z=?, i=?, j=?, k=?, w=?, obj_shape=?, obj_type=?, pcd_id=?
    WHERE id = ?;
    ''',(x,y,z,i,j,k,w,obj_shape,obj_type,pcd_id, id))
    grasp_id = cursor.lastrowid
    conn.commit()
    conn.close()
    return grasp_id

def update_pcd_to_db_by_id(pcd, id):
    #print(pcd.shape)
    binary_stream = BytesIO()
    np.save(binary_stream,pcd)
    pcd_binary = binary_stream.getvalue()
    conn = sqlite3.connect(DATABASE_PATH)
    cursor = conn.cursor()
    cursor.execute('''
    UPDATE point_clouds_table
    SET pcd_binary = ?
    WHERE id = ?
    ''',(pcd_binary, id))
    pcd_id = cursor.lastrowid
    conn.commit()
    conn.close()
    return pcd_id


def save_grasp_to_db(grasp,obj_shape,obj_type,pcd_id):
    x,y,z,i,j,k,w = grasp
    conn = sqlite3.connect(DATABASE_PATH)
    cursor = conn.cursor()
    cursor.execute('''
    INSERT INTO grasps_table(x,y,z,i,j,k,w,obj_shape,obj_type,pcd_id)
    VALUES (?,?,?,?,?,?,?,?,?,?)
    ''',(x,y,z,i,j,k,w,obj_shape,obj_type,pcd_id))
    grasp_id = cursor.lastrowid
    conn.commit()
    conn.close()
    return grasp_id

def save_pcd_to_db(pcd):
    #print(pcd.shape)
    binary_stream = BytesIO()
    np.save(binary_stream,pcd)
    pcd_binary = binary_stream.getvalue()
    conn = sqlite3.connect(DATABASE_PATH)
    cursor = conn.cursor()
    cursor.execute('''
    INSERT INTO point_clouds_table(pcd_binary)
    VALUES (?)
    ''',(pcd_binary,))
    pcd_id = cursor.lastrowid
    conn.commit()
    conn.close()
    return pcd_id

def load_grasp_from_db(id):
    conn = sqlite3.connect(DATABASE_PATH)
    cursor = conn.cursor()
    cursor.execute("SELECT x,y,z,i,j,k,w, (SELECT pcd_binary FROM point_clouds_table WHERE point_clouds_table.pcd_id = grasps_table.pcd_id) FROM grasps_table WHERE grasp_id = {}".format(id))
    row = cursor.fetchall()
    conn.commit()
    conn.close()
    return row

def load_pcd_from_db(id):
    conn = sqlite3.connect(DATABASE_PATH)
    cursor = conn.cursor()
    cursor.execute("SELECT pcd_binary FROM point_clouds_table WHERE pcd_id = {}".format(id))
    row = cursor.fetchone()
    conn.commit()
    conn.close()
    return row

def show_pcd_from_file(path):
    data = np.load(os.path.abspath(path), allow_pickle=True)
    pcd = data['pcd']
    pcd = pcd.reshape(-1)
    pcd = rf.structured_to_unstructured(pcd)
    pcd = pcd[~np.isnan(pcd).any(axis=1)]
    #view = data['points']
    #color = data['colors']
    grip = data['grasp']
    score = data['scores']
    obj_pos = data['obj_relative_pos']
    gr_pose = data['gripper_origin']
    print(grip)
    print(score)
    print(obj_pos)
    print(gr_pose)
    view_point_cloud = open3d.geometry.PointCloud()
    view_point_cloud.points = open3d.utility.Vector3dVector(copy.deepcopy(pcd[:,:3]))
    view_point_cloud.colors = open3d.utility.Vector3dVector(copy.deepcopy(pcd[:,3:6]))
    open3d.visualization.draw_geometries([view_point_cloud])

def show_pcd_from_npmatrix(mat):
    rgb = copy.deepcopy(mat)
    rgb = rgb.reshape(-1)
    rgb = rf.structured_to_unstructured(rgb)
    rgb = rgb[~np.isnan(rgb).any(axis=1)]
    view_point_cloud = open3d.geometry.PointCloud()
    debug_type(rgb, "Reshaped pcd ")
    view_point_cloud.points = open3d.utility.Vector3dVector(copy.deepcopy(rgb[:,:3]))
    view_point_cloud.colors = open3d.utility.Vector3dVector(copy.deepcopy(rgb[:,3:6]))
    open3d.visualization.draw_geometries([view_point_cloud])

def show_pcd_from_unstructured_npmatrix(mat):
    rgb = copy.deepcopy(mat)
    rgb = rgb.reshape(-1)
    #rgb = rf.structured_to_unstructured(rgb)
    rgb = rgb[~np.isnan(rgb).any(axis=1)]
    view_point_cloud = open3d.geometry.PointCloud()
    debug_type(rgb, "Reshaped pcd ")
    view_point_cloud.points = open3d.utility.Vector3dVector(copy.deepcopy(rgb[:,:3]))
    #view_point_cloud.colors = open3d.utility.Vector3dVector(copy.deepcopy(rgb[:,3:6]))
    open3d.visualization.draw_geometries([view_point_cloud])

def nparray_pc_to_torch(pc):
    pc_back, color_back = copy.deepcopy(pc[:,:3]), copy.deepcopy(pc[:,3:6])
    select_point_index = None
    if len(pc) >= MAX_POINTS:
        select_point_index = np.random.choice(len(pc), MAX_POINTS, replace=False)
    elif len(pc) < MAX_POINTS:
        select_point_index = np.random.choice(len(pc), MAX_POINTS, replace=True)
    pc = pc[select_point_index]
    pc_torch = torch.Tensor(pc).view(1, -1, 6)
    if gpus != -1:
        pc_torch = pc_torch.cuda()
    print("Converted tensor characteristics")
    print(pc_torch)
    print(pc_torch.shape)
    print(type(pc_torch))
    return pc_back, color_back, pc_torch

def npmatrix_to_torch(pcd):
    points = rf.structured_to_unstructured(pcd)
    points = torch.tensor(points[:,:,:3],dtype=torch.float32)
    points = torch.permute(points,(2,1,0))
    points.to(DEVICE)
    return points

def save_data_to_file(resp, file_num=1):
    pcd = resp.pointcloud
    #pc = ros_pc2_to_nparray(pcd)
    #pc, color, _ = nparray_pc_to_torch(pc)
    pcd = ros_pc2_to_npmatrix(pcd)
    grasp = resp.final_grasp_q
    gr_pose = resp.gripper_pose
    obj_relative_pos = resp.obj_relative_pos
    #obj_relative_pos = camera_link_to_optical_frame(obj_relative_pos)
    #obj_relative_pos = camera_link_to_base_link(obj_relative_pos)
    head_pose_q = resp.head_pose_q
    obj_type = resp.obj_type
    score = resp.score
    u, v, obj_in_range = find_nearest_pt_in_pc(pcd, obj_relative_pos)
    if obj_in_range:
        file_path = DATASET_PATH + "example_" + str(file_num) + ".p"

        cut_pcd = cut_pc(u,v,pcd)
        #show_rgb(cut_pc)

        save_to_file(cut_pcd, grasp, gr_pose, obj_relative_pos, head_pose_q, obj_type, score, file_path)
        return 1
    else:
        return 0

def camera_link_to_optical_frame(pt):
    target_pt = Point()
    target_pt.x = -pt.y
    target_pt.y = -pt.z
    target_pt.z = pt.x
    return target_pt

def camera_link_to_base_link(pt):
    global tf_listener, tf_buf
    # tf_buf = tf2_ros.Buffer()
    # tf_listener = tf2_ros.TransformListener(tf_buf)
    pst = PoseStamped()
    pst.header.frame_id = 'camera_link'
    pst.pose.position = pt
    pst.pose.orientation.w = 1
    target_pt = tf_buf.transform(pst,'base_link')
    return target_pt.pose.position

def create_marker_from_pt(pt):
    global marker_pub
    marker = Marker()
    marker.header.frame_id = "camera_rgb_optical_frame"
    marker.type = Marker.SPHERE
    marker.ns = "obje"
    marker.header.stamp = rospy.Time.now()
    marker.action = marker.ADD
    marker.id = 1
    marker.scale.x, marker.scale.y, marker.scale.z = 0.02, 0.2, 0.2
    marker.color.r, marker.color.g, marker.color.b, marker.color.a = 20, 50, 100, 1.0
    marker.lifetime = rospy.Duration(10)
    marker.pose.position = pt
    marker.pose.orientation.w = 1
    marker_pub.publish(marker)

def find_nearest_pt_in_pc(pc, pt):
    valid = False
    matrix = copy.deepcopy(pc)
    matrix = matrix.reshape(-1)
    matrix = rf.structured_to_unstructured(matrix)
    search_vec = np.array([pt.x, pt.y, pt.z])
    nearest = cKDTree(matrix[:,:3]).query(search_vec, k=1)[1]
    u = math.floor(nearest/pc.shape[1])
    v = nearest%pc.shape[1]
    if 100 < u < 380 and 100 < v < 540: valid = True
    #print(u,v,valid)
    return u,v,valid

def find_pt_in_pc(position_obj, pc):
    min_dist = 100000
    u, v = 0, 0
    for i in range(len(pc)):
        for j in range(len(pc[0])):
            dist = np.linalg.norm(np.array([pc[i,j]['x'] - position_obj.x, pc[i,j]['y'] - position_obj.y, pc[i,j]['z'] - position_obj.z]))
            if dist < min_dist:
                min_dist = dist
                u = i
                v = j
    return u, v
                

def cut_pc(u,v, pc):
    l, w = 200, 200
    cropped_pc = pc[(u - int(l/2)): (u + int(l/2)) , (v - int(l/2)) : (v + int(l/2))]
    return cropped_pc


def show_rgb(pc):
    img_xyz = ros_numpy.point_cloud2.pointcloud2_to_array(pc)  # dim 480 x 640, 
    rgb_array = img_xyz['rgb'].copy()     # Pass a copy of rgb float32, 480 x 640
    rgb_array.dtype = np.uint32       # Config data type of elements from array
    r,g,b = ((rgb_array >> 16) & 255), ((rgb_array >> 8) & 255), (rgb_array & 255)  # 480 x 640 c/u
    img_bgr = cv2.merge((np.asarray(b,dtype='uint8'),np.asarray(g,dtype='uint8'),np.asarray(r,dtype='uint8')))
    cv2.imshow("imagen sin plano #1", img_bgr) #*****************
    cv2.waitKey(0)


def main():
    global marker_pub, tf_listener, tf_buf
    print("Dataset utils started")
    obj_shape = rospy.get_param("/obj","056_tennis_ball")
    get_object_relative_pose = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    marker_pub = rospy.Publisher("/vision/object_recognition/markers", Marker, queue_size = 10)
    rospy.init_node('dataset_utils')
    loop = rospy.Rate(1)
    while not rospy.is_shutdown():
        print("Choose command")
        command = input()
        if command == 'y':
            pcd = rospy.wait_for_message("/camera/depth_registered/points", PointCloud2)
            pc = ros_pc2_to_nparray(pcd)
            pc, color, _ = nparray_pc_to_torch(pc)
            grasp = [1, 1, 1, 1, 1, 1, 1]
            gr_pose = "siu"
            obj_relative_pos ="Object pos"
            head_pose_q = [0, 0]
            obj_type = "apple"
            score = 10
            file_path = DATASET_PATH + "test1.p"
            save_to_file(pc, color, grasp, gr_pose, obj_relative_pos, head_pose_q, obj_type, score, file_path)
        if command == 's':
            print("Show number")
            num = input()
            show_pcd_from_file(DATASET_PATH + "example_" + num + ".p")
        if command == 'p':
            pcd = rospy.wait_for_message("/camera/depth_registered/points", PointCloud2)
            mat = ros_pc2_to_npmatrix(pcd)
            show_pcd_from_npmatrix(mat)
        if command == 'c':
            pcd = rospy.wait_for_message("/camera/depth_registered/points", PointCloud2)
            obj_pt = get_object_relative_pose(obj_shape,"justina::camera_link").pose.position
            obj_pt = camera_link_to_optical_frame(obj_pt)
            create_marker_from_pt(obj_pt)
        if command == 'r':
            pcd = rospy.wait_for_message("/camera/depth_registered/points", PointCloud2)
            obj_pt = get_object_relative_pose("justina_gripper","justina::camera_link").pose.position
            mat = ros_pc2_to_npmatrix(pcd)
            t_pt = camera_link_to_optical_frame(obj_pt)
            u, v = find_nearest_pt_in_pc(mat,t_pt)
            found_pt = Point(x=mat[u,v]['x'], y=mat[u,v]['y'], z=mat[u,v]['z'])
            create_marker_from_pt(found_pt)
        if command == 'l':
            print("What grasp to get?")
            id = input()
            g = load_grasp_from_db(int(id))
            # print(type(g))
            # print(len(g))
            # print(type(g[0]))
            # print(len(g[0]))
            print(type(g[0][7]))
            print(len(g[0][7]))
            #pcd = np.array(g[0][7],dtype=np.float32)
            pcd = np.frombuffer(g[0][7],dtype='float')
            pcd = pcd.reshape((480,640,3))
            print(type(pcd))
            print(pcd.shape)
            print(type(pcd[0]))
            print(pcd[0].shape)
            show_pcd_from_unstructured_npmatrix(pcd)
            #fileobj = BytesIO(bytes.fromhex(g[0][7]))
            #pcd = np.load(fileobj)
            # pcd = np.frombuffer(g[0][7])
            # pcd.reshape((480,640,3))
            # show_pcd_from_npmatrix(pcd)
        if command == 'sdb':
            print("What grasp to get?")
            id = input()
            g = load_pcd_from_db(int(id))
            print(type(g))
            print(len(g))
            print(type(g[0]))
            print(len(g[0]))
            pcd = np.load(BytesIO(g[0]))
            print(pcd)
            show_pcd_from_npmatrix(pcd)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass