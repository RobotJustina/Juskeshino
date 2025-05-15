#!/usr/bin/env python3

import rospy 
import rospkg 
import random
import math
import os
import matplotlib.pyplot as plt
import tf.transformations as tft
import numpy as np
import numpy.lib.recfunctions as rf
import vg
import tf2_ros
from tf2_geometry_msgs import PointStamped
from vision_msgs.srv import PreprocessPointCloud, PreprocessPointCloudRequest, RecognizeObject, RecognizeObjectRequest
from gazebo_msgs.msg import ModelState, ContactsState
from gazebo_msgs.srv import SetModelState, GetModelState, SetModelConfiguration, SetModelConfigurationRequest
from std_msgs.msg import String, Float64MultiArray, Header
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Pose, Point, Quaternion  
from manip_msgs.srv import Graspnet, GraspnetRequest
#from train import load_model, DEVICE
import geometry_msgs
import grasp_network as gn
#from position_network import Position_network, load_model, DEVICE
#from joint_var_network import Joint_Mixture_Density_Grasp_Network, quick_create_test, DEVICE
from visualization_msgs.msg import Marker
import geomstats.backend as gs
from geomstats.geometry.hypersphere import Hypersphere, HypersphereMetric
from ros_np_multiarray import ros_np_multiarray as rnm
#from ros_np_multiarray.src.ros_np_multiarray.ros_np_multiarray import _numpy_to_multiarray


#from ...manipulation.object_manipulation.scripts import dataset_utils as dutils
#import ...manipulation.object_manipulation.scripts.dataset_utils as dtutils
import torch

MODELS_PATH = "/home/robocup/Juskeshino/catkin_ws/src/graspnet/models/"
# Device configuration
DEVICE = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

GRIPPER_ORIGIN = ModelState()
GRIPPER_ORIGIN.model_name = "justina_gripper"
GRIPPER_ORIGIN.pose = Pose()
GRIPPER_ORIGIN.pose.orientation.w = 1
GRIPPER_ORIGIN.reference_frame = "world"

def broadcaster_frame_object(frame, child_frame, pose):   # Emite la transformacion en el frame base_link,
    #br = tf2_ros.TransformBroadcaster()
    br =  tf2_ros.StaticTransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()
    t.header.frame_id = frame
    t.child_frame_id = child_frame 
    t.header.stamp = rospy.Time.now()
    t.transform.translation.x = pose.position.x
    t.transform.translation.y = pose.position.y
    t.transform.translation.z = pose.position.z
    t.transform.rotation.x = pose.orientation.x
    t.transform.rotation.y = pose.orientation.y
    t.transform.rotation.z = pose.orientation.z
    t.transform.rotation.w = pose.orientation.w
    br.sendTransform(t)

def get_Z_obj():
    global obj_shape
    z_obj_dic = {            # z1  , z2
        '024_bowl':          [0.724, 0.724],
        '001_chips_can':     [0.740, 0.800], 
        '007_tuna_fish_can': [0.743, 0.717],
        '056_tennis_ball':   [0.732, 0.732],
        '006_mustard_bottle':[0.729, 0.783],
        '003_cracker_box':   [0.780, 0.805], 
        '077_rubiks_cube':   [0.728, 0.728],
        '011_banana':        [0.717, 0.717], 
        '048_hammer':        [0.716, 0.716]
    }
    z1 = z_obj_dic[obj_shape][0]
    z2 = z_obj_dic[obj_shape][1]
    return z1, z2



def categorize_objs(name):
    dishes    = ['024_bowl']
    prismatic = ['001_chips_can', '007_tuna_fish_can']
    spherical = ['054_softball', '055_baseball', '056_tennis_ball']
    flat      = ['006_mustard_bottle']
    box       = ['pudding_box', '003_cracker_box']
    cubic     = ['077_rubiks_cube']
    two_faces = ['011_banana', '048_hammer', '044_flat_screwdriver']
    if   name in dishes:    return 'dishes'
    elif name in prismatic: return 'prismatic'
    elif name in spherical: return 'spherical'
    elif name in flat:      return "flat"
    elif name in box:       return 'box'
    elif name in cubic:     return 'cubic'
    elif name in two_faces: return '2faces'

    

def rotation_object():
    global obj_shape
    geometric_shape_dic = {
                            "dishes":   [[0, 0, np.deg2rad(random.randint(0, int(359)))]],
                            "prismatic":[[0, 0, np.deg2rad(random.randint(0, int(359)))], [0, 1.57, np.deg2rad(random.randint(0, int(359)))] ],
                            "spherical":[[np.deg2rad(random.randint(0, int(359))) , np.deg2rad(random.randint(0, int(359))) ,np.deg2rad(random.randint(0, int(359)))]],
                            "flat":     [[0, 0, np.deg2rad(random.randint(0, int(359)))], [0, 1.57, np.deg2rad(random.randint(0, int(359)))] ,  [0, -1.57, np.deg2rad(random.randint(0, int(359)))] ],
                            "cubic":    [ [0, 1.57 , np.deg2rad(random.randint(0, int(359)))] ,  [0, 3.14, np.deg2rad(random.randint(0, int(359)))],  [0, 4.71, np.deg2rad(random.randint(0, int(359)))],  [0, 6.28, np.deg2rad(random.randint(0, int(359)))],
                                        [1.57, 0 , np.deg2rad(random.randint(0, int(359)))] ,  [3.14, 0,  np.deg2rad(random.randint(0, int(359)))], [4.71, 0, np.deg2rad(random.randint(0, int(359)))],  [6.28, 0, np.deg2rad(random.randint(0, int(359)))]],
                            "box":      [[0, 0, np.deg2rad(random.randint(0, int(359)))],  [3.14, 0,  np.deg2rad(random.randint(0, int(359)))],
                                        [1.57, 0 , np.deg2rad(random.randint(0, int(359)))] , [4.71, 0, np.deg2rad(random.randint(0, int(359)))]],
                            "2faces":   [[0, 0, np.deg2rad(random.randint(0, int(359)))] ,  [0, 3.14, np.deg2rad(random.randint(0, int(359)))]]
    }
    rotation = random.choice(geometric_shape_dic[categorize_objs(obj_shape)])
    z1, z2 = get_Z_obj()
    
    if(((int(np.rad2deg(rotation[0])) == 0) or ((int(np.rad2deg(rotation[0])) > 175) and  (int(np.rad2deg(rotation[0])) < 185)))  and (int(np.rad2deg(rotation[1])) == 0)): z = z2
    else:z = z1
    quaternion_obj = tft.quaternion_from_euler(rotation[0],rotation[1],rotation[2] ,'sxyz')
    return quaternion_obj, z


def generate_random_pose():
    global z
    rpose = Pose()
    rpose.position.x = random.randint(210,310)/100
    rpose.position.y = random.randint(218,245)/100
    q, z = rotation_object()
    rpose.position.z = z
    rpose.orientation.x = q[0]
    rpose.orientation.y = q[1]
    rpose.orientation.z = q[2]
    rpose.orientation.w = q[3]
    return rpose

def normalize(v):
    norm = np.linalg.norm(v)
    if norm == 0: 
       return v
    return v / norm

def tensor_to_pose(tensor):
    nppose = tensor.detach().cpu().numpy()
    nppose = nppose[0]
    #i,j,k,c,s = nppose[3:]
    #ang = math.atan2(c,s)
    #vec = normalize([i,j,k])*math.sin(ang)
    #quat = np.append(vec,[math.cos(ang)])
    #quat = normalize(nppose[3:])
    rpose = Pose()
    rpose.position.x = nppose[0]
    rpose.position.y = nppose[1]
    rpose.position.z = nppose[2]
    # rpose.orientation.x = quat[0]
    # rpose.orientation.y = quat[1]
    # rpose.orientation.z = quat[2]
    # rpose.orientation.w = quat[3]
    rpose.orientation.x = nppose[3]
    rpose.orientation.y = nppose[4]
    rpose.orientation.z = nppose[5]
    rpose.orientation.w = nppose[6]
    return rpose

def tensor_to_point(tensor):
    nppose = tensor.detach().cpu().numpy()
    nppose = nppose[0]
    rpose = Point()
    rpose.x = nppose[0]
    rpose.y = nppose[1]
    rpose.z = nppose[2]

    return rpose

def graspnet_ref_to_gripper_ref(q):
    ql = [q.x,q.y,q.z,q.w]
    rot = tft.quaternion_matrix(ql)
    y = rot[:3,1]
    rot[:3,2] = -rot[:3,2]
    rot[:3,1] = rot[:3,0]
    rot[:3,0] = y
    #print(rot)
    qf = tft.quaternion_from_matrix(rot)
    qr = Quaternion()
    qr.x = qf[0]
    qr.y = qf[1]
    qr.z = qf[2]
    qr.w = qf[3]
    return qr


def change_gazebo_object_pose(state_msg, state_pose, mod_name):
    global set_state
    state_msg.model_name = mod_name
    state_msg.pose = state_pose
    #rospy.wait_for_service('/gazebo/set_model_state')
    try:
        #set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        resp = set_state( state_msg )

    except rospy.ServiceException:
        pass   

def create_origin_pose():
    jop = Pose()
    jop.position.x = 2.6
    jop.position.y = 1.8
    jop.position.z = 0.06
    jop.orientation.x = 0
    jop.orientation.y = 0
    jop.orientation.z = 0.7068252
    jop.orientation.w = 0.7073883
    return jop

def create_cube_marker_from_pt(ptlist, size, id):
    global marker_pub
    marker = Marker()
    marker.header.frame_id = "grasp_frame"
    marker.type = Marker.CUBE_LIST
    marker.ns = "gr"
    marker.header.stamp = rospy.Time.now()
    marker.action = marker.ADD
    marker.id = id
    #marker.scale.x, marker.scale.y, marker.scale.z = 0.04, 0.005, 0.1
    marker.scale.x, marker.scale.y, marker.scale.z = size
    marker.color.r, marker.color.g, marker.color.b, marker.color.a = 20, 50, 100, 1.0
    marker.lifetime = rospy.Duration(100)
    marker.pose.position = Point(x=0,y=0,z=0)
    marker.pose.orientation.w = 1
    #marker.points = [Point(y=0.1,z=-0.1),Point(y=0.1,z=0.1),Point(y=-0.1,z=0.1),Point(y=0.1,z=0.1)]
    marker.points = ptlist
    marker_pub.publish(marker)

def create_arrow_marker_from_pt(ptlist):
    global marker_pub
    marker = Marker()
    marker.header.frame_id = "grasp_frame"
    marker.type = Marker.ARROW
    marker.ns = "gr"
    marker.header.stamp = rospy.Time.now()
    marker.action = marker.ADD
    marker.id = 3
    marker.scale.x, marker.scale.y, marker.scale.z = 0.03, 0.05, 0.05
    marker.color.r, marker.color.g, marker.color.b, marker.color.a = 20, 50, 100, 1.0
    marker.lifetime = rospy.Duration(100)
    #marker.pose.position = pt
    #marker.pose.orientation.w = 1
    marker.points = ptlist
    marker_pub.publish(marker)
    
def create_points_marker_from_pt(ptlist, size, id):
    global marker_pub
    marker = Marker()
    marker.header.frame_id = "base_link"
    marker.type = Marker.POINTS
    marker.ns = "gr"
    marker.header.stamp = rospy.Time.now()
    marker.action = marker.ADD
    marker.id = id
    #marker.scale.x, marker.scale.y, marker.scale.z = 0.04, 0.005, 0.1
    marker.scale.x, marker.scale.y = 0.01, 0.01
    marker.color.r, marker.color.g, marker.color.b, marker.color.a = 20, 50, 100, 1.0
    marker.lifetime = rospy.Duration(100)
    marker.pose.position = Point(x=0,y=0,z=0)
    marker.pose.orientation.w = 1
    #marker.points = [Point(y=0.1,z=-0.1),Point(y=0.1,z=0.1),Point(y=-0.1,z=0.1),Point(y=0.1,z=0.1)]
    marker.points = ptlist
    marker_pub.publish(marker)

def reset_simulation():
    global justina_origin_pose, obj_shape, msg_la, pub_la, pub_hd, msg_hd, pub_object, num_loops, left_gripper_made_contact, right_gripper_made_contact, grasp_attempts
    change_gazebo_object_pose(state_msg, generate_random_pose(), obj_shape)
    change_gazebo_object_pose(state_msg, justina_origin_pose, "justina")
    msg_hd.data = [random.uniform(-0.4,0.4),-random.uniform(1.0, 1.2)]
    num_loops = 0
    left_gripper_made_contact = False
    right_gripper_made_contact = False
    #pub_la.publish(msg_la)
    pub_hd.publish(msg_hd)
    rospy.sleep(0.1)
    #pub_object.publish(obj_shape)
    #pub_object.publish(obj_shape)
    grasp_attempts = 0

def set_gripper_joints(angle):
    global set_gripper_conf
    conf = SetModelConfigurationRequest(
        model_name = 'justina_gripper',
        urdf_param_name = 'gripper_description',
        joint_names = ['gr_la_grip_left', 'gr_la_grip_right'],
        joint_positions = [angle, angle]
    )
    resp = set_gripper_conf(conf)
    return resp

def check_for_contact():
    global obj_shape
    left_gripper_contacts = rospy.wait_for_message('/gr_left_arm_grip_left_sensor' ,ContactsState,5).states
    right_gripper_contacts = rospy.wait_for_message('/gr_left_arm_grip_right_sensor' ,ContactsState,5).states
    left_gripper_made_contact = len(left_gripper_contacts) > 0 and obj_shape in left_gripper_contacts[0].collision1_name
    right_gripper_made_contact = len(right_gripper_contacts) > 0 and obj_shape in right_gripper_contacts[0].collision1_name
    return left_gripper_made_contact and right_gripper_made_contact

def evaluate_grip(gr_pose):
    global set_state
    object_grasped = False
    set_gripper_joints(0.707)
    gr_state = ModelState(
        model_name = 'justina_gripper',
        pose = gr_pose,
        reference_frame = 'base_link'
    )
    set_state(gr_state)
    rospy.sleep(0.01)
    for ds in range(70, -10, -5):
        set_gripper_joints(ds/10)
        #object_grasped = object_grasped or check_for_contact()
    #print(object_grasped)
    object_grasped = check_for_contact()
    set_state(GRIPPER_ORIGIN)
    rospy.sleep(0.01)
    return object_grasped



def main():
    #torch.set_default_dtype(torch.float32)
    #torch.set_default_device(DEVICE)
    global ik_srv, state_msg, grasp_trajectory_found, justina_origin_pose, obj_shape, left_gripper_made_contact, right_gripper_made_contact, grasp_attempts, msg_la, pub_la, pub_hd, msg_hd, pub_object, num_loops
    global set_state, z, marker_pub, set_gripper_conf
    z = 0.74
    state_msg = ModelState()
    deserialized_gripper_model_state = ModelState()
    justina_origin_pose = create_origin_pose()
    msg_la = Float64MultiArray()
    msg_hd = Float64MultiArray()
    msg_la.data = [-1.0, 0.2, 0.0, 1.55, 0.0, 1.24, 0.0]
    msg_hd.data = [0,-1.3]
    obj_shape = '056_tennis_ball'
    rospy.init_node('network_tester')
    print("Starting grip test")
    cltGetObjectPose           = rospy.ServiceProxy("/vision/obj_segmentation/get_obj_pose",  RecognizeObject     ) 
    cltDetectRecogObject       = rospy.ServiceProxy("/vision/obj_reco/detect_and_recognize_object", RecognizeObject     )
    get_object_relative_pose   = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    set_state                  = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    set_gripper_conf           = rospy.ServiceProxy('/gazebo/set_model_configuration', SetModelConfiguration)
    transform_pointcloud       = rospy.ServiceProxy("/vision/point_cloud_to_base_link",PreprocessPointCloud)
    marker_pub                 = rospy.Publisher("/vision/object_recognition/markers", Marker, queue_size = 10)
    graspnet_qry               = rospy.ServiceProxy('/manipulation/grasp/graspnet_request' ,Graspnet)
    pub_hd = rospy.Publisher("/hardware/head/goal_pose", Float64MultiArray, queue_size=10)
    obj_shape = rospy.get_param("/obj","056_tennis_ball")


    rospy.sleep(1)
    loop = rospy.Rate(1)
    #grasp_network = load_model(MODELS_PATH + "dual_fb_jmdn128_dropout_wgp_model_sql_5ks_nwl_dh_tansphere_scratch_ep92.pt")
    #grasp_network.eval()

    #pos_network = load_model(model_path=MODELS_PATH + "dummy_pos_net_1ks_ep50.pt")
    #pos_network.eval()

    #jmdgn = quick_create_test()
    #jmdgn.eval()

    while not rospy.is_shutdown():
        print("Type r to reset sim to a random pose, and l to loop simulation for samples")
        command = input()
        if command == "r": 
            reset_simulation()
            pcd = rospy.wait_for_message("/camera/depth_registered/points", PointCloud2)
            #pcd = transform_pointcloud(PreprocessPointCloudRequest(pcd)).output_cloud
            obj_pt = get_object_relative_pose(obj_shape,"justina::camera_link").pose.position
            mat = gn.ros_pc2_to_npmatrix(pcd)
            #t_pt = obj_pt
            t_pt = gn.camera_link_to_optical_frame(obj_pt)
            u, v, object_in_range = gn.find_nearest_pt_in_pc(mat,t_pt)
            if object_in_range:
                pcd = gn.cut_pc(u,v,mat)
                pcd = gn.npmatrix_to_torch(pcd)
                pcd = pcd.unsqueeze(0).to(DEVICE)
                print(pcd)
                with torch.no_grad():
                    predicted_gripper_center_pose = grasp_network(pcd)
                print(predicted_gripper_center_pose)
                predicted_pose = tensor_to_pose(predicted_gripper_center_pose)
                #predicted_pose.orientation.normalize()
                broadcaster_frame_object("camera_rgb_optical_frame","grasp_frame",predicted_pose)
                #broadcaster_frame_object("base_link","grasp_frame",predicted_pose)
                print(predicted_pose)
            #print("sure")
        if command == "rb":
            reset_simulation()
            pcd = rospy.wait_for_message("/camera/depth_registered/points", PointCloud2)
            pcd = transform_pointcloud(PreprocessPointCloudRequest(pcd)).output_cloud
            obj_pt = get_object_relative_pose(obj_shape,"justina::base_link").pose.position
            mat = gn.ros_pc2_to_npmatrix(pcd)
            t_pt = obj_pt
            #t_pt = gn.c(obj_pt)
            u, v, object_in_range = gn.find_nearest_pt_in_pc(mat,t_pt)
            if object_in_range:
                pcd = gn.cut_pc(u,v,mat)
                pcd = rf.structured_to_unstructured(pcd)
                pcd = rnm.to_multiarray_f32(pcd)
                #pcd = gn.npmatrix_to_torch(pcd)
                #pcd = pcd.unsqueeze(0).to(DEVICE)
                #print(pcd)
                #with torch.no_grad():
                    #pos, ori, x = grasp_network(pcd)
                    #with torch.device(DEVICE):
                    #    pos, ori, x = jmdgn(pcd)
                    #predicted_gripper_center_pose = torch.cat((pos,ori),dim=1)
                predicted_gripper_center_pose = graspnet_qry(pcd).predicted_grasp
                print(predicted_gripper_center_pose)
                #predicted_pose = tensor_to_pose(predicted_gripper_center_pose)
                #predicted_pose.orientation.normalize()
                #broadcaster_frame_object("camera_rgb_optical_frame","grasp_frame",predicted_pose)
                broadcaster_frame_object("base_link","grasp_frame",predicted_gripper_center_pose)
                ptlist = [Point(x=0.04),Point(x=-0.04)]
                create_cube_marker_from_pt(ptlist,[0.005, 0.04, 0.1],1)
                create_cube_marker_from_pt([Point(z=-0.03)],[0.06, 0.03525, 0.03525],2)
                create_arrow_marker_from_pt([Point(z=-0.03),Point(z=-0.03,y=0.1)])
                #print(predicted_gripper_center_pose)

        if command == "s":
            reset_simulation()
            pcd = rospy.wait_for_message("/camera/depth_registered/points", PointCloud2)
            pcd = transform_pointcloud(PreprocessPointCloudRequest(pcd)).output_cloud
            obj_pt = get_object_relative_pose(obj_shape,"justina::base_link").pose.position
            mat = gn.ros_pc2_to_npmatrix(pcd)
            t_pt = obj_pt
            #t_pt = gn.c(obj_pt)
            u, v, object_in_range = gn.find_nearest_pt_in_pc(mat,t_pt)
            if object_in_range:
                pcd = gn.cut_pc(u,v,mat)
                pcd = rf.structured_to_unstructured(pcd)
                pcd = rnm.to_multiarray_f32(pcd)
                predicted_gripper_center_pose = graspnet_qry(pcd).predicted_grasp
                print(predicted_gripper_center_pose)
                broadcaster_frame_object("base_link","grasp_frame",predicted_gripper_center_pose)
                ptlist = [Point(x=0.04),Point(x=-0.04)]
                create_cube_marker_from_pt(ptlist,[0.005, 0.04, 0.1],1)
                create_cube_marker_from_pt([Point(z=-0.03)],[0.06, 0.03525, 0.03525],2)
                create_arrow_marker_from_pt([Point(z=-0.03),Point(z=-0.03,y=0.1)])
                #predicted_gripper_center_pose.orientation = graspnet_ref_to_gripper_ref(predicted_gripper_center_pose.orientation)
                evaluate_grip(predicted_gripper_center_pose)
                #print(predicted_gripper_center_pose)

        if command == "sl":
            attempts = 0
            grasped = 0
            while(attempts < 200):
                reset_simulation()
                pcd = rospy.wait_for_message("/camera/depth_registered/points", PointCloud2)
                pcd = transform_pointcloud(PreprocessPointCloudRequest(pcd)).output_cloud
                obj_pt = get_object_relative_pose(obj_shape,"justina::base_link").pose.position
                mat = gn.ros_pc2_to_npmatrix(pcd)
                t_pt = obj_pt
                #t_pt = gn.c(obj_pt)
                u, v, object_in_range = gn.find_nearest_pt_in_pc(mat,t_pt)
                if object_in_range:
                    pcd = gn.cut_pc(u,v,mat)
                    pcd = rf.structured_to_unstructured(pcd)
                    pcd = rnm.to_multiarray_f32(pcd)
                    predicted_gripper_center_pose = graspnet_qry(pcd).predicted_grasp
                    print(predicted_gripper_center_pose)
                    broadcaster_frame_object("base_link","grasp_frame",predicted_gripper_center_pose)
                    ptlist = [Point(x=0.04),Point(x=-0.04)]
                    create_cube_marker_from_pt(ptlist,[0.005, 0.04, 0.1],1)
                    create_cube_marker_from_pt([Point(z=-0.03)],[0.06, 0.03525, 0.03525],2)
                    create_arrow_marker_from_pt([Point(z=-0.03),Point(z=-0.03,y=0.1)])
                    #predicted_gripper_center_pose.orientation = graspnet_ref_to_gripper_ref(predicted_gripper_center_pose.orientation)
                    if evaluate_grip(predicted_gripper_center_pose):
                        grasped = grasped + 1
                    #print(predicted_gripper_center_pose)
                    attempts = attempts + 1
                    rospy.sleep(0.01)
            print("Grasp Accuracy:" + str((grasped/attempts)*100) + '%')


        if command == "t2":     # test sistema 2
            attempts = 0 
            grasped = 0
            req     = RecognizeObjectsRequest()
            ObjPose = RecognizeObjectRequest()
            while(attempts < 200):
                reset_simulation()
                req.point_cloud = rospy.wait_for_message("/camera/depth_registered/points", PointCloud2)

                try:
                    resp = cltDetectRecogObject(req)
                    
                    ObjPose.point_cloud = resp.recog_object.point_cloud
                    ObjPose.name        = resp.recog_object.id
                    ObjPose = cltGetObjectPose(ObjPose)
                    print("name", ObjPose.name)
                except:
                    print(".....")
                    
                """
                pcd = transform_pointcloud(PreprocessPointCloudRequest(pcd)).output_cloud
                obj_pt = get_object_relative_pose(obj_shape,"justina::base_link").pose.position
                mat = gn.ros_pc2_to_npmatrix(pcd)
                t_pt = obj_pt
                #t_pt = gn.c(obj_pt)
                u, v, object_in_range = gn.find_nearest_pt_in_pc(mat,t_pt)
                if object_in_range:
                    pcd = gn.cut_pc(u,v,mat)
                    pcd = rf.structured_to_unstructured(pcd)
                    pcd = rnm.to_multiarray_f32(pcd)
                    predicted_gripper_center_pose = graspnet_qry(pcd).predicted_grasp
                    print(predicted_gripper_center_pose)
                    broadcaster_frame_object("base_link","grasp_frame",predicted_gripper_center_pose)
                    ptlist = [Point(x=0.04),Point(x=-0.04)]
                    create_cube_marker_from_pt(ptlist,[0.005, 0.04, 0.1],1)
                    create_cube_marker_from_pt([Point(z=-0.03)],[0.06, 0.03525, 0.03525],2)
                    create_arrow_marker_from_pt([Point(z=-0.03),Point(z=-0.03,y=0.1)])
                    #predicted_gripper_center_pose.orientation = graspnet_ref_to_gripper_ref(predicted_gripper_center_pose.orientation)
                    if evaluate_grip(predicted_gripper_center_pose):
                        grasped = grasped + 1
                    #print(predicted_gripper_center_pose)
                    attempts = attempts + 1
                    rospy.sleep(0.01)
            print("Grasp Accuracy:" + str((grasped/attempts)*100) + '%')
            """


        if command == "p":
            reset_simulation()
            ptlist = []
            head = Header()
            pcd = rospy.wait_for_message("/camera/depth_registered/points", PointCloud2)
            pcd = transform_pointcloud(PreprocessPointCloudRequest(pcd)).output_cloud
            obj_pt = get_object_relative_pose(obj_shape,"justina::base_link").pose.position
            mat = gn.ros_pc2_to_npmatrix(pcd)
            t_pt = obj_pt
            #t_pt = gn.c(obj_pt)
            u, v, object_in_range = gn.find_nearest_pt_in_pc(mat,t_pt)
            if object_in_range:
                pcd = gn.cut_pc(u,v,mat)
                pcd = gn.npmatrix_to_torch(pcd)
                pcd = pcd.unsqueeze(0).to(DEVICE)
                #print(pcd)
                with torch.no_grad():
                    for i in range(100):
                        _, pos  = pos_network(pcd)
                        ptlist.append(tensor_to_point(pos))
                create_points_marker_from_pt(ptlist,[0.04, 0.005, 0.1],5)
                rospy.sleep(10)

        if command == "jp":
            reset_simulation()
            ptlist = []
            head = Header()
            pcd = rospy.wait_for_message("/camera/depth_registered/points", PointCloud2)
            pcd = transform_pointcloud(PreprocessPointCloudRequest(pcd)).output_cloud
            obj_pt = get_object_relative_pose(obj_shape,"justina::base_link").pose.position
            mat = gn.ros_pc2_to_npmatrix(pcd)
            t_pt = obj_pt
            #t_pt = gn.c(obj_pt)
            u, v, object_in_range = gn.find_nearest_pt_in_pc(mat,t_pt)
            if object_in_range:
                pcd = gn.cut_pc(u,v,mat)
                pcd = gn.npmatrix_to_torch(pcd)
                pcd = pcd.unsqueeze(0).to(DEVICE)
                #print(pcd)
                with torch.no_grad():
                    for i in range(100):
                        pos, ori, x  = jmdgn(pcd)
                        ptlist.append(tensor_to_point(pos))
                create_points_marker_from_pt(ptlist,[0.04, 0.005, 0.1],5)
                rospy.sleep(10)
                # print(predicted_gripper_center_pose)
                # predicted_pose = tensor_to_pose(predicted_gripper_center_pose)
                # #predicted_pose.orientation.normalize()
                # #broadcaster_frame_object("camera_rgb_optical_frame","grasp_frame",predicted_pose)
                # broadcaster_frame_object("base_link","grasp_frame",predicted_pose)
                # ptlist = [Point(x=0.04),Point(x=-0.04)]
                # create_cube_marker_from_pt(ptlist,[0.005, 0.04, 0.1],1)
                # create_cube_marker_from_pt([Point(z=-0.03)],[0.06, 0.03525, 0.03525],2)
                # create_arrow_marker_from_pt([Point(z=-0.03),Point(z=-0.03,y=0.1)])
                # print(predicted_pose)

        loop.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass