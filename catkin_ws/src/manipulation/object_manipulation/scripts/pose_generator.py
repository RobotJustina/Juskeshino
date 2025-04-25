#!/usr/bin/env python3

import rospy 
import rospkg 
import random
import math
import os
import matplotlib.pyplot as plt
import tf.transformations as tft
import numpy as np
import vg
import tf
import tf2_ros
import h5py
import torch
import geometry_msgs
import geomstats.backend as gs
from geomstats.geometry.hypersphere import Hypersphere, HypersphereMetric
from gazebo_msgs.msg import ModelState, ContactsState 
from gazebo_msgs.srv import SetModelState, GetModelState
from std_msgs.msg import String, Float64MultiArray, Header
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Pose, Quaternion, Point, PoseStamped
from manip_msgs.srv import DataCapture, InverseKinematicsPose2TrajRequest, InverseKinematicsPose2Traj
from vision_msgs.srv import PreprocessPointCloud, PreprocessPointCloudRequest
from visualization_msgs.msg import Marker

from dataset_utils import save_data_to_file, find_nearest_pt_in_pc, camera_link_to_optical_frame, ros_pc2_to_npmatrix, save_pcd_to_db, save_grasp_to_db, cut_pc
BASE_JUSTINA_VECTOR = np.array([0.0,-1.0,0.0])
MANIF = Hypersphere(3)
GRASP_TO_PCD_RATIO = 10
CONIC_ANGLE = math.cos(math.radians(70))
VG_PLANE = {
    "XY": vg.basis.z,
    "YZ": vg.basis.x,
    "ZX": vg.basis.y
}
BG_PATH = "/home/robocup/billion_grasps/21_ycb_object_grasps/"

def get_Z_obj():
    global obj_shape
    z_obj_dic = {             # z1  , z2
        '001_chips_can':        [0.740, 0.800, 0], 
        '002_master_chef_can':  [0.751, 0.770, 0],
        '003_cracker_box':      [0.780, 0.805, 0], 
        '004_sugar_box':        [0.749, 0.790, 0],
        '005_tomato_soup_can':  [0.733, 0.758, 0],
        '006_mustard_bottle':   [0.729, 0.783, 0],
        '007_tuna_fish_can':    [0.743, 0.717, 0],
        '008_pudding_box':      [0.746, 0.754, 0.717],
        '009_gelatin_box':      [0.737, 0.746, 0.714],
        '010_potted_meat_can':  [0.727, 0.740, 0],
        '011_banana':           [0.717, 0.717, 0], 
        '019_pitcher_base':     [0.835, 0.835, 0],
        '021_bleach_cleanser':  [0.728, 0.804, 0],
        '024_bowl':             [0.724, 0.724, 0],
        '025_mug':              [0.736, 0.736, 0],
        '035_power_drill':      [0.818, 0.728, 0],
        '036_wood_block':       [0.745, 0.806, 0],
        '037_scissors':         [0.708, 0.708, 0],
        '040_large_marker':     [0.709, 0.709, 0],
        '048_hammer':           [0.716, 0.716, 0],
        '051_large_clamp':      [0.718, 0.718, 0],
        '052_extra_large_clamp':[0.716, 0.716, 0],
        '056_tennis_ball':      [0.732, 0.732, 0],
        '061_foam_brick':       [0.738, 0.725, 0],
        '077_rubiks_cube':      [0.728, 0.728, 0]
    }
    z1 = z_obj_dic[obj_shape][0]
    z2 = z_obj_dic[obj_shape][1]
    z3 = z_obj_dic[obj_shape][2]
    return z1, z2, z3



def categorize_objs(name):
    drill     = ['035_power_drill']
    dishes    = ['024_bowl', '019_pitcher_base', '025_mug', ]
    prismatic = ['001_chips_can', '007_tuna_fish_can', '002_master_chef_can', '005_tomato_soup_can']
    spherical = ['054_softball', '055_baseball', '056_tennis_ball']
    flat      = ['006_mustard_bottle', '021_bleach_cleanser', '035_power_drill', ]
    box       = ['003_cracker_box', '004_sugar_box', '036_wood_block', '061_foam_brick']
    small_box = ['pudding_box',  '009_gelatin_box']
    cubic     = ['077_rubiks_cube']
    two_faces = ['011_banana', '048_hammer', '044_flat_screwdriver', '037_scissors', '040_large_marker', '051_large_clamp', '052_extra_large_clamp']
    if   name in dishes:    return 'dishes'
    elif name in prismatic: return 'prismatic'
    elif name in spherical: return 'spherical'
    elif name in flat:      return "flat"
    elif name in box:       return 'box'
    elif name in cubic:     return 'cubic'
    elif name in two_faces: return '2faces'
    elif name in small_box: return 'small_box'
    elif name in small_box: return 'drill'

    

def rotation_object():
    global obj_shape
    geometric_shape_dic = {
                            "drill":    [[0, 0, np.deg2rad(random.randint(0, int(359)))], [0, 1.57, np.deg2rad(random.randint(0, int(359)))]],
                            "dishes":   [[0, 0, np.deg2rad(random.randint(0, int(359)))]],
                            "prismatic":[[0, 0, np.deg2rad(random.randint(0, int(359)))], [0, 1.57, np.deg2rad(random.randint(0, int(359)))] ],
                            "spherical":[[np.deg2rad(random.randint(0, int(359))) , np.deg2rad(random.randint(0, int(359))) ,np.deg2rad(random.randint(0, int(359)))]],
                            "flat":     [[0, 0, np.deg2rad(random.randint(0, int(359)))], [0, 1.57, np.deg2rad(random.randint(0, int(359)))] ,  [0, -1.57, np.deg2rad(random.randint(0, int(359)))] ],
                            "cubic":    [ [0, 1.57 , np.deg2rad(random.randint(0, int(359)))] ,  [0, 3.14, np.deg2rad(random.randint(0, int(359)))],  [0, 4.71, np.deg2rad(random.randint(0, int(359)))],  [0, 6.28, np.deg2rad(random.randint(0, int(359)))],
                                        [1.57, 0 , np.deg2rad(random.randint(0, int(359)))] ,  [3.14, 0,  np.deg2rad(random.randint(0, int(359)))], [4.71, 0, np.deg2rad(random.randint(0, int(359)))],  [6.28, 0, np.deg2rad(random.randint(0, int(359)))]],
                            "box":      [[0, 0, np.deg2rad(random.randint(0, int(359)))],  [3.14, 0,  np.deg2rad(random.randint(0, int(359)))],
                                        [1.57, 0 , np.deg2rad(random.randint(0, int(359)))] , [4.71, 0, np.deg2rad(random.randint(0, int(359)))]],
                            "small_box":[[0, 0, np.deg2rad(random.randint(0, int(359)))], 
                                         [1.57, 0 , np.deg2rad(random.randint(0, int(359)))],
                                         [0, 1.57 , np.deg2rad(random.randint(0, int(359)))],
                                         ],
                            "2faces":   [[0, 0, np.deg2rad(random.randint(0, int(359)))] ,  [0, 3.14, np.deg2rad(random.randint(0, int(359)))]]
    }
    rotation = random.choice(geometric_shape_dic[categorize_objs(obj_shape)])
    z1, z2 , z3 = get_Z_obj()
    
    if obj_shape == "small_box":
        quaternion_obj = tft.quaternion_from_euler(rotation[0],rotation[1],rotation[2] ,'sxyz')
        if (rotation[0]  == 0) and (rotation[1] == 0):
            return z3
        if (rotation[0]  == 0) and (rotation[1] != 0):
            return z2
        if (rotation[1]  == 0) and (rotation[0] != 0):
            return z1



    if(((int(np.rad2deg(rotation[0])) == 0) or ((int(np.rad2deg(rotation[0])) > 175) and  (int(np.rad2deg(rotation[0])) < 185)))  and (int(np.rad2deg(rotation[1])) == 0)): z = z2
    else:z = z1
    quaternion_obj = tft.quaternion_from_euler(rotation[0],rotation[1],rotation[2] ,'sxyz')
    return quaternion_obj, z


def generate_random_pose():
    global z
    rpose = Pose()
    rpose.position.x = random.randint(210,310)/100
    rpose.position.y = random.randint(210,245)/100
    q, z = rotation_object()
    rpose.position.z = z
    rpose.orientation.x = q[0]
    rpose.orientation.y = q[1]
    rpose.orientation.z = q[2]
    rpose.orientation.w = q[3]
    return rpose


def generate_random_points(num):
    ptlist = [Point(x=random.uniform(-0.2,0.2),y=random.uniform(-0.2,0.2),z=random.uniform(-0.2,0.2)) for i in range(num)]
    return ptlist
    
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

def get_new_pcd():
    global pub_hd, msg_hd
    msg_hd.data = [random.uniform(-0.4,0.4),-random.uniform(1.0, 1.2)]
    #msg_hd.data = [0,-1.3]
    pub_hd.publish(msg_hd)
    #rospy.sleep(0.005)
    pcd = rospy.wait_for_message("/camera/depth_registered/points", PointCloud2)
    return msg_hd.data, pcd

def callback_grasp_status(msg):
    global grasp_trajectory_found, grasp_attempts
    if msg.data == "SUCCESS":
        grasp_trajectory_found = True
    else:
        grasp_trajectory_found = False
        grasp_attempts += 1
        #reset_simulation()

def callback_left_grip_sensor(msg):
    global left_gripper_made_contact, obj
    if len(msg.states) > 0 and obj in msg.states[0].collision1_name:
        left_gripper_made_contact = True
        
def callback_right_grip_sensor(msg):
    global right_gripper_made_contact, obj
    if len(msg.states) > 0 and obj in msg.states[0].collision1_name:
        right_gripper_made_contact = True

def create_origin_pose():
    jop = Pose()
    jop.position.x = 2.6
    jop.position.y = 1.8
    jop.position.z = 0.0
    jop.orientation.x = 0
    jop.orientation.y = 0
    jop.orientation.z = 0.7068252
    jop.orientation.w = 0.7073883
    return jop
    
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
    
def show_graph(V1,V2,W1,W2):
    fig, ax = plt.subplots(figsize=(10, 10))
    ax.quiver(0, 0, V1, V2, angles='xy', scale_units='xy', scale=1, color='r')
    ax.quiver(0, 0, W1, W2, angles='xy', scale_units='xy', scale=1, color='b')
    ax.set_xlim([-3, 3])
    ax.set_ylim([-3, 3])
    plt.grid()
    plt.show()

def get_angle_in_plane(gr_point, obj_point, plane="XY",show_plot=False):
    gr_vector = np.array([gr_point.x - obj_point.x, gr_point.y - obj_point.y, gr_point.z - obj_point.z])
    gr_vector = gr_vector/np.linalg.norm(gr_vector)
    angle_XY = vg.angle(gr_vector, BASE_JUSTINA_VECTOR, look=vg.basis.z) #Symmetric, sign does not matter
    angle_YZ = vg.signed_angle(gr_vector, BASE_JUSTINA_VECTOR, look=vg.basis.x)
    angle_ZX = vg.signed_angle(gr_vector, np.array([1.0,0.0,0.0]), look=vg.basis.y)
    gripper_side = gr_vector > 0 #If true gripper side = right, else its left
    #print(angle_XY, angle_YZ, angle_ZX)
    if show_plot:
        if plane == "XY":
            show_graph(gr_vector[0], gr_vector[1],BASE_JUSTINA_VECTOR[0], BASE_JUSTINA_VECTOR[1])
        if plane == "YZ":
            show_graph(gr_vector[1], gr_vector[2],BASE_JUSTINA_VECTOR[0], BASE_JUSTINA_VECTOR[1])
        if plane == "ZX":
            show_graph(gr_vector[0], gr_vector[2],1.0, 0.0)
    return angle_XY, angle_YZ, angle_ZX, gripper_side

def check_for_contact():
    left_gripper_made_contact = len(rospy.wait_for_message('/gr_left_arm_grip_left_sensor' ,ContactsState,5).states) > 0
    right_gripper_made_contact = len(rospy.wait_for_message('/gr_left_arm_grip_right_sensor' ,ContactsState,5).states) > 0
    return left_gripper_made_contact or right_gripper_made_contact

def is_pose_valid(angle_XY, angle_YZ, angle_ZX):
    if check_for_contact(): return False
    if angle_XY >= 90: return False
    if not (10 <= angle_YZ <= 90): return False
    if not (10 <= angle_ZX <= 170): return False

    return True 

def get_orientation_in_range(q):
    ql = [q.x,q.y,q.z,q.w]
    rot = tft.quaternion_matrix(ql)
    #print(rot)
    x = rot[:3,0]
    #print(x)
    in_range_z = np.dot(x,np.array([0,0,-1])) > 0
    in_range_y = np.dot(x,np.array([0,-1,0])) > 0
    #print(in_range_y,in_range_z)
    if in_range_z or in_range_y:
        rot[:3,:2] = -rot[:3,:2]
    #print(rot)
    qf = tft.quaternion_from_matrix(rot)
    qr = Quaternion()
    qr.x = qf[0]
    qr.y = qf[1]
    qr.z = qf[2]
    qr.w = qf[3]
    return qr

def get_antipodal_in_range(q):
    ql = [q.x,q.y,q.z,q.w]
    rot = tft.quaternion_matrix(ql)
    #print(rot)
    y = rot[:3,1]
    #print(x)
    #in_range_z = np.dot(y,np.array([0,0,-1])) > 0
    #in_range_y = np.dot(x,np.array([0,-1,0])) > 0
    #print(in_range_y,in_range_z)
    if y[2] < 0:
        rot[:3,:2] = -rot[:3,:2]
    #print(rot)
    qf = tft.quaternion_from_matrix(rot)
    qr = Quaternion()
    qr.x = qf[0]
    qr.y = qf[1]
    qr.z = qf[2]
    qr.w = qf[3]
    return qr

def quaternion_in_manifold(q):
    s = torch.tensor([q.x,q.y,q.z,q.w],dtype=torch.float64)
    return MANIF.belongs(s).item()

def get_quaternion_in_hemihypersphere_dot(q):
    qo = np.array([q.x,q.y,q.z,q.w])
    if np.dot(qo,np.array([0,0,0,-1])) < 0:
        #print("Positive quaternion :)")
        return q
    else:
        qo = qo * -1
        qr = Quaternion()
        qr.x = qo[0]
        qr.y = qo[1]
        qr.z = qo[2]
        qr.w = qo[3]
        #print("Rotating quaternion :O")
        return qr

def get_quaternion_in_hemihypersphere(q):
    if q.w >= 0:
        #print("Positive quaternion :)")
        return q
    else:
        #qo = qo * -1
        qr = Quaternion()
        qr.x = -q.x
        qr.y = -q.y
        qr.z = -q.z
        qr.w = -q.w
        #print("Rotating quaternion :O")
        return qr

def pose_to_nparray(ps):
    return np.array([ps.position.x,ps.position.y,ps.position.z,ps.orientation.x,ps.orientation.y,ps.orientation.z,ps.orientation.w])

def score_calculation(articular_array):
    a = sum(articular_array)

def gripper_in_conic(gp,ob):
    g = np.asarray([gp.x,gp.y,gp.z])
    o = np.asarray([ob.x,ob.y,ob.z])
    axis = o/np.linalg.norm(o)
    #print(np.dot(((g-o)/np.linalg.norm(g-o)),axis))
    in_conic = np.dot(((g-o)/np.linalg.norm(g-o)),axis) < CONIC_ANGLE
    #print("In conic:", in_conic)
    return in_conic

def grasp_in_ob_origin_conic(gp,cam):
    g = np.asarray([gp.x,gp.y,gp.z])
    cam = np.asarray([cam.x,cam.y,cam.z])
    axis = cam/np.linalg.norm(cam)
    #print(np.dot(((g-o)/np.linalg.norm(g-o)),axis))
    in_conic = np.dot(((g)/np.linalg.norm(g)),axis) > CONIC_ANGLE
    #print("In conic:", in_conic)
    return in_conic

def get_vision_angle(gp,cam):
    objgpr = np.asarray([gp.x,gp.y,gp.z])
    objgpr = objgpr/np.linalg.norm(objgpr)
    objcam = np.asarray([-cam.x,-cam.y,-cam.z])
    objcam = objcam/np.linalg.norm(objcam)
    vis_ang = vg.angle(objcam,objgpr)
    return vis_ang

def get_ik_la(msg_pose):
    global ik_srv
    roll,pitch,yaw = tft.euler_from_quaternion( [msg_pose.orientation.x , msg_pose.orientation.y , 
                                                 msg_pose.orientation.z , msg_pose.orientation.w ])
    ik_msg = InverseKinematicsPose2TrajRequest()
    ik_msg.x         = msg_pose.position.x
    ik_msg.y         = msg_pose.position.y
    ik_msg.z         = msg_pose.position.z
    ik_msg.roll      = roll    
    ik_msg.pitch     = pitch
    ik_msg.yaw       = yaw
    ik_msg.duration  = 0
    ik_msg.time_step = 0.05
    try:
        resp_ik_srv = ik_srv(ik_msg)    # Envia al servicio de IK
        print("Approved pose")
        return resp_ik_srv.articular_trajectory
    except:
        print("Could not find IK")

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

def main():
    global ik_srv, state_msg, grasp_trajectory_found, justina_origin_pose, obj_shape, left_gripper_made_contact, right_gripper_made_contact, grasp_attempts, msg_la, pub_la, pub_hd, msg_hd, pub_object, num_loops
    global set_state, get_object_relative_pose, z, marker_pub, tf_listener
    state_msg = ModelState()
    deserialized_gripper_model_state = ModelState()
    justina_origin_pose = create_origin_pose()
    msg_la = Float64MultiArray()
    msg_hd = Float64MultiArray()
    msg_la.data = [-1.0, 0.2, 0.0, 1.55, 0.0, 1.24, 0.0]
    msg_hd.data = [0,-1.3]
    msg_capture = String()
    grasp_trajectory_found = False
    left_gripper_made_contact = False
    right_gripper_made_contact = False
    grasp_attempts = 0
    num_loops = 0
    found_grasps = 0
    obj_shape = '001_chips_can'
    rospy.init_node('dataset_generator')
    print("Starting grip test")
    # rospy.Subscriber('/manipulation/grasp/grasp_status' ,String ,callback_grasp_status)
    # rospy.Subscriber('/gr_left_arm_grip_left_sensor' ,ContactsState ,callback_left_grip_sensor)
    # rospy.Subscriber('/gr_left_arm_grip_right_sensor' ,ContactsState ,callback_right_grip_sensor)
    rospy.wait_for_service('/manipulation/grasp/data_capture_service')
    ik_srv           = rospy.ServiceProxy( '/manipulation/la_ik_trajectory' , InverseKinematicsPose2Traj )
    get_object_relative_pose = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    capture = rospy.ServiceProxy('/manipulation/grasp/data_capture_service', DataCapture)
    set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    transform_pointcloud = rospy.ServiceProxy("/vision/point_cloud_to_base_link",PreprocessPointCloud)
    #pub_object = rospy.Publisher("/plannning/simple_task/take_object", String, queue_size=10)
    pub_la = rospy.Publisher("/hardware/left_arm/goal_pose", Float64MultiArray, queue_size=10)
    pub_hd = rospy.Publisher("/hardware/head/goal_pose", Float64MultiArray, queue_size=10)
    obj_shape = rospy.get_param("/obj","056_tennis_ball")
    sample_start = rospy.get_param("/sample_start",0)
    sample_stop = rospy.get_param("/sample_stop",100)
    marker_pub = rospy.Publisher("/vision/object_recognition/markers", Marker, queue_size = 10)

    tf_buf = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buf)

    print(sample_start,sample_stop)
    rospy.sleep(1)

    msg = ModelState()
    msg.reference_frame
    msg.model_name = obj_shape
    z = 0.74
    initial_pose = Pose()
    initial_pose.position.x = random.randint(210,310)/100
    initial_pose.position.y = random.randint(218,245)/100
    initial_pose.position.z = z
    initial_pose.orientation.x = 0
    initial_pose.orientation.y = 0
    initial_pose.orientation.z = 0
    initial_pose.orientation.w = 1
    msg.pose = initial_pose
    set_state(msg)
    z = get_object_relative_pose(obj_shape,"world").pose.position.z

    reset_simulation()
    pub_hd.publish(msg_hd)
    #POSE_DATA_PATH = "./catkin_ws/src/manipulation/object_manipulation/pose_data/"
    #objmanpkg_path = rospkg.get_ros_package_path()
    #print(objmanpkg_path)
    rospack = rospkg.RosPack()
    objmanpkg_path = rospack.get_path('object_manipulation')
    print(objmanpkg_path)
    POSE_DATA_PATH = objmanpkg_path + "/pose_data/"
    pose_num = 0
    loop = rospy.Rate(1)
    space = Hypersphere(3)

    command = rospy.get_param('/cmd',"default")

    while not rospy.is_shutdown():
        print("Type r to reset sim to a random pose, and l to loop simulation for samples")
        if command == "default": 
            command = input()
            rospy.set_param('/cmd',command)
        if command == "r": 
            #obj_pose = get_object_relative_pose("justina_gripper::left_arm_grip_center",obj_shape).pose
            reset_simulation()
            pose_num = 1
            file_name = POSE_DATA_PATH + obj_shape + str(pose_num)
            in_file = open(file_name, "rb") # opening for [r]eading as [b]inary
            file_serialized_gripper_model_state = in_file.read() # if you only wanted to read 512 bytes, do .read(512)
            in_file.close()
            deserialized_gripper_model_state.deserialize(file_serialized_gripper_model_state)
            deserialized_gripper_model_state.reference_frame = '056_tennis_ball'
            #print(deserialized_gripper_model_state)
            set_state(deserialized_gripper_model_state)
        if command == "p":
            gr_pose_relative_to_base_link = get_object_relative_pose("justina_gripper","justina::left_arm_link7").pose
            roll,pitch,yaw = tft.euler_from_quaternion([gr_pose_relative_to_base_link.orientation.x,
                                                        gr_pose_relative_to_base_link.orientation.y,
                                                        gr_pose_relative_to_base_link.orientation.z,
                                                        gr_pose_relative_to_base_link.orientation.w])
            print(math.degrees(roll),math.degrees(pitch),math.degrees(yaw))
        if command =="a":
            angle_XY, angle_YZ, angle_ZX, gripper_side = get_angle_in_plane(get_object_relative_pose("justina_gripper","world").pose.position, 
                                                              get_object_relative_pose(obj_shape,"world").pose.position,"XY")
            print(is_pose_valid(angle_XY, angle_YZ, angle_ZX,gripper_side))
        if command == 'f':
            pose_num = 0
            while(pose_num<34):
                pose_num = pose_num + 1
                file_name = POSE_DATA_PATH + obj_shape + str(pose_num)
                in_file = open(file_name, "rb") # opening for [r]eading as [b]inary
                file_serialized_gripper_model_state = in_file.read() # if you only wanted to read 512 bytes, do .read(512)
                in_file.close()
                deserialized_gripper_model_state.deserialize(file_serialized_gripper_model_state)
                deserialized_gripper_model_state.reference_frame = obj_shape
                set_state(deserialized_gripper_model_state)
                rospy.sleep(0.75)
                angle_XY, angle_YZ, angle_ZX = get_angle_in_plane(get_object_relative_pose("justina_gripper","world").pose.position, 
                                                              get_object_relative_pose(obj_shape,"world").pose.position,"XY")
                print(is_pose_valid(angle_XY, angle_YZ, angle_ZX))
        if command == 'l':            
            print("Start from sample number:")
            found_grasps = int(input())
            print("How many samples to take?")
            desired_samples = int(input()) + found_grasps + 1
            while((not rospy.is_shutdown()) and found_grasps < desired_samples):
                reset_simulation()
                pose_num = 1
                print("Testing new position")
                file_name = POSE_DATA_PATH + obj_shape + str(pose_num)
                while(os.path.exists(file_name) and found_grasps < desired_samples):
                    in_file = open(file_name, "rb") # opening for [r]eading as [b]inary
                    file_serialized_gripper_model_state = in_file.read() 
                    in_file.close()
                    deserialized_gripper_model_state.deserialize(file_serialized_gripper_model_state)
                    deserialized_gripper_model_state.reference_frame = obj_shape
                    set_state(deserialized_gripper_model_state)
                    rospy.sleep(0.001)
                    angle_XY, angle_YZ, angle_ZX, gripper_side = get_angle_in_plane(get_object_relative_pose("justina_gripper","world").pose.position, 
                                                                get_object_relative_pose(obj_shape,"world").pose.position,"XY")
                    if is_pose_valid(angle_XY, angle_YZ, angle_ZX) and found_grasps < desired_samples: 
                        data = capture("Found grasp")
                        found_grasps = found_grasps + save_data_to_file(data,found_grasps)
                        print("Found Grasp: ",found_grasps)
                        for i in range(3):
                            if found_grasps < desired_samples:
                                new_hd, new_pcd = get_new_pcd()
                                new_pcd = transform_pointcloud(PreprocessPointCloudRequest(new_pcd)).output_cloud
                                data.head_pose_q = new_hd
                                data.pointcloud = new_pcd
                                found_grasps = found_grasps + save_data_to_file(data,found_grasps)
                                print("Found Grasp: ",found_grasps)
                            else: break
                    pose_num = pose_num + 1
                    file_name = POSE_DATA_PATH + obj_shape + str(pose_num)
            print("Finished taking samples")
        if command == 'c':
            while((not rospy.is_shutdown())):
                reset_simulation()
                pose_num = 1
                print("Testing new position")
                file_name = POSE_DATA_PATH + obj_shape + str(pose_num)
                angles = []
                while(os.path.exists(file_name)):
                    angles.append(256)
                    in_file = open(file_name, "rb") # opening for [r]eading as [b]inary
                    file_serialized_gripper_model_state = in_file.read() 
                    in_file.close()
                    deserialized_gripper_model_state.deserialize(file_serialized_gripper_model_state)
                    deserialized_gripper_model_state.reference_frame = obj_shape
                    set_state(deserialized_gripper_model_state)
                    rospy.sleep(0.001)
                    angle_XY, angle_YZ, angle_ZX, gripper_side = get_angle_in_plane(get_object_relative_pose("justina_gripper","world").pose.position, 
                                                                get_object_relative_pose(obj_shape,"world").pose.position,"XY")
                    gpwrtcam = get_object_relative_pose("justina_gripper","justina::camera_link").pose.position
                    objwrtcam = get_object_relative_pose(obj_shape,"justina::camera_link").pose.position
                    if is_pose_valid(angle_XY, angle_YZ, angle_ZX) and gripper_in_conic(gpwrtcam,objwrtcam):
                        angles[pose_num-1] = get_vision_angle(deserialized_gripper_model_state.pose.position,objwrtcam)
                        print(pose_num,angles[pose_num-1])
                        rospy.sleep(2)
                    pose_num = pose_num + 1
                    file_name = POSE_DATA_PATH + obj_shape + str(pose_num)
                print(angles)
                best_pose = angles.index(min(angles)) + 1
                print(best_pose)
                if angles[best_pose-1] != 256:
                    file_name = POSE_DATA_PATH + obj_shape + str(best_pose)
                    in_file = open(file_name, "rb") # opening for [r]eading as [b]inary
                    file_serialized_gripper_model_state = in_file.read() 
                    in_file.close()
                    deserialized_gripper_model_state.deserialize(file_serialized_gripper_model_state)
                    deserialized_gripper_model_state.reference_frame = obj_shape
                    set_state(deserialized_gripper_model_state)
                    rospy.sleep(5)
            print("Finished taking samples")

        if command == 't':
            pose_num = pose_num + 1
            file_name = POSE_DATA_PATH + obj_shape + str(pose_num)
            in_file = open(file_name, "rb") # opening for [r]eading as [b]inary
            file_serialized_gripper_model_state = in_file.read() 
            in_file.close()
            deserialized_gripper_model_state.deserialize(file_serialized_gripper_model_state)
            deserialized_gripper_model_state.reference_frame = obj_shape
            set_state(deserialized_gripper_model_state)
            print(get_object_relative_pose("justina_gripper",'world'))
            uia = input()
            gpos = get_object_relative_pose("justina_gripper",'world').pose
            quat = get_orientation_in_range(gpos.orientation)
            print(quat)
            deserialized_gripper_model_state.pose.orientation = quat
            deserialized_gripper_model_state.pose.position = gpos.position
            deserialized_gripper_model_state.reference_frame = "world"
            set_state(deserialized_gripper_model_state)
            print(get_object_relative_pose("justina_gripper","world"))
            uia = input()

        if command == 'y':            
            #print("Start from sample number:")
            found_grasps = sample_start
            #print("Stop until sample:")
            desired_samples = sample_stop + 1
            rospy.sleep(2)
            while((not rospy.is_shutdown()) and found_grasps < desired_samples):
                reset_simulation()
                pose_num = 1
                print("Testing new position")
                file_name = POSE_DATA_PATH + obj_shape + str(pose_num)
                angles = []
                tpcd = rospy.wait_for_message("/camera/depth_registered/points", PointCloud2)
                gpwrtcam = get_object_relative_pose(obj_shape,"justina::camera_link").pose.position
                _,_, in_frame = find_nearest_pt_in_pc(ros_pc2_to_npmatrix(tpcd),camera_link_to_optical_frame(gpwrtcam))
                if not in_frame: continue
                while(os.path.exists(file_name)):
                    angles.append(256)
                    in_file = open(file_name, "rb") # opening for [r]eading as [b]inary
                    file_serialized_gripper_model_state = in_file.read() 
                    in_file.close()
                    deserialized_gripper_model_state.deserialize(file_serialized_gripper_model_state)
                    deserialized_gripper_model_state.reference_frame = obj_shape
                    set_state(deserialized_gripper_model_state)
                    rospy.sleep(0.001)
                    angle_XY, angle_YZ, angle_ZX, gripper_side = get_angle_in_plane(get_object_relative_pose("justina_gripper","world").pose.position, 
                                                                get_object_relative_pose(obj_shape,"world").pose.position,"XY")
                    gpwrtcam = get_object_relative_pose("justina_gripper","justina::camera_link").pose.position
                    objwrtcam = get_object_relative_pose(obj_shape,"justina::camera_link").pose.position
                    if is_pose_valid(angle_XY, angle_YZ, angle_ZX) and gripper_in_conic(gpwrtcam,objwrtcam):
                        angles[pose_num-1] = get_vision_angle(deserialized_gripper_model_state.pose.position,objwrtcam)
                        #print(pose_num,angles[pose_num-1])
                        #rospy.sleep(2)
                    pose_num = pose_num + 1
                    file_name = POSE_DATA_PATH + obj_shape + str(pose_num)
                #print(angles)
                best_pose = angles.index(min(angles)) + 1
                print(best_pose)
                if angles[best_pose-1] != 256:
                    file_name = POSE_DATA_PATH + obj_shape + str(best_pose)
                    in_file = open(file_name, "rb") # opening for [r]eading as [b]inary
                    file_serialized_gripper_model_state = in_file.read() 
                    in_file.close()
                    deserialized_gripper_model_state.deserialize(file_serialized_gripper_model_state)
                    deserialized_gripper_model_state.reference_frame = obj_shape
                    set_state(deserialized_gripper_model_state)
                    gpos = get_object_relative_pose("justina_gripper",'world').pose
                    quat = get_orientation_in_range(gpos.orientation)
                    #print(quat)
                    #quat = get_quaternion_in_hemihypersphere(quat)
                    #print(quat)
                    deserialized_gripper_model_state.pose.orientation = quat
                    deserialized_gripper_model_state.pose.position = gpos.position
                    deserialized_gripper_model_state.reference_frame = "world"
                    set_state(deserialized_gripper_model_state)
                    data = capture("Found grasp")
                    found_grasps = found_grasps + save_data_to_file(data,found_grasps)
                    rospy.set_param('/sample_start',found_grasps)
            print("Finished taking samples")
            rospy.set_param('/cmd',"default")
            rospy.signal_shutdown('Finished taking samples')
        if command == 'e':
 
            while((not rospy.is_shutdown())):
                msg.pose = generate_random_pose()
                set_state(msg)
                get_object_relative_pose(obj_shape,"world").pose.position

                rospy.sleep(4)
        if command == 'h':
            show_rviz = True
            reset_simulation()
            FILE_PATH = BG_PATH + obj_shape + "/grasps.h5"
            f = h5py.File(FILE_PATH,'r')
            index = np.random.choice(len(f['poses']),500, replace=False)
            grasp = f['poses'][:]
            grasp = grasp[index]
            head = Header(frame_id='object_frame')
            og_pose = PoseStamped()
            og_pose.header.frame_id = "camera_rgb_optical_frame"
            og_pose.pose.orientation.w = 1
            objwrtcam = tf_buf.transform(og_pose, "object_frame").pose.position
            pose_list = np.array([PoseStamped(header=head, pose=Pose(position=Point(x=g[0],y=g[1],z=g[2]),orientation=Quaternion(x=g[3],y=g[4],z=g[5], w=g[6]))) for g in grasp])
            print(len(pose_list))
            pose_list = pose_list[[grasp_in_ob_origin_conic(ps.pose.position,objwrtcam) for ps in pose_list]]
            print(len(pose_list))
            target_pose = [tf_buf.transform(pose, "base_link") for pose in pose_list]
            if show_rviz:
                ptlist = np.array([tpose.pose.position for tpose in target_pose])
                ptlist = ptlist[[pt.z > 0.78 for pt in ptlist]]
                #ptlist = np.array(generate_random_points(5000)) ##To visualize first change the create_points_marker frame to object frame
                #ptlist = ptlist[[grasp_in_ob_origin_conic(pt,objwrtcam) for pt in ptlist]]
                create_points_marker_from_pt(ptlist,[0.04, 0.005, 0.1],5)
                # predicted_pose = target_pose[0].pose
                # ptlist = [Point(x=0.04),Point(x=-0.04)]
                # broadcaster_frame_object("base_link","grasp_frame",predicted_pose)
                # create_cube_marker_from_pt(ptlist,[0.005, 0.04, 0.1],1)
                # create_cube_marker_from_pt([Point(z=-0.03)],[0.06, 0.03525, 0.03525],2)
                # create_arrow_marker_from_pt([Point(z=-0.03),Point(z=-0.03,y=0.1)])
            rospy.sleep(10)

        if command == 'u':
            show_rviz = True
            FILE_PATH = BG_PATH + obj_shape + "/grasps.h5"
            f = h5py.File(FILE_PATH,'r')
            found_examples = sample_start
            desired_samples = sample_stop
            og_pose = PoseStamped()
            og_pose.header.frame_id = "camera_rgb_optical_frame"
            og_pose.pose.orientation.w = 1
            poses = f['poses'][:]
            while(not rospy.is_shutdown()):
                reset_simulation()
                index = np.random.choice(len(f['poses']),GRASP_TO_PCD_RATIO*30, replace=False)
                grasp = poses[index]
                head = Header(frame_id='object_frame')
                pose_list = np.array([PoseStamped(header=head, pose=Pose(position=Point(x=g[0],y=g[1],z=g[2]),orientation=Quaternion(x=g[3],y=g[4],z=g[5], w=g[6]))) for g in grasp])
                objwrtcam = tf_buf.transform(og_pose, "object_frame").pose.position
                pose_list = pose_list[[grasp_in_ob_origin_conic(ps.pose.position,objwrtcam) for ps in pose_list]]
                target_pose = np.array([tf_buf.transform(pose, "base_link") for pose in pose_list])
                target_pose = target_pose[[tp.pose.position.z > 0.78 for tp in target_pose]]
                for tp in target_pose: tp.pose.orientation = get_quaternion_in_hemihypersphere(get_antipodal_in_range(tp.pose.orientation))
                valid = [quaternion_in_manifold(tp.pose.orientation) for tp in target_pose]
                print(valid)
                broadcaster_frame_object('base_link','grasp_frame',target_pose[0].pose)
                ptlist = [Point(x=0.04),Point(x=-0.04)]
                create_cube_marker_from_pt(ptlist,[0.005, 0.04, 0.1],1)
                create_cube_marker_from_pt([Point(z=-0.03)],[0.06, 0.03525, 0.03525],2)
                create_arrow_marker_from_pt([Point(z=-0.03),Point(z=-0.03,y=0.1)])
                rospy.sleep(15)

        if command == 'db':
            show_rviz = True
            FILE_PATH = BG_PATH + obj_shape + "/grasps.h5"
            f = h5py.File(FILE_PATH,'r')
            found_examples = sample_start
            desired_samples = sample_stop
            head = Header(frame_id='object_frame')
            og_pose = PoseStamped()
            og_pose.header.frame_id = "camera_rgb_optical_frame"
            og_pose.pose.orientation.w = 1
            #index = np.random.choice(len(f['poses']),500, replace=False)
            poses = f['poses'][:]
            while((not rospy.is_shutdown()) and found_examples < desired_samples):
                reset_simulation()
                tpcd = rospy.wait_for_message("/camera/depth_registered/points", PointCloud2)
                tpcd = transform_pointcloud(PreprocessPointCloudRequest(tpcd)).output_cloud
                obpos = get_object_relative_pose(obj_shape,"justina::base_link").pose.position
                _,_, in_frame = find_nearest_pt_in_pc(ros_pc2_to_npmatrix(tpcd),obpos)
                if not in_frame: continue
                index = np.random.choice(len(f['poses']),GRASP_TO_PCD_RATIO*80, replace=False)
                grasp = poses[index]
                pose_list = np.array([PoseStamped(header=head, pose=Pose(position=Point(x=g[0],y=g[1],z=g[2]),orientation=Quaternion(x=g[3],y=g[4],z=g[5], w=g[6]))) for g in grasp])
                objwrtcam = tf_buf.transform(og_pose, "object_frame").pose.position
                pose_list = pose_list[[grasp_in_ob_origin_conic(ps.pose.position,objwrtcam) for ps in pose_list]]
                target_pose = np.array([tf_buf.transform(pose, "base_link") for pose in pose_list])
                target_pose = target_pose[[tp.pose.position.z > 0.78 for tp in target_pose]]
                for tp in target_pose: tp.pose.orientation = get_quaternion_in_hemihypersphere(get_antipodal_in_range(tp.pose.orientation))
                target_pose = target_pose[[quaternion_in_manifold(tp.pose.orientation) for tp in target_pose]]
                if len(target_pose) < GRASP_TO_PCD_RATIO + 5: continue
                target_pose = target_pose[np.random.choice(len(target_pose),GRASP_TO_PCD_RATIO, replace=False)]
                valid_grasps = [pose_to_nparray(tp.pose) for tp in target_pose]
                #print(len(valid_grasps))
                ##Saving
                tpcd = rospy.wait_for_message("/camera/depth_registered/points", PointCloud2)
                tpcd = transform_pointcloud(PreprocessPointCloudRequest(tpcd)).output_cloud
                obpos = get_object_relative_pose(obj_shape,"justina::base_link").pose.position
                tpcd = ros_pc2_to_npmatrix(tpcd)
                u,v, in_frame = find_nearest_pt_in_pc(tpcd,obpos)
                if not in_frame: continue
                tpcd = cut_pc(u,v,tpcd)
                found_examples = save_pcd_to_db(tpcd)
                for valid in valid_grasps:
                    found_grasps = save_grasp_to_db(valid,obj_shape,categorize_objs(obj_shape),found_examples)
                if show_rviz:
                    ptlist = [tp.pose.position for tp in target_pose]
                    create_points_marker_from_pt(ptlist,[0.04, 0.005, 0.1],1)
                print(found_examples,found_grasps)
                rospy.sleep(0.1)
            print("Finished taking samples")
            rospy.set_param('/cmd',"default")
            rospy.signal_shutdown('Finished taking samples')
        loop.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass