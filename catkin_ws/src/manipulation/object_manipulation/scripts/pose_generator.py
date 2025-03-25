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
from gazebo_msgs.msg import ModelState, ContactsState 
from gazebo_msgs.srv import SetModelState, GetModelState
from std_msgs.msg import String, Float64MultiArray
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Pose, Quaternion  
from manip_msgs.srv import DataCapture, InverseKinematicsPose2TrajRequest, InverseKinematicsPose2Traj
from vision_msgs.srv import PreprocessPointCloud, PreprocessPointCloudRequest
from dataset_utils import save_data_to_file
BASE_JUSTINA_VECTOR = np.array([0.0,-1.0,0.0])
CONIC_ANGLE = math.cos(math.radians(30))
VG_PLANE = {
    "XY": vg.basis.z,
    "YZ": vg.basis.x,
    "ZX": vg.basis.y
}


def get_Z_obj():
    global z1, z2, obj_shape
    z_obj_dic = {
        '024_bowl':          [0.724, 0.724],
        '001_chips_can':     [0.739, 0.799], 
        '007_tuna_fish_can': [0.716, 0.742],
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
    dishes = ['024_bowl']
    prismatic = ['001_chips_can', '007_tuna_fish_can']
    spherical = ['054_softball', '055_baseball', '056_tennis_ball']
    flat = ['006_mustard_bottle']
    box = ['pudding_box', '077_rubiks_cube']
    two_faces = ['011_banana', '048_hammer', '044_flat_screwdriver']
    if   name in dishes:    return 'dishes'
    elif name in prismatic: return 'prismatic'
    elif name in spherical: return 'spherical'
    elif name in flat:      return "flat"
    elif name in box:       return 'box'
    elif name in two_faces: return '2faces'
    

def rotation_object():
    global obj_shape, z
    geometric_shape_dic = {
                            "dishes":     [[0, 0, np.deg2rad(random.randint(0, int(359)))]],
                            "prismatic":    [[0, 0, np.deg2rad(random.randint(0, int(359)))], [0, 1.57, np.deg2rad(random.randint(0, int(359)))] ],
                            "spherical":[[np.deg2rad(random.randint(0, int(359))) , np.deg2rad(random.randint(0, int(359))) ,np.deg2rad(random.randint(0, int(359)))]],
                            "flat":     [[0, 0, np.deg2rad(random.randint(0, int(359)))], [0, 1.57, np.deg2rad(random.randint(0, int(359)))] ,  [0, -1.57, np.deg2rad(random.randint(0, int(359)))] ],
                            "cubic":      [ [0, 1.57 , np.deg2rad(random.randint(0, int(359)))] ,  [0, 3.14, np.deg2rad(random.randint(0, int(359)))],  [0, 4.71, np.deg2rad(random.randint(0, int(359)))],  [0, 6.28, np.deg2rad(random.randint(0, int(359)))],
                                          [1.57, 0 , np.deg2rad(random.randint(0, int(359)))] ,  [3.14, 0,  np.deg2rad(random.randint(0, int(359)))], [4.71, 0, np.deg2rad(random.randint(0, int(359)))],  [6.28, 0, np.deg2rad(random.randint(0, int(359)))]],
                            "box":        [[0, 3.14, np.deg2rad(random.randint(0, int(359)))],
                                          [1.57, 0 , np.deg2rad(random.randint(0, int(359)))] ,  [3.14, 0,  np.deg2rad(random.randint(0, int(359)))], [4.71, 0, np.deg2rad(random.randint(0, int(359)))],  [6.28, 0, np.deg2rad(random.randint(0, int(359)))]],
                            "2faces":    [[0, 0, np.deg2rad(random.randint(0, int(359)))] ,  [0, 3.14, np.deg2rad(random.randint(0, int(359)))]]
    }

    rotation = random.choice(geometric_shape_dic[categorize_objs(obj_shape)])
    z1, z2 = get_Z_obj()
    print("rotacion",np.rad2deg(rotation[0]), np.rad2deg(rotation[1]))
    if((np.rad2deg(rotation[0]) or np.rad2deg(rotation[1])) <= 0):
        z = z2
        print("z2", z2)

    else:
        z = z1
        print("z1", z1)    
        

    quaternion_obj = tft.quaternion_from_euler(rotation[0],rotation[1],rotation[2] ,'sxyz')

    return quaternion_obj


def generate_random_pose():
    global z, z1, z2
    rpose = Pose()
    rpose.position.x = random.randint(210,310)/100
    rpose.position.y = random.randint(218,245)/100
    rpose.position.z = z
    q = rotation_object()
    rpose.orientation.x = q[0]
    rpose.orientation.y = q[1]
    rpose.orientation.z = q[2]
    rpose.orientation.w = q[3]
    return rpose

    

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
    jop.position.z = 0.06
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

def get_quaternion_in_range(q):
    ql = [q.x,q.y,q.z,q.w]
    rot = tft.quaternion_matrix(ql)
    print(rot)
    x = rot[:3,0]
    print(x)
    in_range_z = np.dot(x,np.array([0,0,-1])) > 0
    in_range_y = np.dot(x,np.array([0,-1,0])) > 0
    print(in_range_y,in_range_z)
    if in_range_z or in_range_y:
        rot[:3,:2] = -rot[:3,:2]
    print(rot)
    qf = tft.quaternion_from_matrix(rot)
    qr = Quaternion()
    qr.x = qf[0]
    qr.y = qf[1]
    qr.z = qf[2]
    qr.w = qf[3]
    return qr


def score_calculation(articular_array):
    a = sum(articular_array)

def gripper_in_conic(gp,ob):
    g = np.asarray([gp.x,gp.y,gp.z])
    o = np.asarray([ob.x,ob.y,ob.z])
    axis = o/np.linalg.norm(o)
    print(np.dot(((g-o)/np.linalg.norm(g-o)),axis))
    in_conic = np.dot(((g-o)/np.linalg.norm(g-o)),axis) < CONIC_ANGLE
    print("In conic:", in_conic)
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



def main():
    global ik_srv, state_msg, grasp_trajectory_found, justina_origin_pose, obj_shape, left_gripper_made_contact, right_gripper_made_contact, grasp_attempts, msg_la, pub_la, pub_hd, msg_hd, pub_object, num_loops
    global set_state, get_object_relative_pose, z
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
    POSE_DATA_PATH = "./catkin_ws/src/manipulation/object_manipulation/pose_data/"
    pose_num = 0
    loop = rospy.Rate(1)



    while not rospy.is_shutdown():
        print("Type r to reset sim to a random pose, and l to loop simulation for samples")
        command = input()
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
            quat = get_quaternion_in_range(gpos.orientation)
            print(quat)
            deserialized_gripper_model_state.pose.orientation = quat
            deserialized_gripper_model_state.pose.position = gpos.position
            deserialized_gripper_model_state.reference_frame = "world"
            set_state(deserialized_gripper_model_state)
            print(get_object_relative_pose("justina_gripper","world"))
            uia = input()
        
        if command == 'e':
 
            while((not rospy.is_shutdown())):
                msg.pose = generate_random_pose()
                set_state(msg)
                get_object_relative_pose(obj_shape,"world").pose.position

                rospy.sleep(4)

        loop.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass