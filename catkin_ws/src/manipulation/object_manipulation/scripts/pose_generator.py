#!/usr/bin/env python3

import rospy 
import rospkg 
import random
import math
import matplotlib.pyplot as plt
import tf.transformations as tft
import numpy as np
import vg
import tf
from gazebo_msgs.msg import ModelState, ContactsState 
from gazebo_msgs.srv import SetModelState, GetModelState
from std_msgs.msg import String, Float64MultiArray
from geometry_msgs.msg import Pose, PointStamped  
from manip_msgs.srv import DataCapture

BASE_JUSTINA_VECTOR = np.array([0.0,-1.0,0.0])
VG_PLANE = {
    "XY": vg.basis.z,
    "YZ": vg.basis.x,
    "ZX": vg.basis.y
}

def generate_random_pose():
    rpose = Pose()
    rpose.position.x = random.randint(205,310)/100
    rpose.position.y = random.randint(205,225)/100
    rpose.position.z = 0.745
    rpose.orientation.x = random.randint(-315,315)/100
    rpose.orientation.y = random.randint(-315,315)/100
    rpose.orientation.z = random.randint(-315,315)/100
    rpose.orientation.w = 0
    return rpose

def change_gazebo_object_pose(state_msg, state_pose, mod_name):
    state_msg.model_name = mod_name
    state_msg.pose = state_pose
    rospy.wait_for_service('/gazebo/set_model_state')
    try:
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        resp = set_state( state_msg )

    except rospy.ServiceException:
        pass      

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
    jop.position.x = 2.46
    jop.position.y = 1.46
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
    num_loops = 0
    left_gripper_made_contact = False
    right_gripper_made_contact = False
    pub_la.publish(msg_la)
    pub_hd.publish(msg_hd)
    rospy.sleep(0.1)
    pub_object.publish(obj_shape)
    pub_object.publish(obj_shape)
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

def main():
    global state_msg, grasp_trajectory_found, justina_origin_pose, obj_shape, left_gripper_made_contact, right_gripper_made_contact, grasp_attempts, msg_la, pub_la, pub_hd, msg_hd, pub_object, num_loops
    state_msg = ModelState()
    deserialized_gripper_model_state = ModelState()
    justina_origin_pose = create_origin_pose()
    msg_la = Float64MultiArray()
    msg_hd = Float64MultiArray()
    msg_la.data = [0,0,0,0,0,0,0]
    msg_hd.data = [0,0]
    msg_capture = String()
    grasp_trajectory_found = False
    left_gripper_made_contact = False
    right_gripper_made_contact = False
    grasp_attempts = 0
    num_loops = 0
    found_grasps = 0
    obj_shape = '056_tennis_ball'
    rospy.init_node('object_grip_test')
    print("Starting grip test")
    # rospy.Subscriber('/manipulation/grasp/grasp_status' ,String ,callback_grasp_status)
    # rospy.Subscriber('/gr_left_arm_grip_left_sensor' ,ContactsState ,callback_left_grip_sensor)
    # rospy.Subscriber('/gr_left_arm_grip_right_sensor' ,ContactsState ,callback_right_grip_sensor)
    rospy.wait_for_service('/manipulation/grasp/data_capture_service')
    get_object_relative_pose = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    capture = rospy.ServiceProxy('/manipulation/grasp/data_capture_service', DataCapture)
    set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    pub_object = rospy.Publisher("/plannning/simple_task/take_object", String, queue_size=10)
    pub_la = rospy.Publisher("/hardware/left_arm/goal_pose", Float64MultiArray, queue_size=10)
    pub_hd = rospy.Publisher("/hardware/head/goal_pose", Float64MultiArray, queue_size=10)
    obj_shape = rospy.get_param("/obj","056_tennis_ball")
    rospy.sleep(1)
    reset_simulation()
    POSE_DATA_PATH = "./catkin_ws/src/manipulation/object_manipulation/pose_data/"
    pose_num = 0
    loop = rospy.Rate(1)
    while not rospy.is_shutdown():
        print("Type r to reset sim to a random pose")
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
            while(not rospy.is_shutdown()):
                reset_simulation()
                #resp = capture("Initial Conditions")
                print("Saved initial conditions")
                pose_num = 0
                found_grasps = 0
                while(pose_num<15):
                    pose_num = pose_num + 1
                    file_name = POSE_DATA_PATH + obj_shape + str(pose_num)
                    in_file = open(file_name, "rb") # opening for [r]eading as [b]inary
                    file_serialized_gripper_model_state = in_file.read() # if you only wanted to read 512 bytes, do .read(512)
                    in_file.close()
                    deserialized_gripper_model_state.deserialize(file_serialized_gripper_model_state)
                    deserialized_gripper_model_state.reference_frame = obj_shape
                    set_state(deserialized_gripper_model_state)
                    rospy.sleep(0.001)
                    angle_XY, angle_YZ, angle_ZX, gripper_side = get_angle_in_plane(get_object_relative_pose("justina_gripper","world").pose.position, 
                                                                get_object_relative_pose(obj_shape,"world").pose.position,"XY")
                    if is_pose_valid(angle_XY, angle_YZ, angle_ZX): 
                        rospy.sleep(0.5)
                        capture("Found grasp")
                        rospy.sleep(1)
                        found_grasps = found_grasps + 1
                        print("Found Grasp: ",found_grasps)

                        rospy.sleep(1)

        loop.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass