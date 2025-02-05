#!/usr/bin/env python3

import rospy 
import rospkg 
import random
from gazebo_msgs.msg import ModelState, ContactsState 
from gazebo_msgs.srv import SetModelState
from std_msgs.msg import String, Float64MultiArray
from geometry_msgs.msg import Pose  
from manip_msgs.srv import DataCapture

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
    global justina_origin_pose, obj, msg_la, pub_la, pub_hd, msg_hd, pub_object, num_loops, left_gripper_made_contact, right_gripper_made_contact, grasp_attempts
    change_gazebo_object_pose(state_msg, generate_random_pose(), obj)
    change_gazebo_object_pose(state_msg, justina_origin_pose, "justina")
    num_loops = 0
    left_gripper_made_contact = False
    right_gripper_made_contact = False
    pub_la.publish(msg_la)
    pub_hd.publish(msg_hd)
    rospy.sleep(0.1)
    pub_object.publish(obj)
    pub_object.publish(obj)
    grasp_attempts = 0
    

def main():
    global state_msg, grasp_trajectory_found, justina_origin_pose, obj, left_gripper_made_contact, right_gripper_made_contact, grasp_attempts, msg_la, pub_la, pub_hd, msg_hd, pub_object, num_loops
    state_msg = ModelState()
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
    obj_shape = 'apple'
    rospy.init_node('object_grip_test')
    print("Starting grip test")
    rospy.Subscriber('/manipulation/grasp/grasp_status' ,String ,callback_grasp_status)
    rospy.Subscriber('/left_arm_grip_left_sensor' ,ContactsState ,callback_left_grip_sensor)
    rospy.Subscriber('/left_arm_grip_right_sensor' ,ContactsState ,callback_right_grip_sensor)
    rospy.wait_for_service('/manipulation/grasp/data_capture_service')
    capture = rospy.ServiceProxy('/manipulation/grasp/data_capture_service', DataCapture)
    set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    pub_object = rospy.Publisher("/plannning/simple_task/take_object", String, queue_size=10)
    pub_la = rospy.Publisher("/hardware/left_arm/goal_pose", Float64MultiArray, queue_size=10)
    pub_hd = rospy.Publisher("/hardware/head/goal_pose", Float64MultiArray, queue_size=10)
    obj = rospy.get_param("~obj","apple")
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
            pose_num = pose_num + 1
            file_name = POSE_DATA_PATH + obj_shape + str(pose_num)
            in_file = open(file_name, "rb") # opening for [r]eading as [b]inary
            file_serialized_gripper_model_state = in_file.read() # if you only wanted to read 512 bytes, do .read(512)
            in_file.close()
            deserialized_gripper_model_state = ModelState()
            deserialized_gripper_model_state.deserialize(file_serialized_gripper_model_state)
            deserialized_gripper_model_state.reference_frame = 'apple'
            print(deserialized_gripper_model_state)
            set_state(deserialized_gripper_model_state)
        loop.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass