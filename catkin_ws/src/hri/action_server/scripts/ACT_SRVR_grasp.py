#!/usr/bin/env python3
import rospy
import sys
import tf2_ros
import tf2_geometry_msgs
import numpy as np
import smach
from smach_ros import ActionServerWrapper
import moveit_commander
from geometry_msgs.msg import Pose, Point, Quaternion, PointStamped, PoseStamped, TransformStamped
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import CollisionObject, AttachedCollisionObject
from action_server.msg import GraspAction

from tf.transformations import euler_from_quaternion, quaternion_from_euler

from utils import grasp_utils
#from utils.grasp_utils import *

class GraspingStateMachine:
    def __init__(self):
        # Import gripper controller
        self.gripper = grasp_utils.GRIPPER()
        self.brazo = grasp_utils.ARM()

        # Inicializar tf2_ros
        self.tf2_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf2_buffer)
        #self.br = tf2_ros.StaticTransformBroadcaster()

        # Inicializar MoveIt
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.whole_body = moveit_commander.MoveGroupCommander("whole_body")
        #self.whole_body_w = moveit_commander.MoveGroupCommander("whole_body_weighted")
        #self.arm = moveit_commander.MoveGroupCommander("arm")
        self.grasp_approach = "above" #above / frontal
        self.eef_link = self.whole_body.get_end_effector_link()

        # Moveit setup
        #self.scene.remove_attached_object(self.eef_link, name="objeto")
        self.whole_body.allow_replanning(True)
        self.whole_body.set_num_planning_attempts(10)
        self.whole_body.set_planning_time(10.0)
        #self.whole_body_w.allow_replanning(True)
        #self.whole_body_w.set_num_planning_attempts(10)
        #self.whole_body_w.set_planning_time(10.0)
        self.whole_body.set_workspace([-2.0, -2.0, 0.0, 2.0, 2.0, 2.0])
        #self.whole_body_w.set_workspace([-2.0, -2.0, 2.0, 2.0])
        self.planning_frame = self.whole_body.get_planning_frame()
        print(self.planning_frame)
        self.gripper.open()


        # Crear la máquina de estados SMACH
        self.sm = smach.StateMachine(outcomes=['success', 'failure'],
                                     input_keys=["goal"])
        with self.sm:
            smach.StateMachine.add('APPROACH', smach.CBState(self.approach, outcomes=['success', 'failed']),
                                   transitions={'success':'GRASP', 'failed':'APPROACH'})
            smach.StateMachine.add('GRASP', smach.CBState(self.grasp, outcomes=['success', 'failed']),
                                   transitions={'success':'RETREAT', 'failed': 'GRASP'})
            smach.StateMachine.add('RETREAT', smach.CBState(self.retreat, outcomes=['success', 'failed']),
                                   transitions={'success':'NEUTRAL_POSE', 'failed': 'RETREAT'})
            smach.StateMachine.add('NEUTRAL_POSE', smach.CBState(self.neutral_pose, outcomes=['success', 'failed']),
                        transitions={'success':'success', 'failed': 'NEUTRAL_POSE'})

        self.wrapper = ActionServerWrapper("grasp_server", GraspAction,
                                           wrapped_container = self.sm,
                                           succeeded_outcomes=["succeeded"],
                                           aborted_outcomes=["failed"],
                                           preempted_outcomes=["preempted"],
                                           goal_key='goal',
                                           result_key="action_done")
        self.wrapper.run_server()



    # SMACH states ------------------------------------------------------
    def approach(self, userdata):
        #Add primitive objets to planning scene
        goal = self.sm.userdata.goal.target_pose.data
        pos = [goal[0], goal[1], goal[2]]
        self.add_collision_object(position = pos,dimensions = [0.05, 0.05, 0.05])

        self.gripper.open()
        pose_goal = [goal[0], goal[1], goal[2]]
        if self.grasp_approach == "frontal":
            self.target_pose = self.calculate_frontal_approach(target_position=pose_goal)
            print(self.target_pose)
        elif self.grasp_approach == "above":
            self.target_pose = self.calculate_above_approach(target_position=pose_goal)
            print(self.target_pose)
        rospy.sleep(0.5)
        succ = self.move_to_pose(self.whole_body, self.target_pose)
        if succ:
            return 'success'
        else:
            return 'failed'

    def grasp(self, userdata):
        #Plan to grasp object
        #pose_goal = self.target_pose
        #grasp_pose = pose_goal
        '''if self.grasp_approach == "frontal":
            grasp_pose.position.x += 0.05   # Adelantarse a la pieza
        elif self.grasp_approach == "above":
            grasp_pose.position.z -= 0.05  # Descender hacia la pieza
        succ = self.move_to_pose(self.whole_body, grasp_pose)'''

        joint_values = self.brazo.get_joint_values()
        joint_values[0] -= 0.8
        self.brazo.set_joint_values(joint_values)
        self.gripper.close(0.03)
        #self.attach_object()
        return 'success'
        # if succ:
        #     return 'success'
        # else:
        #     return 'failed'

    def retreat(self, userdata):
        # pose_goal = self.target_pose
        #pose_goal.position.x -= 0.13
        # pose_goal.position.z += 0.13
        # position_goal = [pose_goal.position.x, pose_goal.position.y, pose_goal.position.z]
        # succ = self.move_to_position(self.whole_body, position_goal)  # Retirarse a una posición segura
        joint_values = self.brazo.get_joint_values()
        joint_values[0] += 0.15
        self.brazo.set_joint_values(joint_values)
        return 'success'
        # if succ:
        #     return 'success'
        # else:
        #     return 'failed'
    
    def neutral_pose(self, userdata):
        self.brazo.set_named_target('neutral')
        # self.whole_body.go()
        return "success"
    
    # ----------------------------------------------------------
    def move_to_position(self, group, position_goal):
        group.set_start_state_to_current_state()
        group.set_position_target(position_goal)
        succ = group.go(wait= True)
        rospy.sleep(0.5)
        group.stop()
        return succ

    def move_to_pose(self, group, pose_goal):
        group.set_start_state_to_current_state()
        group.set_pose_target(pose_goal)
        succ = group.go(wait=True)
        rospy.sleep(0.5)
        group.stop()
        return succ

    def add_collision_object(self, position = [0, 0, 0], rotation = [0,0,0,1], dimensions = [0.1 ,0.1, 0.1]):
        object_pose = PoseStamped()
        object_pose.header.frame_id = self.whole_body.get_planning_frame()
        object_pose.pose.position.x = position[0]
        object_pose.pose.position.y = position[1]
        object_pose.pose.position.z = position[2]
        object_pose.pose.orientation.x = rotation[0]
        object_pose.pose.orientation.y = rotation[1]
        object_pose.pose.orientation.z = rotation[2]
        object_pose.pose.orientation.w = rotation[3]
        self.scene.add_box("objeto", object_pose, size = (dimensions[0], dimensions[1], dimensions[2]))

    def attach_object(self):

        grasping_group = "arm"
        touch_links = self.robot.get_link_names(group=grasping_group)

        self.scene.attach_box(self.eef_link, "objeto", touch_links=touch_links)

    def execute_cb(self, goal):
        rospy.loginfo('Received action goal: %s', goal)
        self.sm.userdata.goal = goal
        if len(goal) == 3:
            self.wrapper.server.set_succeeded()
            outcome = self.sm.execute()
        else:
            rospy.loginfo("Goal not valid")

    def calculate_frontal_approach(self, target_position = [0.0, 0.0, 0.0]):
        object_point = PointStamped()
        object_point.header.frame_id = "map"
        object_point.point.x = target_position[0]
        object_point.point.y = target_position[1]
        object_point.point.z = target_position[2]

        #transformar la posicion del objeto al marco de referencia de la base del robot
        try:
            transformed_object_point = self.tf2_buffer.transform(object_point, "base_link", timeout=rospy.Duration(1))
            transformed_base = self.tf2_buffer.lookup_transform("odom", "base_link", rospy.Time(0), timeout=rospy.Duration(1))
        except :
            rospy.WARN("Error al transformar la posicion del objeto al marco de referencia")
            return None, None
        
        approach_pose = Pose()
        approach_pose.position.x = transformed_object_point.point.x - 0.12
        approach_pose.position.y = transformed_object_point.point.y
        approach_pose.position.z = transformed_object_point.point.z

        quat_base = [transformed_base.transform.rotation.x,
                               transformed_base.transform.rotation.y,
                               transformed_base.transform.rotation.z,
                               transformed_base.transform.rotation.w]
        _,_,theta = euler_from_quaternion(quat_base)
        quat = quaternion_from_euler(np.pi, -np.pi/2, -theta, axes='sxyx')
        approach_pose.orientation = Quaternion(*quat)
        return approach_pose
    
    def calculate_above_approach(self, target_position = [0.0, 0.0, 0.0]):
        object_point = PointStamped()
        object_point.header.frame_id = "base_link"
        object_point.point.x = target_position[0]
        object_point.point.y = target_position[1]
        object_point.point.z = target_position[2]

        #transformar la posicion del objeto al marco de referencia de la base del robot
        try:
            transformed_object_point = self.tf2_buffer.transform(object_point, "base_link", timeout=rospy.Duration(1))
            transformed_base = self.tf2_buffer.lookup_transform("odom", "base_link", rospy.Time(0), timeout=rospy.Duration(1))
        except :
            rospy.WARN("Error al transformar la posicion del objeto al marco de referencia")
            return None
        
        approach_pose = Pose()
        approach_pose.position.x = transformed_object_point.point.x
        approach_pose.position.y = transformed_object_point.point.y
        approach_pose.position.z = transformed_object_point.point.z + 0.12

        #print(transformed_base)
        quat_base = [transformed_base.transform.rotation.x,
                               transformed_base.transform.rotation.y,
                               transformed_base.transform.rotation.z,
                               transformed_base.transform.rotation.w]
        _,_,theta = euler_from_quaternion(quat_base)
        quat = quaternion_from_euler(-theta, 0.0, np.pi, 'szyx')
        approach_pose.orientation = Quaternion(*quat)

        return approach_pose
if __name__ == '__main__':
    rospy.init_node('grasping_demo')
    grasping_sm = GraspingStateMachine()
    #rospy.spin()
