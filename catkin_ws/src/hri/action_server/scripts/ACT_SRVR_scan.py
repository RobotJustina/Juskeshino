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

from utils.grasp_utils import *

class GraspingStateMachine:
    def __init__(self):
        # Import gripper controller
        self.gripper = GRIPPER()

        # Inicializar tf2_ros
        self.tf2_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf2_buffer)
        self.br = tf2_ros.StaticTransformBroadcaster()

        # Crear la máquina de estados SMACH
        self.sm = smach.StateMachine(outcomes=['success', 'failure'])
        with self.sm:
            smach.StateMachine.add('APPROACH', smach.CBState(self.approach, outcomes=['success']),
                                   transitions={'success':'GRASP'})
            smach.StateMachine.add('GRASP', smach.CBState(self.grasp, outcomes=['success']),
                                   transitions={'success':'RETREAT'})
            smach.StateMachine.add('RETREAT', smach.CBState(self.retreat, outcomes=['success']),
                                   transitions={'success':'success'})

        # Inicializar el servidor de acción
        #self.server = SimpleActionServer('follow_server', FollowAction, execute_cb=self.execute_cb, auto_start=False)
        #self.server.start()
        self.wrapper = ActionServerWrapper("visualize_server", VisualizeAction,
                                           wrapped_container = self.sm,
                                           succeeded_outcomes=["succeeded"],
                                           aborted_outcomes=["failed"],
                                           preempted_outcomes=["preempted"],
                                           goal_key='goal',
                                           result_key="action_done")
        self.wrapper.run_server()

        outcome = self.sm.execute()


    # SMACH states ------------------------------------------------------
    def approach(self, userdata):
        return 'success'

    def grasp(self, userdata):
        
        return 'success'

    def retreat(self, userdata):
        pose_goal = self.target_pose
        #pose_goal.position.x -= 0.13
        pose_goal.position.z += 0.13
        position_goal = [pose_goal.position.x, pose_goal.position.y, pose_goal.position.z]
        self.move_to_position(self.whole_body, position_goal)  # Retirarse a una posición segura
        return 'success'
    
    def neutral_pose(self, userdata):
        self.whole_body.set_named_target("neutral")
        self.whole_body.go()
        return "success"
    
    # ----------------------------------------------------------
    def move_to_position(self, group, position_goal):
        group.set_start_state_to_current_state()
        group.set_position_target(position_goal)
        plan = group.go(wait= True)
        rospy.sleep(0.5)
        group.stop()

    def move_to_pose(self, group, pose_goal):
        group.set_start_state_to_current_state()
        group.set_pose_target(pose_goal)
        plan = group.go(wait=True)
        rospy.sleep(0.5)
        group.stop()

    def add_collision_object(self, position = [0, 0, 0], dimensions = [0.1 ,0.1, 0.1]):
        # collision_object = CollisionObject()
        # collision_object.id = "objeto"
        # collision_object.header.frame_id = "map" #self.group.get_planning_frame()
        # primitive = SolidPrimitive()
        # primitive.type = SolidPrimitive.BOX
        # primitive.dimensions = dimensions
        # pose = Pose()
        # pose.position.x = position[0]
        # pose.position.y = position[1]
        # pose.position.z = position[2]
        # collision_object.primitives.append(primitive)
        # collision_object.primitive_poses.append(pose)
        # self.scene.add_object(collision_object)
        object_pose = PoseStamped()
        object_pose.header.frame_id = self.whole_body.get_planning_frame()
        object_pose.pose = self.target_pose
        self.scene.add_box("objeto", object_pose, size = (dimensions[0], dimensions[1], dimensions[2]))

    def attach_object(self):

        grasping_group = "arm"
        touch_links = self.robot.get_link_names(group=grasping_group)

        self.scene.attach_box(self.eef_link, "objeto", touch_links=touch_links)
        # attached_object = AttachedCollisionObject()
        # attached_object.object.id = "objeto"
        # attached_object.link_name = "hand_palm_link"

        # attached_object.object.operation = attached_object.object.ADD

        # self.scene.attach_object(attached_object)

    def execute_cb(self, goal):
        rospy.loginfo('Received action goal: %s', goal)
        self.sm.userdata.goal = goal
        self.wrapper.server.set_succeeded()
        outcome = self.sm.execute()
        # result = FollowResult()
        # result.success = True if outcome == 'success' else False
        # self.server.set_succeeded(result)

if __name__ == '__main__':
    rospy.init_node('grasping_demo')
    grasping_sm = GraspingStateMachine()
    #rospy.spin()
