#!/usr/bin/env python3
import rospy
import tf
from interactive_markers.interactive_marker_server import InteractiveMarkerFeedback, InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarkerControl, Marker, InteractiveMarker
from geometry_msgs.msg import Point, PointStamped, PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Header, Bool
from actionlib_msgs.msg import GoalStatus
import sys
import termios
import tty
from select import select
from scipy.interpolate import interp1d
import numpy as np
import math

from juskeshino_tools.JuskeshinoNavigation import JuskeshinoNavigation
# from general_utils import files_utils


marker_server = None
nodes_count = 0


def processFeedback(feedback):
    global path, goal_pub

    s = "Marker '" + feedback.marker_name
    s += "' / control '" + feedback.control_name + "'"
    if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
        x = round(feedback.pose.position.x, 2)
        y = round(feedback.pose.position.y, 2)
        p = str(x) + ", " + str(y)
        print('', end='\r')
        rospy.loginfo(s + ": pose changed to (" + p + ", 0.0)")
        path = get_flat_path()
        path
        
        if feedback.marker_name == "path_node_" + str(nodes_count):
            marker = marker_server.get(feedback.marker_name)
            point = PointStamped()
            point.header.frame_id = "odom"
            point.header.stamp = rospy.Time.now()
            point.point = marker.pose.position
            goal_pub.publish(point)


def create_marker(interaction_mode, position):
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "odom"
    int_marker.pose.position = position
    int_marker.scale = 1
    int_marker.name = "path_node_" + str(nodes_count)
    int_marker.description = "path_node_" + str(nodes_count)

    marker = Marker()
    marker.type = Marker.SPHERE
    marker.scale.x = 0.1
    marker.scale.y = 0.1
    marker.scale.z = 0.1
    marker.color.r = 0.8
    marker.color.g = 0.8
    marker.color.b = 0.2
    marker.color.a = 1.0

    control = InteractiveMarkerControl()
    control.always_visible = True
    control.markers.append(marker)
    int_marker.controls.append(control)
    int_marker.controls[0].interaction_mode = interaction_mode

    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 1
    control.orientation.y = 0
    control.orientation.z = 0
    control.name = "move_x"
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    int_marker.controls.append(control)

    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 0
    control.orientation.z = 1
    control.name = "move_y"
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    int_marker.controls.append(control)
    int_marker.scale = 0.4
    marker_server.insert(int_marker, processFeedback)
    marker_server.applyChanges()
    return int_marker


def globalGoalCallback(msg):
    global goal_x, goal_y
    global path, nodes_count, goal_pub

    goal_x = msg.point.x
    goal_y = msg.point.y
    position = Point(round(goal_x, 2), round(goal_y, 2), 0)
    nodes_count += 1
    create_marker(InteractiveMarkerControl.NONE, position)
    print(f"Clicked ({goal_x:.2f}, {goal_y:.2f}, 0)")
    print("\rnodes: ", nodes_count)
    path = get_flat_path()
    
    point = PointStamped()
    point.header.frame_id = "odom"
    point.header.stamp = rospy.Time.now()
    point.point = position
    goal_pub.publish(point)


def stopSaveDataCallback(msg):
    global save_enable
    print(">> GOAL status reached:", msg.status)
    save_enable = False


def get_key(set, timeout):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select([sys.stdin], [], [], timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, set)
    return key


def remove_nodes():
    global nodes_count

    if nodes_count > 0:
        marker_server.erase("path_node_" + str(nodes_count))
        marker_server.applyChanges()
        nodes_count -= 1
    else:
        print("\rnodes not found",)


def clear_nodes():
    global path

    for _ in range(nodes_count):
        remove_nodes()
    print("\rnodes: ", nodes_count)
    path = get_flat_path()
    path_pub.publish(path)


def get_robot_pose_odom():
    listener = tf.TransformListener()
    listener.waitForTransform('odom', 'base_link', rospy.Time(0), rospy.Duration(1.0))
    ([x, y, z], rot) = listener.lookupTransform('odom', 'base_link', rospy.Time(0))
    return [x, y, z, rot]


def restart_path(p):
    while True:
        [x, y, _, rot] = get_robot_pose_odom()
        theta = 2*math.atan2(rot[2], rot[3])
        if abs(theta) > math.pi:
            theta = theta - (np.sign(theta)*2*math.pi)
        pos = p.pose.position
        dx = pos.x - x
        dy = pos.y - y
        angle_to_goal = math.atan2(dy, dx)
        delta_angle = angle_to_goal - theta
        if abs(delta_angle) > math.pi:
            delta_angle = delta_angle - (np.sign(delta_angle)*2*math.pi)
        JuskeshinoNavigation.startMoveDistAngle(0, delta_angle)
        rospy.sleep(0.1)
        if abs(delta_angle) < 0.3:
            JuskeshinoNavigation.startMoveDistAngle(0, 0)
            break
    JuskeshinoNavigation.startMoveDist(0.5)
    rospy.sleep(1.5)


def get_flat_path():
    path = Path()
    path.header = Header(frame_id='odom')
    if nodes_count > 0:
        x, y, _, rot = get_robot_pose_odom()
        pose = PoseStamped()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = rot[0]
        pose.pose.orientation.y = rot[1]
        pose.pose.orientation.z = rot[2]
        pose.pose.orientation.w = rot[3]
        path.poses.append(pose)
        for i in range(1, nodes_count + 1):
            pose = PoseStamped()
            mark_name = "path_node_" + str(i)
            marker = marker_server.get(mark_name)
            pose.pose = marker.pose
            path.poses.append(pose)
    return path


def spline_curve():
    global path

    x = []
    y = []
    for p in path.poses:
        x.append(p.pose.position.x)
        y.append(p.pose.position.y)

    x = np.expand_dims(x, axis=0)
    y = np.expand_dims(y, axis=0)
    xy = np.vstack((x, y))
    N = nodes_count+1
    t = np.linspace(0, 1, N)
    spline = interp1d(t, xy, kind="quadratic", bounds_error=False)
    t_fine = np.linspace(0, 1, N**2)
    x_fine, y_fine = spline(t_fine)

    path = Path()
    path.header = Header(frame_id='odom')
    for i in range(len(x_fine)):
        pose = PoseStamped()
        pose.pose.position.x = round(x_fine[i], 2)
        pose.pose.position.y = round(y_fine[i], 2)
        path.poses.append(pose)

    return path

def calculate_soft_path():
    global path, old_path, path_pub
    path = get_flat_path()
    if nodes_count > 1:
        path = spline_curve()
    path_pub.publish(path)
    old_path = get_flat_path()


def is_same_pat(path1, path2):
    count = 1
    if len(path1.poses) == len(path2.poses):
        for p1, p2 in zip(path1.poses, path2.poses):
            if p1.pose != p2.pose and count < 0:
                return False
            count += 1
        return True
    else: 
        return False



if __name__ == "__main__":
    global path, old_path
    global path_pub, save_enable, goal_pub

    rospy.init_node("path_controls")

    JuskeshinoNavigation.setNodeHandle()

    rospy.Subscriber('/clicked_point', PointStamped, globalGoalCallback)
    rospy.Subscriber('/simple_move/goal_reached', GoalStatus, stopSaveDataCallback)
    path_pub = rospy.Publisher('/mapless_nav/goal_path', Path, queue_size=10)
    goal_path_pub = rospy.Publisher('/simple_move/goal_path', Path, queue_size=10)
    save_path_pub = rospy.Publisher('/mapless_nav/save_dataset', Bool, queue_size=10)
    goal_pub = rospy.Publisher('/mapless_nav/goal', PointStamped, queue_size=10)

    set = termios.tcgetattr(sys.stdin)
    key_timeout = rospy.get_param("~key_timeout", 0.5)
    marker_server = InteractiveMarkerServer("path_controls")
    path = Path()
    path.header = Header(frame_id='odom')
    old_path = path
    save_enable = False

    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        key = get_key(set, key_timeout)
        key = key.lower()

        if key == 'q':
            print(key)
            rospy.logwarn("Exit selected")
            clear_nodes()
            rospy.signal_shutdown('')

        elif key == 'r':
            print(key + "- remove node", end="\r")
            remove_nodes()
            print("\rnodes: ", nodes_count)
            path = get_flat_path()

        elif key == 'c':
            rospy.logwarn(key + "- remove all nodes")
            clear_nodes()

        elif key == 'p':
            rospy.logwarn(key + "- path preview")
            calculate_soft_path()

        elif key == 'n' or key == 's':
            cad = "- navigate"
            
            if nodes_count == 0:
                rospy.logerr("Use Publish Point to inset path nodes")
                continue
            path = get_flat_path()
            if is_same_pat(old_path, path):
                restart_path(path.poses[-2])
            if key == 's':
                cad = "- navigate and save"
                save_enable = True
            rospy.logwarn(key + cad)
            calculate_soft_path()
            if nodes_count > 1:
                path = get_flat_path()
                path = spline_curve()
                path_pub.publish(path)
            goal_path_pub.publish(path)

        else:
            print('', end="\r")  # clear whole line
            sys.stdout.write('\x1b[2K')

        path_pub.publish(path)
        save_path_pub.publish(save_enable)
        rate.sleep()
