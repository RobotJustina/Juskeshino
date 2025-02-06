#!/usr/bin/env python3
import rospy
import tf
from interactive_markers.interactive_marker_server import InteractiveMarkerFeedback, InteractiveMarkerServer
from interactive_markers.menu_handler import MenuHandler
from visualization_msgs.msg import InteractiveMarkerControl, Marker, InteractiveMarker
from geometry_msgs.msg import Point, PointStamped, PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Header
import sys
import termios
import tty
from select import select
from scipy.interpolate import interp1d
import matplotlib.pyplot as plt
import numpy as np
from juskeshino_tools.JuskeshinoNavigation import JuskeshinoNavigation
from general_utils import files_utils
import math

marker_server = None
menu_handler = MenuHandler()
counter = 0
nodes_count = 0
splin_path = False

def processFeedback(feedback):
    global path

    s = "Marker '" + feedback.marker_name
    s += "' / control '" + feedback.control_name + "'"
    if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
        x = round(feedback.pose.position.x, 2)
        y = round(feedback.pose.position.y, 2)
        p = str(x) + ", " + str(y)
        print('', end='\r')
        rospy.loginfo(s + ": pose changed to (" + p + ", 0.0)")
        path = getPath()


def createMarker(interaction_mode, position):
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
    global path, nodes_count

    goal_x = msg.point.x
    goal_y = msg.point.y
    position = Point(round(goal_x, 2), round(goal_y, 2), 0)
    nodes_count += 1
    createMarker(InteractiveMarkerControl.NONE, position)
    print(f"Clicked ({goal_x:.2f}, {goal_y:.2f}, 0)")
    print("\rnodes: ", nodes_count)
    path = getPath()


def getKey(set, timeout):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select([sys.stdin], [], [], timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, set)
    return key


def removeNodes():
    global nodes_count
    if nodes_count > 0:
        marker_server.erase("path_node_" + str(nodes_count))
        marker_server.applyChanges()
        nodes_count -= 1
    else:
        print("\rnodes not found",)


def clear_nodes():
    global path
    for i in range(nodes_count):
        removeNodes()
    print("\rnodes: ", nodes_count)
    path = getPath()
    path_pub.publish(path)
    splin_path = False


def getRobotPoseOdom():
    listener = tf.TransformListener()
    listener.waitForTransform(
        'odom', 'base_link', rospy.Time(0), rospy.Duration(1.0))
    ([x, y, z], rot) = listener.lookupTransform(
        'odom', 'base_link', rospy.Time(0))
    return [x, y, z, rot]


def getPath():
    path = Path()
    path.header = Header(frame_id='odom')
    if nodes_count > 0:
        x, y, _, rot = getRobotPoseOdom()
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


def splineCurve():
    global path

    x = []
    y = []
    for p in path.poses:
        x.append(p.pose.position.x)
        y.append(p.pose.position.y)

    x = np.expand_dims(x, axis=0)
    y = np.expand_dims(y, axis=0)
    print(x.shape)
    print(y.shape)
    xy = np.vstack((x, y))
    print(xy.shape)
    N = nodes_count+1
    print("n", nodes_count)
    t = np.linspace(0, 1, N)
    spline = interp1d(t, xy, kind="quadratic", bounds_error=False)
    t_fine = np.linspace(0, 1, N**2)
    x_fine, y_fine = spline(t_fine)
    print(x_fine)
    print(x_fine.shape)

    path = Path()
    path.header = Header(frame_id='odom')
    for i in range(len(x_fine)):
        pose = PoseStamped()
        pose.pose.position.x = round(x_fine[i], 2)
        pose.pose.position.y = round(y_fine[i], 2)
        path.poses.append(pose)

    return path


def navigate():
    global path
    pt = Path()
    #pt.poses
    first = False
    count = 0
    for p in path.poses:
        if first:
            pass
            #first = False
        else:    
            print(type(p))
            print(p)
            rot = p.pose.orientation
            a = 2*math.atan2(rot.z, rot.w)
            a = a - 2*math.pi if a > math.pi else a
            JuskeshinoNavigation.startGetCloseXYAOdom(p.pose.position.x, p.pose.position.y, a)
            count += 1
            first = True
    print(count)

if __name__ == "__main__":
    global path

    rospy.init_node("path_controls")
    jn = JuskeshinoNavigation()
    
    JuskeshinoNavigation.setNodeHandle()

    rospy.Subscriber('/clicked_point', PointStamped, globalGoalCallback)
    # /simple_move/goal_path # /goal_path
    path_pub = rospy.Publisher('/goal_path', Path, queue_size=10)
    # /simple_move/goal_path # /goal_path
    goal_path_pub = rospy.Publisher(
        '/mapless/goal_path', Path, queue_size=10)

    set = termios.tcgetattr(sys.stdin)
    key_timeout = rospy.get_param("~key_timeout", 0.5)
    marker_server = InteractiveMarkerServer("path_controls")
    path = Path()
    path.header = Header(frame_id='odom')

    rate = rospy.Rate(5)  # hz
    while not rospy.is_shutdown():
        key = getKey(set, key_timeout)
        key = key.lower()

        if key == 'q':
            print(key)
            rospy.logwarn("Exit selected")
            clear_nodes()
            rospy.signal_shutdown('')
        elif key == 'r':
            print(key + "- remove node")
            removeNodes()
            print("\rnodes: ", nodes_count)
            path = getPath()
        elif key == 'c':
            rospy.logwarn(key + "- remove all nodes")
            clear_nodes()
        elif key == 'p':
            rospy.logwarn(key + "- path preview")
            
            path = getPath()
            print(nodes_count)
            if nodes_count > 1:
                path = splineCurve()
            x, y, a = JuskeshinoNavigation.getRobotPoseWrtOdom()
            print(x, y, a)
            path_pub.publish(path)
            
        elif key == 'n':
            rospy.logwarn(key + "- navigate")

            path = getPath()
            print(nodes_count)
            if nodes_count > 1:
                path = splineCurve()
            x, y, a = JuskeshinoNavigation.getRobotPoseWrtOdom()
            print(x, y, a)
            path_pub.publish(path)
            goal_path_pub.publish(path)

        else:
            print('', end="\r")  # clear whole line
            sys.stdout.write('\x1b[2K')

        path_pub.publish(path)
        rate.sleep()

    # rospy.spin()
