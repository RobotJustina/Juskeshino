#!/usr/bin/env python
import rospy
from interactive_markers.interactive_marker_server import InteractiveMarkerFeedback, InteractiveMarkerServer
from interactive_markers.menu_handler import MenuHandler
from visualization_msgs.msg import InteractiveMarkerControl, Marker, InteractiveMarker
from geometry_msgs.msg import Point, PointStamped
import sys
import termios
import tty
from select import select


server = None
menu_handler = MenuHandler()
counter = 0
nodes_count = 0
nodes = []


def processFeedback(feedback):
    s = "Marker '" + feedback.marker_name
    s += "' / control '" + feedback.control_name + "'"
    if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
        x = round(feedback.pose.position.x, 2)
        y = round(feedback.pose.position.y, 2)
        p = str(x) + ", " + str(y)
        rospy.loginfo(s + ": pose changed to (" + p + ", 0.0)")

    server.applyChanges()


def makeMarkerControl(msg):
    control = InteractiveMarkerControl()
    control.always_visible = True
    marker = Marker()
    marker.type = Marker.SPHERE
    marker.scale.x = msg.scale * 0.1
    marker.scale.y = msg.scale * 0.1
    marker.scale.z = msg.scale * 0.1
    marker.color.r = 0.8
    marker.color.g = 0.8
    marker.color.b = 0.2
    marker.color.a = 1.0
    control.markers.append(marker)
    msg.controls.append(control)
    return control


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
    server.insert(int_marker, processFeedback)
    return int_marker


def globalGoalCallback(msg):
    global goal_x
    global goal_y
    global nodes_count
    goal_x = msg.point.x
    goal_y = msg.point.y
    # rospy.DEBUG("CLIKED POINT>>>")

    position = Point(round(goal_x, 2), round(goal_y, 2), 0)
    nodes_count += 1
    marker_n = createMarker(InteractiveMarkerControl.NONE, position)
    server.applyChanges()
    nodes.append(marker_n)
    print(f"Clicked ({goal_x:.2f}, {goal_y:.2f}, 0)")
    print("\rnodes: ", nodes_count)


def getKey(set, timeout):
    tty.setraw(sys.stdin.fileno())
    # sys.stdin.read() returns a string on Linux
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
        server.erase("path_node_" + str(nodes_count))
        server.applyChanges()
        nodes_count -= 1
    else:
        print("\rnodes not found",)


if __name__ == "__main__":
    set = termios.tcgetattr(sys.stdin)
    rospy.init_node("path_controls")

    key_timeout = rospy.get_param("~key_timeout", 0.5)

    server = InteractiveMarkerServer("path_controls")
    # goal_pub = rospy.Publisher("/goal", Point, queue_size=10)
    rospy.Subscriber('/clicked_point', PointStamped, globalGoalCallback)

    rate = rospy.Rate(5)  # hz
    while not rospy.is_shutdown():
        key = getKey(set, key_timeout)
        key = key.lower()

        if key == 'q':
            print(key)
            rospy.logwarn("Exit selected")
            rospy.signal_shutdown('')
        elif key == 'r':
            print(key + "- remove node")
            removeNodes()
            print("\rnodes: ", nodes_count)
        elif key == 'c':
            # print(key + "- remove all nodes")
            rospy.logwarn(key + "- remove all nodes")
            for i in range(nodes_count):
                removeNodes()
            print("\rnodes: ", nodes_count)
        elif key == 'p':
            print(key + "- generate path")

        else:
            print('', end="\r")  # clear whole line
            sys.stdout.write('\x1b[2K')
            # print('', end='\r')

        rate.sleep()

    # rospy.spin()
