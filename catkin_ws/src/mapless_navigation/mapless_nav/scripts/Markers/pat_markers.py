#!/usr/bin/env python
import rospy

from interactive_markers.interactive_marker_server import InteractiveMarkerFeedback, InteractiveMarkerServer
from interactive_markers.menu_handler import MenuHandler
from visualization_msgs.msg import InteractiveMarkerControl, Marker, InteractiveMarker
from geometry_msgs.msg import Point
from tf.broadcaster import TransformBroadcaster
from math import sin

server = None
menu_handler = MenuHandler()
br = None
counter = 0
nodes_count = 0


def frameCallback(msg):
    global counter, br
    time = rospy.Time.now()
    br.sendTransform((0, 0, sin(counter/140.0)*2.0),
                     (0, 0, 0, 1.0), time, "base_link", "moving_frame")
    counter += 1


def processFeedback(feedback):
    s = "Feedback from marker '" + feedback.marker_name
    s += "' / control '" + feedback.control_name + "'"
    mp = ""
    if feedback.mouse_point_valid:
        mp = " at " + str(feedback.mouse_point.x)
        mp += ", " + str(feedback.mouse_point.y)
        mp += ", " + str(feedback.mouse_point.z)
        mp += " in frame " + feedback.header.frame_id

<<<<<<< HEAD
    if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
        rospy.loginfo(s + ": pose changed")

=======
    if feedback.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
        rospy.loginfo(s + ": button click" + mp + ".")
    elif feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
        rospy.loginfo(s + ": menu item " +
                      str(feedback.menu_entry_id) + " clicked" + mp + ".")
    elif feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
        rospy.loginfo(s + ": pose changed")
    elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_DOWN:
        rospy.loginfo(s + ": mouse down" + mp + ".")
    elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
        rospy.loginfo(s + ": mouse up" + mp + ".")
>>>>>>> 77d66db696dbefefbfe0de9ae07fe9c938b63bf8
    server.applyChanges()


def makeMarkerControl(msg):
    control = InteractiveMarkerControl()
    control.always_visible = True
<<<<<<< HEAD
=======

>>>>>>> 77d66db696dbefefbfe0de9ae07fe9c938b63bf8
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
<<<<<<< HEAD
=======

>>>>>>> 77d66db696dbefefbfe0de9ae07fe9c938b63bf8
    return control


def createMarker(interaction_mode, position):
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "base_link"
    int_marker.pose.position = position
    int_marker.scale = 1

    int_marker.name = "path_node_" + str(nodes_count + 1)
    int_marker.description = "path_node_" + str(nodes_count + 1)
    int_marker.description.center

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
    int_marker.scale = 0.25
    server.insert(int_marker, processFeedback)
    menu_handler.apply(server, int_marker.name)


if __name__ == "__main__":
    rospy.init_node("basic_controls")
    br = TransformBroadcaster()

    # create a timer to update the published transforms
    rospy.Timer(rospy.Duration(0.01), frameCallback)
    server = InteractiveMarkerServer("basic_controls")

<<<<<<< HEAD
=======
    menu_handler.insert("First Entry", callback=processFeedback)
    menu_handler.insert("Second Entry", callback=processFeedback)
    sub_menu_handle = menu_handler.insert("Submenu")
    menu_handler.insert("First Entry", parent=sub_menu_handle,
                        callback=processFeedback)
    menu_handler.insert(
        "Second Entry", parent=sub_menu_handle, callback=processFeedback)

>>>>>>> 77d66db696dbefefbfe0de9ae07fe9c938b63bf8
    position = Point(0, 0, 0)
    createMarker(InteractiveMarkerControl.NONE, position)
    server.applyChanges()
    rospy.spin()
