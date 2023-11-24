#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import rospkg
from std_msgs.msg import Int16
from visualization_msgs.msg import Marker, MarkerArray
import tf
import math
import numpy as np
from hmm_nav2.srv import ViterbiObservationState, ViterbiObservationStateRequest

symbol_read = 0

def create_sphere_marker(px, py, pz=0, count=0, scale=1.0, r=0.19, g=1.0, b=0.0, a=1.0):
    sphere_marker = Marker()
    sphere_marker.header.frame_id = 'map'
    sphere_marker.ns = "Sphere_"  # unique ID
    sphere_marker.id = count
    sphere_marker.type = Marker().SPHERE
    sphere_marker.action = Marker().ADD
    sphere_marker.scale.x = scale
    sphere_marker.scale.y = scale
    sphere_marker.scale.z = scale
    sphere_marker.color.r = r
    sphere_marker.color.g = g
    sphere_marker.color.b = b
    sphere_marker.color.a = a
    sphere_marker.pose.position.x = px
    sphere_marker.pose.position.y = py
    sphere_marker.pose.position.z = pz
    sphere_marker.pose.orientation.x = 0.0
    sphere_marker.pose.orientation.y = 0.0
    sphere_marker.pose.orientation.z = 0.0
    sphere_marker.pose.orientation.w = 1.0
    return sphere_marker


def get_text_marker(id, scale):
    marker = Marker()
    marker.type = Marker.TEXT_VIEW_FACING
    marker.id = id
    marker.scale.x = scale
    marker.scale.y = scale
    marker.scale.z = scale
    marker.header.frame_id = 'map'
    marker.color.a = 1.0
    marker.color.r = 0.6
    marker.color.g = 0.3
    marker.color.b = 0.3
    marker.text = "S:" + str(id)
    return marker


def catch_states_callback(symbol):
    global symbol_read
    symbol_read = symbol.data  # Overwrite at high frequency

    
def get_current_position(listener, centroids):
    listener.waitForTransform('map', 'base_footprint', rospy.Time(0), rospy.Duration(1.0))
    (pose, rotation) = listener.lookupTransform('map', 'base_footprint', rospy.Time(0))
    angle = 2*math.atan2(rotation[2], rotation[3])
    angle = angle - 2*math.pi if angle > math.pi else angle
    pos_marker = create_sphere_marker(pose[0], pose[1], 1.2, 0, 0.2)

    xy_c = centroids[:,:2]
    pos_center = [pose[0], pose[1]]
    
    closest_node = np.argmin(np.linalg.norm(pos_center - xy_c, axis=1))
    close_center = [centroids[closest_node][0], centroids[closest_node][1]]  # closest centroid(x, y)
    closest_marker = create_sphere_marker(close_center[0], close_center[1], scale=0.2, r=1, g=1, b=.32, a=0.8)
    return pos_marker, closest_marker

def show_sequence_states(centroids, states):
    global queue_size
    marker_array = MarkerArray()
    color_gradient = 1.0/len(centroids)
    color = 1.0
    for i, state in enumerate(states.states):
        coords = centroids[state]
        sphere = create_sphere_marker(coords[0], coords[1], 0, i, 0.1, 1-color, 0.2, color)
        sphere.lifetime = rospy.Duration(queue_size*0.66)
        marker_array.markers.append(sphere)
        color -= color_gradient
    
    return marker_array
    


if __name__ == "__main__":
    rospy.init_node('viterbi_clt_node', anonymous=True)

    rospy.wait_for_service("viterbi_hmm2")
    listener = tf.TransformListener()
    
    rospy.Subscriber("/hmm_nav2/Quantized_Symbol/", Int16, catch_states_callback, queue_size=1)
    pub_pos_marker = rospy.Publisher('/hmm_nav2/State_Gt/', Marker, queue_size=1)
    pub_close_marker = rospy.Publisher('/hmm_nav2/State_Closest/', Marker, queue_size=1)
    pub_cstate_marker = rospy.Publisher('/hmm_nav2/State_Calculated/', MarkerArray, queue_size=1)
    try:
        viterbi_service = rospy.ServiceProxy("viterbi_hmm2", ViterbiObservationState)
        print("response ") 
    except rospy.ServiceException as service_ex:
        rospy.logerr("Error %s", service_ex)

    file_path = rospkg.RosPack().get_path('create_dataset') + '/HMM/'
    ccxyth = np.load(file_path + 'ccxyth.npy')
     
    queue_obs = []
    queue_size = 20 # 20 6
    symbol_count = 0

    sleep_rate = 0.5
    while not rospy.is_shutdown():
        gt_marker, close_marker = get_current_position(listener, ccxyth)
        pub_pos_marker.publish(gt_marker)
        pub_close_marker.publish(close_marker)
        
        if (symbol_count > queue_size):
            rospy.loginfo("Response <-")
            states_response = viterbi_service(queue_obs)
            print(states_response)
            states_markers = show_sequence_states(ccxyth, states_response)
            pub_cstate_marker.publish(states_markers)
            symbol_count = 0
            queue_obs = []
        else:
            rospy.loginfo("symbol read %d", symbol_read)
            queue_obs.append(symbol_read)
            symbol_count += 1

        rospy.sleep(sleep_rate)