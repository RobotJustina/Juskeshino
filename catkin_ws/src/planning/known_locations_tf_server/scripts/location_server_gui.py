#!/usr/bin/env python3

import rospy
import rospkg
import yaml
import tf2_ros
import tf
# import math
# import numpy as np
# from std_msgs.msg import String
from known_locations_tf_server.srv import *
from geometry_msgs.msg import TransformStamped
# from copy import deepcopy
# from utils.know_utils import *

class LocationServer:
    def __init__(self):
        # Node initialization
        rospy.init_node('location_server')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.tf_static_broadcaster = tf2_ros.StaticTransformBroadcaster()
        # Get params known location and receptionist knowledge files
        self.file_name = rospy.get_param("~locations_file_name", "/known_locations.yaml")
        self.knowledge_file = rospy.get_param("~knowledge_file_name", "/receptionist_knowledge.yaml")
        self.rospack = rospkg.RosPack()
        self.known_locations = self.load_file(self.file_name)
        self.knowledge = self.load_file(self.knowledge_file)
        rospy.loginfo(f"Known locations server available using {self.file_name} and {self.knowledge_file}")
        
        # Publicar las tf's e inicializar los servicios
        self.publish_all_tf()
        self.location_add_service = rospy.Service('/known_location_add', Locations_server, self.handle_location_add)
        self.knowledge_place_add_service = rospy.Service('/knowledge_place_add', Locations_server, self.handle_knowledge_place_add)
        rospy.loginfo("Known locations detection service available")
        rospy.loginfo("Services: /known_location_add , /knowledge_place_add ")
        rospy.spin()


    #Read known locations file 
    def load_file(self, file_name):
        file_path = self.rospack.get_path('config_files') + file_name
        try:
            with open(file_path, 'r') as file:
                file_content = yaml.safe_load(file)
                if not file_content:
                    rospy.logwarn("No known locations found in YAML file.")
                    return {}
                else:
                    return file_content
        except FileNotFoundError:
            rospy.logwarn(f"File '{file_path}' not found. Creating a new one.")
            return {}
        

    def write_yaml(self, file_name, data):
        try:
            file_path = self.rospack.get_path('config_files') + file_name
            rospy.loginfo("escibiendo...")
            with open(file_path, 'w') as file:
                yaml.dump(data, file, default_flow_style=False)
            return True
        except Exception as e:
            rospy.logerr(f"Error writing to YAML file '{file_name}': {str(e)}")
            return False
    
    #publish tf's
    def publish_all_tf(self):
        self.publish_tf_locations(self.known_locations)
        self.publish_tf_knowledge(self.knowledge)

    def publish_tf_locations(self, known_locations):
        for location_name, data in known_locations.items():
            #print(data)
            pose = [data[0]['x'], data[1]['y'], 0]  # Z coordinate assumed to be 0
            quat = [data[3]['qx'], data[4]['qy'], data[5]['qz'], data[6]['qw']]
            transform = self.create_transform(pose, quat, location_name)
            self.tf_static_broadcaster.sendTransform(transform)
            rospy.sleep(0.03)

    def publish_tf_knowledge(self, knowledge):
        for place_name, place_data in knowledge["Places"].items():
            pose = [place_data["location"]["x"], place_data["location"]["y"], 0]
            quat = tf.transformations.quaternion_from_euler(0, 0, place_data["location"]["theta"])
            transform = self.create_transform(pose, quat, place_name)
            self.tf_static_broadcaster.sendTransform(transform)
            rospy.sleep(0.03)

    def publish_tf_by_data(self, pose, quat, tf_name):
        transform = self.create_transform(pose, quat, tf_name)
        self.tf_static_broadcaster.sendTransform(transform)

    def create_transform(self, pose, quat, child_frame, parent_frame='map'):
        transform = TransformStamped()
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = parent_frame
        transform.child_frame_id = child_frame
        transform.transform.translation.x = pose[0]
        transform.transform.translation.y = pose[1]
        transform.transform.translation.z = pose[2]
        transform.transform.rotation.x = quat[0]
        transform.transform.rotation.y = quat[1]
        transform.transform.rotation.z = quat[2]
        transform.transform.rotation.w = quat[3]
        return transform
    
    def get_robot_location(self):
        pose  = None
        quat = None
        succ = False
        try:
            transform = self.tf_buffer.lookup_transform('map', 'base_link', rospy.Time(0),)
            pose = [transform.transform.translation.x, transform.transform.translation.y, 0]
            quat = [transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w]
            succ = True

            print(pose, quat, succ)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.ERROR("Failed to get robot position")
        finally:
            return succ, pose, quat 


    def handle_location_add(self, req):
        resp = Locations_serverResponse()
        succ, pose, quat = self.get_robot_location()
        print(succ, pose)
        if succ:
            location_name = req.location_name
            self.known_locations[location_name] = [
                {'x': round(pose[0], 2)}, {'y': round(pose[1], 2)}, {'theta': round(tf.transformations.euler_from_quaternion(quat)[2], 2)},
                {'qx': quat[0]}, {'qy': quat[1]}, {'qz': quat[2]}, {'qw': quat[3]}
            ]
            resp.success = self.write_yaml(self.file_name, self.known_locations)
            self.publish_tf_by_data(pose, quat, location_name)
        else:
            resp.success = False
        return resp

    def handle_knowledge_place_add(self, req):
        resp = Locations_serverResponse()
        succ, pose, quat = self.get_robot_location()
        if succ:
            place_number = len(self.knowledge["Places"])  # Calculate the next place number
            place_name = f"Place_{place_number}"
            self.knowledge["Places"][place_name] = {
                "location": {
                    "x": round(pose[0], 2),
                    "y": round(pose[1], 2),
                    "theta": round(tf.transformations.euler_from_quaternion(quat)[2], 2)
                },
                "occupied": 'None'
            }
            resp.success = self.write_yaml(self.knowledge_file, self.knowledge)
            self.publish_tf_by_data(pose, quat, place_name)
        else:
            resp.success = False
        return resp

if __name__ == "__main__":
    try:
        LocationServer()
    except rospy.ROSInterruptException:
        pass
