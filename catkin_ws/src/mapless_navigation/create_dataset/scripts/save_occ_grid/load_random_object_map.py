#! /usr/bin/env python3
import rospy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker
import tf.transformations
import numpy as np
import rospkg
import math
import tf
import sys
from gazebo_msgs.srv import SpawnModel, SpawnModelRequest, SpawnModelResponse
from copy import deepcopy
import os
import argparse

np.set_printoptions(suppress=True)
np.set_printoptions(threshold=sys.maxsize)

parser = argparse.ArgumentParser()
parser.add_argument("--n_objects", default="5", type=str, help="Number of objects to spawn, default: 5")
args = parser.parse_args(rospy.myargv()[1:])


package_path = rospkg.RosPack().get_path("gazebo_envs")
objects_path = package_path + '/models/'
model_objects = ['kitchen_chair/', 'high_shelf/', 'kitchen_lowtable/', 
           'kitchen_table/', 'sofa-fix/', 'wagon/', 'Desk_01/',
           'Desk_02/', 'Desk_03/', 'Desk_04/', 'Shelf_02/',
           'Table_01/', 'Table_02/', 'Table_03/', 'Table_04/',
           'Table_05/', 'ToolBox_red/', 'openable_living_sideboard/']


class Map:
    def __init__(self, resolution, center, w, h, occ_mat):
        self.resolution = resolution
        self.center = center
        self.width = w
        self.height = h
        self.map = occ_mat
        # 0: Free, 100: occ, -1: unknown
        self.free_spaces = np.argwhere(self.map==0) 
        self.point = PointStamped()
        self.pose = PoseStamped()
        self.marker = new_marker()
        self.temp = 0
        # init point
        self.point.header.frame_id = "map"
        self.point.header.stamp = rospy.Time.now()
        self.point.point = Point(0, 0, 0)
        # init pose
        self.pose.header.frame_id = "map"
        self.pose.header.stamp = rospy.Time.now()

    def map_to_world(self, coord):
        x_world = self.center[0] + coord.x * self.resolution
        y_world = self.center[1] + coord.y * self.resolution
        return x_world, y_world
    
    def get_free_random_pose(self):
        # Get free position
        max = len(self.free_spaces)
        selection = np.random.uniform(0, max, max//10)
        index = np.random.randint(0, selection.size)
        selection = int(selection[index])
        map_x = self.free_spaces[selection][1]
        map_y = self.free_spaces[selection][0]
        position = Point(round(map_x, 2), round(map_y, 2), 0)
        x_world, y_world = self.map_to_world(position)
        # Collision point for testing: 3.15 0.35
        self.point.point = Point(round(x_world, 2), round(y_world, 2), 0)
        # Random rotation
        rot_z = (np.random.random() *2* math.pi) - math.pi     
        rot = tf.transformations.quaternion_from_euler(0.0, 0.0, np.round(rot_z, 2))
        self.pose.pose.position = self.point.point
        self.pose.pose.orientation = Quaternion(rot[0], rot[1], rot[2], rot[3])
        return self.pose
    
    def get_point_stamped(self):
        return self.point
    
    def get_marker(self):
        self.marker.pose = self.pose.pose
        return self.marker


# Functions
def new_marker():
    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()
    marker.type = Marker.ARROW
    marker.scale = Point(0.5, 0.1, 0.1)
    marker.color.r = 0.2
    marker.color.g = 0.8
    marker.color.b = 0.2
    marker.color.a = 1.0
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.header.stamp = rospy.Time.now()
    pose.pose.position =  Point(0, 0, 0.1)
    pose.pose.orientation = Quaternion(0, 0, 0, 1)
    return marker


def map_info(msg):
    # val 0 is free
    print(type(msg))
    res = np.round(msg.info.resolution, 3)
    cent = np.array([msg.info.origin.position.x, msg.info.origin.position.y])
    width = msg.info.width
    height = msg.info.height
    map_grid = np.array(msg.data)
    map_grid = np.reshape(map_grid, (height, width))
    occ_map = Map(res, cent, width, height, map_grid)
    return occ_map
    

def create_object_request(sdf_model, model_name, pose):
    obj = deepcopy(sdf_model)
    spw_req = SpawnModelRequest()
    spw_req.model_name = model_name
    spw_req.model_xml = obj
    spw_req.initial_pose.position = pose.pose.position
    spw_req.initial_pose.orientation = pose.pose.orientation
    return spw_req


def load_files(path):
    for file_name in os.listdir(path):
        if file_name.endswith('.sdf'):
            print(path+file_name)
            try:
                with open(path + file_name, 'r') as file:
                    sdf = file.read()
                file.close()
                return sdf
            except Exception as e:
                str = "An error occurred:" + e
                rospy.logerr(str)
                return None


def spawn_position():
    map_msg = rospy.wait_for_message('/map', OccupancyGrid, timeout=5)
    occ_map = map_info(map_msg)
    pose = occ_map.get_free_random_pose()
    return pose


def main():
    rospy.init_node("load_random_object_map")
    rospy.loginfo("INITIALIZING load_random_object_map")
    
    spawn_srv = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    rospy.loginfo("Waiting for /gazebo/spawn_sdf_model service...")
    spawn_srv.wait_for_service()
    rospy.loginfo("Connected to service!")

    objects_count = np.zeros(len(model_objects), dtype=int)
    print("Objects to spawn:", int(args.n_objects))

    max_objects = int(args.n_objects)
    for i in range(max_objects):
        obj_indx = np.random.randint(0, len(model_objects))
        objects_count[obj_indx] = objects_count[obj_indx] + 1
        name = model_objects[obj_indx][:-1] + '_' + str(objects_count[obj_indx])
        print(obj_indx, name)
        pose = spawn_position()
        sdf_object = load_files(objects_path + model_objects[obj_indx])
        spw_req = create_object_request(sdf_object, name, pose) 
        spawn_srv.call(spw_req)

    print(objects_count)
    rospy.sleep(1.0)


if __name__ == "__main__":
     main()