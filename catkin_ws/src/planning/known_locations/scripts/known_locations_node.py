#!/usr/bin/env python
import yaml
import math
import rospy
import tf
from planning_msgs.msg import *
from planning_msgs.srv import *
from visualization_msgs.msg import MarkerArray, Marker 

def load_from_yaml(locs_file):
    print("KnownLocations.->Loading known locations from file: " + locs_file)
    try:
        f = open(locs_file,'r')
        locations = yaml.safe_load(f)
    except:
        locations = {}
        print("KnownLocations.->Cannot load locations from file " + locs_file)
        exit(-1)
    for k in locations.keys():
        loc = Location()
        loc.name = k
        loc.pose.position.x = locations[k]["x"]
        loc.pose.position.y = locations[k]["y"]
        loc.pose.orientation.w = math.cos(locations[k]["theta"]/2)
        loc.pose.orientation.z = math.sin(locations[k]["theta"]/2)
        loc.parent = locations[k]["parent"]
        locations[k] = loc 
    return locations

def save_to_yaml(locs_file, locations):
    print("KnownLocations.->Saving known locations to file: " + locs_file)
    locs_dict = {}
    for k in locations.keys():
        a = math.atan2(locations[k].pose.orientation.z, locations[k].pose.orientation.w) * 2
        locs_dict[k] = {"x":locations[k].pose.position.x,
                        "y":locations[k].pose.position.y,
                        "theta": a,
                        "parent": locations[k].parent}
    try:
        f = open(locs_file, 'w')
        yaml.dump(locs_dict, f, sort_keys=False)
        return True
    except:
        return False

def set_known_location(location):
    global locations, locs_file
    q = location.pose.orientation
    if location.name.isidentifier():
        if abs(q.x**2 + q.y**2 + q.z**2 + q.w**2 - 1.0) < 0.01:
            locations[location.name] = location
            save_to_yaml(locs_file, locations)
            return True
        else:
            print("KnownLocations.->Invalid quaternion value for orientation!!!!")
    else:
        print("KnownLocations.->Invalid location name!!!!")
    return False

def set_known_location_callback(req):
    success = set_known_location(req.location)
    return SetLocationResponse(success=success)

def set_known_locations_callback(req):
    success = True
    for loc in req.locations:
        success = success and set_known_location(loc)
    return SetLocationResponse(success=success)

def get_all_known_locations_callback(req):
    global locations
    return GetLocationsResponse(locations=locations.values())

def get_known_location_callback(req):
    global locations
    try:
        loc = locations[req.name]
        return GetLocationResponse(location=loc)
    except:
        return None

def main():
    global locations, locs_file
    print("INITIALIZING KNOWN LOCATIONS NODE (In an Oscarly manner)...")
    rospy.init_node("known_locations")
    locs_file = rospy.get_param("~locations", "../../../config_files/navigation/known_locations.yaml")
    pub_markers = rospy.Publisher("/planning/known_location_markers", MarkerArray, queue_size=1)
    rospy.Service("/planning/set_known_location", SetLocation, set_known_location_callback)
    rospy.Service("/planning/set_known_locations", SetLocations, set_known_locations_callback)
    rospy.Service("/planning/get_all_known_locations", GetLocations, get_all_known_locations_callback)
    rospy.Service("/planning/get_known_location", GetLocation, get_known_location_callback)
    br = tf.TransformBroadcaster()
    loop = rospy.Rate(5)

    locations = load_from_yaml(locs_file)
    mrks = MarkerArray()
    id = 0
    for k in locations.keys():
        mrk  = Marker()
        mrk.header.frame_id = "map"
        mrk.ns = "known_locations"
        mrk.id = id
        mrk.type = Marker.TEXT_VIEW_FACING
        mrk.action = Marker.ADD
        mrk.pose = locations[k].pose
        mrk.scale.z = 0.3
        mrk.color.b = 1.0
        mrk.color.a = 1.0
        mrk.text = k
        id += 1
        mrks.markers.append(mrk)
    
    while not rospy.is_shutdown():
        for k in locations.keys():
            p = locations[k].pose.position
            q = locations[k].pose.orientation
            br.sendTransform((p.x, p.y, p.z),(q.x, q.y, q.z, q.w), rospy.Time.now(), "location_"+k, "map")
        pub_markers.publish(mrks)
        loop.sleep()

if __name__ == "__main__":
    main()
