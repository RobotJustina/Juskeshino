#!/usr/bin/env python3
import rospy
import tf
import tf2_ros
from nav_msgs.msg import Path
from std_msgs.msg import Int8
from geometry_msgs.msg import Point, PointStamped, PoseStamped, Quaternion
from nav_msgs.msg import OccupancyGrid
from juskeshino_tools.JuskeshinoNavigation import JuskeshinoNavigation
import time, math
import numpy as np
import csv
import rospkg
import pandas as pd 

package_path = rospkg.RosPack().get_path("create_dataset")
file_path = package_path + "/scripts/save_occ_grid/" 

navigating = False
point1 = [0, 0]
point2 = [0, 0]
distance = 0
trajectory = []
nav_fails = 0


def goalCallback(msg):
    global navigating, experiments
    global fail_time, nav_fails, nav_time
    global point1, point2, distance, trajectory
    global t0
    
    print("\n>> GOAL status reached:", msg.data)
    if msg.data == 3:
        navigating = False
        nav_fails = 0
        nav_time = time.time()
        print("Timer: ", nav_time - t0)
        d = round(math.dist(point1, point2), 2)
        data = [experiments, round(nav_time - t0, 2), point1, point2, d, True, trajectory]
        if data[1] > 1:
            with open(file_path+'nav_register.csv', 'a') as f:
                writer = csv.writer(f)
                writer.writerow(data)
                rospy.sleep(0.2)
        else:
            experiments -= 1

    if msg.data == 4:
        nav_fails += 1
        fail_time = time.time()
        print(" X -- Fails", nav_fails)
        if nav_fails > 1:
            print("Stop nav")
            nav_fails = 0
            navigating = False
            print("Timer: ", fail_time - t0)
            d = round(math.dist(point1, point2), 2)
            data = [experiments, round(fail_time - t0, 2), point1, point2, d, False, trajectory]
            if data[1] > 1:
                with open(file_path+'nav_register.csv', 'a') as f:
                    writer = csv.writer(f)
                    writer.writerow(data)
                    rospy.sleep(0.2)
            else:
                experiments -= 1


def get_position():
    global listener
    ([x, y, z], rot) = listener.lookupTransform("odom", 'base_link', rospy.Time(0))
    angle = 2*math.atan2(rot[2], rot[3])
    angle = angle - 2*math.pi if angle > math.pi else angle
    return [round(x, 2), round(y, 2)]


def load_free_map():
    print("Load coords ...")
    df_map = pd.read_csv(file_path+"free_map.csv",  sep=",", header=0,index_col=0)
    map = df_map.values
    df_map_inf = pd.read_csv(file_path+"free_map_info.csv",  sep=",", header=0,index_col=0)
    map_info = df_map_inf.to_numpy()
    resolution = float(map_info[0][0])
    string = map_info[1][0]
    string = string[1:-1] # Remove brackets
    center = [float(x) for x in string.split()]
    width = int(map_info[2][0])
    height = int(map_info[3][0])
    map_info = [resolution, center, width, height]

    return map, map_info


def random_goal(free_spaces, map_info):
    # val 0 is free
    res = map_info[0]
    cent = map_info[1]
    max = len(free_spaces)
    selection = int(np.random.uniform(0, max))
    map_x = free_spaces[selection][1]
    map_y = free_spaces[selection][0]
    position = Point(round(map_x, 2), round(map_y, 2), 0)
    x_world = cent[0] + position.x * res
    y_world = cent[1] + position.y * res

    point = PointStamped()
    point.header.frame_id = "map"
    point.header.stamp = rospy.Time.now()
    point.point = Point(round(x_world, 2), round(y_world, 2), 0)
    pose = PoseStamped() 
    pose.header.frame_id = "map"
    pose.header.stamp = rospy.Time.now()
    rot = tf.transformations.quaternion_from_euler(0.0, 0.0, 0.0)
    pose.pose.position = point.point
    pose.pose.orientation = Quaternion(rot[0], rot[1], rot[2], rot[3])
    return pose


def main():
    global navigating, listener, t0
    global point1, point2, nav_fails
    global trajectory, distance
    global experiments

    rospy.init_node('results_time_models')
    rospy.logwarn("results time models")
    JuskeshinoNavigation.setNodeHandle()
    listener = tf.TransformListener()

    rospy.Subscriber('/maples_nav/goal_reached', Int8, goalCallback)

    move_goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
    goal_stat_pub = rospy.Publisher('/maples_nav/goal_reached', Int8, queue_size=1)   

    free_spaces, map_info = load_free_map()
    with open(file_path+'nav_register.csv', 'w') as f:
        writer = csv.writer(f)
        writer.writerow(['Experiment', 'Time', 'origin', 'objective', 'distance', 'arrive', 'trajectory'])
    experiments = 0
    while not rospy.is_shutdown():

        if not navigating:
            pose = random_goal(free_spaces, map_info)
            goal_x = pose.pose.position.x
            goal_y = pose.pose.position.y
            point1 = [round(goal_x, 2), round(goal_y, 2)]
            point2 = get_position()
 
            distance = math.dist(point1, point2)
            if distance < 2.5 and distance > 1.5:
                experiments += 1
                print("\n>> Experiment ", experiments)
                nav_fails = 0
                trajectory = []
                print("distance:", distance)
                move_goal_pub.publish(pose)
                t0 = time.time()
                navigating = True

        else:
            trajectory.append(get_position())
            rospy.sleep(.5)
            timer = time.time() - t0
            if timer > 60:
                goal_stat_pub.publish(4)
                print("Nav Fail, time:", timer)


if __name__ == '__main__':
    main()
