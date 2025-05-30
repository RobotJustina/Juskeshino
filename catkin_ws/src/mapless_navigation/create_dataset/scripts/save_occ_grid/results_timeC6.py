#!/usr/bin/env python3
import rospy
import tf
import tf2_ros
from nav_msgs.msg import Path
from geometry_msgs.msg import Point, PointStamped, PoseStamped, Quaternion
from nav_msgs.msg import OccupancyGrid
from actionlib_msgs.msg import GoalStatus
from juskeshino_tools.JuskeshinoNavigation import JuskeshinoNavigation
import time, math
import numpy as np
import csv
import rospkg
import torch

package_path = rospkg.RosPack().get_path("create_dataset")
file_path = package_path + "/scripts/save_occ_grid/" 

navigating = False
point1 = [0, 0]
point2 = [0, 0]
distance = 0
trajectory = []
nav_fails = 0
def goalCallback(msg):
    global navigating
    global fail_time, nav_fails, nav_time
    global point1, point2, distance, trajectory
    
    print("\n>> GOAL status reached:", msg.status)
    if msg.status == 3:
        if msg.goal_id.id == "-1":
            navigating = False
            nav_fails = 0
            nav_time = time.time()
            print("Timer: ", nav_time - t0)
            d = round(math.dist(point1, point2), 2)
            data = [round(nav_time - t0, 2), point1, point2, d, True, trajectory]
            with open(file_path+'nav_register.csv', 'a') as f:
                writer = csv.writer(f)
                writer.writerow(data)
    if msg.status == 4:
        nav_fails += 1
        fail_time = time.time()
        print(" X -- Fails", nav_fails)
        #if nav_fails > 2:
        print("Stop nav")
        rospy.sleep(2)
        nav_fails = 0
        navigating = False
        print("Timer: ", fail_time - t0)
        d = round(math.dist(point1, point2), 2)
        data = [round(fail_time - t0, 2), point1, point2, d, False, trajectory]
        with open(file_path+'nav_register.csv', 'a') as f:
            writer = csv.writer(f)
            writer.writerow(data)


def get_position():
    global listener
    ([x, y, z], rot) = listener.lookupTransform("map", 'base_link', rospy.Time(0))
    angle = 2*math.atan2(rot[2], rot[3])
    angle = angle - 2*math.pi if angle > math.pi else angle
    return [round(x, 2), round(y, 2)]




def random_goal(msg):
    # val 0 is free
    res = np.round(msg.info.resolution, 3)
    cent = np.array([msg.info.origin.position.x, msg.info.origin.position.y])
    width = msg.info.width
    height = msg.info.height
    map_grid = np.array(msg.data)
    map_grid = np.reshape(map_grid, (height, width))
    #occ_map = Map(res, cent, width, height, map_grid)   
    free_spaces = np.argwhere(map_grid==0)

    # get_free_random_pose(self)
    # Get free position
    max = len(free_spaces)
    selection = int(np.random.uniform(0, max))
    #index = np.random.randint(0, selection.size)
    #selection = int(selection[index])
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

    rospy.init_node('follow_path')
    rospy.logwarn("follow path")
    JuskeshinoNavigation.setNodeHandle()
    listener = tf.TransformListener()

    rospy.Subscriber('/simple_move/goal_reached', GoalStatus, goalCallback)

    move_goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
    map_msg = rospy.wait_for_message('/augmented_map', OccupancyGrid, timeout=5)
    with open(file_path+'nav_register.csv', 'w') as f:
        writer = csv.writer(f)
        writer.writerow(['Time', 'origin', 'objective', 'distance', 'arrive', 'trajectory'])

    objectives = []
    with open(file_path+'reference_nav_C6.csv', 'r') as f:
        csvFile = csv.reader(f)
        header = True
        for lines in csvFile:
            if header:
                header = False
            else:
                line = lines[2][1:-1].split(',')
                line[0] = float(line[0])
                line[1] = float(line[1])
                objectives.append(line)

    experiments = 0
    while not rospy.is_shutdown():

        if not navigating:
            # pose = random_goal(map_msg)
            # goal_x = pose.pose.position.x
            # goal_y = pose.pose.position.y
            #point1 = [round(goal_x, 2), round(goal_y, 2)]
            x_world = objectives[experiments][0]
            y_world = objectives[experiments][1]

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
            
            point1 = [x_world, y_world]
            point2 = get_position()
 
            distance = math.dist(point1, point2)
            experiments += 1
            print("\n>> Experiment ", experiments)
            nav_fails = 0
            trajectory = []
            print("distance:", distance)
            move_goal_pub.publish(pose)
            t0 = time.time()
            navigating = True
            if experiments == 50:
                rospy.signal_shutdown('')
                print("Exit")
        else:
            trajectory.append(get_position())
            #print(get_position())

            rospy.sleep(.5)
        #print("time:", time.time() - t0)
    


if __name__ == '__main__':
    main()
