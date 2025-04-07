#! /usr/bin/env python3
import rospy
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from geometry_msgs.msg import Twist, PointStamped
from visualization_msgs.msg import Marker
import tf.transformations
from actionlib_msgs.msg import GoalStatus
import numpy as np
import rospkg
import math
import tf
from datetime import datetime
import sys
from general_utils import files_utils
import time


np.set_printoptions(suppress=True)
np.set_printoptions(threshold=sys.maxsize)

package_path = rospkg.RosPack().get_path("mapless_nav")
save_path = package_path + "/scripts/TorchModels/data/"
file_name = "data_nav_map"
goal_x, goal_y = 0.0, 0.0
npz_data = []
data_Y = [0.0, 0.0, 0.0]  # [l_vel_x, l_vel_y, a_vel_z]
recording = False
# Value between (5, 20)
samples_average = 10
callback_count = 1
rate = 0
reach_objective = 0
navigating = False

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
        self.marker = Marker()

        self.point.header.frame_id = "map"
        self.point.header.stamp = rospy.Time.now()
        self.point.point = Point(0, 0, 0)
        self.marker.header.frame_id = "map"
        self.marker.header.stamp = rospy.Time.now()
        self.marker.type = Marker.ARROW
        self.marker.scale = Point(0.5, 0.1, 0.1)
        self.marker.color.r = 0.2
        self.marker.color.g = 0.8
        self.marker.color.b = 0.2
        self.marker.color.a = 1.0
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


def stopSaveDataCallback(msg):
    global recording, navigating
    global reach_objective
    
    print("\n>> GOAL status reached:", msg.status)
    if msg.status == 3:
        reach_objective += 1
        recording = True
        if reach_objective == 2:
            reach_objective = 0
            navigating = False


def get_position():
    global listener
    ([x, y, z], rot) = listener.lookupTransform("map", 'base_link', rospy.Time(0))
    angle = 2*math.atan2(rot[2], rot[3])
    angle = angle - 2*math.pi if angle > math.pi else angle
    return x, y, angle


def target_direction():
    global goal_x, goal_y
    
    robot_x, robot_y, robot_a = get_position()
    ang_pos = math.atan2(goal_y-robot_y, goal_x-robot_x)
    distance = math.sqrt((goal_y-robot_y)**2 + (goal_x-robot_x)**2)
    if ang_pos > math.pi:
        ang_pos = ang_pos - 2*math.pi

    angle = ang_pos - robot_a
    if(angle >= math.pi):
        angle = angle - 2*math.pi
    if(angle < -math.pi):
        angle = angle + 2*math.pi
    
    return [distance, angle]


def occGridCallback(msg):
    global npz_data, data_Y
    global samples_average
    global rate, callback_count
    
    callback_count+=1

    data = np.asarray(msg.data, dtype=np.float32)
    data = np.reshape(data, (msg.info.height, msg.info.width))
    data = np.rot90(np.flip(data, axis=0))
    
    d, th = target_direction()
    tgt = np.array([round(d, 2), round(th, 2)])
    tgt = np.asarray(tgt, dtype=np.float32)
    data_Y = np.asarray(data_Y, dtype=np.float32)
    sample = {'features':{'occ_grid':data, 'target':tgt}, 'labels':data_Y}
    """
    # 1m = 20 pixels 
    # data dim(n+2 x n): ch0 nxn matrix is occ_grid 
    # row n+1 = distance_to_target 
    # row n+2 = theta_to_target 
    # vect_ydat dim(3) label info = l_vel_x, l_vel_y, a_vel_z
    """

    if recording:
        t_lim = math.ceil(rate / samples_average)
        if callback_count % t_lim == t_lim-1:
            if abs(data_Y[0]) > 0.1 or abs(data_Y[1]) > 0.1 or abs(data_Y[2]) > 0.1:
                npz_data.append(sample)


def cmdVelCallback(msg):
    global data_Y

    x = round(msg.linear.x, 3)
    y = round(msg.linear.y, 3)
    z = round(msg.angular.z, 3)
    data_Y = np.array([x, y, z])


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
    

def main():
    global listener
    global navigating, recording
    global goal_x, goal_y, npz_data
    global rate, callback_count

    rospy.init_node("justina_occgrid_data_nav_map")
    rospy.loginfo("INITIALIZING justina_occgrid_data_nav_map")
    
    listener = tf.TransformListener()
    listener.waitForTransform("map", "base_link", rospy.Time(), rospy.Duration(4.0))

    rospy.Subscriber('/simple_move/goal_reached', GoalStatus, stopSaveDataCallback)
    rospy.Subscriber("/re_local_occ_grid", OccupancyGrid, occGridCallback)
    rospy.Subscriber("/hardware/mobile_base/cmd_vel", Twist, cmdVelCallback)

    pubHeadPos = rospy.Publisher("/hardware/head/goal_pose", Float64MultiArray, queue_size=1)
    goal_pub = rospy.Publisher('/mapless_nav/goal', PointStamped, queue_size=10)
    marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
    move_goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)

    ([x, y, _], _) = listener.lookupTransform("map", 'base_link', rospy.Time(0))
    goal_x, goal_y = x, y-0.1


    map_msg = rospy.wait_for_message('/augmented_map', OccupancyGrid, timeout=5)
    occ_map = map_info(map_msg)
    msgHeadPos = Float64MultiArray()
    msgHeadPos.data = [0.0, -0.4]
    pubHeadPos.publish(msgHeadPos)
    rospy.sleep(1)
    save_data = False



    loop = rospy.Rate(1)

    while not rospy.is_shutdown():

        start = time.time()
        d, th = target_direction()
        x, y, a = get_position()
        cad = f"(Distancia, Angulo)= ({d:.3f}, {th:.3f})"
        cad += f" || Posicion actual = ({x:.3f}, {y:.3f}, {a:.3f})"

        if not navigating:# and not recording:
            pose = occ_map.get_free_random_pose()
            goal_pub.publish(occ_map.get_point_stamped())
            marker_pub.publish(occ_map.get_marker())
            move_goal_pub.publish(pose)
            print(f"\nNew goal ({pose.pose.position.x:.3f}, {pose.pose.position.y:.3f})")#, end="\r")
            goal_x = pose.pose.position.x
            goal_y = pose.pose.position.y

            if save_data:
                date_time = str(datetime.now())
                date_time = date_time.replace(" ", "_")
                date_time = date_time.replace(":", "-")[:-7]
                path = save_path + file_name + "_" + date_time
                print("\nSave .npz", path)
                print("Samples:", len(npz_data))
                npz_data = np.asarray(npz_data)
                np.savez(path,data=npz_data)
                npz_data = []
                save_data = False

            rospy.sleep(2)
            navigating = True
            recording = True
        else:
            if recording:
                cad += " Recording * "
                save_data = True
            else:
                cad += " No recording"
        
        print(" "*100, end='\r')
        print(cad, end='\r')
        loop.sleep()
        rate = round(callback_count/ (time.time() - start))
        callback_count = 1

if __name__ == "__main__":
     main()