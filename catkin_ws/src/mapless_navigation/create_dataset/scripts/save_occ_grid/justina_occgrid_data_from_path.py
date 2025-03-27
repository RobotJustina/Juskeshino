#! /usr/bin/env python3
import rospy
from std_msgs.msg import Bool, Float64MultiArray
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from geometry_msgs.msg import Twist, PointStamped
import numpy as np
import rospkg
import math
import tf
from datetime import datetime
import sys
from general_utils import files_utils
import time

np.set_printoptions(suppress=True)

package_path = rospkg.RosPack().get_path("mapless_nav")
save_path = package_path + "/scripts/TorchModels/data/"
file_name = "data_from_path"
goal_x, goal_y = 0.0, 0.0
npz_data = []
data_Y = [0.0, 0.0, 0.0]  # [l_vel_x, l_vel_y, a_vel_z]
recording = False
# Value between (5, 20)
samples_average = 10
callback_count = 1
rate = 0

def clickPointCallback(msg):
    global goal_x, goal_y
    
    goal_x = msg.point.x
    goal_y = msg.point.y
    print(f"\nNew goal ({goal_x:.3f}, {goal_y:.3f})", end="\r")


def updateGoalCallback(msg):
    global goal_x, goal_y

    goal_x = msg.point.x
    goal_y = msg.point.y
    print(f"\nNew goal ({goal_x:.3f}, {goal_y:.3f})", end="\r")


def recordCallback(msg):
    global recording
    recording = msg.data


def cmdVelCallback(msg):
    global data_Y
    data_Y[1] = round(msg.linear.y, 3)


def getOdomCallback(msg):
    global robot_pos_x, robot_pos_y
    global data_Y, robot_theta

    robot_pos_x = msg.pose.pose.position.x
    robot_pos_y = msg.pose.pose.position.y
    robot_orient_z = msg.pose.pose.orientation.z
    robot_orient_w = msg.pose.pose.orientation.w
    th = 2*math.atan2(robot_orient_z, robot_orient_w)
    if abs(th) > math.pi:
        robot_theta = th - (np.sign(th)*2*math.pi)
    else:
        robot_theta = th

    data_Y[0] = round(msg.twist.twist.linear.x, 3)
    #/odom doesn't have linear.y
    data_Y[2] = round(msg.twist.twist.angular.z, 3)


def getTargetCallback(msg):
    global goal_x, goal_y

    pose = msg.poses[-1]
    goal_x = pose.pose.position.x
    goal_y = pose.pose.position.y


def target_direction():
    global goal_x, goal_y
    global robot_pos_x, robot_pos_y, robot_theta
    
    ang_pos = math.atan2(goal_y-robot_pos_y, goal_x-robot_pos_x)
    distance = math.sqrt((goal_y-robot_pos_y)**2 + (goal_x-robot_pos_x)**2)
    if ang_pos > math.pi:
        ang_pos = ang_pos - 2*math.pi

    angle = ang_pos - robot_theta
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
            npz_data.append(sample)


def main():
    global goal_x, goal_y, npz_data
    global recording
    global robot_pos_x, robot_pos_y, robot_theta
    global rate, callback_count

    rospy.init_node("justina_occgrid_data_from_path")
    rospy.loginfo("INITIALIZING justina_occgrid_data_from_path")

    listener = tf.TransformListener()
    listener.waitForTransform("odom", "base_link", rospy.Time(), rospy.Duration(4.0))

    rospy.Subscriber("/clicked_point", PointStamped, clickPointCallback)
    #rospy.Subscriber("/local_occ_grid", OccupancyGrid, occGridCallback)
    rospy.Subscriber("/re_local_occ_grid", OccupancyGrid, occGridCallback)
    rospy.Subscriber("/simple_move/goal_path", Path, getTargetCallback)
    rospy.Subscriber("/odom", Odometry, getOdomCallback)
    rospy.Subscriber("/hardware/mobile_base/cmd_vel", Twist, cmdVelCallback)
    rospy.Subscriber("mapless_nav/save_dataset", Bool, recordCallback)
    rospy.Subscriber("/mapless_nav/goal", PointStamped, updateGoalCallback)

    pubHeadPos = rospy.Publisher("/hardware/head/goal_pose", Float64MultiArray, queue_size=1)
    
    ([robot_pos_x, robot_pos_y, _], _) = listener.lookupTransform("odom", 'base_link', rospy.Time(0))
    goal_x = robot_pos_x
    goal_y = robot_pos_y -0.1
    robot_theta = 0.0

    msgHeadPos = Float64MultiArray()
    msgHeadPos.data = [0.0, -0.4]
    pubHeadPos.publish(msgHeadPos)
    rospy.sleep(1)
    pubHeadPos.publish(msgHeadPos)

    if not files_utils.DirectoryUtils.existDir(save_path, True):
        rospy.logwarn("creating folder" + save_path)
        files_utils.DirectoryUtils.createDir(save_path, True)

    loop = rospy.Rate(1)
    npz_data = []
    save_data = False
    while not rospy.is_shutdown():
        start = time.time()
        d, th = target_direction()
        cad = f"(Distancia, Angulo)= ({d:.3f}, {th:.3f})"
        cad += f" || Posicion actual = ({robot_pos_x:.3f}, {robot_pos_y:.3f}, {robot_theta:.3f})"
        if recording:
            cad += " Recording * "
            save_data = True
        else:
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
            cad += " No recording"
                
        print('', end="\r")  # clear whole line
        sys.stdout.write('\x1b[2K')
        print(cad, end='\r')
        loop.sleep()
        rate = round(callback_count/ (time.time() - start))
        callback_count = 1


if __name__ == "__main__":
     main()