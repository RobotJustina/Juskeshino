#! /usr/bin/env python3
import rospy
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Empty
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Twist, PointStamped
import numpy as np
import rospkg
import math
import tf
import sys
from datetime import datetime
import tty
import termios
from select import select
from general_utils import files_utils

#np.set_printoptions(threshold=sys.maxsize)
np.set_printoptions(suppress=True)

package_path = rospkg.RosPack().get_path("mapless_nav")
save_path = package_path + "/scripts/TorchModels/data/"
file_name = "data_controller"

npz_data = []
goal_x, goal_y = 0.0, 0.0
ang_vel_z, lin_vel_x = 0.0, 0.0
last_goal = [0.0, 0.0]
data_Y = None
cmd_vel_pub = None
recording = False
category_y = False


def clickPointCallback(msg):
    global goal_x, goal_y
    goal_x = msg.point.x
    goal_y = msg.point.y
    print(f"\nNew goal ({goal_x:.3f}, {goal_y:.3f})")


def get_key(set, timeout):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select([sys.stdin], [], [], timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, set)
    return key


def get_position():
    global listener
    ([x, y, z], rot) = listener.lookupTransform("odom", 'base_link', rospy.Time(0))
    angle = 2*math.atan2(rot[2], rot[3])
    angle = angle - 2*math.pi if angle > math.pi else angle
    return x, y, angle


def target_direction():
    global goal_x
    global goal_y
    
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
    global data_Y

    data = np.asarray(msg.data, dtype=np.float32)
    data = np.reshape(data, (msg.info.height, msg.info.width))
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
        npz_data.append(sample)


def cmdVelCallback(msg):
    global data_Y
    x = round(msg.linear.x, 3)
    y = round(msg.linear.y, 3)
    z = round(msg.angular.z, 3)
    data_Y = np.array([x, y, z])
    

def stopCallback(msg):
    cmd_vel_pub.publish(Twist())


def main():
    global lin_vel_x, ang_vel_z, listener
    global goal_x, goal_y, cmd_vel_pub
    global recording
    global data_Y, npz_data
    
    start = True
    rospy.init_node("justina_occgrid_data")
    rospy.loginfo("INITIALIZING justina_occgrid_data")

    listener = tf.TransformListener()
    listener.waitForTransform("odom", "base_link", rospy.Time(), rospy.Duration(4.0))

    rospy.Subscriber("/clicked_point", PointStamped, clickPointCallback)
    rospy.Subscriber("/hardware/mobile_base/cmd_vel", Twist, cmdVelCallback)
    rospy.Subscriber("/local_occ_grid", OccupancyGrid, occGridCallback)
    rospy.Subscriber("/stop", Empty, stopCallback)  # Button (B)

    cmd_vel_pub = rospy.Publisher("/hardware/mobile_base/cmd_vel", Twist, queue_size=10)
    pubHeadPos = rospy.Publisher("/hardware/head/goal_pose", Float64MultiArray, queue_size=1)
    
    set = termios.tcgetattr(sys.stdin)
    key_timeout = rospy.get_param("~key_timeout", 0.5)
    if start:
        ([x, y, _], _) = listener.lookupTransform("odom", 'base_link', rospy.Time(0))
        goal_x, goal_y = x, y-0.1
        start = False
        msgHeadPos = Float64MultiArray()
        msgHeadPos.data = [0.0, -0.4]
        pubHeadPos.publish(msgHeadPos)
        rospy.sleep(0.5)
        pubHeadPos.publish(msgHeadPos)

    if not files_utils.DirectoryUtils.existDir(save_path, True):
        rospy.logwarn("creating folder" + save_path)
        files_utils.DirectoryUtils.createDir(save_path, True)

    rospy.logwarn("Save Y as 3d vector (linv_x, linv_y, Avel_z)")
    data_Y = [0.0, 0.0, 0.0]
    
    loop = rospy.Rate(15)
    save_data = False
    
    while not rospy.is_shutdown():
        key = get_key(set, key_timeout)
        key = key.lower()
        if key == 's' or key == ' ':
            recording = not recording
        if key == 'q':
            print(key)
            rospy.logwarn("Exit selected")
            rospy.signal_shutdown('')
        
        d, th = target_direction()
        x, y, a = get_position()
        cad = f"(Distancia, Angulo)= ({d:.3f}, {th:.3f})"
        cad += f" || Posicion actual = ({x:.3f}, {y:.3f}, {a:.3f})"
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
        
        print(" "*100, end='\r')
        print(cad, end='\r')
        loop.sleep()
    

if __name__ == "__main__":
     main()
     print()