#! /usr/bin/env python3
import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Empty
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Twist, PointStamped
import numpy as np
import rospkg
import math
import tf
import sys
from datetime import datetime

#np.set_printoptions(threshold=sys.maxsize)
np.set_printoptions(suppress=True)

package_path = rospkg.RosPack().get_path("mapless_nav")
save_path = package_path + "/scripts/TorchModels/data/"
file_name = "data_dep1"

lin_vel_x, goal_x, goal_y = 0.0, 0.0, 0.0
ang_vel_z = 1.0
last_goal = [0.0, 0.0]
data_X = [0.0, 0.0]
data_Y = [0.0, 0.0] #TODO: add lin_vel_y
cmd_vel_pub = None
recording = False
y_button = 0
category_y = True

def clickPointCallback(msg):
    global goal_x, goal_y
    goal_x = msg.point.x
    goal_y = msg.point.y
    print(f"\nNew goal ({goal_x:.3f}, {goal_y:.3f})")


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
    global data_X
    data = np.asarray(msg.data)
    data = np.reshape(data, (msg.info.height, msg.info.width))
    other_features = np.zeros(msg.info.height)
    other_features[:2] = target_direction()
    other_features[2:4] = data_Y
    # mat(81x80) ch0 80x80=occ_grid, mat[81]=vect(80) 
    data_X = np.vstack((data, other_features))


def cmdVelCallback(msg):
    global data_Y
    x = msg.linear.x
    z = msg.angular.z
    if category_y:
        if x > 0.5 and z < 0.2:
            data_Y = [x, z]    
    else:
        data_Y = [x, z]
    #print("\n", data_Y)
    

def stopCallback(msg):
    twist_msg = Twist()
    twist_msg.linear.x = 0.0
    twist_msg.angular.z = 0.0
    cmd_vel_pub.publish(twist_msg)


def recordCallback(msg):
    global recording, y_button
    if y_button > 1:
        recording = not recording
        y_button = 0
    y_button += 1


def main():
    global lin_vel_x, ang_vel_z, listener
    global goal_x, goal_y, cmd_vel_pub
    global recording, y_button, category_y
    start = True
    rospy.init_node("justina_occgrid_data")
    rospy.loginfo("INITIALIZING justina_occgrid_data")

    listener = tf.TransformListener()
    listener.waitForTransform("odom", "base_link", rospy.Time(), rospy.Duration(4.0))

    rospy.Subscriber("/clicked_point", PointStamped, clickPointCallback)
    rospy.Subscriber("/cmd_vel", Twist, cmdVelCallback)
    rospy.Subscriber("/local_occ_grid", OccupancyGrid, occGridCallback)
    rospy.Subscriber("/stop", Empty, stopCallback)  # Button (B)
    rospy.Subscriber("/hardware/robot_state/skip_state", Empty, recordCallback)  # Button (Y)
    goal_pub = rospy.Publisher("/NN_goal", Float32MultiArray, queue_size=10)
    cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    msg_pos = Float32MultiArray()
    
    if start:
        ([x, y, _], _) = listener.lookupTransform("odom", 'base_link', rospy.Time(0))
        goal_x, goal_y = x, y-0.1
        start = False

    msg_pos.data = target_direction()
    goal_pub.publish(msg_pos)
    
    loop = rospy.Rate(4)
    npz_data = []
    save_data = False
    while not rospy.is_shutdown():
        msg_pos.data = target_direction()
        goal_pub.publish(msg_pos)
        d, th = target_direction()
        #print(f"(Distancia, Angulo)= ({d:.3f}, {th:.3f})", end='\r')
        x, y, a = get_position()
        #print(f"(Distancia, Angulo)= ({x:.3f}, {y:.3f}, {a:.3f})", end='\r')

        cad = f"(Distancia, Angulo)= ({d:.3f}, {th:.3f})"
        cad += f" || Posicion actual = ({x:.3f}, {y:.3f}, {a:.3f})"
        if recording:
            cad += " Recording * "
            npz_data.append(data_X)
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
            
        print(cad, end='\r')
        #print(cad)
        loop.sleep()
    

if __name__ == "__main__":
     main()
     print()