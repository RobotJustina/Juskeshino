#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Twist, PointStamped
import numpy as np
import rospkg
import torch
import math
import tf
import sys

# np.set_printoptions(threshold=sys.maxsize)
np.set_printoptions(suppress=True)

# from mapless_nav.scripts.TorchModels.utils import models

package_path = rospkg.RosPack().get_path("mapless_nav")
# model_path = package_path + "/scripts/TorchModels/modelo.pth"
# print("model path: ", model_path)
# model = models.Red_conv(3)
# model.load_state_dict(torch.load(model_path))
# disp = 'cuda' if torch.cuda.is_available() else 'cpu'
# model.to(disp)
lin_vel_x, goal_x, goal_y = 0.0, 0.0, 0.0
ang_vel_z = 1.0
last_goal = [0.0, 0.0]
data_X = []
data_Y = []


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
    other_features = np.zeros((msg.info.height, msg.info.width))
    other_features[0, :2] = target_direction()
    # img(80x80) ch0, vet(2) ch2
    data_X = np.array([data, other_features], dtype=np.float32)


def cmdVelCallback(msg):
    global data_Y
    data_Y = [msg.linear.x, msg.angular.z]
    

def main():
    global lin_vel_x, ang_vel_z, listener
    global goal_x, goal_y
    start = True
    rospy.init_node("justina_nav_data")
    rospy.loginfo("INITIALIZING justina_nav_data")

    listener = tf.TransformListener()
    listener.waitForTransform("odom", "base_link", rospy.Time(), rospy.Duration(4.0))

    rospy.Subscriber("/clicked_point", PointStamped, clickPointCallback)
    rospy.Subscriber("/cmd_vel", Twist, cmdVelCallback)
    rospy.Subscriber("/local_occ_grid", OccupancyGrid, occGridCallback)
    goal_pub = rospy.Publisher("/NN_goal", Float32MultiArray, queue_size=10)
    msg_pos = Float32MultiArray()
    
    if start:
        ([x, y, _], _) = listener.lookupTransform("odom", 'base_link', rospy.Time(0))
        goal_x, goal_y = x, y-0.1
        start = False

    msg_pos.data = target_direction()
    goal_pub.publish(msg_pos)

    
    loop = rospy.Rate(20)
    while not rospy.is_shutdown():
        # TODO: Delete
        # cmd_vel_pub.publish(twist_msg)
        # twist_msg.linear.x = lin_vel_x
        # twist_msg.angular.z = ang_vel_z
        # cmd_vel_pub.publish(twist_msg)
        msg_pos.data = target_direction()
        goal_pub.publish(msg_pos)
        d, th = target_direction()
        #print(f"(Distancia, Angulo)= ({d:.3f}, {th:.3f})", end='\r')
        x, y, a = get_position()
        #print(f"(Distancia, Angulo)= ({x:.3f}, {y:.3f}, {a:.3f})", end='\r')

        str = f"(Distancia, Angulo)= ({d:.3f}, {th:.3f})"
        str += f" || Posicion actual = ({x:.3f}, {y:.3f}, {a:.3f})"
        #print(str, end='\r')


        loop.sleep
    


if __name__ == "__main__":
     main()
     print()