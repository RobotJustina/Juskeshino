#! /usr/bin/env python3
import rospy
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Twist, PointStamped
import numpy as np
import rospkg
import torch
import math
import tf

#from mapless_nav.scripts.TorchModels.utils import models

package_path = rospkg.RosPack().get_path("mapless_nav")
#model_path = package_path + "/scripts/TorchModels/modelo.pth"
#print("model path: ", model_path)
#model = models.Red_conv(3)
#model.load_state_dict(torch.load(model_path))
#disp = 'cuda' if torch.cuda.is_available() else 'cpu'
#model.to(disp)
lin_vel_x, goal_x, goal_y = 0.0, 0.0, 0.0
ang_vel_z = 1.0
last_goal = [0, 0]


# def occGridArrayCallback(msg):
#     global model, disp, last_goal, lin_vel_x, ang_vel_z
#     """
#     simple_data = msg.data  # Default is a tuple (6400)
#     print(len(simple_data))
#     print(type(simple_data))
#     """
#     # Transform to List and concatenate
#     data_list = list(msg.data)
#     data_list += [0, 0]  # TODO: Concatenate las goal shape:(6402)

#     # print("occ_g_list:", len(data_list))
#     # print("data_list:", type(data_list))
#     # print("---")
#     # Transform to numpy arrray
#     data_np = np.asarray(data_list)

#     # print("data_np shape=", data_np.shape)
#     # print("data_np:", type(data_np))
    
#     # Expand dims (1 batch, flat 6402)
#     data_np = np.expand_dims(data_np, 0)
#     # print("data_np shape=", data_np.shape)
#     # print(">>")
#     # Data to Torch tensor
#     data_x_tensor = torch.tensor(data_np)
#     # Select GPU/CPU and precision f32
#     # data_x_tensor = data_x_tensor.to(torch.device(disp), torch.float32)
    
#     """
#     model.eval()
#     output = model(example_image)
#     """

#     # if(abs(last_goal[0])>0.2):
#     #     with torch.no_grad():
#     #         y_pred = model(data_x_tensor)
#     #     y_pred = y_pred.cpu().numpy()
#     #     lin_vel_x = y_pred[0,0]
#     #     ang_vel_z = y_pred[0,1]
#     # else:
#     #     lin_vel_x = 0.0
#     #     ang_vel_z = 0.0


def goalCallback(msg):
	global last_goal
	last_goal = list(msg.data)

    
def clickPointCallback(msg):
    global goal_x, goal_y
    goal_x = msg.point.x
    goal_y = msg.point.y
    print(f"\nNew goal ({goal_x:.3f}, {goal_y:.3f})")


def target_direction():
    global listener
    global goal_x
    global goal_y
    try:
        ([x, y, z], rot) = listener.lookupTransform("odom", 'base_link', rospy.Time(0))
        angle = 2*math.atan2(rot[2], rot[3])
        angle = angle - 2*math.pi if angle > math.pi else angle
        robot_x = x
        robot_y = y
        robot_a = angle
    except:
        robot_x = 0.0
        robot_y = 0.0
        robot_a = 0.0

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




# TODO: Use this one to make a 80x80 matrix
# def occGridCallback(msg):SingleConvModel()
#     data = np.asarray(msg.data)

#     print("occ_g:", data.shape)


def main():
    global lin_vel_x, ang_vel_z, listener
    global goal_x, goal_y
    rospy.init_node("justina_nav_data")
    rospy.loginfo("INITIALIZING justina_nav_data")

    listener = tf.TransformListener()
    listener.waitForTransform("odom", "base_link", rospy.Time(), rospy.Duration(4.0))

    #rospy.Subscriber("/local_occ_grid_array", Float32MultiArray, occGridArrayCallback)
    rospy.Subscriber("/NN_goal", Float32MultiArray, goalCallback)
    rospy.Subscriber("/clicked_point", PointStamped, clickPointCallback)
    goal_pub = rospy.Publisher("/NN_goal", Float32MultiArray, queue_size=10)
    msg_pos = Float32MultiArray()
    msg_pos.data = [0.0, 0.0]
    goal_pub.publish(msg_pos)
    
    #loc_occ_sub = rospy.Subscriber("/local_occ_grid", OccupancyGrid, occGridCallback)

    cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    twist_msg = Twist()
    
    
    loop = rospy.Rate(20)
    while not rospy.is_shutdown():
        # cmd_vel_pub.publish(twist_msg)
        # twist_msg.linear.x = lin_vel_x
        # twist_msg.angular.z = ang_vel_z
        # cmd_vel_pub.publish(twist_msg)
        goal_pub.publish(msg_pos)
        d, th= target_direction()
        print(f"(Distancia, Angulo)= ({d:.3f}, {th:.3f})", end='\r')
        
        # twist_msg.linear.x = 0.0
        # twist_msg.angular.z = 0.0
        # cmd_vel_pub.publish(twist_msg)

        loop.sleep     

if __name__ == "__main__":
     main()