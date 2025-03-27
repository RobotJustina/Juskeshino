#! /usr/bin/env python3
import rospy
from std_msgs.msg import Float32MultiArray, Float64MultiArray
from geometry_msgs.msg import Twist, PointStamped
from nav_msgs.msg import OccupancyGrid, Odometry
import numpy as np
import rospkg
import torch

from TorchModels.utils import models


import TorchModels.utils.utilities as l_util 

# >>>Select model
n_channels = 4

occ_grid_meters = 4
model = models.Param_CNN(channels=n_channels, img_size=int(20*occ_grid_meters))

package_path = rospkg.RosPack().get_path("mapless_nav")
model_path = package_path + "/scripts/TorchModels/" 
model_path += model.name + ".pth"
print("model path: ", model_path)
model.load_state_dict(torch.load(model_path))
disp = 'cuda' if torch.cuda.is_available() else 'cpu'
model.to(disp)
last_goal = [0, 0] # d, th
rospack = rospkg.RosPack()
init_time = -1.0
linx = 0.0
liny = 0.0
angz = 0.0
target_reached = True
robot_pos_x, robot_pos_y = 0, 0
speed_factor = 2



def callback_goal(msg):
    global last_goal, target_reached
    if target_reached:
        last_goal[0] = 0.0
        last_goal[1] = 0.0
    else:
        last_goal = list(msg.data)


def callback_point(msg):
    global target_reached

    print(f"\nNew goal: ({msg.point.x:.3f}, {msg.point.y:.3f})", end="\r") 
    print(end='\n')
    target_reached = False


# def occ_grid_image(image=None, occ_grid_meters=4):
    
    
#     return image


def occGridCallback(msg):
    global data_X, linx, liny, angz
    global model, last_goal, disp, init_time
    global target_reached, speed_factor
    
    data = np.asarray(msg.data)
    rows = msg.info.height
    data = np.reshape(data, (rows, rows))
    data = np.rot90(np.flip(data, axis=0))


    #other_features = np.zeros(msg.info.height)
    #other_features[:2] = [round(last_goal[0], 2), round(last_goal[1], 2)]
    other_features = np.array([np.ones(rows)*round(last_goal[0], 2), 
                       np.ones(rows)*round(last_goal[1], 2)], dtype=np.float32)
    """
    # 1m = 20 pixels 
    # data dim(n+2 x n): ch0 nxn matrix is occ_grid 
    # row n+1 = distance_to_target 
    # row n+2 = theta_to_target 
    # vect_ydat dim(3) label info = l_vel_x, l_vel_y, a_vel_z
    """
    #print("entr shape", data.shape)
    #print("model shape", model.img_shape)
    #l_util.show_image_gray(data)


    data_X = np.vstack((data, other_features))
    entrada = np.asarray(data_X)

    # Config input to model



    #print("entr shape", entrada.shape)
    #l_util.show_image_gray(entrada, "vstak")

    #print("entrada", entrada.shape)
    batch = []
    for i in range(n_channels):
        batch.append(entrada)
    entrada = np.array(batch)
    #entrada = np.expand_dims(entrada, axis=0)
    entrada = np.expand_dims(entrada, axis=0)
    

    #print("entrada2", entrada.shape)
    x_ent = torch.tensor(entrada)
    x_ent = x_ent.to(torch.device(disp), torch.float32)

    # print("last_goal", abs(last_goal[0]))
    if (abs(last_goal[0]) > 0.3):
        with torch.no_grad():
            y_pred = model(x_ent)
        y_pred = y_pred.cpu().numpy()[0]

        # advance = False
        # if advance:
        #     print('last_goal', last_goal)
        #     if abs(last_goal[1]) < 0.5:
        #         linx = y_pred[0] * speed_factor
        #     else:
        #         linx = y_pred[0] / (10*abs(last_goal[1]))
        #         print("XXXXXXXXXXXXXXXX linx", linx, end='\n')
        #         print("XXXXXXXXXXXXXXXX linx", linx, end='\n')
                
        #     #linx = 0
        #     if linx > 1:
        #         linx = 1
        # else:
        #     linx = 0

        #linx = speed_factor*( (linx+1)/2 )
        #linx = 0.2
        linx = y_pred[0]

        """Deleted"""
        #liny = y_pred[1]

        """changed"""
        angz = y_pred[1]
        #angz = y_pred[2]
        #angz = (angz -0.5)*100
        # if angz > 2:
        #     angz = 2
        #     linx = 0.01
        # elif angz < -2:
        #     angz = -2
        #     linx = 0.01

    else:
        linx = 0.0
        liny = 0.0
        angz = 0.0
        if last_goal[0] != 0 and last_goal[1] != 0:
            target_reached = True


def getOdomCallback(msg):
    global robot_pos_x, robot_pos_y
    global robot_theta

    robot_pos_x = msg.pose.pose.position.x
    robot_pos_y = msg.pose.pose.position.y
    

def shutdown_stop():
    global pub_cmd
    pub_cmd.publish(Twist())


def main():
    global linx, liny, angz
    global pub_cmd, last_goal
    global target_reached
    rospy.init_node("NN_out")
    rospy.Subscriber("/NN_goal", Float32MultiArray, callback_goal)
    rospy.Subscriber("/clicked_point", PointStamped, callback_point)
    rospy.Subscriber("/odom", Odometry, getOdomCallback)
    rospy.Subscriber("/local_occ_grid", OccupancyGrid, occGridCallback)
    pub_cmd = rospy.Publisher(
        "/hardware/mobile_base/cmd_vel", Twist, queue_size=10)
    pubHeadPos = rospy.Publisher("/hardware/head/goal_pose", Float64MultiArray, queue_size=1)
    print("NN_out has been started")
    
    loop = rospy.Rate(5)
    msg = Twist()
    msgHeadPos = Float64MultiArray()
    msgHeadPos.data = [0.0, -0.4]
    pubHeadPos.publish(msgHeadPos)
    rospy.sleep(0.5)
    pubHeadPos.publish(msgHeadPos)
    cad = ""
    while not rospy.is_shutdown():
        msg.linear.x = linx
        #msg.linear.y = liny
        msg.angular.z = angz
        d = last_goal[0]
        th = last_goal[1]
        cad = f"trg({d:.4f}, {th:.4f}) || "
        cad += f"vel(l_x, a_z) = ({linx:.3f},{angz:.3f})"
        if target_reached:
            cad += " > target reached"
        pub_cmd.publish(msg)
        
        print(" "*100, end='\r')
        print(cad, end='\r')
        rospy.on_shutdown(shutdown_stop)
        loop.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
