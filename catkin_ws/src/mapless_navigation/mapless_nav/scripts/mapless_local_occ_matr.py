#! /usr/bin/env python3
import rospy
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist, PointStamped
from nav_msgs.msg import OccupancyGrid, Odometry
import numpy as np
import rospkg
import torch

from TorchModels.utils import models

package_path = rospkg.RosPack().get_path("mapless_nav")
model_path = package_path + "/scripts/TorchModels/CNN_Reg3out.pth"
print("model path: ", model_path)
model = models.CNN_Reg3out()
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
speed_factor = 1

# def callback_grid(msg):
#     # print("callback_grid")
#     global model, last_goal, disp, linx, angz, C, init_time
#     grid = list(msg.data)
#     entrada = grid+last_goal
#     entrada = np.asarray(entrada)
#     entrada = np.expand_dims(entrada, axis=0)
#     x_ent = torch.tensor(entrada)
#     x_ent = x_ent.to(torch.device(disp), torch.float32)
#     # print("last_goal", abs(last_goal[0]))
#     if (abs(last_goal[0]) > 0.3):
#         # print("on if")
#         with torch.no_grad():
#             y_pred = model(x_ent)
#         y_pred = y_pred.cpu().numpy()
#         index = int(np.argmax(y_pred))
#         linx = C[index, 0]
#         angz = C[index, 1]
#     else:
#         # print("on else")
#         # linx=0.0
#         # angz=0.0
#         if (init_time != -1.0 and (linx+angz) > 0):
#             tiempo = rospy.get_time()
#             print(f"inicio: {init_time}, {tiempo}")
#             tiempo = tiempo-init_time
#             print(f"Time: {tiempo}")
#             init_time = -1.0
#         linx = 0.0
#         angz = 0.0
#     print()


def callback_goal(msg):
    # print("callback_goal")
    global last_goal, target_reached
    global robot_pos_x, robot_pos_y
    
    # print("last_goal", last_goal)
    # print("msg.data", msg.data)
    if target_reached:
        last_goal[0] = 0.0
        last_goal[1] = 0.0
        print("> target reached")
    else:
        last_goal = list(msg.data)


def callback_point(msg):
    global init_time, target_reached
    init_time = rospy.get_time()
    # print(f"Init time: {init_time}")
    target_reached = False
    print(f"New goal: {msg.point.x, msg.point.y}")


def occGridCallback(msg):
    global data_X, linx, liny, angz
    global model, last_goal, disp, init_time

    global target_reached, speed_factor
    
    data = np.asarray(msg.data)
    data = np.reshape(data, (msg.info.height, msg.info.width))
    other_features = np.zeros(msg.info.height)
    other_features[:2] = [round(last_goal[0], 2), round(last_goal[1], 2)]

    """
    # MAT dim(81x80): ch0 80x80=occ_grid, mat[81]=vect_ydat dim(80)
    # 80x80 matrix is occ_grid data, row 81 is a vect_ydat with label info
    # vect_ydat dim(80) = distance_to_target, theta_to_target, zeros...
    """
    data_X = np.vstack((data, other_features))
    entrada = np.asarray(data_X)

    entrada = np.expand_dims(entrada, axis=0)
    entrada = np.expand_dims(entrada, axis=0)
    print("entrada", entrada.shape)
    x_ent = torch.tensor(entrada)
    x_ent = x_ent.to(torch.device(disp), torch.float32)

    # print("last_goal", abs(last_goal[0]))
    if (abs(last_goal[0]) > 0.3):
        print("++ on if")
        with torch.no_grad():
            y_pred = model(x_ent)
        # y_pred=  l_vel_x, l_vel_y, a_vel_z
        y_pred = y_pred.cpu().numpy()[0]
        print("y_pred", y_pred)
        y_pred*= speed_factor 
        linx = y_pred[0]
        liny = y_pred[1]
        angz = y_pred[2]

    else:
        print("-- else")
        if (init_time != -1.0 and (linx+angz) > 0):
            tiempo = rospy.get_time()
            print(f"inicio: {init_time}, {tiempo}")
            tiempo = tiempo-init_time
            print(f"Time: {tiempo}")
            init_time = -1.0
        linx = 0.0
        liny = 0.0
        angz = 0.0
        target_reached = True
    print()


def getOdomCallback(msg):
    global robot_pos_x, robot_pos_y
    global robot_theta

    robot_pos_x = msg.pose.pose.position.x
    robot_pos_y = msg.pose.pose.position.y
    # robot_orient_z = msg.pose.pose.orientation.z
    # robot_orient_w = msg.pose.pose.orientation.w
    # th = 2*math.atan2(robot_orient_z, robot_orient_w)
    # if abs(th) > math.pi:
    #     robot_theta = th - (np.sign(th)*2*math.pi)
    # else:
    #     robot_theta = th
    print(f"robot ({robot_pos_x:.2f},{robot_pos_y:.2f})")
    

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
    # rospy.Subscriber("/local_occ_grid_array", Float32MultiArray, callback_grid)
    rospy.Subscriber("/local_occ_grid", OccupancyGrid, occGridCallback)
    pub_cmd = rospy.Publisher(
        "/hardware/mobile_base/cmd_vel", Twist, queue_size=10)
    # pub_cmd = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    print("NN_out has been started")
    loop = rospy.Rate(1)
    msg = Twist()
    
    while not rospy.is_shutdown():
        # pub_cmd.publish(msg)
        msg.linear.x = linx
        #msg.linear.y = liny
        msg.angular.z = angz
        print("linx", linx)
        print("ang z", angz)
        print("target_reached", target_reached)
        print()
        pub_cmd.publish(msg)
        rospy.on_shutdown(shutdown_stop)
        loop.sleep()
    
        


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
