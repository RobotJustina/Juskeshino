#! /usr/bin/env python3
import rospy
from std_msgs.msg import Float32MultiArray, Float64MultiArray, Int8
from geometry_msgs.msg import Twist, PointStamped, PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
import numpy as np
import rospkg
import torch


from TorchModels.utils import models

# >>>Select model
n_channels = 4
outputs = 3
occ_grid_meters = 4
model = models.Param_CNN_B(channels=n_channels, img_size=int(20*occ_grid_meters))
#model = models.RNN(channels=n_channels, img_size=int(20*occ_grid_meters)
#                   ,hidden_size=350, num_layers=4)

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
speed_factor = 2.0


def moveGoalCallback(msg):
    global target_reached
    pos = msg.pose.position

    clp_pub = rospy.Publisher("/clicked_point", PointStamped, queue_size=10)
    point = PointStamped()
    point.header.frame_id = "odom"
    point.header.stamp = rospy.Time.now()
    point.point = pos
    clp_pub.publish(point)
    print(f"\nNew goal: ({pos.x:.3f}, {pos.y:.3f})", end="\r") 
    print(end='\n')
    target_reached = False


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


def occGridCallback(msg):
    global data_X, linx, liny, angz
    global model, last_goal, disp, init_time
    global target_reached, speed_factor
    global goal_stat_pub
    
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
    data_X = np.vstack((data, other_features))
    entrada = np.asarray(data_X)

    # Config input to model
    batch = []
    for i in range(n_channels):
        batch.append(entrada)
    entrada = np.array(batch)
    entrada = np.expand_dims(entrada, axis=0)
    x_ent = torch.tensor(entrada)
    x_ent = x_ent.to(torch.device(disp), torch.float32)

    turn_help = True
    #>> 3 params >>
    if (abs(last_goal[0]) > 0.5):
        with torch.no_grad():
            y_pred = model(x_ent)
        
        if model.name == "RNN":
            y_pred = y_pred.cpu().numpy()
        else:
            y_pred = y_pred.cpu().numpy()[0]

        if turn_help:
            if abs(last_goal[1]) < 0.5:
                linx = y_pred[0] #* speed_factor
                angz = y_pred[2] *0.4
                liny = y_pred[1]
            else:
                linx = y_pred[0] / (5*abs(last_goal[1]))
                liny = y_pred[1] / (10*abs(last_goal[1]))
                print("control linx", linx, end='\n')
                print("control liny", liny, end='\n')
                angz = y_pred[2] * 2.0
            
        else:
            linx = y_pred[0]
            """Deleted"""
            liny = y_pred[1]
            """changed"""
            angz = y_pred[2]

    else:
        linx = 0.0
        liny = 0.0
        angz = 0.0
        if last_goal[0] != 0 and last_goal[1] != 0:
            target_reached = True
            status = Int8()
            status = 3
            goal_stat_pub.publish(status)

    
def shutdown_stop():
    global pub_cmd
    pub_cmd.publish(Twist())


def main():
    global linx, liny, angz
    global pub_cmd, last_goal
    global target_reached, goal_stat_pub
    rospy.init_node("NN_out")
    rospy.Subscriber("/NN_goal", Float32MultiArray, callback_goal)
    rospy.Subscriber("/clicked_point", PointStamped, callback_point)
    rospy.Subscriber("/local_occ_grid", OccupancyGrid, occGridCallback)
    rospy.Subscriber('/move_base_simple/goal', PoseStamped, moveGoalCallback)
    pub_cmd = rospy.Publisher(
        "/hardware/mobile_base/cmd_vel", Twist, queue_size=10)
    pubHeadPos = rospy.Publisher("/hardware/head/goal_pose", Float64MultiArray, queue_size=1)
    print("NN_out has been started")
    goal_stat_pub = rospy.Publisher('/maples_nav/goal_reached', Int8, queue_size=1)
    
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
        msg.linear.y = liny
        msg.angular.z = angz
        d = last_goal[0]
        th = last_goal[1]
        cad = f"trg({d:.4f}, {th:.4f}) || "
        if outputs == 3:
            cad += f"vel(l_x, l_y, a_z) = ({linx:.3f},{liny:.3f},{angz:.3f})"
        else:    
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
