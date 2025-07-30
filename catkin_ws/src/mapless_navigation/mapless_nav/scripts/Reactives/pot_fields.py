#!/usr/bin/env python3

import rospy
import tf
import math
import numpy
from geometry_msgs.msg import Twist, PoseStamped, Point, Vector3, PointStamped
from std_msgs.msg import Float64MultiArray, Bool, Float32MultiArray, Int8
from visualization_msgs.msg import Marker
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
import numpy as np
import sys
import time

listener    = None
pub_cmd_vel = None
pub_markers = None
laser_readings = None
target_reached = True
last_goal = [0, 0] # d, th
v_max = 0.4
w_max = 0.8
nav_fails = 0
stop_nav = False
t0 = None
slow = 0
scape_force = 1

def calculate_control(goal_x, goal_y, alpha, beta):
    v,w = 0,0
    error_a = math.atan2(goal_y, goal_x)
    v = v_max * math.exp(-(error_a * error_a)/alpha)
    w = w_max * (2/(1 + math.exp(-error_a/beta)) +1)
    if error_a < 0:
        w = -w

    return [v,w]


def attraction_force(goal_x, goal_y, eta):
    force_x, force_y = 0,0 
    q_goal = get_goal_point_wrt_robot(goal_x, goal_y)
    q_unit = q_goal / numpy.linalg.norm(q_goal)
    force_x, force_y = -eta * q_unit
    
    return numpy.asarray([force_x, force_y])


def rejection_force(laser_readings, zeta, d0):
    N = len(laser_readings)
    if N == 0:
        return [0, 0]
    force_x, force_y = 0, 0
    for d, th in laser_readings: # R to L
        if d < d0:
            f_rej = zeta * math.sqrt(1/d - 1/d0)
            force_x += f_rej*math.cos(th)
            force_y += f_rej*math.sin(th)
        else:
            f_rej = 0

    force_x = force_x / N
    force_y = force_y / N

    return numpy.asarray([force_x, force_y])


def move_by_pot_fields(global_goal_x, global_goal_y, epsilon, tol, eta, zeta, d0, alpha, beta, gamma):
    global rej_cloud, scape_force
    global target_reached, stop_nav, nav_fails
    global goal_stat_pub, t0

    [g_x, g_y] = get_goal_point_wrt_robot(global_goal_x, global_goal_y)
    distance = math.sqrt(g_x*g_x + g_y*g_y)

    rospy.sleep(0.1)
    loop = rospy.Rate(20)
    t0 = time.time()
    while distance > tol and not rospy.is_shutdown():
        print("scape_force", scape_force)
        a_force = attraction_force(global_goal_x, global_goal_y, eta)
        r_force = rejection_force(laser_readings, zeta, d0)
        r_force += numpy.asarray([rej_cloud[0], 4.0 *rej_cloud[1]])
        force = a_force + r_force *scape_force
        next_p = -epsilon*force
        print(next_p)
        v, w = calculate_control(next_p[0], next_p[1], alpha, beta)
        publish_speed_and_forces(v, w, a_force, r_force, force, gamma)
        [g_x, g_y] = get_goal_point_wrt_robot(global_goal_x, global_goal_y)
        distance = math.sqrt(g_x*g_x + g_y*g_y)
        loop.sleep()
        if stop_nav:
            print(" STOPPED!")
            rospy.sleep(0.1)
            print(" STOPPED!")
            return
    
    print("Target reached")
    status = Int8()
    status = 3
    goal_stat_pub.publish(status)
    nav_fails = 0
    target_reached = True
    return
        

def get_goal_point_wrt_robot(goal_x, goal_y):
    robot_x, robot_y, robot_a = get_robot_pose(listener)
    delta_x = goal_x - robot_x
    delta_y = goal_y - robot_y
    goal_x =  delta_x*math.cos(robot_a) + delta_y*math.sin(robot_a)
    goal_y = -delta_x*math.sin(robot_a) + delta_y*math.cos(robot_a)

    return [goal_x, goal_y]


def get_robot_pose(listener):
    try:
        ([x, y, z], [qx,qy,qz,qw]) = listener.lookupTransform('odom', 'base_link', rospy.Time(0))
        return [x, y, 2*math.atan2(qz, qw)]
    except:
        return [0,0,0]


def publish_speed_and_forces(v, w, Fa, Fr, F, gamma):
    loop = rospy.Rate(20)
    pub_cmd_vel.publish(Twist(linear=Vector3(x=v, y= -gamma *rej_cloud[1]), angular=Vector3(z=w)))
    pub_markers.publish(get_force_marker(Fa[0], Fa[1], [0.0, 0.0, 1.0, 1.0], 0))
    pub_markers.publish(get_force_marker(Fr[0], Fr[1], [1.0, 0.0, 0.0, 1.0], 1))
    pub_markers.publish(get_force_marker(F [0], F [1], [0.0, 0.6, 0.0, 1.0], 2))
    loop.sleep()


def get_force_marker(force_x, force_y, color, id):
    mrk = Marker()
    mrk.header.frame_id = "base_link"
    mrk.header.stamp = rospy.Time.now()
    mrk.ns = "pot_fields"
    mrk.id = id
    mrk.type = Marker.ARROW
    mrk.action = Marker.ADD
    mrk.pose.orientation.w = 1
    mrk.color.r, mrk.color.g, mrk.color.b, mrk.color.a = color
    mrk.scale.x, mrk.scale.y, mrk.scale.z = [0.07, 0.1, 0.15]
    mrk.points.append(Point(x=0, y=0))
    mrk.points.append(Point(x=-force_x, y=-force_y))
    return mrk


def callback_scan(msg):
    global laser_readings
    laser_readings = [[msg.ranges[i], msg.angle_min+i*msg.angle_increment] for i in range(len(msg.ranges))]


def moveGoalCallback(msg):
    global target_reached

    target_reached = False
    pos = msg.pose.position
    clp_pub = rospy.Publisher("/clicked_point", PointStamped, queue_size=10)
    point = PointStamped()
    point.header.frame_id = "odom"
    point.header.stamp = rospy.Time.now()
    point.point = pos
    clp_pub.publish(point)
    print(f"\nNew goal: ({pos.x:.3f}, {pos.y:.3f})", end="\r")
    print(end='\n')


def shutdown_stop():
    print("Exit ...")
    pub_cmd_vel.publish(Twist())
    pub_cmd_vel.publish(Twist())
    rospy.sleep(0.5)


def rejCloudCallback(msg):
    global rej_cloud
    if msg.x == np.nan or msg.y == np.nan:
        rej_cloud = numpy.asarray([0, 0])    
    rej_cloud = numpy.asarray([-msg.x, -msg.y])


def movingCallback(msg):
    global t0, slow, scape_force
    
    if enable_obst_detect:
        v_x = msg.linear.x
        if v_x < 0.001:
            slow += 1
        else:
            slow = 0

        if slow == 1:
            t0 = time.time()
        if slow > 1:
            timer = time.time() - t0
            print(f"msg.linear.x {v_x:.5f}, timer: {timer:.2f}")
            if timer >= 6.0 and timer <6.5:
                scape_force = 100
                t0 = time.time()
                rospy.sleep(1.5)
                return
        scape_force = 1

def callback_point(msg):
    global target_reached
    global stop_nav

    blanks = " "*110
    sys.stdout.write("\033[A" + blanks + "\r")
    sys.stdout.flush()
    str = f"\nNew goal: ({msg.point.x:.3f}, {msg.point.y:.3f})" + " "*100
    sys.stdout.write(str)
    sys.stdout.flush()
    sys.stdout.write("\n")
    sys.stdout.flush()
    target_reached = False

    enable_obst_detect = Bool()
    enable_obst_detect.data = True
    pubObstDetEnable.publish(enable_obst_detect)
    [goal_x, goal_y] = [msg.point.x, msg.point.y]
    epsilon = rospy.get_param('~epsilon', 0.5)   # 0.5  # Next point scaler
    tol     = rospy.get_param('~tol', 0.5)      # 0.5
    eta     = rospy.get_param('~eta', 1.5)       # 2.0  Attraction scaler  
    zeta    = rospy.get_param('~zeta', 8.5)      # 6.0  Lid scaler
    d0      = rospy.get_param('~d0', 0.5)        # 1.0   Lid param
    alpha   = rospy.get_param('~alpha', 0.8)    #.48 0.5  control v
    beta    = rospy.get_param('~beta', 0.2)    #.9 0.5   control w
    gamma    = rospy.get_param('~gamma', 0.4)     # 4.0   Cloud scaler
    move_by_pot_fields(goal_x, goal_y, epsilon, tol, eta, zeta, d0, alpha, beta, gamma)
    pub_cmd_vel.publish(Twist())
    stop_nav = False
    enable_obst_detect.data = False
    pubObstDetEnable.publish(enable_obst_detect)


def callback_goal(msg):
    global last_goal, target_reached
    global interrupt
    if target_reached:
        last_goal[0] = 0.0
        last_goal[1] = 0.0
    else:
        last_goal = list(msg.data)
        if last_goal[0] < 0.5:
            interrupt = True
        else:
            interrupt = False


def statusCallback(msg):
    global stop_nav, nav_fails

    if msg.data == 4:
        nav_fails += 1
        print("nav_fails", nav_fails)
        if nav_fails > 2:
            stop_nav = True
            nav_fails = 0

def main():
    global listener, pub_cmd_vel, pub_markers
    global pubObstDetEnable, rej_cloud, enable_obst_detect
    global goal_stat_pub

    print("pot_fields")
    rospy.init_node("pot_fields")
    rospy.Subscriber("/hardware/scan", LaserScan, callback_scan)
    rospy.Subscriber('/move_base_simple/goal', PoseStamped, moveGoalCallback)
    rospy.Subscriber("/navigation/obs_detector/pf_rejection_force", Vector3, rejCloudCallback)
    rospy.Subscriber('/hardware/mobile_base/cmd_vel', Twist, movingCallback)
    rospy.Subscriber("/clicked_point", PointStamped, callback_point)
    rospy.Subscriber("/NN_goal", Float32MultiArray, callback_goal)
    rospy.Subscriber('/maples_nav/goal_reached', Int8, statusCallback)

    pub_cmd_vel = rospy.Publisher('/hardware/mobile_base/cmd_vel', Twist,  queue_size=10)#/cmd_vel
    pub_markers = rospy.Publisher('/navigation/pot_field_markers', Marker, queue_size=10)
    listener = tf.TransformListener()
    pubObstDetEnable = rospy.Publisher("/navigation/obs_detector/enable", Bool, queue_size=1)
    pubHeadPos = rospy.Publisher("/hardware/head/goal_pose", Float64MultiArray, queue_size=1)
    goal_stat_pub = rospy.Publisher('/maples_nav/goal_reached', Int8, queue_size=1)
    start_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)

    msgHeadPos = Float64MultiArray()
    msgHeadPos.data = [0.0, -0.4]
    pubHeadPos.publish(msgHeadPos)
    rospy.sleep(1)
    pubHeadPos.publish(msgHeadPos)
    enable_obst_detect = Bool()
    enable_obst_detect.data = False
    pubObstDetEnable.publish(enable_obst_detect)
    rej_cloud = numpy.asarray([0, 0])

    get_robot_pose(listener)
    ([x, y, z], [qx,qy,qz,qw]) = listener.lookupTransform('odom', 'base_link', rospy.Time(0))

    msg = PoseStamped()
    msg.header.frame_id = "odom"
    msg.pose.position.x = x
    msg.pose.position.y = y
    msg.pose.position.z = 0
    msg.pose.orientation.x = qx
    msg.pose.orientation.y = qy
    msg.pose.orientation.z = qz
    msg.pose.orientation.w = qw
    start_pub.publish(msg)
    #JuskeshinoNavigation.pubMvnPlnGetCloseXYA.publish(msg)

    rospy.on_shutdown(shutdown_stop)
    rospy.spin()


if __name__ == '__main__':
    try:
        # feat = Features()
        main()
    except rospy.ROSInterruptException:
        pass