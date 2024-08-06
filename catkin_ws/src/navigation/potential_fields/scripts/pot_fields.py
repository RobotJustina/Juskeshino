#!/usr/bin/env python3
#
# MOBILE ROBOTS - FI-UNAM, 2024-1
# ASSIGNMENT 06 - OBSTACLE AVOIDANCE BY POTENTIAL FIELDS
#
# Instructions:
# Complete the code to implement obstacle avoidance by potential fields
# using the attractive and repulsive fields technique.
# Tune the constants d0, eta and zeta to get a smooth movement. 
#

import rospy
import tf
import math
import time
from std_msgs.msg import Header
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PointStamped

NAME = "FULL NAME"

listener    = None
pub_cmd_vel = None
pub_markers = None

def calculate_control(robot_x, robot_y, robot_a, goal_x, goal_y):
    cmd_vel = Twist()
    #
    # TODO:
    # Implement the control law given by:
    #
    # v = v_max*math.exp(-error_a*error_a/alpha)
    # w = w_max*(2/(1 + math.exp(-error_a/beta)) - 1)
    #
    # where error_a is the angle error and
    # v and w are the linear and angular speeds.
    # v_max, w_max, alpha and beta, are design constants.
    # Maximum linear and angular speeds must be 0.5 and 1.0 respectively.
    # Store the resulting v and w in the Twist message 'cmd_vel'
    # and return it (check online documentation for the Twist message).
    # Remember to keep error angle in the interval (-pi,pi]
    #
    
    v_max = 0.5
    w_max = 1.0
    alpha = 0.2
    beta  = 0.5
    [error_x, error_y] = [goal_x - robot_x, goal_y - robot_y]
    error_a = (math.atan2(error_y, error_x) - robot_a)%(2*math.pi)
    error_d = math.sqrt(error_x**2 + error_y**2)
    if error_a  >  math.pi:
        error_a -= 2*math.pi
    cmd_vel.linear.x  = min(v_max, error_d)*math.exp(-error_a*error_a/alpha)
    cmd_vel.angular.z = w_max*(2/(1 + math.exp(-error_a/beta)) - 1)
    return cmd_vel

def attraction_force(robot_x, robot_y, goal_x, goal_y):
    #
    # TODO:
    # Calculate the attraction force, given the robot and goal positions.
    # Return a tuple of the form [force_x, force_y]
    # where force_x and force_y are the X and Y components
    # of the resulting attraction force w.r.t. map.
    #
    eta = 2.0  #Attraction constant
    force_x, force_y  = robot_x - goal_x, robot_y - goal_y
    mag = math.sqrt(force_x**2 + force_y**2)
    return [eta*force_x/mag, eta*force_y/mag] if mag != 0 else [0,0]

def rejection_force(robot_a, laser_readings):
    #
    # TODO:
    # Calculate the total rejection force given by the average
    # of the rejection forces caused by each laser reading.
    # laser_readings is an array where each element is a tuple [distance, angle]
    # both measured w.r.t. ROBOT's FRAME. 
    # See lecture notes for equations to calculate rejection forces.
    # Return a tuple of the form [force_x, force_y]
    # where force_x and force_y are the X and Y components
    # of the resulting rejection force w.r.t. MAP.
    # WARNING: Some laser readings could have distance=0 due to simulated reading errors. 
    #
    zeta = 6.0 #Rejection constant
    d0   = 1.0 #Distance of influence
    force_x, force_y = 0, 0
    for [distance, angle] in laser_readings:
        mag = zeta*math.sqrt(1/distance - 1/d0) if distance < d0 and distance > 0 else 0
        force_x += mag*math.cos(angle + robot_a)
        force_y += mag*math.sin(angle + robot_a)
    return [force_x/len(laser_readings), force_y/len(laser_readings)] if len(laser_readings) != 0 else [0,0]

def callback_pot_fields_goal(msg):
    goal_x = msg.point.x
    goal_y = msg.point.y
    print("Moving to goal point " + str([goal_x, goal_y]) + " by potential fields"    )
    loop = rospy.Rate(20)
    global laser_readings

    #
    # TODO:
    # Move the robot towards goal point using potential fields.
    # Remember goal point is a local minimun in the potential field, thus,
    # it can be reached by the gradient descend algorithm.
    # Sum of attraction and rejection forces is the gradient of the potential field,
    # then, you can reach the goal point with the following pseudocode:
    #
    # Set constant epsilon (0.5 is a good start)
    # Set tolerance  (0.1 is a good start)
    # Get robot position by calling robot_x, robot_y, robot_a = get_robot_pose(listener)
    # Calculate distance to goal as math.sqrt((goal_x - robot_x)**2 + (goal_y - robot_y)**2)
    # WHILE distance_to_goal_point > tolerance and not rospy.is_shutdown():
    #     Calculate attraction force Fa by calling [fax, fay] = attraction_force(robot_x, robot_y, goal_x, goal_y)
    #     Calculate rejection  force Fr by calling [frx, fry] = rejection_force (robot_x, robot_y, robot_a, laser_readings)
    #     Calculate resulting  force F = Fa + Fr
    #     Calculate next local goal point BY GRADIENT DESCEND P = [px, py] = Pr - epsilon*F
    #
    #     Calculate control signals by calling msg_cmd_vel = calculate_control(robot_x, robot_y, robot_a, px, py)
    #     Send the control signals to mobile base by calling pub_cmd_vel.publish(msg_cmd_vel)
    #     Call draw_force_markers(robot_x, robot_y, afx, afy, rfx, rfy, fx, fy, pub_markers)  to draw all forces
    #
    #     Wait a little bit of time by calling loop.sleep()
    #     Update robot position by calling robot_x, robot_y, robot_a = get_robot_pose(listener)
    #     Recalculate distance to goal position
    #  Publish a zero speed (to stop robot after reaching goal point)

    time_init=rospy.get_time()

    robot_x, robot_y, robot_a = get_robot_pose(listener)
    dist_to_goal = math.sqrt((goal_x - robot_x)**2 + (goal_y - robot_y)**2)
    tolerance = 0.1
    epsilon = 0.5
    while dist_to_goal > tolerance and not rospy.is_shutdown():
        afx, afy = attraction_force(robot_x, robot_y, goal_x, goal_y)
        rfx, rfy = rejection_force (robot_a, laser_readings)
        [fx, fy] = [afx + rfx, afy + rfy]
        [px, py] = [robot_x - epsilon*fx, robot_y - epsilon*fy]
        
        msg_cmd_vel = calculate_control(robot_x, robot_y, robot_a, px, py)
        pub_cmd_vel.publish(msg_cmd_vel)
        draw_force_markers(robot_x, robot_y, afx, afy, rfx, rfy, fx, fy, pub_markers)

        loop.sleep()
        robot_x, robot_y, robot_a = get_robot_pose(listener)
        dist_to_goal = math.sqrt((goal_x - robot_x)**2 + (goal_y - robot_y)**2)
    pub_cmd_vel.publish(Twist())

    time=rospy.get_time()-time_init
    print(f"Goal point reached in {time} seconds")

def get_robot_pose(listener):
    try:
        ([x, y, z], rot) = listener.lookupTransform('odom', 'base_link', rospy.Time(0))
        a = 2*math.atan2(rot[2], rot[3])
        a = a - 2*math.pi if a > math.pi else a
        return [x, y, a]
    except:
        pass
    return [0,0,0]

def callback_scan(msg):
    global laser_readings
    laser_readings = [[msg.ranges[i], msg.angle_min+i*msg.angle_increment] for i in range(len(msg.ranges))]

def draw_force_markers(robot_x, robot_y, attr_x, attr_y, rej_x, rej_y, res_x, res_y, pub_markers):
    pub_markers.publish(get_force_marker(robot_x, robot_y, attr_x, attr_y, [0,0,1,1]  , 0))
    pub_markers.publish(get_force_marker(robot_x, robot_y, rej_x,  rej_y,  [1,0,0,1]  , 1))
    pub_markers.publish(get_force_marker(robot_x, robot_y, res_x,  res_y,  [0,0.6,0,1], 2))

def get_force_marker(robot_x, robot_y, force_x, force_y, color, id):
    hdr = Header(frame_id="odom", stamp=rospy.Time.now())
    mrk = Marker(header=hdr, ns="pot_fields", id=id, type=Marker.ARROW, action=Marker.ADD)
    mrk.pose.orientation.w = 1
    mrk.color.r, mrk.color.g, mrk.color.b, mrk.color.a = color
    mrk.scale.x, mrk.scale.y, mrk.scale.z = [0.07, 0.1, 0.15]
    mrk.points.append(Point(x=robot_x, y=robot_y))
    mrk.points.append(Point(x=(robot_x - force_x), y=(robot_y - force_y)))
    return mrk

def main():
    global listener, pub_cmd_vel, pub_markers
    print("Potential fields...")
    rospy.init_node("pot_fields")
    #rospy.Subscriber("/hardware/scan", LaserScan, callback_scan)
    rospy.Subscriber("/laser_mod", LaserScan, callback_scan)
    rospy.Subscriber("/clicked_point", PointStamped, callback_pot_fields_goal)
    pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist,  queue_size=10)
    pub_markers = rospy.Publisher('/navigation/pot_field_markers', Marker, queue_size=10)
    listener = tf.TransformListener()
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    
