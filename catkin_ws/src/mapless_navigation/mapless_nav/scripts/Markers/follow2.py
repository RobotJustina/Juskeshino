#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import Point, PointStamped, PoseStamped
from geometry_msgs.msg import Twist
from juskeshino_tools.JuskeshinoNavigation import JuskeshinoNavigation
import math
import numpy as np
th_d = 0.1


class PID:
    """
    Discrete PID control
    """

    def __init__(self, P=0.0, I=0.0, D=0.0, Derivator=0, Integrator=0, Integrator_max=10, Integrator_min=-10):
        self.Kp = P
        self.Ki = I
        self.Kd = D
        self.Derivator = Derivator
        self.Integrator = Integrator
        self.Integrator_max = Integrator_max
        self.Integrator_min = Integrator_min
        self.set_point = 0.0
        self.error = 0.0

    def update(self, current_value):
        self.error = self.set_point - current_value
        if self.error > math.pi:  # specific design for circular situation
            self.error = self.error - 2*math.pi
        elif self.error < -math.pi:
            self.error = self.error + 2*math.pi

        self.P_value = self.Kp * self.error
        self.D_value = self.Kd * (self.error - self.Derivator)
        self.Derivator = self.error
        self.Integrator = self.Integrator + self.error

        if self.Integrator > self.Integrator_max:
            self.Integrator = self.Integrator_max
        elif self.Integrator < self.Integrator_min:
            self.Integrator = self.Integrator_min

        self.I_value = self.Integrator * self.Ki
        PID = self.P_value + self.I_value + self.D_value
        return PID

    def setPoint(self, set_point):
        self.set_point = set_point
        self.Derivator = 0
        self.Integrator = 0

    def setPID(self, set_P=0.0, set_I=0.0, set_D=0.0):
        self.Kp = set_P
        self.Ki = set_I
        self.Kd = set_D


def followPathCallback(msg):
    global pub
    print("navigating")
    m = Path()
    first = True
    speed = Twist()
    count = 0
    for p in msg.poses:
        if first:
            first = False
        else:

            angle_to_goal, theta, dx, dy = update_delta(p)
            distance = math.sqrt(math.pow(dx, 2) + math.pow(dy, 2))
            while distance >= 0.35:
                delta_angle = angle_to_goal - theta
                if abs(delta_angle) > 0.2:
                    print("d_a:", delta_angle)
                    print("distance", distance)
                    scale = 0.1
                    dir = (delta_angle) / abs(delta_angle)
                    angSpeed = min(0.5, math.pow(delta_angle, 2) / scale)
                    speed.angular.z = dir*angSpeed
                    # print("a_sp", angSpeed)
                    speed.linear.x = math.pow(.1 / delta_angle, 2)
                    # print("linear:", speed.linear.x)
                    print()
                elif abs(delta_angle) > 1.6:
                    speed.linear.x = 0.01
                else:
                    speed.angular.z = 0.0
                    speed.linear.x = 0.3
                pub.publish(speed)

                angle_to_goal, theta, dx, dy = update_delta(p)
                distance = math.sqrt(math.pow(dx, 2) + math.pow(dy, 2))

            print("-------->")
            print()

        count += 1

    print(">>>")
    print()
    speed.linear.x = 0.0
    speed.angular.z = 0.0
    pub.publish(speed)


# def pathFollowerCalback(msg):
#     m = Path()
#     first = True
#     count = 0
#     # for point in msg.poses:
#     #     count += 1
#     #     if count > 2:
#     #         print("node", count)
#     #         #print(point.pose)
#     #         print()
#     #         angle_to_goal, theta, dx, dy = update_delta(p)

#     #         JuskeshinoNavigation.startMoveDistAngle(0, angle_to_goal)
#     #         #JuskeshinoNavigation.startMoveDist(1)
#     for p in msg.poses:
#         count += 1
#         if first:
#             first = False
#         else:

#             angle_to_goal, theta, dx, dy = update_delta(p)
#             delta_angle = angle_to_goal - theta
#             distance = math.sqrt(math.pow(dx, 2) + math.pow(dy, 2))
#             print("a_goal", angle_to_goal)
#             print("d_a", delta_angle)
#             while abs(delta_angle) > 0.3 and abs(delta_angle) < 6:
#                 print("a_goal", angle_to_goal)
#                 print("d_a", delta_angle)
#                 dir = (delta_angle) / abs(delta_angle)
#                 delta_angle = delta_angle - 2*math.pi if delta_angle > math.pi else delta_angle
#                 JuskeshinoNavigation.startMoveDistAngle(0, delta_angle)
#                 angle_to_goal, theta, dx, dy = update_delta(p)
#                 delta_angle = angle_to_goal - theta
#             print("dist", distance)
#             while distance > 0.3:
#                 print("dist", distance)
#                 print("d_a", delta_angle)
#                 speed_factor = 0.8
#                 if abs(delta_angle) > 0.3:
#                     JuskeshinoNavigation.startMoveDistAngle(0.1, delta_angle)
#                 else:
#                     JuskeshinoNavigation.startMoveDist(distance*speed_factor)
#                 angle_to_goal, theta, dx, dy = update_delta(p)
#                 distance = math.sqrt(math.pow(dx, 2) + math.pow(dy, 2))
#                 delta_angle = angle_to_goal - theta
#                 delta_angle = delta_angle - 2*math.pi if delta_angle > math.pi else delta_angle

#             print()
#     print("-------")

def pathFollowerCalback(msg):
    m = Path()
    first = True
    count = 0
    # for point in msg.poses:
    #     count += 1
    #     if count > 2:
    #         print("node", count)
    #         #print(point.pose)
    #         print()
    #         angle_to_goal, theta, dx, dy = update_delta(p)

    #         JuskeshinoNavigation.startMoveDistAngle(0, angle_to_goal)
    #         #JuskeshinoNavigation.startMoveDist(1)
    for p in msg.poses:
        count += 1
        if first:
            first = False
        else:

            angle_to_goal, theta, dx, dy = update_delta(p)
            delta_angle = angle_to_goal - theta
            distance = math.sqrt(math.pow(dx, 2) + math.pow(dy, 2))
            print("a_goal", angle_to_goal)
            print("d_a", delta_angle)
            while abs(delta_angle) > 0.3 and distance < 0.3:
                print("a_goal", angle_to_goal)
                print("d_a", delta_angle)
                dir = (delta_angle) / abs(delta_angle)
                delta_angle = delta_angle - 2*math.pi if delta_angle > math.pi else delta_angle
                JuskeshinoNavigation.startMoveDistAngle(0, delta_angle)
                angle_to_goal, theta, dx, dy = update_delta(p)
                delta_angle = angle_to_goal - theta
            print("dist", distance)
            while distance > 0.3:
                print("dist", distance)
                print("d_a", delta_angle)
                speed_factor = 1.2
                if abs(delta_angle) > 0.3:
                    JuskeshinoNavigation.startMoveDistAngle(0.1, delta_angle)
                else:
                    JuskeshinoNavigation.startMoveDist(distance*speed_factor)
                angle_to_goal, theta, dx, dy = update_delta(p)
                distance = math.sqrt(math.pow(dx, 2) + math.pow(dy, 2))
                delta_angle = angle_to_goal - theta
                delta_angle = delta_angle - 2*math.pi if delta_angle > math.pi else delta_angle

            print()
    print("-------")


def update_delta(p):
    x, y, theta = JuskeshinoNavigation.getRobotPoseWrtOdom()
    # print(f"robot: ({x:.2f}, {y:.2f}, {theta:.2f})")
    pos = p.pose.position
    # print(f"point: ({pos.x:.2f}, {pos.y:.2f}, {p.pose.orientation.z:.2f})")
    dx = pos.x - x
    dy = pos.y - y
    angle_to_goal = math.atan2(dy, dx)
    angle_to_goal = angle_to_goal - 2 * \
        math.pi if angle_to_goal > math.pi else angle_to_goal
    return angle_to_goal, theta, dx, dy


def controlCallback(msg):
    global pid_controller, pub, vel
    
    first = True
    count = 0
    for p in msg.poses:
        count += 1
        if first:
            first = False
        else:
            x, y, rth = JuskeshinoNavigation.getRobotPoseWrtOdom()
            dx = p.pose.position.x - x
            dy = p.pose.position.y - y
            direction_vect = np.array([dx, dy])
            # Normalization
            direction_vect = direction_vect / math.sqrt(dx*dx + dy*dy)
            theta = math.atan2(dy, dx)
            pid_controller.setPID(1, 0, 0)
            pid_controller.setPoint(theta)
            rospy.logwarn("### PID: set target theta = " + str(theta) + " ###")





            # Adjust orientation first
            while not rospy.is_shutdown():
                _, _, rth = JuskeshinoNavigation.getRobotPoseWrtOdom()
                angular = pid_controller.update(rth)
                print("ang",angular)
                if abs(angular) > 0.2:
                    angular = angular/abs(angular)*0.2
                if abs(angular) < 0.1: #0.01
                    break
                vel.linear.x = 0
                vel.angular.z = angular
                pub.publish(vel)
            








            # angle_to_goal, theta, dx, dy = update_delta(p)
            # delta_angle = angle_to_goal - theta
            # distance = math.sqrt(math.pow(dx, 2) + math.pow(dy, 2))
            # print("a_goal", angle_to_goal)
            # print("d_a", delta_angle)
            # while not rospy.is_shutdown():
            #     if abs(delta_angle) > 0.2:
            #         print("a_goal", angle_to_goal)
            #         print("d_a", delta_angle)
            #         dir = (delta_angle) / abs(delta_angle)
            #         delta_angle = delta_angle - 2*math.pi if delta_angle > math.pi else delta_angle
            #         print("d_a2", delta_angle)
            #         JuskeshinoNavigation.startMoveDistAngle(0, delta_angle)
            #         angle_to_goal, theta, dx, dy = update_delta(p)
            #         delta_angle = angle_to_goal - theta
            #     else:
            #         break
            # angular = pid_controller.update(rth)


            vel.linear.x = 0
            vel.angular.z = 0
            pub.publish(vel)

            pid_controller.setPoint(theta)
            pid_controller.setPID(1, 0.02, 0.2)
            speed_factor = 0.5
            while not rospy.is_shutdown():
                x, y, rth = JuskeshinoNavigation.getRobotPoseWrtOdom()
                dx = p.pose.position.x - x
                dy = p.pose.position.y - y
                vector = np.array([dx, dy])
                linear = np.dot(vector, direction_vect) # projection
                if abs(linear) > 0.2:
                    linear = linear/abs(linear)*speed_factor#*0.2
                
                angular = pid_controller.update(rth)
                print("ang2",angular)
                print("lin", linear)
                if abs(angular) > 0.2:
                    angular = angular/abs(angular)*0.2

                if abs(linear) < 0.1 and abs(angular) < 0.2:  # 0.01  --    0.1
                    break
                vel.linear.x = linear
                vel.angular.z = angular
                pub.publish(vel)

            vel.linear.x = 0
            vel.angular.z = 0
            pub.publish(vel)


def main():
    global pub, pid_controller, vel

    rospy.init_node('follow_path')
    rospy.logwarn("follow path")
    JuskeshinoNavigation.setNodeHandle()

    rospy.Subscriber('/mapless/goal_path', Path, pathFollowerCalback) #pathFollowerCalback
    pub = rospy.Publisher("/hardware/mobile_base/cmd_vel", Twist, queue_size=1)

    pid_controller = PID(0, 0, 0)
    vel = Twist()
    rospy.spin()


if __name__ == '__main__':
    main()
