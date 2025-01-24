#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point, PointStamped, PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Header
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d

x = [2.1, 4.2, 4.8, 6.1, 8.0, 10.2, 12.5, 11.0, 10.7]
y = [2.5, 4.0, 6.1, 7.5, 6.5, 6.1, 5.0, 4.7, 5.5]

path_points = []


def pointCallback(msg):
    global goal_x
    global goal_y
    global nodes_count
    goal_x = msg.point.x
    goal_y = msg.point.y
    point = Point(goal_x, goal_y, 0)
    goal_pub.publish(point)

def callback(msg):
    x = msg.x
    y = msg.y
    print(f'point: ({x:.2}, {y:.2}, 0)')


def splineCurve():
    rng = np.random.default_rng()

    N = len(x)
    X = np.expand_dims(x, axis=0)
    print(type(X))
    print(X.shape)
    Y = np.expand_dims(y, axis=0)
    print(type(Y))
    print(Y.shape)
    xy = np.vstack((X, Y))
    t = np.linspace(0, 1, N)
    spline = interp1d(t, xy, kind="quadratic", bounds_error=False)

    t_fine = np.linspace(0, 1, N**2)
    x_fine, y_fine = spline(t_fine)

    fig, ax = plt.subplots()
    ax.plot(x_fine, y_fine)
    ax.plot(x, y, ".")
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    plt.show()


def smoothPath():
    pass

def get_path(path):
    for p in path_points:
        print(p)
        pose_stamped = PoseStamped()
        pose_stamped.pose.position.x = p.x
        pose_stamped.pose.position.y = p.y
        pose_stamped.pose.position.z = 0.0
        path.poses.append(pose_stamped)

    return path
        

if __name__ == '__main__':
    global goal_pub
    
    rospy.init_node('show_path')
    
    rospy.Subscriber('/clicked_point', PointStamped, pointCallback)
    goal_pub = rospy.Publisher("/goal", Point, queue_size=10)
    rospy.Subscriber('/goal', Point, callback)

    path_pub = rospy.Publisher('/goal_path', Path, queue_size=10)
    path = Path()
    path.header = Header(frame_id='odom')

    for i in range(len(x)):
        point = Point(x[i], y[i], 0)
        path_points.append(point)
        # print("add point:")
        # print(point)

    path = get_path(path)

    rate = rospy.Rate(5)  # hz
    while not rospy.is_shutdown():
        splineCurve()
        path_pub.publish(path)
        rate.sleep()

    #rospy.spin()