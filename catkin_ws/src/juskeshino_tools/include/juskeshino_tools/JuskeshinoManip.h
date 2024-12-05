#pragma once
#include <iostream>
#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"
#include "std_msgs/Empty.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point32.h"
#include "geometry_msgs/Point.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/Image.h"


class JuskeshinoManip
{
private:
    static bool is_node_set;
    
public:
    static bool setNodeHandle(ros::NodeHandle* nh);

    static bool isHdGoalReached();
    static bool isLaGoalReached();
    static bool isRaGoalReached();
    static bool waitForHdGoalReached(int timeOut_ms);
    static bool waitForLaGoalReached(int timeOut_ms);
    static bool waitForRaGoalReached(int timeOut_ms);

    static bool startHdGoToArticular(std::vector<double> goal_q);
    static bool startLaGoToArticular(std::vector<double> goal_q);
    static bool startRaGoToArticular(std::vector<double> goal_q);
    static bool hdGoToArticular(std::vector<double> goal_q, , int timeOut_ms);
    static bool laGoToArticular(std::vector<double> goal_q, , int timeOut_ms);
    static bool raGoToArticular(std::vector<double> goal_q, , int timeOut_ms);

    static bool startHdGoToArticular(std::vector<double> goal_q);
    static bool startLaGoToArticular(std::vector<double> goal_q);
    static bool startRaGoToArticular(std::vector<double> goal_q);
    static bool hdGoToArticular(std::vector<double> goal_q, , int timeOut_ms);
    static bool laGoToArticular(std::vector<double> goal_q, , int timeOut_ms);
    static bool raGoToArticular(std::vector<double> goal_q, , int timeOut_ms);
};
