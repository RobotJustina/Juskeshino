#pragma once
#include <iostream>
#include <cmath>
#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/GetMap.h"
#include "nav_msgs/Path.h"
#include "actionlib_msgs/GoalStatus.h"
#include "hardware_msgs/GetRgbd.h"
#include "tf/transform_listener.h"

class JuskeshinoNavigation
{
private:
    static bool is_node_set;
    //Subscribers for stop signals
    static ros::Subscriber subStop;
    static ros::Subscriber subNavigationStop;
    //Subscribers for checking goal-pose-reached signal
    static ros::Subscriber subNavigationStatus;
    static ros::Subscriber subSimpleMoveStatus;
    //Publishers and subscribers for operating the simple_move node
    static ros::Publisher pubSimpleMoveDist;
    static ros::Publisher pubSimpleMoveDistAngle;
    static ros::Publisher pubSimpleMoveLateral;
    //Publishers and subscribers for mvn_pln
    static ros::Publisher pubMvnPlnGetCloseXYA;
    static ros::Publisher pubNavigationStop;
    //Publishers and subscribers for localization
    static tf::TransformListener* tf_listener;

    //Variables for justina navigation:
    static actionlib_msgs::GoalStatus  _navigation_status;
    static actionlib_msgs::GoalStatus  _simple_move_status;
    static bool _stop;

public:
    //
    //The startSomething functions, only publish the goal pose or path and return inmediately after starting movement
    //The others, block until a goal-reached signal is received
    //
    
    static bool setNodeHandle(ros::NodeHandle* nh);
    //Methods for checking if goal position is reached.
    static bool isLocalGoalReached();
    static bool isGlobalGoalReached();
    static bool waitForLocalGoalReached(int timeOut_ms);
    static bool waitForGlobalGoalReached(int timeOut_ms);
    //Methods for robot localization
    static void getRobotPoseWrtMap(float& currentX, float& currentY, float& currentTheta);
    static void getRobotPoseWrtOdom(float& currentX, float& currentY, float& currentTheta);
    //These methods use the simple_move node
    static void startMoveDist(float distance);
    static void startMoveDistAngle(float distance, float angle);
    static void startMoveLateral(float distance);
    static bool moveDist(float distance, int timeOut_ms);
    static bool moveDistAngle(float distance, float angle, int timeOut_ms);
    static bool moveLateral(float distance, int timeOut_ms);

    //These methods use the mvn_pln node.
    static void startGetClose(float x, float y, float angle);
    static void startGetClose(std::string location);
    static bool getClose(float x, float y, float angle, int timeOut_ms);
    static bool getClose(std::string location, int timeOut_ms);
    static void stopNavigation();

    //Callbacks for subscribers
    static void callbackStop(const std_msgs::Empty::ConstPtr& msg);
    static void callbackNavigationStop(const std_msgs::Empty::ConstPtr& msg);
    static void callbackSimpleMoveStatus(const actionlib_msgs::GoalStatus::ConstPtr& msg);
    static void callbackNavigationStatus(const actionlib_msgs::GoalStatus::ConstPtr& msg);
};
