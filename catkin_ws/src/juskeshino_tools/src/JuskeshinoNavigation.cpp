#include "juskeshino_tools/JuskeshinoNavigation.h"

bool JuskeshinoNavigation::is_node_set;
actionlib_msgs::GoalStatus JuskeshinoNavigation::_navigation_status;
actionlib_msgs::GoalStatus JuskeshinoNavigation::_simple_move_status;
bool JuskeshinoNavigation::_stop;

//Subscribers for stop signals
ros::Subscriber JuskeshinoNavigation::subStop;
ros::Subscriber JuskeshinoNavigation::subNavigationStop;
//Subscribers for checking goal-pose-reached signal
ros::Subscriber JuskeshinoNavigation::subNavigationStatus;
ros::Subscriber JuskeshinoNavigation::subSimpleMoveStatus;
//Publishers and subscribers for operating the simple_move node
ros::Publisher JuskeshinoNavigation::pubSimpleMoveDist;
ros::Publisher JuskeshinoNavigation::pubSimpleMoveDistAngle;
ros::Publisher JuskeshinoNavigation::pubSimpleMoveLateral;
//Publishers and subscribers for mvn_pln
ros::Publisher JuskeshinoNavigation::pubMvnPlnGetCloseXYA;
ros::Publisher JuskeshinoNavigation::pubNavigationStop;
//Publishers and subscribers for localization
tf::TransformListener* JuskeshinoNavigation::tf_listener;

bool JuskeshinoNavigation::setNodeHandle(ros::NodeHandle* nh)
{
    if(JuskeshinoNavigation::is_node_set)
        return true;
    if(nh == 0)
        return false;

    std::cout << "JuskeshinoNavigation.->Setting ros node..." << std::endl;
    subStop                = nh->subscribe("/stop"                    , 10, &JuskeshinoNavigation::callbackStop);
    subNavigationStop      = nh->subscribe("/navigation/stop"         , 10, &JuskeshinoNavigation::callbackNavigationStop);
    subNavigationStatus    = nh->subscribe("/navigation/status"       , 10, &JuskeshinoNavigation::callbackNavigationStatus);
    subSimpleMoveStatus    = nh->subscribe("/simple_move/goal_reached", 10, &JuskeshinoNavigation::callbackSimpleMoveStatus);
    pubSimpleMoveDist      = nh->advertise<std_msgs::Float32          >("/simple_move/goal_dist", 10);
    pubSimpleMoveDistAngle = nh->advertise<std_msgs::Float32MultiArray>("/simple_move/goal_dist_angle", 10);
    pubSimpleMoveLateral   = nh->advertise<std_msgs::Float32          >("/simple_move/goal_dist_lateral", 10);
    pubMvnPlnGetCloseXYA   = nh->advertise<geometry_msgs::PoseStamped >("/move_base_simple/goal", 10);
    pubNavigationStop      = nh->advertise<std_msgs::Empty>            ("/navigation/stop", 10);
    tf_listener = new tf::TransformListener();

    is_node_set = true;
    _stop = false;
    _navigation_status.status  = actionlib_msgs::GoalStatus::PENDING;
    _simple_move_status.status = actionlib_msgs::GoalStatus::PENDING;

    ros::Rate loop(10);
    int counter = 3;
    while(ros::ok() && counter > 0)
    {
        counter--;
        ros::spinOnce();
        loop.sleep();
    }
    return true;
}

//Methods for checking if goal position is reached.
bool JuskeshinoNavigation::isLocalGoalReached()
{
    return _simple_move_status.status == actionlib_msgs::GoalStatus::SUCCEEDED;
}

bool JuskeshinoNavigation::isGlobalGoalReached()
{
    return _navigation_status.status == actionlib_msgs::GoalStatus::SUCCEEDED;
}

bool JuskeshinoNavigation::waitForLocalGoalReached(int timeOut_ms)
{
    JuskeshinoNavigation::_stop = false;
    JuskeshinoNavigation::_simple_move_status.status = actionlib_msgs::GoalStatus::PENDING;
    int attempts = timeOut_ms/100;
    ros::Rate loop(10);
    while(ros::ok() && JuskeshinoNavigation::_simple_move_status.status == actionlib_msgs::GoalStatus::PENDING &&
          !JuskeshinoNavigation::_stop && attempts-- >= 0)
    {
        loop.sleep();
        ros::spinOnce();
    }
    
    while(ros::ok() && JuskeshinoNavigation::_simple_move_status.status == actionlib_msgs::GoalStatus::ACTIVE &&
          !JuskeshinoNavigation::_stop && attempts-- >= 0)
    {
        loop.sleep();
        ros::spinOnce();
    }
    JuskeshinoNavigation::_stop = false; //This flag is set True in the subscriber callback
    return JuskeshinoNavigation::_simple_move_status.status == actionlib_msgs::GoalStatus::SUCCEEDED;
}

bool JuskeshinoNavigation::waitForGlobalGoalReached(int timeOut_ms)
{
    JuskeshinoNavigation::_stop = false;
    JuskeshinoNavigation::_navigation_status.status = actionlib_msgs::GoalStatus::PENDING;
    int attempts = timeOut_ms/100;
    ros::Rate loop(10);
    while(ros::ok() && JuskeshinoNavigation::_navigation_status.status == actionlib_msgs::GoalStatus::PENDING &&
          !JuskeshinoNavigation::_stop && attempts-- >= 0)
    {
        loop.sleep();
        ros::spinOnce();
    }
    while(ros::ok() && JuskeshinoNavigation::_navigation_status.status == actionlib_msgs::GoalStatus::ACTIVE &&
          !JuskeshinoNavigation::_stop && attempts-- >= 0)
    {
        loop.sleep();
        ros::spinOnce();
    }
    JuskeshinoNavigation::_stop = false;
    return JuskeshinoNavigation::_navigation_status.status == actionlib_msgs::GoalStatus::SUCCEEDED;
}

//Methods for robot localization
void JuskeshinoNavigation::getRobotPoseWrtMap(float& currentX, float& currentY, float& currentTheta)
{
    tf::StampedTransform transform;
    tf::Quaternion q;
    JuskeshinoNavigation::tf_listener->lookupTransform("map", "base_link", ros::Time(0), transform);
    currentX = transform.getOrigin().x();
    currentY = transform.getOrigin().y();
    q = transform.getRotation();
    currentTheta = atan2((float)q.z(), (float)q.w()) * 2;
}

void JuskeshinoNavigation::getRobotPoseWrtOdom(float& currentX, float& currentY, float& currentTheta)
{
    tf::StampedTransform transform;
    tf::Quaternion q;
    JuskeshinoNavigation::tf_listener->lookupTransform("odom", "base_link", ros::Time(0), transform);
    currentX = transform.getOrigin().x();
    currentY = transform.getOrigin().y();
    q = transform.getRotation();
    currentTheta = atan2((float)q.z(), (float)q.w()) * 2;
}

//These methods use the simple_move node
void JuskeshinoNavigation::startMoveDist(float distance)
{
    std_msgs::Float32 msg;
    msg.data = distance;
    JuskeshinoNavigation::_simple_move_status.status = actionlib_msgs::GoalStatus::PENDING;
    pubSimpleMoveDist.publish(msg);
    ros::spinOnce();
    ros::Duration(0.0333333).sleep();
}

void JuskeshinoNavigation::startMoveDistAngle(float distance, float angle)
{
    std_msgs::Float32MultiArray msg;
    msg.data.push_back(distance);
    msg.data.push_back(angle);
    JuskeshinoNavigation::_simple_move_status.status = actionlib_msgs::GoalStatus::PENDING;
    pubSimpleMoveDistAngle.publish(msg);
    ros::spinOnce();
    ros::Duration(0.0333333).sleep();
}

void JuskeshinoNavigation::startMoveLateral(float distance)
{
    std_msgs::Float32 msg;
    msg.data = distance;
    JuskeshinoNavigation::_simple_move_status.status = actionlib_msgs::GoalStatus::PENDING;
    pubSimpleMoveLateral.publish(msg);
    ros::spinOnce();
    ros::Duration(0.0333333).sleep();
}

bool JuskeshinoNavigation::moveDist(float distance, int timeOut_ms)
{
    JuskeshinoNavigation::startMoveDist(distance);
    return JuskeshinoNavigation::waitForLocalGoalReached(timeOut_ms);
}

bool JuskeshinoNavigation::moveDistAngle(float distance, float angle, int timeOut_ms)
{
    JuskeshinoNavigation::startMoveDistAngle(distance, angle);
    return JuskeshinoNavigation::waitForLocalGoalReached(timeOut_ms);
}

bool JuskeshinoNavigation::moveLateral(float distance, int timeOut_ms)
{
    JuskeshinoNavigation::startMoveLateral(distance);
    return JuskeshinoNavigation::waitForLocalGoalReached(timeOut_ms);
}

//These methods use the mvn_pln node.
void JuskeshinoNavigation::startGetClose(float x, float y, float angle)
{
    geometry_msgs::PoseStamped msg;
    msg.header.frame_id = "map";
    msg.pose.position.x = x;
    msg.pose.position.y = y;
    msg.pose.position.z = 0;
    msg.pose.orientation.x = 0;
    msg.pose.orientation.y = 0;
    msg.pose.orientation.z = sin(angle/2);
    msg.pose.orientation.w = cos(angle/2);
    JuskeshinoNavigation::_navigation_status.status = actionlib_msgs::GoalStatus::PENDING;
    pubMvnPlnGetCloseXYA.publish(msg);
    ros::spinOnce();
    ros::Duration(0.0333333).sleep();
}

void JuskeshinoNavigation::startGetClose(std::string location)
{
    
}

bool JuskeshinoNavigation::getClose(float x, float y, float angle, int timeOut_ms)
{
    JuskeshinoNavigation::startGetClose(x,y,angle);
    return JuskeshinoNavigation::waitForGlobalGoalReached(timeOut_ms);
}

bool JuskeshinoNavigation::getClose(std::string location, int timeOut_ms)
{
    
}

void JuskeshinoNavigation::stopNavigation()
{
    std_msgs::Empty msg;
    pubNavigationStop.publish(msg);
}


//Callbacks for subscribers
void JuskeshinoNavigation::callbackStop(const std_msgs::Empty::ConstPtr& msg)
{
    JuskeshinoNavigation::_stop = true;
}

void JuskeshinoNavigation::callbackNavigationStop(const std_msgs::Empty::ConstPtr& msg)
{
    JuskeshinoNavigation::_stop = true;
}

void JuskeshinoNavigation::callbackSimpleMoveStatus(const actionlib_msgs::GoalStatus::ConstPtr& msg)
{
    JuskeshinoNavigation::_simple_move_status = *msg;
}

void JuskeshinoNavigation::callbackNavigationStatus(const actionlib_msgs::GoalStatus::ConstPtr& msg)
{
    JuskeshinoNavigation::_navigation_status = *msg;
}

