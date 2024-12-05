#include "ros/ros.h"
#include "juskeshino_tools/JuskeshinoVision.h"
#include "juskeshino_tools/JuskeshinoNavigation.h"

#define SM_INIT                 0 
#define SM_WAITING_FOR_NEW_TASK 10
#define SM_GO_TO_GOAL_POSE      20


int main(int argc, char** argv)
{
    std::cout << "INITIALIZING DEMO NODE ... " << std::endl;
    ros::init(argc, argv, "act_pln");
    ros::NodeHandle n;
    ros::Rate loop(10);
    JuskeshinoNavigation::setNodeHandle(&n);

    while(ros::ok())
    {
        JuskeshinoNavigation::moveDist(0.5, 5000);
        JuskeshinoNavigation::moveDistAngle(0.0, 3.141592, 5000);
        JuskeshinoNavigation::moveDist(0.5, 5000);
        JuskeshinoNavigation::moveDistAngle(0.0, 3.141592, 5000);
        ros::spinOnce();
        loop.sleep();
    }
}
