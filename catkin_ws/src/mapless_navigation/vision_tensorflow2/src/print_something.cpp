#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/String.h>

#include "general_utils/files_utils.h"


int main(int argc, char **argv)
{

    ROS_INFO("Initializing printer_node...");
    /**
     * The ros::init() function needs to see argc and argv so that it can perform
     * any ROS arguments and name remapping that were provided at the command line.
     * For programmatic remappings you can use a different version of init() which takes
     * remappings directly, but for most command-line programs, passing argc and argv is
     * the easiest way to do it.  The third argument to init() is the name of the node.
     *
     * You must call one of the versions of ros::init() before using any other
     * part of the ROS system.
     */
    ros::init(argc, argv, "printer_node");
    ros::NodeHandle nh;


    ros::Rate loop_rate(5);
    

    //int counter = 0;
    while (ros::ok())
    {
        testFunction();
        
        ros::spinOnce();
        loop_rate.sleep();
    }
    ros::spin();
    return 0;
}