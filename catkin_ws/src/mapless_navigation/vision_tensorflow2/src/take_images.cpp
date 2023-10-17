#include <ros/ros.h>
#include <ros/package.h> // TODO: check
#include <image_transport/image_transport.h>
//#include <opencv2/highgui/highgui.hpp>  // <<<---error!!
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/String.h>

#include "general_utils/files_utils.h"

std::string path_files;

void imageCallback(const sensor_msgs::ImageConstPtr &msg)
{   
    int count = 0;
    int cv_writer;
    try
    {
        cv::imshow("Camera_viewer", cv_bridge::toCvShare(msg, "bgr8")->image); 
        std::string frame_id = std::to_string(msg->header.stamp.sec) + "_" + std::to_string(msg->header.stamp.nsec).substr(0, 4);
        std::string img_name = msg->header.frame_id + "_" + frame_id.c_str() + ".jpg";

        cv_writer = cv::imwrite((path_files + img_name), cv_bridge::toCvShare(msg, "bgr8")->image);
        if(cv_writer)
            ROS_INFO_STREAM("image saved: " << img_name.c_str() << std::endl);
        else    
            ROS_WARN("image not saved!");
        
        cv::waitKey(1);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

int main(int argc, char **argv)
{

    ROS_DEBUG("Initializing image_taker...");
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
    ros::init(argc, argv, "image_taker");
    ros::NodeHandle nh;
    // ros::Publisher image_pub = n.advertise<std_msgs::String>("image_viewer", 10);
    cv::namedWindow("Camera_viewer");
    image_transport::ImageTransport img_t(nh);
    image_transport::Subscriber img_sub = img_t.subscribe("hsrb/head_center_camera/image_raw", 1, imageCallback);
    
    path_files = ros::package::getPath("vision_tensorflow2") + "/images/";
    ROS_INFO("the path: %s", path_files.c_str());
    //DirectoryUtils::replaceDir("../images", true);
    DirectoryUtils::replaceDir(path_files, true);

    ros::Rate loop_rate(2);

    //int counter = 0;
    while (ros::ok())
    {
 

        //counter++;
        ros::spinOnce();
        loop_rate.sleep();
    }
    ros::spin(); // TODO: Remove
    cv::destroyWindow("Camera_viewer");
    return 0;
}