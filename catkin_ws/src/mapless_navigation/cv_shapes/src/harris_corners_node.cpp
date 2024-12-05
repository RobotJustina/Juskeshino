#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/PoseStamped.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

#include "general_utils/cv_featues_utils.h"

int threshold = 135;

void void_func(int, void *)
{
}

void positionPoseCallback(const geometry_msgs::PoseStamped &msg)
{
    ROS_INFO_STREAM("Pose: " << std::endl << msg.pose.position << std::endl);
}

void harrisCornersCallback(const sensor_msgs::ImageConstPtr &msg)
{
    cv::Mat image;
    try
    {
        image = cv_bridge::toCvShare(msg, "bgr8")->image;
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("Could not convert to image!");
    }
    CVFeatures::harrisCorners("Corners window", image, threshold, true);
}

void harrisCornersCompImgCallback(const sensor_msgs::CompressedImageConstPtr &msg)
{
    cv::Mat compress_image;
    try
    {
        compress_image = cv::imdecode(cv::Mat(msg->data), 1);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("Could not convert to image!");
    }
    CVFeatures::harrisCorners("Corners window", compress_image, threshold, true);
}

void showComprImgCallback(const sensor_msgs::CompressedImageConstPtr &msg)
{
    try
    {
        cv::Mat compress_image = cv::imdecode(cv::Mat(msg->data), 1);
        cv::imshow("img_compressed_view", compress_image);
        cv::waitKey(1);
    }
    catch (const cv::Exception &ex)
    {
        ROS_ERROR("Could not convert to image! %s", ex.what());
    }
}

int main(int argc, char *argv[])
{
    // Initialize the node
    ros::init(argc, argv, "harris_corners_node");
    // Start the node by initialising a node handle
    ros::NodeHandle nh("~");
    // Display the namespace of the node handle
    ROS_INFO_STREAM("namespace of nh = " << nh.getNamespace());
    cv::namedWindow("img_compressed_view");

    cv::namedWindow("Corners window");
    cv::createTrackbar("Threshold: ", "Corners window", &threshold, 255);

    ros::Subscriber sub_show_position = nh.subscribe("/hsrb/base_pose", 10, positionPoseCallback);
    ros::Subscriber sub_show_compr_img = nh.subscribe("/hsrb/head_center_camera/image_raw/compressed", 2, showComprImgCallback);
    //ros::Subscriber sub_harris_corners_c_img = nh.subscribe("/hsrb/head_center_camera/image_raw/compressed", 2, harrisCornersCompImgCallback);
    ros::Subscriber sub_harris_corners = nh.subscribe("/hsrb/head_center_camera/image_raw", 2, harrisCornersCallback); // more edges visible

    ros::Rate loop_rate(5);
    // int counter = 0;
    while (ros::ok())
    {
        // ROS_INFO("loop");

        ros::spinOnce();
        loop_rate.sleep();
    }
    ros::spin();
    cv::destroyWindow("Corners window");
    cv::destroyWindow("img_compressed_view");
    // Main has ended, return 0
    return 0;
}
