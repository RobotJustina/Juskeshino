#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "opencv2/opencv.hpp"
#include "Utils.h"

tf::TransformListener* tf_listener;

void vq_cuda_main(unsigned char* src_bgr, float* src_xyz, unsigned char* dst_bgr, float* dst_xyz, double* tf, float* centroids);

void callback_point_cloud(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    cv::Mat bgr, xyz;
    cv::Mat bgr_processed = cv::Mat::zeros(480, 640, CV_8UC3);
    cv::Mat xyz_processed = cv::Mat::zeros(480, 640, CV_32FC3);
    Utils::pointcloud_msg2_cv_mat(msg, bgr, xyz);
    double* tf = Utils::get_eigen_transform("base_link", msg->header.frame_id, tf_listener).data();
    // for(int i=0; i<16; i++)
    //    std::cout << tf[i] << "   ";
    // std::cout << std::endl;
    float centroids[64*3];
    vq_cuda_main(bgr.data, (float*)xyz.data, bgr_processed.data, (float*)xyz_processed.data, tf, centroids);
    std::cout << centroids[0] << "  " << centroids[1] << "  " << centroids[2] << std::endl;
    cv::imshow("BGR", bgr);
    cv::imshow("New BGR", bgr_processed);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vq_cuda");	  
    ros::NodeHandle n;
    ros::Subscriber sub_point_cloud = n.subscribe("/hardware/realsense/points", 1, callback_point_cloud);
    ros::Rate loop_rate(30);
    tf_listener = new tf::TransformListener();
    
    while(ros::ok() && cv::waitKey(10) != 27)
    {    
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
