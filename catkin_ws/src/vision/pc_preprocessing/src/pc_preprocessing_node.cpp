#include "opencv2/opencv.hpp"
#include "ros/ros.h"
#include "vision_msgs/PreprocessPointCloud.h"
#include "pcl_ros/transforms.h"

tf::TransformListener* tf_listener;

bool callback_point_cloud_prepocessing(vision_msgs::PreprocessPointCloud::Request& req, vision_msgs::PreprocessPointCloud::Response& resp)
{
    if(req.input_cloud.header.frame_id != "base_link")
    {
        std::cout << "ObjReco.->Transforming point cloud to base link reference" << std::endl;
        return pcl_ros::transformPointCloud("base_link", req.input_cloud, resp.output_cloud, *tf_listener);
    }
    resp.output_cloud = req.input_cloud;
    return true;
}



bool callback_transform_cloud_to_base_link(vision_msgs::PreprocessPointCloud::Request& req, vision_msgs::PreprocessPointCloud::Response& resp)
{
    if(req.input_cloud.header.frame_id != "base_link")
    {
        std::cout << "ObjReco.->Transforming point cloud to base link reference" << std::endl;
        return pcl_ros::transformPointCloud("base_link", req.input_cloud, resp.output_cloud, *tf_listener);
    }
    resp.output_cloud = req.input_cloud;
    return true;
}

int main(int argc, char** argv)
{
    std::cout << "INITIALIZING OBJECT RECOGNIZER BY MR. YISUS (CORRECTED AND IMPROVED BY MARCOSOFT)" << std::endl;
    ros::init(argc, argv, "obj_reco_node");
    ros::NodeHandle n;


    ros::ServiceServer srvTransformCloud  = n.advertiseService("/vision/point_cloud_to_base_link", callback_transform_cloud_to_base_link);
    
    ros::Rate loop(30);
    tf_listener = new tf::TransformListener();

    while(ros::ok() && cv::waitKey(10) != 27)
    {
        ros::spinOnce();
        loop.sleep();
    }
}
