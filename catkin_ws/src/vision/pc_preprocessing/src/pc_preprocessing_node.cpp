#include "ros/ros.h"
#include <ros/package.h>
#include "vision_msgs/FindLines.h"
#include "vision_msgs/FindPlanes.h"
#include "vision_msgs/RecognizeObjects.h"
#include "vision_msgs/RecognizeObject.h"
#include "vision_msgs/VisionObject.h"
#include "vision_msgs/TrainObject.h"
#include "vision_msgs/PreprocessPointCloud.h"
#include "visualization_msgs/MarkerArray.h"
#include "Utils.h"
#include "GeometricFeatures.h"
#include "ObjectRecognizer.h"
#include <cv_bridge/cv_bridge.h>

tf::TransformListener* tf_listener;
ros::Publisher pubMarkers;
ros::Publisher pubMarkerArray;
ros::Publisher pubResultCloud;
ros::ServiceClient cltRecogYolo;

bool  debug = false;
float min_x =  0.3;
float min_y = -2.0;
float min_z =  0.3;
float max_x =  2.0;
float max_y =  2.0;
float max_z =  2.0;
float normal_min_z = 0.8;
int   canny_threshold1  = 30;
int   canny_threshold2  = 100;
int   canny_window_size = 3;
int   hough_min_rho  = 0;
int   hough_max_rho  = 800;
int   hough_step_rho = 10;
float hough_min_theta  = 1.5708-0.5;
float hough_max_theta  = 1.5708+0.5;
float hough_step_theta = 0.03;
float hough_threshold  = 400;
float plane_dist_threshold = 0.05;
float plane_min_area = 0.5;
int   histogram_size = 18;
int   min_points_per_object = 1000;
bool  use_yolo = true;
std::string training_dir;




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
    ros::ServiceServer srvTransformCloud  = n.advertiseService("/vision/point_cloud_prepocessing", callback_point_cloud_prepocessing);

    
    ros::Rate loop(30);
    tf_listener = new tf::TransformListener();

    while(ros::ok() && cv::waitKey(10) != 27)
    {
        ros::spinOnce();
        loop.sleep();
    }
}
