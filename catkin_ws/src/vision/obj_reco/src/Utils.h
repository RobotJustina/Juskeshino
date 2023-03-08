#pragma once
#include "opencv2/opencv.hpp"
#include "sensor_msgs/PointCloud2.h"
#include "pcl_ros/transforms.h"
#include "visualization_msgs/Marker.h"

class Utils
{
public:
    Utils();
    ~Utils();

    static bool debug;
    static void pointcloud_msg2_cv_mat(sensor_msgs::PointCloud2& pc_msg, cv::Mat& bgr_dest, cv::Mat& pc_dest);
    static void filter_by_distance(cv::Mat& cloud, cv::Mat& img, float min_x, float min_y, float min_z, float max_x,
                                   float max_y, float max_z, cv::Mat& filtered_cloud, cv::Mat& filtered_img);
    static void transform_cloud_wrt_base(sensor_msgs::PointCloud2& cloud, cv::Mat& bgr_dest, cv::Mat& cloud_dest,
                                         tf::TransformListener* tf_listener);

    static visualization_msgs::Marker get_lines_marker(std::vector<cv::Vec3f> lines);
    static std::vector<geometry_msgs::Point> get_lines_msg(std::vector<cv::Vec3f> line);

    static visualization_msgs::Marker get_plane_marker(std::vector<cv::Vec3f> plane);
};
