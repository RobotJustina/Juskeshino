#pragma once
#include "opencv2/opencv.hpp"
#include "sensor_msgs/PointCloud2.h"
#include "pcl_ros/transforms.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "vision_msgs/RecognizeObjects.h"
#include "vision_msgs/RecognizeObject.h"
#include <cv_bridge/cv_bridge.h>

class Utils
{
public:
    Utils();
    ~Utils();

    static bool debug;
    static void pointcloud_msg2_cv_mat(sensor_msgs::PointCloud2& pc_msg, cv::Mat& bgr_dest, cv::Mat& pc_dest);
    static void cv_mat2_pointcloud_msg(cv::Mat& src_bgr, cv::Mat& src_xyz, std::string frame_id, sensor_msgs::PointCloud2& msg_cloud);
    static void filter_by_distance(cv::Mat& cloud, cv::Mat& img, float min_x, float min_y, float min_z, float max_x,
                                   float max_y, float max_z, cv::Mat& filtered_cloud, cv::Mat& filtered_img);
    static void transform_cloud_wrt_base(sensor_msgs::PointCloud2& cloud, cv::Mat& bgr_dest, cv::Mat& cloud_dest,
                                         tf::TransformListener* tf_listener);

    static visualization_msgs::Marker get_lines_marker(std::vector<cv::Vec3f> lines);
    static std::vector<geometry_msgs::Point> get_lines_msg(std::vector<cv::Vec3f> line);

    static visualization_msgs::Marker get_plane_marker(std::vector<cv::Vec3f> plane, float thickness);

    static vision_msgs::RecognizeObjects::Response get_recog_objects_response(std::vector<cv::Mat>& objects_bgr, std::vector<cv::Mat>& objects_xyz,
                                                                              std::vector<cv::Mat>& objects_masks, std::vector<std::string>& labels,
                                                                              std::vector<double>& confidences, cv::Mat& result_img, std::string frame_id);
    static vision_msgs::RecognizeObject::Response get_recog_object_response(cv::Mat& obj_bgr, cv::Mat& obj_xyz, cv::Mat& obj_msk, std::string label,
                                                                            double confidence, std::string frame_id);
    static vision_msgs::VisionObject get_vision_object_msg(cv::Mat& obj_bgr, cv::Mat& obj_xyz, cv::Mat& obj_mask, std::string label,
                                                           double confidence, std::string frame_id);

    static visualization_msgs::MarkerArray get_objects_markers(std::vector<vision_msgs::VisionObject>& objs);
    static visualization_msgs::MarkerArray get_object_marker(vision_msgs::VisionObject& obj);
};
