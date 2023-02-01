#include "geometry_msgs/Point.h"
#include "shape_msgs/Plane.h"
#include "sensor_msgs/PointCloud2.h"
#include "opencv2/opencv.hpp"
#include "Utils.h"

class PlaneExtractor
{
public:
    PlaneExtractor();
    ~PlaneExtractor();

    static float normals_tol;
    static int   canny_threshold1;
    static int   canny_threshold2;
    static int   canny_window_size;
    static int   hough_threshold;
    static int   hough_min_rho;
    static int   hough_max_rho;
    static int   hough_step_rho;
    static float hough_min_theta;
    static float hough_max_theta;
    static float hough_step_theta;
    
    static std::vector<cv::Vec3f> get_horizontal_planes(sensor_msgs::PointCloud2& point_cloud_msg, tf::TransformListener* tf_listener);
    static cv::Mat get_horizontal_normals(cv::Mat& cloud);
    static std::vector<cv::Vec3f> find_horizontal_lines(cv::Mat& normals, cv::Mat& cloud, cv::Mat& output_bgr);
    static std::vector<cv::Vec3f> find_nearest_horizontal_line(std::vector<cv::Vec3f>& lines);
    static std::vector<geometry_msgs::Point> find_table_border(sensor_msgs::PointCloud2& point_cloud_msg, tf::TransformListener* tf_listener);
};
