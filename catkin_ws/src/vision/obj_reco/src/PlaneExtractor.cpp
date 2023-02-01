#include "PlaneExtractor.h"
#include "Utils.h"

float PlaneExtractor::normals_tol = 0.8;
int   PlaneExtractor::canny_threshold1 = 30;
int   PlaneExtractor::canny_threshold2 = 100;
int   PlaneExtractor::canny_window_size = 3;
int   PlaneExtractor::hough_threshold = 400;
int   PlaneExtractor::hough_min_rho  = 0;
int   PlaneExtractor::hough_max_rho  = 800;
int   PlaneExtractor::hough_step_rho = 10; 
float PlaneExtractor::hough_min_theta  = 1.5708-0.5;
float PlaneExtractor::hough_max_theta  = 1.5708+0.5;
float PlaneExtractor::hough_step_theta = 0.03;

std::vector<cv::Point> PlaneExtractor::extract_horizontal_planes(sensor_msgs::PointCloud2& point_cloud_msg, tf::TransformListener* tf_listener)
{
    cv::Mat img, cloud;
    Utils::transform_cloud_wrt_base(point_cloud_msg, img, cloud, tf_listener);
    Utils::filter_by_distance(cloud, img, cloud, img);
    cv::Mat normals = PlaneExtractor::get_horizontal_normals(cloud);
    if(Utils::debug) cv::imshow("Horizontal normals", cloud);
}

cv::Mat PlaneExtractor::get_horizontal_normals(cv::Mat& cloud)
{
    float mag;
    cv::Vec3f p, p_ne, p_se, p_nw, p_sw, v1, v2, normal;
    cv::Mat normals = cv::Mat::zeros(cv::Size(cloud.cols, cloud.rows), CV_32FC3);
    for(size_t i=1; i<cloud.rows-1; i++)
        for(size_t j=1; j<cloud.cols-1; j++)
        {
            p    = cloud.at<cv::Vec3f>(i,j);
            p_ne = cloud.at<cv::Vec3f>(i+1,j+1);
            p_nw = cloud.at<cv::Vec3f>(i+1,j-1);
            p_sw = cloud.at<cv::Vec3f>(i-1,j-1);
            p_se = cloud.at<cv::Vec3f>(i-1,j+1);
            if(p == cv::Vec3f(0,0,0) || p_ne == cv::Vec3f(0,0,0) || p_nw == cv::Vec3f(0,0,0) ||
               p_sw == cv::Vec3f(0,0,0) || p_se == cv::Vec3f(0,0,0))
                continue;
            normal = (p_ne - p_sw).cross(p_nw - p_se);
            mag = sqrt(normal[0]*normal[0] + normal[1]*normal[1] + normal[2]*normal[2]);
            if(mag != 0) normal /= mag;
            if(normal[2] < 0) normal = -normal;
            if(normal[2] < PlaneExtractor::normals_tol)
                continue;
            normals.at<cv::Vec3f>(i,j) = normal;
        }
    if(Utils::debug)
        cv::imshow("Horizontal Normals", normals);
    return normals;
}

std::vector<cv::Vec3f> PlaneExtractor::find_horizontal_lines(cv::Mat& normals, cv::Mat& cloud, cv::Mat& output_bgr)
{
    cv::Mat grayscale_normals, borders;
    
    cv::cvtColor(normals, grayscale_normals, cv::COLOR_BGR2GRAY);
    grayscale_normals.convertTo(grayscale_normals, CV_8UC1, 255);

    cv::Canny(grayscale_normals, borders, PlaneExtractor::canny_threshold1, PlaneExtractor::canny_threshold2, PlaneExtractor::canny_window_size);
    std::vector<cv::Vec2f> lines_img;
    std::vector<cv::Vec3f> lines = Utils::hough_lines(borders, cloud,
                                                      PlaneExtractor::hough_min_rho,   PlaneExtractor::hough_max_rho,   PlaneExtractor::hough_step_rho,
                                                      PlaneExtractor::hough_min_theta, PlaneExtractor::hough_max_theta, PlaneExtractor::hough_step_theta,
                                                      PlaneExtractor::hough_threshold, lines_img);
    if(Utils::debug)
    {
        Utils::draw_lines(output_bgr, lines_img);
        cv::imshow("Hough Lines", output_bgr);
        cv::imshow("Canny borders", borders);
    }
    return lines;
}

std::vector<cv::Vec3f> PlaneExtractor::find_nearest_horizontal_line(std::vector<cv::Vec3f>& lines)
{
    float min_dist = 1000;
    int nearest_line_idx = -1;
    cv::Vec3f p1, p2;
    std::vector<cv::Vec3f> nearest_line;
    if(lines.size() < 2) return nearest_line;
    for(int i=0; i<lines.size(); i+=2)
    {
        p1 = lines[i  ];
        p2 = lines[i+1];
        float d = Utils::dist_point_to_segment(0, 0, 0, p1[0], p1[1], p1[2], p2[0], p2[1], p2[2]);
        if(d < min_dist)
        {
            min_dist = d;
            nearest_line_idx = i;
        }
        if(Utils::debug)
            std::cout << "ObjReco-Planes->Candidate Line " << i/2 << ": " << p1 << "    " << p2 << "   distance: " << d << std::endl;
    }
    nearest_line.push_back(lines[nearest_line_idx]);
    nearest_line.push_back(lines[nearest_line_idx+1]);
    if(Utils::debug) std::cout << "ObjReco-Planes->Nearest line: " << lines[nearest_line_idx] << "  " << lines[nearest_line_idx + 1] <<std::endl;
    return nearest_line;
}

std::vector<geometry_msgs::Point> PlaneExtractor::find_table_border(sensor_msgs::PointCloud2& point_cloud_msg, tf::TransformListener* tf_listener)
{
    cv::Mat img, cloud;
    Utils::transform_cloud_wrt_base(point_cloud_msg, img, cloud, tf_listener);
    Utils::filter_by_distance(cloud, img, cloud, img);
    cv::Mat normals = PlaneExtractor::get_horizontal_normals(cloud);
    std::vector<cv::Vec3f> lines = PlaneExtractor::find_horizontal_lines(normals, cloud, img);
    std::vector<cv::Vec3f> nearest_line = find_nearest_horizontal_line(lines);
    return Utils::get_line_msg(nearest_line);
}
