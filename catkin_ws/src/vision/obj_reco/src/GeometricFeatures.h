#include "opencv2/opencv.hpp"

class GeometricFeatures
{
public:
    GeometricFeatures();
    ~GeometricFeatures();

    static cv::Mat get_horizontal_normals(cv::Mat& cloud, float normal_min_z, bool debug=false);
    static std::vector<cv::Vec3f> get_horizontal_planes(cv::Mat& cloud, float normal_min_z, float dist_threshold, float min_area,
                                                        cv::Mat& output_img_mask, bool debug=false);
    static std::vector<cv::Vec3f> above_horizontal_plane(cv::Mat& cloud, std::vector<cv::Vec3f> plane, float dist_threshold,
                                                         cv::Mat& output_img_mask, bool debug);
    
    static std::vector<cv::Vec3f> plane_from_points(cv::Vec3f p1, cv::Vec3f p2, cv::Vec3f p3);
    static std::vector<cv::Vec3f> plane_by_pca(cv::Mat points, float& approx_area, bool debug=false);
    static std::vector<cv::Vec3f> plane_by_ransac(cv::Mat cloud, cv::Mat mask, float normal_min_z, float distance_tol, float min_area,
                                                  std::vector<cv::Vec3f>& inliers, std::vector<cv::Vec3f>& outliers, bool debug=false);

    static float dist_point_to_line_segment(float px, float py, float pz, float x1, float y1, float z1, float x2, float y2, float z2);
    static std::vector<cv::Vec3f> line_by_pca(cv::Mat& points);
    static std::vector<cv::Vec3f> hough_lines(cv::Mat img_bin, cv::Mat xyz, float d_min, float d_max, int d_step, float theta_min,
                                              float theta_max, float theta_step, int threshold, cv::Mat& output_bgr, bool debug);
    
    static std::vector<cv::Vec3f> find_table_edge(cv::Mat& cloud, float normal_min_z, int canny_threshold1, int canny_threshold2,
                                                  int canny_window_size, int hough_min_rho, int hough_max_rho, int hough_step_rho,
                                                  float hough_min_theta, float hough_max_theta, float hough_step_theta, float hough_threshold,
                                                  cv::Mat& output_bgr, bool debug);
};
