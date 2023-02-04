#include "opencv2/opencv.hpp"

class ObjectRecognizer
{
public:
    ObjectRecognizer();
    ~ObjectRecognizer();

    static bool segment_by_contours(cv::Mat cloud, cv::Mat img_bgr, cv::Mat bin_mask, int min_points, std::vector<cv::Mat>& objects_bgr,
                                    std::vector<cv::Mat>& objects_xyz, std::vector<cv::Mat>& object_masks, bool debug=false);
    static cv::Mat get_hue_histogram(cv::Mat img_bgr, cv::Mat maks, bool accumulate=false);
};
