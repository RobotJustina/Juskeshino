#include "opencv2/opencv.hpp"
#include "boost/filesystem.hpp"
#include "boost/date_time/posix_time/posix_time.hpp"
#include "ObjectPattern.h"

class ObjectRecognizer
{
public:
    ObjectRecognizer();
    ~ObjectRecognizer();

    static std::map<std::string, ObjectPattern> obj_patterns;

    static bool segment_by_contours(cv::Mat cloud, cv::Mat img_bgr, cv::Mat bin_mask, int min_points, std::vector<cv::Mat>& objects_bgr,
                                    std::vector<cv::Mat>& objects_xyz, std::vector<cv::Mat>& object_masks, bool debug=false);
    static cv::Mat get_hue_histogram(cv::Mat img_bgr, cv::Mat maks, int hist_size, bool accumulate=false, cv::Mat current_hist = cv::Mat());
    static bool store_object_example(cv::Mat img_bgr, cv::Mat mask, std::string name, std::string training_folder, bool debug=false);
    static bool train_from_folder(std::string training_folder, int histogram_size);
    static bool recognize(cv::Mat img_bgr, cv::Mat mask, int hist_size, std::string& recognized);
};
