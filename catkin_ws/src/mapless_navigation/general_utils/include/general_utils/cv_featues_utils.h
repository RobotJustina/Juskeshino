#pragma once
#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

// CV Features >>
class CVFeatures
{
    private:
        CVFeatures() = delete;  // Disallow object creation
        ~CVFeatures() = delete; // Disallow object deletion
    public:
        static void harrisCorners(std::string window_name, cv::Mat image, int threshold, bool verbose = false);
};
// CV Features <<