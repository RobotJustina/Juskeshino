#include "general_utils/cv_featues_utils.h"

// CV Features >>
void CVFeatures::harrisCorners(std::string window_name, cv::Mat image, int threshold, bool verbose){
    //ROS_INFO("Test cv utils");
    cv::Mat img_gray;
    int blockSize = 2;
    int apertureSize = 3;
    double k = 0.04;
    try
    {

        //cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;
        cv::cvtColor(image, img_gray, cv::COLOR_RGB2GRAY);

        cv::Mat dst = cv::Mat::zeros(image.size(), CV_32FC1);
        cv::cornerHarris(img_gray, dst, blockSize, apertureSize, k);

        cv::Mat dst_norm, dst_norm_scaled;
        cv::normalize(dst, dst_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat());
        cv::convertScaleAbs(dst_norm, dst_norm_scaled);
        for (int i = 0; i < dst_norm.rows; i++)
        {
            for (int j = 0; j < dst_norm.cols; j++)
            {
                if ((int)dst_norm.at<float>(i, j) > threshold)
                {
                    cv::circle(dst_norm_scaled, cv::Point(j, i), 5, cv::Scalar(0), 2, 8, 0);
                }
            }
        }

        cv::imshow(window_name, dst_norm_scaled);
        cv::waitKey(1);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("Could not convert to image!");
    }
}
// CV Features <<