#include "ObjectRecognizer.h"

bool ObjectRecognizer::segment_by_contours(cv::Mat cloud, cv::Mat img_bgr, cv::Mat bin_mask, int min_points, std::vector<cv::Mat>& objects_bgr,
                                           std::vector<cv::Mat>& objects_xyz, std::vector<cv::Mat>& object_masks, bool debug)
{
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    std::cout << "before find contours" << std::endl;
    cv::findContours(bin_mask, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    std::cout << "after find contours" << std::endl;
    object_masks.resize(contours.size());    
    std::vector<cv::Mat> detected_obj_masks_8UC3(contours.size());
    std::vector<cv::Mat> detected_obj_masks_32FC3(contours.size());
    objects_bgr.clear();
    objects_xyz.clear();
    for(int i=0; i< object_masks.size(); i++) object_masks[i] = cv::Mat::zeros(img_bgr.rows, img_bgr.cols, CV_8UC1);
    for( size_t i = 0; i< contours.size(); i++ )
    {
        std::cout << "before draw contours " << i << std::endl;
        cv::drawContours(object_masks[i], contours, (int)i, 255, -1, cv::LINE_8, hierarchy, 0);
        std::cout << "after draw contours " << i << std::endl;
        if(cv::countNonZero(object_masks[i]) < min_points)
            continue;
        cv::cvtColor(object_masks[i], detected_obj_masks_8UC3[i], cv::COLOR_GRAY2BGR);
        detected_obj_masks_8UC3[i].convertTo(detected_obj_masks_32FC3, CV_32FC3);
        cv::Mat obj_bgr, obj_xyz;
        cv::bitwise_and(img_bgr, detected_obj_masks_8UC3[i] , obj_bgr);
        cv::bitwise_and(cloud  , detected_obj_masks_32FC3[i], obj_xyz);
        objects_bgr.push_back(obj_bgr.clone());
        objects_xyz.push_back(obj_xyz.clone());
        if(debug)
            cv::imshow("Detected object " + std::to_string(i), obj_bgr);
    }
    
}
