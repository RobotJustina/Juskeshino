#include "ObjectRecognizer.h"

std::map<std::string, ObjectPattern> ObjectRecognizer::obj_patterns;

bool ObjectRecognizer::segment_by_contours(cv::Mat cloud, cv::Mat img_bgr, cv::Mat bin_mask, int min_points, std::vector<cv::Mat>& objects_bgr,
                                           std::vector<cv::Mat>& objects_xyz, std::vector<cv::Mat>& objects_masks, bool debug)
{
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(bin_mask, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    std::vector<cv::Mat> contour_masks;
    contour_masks.resize(contours.size());
    for(int i=0; i<contour_masks.size(); i++) contour_masks[i] = cv::Mat::zeros(img_bgr.rows, img_bgr.cols, CV_8UC1);

    objects_bgr.clear();
    objects_xyz.clear();
    objects_masks.clear();

    for( size_t i = 0; i< contours.size(); i++ )
    {
        cv::drawContours(contour_masks[i], contours, (int)i, 255, -1, cv::LINE_8, hierarchy, 0);
        int points_in_object = cv::countNonZero(contour_masks[i]);
        if(debug) std::cout << "ObjRecognizer.->Obj detected with " << points_in_object << " points. " << std::endl;
        if( points_in_object < min_points)
            continue;
        cv::Mat obj_bgr = img_bgr.clone().setTo(0, contour_masks[i]==0);
        cv::Mat obj_xyz = cloud.clone().setTo(0, contour_masks[i]==0);
        cv::Mat obj_msk = contour_masks[i].clone();
        objects_bgr.push_back(obj_bgr);
        objects_xyz.push_back(obj_xyz);
        objects_masks.push_back(obj_msk);
        if(debug){
            cv::imshow("Detected object bgr" + std::to_string(i), obj_bgr);
            cv::imshow("Detected object cloud" + std::to_string(i), obj_xyz);
            cv::imshow("Detected object mask " + std::to_string(i), obj_msk);
        }
    }
    return objects_bgr.size() > 0;
}

cv::Mat ObjectRecognizer::get_hue_histogram(cv::Mat img_bgr, cv::Mat mask, int hist_size, bool accumulate, cv::Mat current_hist)
{
    cv::Mat histogram = cv::Mat::zeros(1, hist_size, CV_32F);
    cv::Mat img_hsv;
    cv::cvtColor(img_bgr, img_hsv, cv::COLOR_BGR2HSV);
    for(int i=0; i< img_hsv.rows; i++)
        for(int j=0; j<img_hsv.cols; j++)
            if(img_hsv.at<cv::Vec3b>(i,j)[2] > 0)
                histogram.at<float>(0, img_hsv.at<cv::Vec3b>(i,j)[0]/(hist_size-1))++;
    cv::normalize(histogram, histogram, 1.0, 0, cv::NORM_INF);
    return histogram;
}

bool ObjectRecognizer::store_object_example(cv::Mat img_bgr, cv::Mat mask, std::string name, std::string training_folder, bool debug)
{
    if( !boost::filesystem::exists(training_folder))
    {
        std::cout <<"ObjReco.->Training folder does not exist!!! :'( " << training_folder << std::endl;
        return false;
    }
    training_folder += (training_folder.back() == '/' ? "" : "/") + name + "/";
    if(!boost::filesystem::exists(training_folder))
        boost::filesystem::create_directory(training_folder); 
    std::string file_name = name + boost::posix_time::to_iso_string(boost::posix_time::second_clock::universal_time()) + ".jpg";
    std::string file_path = training_folder + file_name;
    if(!boost::filesystem::portable_file_name(file_name))
    {
        std::cout << "ObjReco.->Object name or training folder has no valid names: " << file_path << std::endl;
        return false;
    }
    std::cout << "ObjReco.->Storing " << name << " example into file " << file_path << std::endl;
    cv::Mat example = cv::Mat(img_bgr, cv::boundingRect(mask));
    if(debug) cv::imshow("Example", example);
    cv::imwrite(file_path, example);
    return true;
}

bool ObjectRecognizer::train_from_folder(std::string training_folder, int histogram_size, bool debug)
{
    if( !boost::filesystem::exists(training_folder))
    {
        std::cout <<"ObjReco.->Training folder does not exist!!! :'( " << training_folder << std::endl;
        return false;
    }
    std::cout << "ObjReco.->Loading patterns from folder " << training_folder << std::endl;
    boost::filesystem::directory_iterator end_itr;
    for(boost::filesystem::directory_iterator folder_itr(training_folder); folder_itr != end_itr; ++folder_itr)
        if(boost::filesystem::is_directory(folder_itr->status()))
        {
            std::string obj_folder = folder_itr->path().string();
            std::string obj_name = obj_folder.substr(obj_folder.find_last_of("/") + 1);
            std::cout << "Loading pattern for : " << obj_name << std::endl;
            cv::Mat histogram = cv::Mat::zeros(1, histogram_size, CV_32F);
            for(boost::filesystem::directory_iterator file_itr(folder_itr->path()); file_itr != end_itr; ++file_itr)
                if(!boost::filesystem::is_directory(file_itr->status()))
                {
                    cv::Mat img_hsv;
                    cv::Mat img_bgr = cv::imread(file_itr->path().string());
                    cv::cvtColor(img_bgr, img_hsv, cv::COLOR_BGR2HSV);
                    for(int i=0; i< img_hsv.rows; i++)
                        for(int j=0; j<img_hsv.cols; j++)
                            if(img_hsv.at<cv::Vec3b>(i,j)[2] > 0)
                                histogram.at<float>(0, img_hsv.at<cv::Vec3b>(i,j)[0]/(histogram_size-1))++;
                }
            cv::normalize(histogram, histogram, 1.0, 0, cv::NORM_INF);
            if(debug) std::cout << histogram << std::endl;
            ObjectRecognizer::obj_patterns[obj_name] = ObjectPattern(obj_name, histogram);
        }
    return true;
}

bool ObjectRecognizer::recognize(cv::Mat img_bgr, cv::Mat mask, int hist_size, std::string& recognized, double& similarity, bool debug)
{
    cv::Mat hist = get_hue_histogram(img_bgr, mask, hist_size);
    if(debug) std::cout << "Histogram detected: " << hist << std::endl;
    similarity = -1;
    recognized = "";
    for(std::map<std::string, ObjectPattern>::iterator it = ObjectRecognizer::obj_patterns.begin(); it!=ObjectRecognizer::obj_patterns.end(); it++)
    {
        double obj_similarity = cv::compareHist(hist, it->second.histogram, cv::HISTCMP_CORREL);
        if(obj_similarity > similarity)
        {
            recognized = it->first;
            similarity = obj_similarity;
        }
        if(debug) std::cout << "Comparison with " << it->first << ": " << cv::compareHist(hist, it->second.histogram, cv::HISTCMP_CORREL) << std::endl;
    }
    return recognized != "";
}
