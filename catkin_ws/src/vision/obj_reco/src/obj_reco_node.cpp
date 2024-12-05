#include "ros/ros.h"
#include <ros/package.h>
#include "vision_msgs/FindLines.h"
#include "vision_msgs/FindPlanes.h"
#include "vision_msgs/RecognizeObjects.h"
#include "vision_msgs/RecognizeObject.h"
#include "vision_msgs/VisionObject.h"
#include "vision_msgs/TrainObject.h"
#include "vision_msgs/PreprocessPointCloud.h"
#include "visualization_msgs/MarkerArray.h"
#include "Utils.h"
#include "GeometricFeatures.h"
#include "ObjectRecognizer.h"
#include <cv_bridge/cv_bridge.h>

tf::TransformListener* tf_listener;
ros::Publisher pubMarkers;
ros::Publisher pubMarkerArray;
ros::Publisher pubResultCloud;

bool  debug = false;
float min_x =  0.3;
float min_y = -2.0;
float min_z =  0.3;
float max_x =  2.0;
float max_y =  2.0;
float max_z =  2.0;
float normal_min_z = 0.8;
int   canny_threshold1  = 30;
int   canny_threshold2  = 100;
int   canny_window_size = 3;
int   hough_min_rho  = 0;
int   hough_max_rho  = 800;
int   hough_step_rho = 10;
float hough_min_theta  = 1.5708-0.5;
float hough_max_theta  = 1.5708+0.5;
float hough_step_theta = 0.03;
float hough_threshold  = 400;
float plane_dist_threshold = 0.05;
float plane_min_area = 0.5;
int   histogram_size = 18;
int   min_points_per_object = 1000;
std::string training_dir;

bool callback_find_table_edge(vision_msgs::FindLines::Request& req, vision_msgs::FindLines::Response& resp)
{
    std::cout << std::endl << "ObjReco.->Executing srvFindLines (Jebusian method)." << std::endl;
    cv::Mat img, cloud;
    std::vector<cv::Vec3f> edge_line;
        
    Utils::transform_cloud_wrt_base(req.point_cloud, img, cloud, tf_listener);
    Utils::filter_by_distance(cloud, img, min_x, min_y, min_z, max_x, max_y, max_z, cloud, img);
    edge_line = GeometricFeatures::find_table_edge(cloud, normal_min_z, canny_threshold1, canny_threshold2, canny_window_size, hough_min_rho, hough_max_rho,
                                                   hough_step_rho, hough_min_theta, hough_max_theta, hough_step_theta, hough_threshold, img, debug);
    resp.lines = Utils::get_lines_msg(edge_line);
    pubMarkers.publish(Utils::get_lines_marker(edge_line));
    if(edge_line.size() > 0) std::cout << "ObjReco.->Found line: " << resp.lines[0] << " - " <<resp.lines[1] <<std::endl;
    else std::cout << "ObjReco.->Cannot find lines. " << std::endl;
    return resp.lines.size() > 0;
}

bool callback_find_planes(vision_msgs::FindPlanes::Request& req, vision_msgs::FindPlanes::Response& resp)
{
    std::cout << std::endl << "ObjReco.->Executing srvFindPlanes." << std::endl;
    cv::Mat img, cloud, output_mask;
        
    Utils::transform_cloud_wrt_base(req.point_cloud, img, cloud, tf_listener);
    Utils::filter_by_distance(cloud, img, min_x, min_y, min_z, max_x, max_y, max_z, cloud, img);
    std::vector<cv::Vec3f> plane = GeometricFeatures::get_horizontal_planes(cloud, normal_min_z, plane_dist_threshold, plane_min_area, output_mask, debug);
    pubMarkers.publish(Utils::get_plane_marker(plane, 2*plane_dist_threshold));
    if(plane.size() > 0) std::cout << "ObjReco.->Found Plane. Center: " << plane[0] << " Normal: " << plane[1] <<std::endl;
    else std::cout << "ObjReco.->Cannot find plane. " << std::endl;
    return plane.size() > 0;
}

bool callback_detect_and_recog_objs(vision_msgs::RecognizeObjects::Request& req, vision_msgs::RecognizeObjects::Response& resp)
{
    std::cout << "ObjReco.->Recognizing objects by Jebug's method (corrected and improved by Marcosoft)." << std::endl;
    cv::Mat img, cloud, output_mask;
    if(debug) cv::destroyAllWindows();
        
    Utils::transform_cloud_wrt_base(req.point_cloud, img, cloud, tf_listener);
    Utils::filter_by_distance(cloud, img, min_x, min_y, min_z, max_x, max_y, max_z, cloud, img);
    std::vector<cv::Vec3f> plane = GeometricFeatures::get_horizontal_planes(cloud, normal_min_z, plane_dist_threshold, plane_min_area, output_mask, debug);
    std::vector<cv::Vec3f> points = GeometricFeatures::above_horizontal_plane(cloud, plane, plane_dist_threshold+0.005, output_mask, debug);

    std::vector<cv::Mat> objects_bgr, objects_xyz, objects_masks;
    if(!ObjectRecognizer::segment_by_contours(cloud, img, output_mask, min_points_per_object, objects_bgr, objects_xyz, objects_masks, debug)) return false;

    img = cv::Mat::zeros(img.rows, img.cols, CV_8UC3);
    std::vector<std::string> labels(objects_bgr.size(), "");
    std::vector<double> confidences(objects_bgr.size(), -1);
    for(int i=0; i<objects_bgr.size(); i++){
        ObjectRecognizer::recognize(objects_bgr[i], objects_masks[i], histogram_size, labels[i], confidences[i], debug);
        cv::bitwise_or(img, objects_bgr[i], img);
        std::cout << "ObjReco.->Recognized object: " << labels[i] << " with confidence " << confidences[i] << std::endl;
    }
    //cv::imshow("Points above plane", img);

    resp = Utils::get_recog_objects_response(objects_bgr, objects_xyz, objects_masks, labels, confidences, img, req.point_cloud.header.frame_id);
    pubMarkerArray.publish(Utils::get_objects_markers(resp.recog_objects));
    return resp.recog_objects.size() > 0;
}

bool callback_detect_and_recog_obj(vision_msgs::RecognizeObject::Request& req, vision_msgs::RecognizeObject::Response& resp)
{
    std::cout << "ObjReco.->Trying to recognize " << req.name << " in a Jebusly manner" << std::endl;
    cv::Mat img, cloud, output_mask;
    if(debug) cv::destroyAllWindows();
        
    Utils::transform_cloud_wrt_base(req.point_cloud, img, cloud, tf_listener);
    Utils::filter_by_distance(cloud, img, min_x, min_y, min_z, max_x, max_y, max_z, cloud, img);
    std::vector<cv::Vec3f> plane = GeometricFeatures::get_horizontal_planes(cloud, normal_min_z, plane_dist_threshold, plane_min_area, output_mask, debug);
    std::vector<cv::Vec3f> points = GeometricFeatures::above_horizontal_plane(cloud, plane, plane_dist_threshold+0.005, output_mask, debug);

    std::vector<cv::Mat> objects_bgr, objects_xyz, objects_masks;
    if(!ObjectRecognizer::segment_by_contours(cloud, img, output_mask, min_points_per_object, objects_bgr, objects_xyz, objects_masks, debug)) return false;

    std::string recognized_label = "";
    double confidence = -1;
    double max_confidence = -1;
    int recog_obj_idx = -1;
    for(int i=0; i<objects_bgr.size(); i++){
        ObjectRecognizer::recognize(objects_bgr[i], objects_masks[i], histogram_size, recognized_label, confidence, debug);
        if(recognized_label == req.name && confidence > max_confidence)
        {
            recog_obj_idx = i;
            max_confidence = confidence;
        }
    }
    if(recog_obj_idx < 0) return false;

    resp = Utils::get_recog_object_response(objects_bgr[recog_obj_idx], objects_xyz[recog_obj_idx], objects_masks[recog_obj_idx],
                                            req.name, max_confidence, req.point_cloud.header.frame_id);
    pubMarkerArray.publish(Utils::get_object_marker(resp.recog_object));
    return recog_obj_idx >= 0;
}

bool callback_recog_obj(vision_msgs::RecognizeObjects::Request& req, vision_msgs::RecognizeObjects::Response& resp)
{
    std::cout << "ObjReco.->Recognizing objects by Jebug's method (modified by Iby(✿◠‿◠))." << std::endl;
    cv::Mat img, cv_image, cloud, output_mask;

    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(req.image , sensor_msgs::image_encodings::BGR8);
    cv::imshow("image objecttt", cv_ptr->image);
    cv::waitKey(0);

    cv_bridge::CvImagePtr cv_ptr2;
    cv_ptr2 = cv_bridge::toCvCopy(req.mask , sensor_msgs::image_encodings::MONO8);
    cv::imshow("image mask", cv_ptr2->image);
    cv::waitKey(0);
   

    img = cv::Mat::zeros(img.rows, img.cols, CV_8UC3);
    double confidence;
    vision_msgs::VisionObject obj;
    ObjectRecognizer::recognize(cv_ptr->image , cv_ptr2->image, histogram_size, obj.id, confidence, debug);

    resp.recog_objects.push_back(obj);
    //pubMarkerArray.publish(Utils::get_objects_markers(resp.recog_objects));
    return resp.recog_objects.size() > 0;
}


bool callback_detect_and_train_object(vision_msgs::TrainObject::Request& req, vision_msgs::TrainObject::Response& resp)
{
    std::cout << "ObjReco.->Training object " << req.name << " Jebusly..." << std::endl;
    if(debug) cv::destroyAllWindows();
    if( req.name == "" )
    {
     	std::cout << "ObjReco.->ERROR!: objects must have a name to be trained" << std::endl;
        return false;
    }
    cv::Mat img, cloud, output_mask;   
    Utils::transform_cloud_wrt_base(req.point_cloud, img, cloud, tf_listener);
    Utils::filter_by_distance(cloud, img, min_x, min_y, min_z, max_x, max_y, max_z, cloud, img);
    std::vector<cv::Vec3f> plane = GeometricFeatures::get_horizontal_planes(cloud, normal_min_z, plane_dist_threshold, plane_min_area, output_mask, debug);
    std::vector<cv::Vec3f> points = GeometricFeatures::above_horizontal_plane(cloud, plane, plane_dist_threshold+0.005, output_mask, debug);
    if(points.size() < 1)
    {
        std::cout << "ObjReco.->Cannot find points above a plane" << std::endl;
        return false;
    }
    std::vector<cv::Mat> objects_bgr, objects_xyz, objects_masks;
    bool success = ObjectRecognizer::segment_by_contours(cloud, img, output_mask, min_points_per_object, objects_bgr, objects_xyz, objects_masks, debug);
    if(!success)
    {
        std::cout << "ObjReco.->Cannot detect any object above a plane." << std::endl;
        return false;
    }
    if(objects_bgr.size() != 1)
    {
        std::cout << "ObjReco.->ERROR! Only one object can be placed above a plane to be stored. " << std::endl;
        return false;
    }
    if(!ObjectRecognizer::store_object_example(objects_bgr[0], objects_masks[0], req.name, training_dir, debug))
        return false;
    return true;
}

bool callback_get_points_above_plane(vision_msgs::PreprocessPointCloud::Request& req, vision_msgs::PreprocessPointCloud::Response& resp)
{
    if(debug) cv::destroyAllWindows();
    std::cout << "ObjReco.->Trying to find the points above a plane..." << std::endl;
    cv::Mat img, xyz, output_mask;   
    Utils::transform_cloud_wrt_base(req.input_cloud, img, xyz, tf_listener);
    Utils::filter_by_distance(xyz, img, min_x, min_y, min_z, max_x, max_y, max_z, xyz, img);
    std::vector<cv::Vec3f> plane = GeometricFeatures::get_horizontal_planes(xyz, normal_min_z, plane_dist_threshold, plane_min_area, output_mask, debug);
    std::vector<cv::Vec3f> points = GeometricFeatures::above_horizontal_plane(xyz, plane, plane_dist_threshold+0.005, output_mask, debug);
    if(points.size() < 10)
    {
        std::cout << "ObjReco.->Cannot find enough points above a plane. " << std::endl;
        return false;
    }
    img = img.setTo(0, output_mask==0);
    xyz = xyz.setTo(0, output_mask==0);
    Utils::cv_mat2_pointcloud_msg(img, xyz, req.input_cloud.header.frame_id, resp.output_cloud);
    pubResultCloud.publish(resp.output_cloud);
    return true;
}

int main(int argc, char** argv)
{
    std::cout << "INITIALIZING OBJECT RECOGNIZER BY MR. YISUS (CORRECTED AND IMPROVED BY MARCOSOFT)" << std::endl;
    ros::init(argc, argv, "obj_reco_node");
    ros::NodeHandle n;
    ros::ServiceServer srvFindLines       = n.advertiseService("/vision/line_finder/find_table_edge", callback_find_table_edge);
    ros::ServiceServer srvFindPlanes      = n.advertiseService("/vision/line_finder/find_horizontal_plane_ransac", callback_find_planes);
    ros::ServiceServer srvDetectRecogObjs = n.advertiseService("/vision/obj_reco/detect_and_recognize_objects", callback_detect_and_recog_objs);
    ros::ServiceServer srvDetectRecogObj  = n.advertiseService("/vision/obj_reco/detect_and_recognize_object" , callback_detect_and_recog_obj );
    ros::ServiceServer srvRecogObj        = n.advertiseService("/vision/obj_reco/recognize_object" , callback_recog_obj );
    ros::ServiceServer srvDetectTrainObj  = n.advertiseService("/vision/obj_reco/detect_and_train_object", callback_detect_and_train_object);
    ros::ServiceServer srvProcessCloud    = n.advertiseService("/vision/get_points_above_plane", callback_get_points_above_plane);
    pubMarkers     = n.advertise<visualization_msgs::Marker>("/vision/obj_reco/markers", 1);
    pubMarkerArray = n.advertise<visualization_msgs::MarkerArray>("/vision/obj_reco/marker_array", 1);
    pubResultCloud = n.advertise<sensor_msgs::PointCloud2>("/vision/obj_reco/resulting_cloud", 1);
    
    ros::Rate loop(30);
    tf_listener = new tf::TransformListener();

    training_dir = ros::package::getPath("obj_reco") + std::string("/training_dir");
    if(ros::param::has("~debug"))
        ros::param::get("~debug", debug);
    if(ros::param::has("~training_dir"))
        ros::param::get("~training_dir", training_dir);
    if(ros::param::has("~min_x"))
        ros::param::get("~min_x", min_x);
    if(ros::param::has("~max_x"))
        ros::param::get("~max_x", max_x);
    if(ros::param::has("~min_y"))
        ros::param::get("~min_y", min_y);
    if(ros::param::has("~max_y"))
        ros::param::get("~max_y", max_y);
    if(ros::param::has("~min_z"))
        ros::param::get("~min_z", min_z);
    if(ros::param::has("~max_z"))
        ros::param::get("~max_z", max_z);
    if(ros::param::has("~normal_min_z"))
        ros::param::get("~normal_min_z", normal_min_z);
    if(ros::param::has("~canny_threshold1"))
        ros::param::get("~canny_threshold1", canny_threshold1);
    if(ros::param::has("~canny_threshold2"))
        ros::param::get("~canny_threshold2", canny_threshold2);
    if(ros::param::has("~canny_window_size"))
        ros::param::get("~canny_window_size",canny_window_size);
    if(ros::param::has("~hough_threshold"))
        ros::param::get("~hough_threshold", hough_threshold);
    if(ros::param::has("~hough_min_rho"))
        ros::param::get("~hough_min_rho", hough_min_rho);
    if(ros::param::has("~hough_max_rho"))
        ros::param::get("~hough_max_rho", hough_max_rho);
    if(ros::param::has("~hough_step_rho"))
        ros::param::get("~hough_step_rho", hough_step_rho);
    if(ros::param::has("~hough_min_theta"))
        ros::param::get("~hough_min_theta", hough_min_theta);
    if(ros::param::has("~hough_max_theta"))
        ros::param::get("~hough_max_theta", hough_max_theta);
    if(ros::param::has("~hough_step_theta"))
        ros::param::get("~hough_step_theta", hough_step_theta);
    if(ros::param::has("~plane_dist_threshold"))
        ros::param::get("~plane_dist_threshold", plane_dist_threshold);
    if(ros::param::has("~plane_min_area"))
        ros::param::get("~plane_min_area", plane_min_area);
    if(ros::param::has("~histogram_size"))
        ros::param::get("~histogram_size", histogram_size);
    if(ros::param::has("~min_points_per_object"))
        ros::param::get("~min_points_per_object", min_points_per_object);

    Utils::debug = debug;
    ObjectRecognizer::train_from_folder(training_dir, histogram_size, debug);
    while(ros::ok() && cv::waitKey(10) != 27)
    {
        ros::spinOnce();
        loop.sleep();
    }
}
