#include "ros/ros.h"
#include <ros/package.h>
#include "vision_msgs/FindLines.h"
#include "vision_msgs/FindPlanes.h"
#include "vision_msgs/RecognizeObjects.h"
#include "vision_msgs/RecognizeObject.h"
#include "vision_msgs/VisionObject.h"
#include "vision_msgs/TrainObject.h"
#include "visualization_msgs/MarkerArray.h"
#include "Utils.h"
#include "GeometricFeatures.h"
#include "ObjectRecognizer.h"

tf::TransformListener* tf_listener;
ros::Publisher pubMarkers;

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
    pubMarkers.publish(Utils::get_plane_marker(plane));
    if(plane.size() > 0) std::cout << "ObjReco.->Found Plane. Center: " << plane[0] << " Normal: " << plane[1] <<std::endl;
    else std::cout << "ObjReco.->Cannot find plane. " << std::endl;
    return plane.size() > 0;
}

bool callback_recog_objs(vision_msgs::RecognizeObjects::Request& req, vision_msgs::RecognizeObjects::Response& resp)
{
    std::cout << "ObjReco.->Recognizing objects by Jebug's method." << std::endl;
    cv::Mat img, cloud, output_mask;
        
    Utils::transform_cloud_wrt_base(req.point_cloud, img, cloud, tf_listener);
    Utils::filter_by_distance(cloud, img, min_x, min_y, min_z, max_x, max_y, max_z, cloud, img);
    std::vector<cv::Vec3f> plane = GeometricFeatures::get_horizontal_planes(cloud, normal_min_z, plane_dist_threshold, plane_min_area, output_mask, debug);
    std::vector<cv::Vec3f> points = GeometricFeatures::above_horizontal_plane(cloud, plane, plane_dist_threshold+0.005, output_mask, debug);
    if(points.size() < 10) return false;
    std::vector<cv::Mat> objects_bgr, objects_xyz, objects_masks;
    if(!ObjectRecognizer::segment_by_contours(cloud, img, output_mask, min_points_per_object, objects_bgr, objects_xyz, objects_masks, debug)) return false;
    
    for(int i=0; i<objects_bgr.size(); i++){
        std::string recognized;
        ObjectRecognizer::recognize(objects_bgr[i], objects_masks[i], histogram_size, recognized);
    }
    cv::imshow("Points above plane", img);
    return false;
}

bool callback_recog_obj(vision_msgs::RecognizeObject::Request& req, vision_msgs::RecognizeObject::Response& resp)
{
    std::cout << "ObjReco.->Trying to recognize " << req.name << " by Jebug's method." << std::endl;
    cv::Mat img, cloud;
    //transform_cloud_wrt_base(req.point_cloud, img, cloud);
}

bool callback_train_object(vision_msgs::TrainObject::Request& req, vision_msgs::TrainObject::Response& resp)
{
    std::cout << "ObjReco.->Training object " << req.name << " Jebusly..." << std::endl;
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
    if(objects_bgr.size() != 1)
    {
        std::cout << "ObjReco.->ERROR! Only one object can be placed above a plane to be stored. " << std::endl;
        return false;
    }
    if(!ObjectRecognizer::store_object_example(objects_bgr[0], objects_masks[0], req.name, training_dir, debug))
        return false;
    return true;
}

int main(int argc, char** argv)
{
    std::cout << "INITIALIZING OBJECT RECOGNIZER BY MR. YISUS (CORRECTED AND IMPROVED BY MARCOSOFT)" << std::endl;
    ros::init(argc, argv, "obj_reco_node");
    ros::NodeHandle n;
    ros::ServiceServer srvFindLines  = n.advertiseService("/vision/line_finder/find_table_edge", callback_find_table_edge);
    ros::ServiceServer srvFindPlanes = n.advertiseService("/vision/line_finder/find_horizontal_plane_ransac", callback_find_planes);
    ros::ServiceServer srvRecogObjs  = n.advertiseService("/vision/obj_reco/recognize_objects", callback_recog_objs);
    ros::ServiceServer srvRecogObj   = n.advertiseService("/vision/obj_reco/recognize_object" , callback_recog_obj );
    ros::ServiceServer srvTrainObj   = n.advertiseService("/vision/obj_reco/train_object", callback_train_object);
    pubMarkers    = n.advertise<visualization_msgs::Marker>("/vision/obj_reco/markers", 1);
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

    ObjectRecognizer::train_from_folder(training_dir, histogram_size);
    while(ros::ok() && cv::waitKey(10) != 27)
    {
        ros::spinOnce();
        loop.sleep();
    }
}
