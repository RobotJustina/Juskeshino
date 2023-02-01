#include "ros/ros.h"
#include <ros/package.h>
#include "vision_msgs/FindLines.h"
#include "vision_msgs/RecognizeObjects.h"
#include "vision_msgs/RecognizeObject.h"
#include "vision_msgs/VisionObject.h"
#include "vision_msgs/TrainObject.h"
#include "visualization_msgs/MarkerArray.h"
#include "PlaneExtractor.h"
#include "Utils.h"

tf::TransformListener* tf_listener;
visualization_msgs::Marker table_border_marker;

visualization_msgs::MarkerArray get_detected_objs_markers()
{
    
}

bool callback_find_lines(vision_msgs::FindLines::Request& req, vision_msgs::FindLines::Response& resp)
{
    std::cout << std::endl << "ObjReco.->Executing srvFindLines (Jebusian method)." << std::endl;
    resp.lines = PlaneExtractor::find_table_border(req.point_cloud, tf_listener);
    table_border_marker = Utils::get_lines_marker(resp.lines);
    if(resp.lines.size() > 0) std::cout << "ObjReco.->Found line: " << resp.lines[0] << " - " <<resp.lines[1] <<std::endl;
    else std::cout << "ObjReco.->Cannot find lines. " << std::endl;
    return resp.lines.size() > 0;
}

bool callback_recog_objs(vision_msgs::RecognizeObjects::Request& req, vision_msgs::RecognizeObjects::Response& resp)
{
    std::cout << "ObjReco.->Recognizing objects by Jebug's method." << std::endl;
    cv::Mat img, cloud;
    //transform_cloud_wrt_base(req.point_cloud, img, cloud);
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
    cv::Mat img, cloud;
    //transform_cloud_wrt_base(req.point_cloud, img, cloud);
    return true;
}

int main(int argc, char** argv)
{
    std::cout << "INITIALIZING OBJECT RECOGNIZER BY MR. YISUS (CORRECTED AND IMPROVED BY MARCOSOFT)" << std::endl;
    ros::init(argc, argv, "obj_reco_node");
    ros::NodeHandle n;
    ros::ServiceServer srvFindLines = n.advertiseService("/vision/line_finder/find_lines_ransac", callback_find_lines);
    ros::ServiceServer srvRecogObjs = n.advertiseService("/vision/obj_reco/recognize_objects", callback_recog_objs);
    ros::ServiceServer srvRecogObj  = n.advertiseService("/vision/obj_reco/recognize_object" , callback_recog_obj );
    ros::ServiceServer srvTrainObj  = n.advertiseService("/vision/obj_reco/train_object", callback_train_object);
    ros::Publisher     pubMarkers   = n.advertise<visualization_msgs::Marker>("/vision/obj_reco/markers", 1);
    ros::Rate loop(30);
    tf_listener = new tf::TransformListener();

    std::string training_dir = ros::package::getPath("obj_reco") + std::string("/training_dir");
    if(ros::param::has("~debug"))
        ros::param::get("~debug", Utils::debug);
    if(ros::param::has("~training_dir"))
        ros::param::get("~training_dir", training_dir);
    if(ros::param::has("~min_x"))
        ros::param::get("~min_x", Utils::min_x);
    if(ros::param::has("~max_x"))
        ros::param::get("~max_x", Utils::max_x);
    if(ros::param::has("~min_y"))
        ros::param::get("~min_y", Utils::min_y);
    if(ros::param::has("~max_y"))
        ros::param::get("~max_y", Utils::max_y);
    if(ros::param::has("~min_z"))
        ros::param::get("~min_z", Utils::min_z);
    if(ros::param::has("~max_z"))
        ros::param::get("~max_z", Utils::max_z);
    if(ros::param::has("~normals_tol"))
        ros::param::get("~normals_tol", PlaneExtractor::normals_tol);
    if(ros::param::has("~canny_threshold1"))
        ros::param::get("~canny_threshold1", PlaneExtractor::canny_threshold1);
    if(ros::param::has("~canny_threshold2"))
        ros::param::get("~canny_threshold2", PlaneExtractor::canny_threshold2);
    if(ros::param::has("~canny_window_size"))
        ros::param::get("~canny_window_size", PlaneExtractor::canny_window_size);
    if(ros::param::has("~hough_threshold"))
        ros::param::get("~hough_threshold", PlaneExtractor::hough_threshold);
    if(ros::param::has("~hough_min_rho"))
        ros::param::get("~hough_min_rho", PlaneExtractor::hough_min_rho);
    if(ros::param::has("~hough_max_rho"))
        ros::param::get("~hough_max_rho", PlaneExtractor::hough_max_rho);
    if(ros::param::has("~hough_step_rho"))
        ros::param::get("~hough_step_rho", PlaneExtractor::hough_step_rho);
    if(ros::param::has("~hough_min_theta"))
        ros::param::get("~hough_min_theta", PlaneExtractor::hough_min_theta);
    if(ros::param::has("~hough_max_theta"))
        ros::param::get("~hough_max_theta", PlaneExtractor::hough_max_theta);
    if(ros::param::has("~hough_step_theta"))
        ros::param::get("~hough_step_theta", PlaneExtractor::hough_step_theta);

    while(ros::ok() && cv::waitKey(10) != 27)
    {
        pubMarkers.publish(table_border_marker);
        ros::spinOnce();
        loop.sleep();
    }
}
