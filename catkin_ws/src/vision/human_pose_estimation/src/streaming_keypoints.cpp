#include <math.h>
#include <vector>
#include <string>
#include <iostream>
#include <sstream>

#include <ros/ros.h>
#include <ros/package.h>
#include <ros/topic.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp> 

#include "pose.hpp"
#include "poseEstimation.hpp"

#include <geometry_msgs/Pose.h>
#include "vision_msgs/Keypoint.h"
#include "vision_msgs/HumanCoordinates.h"
#include "vision_msgs/HumanCoordinatesArray.h"

std::string pcl_topic = "/points";

const std::vector<std::string> kpt_names{"nose", "neck", 
	"r_sho", "r_elb", "r_wri", "l_sho", "l_elb", "l_wri",
        "r_hip", "r_knee", "r_ank", "l_hip", "l_knee", "l_ank",
	"r_eye", "l_eye","r_ear", "l_ear"};

cv::Mat image;
sensor_msgs::Image pcl_image;
poseEstimation::poseEstimation *pe;
poseEstimation::poseTracker *ptrack;

class HumanDetector{
  private:
    ros::NodeHandle nh;
    ros::Publisher pub;
    ros::Publisher pubMarker;
    ros::Subscriber subCloud;
    ros::Subscriber subEnable;
  public:
    HumanDetector();
    void pclCallback(const sensor_msgs::PointCloud2ConstPtr& pcl_topic);
    void enableCallback(const std_msgs::Bool::ConstPtr& msg);
    visualization_msgs::Marker getHumanPoseMarkers(vision_msgs::HumanCoordinatesArray& msg);
};

HumanDetector::HumanDetector(){

  pub = nh.advertise<vision_msgs::HumanCoordinatesArray>("/vision/human_pose/human_pose_array", 1);
  pubMarker = nh.advertise<visualization_msgs::Marker>("/vision/human_pose/human_pose_marker", 1);
  subEnable = nh.subscribe("/vision/human_pose/enable", 1, &HumanDetector::enableCallback, this);
}

void HumanDetector::pclCallback(const sensor_msgs::PointCloud2ConstPtr& pcl_topic){
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*pcl_topic, pcl_pc2);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2xyz(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromPCLPointCloud2(pcl_pc2, *cloud2xyz);
  pcl::PointXYZRGB p;
  pcl::toROSMsg (*pcl_topic, pcl_image);

  try{
    image = cv_bridge::toCvCopy(pcl_image, sensor_msgs::image_encodings::BGR8)->image;
  }
  catch (cv_bridge::Exception& e){
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }
	
  std::vector<poseEstimation::Pose> poses = pe->run(image);
  ptrack->track(poses);

  vision_msgs::HumanCoordinatesArray hca;
  hca.header.frame_id = pcl_topic->header.frame_id;
  hca.header.stamp = ros::Time::now();
  hca.number_of_people = poses.size(); //how many people


  for (int people = 0; people < poses.size(); people++){
    poses[people].draw(image, true);

    vision_msgs::HumanCoordinates hc;
    hc.person_id = people;

    for (int i = 0; i < poses[people].keypoints.size(); i++){

      //how to access
      //std::cout << kpt_names[i] << std::endl;
      //std::cout << poses[i].confidence << std::endl; //TODO
      //std::cout << poses[i].track_id <<std::endl; //orientaion?
      //std::cout << poses[i].bbox.x << std::endl;
      //std::cout << poses[i].keypoints << std::endl;
      //poses.size() //how many people
	  
  
      //### calcurate central position of hip 
      //8 -> r_hip 11-> l_hip  18 -> c_hip
      
//      int centroid_x = (poses[people].keypoints[8].x + poses[people].keypoints[11].x) / 2;
//      int centroid_y = (poses[people].keypoints[8].y + poses[people].keypoints[11].y) / 2;
//     
//      int pix_x = poses[people].keypoints[i].x;
//      int pix_y = poses[people].keypoints[i].y;
//
//      if (centroid_x != -1 || centroid_y != -1){
//        poses[people].keypoints[18].x = centroid_x;
//        poses[people].keypoints[18].y = centroid_y;
//      }
//
//
      int pix_x = poses[people].keypoints[i].x;
      int pix_y = poses[people].keypoints[i].y;

      if (pix_x < cloud2xyz->width && pix_y < cloud2xyz->height){
        if (!std::isnan(cloud2xyz->at(pix_x, pix_y).z)){
          p = cloud2xyz->at(pix_x, pix_y);
	  
	  vision_msgs::Keypoint kp;
	  kp.keypoint_name = kpt_names[i];

	  kp.keypoint_coordinates.position.x = p.x;
          kp.keypoint_coordinates.position.y = p.y;
          kp.keypoint_coordinates.position.z = p.z; 
	  kp.confidence = poses[people].confidence;
	  hc.keypoints_array.push_back(kp);
	}
      }
    }
    hca.coordinates_array.push_back(hc);
  }
  pub.publish(hca);
  pubMarker.publish(getHumanPoseMarkers(hca));
  cv::imshow("HumanPoseEstimator",image);
  cv::waitKey(1);
}

void HumanDetector::enableCallback(const std_msgs::Bool::ConstPtr& msg)
{
    if(msg->data)
    {
        std::cout << "HumanPoseEstimator.->Starting human pose detection"<<std::endl;
        subCloud  = nh.subscribe(pcl_topic, 1, &HumanDetector::pclCallback, this);
    }
    else
    {
        std::cout << "HumanPoseEstimator.->Stopping human pose detection..." <<std::endl;
        subCloud.shutdown();
        cv::destroyAllWindows();
    }
}

visualization_msgs::Marker HumanDetector::getHumanPoseMarkers(vision_msgs::HumanCoordinatesArray& msg)
{
    visualization_msgs::Marker mrk;
    mrk.header.frame_id = msg.header.frame_id;
    mrk.header.stamp = ros::Time::now();
    mrk.ns = "human_pose";
    mrk.id = 0;
    mrk.type = visualization_msgs::Marker::SPHERE_LIST;
    mrk.action = visualization_msgs::Marker::ADD;
    mrk.pose.orientation.w = 1.0;
    mrk.scale.x = 0.1;
    mrk.scale.y = 0.1;
    mrk.scale.z = 0.1;
    mrk.color.a = 1.0; // Don't forget to set the alpha!
    mrk.color.r = 1.0;
    mrk.color.g = 0.5;
    mrk.color.b = 0.0;
    for(size_t i=0; i < msg.coordinates_array.size(); i++)
        for(size_t j=0; j < msg.coordinates_array[i].keypoints_array.size(); j++)
        {
            geometry_msgs::Point p;
            p.x = msg.coordinates_array[i].keypoints_array[j].keypoint_coordinates.position.x;
            p.y = msg.coordinates_array[i].keypoints_array[j].keypoint_coordinates.position.y;
            p.z = msg.coordinates_array[i].keypoints_array[j].keypoint_coordinates.position.z;
            mrk.points.push_back(p);
        }
    return mrk;
}

int main(int argc,char **argv){
    std::cout << "INITIALIZING HUMAN POSE ESTIMATOR BY RYOHEI" << std::endl;
    std::string package = "human_pose_estimation";
    std::string path = ros::package::getPath(package);
    std::stringstream ss;
    ss << path << "/models/poseEstimationModel.onnx"; 
    pe = new poseEstimation::poseEstimation(ss.str());
    ptrack = new poseEstimation::poseTracker();
    
    ros::init (argc, argv,"human_pose_detector");
    ros::param::param<std::string>("~point_cloud_topic", pcl_topic, "/points");
    HumanDetector human_detector;
    ros::spin();
    return 0;
}
