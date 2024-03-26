#include <vector>
#include <string>
#include <iostream>
#include <sstream>

#include <ros/ros.h>
#include <ros/package.h>
#include <ros/topic.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

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
#include "vision_msgs/HumanPoseEstimatorResult.h"

//Para Takeshi
//static const std::string pcl_topic = "/hsrb/head_rgbd_sensor/depth_registered/rectified_points";
//para Justina
static const std::string pcl_topic = "/camera/depth_registered/points";

const std::vector<std::string> kpt_names{"nose", "neck",
		"r_sho", "r_elb", "r_wri", "l_sho", "l_elb", "l_wri",
                "r_hip", "r_knee", "r_ank", "l_hip", "l_knee", "l_ank",
                "r_eye", "l_eye","r_ear", "l_ear"};

int frame = 0;

cv::Mat image;
poseEstimation::poseEstimation *pe;
poseEstimation::poseTracker *ptrack;
sensor_msgs::Image pcl_image;

bool srvCallback(vision_msgs::HumanPoseEstimatorResult::Request &req,vision_msgs::HumanPoseEstimatorResult::Response &res){

  ROS_INFO("waiting for PointCloud msg ");
  sensor_msgs::PointCloud2ConstPtr cloud = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(pcl_topic);
  ROS_INFO("receive PointCloud msg ");

  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*cloud,pcl_pc2);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2xyz(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromPCLPointCloud2(pcl_pc2, *cloud2xyz);
  pcl::PointXYZRGB p;
  pcl::toROSMsg(*cloud, pcl_image);
  
  try{
    image = cv_bridge::toCvCopy(pcl_image, sensor_msgs::image_encodings::BGR8)->image;
  }
  catch (cv_bridge::Exception& e){
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }
	
  std::vector<poseEstimation::Pose> poses = pe->run(image);
  ptrack->track(poses);

  vision_msgs::HumanCoordinatesArray hca;
  hca.header.frame_id = cloud->header.frame_id;
  hca.header.stamp = ros::Time::now();


  for (int people = 0; people < poses.size(); people++){
    poses[people].draw(image, true);

    vision_msgs::HumanCoordinates hc;
    hc.person_id = people;

    for (int i = 0; i < poses[people].keypoints.size(); i++){

      //how to access 
      //std::cout << kpt_names[i] << std::endl;
      //std::cout << poses[i].confidence << std::endl; //TODO
      //std::cout << poses[i].track_id << std::endl; //orien
      //std::cout << poses[i].bbox.x << std::endl;
      //std::cout << poses[i].keypoints << std::endl;
      //poses.size() //how many
      
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
  res.coordinates_array = hca;
  frame++;
  cv::imshow("HumanPoseEstimator",image);
  cv::waitKey(10); // debug -> 0 , non-debug -> comment out
  return true;
}
  

int main(int argc,char **argv){
  std::string package = "human_pose_estimation";
  std::string path = ros::package::getPath(package);

  std::stringstream ss;
  ss << path << "/models/poseEstimationModel.onnx"; 
  pe = new poseEstimation::poseEstimation(ss.str());
  ptrack = new poseEstimation::poseTracker();

  ros::init (argc, argv,"PoseEstiomator");
  ros::NodeHandle nh;
  ros::ServiceServer service = nh.advertiseService("pose_estimator_srv", srvCallback);
  ros::Rate loop(10);
  while(ros::ok() && cv::waitKey(10) != 27)
  {
      ros::spinOnce();
      loop.sleep();
  }
  return 0;
}
