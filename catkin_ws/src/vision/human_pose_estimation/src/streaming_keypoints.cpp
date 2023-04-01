#include <math.h>
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

//static const std::string pcl_topic = "/hsrb/head_rgbd_sensor/depth_registered/rectified_points";
static const std::string pcl_topic = "/camera/depth_registered/points";

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
    ros::Subscriber sub; 
  public:
    HumanDetector();
    void pclCallback(const sensor_msgs::PointCloud2ConstPtr& pcl_topic);
};

HumanDetector::HumanDetector(){

  pub = nh.advertise<vision_msgs::HumanCoordinatesArray>("human_coordinates_array", 1);
  sub = nh.subscribe(pcl_topic, 1, &HumanDetector::pclCallback, this);
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
  hca.header.frame_id = "head_rgbd_sensor_rgb_frame";
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
  cv::imshow("HumanPoseEstimator",image);
  cv::waitKey(1);
}


int main(int argc,char **argv){
  printf("main");
  std::string package = "human_pose_estimation";
  std::string path = ros::package::getPath(package);

  std::stringstream ss;
  ss << path << "/models/poseEstimationModel.onnx"; 
  pe = new poseEstimation::poseEstimation(ss.str());
  ptrack = new poseEstimation::poseTracker();

  ros::init (argc, argv,"HumanDetection");
  HumanDetector human_detector;
  ros::spin();
  return 0;
}

