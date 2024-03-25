#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include <geometry_msgs/Pose.h>
#include "vision_msgs/Keypoint.h"
#include "vision_msgs/HumanCoordinates.h"
#include "vision_msgs/HumanCoordinatesArray.h"

vision_msgs::HumanCoordinatesArray HumanArray;
std::vector<vision_msgs::HumanCoordinates> Humans;
vision_msgs::HumanCoordinates Human; 
std_msgs::Bool human_bool;

void keyPointsCallback(const vision_msgs::HumanCoordinatesArray::ConstPtr& msg)
{
  HumanArray = *msg;
  //Humans = HumanArray.coordinates_array;
  Human = HumanArray.coordinates_array[0];

  if(Humans.size() > 0){
	std::cout << "Hay un humano"<< std::endl;
	human_bool.data = true;
  }
  else{
  	std::cout << "No hay un humano" << std::endl;
  	human_bool.data = false;
  }
  //std::cout << Human.coordinates_array << std::endl;
  //std::cout << HumanArray << std::endl;
  //std::cout << typeid(HumanArray).name() << std::endl;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "human_detector");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("human_coordinates_array", 1000, keyPointsCallback);
  ros::Publisher  pub = n.advertise<std_msgs::Bool>("human_detector_bool", 1);
  ros::spin();

  while (ros::ok())
  {
  	pub.publish(human_bool);
  }

  return 0;
}
