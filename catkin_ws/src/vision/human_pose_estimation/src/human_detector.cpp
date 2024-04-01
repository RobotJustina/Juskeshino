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
ros::Publisher pub;
int number_of_people = 0;

void keyPointsCallback(const vision_msgs::HumanCoordinatesArray::ConstPtr& msg)
{
  HumanArray = *msg;
  //Humans = HumanArray.coordinates_array;
  Human = HumanArray.coordinates_array[0];
  number_of_people = HumanArray.number_of_people;
  std::cout << "number_of_people"<< std::endl;
  std::cout << number_of_people << std::endl;

  //if(Humans.size() > 0){
  if(number_of_people > 0){
	std::cout << "Hay un humano"<< std::endl;
	human_bool.data = true;
  pub.publish(human_bool);
  }
  else{
  	std::cout << "No hay un humano" << std::endl;
  	human_bool.data = false;
    pub.publish(human_bool);
  }
  //std::cout << Human.coordinates_array << std::endl;
  //std::cout << HumanArray << std::endl;
  //std::cout << typeid(HumanArray).name() << std::endl;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "human_detector");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("vision/human_pose/human_pose_array", 1000, keyPointsCallback);
  pub = n.advertise<std_msgs::Bool>("/vision/human_pose_estimation/human_detector_bool", 1);
  ros::spin();

  while (ros::ok())
  {
  	//pub.publish(human_bool);
  }

  return 0;
}
