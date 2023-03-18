#include <ros/ros.h>
#include <tf/transform_listener.h>

tf::TransformListener* tf_listener;

int main(int argc, char** argv){
    ros::init(argc, argv, "get_votes");
    ros::NodeHandle n;
    ros::Rate loop(1);
    tf_listener = new tf::TransformListener();
    while(ros::ok())
        std::cout<<"hola funciona el nodo"<<std::endl;
};
