#include <ros/ros.h>

int main(int argc, char** argv){

    ros::init(argc, argv, "lolo");   //Inicia el nodo llamado lolo
    ros::NodeHandle nh;

    ROS_INFO("hola mundo");
    

    return 0;
}