#include "ros/ros.h"
#include "std_msgs"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/Point.h"

int main(int argc , char **argv){
    ros::init(argc, argv, "local_occupancy_grid");
    ros::NodeHandle nh;
    ros::Subscriber occupancy_grid_handle = nh.subscribe("/hardaware/scan")




}
