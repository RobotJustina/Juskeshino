#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/GetMap.h"
#include "nav_msgs/OccupancyGrid.h"
#include "random_numbers/random_numbers.h"
#include "occupancy_grid_utils/ray_tracer.h"
#include "tf/transform_listener.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose.h"
#include "tf/transform_broadcaster.h"

nav_msgs::OccupancyGrid map;
sensor_msgs::LaserScan simulated_scan;
geometry_msgs::Pose robot_pose;
bool map_catch=false;

void callback_occ(const nav_msgs::OccupancyGrid::ConstPtr& msg){
    map = *msg; //std::cout<<"mapa"<<std::endl;
    int arr[]={2801,2802,2881,2882,2883,2962,2963,3042,3043,3044,3123,3124,3203,3204,3282,3283,3362,3363,3441,3442,3443,3521,3522,2961};
    for (int i = 0; i < 24; i++) {
         map.data[arr[i]]=0;
    }
    //for (size_t i = 0; i < map.data.size(); ++i)
    //    if(static_cast<int>(map.data[i])==100)
    //        std::cout<<i<<std::endl;
    map_catch=true;
}

int main(int argc, char** argv)
{
    std::cout << "Simulating lasers" << std::endl;
    ros::init(argc, argv, "sim_laser");
    ros::NodeHandle n("~");
    ros::Rate loop(10);
    ros::Subscriber sub_occ      = n.subscribe("/local_occ_grid", 1, callback_occ);
    ros::Publisher  pub_laser = n.advertise<sensor_msgs::LaserScan>("/hardware/scan", 10);
    //tf::TransformListener listener;
    sensor_msgs::LaserScan real_scan;
    sensor_msgs::LaserScan real_sensor_info;

    real_scan = *ros::topic::waitForMessage<sensor_msgs::LaserScan>("/hardware/scan");
    real_sensor_info = real_scan;
    while(ros::ok())
    {
        //try{
            //tf::StampedTransform t;
            //listener.lookupTransform("odom", "base_link", ros::Time(0), t);
        robot_pose.position.x = 0.17;//t.getOrigin().x();
        robot_pose.orientation.w = 1;//t.getRotation().w();
        //}
        //catch(std::exception &e){
        //    robot_pose.orientation.w = 1;
        //}
        if(map_catch){
            simulated_scan = *occupancy_grid_utils::simulateRangeScan(map, robot_pose, real_sensor_info);
            simulated_scan.header.stamp=ros::Time(0);
            pub_laser.publish(simulated_scan);
        }
        ros::spinOnce();
        loop.sleep();
    }
    return 0;
}
