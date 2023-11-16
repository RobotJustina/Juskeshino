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
    map_catch=true;
}

int main(int argc, char** argv)
{
    std::cout << "Simulating lasers" << std::endl;
    ros::init(argc, argv, "sim_laser");
    ros::NodeHandle n("~");
    ros::Rate loop(10);
    ros::Subscriber sub_occ      = n.subscribe("/local_occ_grid", 1, callback_occ);
    ros::Publisher  pub_laser = n.advertise<sensor_msgs::LaserScan>("/laser_mod", 10);
    tf::TransformListener listener;
    //tf::TransformBroadcaster broadcaster;
    //tf::StampedTransform t;
    sensor_msgs::LaserScan real_scan;
    sensor_msgs::LaserScan real_sensor_info;

    real_scan = *ros::topic::waitForMessage<sensor_msgs::LaserScan>("/hardware/scan");
    real_sensor_info = real_scan;
    while(ros::ok())
    {
        try{
            tf::StampedTransform t;
            listener.lookupTransform("odom", "base_link", ros::Time(0), t);
            robot_pose.position.x = 0.1;//t.getOrigin().x();
            robot_pose.position.y = 0;//t.getOrigin().y();
            robot_pose.orientation.x = 0;//t.getRotation().x();
            robot_pose.orientation.y = 0;//t.getRotation().y();
            robot_pose.orientation.z = 0;//t.getRotation().z();
            robot_pose.orientation.w = 1;//t.getRotation().w();
        }
        catch(std::exception &e){
            robot_pose.position.x = 0;
            robot_pose.position.y = 0;
            robot_pose.orientation.w = 1;
        }
        if(map_catch){
            //simulated_scan.range_max=12;
            simulated_scan = *occupancy_grid_utils::simulateRangeScan(map, robot_pose, real_sensor_info);
            simulated_scan.header.stamp=ros::Time(0);
            //for(int i=0; i<150; i++)
            //    simulated_scan.ranges[i]/=14;
            pub_laser.publish(simulated_scan);
        }
        ros::spinOnce();
        loop.sleep();
    }
    return 0;
}
