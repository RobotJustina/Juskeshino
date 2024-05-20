#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_srvs/Trigger.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud2.h"
#include "tf/transform_listener.h"
#include "tf_conversions/tf_eigen.h"

#define RATE 30

bool  enable = false;
bool  use_lidar  = true;
bool  use_cloud  = false;
float minX =  0.3;
float maxX =  0.7;
float minY = -0.3;
float maxY =  0.3;
float minZ =  0.1;
float maxZ =  1.5;
int   cloud_downsampling = 9;
int   cloud_threshold    = 100;
int   lidar_downsampling = 1;
int   lidar_threshold    = 15;
int   no_data_cloud_counter  = 0;
int   no_data_lidar_counter  = 0;
std::string point_cloud_topic   = "/point_cloud";
std::string laser_scan_topic    = "/scan";

ros::NodeHandle* nh;
ros::Subscriber sub_cloud;
ros::Subscriber sub_lidar;
tf::TransformListener* tf_listener;

void callback_lidar(sensor_msgs::LaserScan::Ptr msg)
{
    no_data_lidar_counter = 0;    
}

void callback_point_cloud(sensor_msgs::PointCloud2::Ptr msg)
{
    no_data_cloud_counter = 0;
    //collision_risk_cloud = check_collision_risk_with_cloud(msg, rejection_force_cloud.x, rejection_force_cloud.y);
}

void callback_enable(const std_msgs::Bool::ConstPtr& msg)
{
    if(msg->data)
    {
        std::cout<<"LocalGridMap.->Starting local grid map builder using: "<<(use_lidar?"lidar ":"")<<(use_cloud?"point_cloud":"")<<std::endl;
        if(use_cloud ) sub_cloud = nh->subscribe(point_cloud_topic, 1, callback_point_cloud );
        if(use_lidar ) sub_lidar = nh->subscribe(laser_scan_topic , 1, callback_lidar);
    }
    else
    {=cloud_downsampling, p += cloud_downsampling*msg->point_step)
    {

        std::cout << "LocalGridMap.->Stopping obstacle detection..." <<std::endl;
        if(use_cloud)   sub_cloud.shutdown();
        if(use_lidar)   sub_lidar.shutdown();
    }
    enable = msg->data;
}

int main(int argc, char** argv)
{
    std::cout << "INITIALIZING LOCAL GRID MAP BUILDER BY MARCOSOFT... " << std::endl;
    ros::init(argc, argv, "local_grid_map");
    ros::NodeHandle n;
    ros::Rate loop(RATE);
    tf_listener = new tf::TransformListener();
    nh = &n;

    float no_sensor_data_timeout = 0.5;
    ros::param::param<bool >("~use_lidar", use_lidar, true);
    ros::param::param<bool >("~use_point_cloud", use_cloud, false);
    ros::param::param<float>("~min_x", minX,  0.30);
    ros::param::param<float>("~max_x", maxX,  0.80);
    ros::param::param<float>("~min_y", minY, -0.30);
    ros::param::param<float>("~max_y", maxY,  0.30);
    ros::param::param<float>("~min_z", minZ,  0.05);
    ros::param::param<float>("~max_z", maxZ,  1.50);
    ros::param::param<float>("~no_sensor_data_timeout" , no_sensor_data_timeout, 0.5);
    ros::param::param<int  >("~cloud_points_threshold" , cloud_threshold, 100);
    ros::param::param<int  >("~cloud_downsampling"     , cloud_downsampling, 9);
    ros::param::param<int  >("~lidar_points_threshold" , lidar_threshold, 10);
    ros::param::param<int  >("~lidar_downsampling"     , lidar_downsampling, 1);
    ros::param::param<std::string>("~point_cloud_topic", point_cloud_topic, "/points");
    ros::param::param<std::string>("~laser_scan_topic" ,  laser_scan_topic , "/scan"  );

    std::cout << "LocalGridMap.->Starting local grid map using: "<<(use_lidar?"lidar ":"")<<(use_cloud?"point_cloud ":"")<<std::endl;
    std::cout << "LocalGridMap.->Using parameters: min_x=" << minX << "  max_x=" << maxX << "  min_y=" << minY << "  max_y=";
    std::cout << maxY << "  min_z=" << minZ << "  max_z=" << maxZ << std::endl;
    std::cout << "LocalGridMap.->Point cloud topic: "<<point_cloud_topic<<"   lidar topic name: " << laser_scan_topic << std::endl;
    std::cout << "LocalGridMap.->Params for cloud: threshold="<<cloud_threshold<<"  downsampling="<<cloud_downsampling<<std::endl;
    std::cout << "LocalGridMap.->Params for lidar: threshold="<<lidar_threshold<<"  downsampling="<<lidar_downsampling<<std::endl;

    std::cout << "LocalGridMap.->Waiting for first messages from active sensors: ";
    std::cout << (use_cloud ? point_cloud_topic : "" ) << " " << (use_lidar ? laser_scan_topic : "") << std::endl;
    boost::shared_ptr<sensor_msgs::PointCloud2 const> ptr_cloud_temp; 
    boost::shared_ptr<sensor_msgs::LaserScan const>   ptr_lidar_temp;
    for(int i=0; i<10 && use_cloud && ptr_cloud_temp==NULL; i++)
        ptr_cloud_temp = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(point_cloud_topic,ros::Duration(1.0));
    for(int i=0; i<10 && use_lidar && ptr_lidar_temp==NULL; i++)
        ptr_lidar_temp = ros::topic::waitForMessage<sensor_msgs::LaserScan>  (laser_scan_topic, ros::Duration(1.0));
    if(use_cloud && ptr_cloud_temp == NULL)
    {
        std::cout << "LocalGridMap.->Cannot get first message for cloud from topic " << point_cloud_topic << std::endl;
        return -1;
    }
    if(use_lidar && ptr_lidar_temp == NULL)
    {
        std::cout << "LocalGridMap.->Cannot get first message for lidar from topic " << laser_scan_topic << std::endl;
        return -1;
    }
    std::cout << "LocalGridMap.->First messages received..." << std::endl;
    
    std::cout << "LocalGridMap.->Waiting for transforms to be available..." << std::endl;
    tf_listener->waitForTransform("map", "base_link", ros::Time(0), ros::Duration(10.0));
    std::cout << "LocalGridMap.->Waiting for sensor transforms" << std::endl;
    if(use_cloud) tf_listener->waitForTransform("base_link",ptr_cloud_temp->header.frame_id,ros::Time(0),ros::Duration(10.0));
    if(use_lidar) tf_listener->waitForTransform("base_link",ptr_lidar_temp->header.frame_id,ros::Time(0),ros::Duration(10.0));
    std::cout << "LocalGridMap.->Sensor transforms are now available"<< std::endl;

    ros::Subscriber subEnable   = n.subscribe("/navigation/local_grid/enable", 1, callback_enable);
        
    while(ros::ok())
    {
        if(enable)
        {
            if(use_lidar && no_data_lidar_counter++ > no_sensor_data_timeout*RATE)
                std::cout << "LocalGridMap.->WARNING!!! No lidar data received from topic: " << laser_scan_topic << std::endl;
            if(use_cloud && no_data_cloud_counter++ > no_sensor_data_timeout*RATE)
                std::cout << "LocalGridMap.->WARNING!!! No cloud data received from topic: " << point_cloud_topic << std::endl;              
        }
        ros::spinOnce();
        loop.sleep();
    }
    delete tf_listener;
}
