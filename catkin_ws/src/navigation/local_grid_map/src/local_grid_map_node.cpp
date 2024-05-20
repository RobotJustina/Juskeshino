#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_srvs/Trigger.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud2.h"
#include "nav_msgs/OccupancyGrid.h"
#include "tf/transform_listener.h"
#include "tf_conversions/tf_eigen.h"

#define RATE 10

bool  enable = false;
bool  use_lidar;
bool  use_cloud;
int   no_data_cloud_counter  = 0;
int   no_data_lidar_counter  = 0;
sensor_msgs::LaserScan msg_lidar;
sensor_msgs::PointCloud2 msg_cloud;

ros::NodeHandle* nh;
ros::Subscriber sub_cloud;
ros::Subscriber sub_lidar;
std::string point_cloud_topic;
std::string laser_scan_topic;

void callback_lidar(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    no_data_lidar_counter = 0;
    msg_lidar = *msg;
}

void callback_point_cloud(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    no_data_cloud_counter = 0;
    msg_cloud = *msg;
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
    {

        std::cout << "LocalGridMap.->Stopping obstacle detection..." <<std::endl;
        if(use_cloud)  sub_cloud.shutdown();
        if(use_lidar)  sub_lidar.shutdown();
    }
    enable = msg->data;
}

Eigen::Affine3d get_transform_to_basefootprint(std::string link_name, tf::TransformListener& tf_listener)
{
    tf::StampedTransform tf;
    tf_listener.lookupTransform("base_link", link_name, ros::Time(0), tf);
    Eigen::Affine3d e;
    tf::transformTFToEigen(tf, e);
    return e;
}

void clear_local_grid(nav_msgs::OccupancyGrid& map)
{
    for(size_t i=0; i < map.data.size(); i++)
        map.data[i] = 0;
}

void update_local_grid_lidar(nav_msgs::OccupancyGrid& map, sensor_msgs::LaserScan& msg, tf::TransformListener& tf_listener,
                             int downsampling, float min_x, float min_y, float min_z, float max_x, float max_y, float max_z,
                             float footprint_min_x, float footprint_min_y, float footprint_max_x, float footprint_max_y)
{
    if(msg.header.frame_id == "")
        return;
    Eigen::Affine3d tf = get_transform_to_basefootprint(msg.header.frame_id, tf_listener);
    for(size_t i=0; i < msg.ranges.size(); i+=downsampling)
    {
        float angle = msg.angle_min + i*msg.angle_increment;
        Eigen::Vector3d v(msg.ranges[i]*cos(angle), msg.ranges[i]*sin(angle), 0);
        v = tf * v;
        if(v.x() > footprint_min_x && v.x() < footprint_max_x && v.y() > footprint_min_y && v.y() < footprint_max_y)
            continue;
        if(v.x() > min_x && v.x() < max_x && v.y() > min_y && v.y() < max_y && v.z() > min_z && v.z() < max_z)
        {
            int w = (int)((v.x() - map.info.origin.position.x)/map.info.resolution);
            int h = (int)((v.y() - map.info.origin.position.y)/map.info.resolution);
            map.data[h*map.info.width + w] = 100;
        }
    }
}

void update_local_grid_cloud(nav_msgs::OccupancyGrid& map, sensor_msgs::PointCloud2& msg, tf::TransformListener& tf_listener,
                             int downsampling, float min_x, float min_y, float min_z, float max_x, float max_y, float max_z,
                             float footprint_min_x, float footprint_min_y, float footprint_max_x, float footprint_max_y)
{
    if(msg.header.frame_id == "")
        return;
    unsigned char* p = (unsigned char*)(&msg.data[0]);
    Eigen::Affine3d tf = get_transform_to_basefootprint(msg.header.frame_id, tf_listener);
    for(size_t i=0; i < msg.width*msg.height; i+=downsampling, p+=downsampling*msg.point_step)
    {
        Eigen::Vector3d v(*((float*)(p)), *((float*)(p+4)), *((float*)(p+8)));
        v = tf * v;
        if(v.x() > footprint_min_x && v.x() < footprint_max_x && v.y() > footprint_min_y && v.y() < footprint_max_y)
            continue;
        if(v.x() > min_x && v.x() < max_x && v.y() > min_y && v.y() < max_y && v.z() > min_z && v.z() < max_z)
        {
            int w = (int)((v.x() - map.info.origin.position.x)/map.info.resolution);
            int h = (int)((v.y() - map.info.origin.position.y)/map.info.resolution);
            map.data[h*map.info.width + w] = 100;
        }
    }
}

nav_msgs::OccupancyGrid inflate_map(nav_msgs::OccupancyGrid& map, float inflation)
{
    /*
     * WARNING!!! It is assumed that map borders (borders with at least 'inflation' thickness)
     * are occupied or unkwnon. Map must be big enough to fulfill this assumption.
     */
    if(inflation <= 0)
        return map;
    
    nav_msgs::OccupancyGrid newMap = map;
    int n = (int)(inflation / map.info.resolution);
    for(int w = n; w < map.info.width - n; w++)
        for(int h = n; h < map.info.height - n; h++)
        {
            int idx = h*map.info.width + w;
            if(map.data[idx] > 0)
                for(int i=-n; i<=n; i++)
                    for(int j=-n; j<=n; j++)
                        newMap.data[idx + j*map.info.width + i] = map.data[idx];
        }
    return newMap;
}

nav_msgs::OccupancyGrid get_cost_map(nav_msgs::OccupancyGrid& map, float cost_radius)
{
    if(cost_radius < 0)
        return map;

    nav_msgs::OccupancyGrid cost_map = map;
    int n = (int)(cost_radius / map.info.resolution);
    for(int w = n; w < map.info.width - n; w++)
        for(int h = n; h < map.info.height - n; h++)
        {
            int idx = h*map.info.width + w;
            if(map.data[idx] > 0)
                for(int i=-n; i<=n; i++)
                    for(int j=-n; j<=n; j++)
                    {
                        int cost = n - std::max(abs(i),abs(j)) + 1;
                        cost_map.data[idx + j*map.info.width + i] = std::max(cost, (int)cost_map.data[idx + j*map.info.width + i]);
                    }
        }
    return cost_map;
}

int main(int argc, char** argv)
{
    std::cout << "INITIALIZING LOCAL GRID MAP BUILDER BY MARCOSOFT... " << std::endl;
    ros::init(argc, argv, "local_grid_map");
    ros::NodeHandle n;
    nh = &n;
    tf::TransformListener tf_listener;
    ros::Rate loop(RATE);
        
    float min_x, max_x, min_y, max_y, min_z, max_z;
    float footprint_min_x, footprint_min_y, footprint_max_x, footprint_max_y;
    int   cloud_downsampling, lidar_downsampling;
    int   cloud_threshold, lidar_threshold;
    float inflation_radius, cost_radius;
    
    float no_sensor_data_timeout = 0.5;
    ros::param::param<bool >("~use_lidar", use_lidar, true);
    ros::param::param<bool >("~use_point_cloud", use_cloud, true);
    ros::param::param<float>("~min_x", min_x, -0.5);
    ros::param::param<float>("~max_x", max_x,  3.5);
    ros::param::param<float>("~min_y", min_y, -2.0);
    ros::param::param<float>("~max_y", max_y,  2.0);
    ros::param::param<float>("~min_z", min_z,  0.05);
    ros::param::param<float>("~max_z", max_z,  1.50);
    ros::param::param<float>("~footprint_min_x", footprint_min_x, -0.3);
    ros::param::param<float>("~footprint_max_x", footprint_max_x,  0.3);
    ros::param::param<float>("~footprint_min_y", footprint_min_y, -0.30);
    ros::param::param<float>("~footprint_max_y", footprint_max_y,  0.30);
    ros::param::param<float>("~no_sensor_data_timeout" , no_sensor_data_timeout, 0.5);
    ros::param::param<int  >("~cloud_points_threshold" , cloud_threshold, 100);
    ros::param::param<int  >("~cloud_downsampling"     , cloud_downsampling, 9);
    ros::param::param<int  >("~lidar_points_threshold" , lidar_threshold, 10);
    ros::param::param<int  >("~lidar_downsampling"     , lidar_downsampling, 1);
    ros::param::param<float>("~inflation_radius"       , inflation_radius, 0.25);
    ros::param::param<float>("~cost_radius"            , cost_radius, 0.5);
    ros::param::param<std::string>("~point_cloud_topic", point_cloud_topic, "/points");
    ros::param::param<std::string>("~laser_scan_topic" ,  laser_scan_topic , "/scan"  );

    std::cout << "LocalGridMap.->Starting local grid map using: "<<(use_lidar?"lidar ":"")<<(use_cloud?"point_cloud ":"")<<std::endl;
    std::cout << "LocalGridMap.->Using parameters: min_x=" << min_x << "  max_x=" << max_x << "  min_y=" << min_y << "  max_y=";
    std::cout << max_y << "  min_z=" << min_z << "  max_z=" << max_z << std::endl;
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
    tf_listener.waitForTransform("map", "base_link", ros::Time(0), ros::Duration(10.0));
    std::cout << "LocalGridMap.->Waiting for sensor transforms" << std::endl;
    if(use_cloud) tf_listener.waitForTransform("base_link",ptr_cloud_temp->header.frame_id,ros::Time(0),ros::Duration(10.0));
    if(use_lidar) tf_listener.waitForTransform("base_link",ptr_lidar_temp->header.frame_id,ros::Time(0),ros::Duration(10.0));
    std::cout << "LocalGridMap.->Sensor transforms are now available"<< std::endl;

    ros::Subscriber sub_enable     = n.subscribe("/navigation/local_grid/enable", 1, callback_enable);
    ros::Publisher  pub_local_grid = n.advertise<nav_msgs::OccupancyGrid>("/navigation/local_grid/local_grid_map", 1);

    nav_msgs::OccupancyGrid local_grid;
    local_grid.header.frame_id = "base_link";
    local_grid.info.resolution = 0.05;
    local_grid.info.origin.position.x = min_x;
    local_grid.info.origin.position.y = min_y;
    local_grid.info.origin.orientation.w = 1.0;
    local_grid.info.width  = (int)round((max_x - min_x)/local_grid.info.resolution);
    local_grid.info.height = (int)round((max_y - min_y)/local_grid.info.resolution);
    local_grid.data.resize(local_grid.info.width*local_grid.info.height);
        
    while(ros::ok())
    {
        if(enable)
        {
            clear_local_grid(local_grid);
            if(use_lidar && no_data_lidar_counter++ > no_sensor_data_timeout*RATE)
                std::cout << "LocalGridMap.->WARNING!!! No lidar data received from topic: " << laser_scan_topic << std::endl;
            else if(use_lidar)
                update_local_grid_lidar(local_grid, msg_lidar, tf_listener, lidar_downsampling, min_x, min_y, min_z,
                                        max_x, max_y, max_z, footprint_min_x, footprint_min_y, footprint_max_x, footprint_max_y);
            if(use_cloud && no_data_cloud_counter++ > no_sensor_data_timeout*RATE)
                std::cout << "LocalGridMap.->WARNING!!! No cloud data received from topic: " << point_cloud_topic << std::endl;
            else if(use_cloud)
                update_local_grid_cloud(local_grid, msg_cloud, tf_listener, cloud_downsampling, min_x, min_y, min_z,
                                        max_x, max_y, max_z, footprint_min_x, footprint_min_y, footprint_max_x, footprint_max_y);

            local_grid = inflate_map(local_grid, inflation_radius);
            local_grid = get_cost_map(local_grid, cost_radius);
            pub_local_grid.publish(local_grid);
        }
        ros::spinOnce();
        loop.sleep();
    }
}
