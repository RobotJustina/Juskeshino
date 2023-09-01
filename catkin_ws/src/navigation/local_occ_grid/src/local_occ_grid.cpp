#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/GetMap.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/LaserScan.h"
#include "tf/transform_listener.h"
#include "tf_conversions/tf_eigen.h"

float min_x =  0.0;
float max_x =  4.0;
float min_y = -2.0;
float max_y =  2.0;
float min_z =  0.05;
float max_z =  1.5;
float resolution = 0.05;
int   cloud_downsampling  = 9;
int   lidar_downsampling  = 2;

bool use_lidar  = true;
bool use_cloud  = false;

tf::TransformListener* listener;
std::string base_link_name    = "/base_link";
std::string point_cloud_frame = "/point_cloud_frame";
std::string point_cloud_topic = "/point_cloud";
std::string laser_scan_frame  = "/laser";
std::string laser_scan_topic  = "/scan";

nav_msgs::OccupancyGrid local_map; //Only obstacles map calculated with all enabled sensors, without inflation. Calculated on service request.
nav_msgs::OccupancyGrid local_inflated_map; //Only obstacles with inflation.

Eigen::Affine3d get_camera_position()
{
    tf::StampedTransform tf;
    listener->lookupTransform(base_link_name, point_cloud_frame, ros::Time(0), tf);
    Eigen::Affine3d e;
    tf::transformTFToEigen(tf, e);
    return e;
}

Eigen::Affine3d get_lidar_position()
{
    tf::StampedTransform tf;
    listener->lookupTransform(base_link_name, laser_scan_frame, ros::Time(0), tf);
    Eigen::Affine3d e;
    tf::transformTFToEigen(tf, e);
    return e;
}

nav_msgs::OccupancyGrid merge_maps(nav_msgs::OccupancyGrid& a, nav_msgs::OccupancyGrid& b)
{
    if(a.info.width != b.info.width || a.info.height != b.info.height)
    {
        std::cout << "LocalOccGrid.->WARNING!!! Cannot merge maps of different sizes!!!" <<std::endl;
        return a;
    }
    nav_msgs::OccupancyGrid c = a;
    for(size_t i=0; i< c.data.size(); i++)
        c.data[i] = (char)std::max((unsigned char)a.data[i], (unsigned char)b.data[i]);
    return c;
}

nav_msgs::OccupancyGrid inflate_map(nav_msgs::OccupancyGrid& map, float inflation)
{
    if(inflation <= 0)
        return map;
    
    nav_msgs::OccupancyGrid newMap = map;
    int n = (int)(inflation / map.info.resolution);
    
    return newMap;
}

bool local_map_with_cloud()
{
    //std::cout << "LocalOccGrid.->Trying to get point cloud from topic: " << point_cloud_topic << std::endl;
    boost::shared_ptr<sensor_msgs::PointCloud2 const> ptr = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(point_cloud_topic,ros::Duration(1.0));
    if(ptr == NULL)
    {
        std::cout << "LocalOccGrid.->ERROR!!! Cannot get point cloud from topic " << point_cloud_topic << std::endl;
        return false;
    }
    unsigned char* p = (unsigned char*)(&ptr->data[0]);
    int cell_x = 0;
    int cell_y = 0;
    int cell   = 0;

    Eigen::Affine3d cam_to_robot = get_camera_position();

    for(size_t i=0; i < ptr->width*ptr->height; i+=cloud_downsampling)
    {
        Eigen::Vector3d v(*((float*)(p)), *((float*)(p+4)), *((float*)(p+8)));
        v = cam_to_robot * v;
        if(v.x() > min_x && v.x() < max_x && v.y() > min_y && v.y() < max_y && v.z() > min_z && v.z() < max_z)
        {
            cell_x = (int)((v.x() - local_map.info.origin.position.x)/local_map.info.resolution);
            cell_y = (int)((v.y() - local_map.info.origin.position.y)/local_map.info.resolution);
            cell   = cell_y * local_map.info.width + cell_x;
            local_map.data[cell] = 100;
        }
        p += cloud_downsampling*ptr->point_step;
    }
    return true;
}

bool local_map_with_lidar()
{
    //std::cout << "LocalOccGrid.->Trying to get laser scan from topic: " << laser_scan_topic << std::endl;
    boost::shared_ptr<sensor_msgs::LaserScan const> ptr = ros::topic::waitForMessage<sensor_msgs::LaserScan>(laser_scan_topic, ros::Duration(1.0));
    if(ptr == NULL)
    {
        std::cout << "LocalOccGrid.->Cannot get laser scan from topic " << laser_scan_topic << std::endl;
        return false;
    }
    int cell_x = 0;
    int cell_y = 0;
    int cell   = 0;

    Eigen::Affine3d lidar_to_robot = get_lidar_position();

    for(size_t i=0; i < ptr->ranges.size(); i+=lidar_downsampling)
    {
        float angle = ptr->angle_min + i*ptr->angle_increment;
        Eigen::Vector3d v(ptr->ranges[i]*cos(angle), ptr->ranges[i]*sin(angle), 0);
        v = lidar_to_robot * v;
        if(v.x() > min_x && v.x() < max_x && v.y() > min_y && v.y() < max_y && v.z() > min_z && v.z() < max_z) 
        {
            cell_x = (int)((v.x() - local_map.info.origin.position.x)/local_map.info.resolution);
            cell_y = (int)((v.y() - local_map.info.origin.position.y)/local_map.info.resolution);
            cell   = cell_y * local_map.info.width + cell_x;
            local_map.data[cell] = 100;
        }
    }
    return true;
}

int main(int argc, char** argv)
{
    std::cout << "INITIALIZING LOCAL OCCUPANCY GRID BY MARCOSOFT..." << std::endl;
    ros::init(argc, argv, "local_occ_grid");
    ros::NodeHandle n;
    ros::Publisher pubLocalOccGrid = n.advertise<nav_msgs::OccupancyGrid>("/local_occ_grid", 10);
    listener = new tf::TransformListener();
    ros::Rate loop(10);

    ros::param::param<bool >("~use_lidar", use_lidar, true);
    ros::param::param<bool >("~use_cloud", use_cloud, false);
    ros::param::param<float>("~min_x", min_x,  0.0);
    ros::param::param<float>("~max_x", max_x,  4.0);
    ros::param::param<float>("~min_y", min_y, -2.0);
    ros::param::param<float>("~max_y", max_y,  2.0);
    ros::param::param<float>("~min_z", min_z,  0.05);
    ros::param::param<float>("~max_z", max_z,  1.5);
    ros::param::param<int  >("~lidar_downsampling", lidar_downsampling, 1);
    ros::param::param<int  >("~cloud_downsampling", cloud_downsampling, 9);
    ros::param::param<std::string>("/base_link_name", base_link_name, "base_link");

    std::cout << "LocalOccGrid.->Parameters: min_x =" << min_x << "  max_x =" << max_x << "  min_y =" << min_y << "  max_y =" << max_y;
    std::cout << "  min_z =" << min_z << "  max_z =" << max_z << std::endl;
    std::cout << "LocalOccGrid.->Parameters: cloud_downsampling="<< cloud_downsampling;
    std::cout << "  lidar_downsampling=" << lidar_downsampling << "  base link name: " << base_link_name << std::endl;

    //
    //Local map initialization
    //
    local_map.info.resolution = resolution;
    local_map.info.width  = (int)((max_x - min_x)/resolution);
    local_map.info.height = (int)((max_y - min_y)/resolution);
    local_map.info.origin.position.x = min_x;
    local_map.info.origin.position.y = min_y;
    local_map.data.resize(local_map.info.width*local_map.info.height);
    for(size_t i=0; i < local_map.data.size(); i++) local_map.data[i] = 0;
    
    std::cout << "LocalOccGrid.->Trying to get first messages from active sensor topics: " << (use_cloud ? point_cloud_topic : "") << "  ";
    std::cout << (use_lidar ? laser_scan_topic : "")<<std::endl;
    boost::shared_ptr<sensor_msgs::PointCloud2 const> ptr_c1  ; 
    boost::shared_ptr<sensor_msgs::LaserScan const>   ptr_scan;
    if(use_cloud)  ptr_c1   = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(point_cloud_topic , ros::Duration(10000.0));
    if(use_lidar)  ptr_scan = ros::topic::waitForMessage<sensor_msgs::LaserScan>(laser_scan_topic, ros::Duration(10000.0));
    std::cout << "LocalOccGrid.->First messages received..." << std::endl;
    if(use_cloud  && ptr_c1   != NULL) point_cloud_frame  = ptr_c1->header.frame_id;
    if(use_lidar  && ptr_scan != NULL) laser_scan_frame   = ptr_scan->header.frame_id;

    std::cout << "LocalOccGrid.->Waiting for sensor transforms" << std::endl;
    if (use_cloud ) listener->waitForTransform(base_link_name, point_cloud_frame , ros::Time(0), ros::Duration(1000.0));
    if (use_lidar)  listener->waitForTransform(base_link_name, laser_scan_frame,   ros::Time(0), ros::Duration(1000.0));
    std::cout << "LocalOccGrid.->Sensor transforms are now available"<< std::endl;
            
    int counter = 0;
    while(ros::ok())
    {
        pubLocalOccGrid.publish(local_map);
        ros::spinOnce();
        loop.sleep();
    }
    return 0;
}
