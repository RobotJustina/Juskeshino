#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "nav_msgs/OccupancyGrid.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/LaserScan.h"
#include "tf/transform_listener.h"
#include "tf_conversions/tf_eigen.h"

boost::shared_ptr<sensor_msgs::LaserScan const> scan_ptr;
boost::shared_ptr<sensor_msgs::PointCloud2 const> cloud_ptr;

void callback_laser_scan(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    scan_ptr = msg;
}

void callback_point_cloud(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    cloud_ptr = msg;
}

Eigen::Affine3d get_sensor_tf(std::string frame_id, std::string base_link_name, tf::TransformListener* listener)
{
    tf::StampedTransform tf;
    listener->lookupTransform(base_link_name, frame_id, ros::Time(0), tf);
    Eigen::Affine3d e;
    tf::transformTFToEigen(tf, e);
    return e;
}

bool fill_map_with_cloud(nav_msgs::OccupancyGrid& map, float min_x, float min_y, float min_z, float max_x, float max_y, float max_z,
                               int downsampling, std::string base_link_name, tf::TransformListener* listener)
{
    if(cloud_ptr == NULL) return false;
    unsigned char* p = (unsigned char*)(&cloud_ptr->data[0]);
    int cell_x = 0;
    int cell_y = 0;
    int cell   = 0;

    Eigen::Affine3d cam_to_robot = get_sensor_tf(cloud_ptr->header.frame_id, base_link_name, listener);

    for(size_t i=0; i < cloud_ptr->width*cloud_ptr->height; i+=downsampling)
    {
        Eigen::Vector3d v(*((float*)(p)), *((float*)(p+4)), *((float*)(p+8)));
        v = cam_to_robot * v;
        if(v.x() > min_x && v.x() < max_x && v.y() > min_y && v.y() < max_y && v.z() > min_z && v.z() < max_z)
        {
            cell_x = (int)((v.x() - map.info.origin.position.x)/map.info.resolution);
            cell_y = (int)((v.y() - map.info.origin.position.y)/map.info.resolution);
            cell   = cell_y * map.info.width + cell_x;
            map.data[cell] = 100;
        }
        p += downsampling*cloud_ptr->point_step;
    }
    return true;
}

bool fill_map_with_lidar(nav_msgs::OccupancyGrid& map, float min_x, float min_y, float min_z, float max_x, float max_y, float max_z,
                               int downsampling, std::string base_link_name, tf::TransformListener* listener)
{
    if(scan_ptr == NULL) return false;
    int cell_x = 0;
    int cell_y = 0;
    int cell   = 0;

    Eigen::Affine3d lidar_to_robot = get_sensor_tf(scan_ptr->header.frame_id, base_link_name, listener);

    for(size_t i=0; i < scan_ptr->ranges.size(); i+=downsampling)
    {
        float angle = scan_ptr->angle_min + i*scan_ptr->angle_increment;
        Eigen::Vector3d v(scan_ptr->ranges[i]*cos(angle), scan_ptr->ranges[i]*sin(angle), 0);
        v = lidar_to_robot * v;
        if(v.x() > min_x && v.x() < max_x && v.y() > min_y && v.y() < max_y && v.z() > min_z && v.z() < max_z) 
        {
            cell_x = (int)((v.x() - map.info.origin.position.x)/map.info.resolution);
            cell_y = (int)((v.y() - map.info.origin.position.y)/map.info.resolution);
            cell   = cell_y * map.info.width + cell_x;
            map.data[cell] = 100;
        }
    }
    return true;
}

int main(int argc, char** argv)
{
    std::cout << "INITIALIZING LOCAL OCCUPANCY GRID BY MARCOSOFT..." << std::endl;
    ros::init(argc, argv, "local_occ_grid_secondary");
    ros::NodeHandle n;
    tf::TransformListener* listener = new tf::TransformListener();
    
    bool use_lidar;
    bool use_cloud;
    float min_x;
    float max_x;
    float min_y;
    float max_y;
    float min_z;
    float max_z;
    float resolution;
    int   cloud_downsampling;
    int   lidar_downsampling;
    std::string scan_topic;
    std::string cloud_topic;
    std::string base_link_name;

    ros::param::param<bool >("~use_lidar", use_lidar, true);
    ros::param::param<bool >("~use_cloud", use_cloud, false);
    ros::param::param<float>("~min_x", min_x,  -10.0);
    ros::param::param<float>("~max_x", max_x,  10.0);
    ros::param::param<float>("~min_y", min_y, -10.0);
    ros::param::param<float>("~max_y", max_y,  10.0);
    ros::param::param<float>("~min_z", min_z,  0.05);
    ros::param::param<float>("~max_z", max_z,  1.5);
    ros::param::param<float>("~resolution", resolution,  0.05);
    ros::param::param<int  >("~lidar_downsampling", lidar_downsampling, 1);
    ros::param::param<int  >("~cloud_downsampling", cloud_downsampling, 9);
    ros::param::param<std::string>("~laser_scan_topic", scan_topic, "/scan");
    ros::param::param<std::string>("~point_cloud_topic", cloud_topic, "/point_cloud");
    ros::param::param<std::string>("/base_link_name", base_link_name, "base_link");

    std::cout << "LocalOccGrid.->Parameters: min_x =" << min_x << "  max_x =" << max_x << "  min_y =" << min_y << "  max_y =" << max_y;
    std::cout << "  min_z =" << min_z << "  max_z =" << max_z << std::endl;
    std::cout << "LocalOccGrid.->Parameters: cloud_downsampling="<< cloud_downsampling;
    std::cout << "  lidar_downsampling=" << lidar_downsampling << "  base link name: " << base_link_name << std::endl;

    //
    //Local map initialization
    //
    nav_msgs::OccupancyGrid local_map;
    local_map.header.frame_id = base_link_name;
    local_map.info.resolution = resolution;
    local_map.info.width  = (int)((max_x - min_x)/resolution);
    local_map.info.height = (int)((max_y - min_y)/resolution);
    local_map.info.origin.position.x = min_x;
    local_map.info.origin.position.y = min_y;
    local_map.info.origin.orientation.w = 1;
    local_map.data.resize(local_map.info.width*local_map.info.height);
    for(size_t i=0; i < local_map.data.size(); i++) local_map.data[i] = 0;
    std_msgs::Float32MultiArray local_map_array;
    local_map_array.data.resize(local_map.info.width*local_map.info.height);
    for(size_t i=0; i < local_map_array.data.size(); i++) local_map_array.data[i] = 0.0;

    //
    //Get first sensor messages and transformations
    //
    std::cout << "LocalOccGrid.->Trying to get first messages from active sensor topics: " << (use_cloud ? cloud_topic : "") << "  ";
    std::cout << (use_lidar ? scan_topic : "")<<std::endl;
    if(use_cloud)  cloud_ptr = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(cloud_topic , ros::Duration(10000.0));
    if(use_lidar)  scan_ptr  = ros::topic::waitForMessage<sensor_msgs::LaserScan>(scan_topic, ros::Duration(10000.0));
    std::cout << "LocalOccGrid.->First messages received. ";
    if(use_cloud  && cloud_ptr != NULL) std::cout << "Cloud frame id: " << cloud_ptr->header.frame_id << "    ";
    if(use_lidar  && scan_ptr  != NULL) std::cout << "Scan frame id: " << scan_ptr->header.frame_id << std::endl;
    std::cout << "LocalOccGrid.->Waiting for transforms for active sensors..." << std::endl;
    if(use_cloud  && cloud_ptr != NULL) listener->waitForTransform(base_link_name, cloud_ptr->header.frame_id, ros::Time(0), ros::Duration(10.0));
    if(use_lidar  && scan_ptr  != NULL) listener->waitForTransform(base_link_name, scan_ptr->header.frame_id, ros::Time(0), ros::Duration(10.0));
    std::cout << "LocalOccGrid.->Sensor transforms are now available"<< std::endl;

    ros::Subscriber sub_lidar;
    ros::Subscriber sub_cloud;
    if(use_lidar) sub_lidar = n.subscribe(scan_topic, 1, callback_laser_scan);
    if(use_cloud) sub_cloud = n.subscribe(cloud_topic, 1 , callback_point_cloud);

    ros::Publisher pubLocalOccGrid = n.advertise<nav_msgs::OccupancyGrid>("/local_occ_grid_2", 10);
    ros::Publisher pubLocalOccGridArray = n.advertise<std_msgs::Float32MultiArray>("/local_occ_grid_array_2", 10);
    ros::Rate loop(10);
    while(ros::ok())
    {
        for(size_t i=0; i < local_map.data.size(); i++) local_map.data[i] = 0;
        if(use_lidar) fill_map_with_lidar(local_map, min_x, min_y, min_z, max_x, max_y, max_z, lidar_downsampling, base_link_name, listener);
        if(use_cloud) fill_map_with_cloud(local_map, min_x, min_y, min_z, max_x, max_y, max_z, cloud_downsampling, base_link_name, listener);
        for(size_t i=0; i < local_map_array.data.size(); i++) local_map_array.data[i] = local_map.data[i];
        pubLocalOccGrid.publish(local_map);
        pubLocalOccGridArray.publish(local_map_array);
        ros::spinOnce();
        loop.sleep();
    }
    return 0;
}
