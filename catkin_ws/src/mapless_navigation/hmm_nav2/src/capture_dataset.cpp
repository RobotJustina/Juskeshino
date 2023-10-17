#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
//#include <laser_geometry/laser_geometry.h>

#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <tf/tf.h> // TODO: Reduce

// Include pcl
#include <pcl_conversions/pcl_conversions.h>
// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

std::string PUBLISH_TOPIC = "/pcl/points";
// ROS Publisher
ros::Publisher pub;

// High fidelity projection
//laser_geometry::LaserProjection projector_;
tf::TransformListener listener_;

void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    ROS_INFO("Seq: [%d]", msg->header.seq);
    ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]", msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    ROS_INFO("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    ROS_INFO("Vel-> Linear: [%f], Angular: [%f]", msg->twist.twist.linear.x, msg->twist.twist.angular.z);
}

void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    std::vector<float> nums = msg->ranges;
    std::string vector_str = "";

    for (auto elem : nums)
    {
        vector_str += " " + std::to_string(elem);
    }

    ROS_INFO_STREAM("ranges = " << vector_str << std::endl);
    ROS_INFO("LaserScan (range)=(%f,%f)", msg->range_min, msg->range_max);
    ROS_INFO("LaserScan (angle)=(%f,%f)", msg->angle_min, msg->angle_max);
    ROS_INFO_STREAM(msg->header);

    // Save

    if (!listener_.waitForTransform(
            msg->header.frame_id,
            "/base_link",
            msg->header.stamp + ros::Duration().fromSec(msg->ranges.size() * msg->time_increment),
            ros::Duration(1.0)))
    {
        return;
    }

    sensor_msgs::PointCloud cloud;
    //projector_.transformLaserScanToPointCloud("/base_link", *msg, cloud, listener_);
    
}

void rgbdCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{
    // Container for original & filtered data
    pcl::PCLPointCloud2 *cloud = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
    pcl::PCLPointCloud2 cloud_filtered;

    // Convert to PCL data type
    pcl_conversions::toPCL(*cloud_msg, *cloud);

    // Perform the actual filtering
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud(cloudPtr);
    sor.setLeafSize(0.1, 0.1, 0.1);
    sor.filter(cloud_filtered);

    // Convert to ROS data type
    sensor_msgs::PointCloud2 output;
    pcl_conversions::moveFromPCL(cloud_filtered, output);

    // Publish the data
    pub.publish(output);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "hmm_capture_data_node");
    ros::NodeHandle nh("~");

    ROS_INFO("Starting  hmm_capture_data_node");

    // Odometry
    // ros::Subscriber sub_odom = nh.subscribe("/hsrb/wheel_odom", 5, odomCallback);

    // LaserScan
    ros::Subscriber sub_laser_scan = nh.subscribe("/hsrb/base_scan", 10, laserScanCallback);

    // RGBD
    // ros::Subscriber sub_rgbd = nh.subscribe("/hsrb/head_rgbd_sensor/depth_registered/rectified_points", 2, rgbdCallback);

    // Create a ROS publisher to PUBLISH_TOPIC with a queue_size of 1
    pub = nh.advertise<sensor_msgs::PointCloud2>(PUBLISH_TOPIC, 1);

    ros::spin();
    return 0;
}
