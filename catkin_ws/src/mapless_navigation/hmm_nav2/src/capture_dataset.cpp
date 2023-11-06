#include <ros/ros.h>
#include <ros/package.h>
#include <fstream>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>
#include <nav_msgs/Odometry.h>
#include <cv_bridge/cv_bridge.h>
// Sync messages
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
// Pcl
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>

#include "general_utils/files_utils.h" // DirectoryUtils

class DataGenerator
{
private:
    // Basics
    ros::NodeHandle nh;
    std::ofstream file_stream;
    bool set_header;
    std::string header_odom_laser;
    std::string path_file;
    bool stabilized = false;
    int time_stamp_secs = 0;
    // RGBD
    ros::Subscriber sub_rgbd;
    // RGBD--Synchronizer
    message_filters::Subscriber<sensor_msgs::Image> sub_rgb;
    message_filters::Subscriber<sensor_msgs::Image> sub_depth;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> SyncPolicyImgImg; // ExactTime Don't work on real robot
    typedef message_filters::Synchronizer<SyncPolicyImgImg> SyncImgImg;
    boost::shared_ptr<SyncImgImg> Syncr_img_img;
    // Laser_scan
    float max_laser_range;
    // Laser_scan--Synchronizer
    message_filters::Subscriber<sensor_msgs::LaserScan> sub_laser_scan;
    message_filters::Subscriber<nav_msgs::Odometry> sub_odom;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, nav_msgs::Odometry> SyncPolicyLaOd;
    typedef message_filters::Synchronizer<SyncPolicyLaOd> SyncrLaOd;
    boost::shared_ptr<SyncrLaOd> Syncr_la_od;

public:
    DataGenerator(bool save_laser_odom = false, bool save_rgbd = false, bool incremental = false)
    {
        // Prepare directory
        path_file = ros::package::getPath("hmm_nav2") + "/Dataset/";
        if(incremental && DirectoryUtils::existDir(path_file)){
            file_stream.open(path_file + "laser_odom_data.csv", std::ios::app);
            set_header = false;
        }
        else{
            bool directory_created; 
            directory_created = DirectoryUtils::replaceDir(path_file + "Images", true); // Create the directory, replace if already exist
            if (!directory_created)
            {
                ROS_ERROR_STREAM("Can't create directory: " << path_file);
                this->~DataGenerator(); // End program if path error occurs
            }
            file_stream.open(path_file + "laser_odom_data.csv", std::ios::out);
            set_header = true;
        }


        if (save_laser_odom)
        {
            ROS_INFO("Save laser_odom enabled");
            max_laser_range = 10.0;
            //Takeshi
            sub_laser_scan.subscribe(nh, "/hsrb/base_scan", 1);
            sub_odom.subscribe(nh, "/hsrb/wheel_odom", 1);
            //Justina
            //sub_laser_scan.subscribe(nh, "/hardware/scan", 1);
            //sub_odom.subscribe(nh, "/amcl_pose", 1);
            Syncr_la_od.reset(new SyncrLaOd(SyncPolicyLaOd(5), sub_laser_scan, sub_odom));
            Syncr_la_od->registerCallback(boost::bind(&DataGenerator::saveLaserOdomCallback, this, _1, _2));
        }

        if (save_rgbd)
        {
            ROS_INFO("Save RGBD enabled");
            //Takeshi
            sub_rgb.subscribe(nh, "/hsrb/head_rgbd_sensor/rgb/image_raw", 10);                // set queue_size to 1 to synchronize with loop_rate
            sub_depth.subscribe(nh, "/hsrb/head_rgbd_sensor/depth_registered/image_raw", 10); // set queue_size to 1 to synchronize with loop_rate
            //Justina
            //sub_rgb.subscribe(nh, "/camera/rgb/image_raw", 10);                // set queue_size to 1 to synchronize with loop_rate
            //sub_depth.subscribe(nh, "/camera/depth_registered/image_raw", 10); // set queue_size to 1 to synchronize with loop_rate
            

            Syncr_img_img.reset(new SyncImgImg(SyncPolicyImgImg(5), sub_rgb, sub_depth));
            Syncr_img_img->registerCallback(boost::bind(&DataGenerator::saveRGBDCallback, this, _1, _2));
        }
    }
    ~DataGenerator()
    {
        if (file_stream.is_open())
            file_stream.close();
    }

    void setStable()
    {
        stabilized = true;
    }

    void saveLaserOdomCallback(const sensor_msgs::LaserScan::ConstPtr &scan, const nav_msgs::Odometry::ConstPtr &odom)
    {
        int ranges_size = scan->ranges.size();
        int index;
        // Get Euler orientation
        tf::Quaternion tf_quaternion(odom->pose.pose.orientation.x, odom->pose.pose.orientation.y, odom->pose.pose.orientation.z, odom->pose.pose.orientation.w);
        tf::Matrix3x3 rot_mat(tf_quaternion);
        double roll, pitch, yaw;
        rot_mat.getRPY(roll, pitch, yaw);

        if (set_header)
        {
            header_odom_laser = "time_stamp,";
            for (int i = 0; i < ranges_size; i++)
            {
                header_odom_laser += "lect_" + std::to_string(i) + ",";
            }
            file_stream << header_odom_laser << "Px,Py,e-th" << std::endl;
            set_header = false;
        }

        // TIME STAMP
        std::string time_frame = std::to_string(scan->header.stamp.sec) + "-" + std::to_string(scan->header.stamp.nsec).substr(0, 3);
        file_stream << time_frame << ",";

        // LASER SCAN
        std::vector<float> laser_lectures = scan->ranges;
        // Lectures: sim_takeshi = 721,  real_takeshi = 963
        for (auto elem : laser_lectures)
        {
            if (elem > max_laser_range)
                file_stream << max_laser_range;
            else
                file_stream << elem;

            file_stream << ",";
        }
        ROS_INFO_STREAM("Ranges saved: " << ranges_size);
        // ODOMETRY
        file_stream << odom->pose.pose.position.x << "," << odom->pose.pose.position.y << "," << yaw << std::endl;
        ROS_INFO_STREAM("save odom: (" << odom->pose.pose.position.x << ", " << odom->pose.pose.position.y << ", " << yaw << ")");
    }

    void saveRGBDCallback(const sensor_msgs::Image::ConstPtr &rgb_msg, const sensor_msgs::Image::ConstPtr &depth_msg)
    {
        cv::Mat rgb_image = cv_bridge::toCvShare(rgb_msg)->image;
        cv::Mat depth_image = cv_bridge::toCvShare(depth_msg)->image;
        if (depth_msg->encoding == "32FC1")
        { // Depth channel
            depth_image.convertTo(depth_image, CV_8UC3, 32.0);
        }
        if (depth_msg->encoding == "16UC1")
        { // Depth channel
            depth_image.convertTo(depth_image, CV_8UC3, 1/12.0); // 1/12.0 to 1/16.0 Note: use the .0
        }
        if (rgb_msg->encoding == "rgb8")
        { // invert channels on real robot
            cv::cvtColor(rgb_image, rgb_image, cv::COLOR_RGB2BGR);
        }


        std::string rgb_frame = std::to_string(rgb_msg->header.stamp.sec) + "-" + std::to_string(rgb_msg->header.stamp.nsec).substr(0, 5);
        std::string rgb_img_name = rgb_msg->header.frame_id.substr(0, 16) + "_image_raw-frame--" + rgb_frame.c_str() + ".png"; // 17-25
        std::string rgb_img_path = path_file + "Images/" + rgb_img_name;

        std::string depth_frame = std::to_string(depth_msg->header.stamp.sec) + "_" + std::to_string(depth_msg->header.stamp.nsec).substr(0, 5);
        std::string depth_img_name = depth_msg->header.frame_id.substr(0, 16) + "_depth_registered_image-frame--" + depth_frame.c_str() + ".png";
        std::string depth_img_path = path_file + "Images/" + depth_img_name;

        bool same_time = rgb_msg->header.stamp.sec == depth_msg->header.stamp.sec;
        if(stabilized && time_stamp_secs != rgb_msg->header.stamp.sec && same_time)
        {
            int rgb_writed = cv::imwrite(rgb_img_path, rgb_image);
            if (rgb_writed)
                ROS_INFO_STREAM("image saved: " << rgb_img_name.c_str());
            else
                ROS_WARN("image not saved!");

            int depth_writed = cv::imwrite(depth_img_path, depth_image);
            if (depth_writed)
                ROS_INFO_STREAM("image saved: " << depth_img_name.c_str());
            else
                ROS_WARN("image not saved!");

            cv::imshow("image_viewer", rgb_image);
            cv::imshow("depth_viewer", depth_image);
            cv::waitKey(1);
        }
        time_stamp_secs = rgb_msg->header.stamp.sec;

    }
};

int main(int argc, char *argv[])
{
    bool enable_laser_odometry = true, enable_rgbd = true; // Change to enable/disable save_topic

    bool save_laser_odometry = false, save_rgbd = false; // DON'T change it, used internally on callback
    ros::init(argc, argv, "hmm_capture_data_node");
    ros::NodeHandle nh("~");
    cv::namedWindow("image_viewer");
    cv::namedWindow("depth_viewer");
    ROS_INFO("Starting  hmm_capture_data_node");

    if (enable_laser_odometry)
        save_laser_odometry = true;
    if (enable_rgbd)
        save_rgbd = true;
    DataGenerator data_obtainer(save_laser_odometry, save_rgbd, false);

    int count = 0;
    ros::Rate loop_rate(1); // <<<<< TIME controller
    while (ros::ok())
    {
        count ++;
        if(count == 5){
            data_obtainer.setStable();
        }
        // Disable and sleep
        if (enable_laser_odometry)
            save_laser_odometry = false;
        if (enable_rgbd)
            save_rgbd = false;
        ROS_INFO_STREAM("callback sleep" << std::endl);
        ros::spinOnce();
        loop_rate.sleep();
        // Enable after sleep
        if (enable_laser_odometry)
            save_laser_odometry = true;
        if (enable_rgbd)
            save_rgbd = true;
    }
    return 0;
}