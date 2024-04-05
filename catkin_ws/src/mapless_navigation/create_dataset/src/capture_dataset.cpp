#include <ros/ros.h>
#include <ros/package.h>
#include <fstream>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>
#include "tf/transform_listener.h"
#include <cv_bridge/cv_bridge.h>
// Sync messages
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>

#include "general_utils/files_utils.h" // DirectoryUtils

class DataGenerator
{
private:
    // Basics
    std::ofstream file_stream;
    bool set_header;
    std::string header_odom_laser;
    std::string path_file;
    bool stabilized = false; // Flag to control when to start recording
    int time_stamp_secs = 0;
    // RGBD--Synchronizer
    message_filters::Subscriber<sensor_msgs::Image> sub_rgb;
    message_filters::Subscriber<sensor_msgs::Image> sub_depth;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> SyncPolicyImgImg; // ExactTime Don't work on real robot
    typedef message_filters::Synchronizer<SyncPolicyImgImg> SyncImgImg;
    boost::shared_ptr<SyncImgImg> Syncr_img_img;
    // Laser_scan
    float max_laser_range;
    ros::Subscriber sub_laser_scan;
    // Robot position
    tf::TransformListener tf_listener;
    tf::StampedTransform transform;
    std::string base_link_name;

public:
    DataGenerator(ros::NodeHandle &node_handle, bool save_laser = false, bool save_rgbd = false, bool incremental_save = false)
    {
        ROS_INFO_STREAM("Node: " << node_handle.getNamespace());
        // Prepare directory
        path_file = ros::package::getPath("create_dataset") + "/Dataset/";
        if (incremental_save && DirectoryUtils::existDir(path_file))
        {
            file_stream.open(path_file + "laser_odom_data.csv", std::ios::app);
            set_header = false;
        }
        else
        {
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

        if (save_laser)
        {
            ROS_INFO("Save laser_odom enabled");
            max_laser_range = 10.0;
            /*
                Use launch files to remap topics

                Takeshi
                "/hsrb/base_scan"

                Justina
                "/hardware/scan"
            */

            sub_laser_scan = node_handle.subscribe("/scan", 1, &DataGenerator::saveLaserPositionCallback, this);
            if (ros::param::has("/base_link_name"))
                ros::param::get("/base_link_name", base_link_name);
            else
                base_link_name = "base_footprint";
        }

        if (save_rgbd)
        {
            ROS_INFO("Save RGBD enabled");
            /*
                Use launch files to remap topics

                Takeshi
                /hsrb/head_rgbd_sensor/rgb/image_raw
                /hsrb/head_rgbd_sensor/depth_registered/image_raw

                Justina Real
                "/camera/rgb/image_color"
                "/hardware/realsense/depth/image_raw"

                Justina Virtual
                /hardware/realsense/rgb/image_raw
                /hardware/realsense/depth/image_raw
            */
            sub_rgb.subscribe(node_handle, "/rgb/image_raw", 10);     // set queue_size to 1 to synchronize with loop_rate
            sub_depth.subscribe(node_handle, "/depth/image_raw", 10); // set queue_size to 1 to synchronize with loop_rate

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

    void saveLaserPositionCallback(const sensor_msgs::LaserScan::ConstPtr &scan)
    {
        int ranges_size = scan->ranges.size();
        float robot_x, robot_y, robot_th;

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

        if (stabilized)
        {
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

            // Robot position
            tf_listener.lookupTransform("map", base_link_name, ros::Time(0), transform);
            robot_x = transform.getOrigin().x();
            robot_y = transform.getOrigin().y();
            tf::Quaternion q = transform.getRotation();
            robot_th = atan2((float)q.z(), (float)q.w()) * 2;
            file_stream << robot_x << "," << robot_y << "," << robot_th << std::endl;
            ROS_INFO_STREAM("save pos: (" << robot_x << ", " << robot_y << ", " << robot_th << ")");
        }
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
        {                                                          // Depth channel
            depth_image.convertTo(depth_image, CV_8UC3, 1 / 12.0); // 1/12.0 to 1/16.0 Note: use the .0
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
        if (stabilized && time_stamp_secs != rgb_msg->header.stamp.sec && same_time) // stabilized && time_stamp_secs != rgb_msg->header.stamp.sec && same_time
        {
            ROS_INFO_STREAM("Image encoding: " << depth_msg->encoding);
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
    bool enable_laser_position = true, enable_rgbd = true; // Change to enable/disable save_topic
    bool incremental_save = false;

    bool save_laser_position = false, save_rgbd = false; // DON'T change it, used internally on callback
    ros::init(argc, argv, "capture_data_node");
    ros::NodeHandle nh("~");
    cv::namedWindow("image_viewer");
    cv::namedWindow("depth_viewer");
    ROS_INFO_STREAM("Starting node: " << nh.getNamespace());

    if (enable_laser_position)
        save_laser_position = true;
    if (enable_rgbd)
        save_rgbd = true;
    DataGenerator data_obtainer(nh, save_laser_position, save_rgbd, incremental_save);

    int count = 0;
    ros::Rate loop_rate(.25); // <<<<< TIME controller
    while (ros::ok())
    {
        count++;
        // Disable and sleep
        if (enable_laser_position)
            save_laser_position = false;
        if (enable_rgbd)
            save_rgbd = false;
        if (count == 4)
        {
            data_obtainer.setStable();
        }
        ROS_INFO_STREAM("callback sleep" << std::endl);
        ros::spinOnce();
        loop_rate.sleep();
        // Enable after sleep
        if (enable_laser_position)
            save_laser_position = true;
        if (enable_rgbd)
            save_rgbd = true;
    }
    return 0;
}