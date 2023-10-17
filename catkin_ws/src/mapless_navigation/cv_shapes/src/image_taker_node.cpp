#include <ros/ros.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/String.h>

#include "general_utils/files_utils.h"

class ImageManager
{
private:
    ros::Publisher pub;
    ros::Subscriber sub;
    image_transport::Subscriber img_sub;
    std::string path_files;
    int cv_writer;

public:
    ImageManager(ros::NodeHandle *node_handle, std::string topic, int queue)
    {
        path_files = ros::package::getPath("cv_shapes") + "/images/";
        // publish

        // subscribe

        image_transport::ImageTransport img_t(*node_handle);
        img_sub = img_t.subscribe(topic, queue, &ImageManager::saveImagesCallback, this);
    }

    std::string getPahFiles(){
        return path_files;
    }

    void saveImagesCallback(const sensor_msgs::ImageConstPtr &msg)
    {
        cv::namedWindow("Camera_viewer");
        try
        {
            cv::imshow("Camera_viewer", cv_bridge::toCvShare(msg, "bgr8")->image);
            std::string frame_id = std::to_string(msg->header.stamp.sec) + "_" + std::to_string(msg->header.stamp.nsec).substr(0, 4);
            std::string img_name = msg->header.frame_id + "_" + frame_id.c_str() + ".jpg";
            std::string img_path = path_files + img_name;

            cv_writer = cv::imwrite(img_path, cv_bridge::toCvShare(msg, "bgr8")->image);
            if(cv_writer)
                ROS_INFO_STREAM("image saved: " << img_name.c_str() << std::endl);
            else    
                ROS_WARN("image not saved!");

            cv::waitKey(1);
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
        }
    }
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "image_taker_node");
    ros::NodeHandle nh;
    
    ImageManager img_taker = ImageManager(&nh, "hsrb/head_center_camera/image_raw", 1);
    DirectoryUtils::replaceDir(img_taker.getPahFiles(), true);
    
    ros::Rate loop_rate(2);
    while (ros::ok())
    {

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
