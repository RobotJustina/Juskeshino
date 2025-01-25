#include "ros/ros.h"
#include "ros/package.h"
#include "Skeleton.h"
#include "SkeletonDetector.h"
#include "std_msgs/Bool.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/Image.h"
#include "vision_msgs/Person.h"
#include "vision_msgs/Persons.h"
#include "vision_msgs/Keypoint.h"
#include "vision_msgs/Gesture.h"
#include "visualization_msgs/Marker.h"
#include "pcl_conversions/pcl_conversions.h"
#include "cv_bridge/cv_bridge.h"
#include "tf/transform_listener.h"
#include "tf_conversions/tf_eigen.h"
#include "opencv2/opencv.hpp"

ros::Subscriber sub_cloud;
ros::NodeHandle* nh;
tf::TransformListener* tf_listener;
SkeletonDetector *sk_detector;
SkeletonTracker* sk_tracker;
ros::Publisher  pub_persons;
ros::Publisher  pub_marker;
const std::vector<std::string> kpt_names{"nose", "neck", "r_sho", "r_elb", "r_wri", "l_sho", "l_elb", "l_wri",
        "r_hip", "r_knee", "r_ank", "l_hip", "l_knee", "l_ank",	"r_eye", "l_eye","r_ear", "l_ear"};

Eigen::Affine3d get_transform_to_baselink(std::string link_name)
{
    tf::StampedTransform tf;
    tf_listener->lookupTransform("base_link", link_name, ros::Time(0), tf);
    Eigen::Affine3d e;
    tf::transformTFToEigen(tf, e);
    return e;
}

void get_body_poses(vision_msgs::Persons& msg)
{
    Eigen::Affine3d tf = get_transform_to_baselink(msg.header.frame_id);
    for(int i=0; i < msg.persons.size(); i++){
        for(int j=0; j<msg.persons[i].skeleton.size(); j++)
            if(msg.persons[i].skeleton[j].id == "nose"){
                Eigen::Vector3d v(msg.persons[i].skeleton[j].pose.position.x, msg.persons[i].skeleton[j].pose.position.y, msg.persons[i].skeleton[j].pose.position.z);
                v = tf * v;
                if(v.z() > 1.2) msg.persons[i].body_pose.id = vision_msgs::Gesture::STANDING;
                else if (v.z() > 0.4) msg.persons[i].body_pose.id = vision_msgs::Gesture::SITTING;
                else msg.persons[i].body_pose.id = vision_msgs::Gesture::LYING;
            }
    }
}

visualization_msgs::Marker get_human_pose_markers(vision_msgs::Persons& msg)
{
    visualization_msgs::Marker mrk;
    mrk.header.frame_id = msg.header.frame_id;
    mrk.header.stamp = ros::Time::now();
    mrk.ns = "human_pose";
    mrk.id = 0;
    mrk.type = visualization_msgs::Marker::SPHERE_LIST;
    mrk.action = visualization_msgs::Marker::ADD;
    mrk.pose.orientation.w = 1.0;
    mrk.scale.x = 0.1;
    mrk.scale.y = 0.1;
    mrk.scale.z = 0.1;
    mrk.color.a = 1.0; // Don't forget to set the alpha!
    mrk.color.r = 1.0;
    mrk.color.g = 0.5;
    mrk.color.b = 0.0;
    for(size_t i=0; i < msg.persons.size(); i++)
        for(size_t j=0; j < msg.persons[i].skeleton.size(); j++)
        {
            geometry_msgs::Point p;
            p.x = msg.persons[i].skeleton[j].pose.position.x;
            p.y = msg.persons[i].skeleton[j].pose.position.y;
            p.z = msg.persons[i].skeleton[j].pose.position.z;
            mrk.points.push_back(p);
        }
    return mrk;
}

void callback_cloud(const sensor_msgs::PointCloud2::ConstPtr& pcl_topic)
{
    sensor_msgs::Image pcl_image;
    cv::Mat image;
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*pcl_topic, pcl_pc2);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2xyz(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromPCLPointCloud2(pcl_pc2, *cloud2xyz);
    pcl::toROSMsg (*pcl_topic, pcl_image);
    
    try{
        image = cv_bridge::toCvCopy(pcl_image, sensor_msgs::image_encodings::BGR8)->image;
    }
    catch (cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }

    std::vector<Skeleton> skeletons = sk_detector->run(image);
    sk_tracker->track(skeletons);

    vision_msgs::Persons msg_persons;
    msg_persons.header.frame_id = pcl_topic->header.frame_id;
    msg_persons.header.stamp = ros::Time::now();
    msg_persons.persons.resize(skeletons.size()); //how many people

    for (int j=0; j < skeletons.size(); j++)
    {
        skeletons[j].draw(image, true);
        msg_persons.persons[j].id = "person"+ std::to_string(j);
        msg_persons.persons[j].name = "person"+ std::to_string(j);
        msg_persons.persons[j].pose.position.x = 0;
        msg_persons.persons[j].pose.position.y = 0;
        msg_persons.persons[j].pose.position.z = 0;
        
        for (int i = 0; i < skeletons[j].keypoints.size(); i++)
        {
            int pix_x = skeletons[j].keypoints[i].x;
            int pix_y = skeletons[j].keypoints[i].y;
            
            if (pix_x < cloud2xyz->width && pix_y < cloud2xyz->height && !std::isnan(cloud2xyz->at(pix_x, pix_y).z))
            {
                pcl::PointXYZRGB p = cloud2xyz->at(pix_x, pix_y);
                vision_msgs::Keypoint kp;
                kp.id = kpt_names[i];
                kp.pose.position.x = p.x;
                kp.pose.position.y = p.y;
                kp.pose.position.z = p.z; 
                kp.confidence = skeletons[j].confidence;
                msg_persons.persons[j].skeleton.push_back(kp);
                msg_persons.persons[j].pose.position.x += p.x;
                msg_persons.persons[j].pose.position.y += p.y;
                msg_persons.persons[j].pose.position.z += p.z;
            }
        }
        if(msg_persons.persons[j].skeleton.size() > 0){
            msg_persons.persons[j].pose.position.x /= msg_persons.persons[j].skeleton.size();
            msg_persons.persons[j].pose.position.y /= msg_persons.persons[j].skeleton.size();
            msg_persons.persons[j].pose.position.z /= msg_persons.persons[j].skeleton.size();
        }else{
            msg_persons.persons[j].pose.position.x = std::nan("0");
            msg_persons.persons[j].pose.position.y = std::nan("0");
            msg_persons.persons[j].pose.position.z = std::nan("0");
        }
        msg_persons.persons[j].header.frame_id = pcl_topic->header.frame_id;
        msg_persons.persons[j].header.stamp = ros::Time::now();
    }
    get_body_poses(msg_persons);
    for (int j=0; j < skeletons.size(); j++)
    {
        std::string str;
        if(msg_persons.persons[j].body_pose.id == vision_msgs::Gesture::STANDING) str = "Standing";
        else if(msg_persons.persons[j].body_pose.id == vision_msgs::Gesture::SITTING) str = "Sitting";
        else str = "Lying";
        cv::putText(image, str, cv::Point(skeletons[j].bbox.x, skeletons[j].bbox.y), cv::FONT_HERSHEY_COMPLEX, 0.6, cv::Scalar(0,255,0));
    }   
    
    pub_persons.publish(msg_persons);
    pub_marker.publish(get_human_pose_markers(msg_persons));
    cv::imshow("HumanPoseEstimator",image);
    cv::waitKey(1);
}

void callback_enable(const std_msgs::Bool::ConstPtr& msg)
{
    if(msg->data)
    {
        std::cout << "SkeletonDetector.->Starting human pose detection"<<std::endl;
        sub_cloud  = nh->subscribe("/camera/depth_registered/points", 1, callback_cloud);
    }
    else
    {
        std::cout << "SkeletonDetector.->Stopping human pose detection..." <<std::endl;
        sub_cloud.shutdown();
        cv::destroyAllWindows();
    }
}

int main(int argc, char** argv)
{
    std::cout << "INITIALIZING SKELETON DETECTOR BY RYOHEI AND MARCOSOFT..." << std::endl;
    ros::init(argc, argv, "skeleton_detetor");
    ros::NodeHandle n;
    nh = &n;
    ros::Rate loop(10);

    std::string path = ros::package::getPath("skeleton_detector");
    std::stringstream ss;
    ss << path << "/models/poseEstimationModel.onnx";
    sk_detector = new SkeletonDetector(ss.str());
    sk_tracker  = new SkeletonTracker();
    tf_listener = new tf::TransformListener();
    tf_listener->waitForTransform("map", "base_link", ros::Time(0), ros::Duration(10.0));

    pub_persons = n.advertise<vision_msgs::Persons>("/vision/human_pose/human_pose_array", 1);
    pub_marker  = n.advertise<visualization_msgs::Marker>("/vision/human_pose/human_pose_marker", 1);
    ros::Subscriber sub_enable  = n.subscribe("/vision/human_pose/enable", 1, callback_enable);

    while(ros::ok())
    {
        loop.sleep();
        ros::spinOnce();
    }
}
