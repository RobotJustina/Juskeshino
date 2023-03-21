#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/PointCloud2.h"
#include "pcl_ros/transforms.h"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Int32MultiArray.h"

//sensor_msgs::PointCloud2::Ptr point_cloud_ptr2;
sensor_msgs::PointCloud2 pc_msg;
cv::Mat bgr_dest;
cv::Mat pc_dest;
int info=0;
//int votes[3][3]={};
tf::TransformListener* tf_listener;
ros::Publisher pub_votes;

void callback_pointcloud(sensor_msgs::PointCloud2 msg){
    int votes[3][3]={};
    float xmin=-0.549;
    float ymin=-1.199;
    float d=0.8;
    pcl_ros::transformPointCloud("base_link", msg, pc_msg, *tf_listener);
    int offset_x = 0;
    int offset_y = 4;
    int offset_z = 8;
    int offset_bgr  = 12;
    for(int i=0; i < pc_msg.fields.size(); i++)
        if (pc_msg.fields[i].name == "x")
            offset_x = pc_msg.fields[i].offset;
        else if (pc_msg.fields[i].name == "y")
            offset_y = pc_msg.fields[i].offset;
        else if (pc_msg.fields[i].name == "z")
            offset_z = pc_msg.fields[i].offset;
        else if (pc_msg.fields[i].name == "rgb")
            offset_bgr = pc_msg.fields[i].offset;
        else if (pc_msg.fields[i].name == "rgba")
            offset_bgr = pc_msg.fields[i].offset;
    bgr_dest = cv::Mat::zeros(pc_msg.height, pc_msg.width, CV_8UC3);
    pc_dest  = cv::Mat::zeros(pc_msg.height, pc_msg.width, CV_32FC3);
    for(int i=0; i < bgr_dest.cols; i++)
        for(int j=0; j < bgr_dest.rows; j++)
        {
            float* x = (float*)&pc_msg.data[(j*pc_msg.width + i)*pc_msg.point_step + offset_x];
            float* y = (float*)&pc_msg.data[(j*pc_msg.width + i)*pc_msg.point_step + offset_y];
            float* z = (float*)&pc_msg.data[(j*pc_msg.width + i)*pc_msg.point_step + offset_z];
            pc_dest.at<cv::Vec3f>(j, i)[0] = *x;
            pc_dest.at<cv::Vec3f>(j, i)[1] = *y;
            pc_dest.at<cv::Vec3f>(j, i)[2] = *z;
            bgr_dest.data[j*bgr_dest.step + i*3]     = pc_msg.data[(j*pc_msg.width + i)*pc_msg.point_step + offset_bgr];
            bgr_dest.data[j*bgr_dest.step + i*3 + 1] = pc_msg.data[(j*pc_msg.width + i)*pc_msg.point_step + offset_bgr + 1];
            bgr_dest.data[j*bgr_dest.step + i*3 + 2] = pc_msg.data[(j*pc_msg.width + i)*pc_msg.point_step + offset_bgr + 2];
            //next lines get the votes in order to know the robot state
            int r= (pc_dest.at<cv::Vec3f>(j, i)[0]-xmin)/d;
            int c= (pc_dest.at<cv::Vec3f>(j, i)[1]-ymin)/d;
            float h= pc_dest.at<cv::Vec3f>(j, i)[2];
            if(r<=2 && r>=0 && c<=2 && c>=0 && h>0.2)
                votes[r][int(2-c)]+=1;

        }
    //for(size_t i=0; i < bgr_dest.cols; i++)
        //for(size_t j=0; j < bgr_dest.rows; j++){
            //int r= (pc_dest.at<cv::Vec3f>(j, i)[0]-xmin)/d;
            //int c= (pc_dest.at<cv::Vec3f>(j, i)[1]-ymin)/d;
            //float h= pc_dest.at<cv::Vec3f>(j, i)[2];
            //if(r<=2 && r>=0 && c<=2 && c>=0 && h>0.2)
                //votes[r][int(2-c)]+=1;
    //}
    info=1;
    for(int i=0; i<=2; i++)
        for(int j=0; j<=2; j++){
            if(votes[i][j]>=10)
                votes[i][j]=1;
            else
                votes[i][j]=0;
    }
    std_msgs::Int32MultiArray msg_votes;
    msg_votes.data.resize(6);
    msg_votes.data[0] = votes[0][0];
    msg_votes.data[1] = votes[0][1];
    msg_votes.data[2] = votes[0][2];
    msg_votes.data[3] = votes[1][0];
    msg_votes.data[4] = votes[1][1];
    msg_votes.data[5] = votes[1][2];
    pub_votes.publish(msg_votes);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "get_votes");
    ros::NodeHandle n;
    tf_listener = new tf::TransformListener();
    ros::Subscriber sub = n.subscribe("/hardware/realsense/points", 10, callback_pointcloud);
    ros::Publisher pub =  n.advertise<std_msgs::Float64MultiArray>("/hardware/head/goal_pose", 10);
    pub_votes=n.advertise<std_msgs::Int32MultiArray>("/votes", 10);
    ros::Rate loop(20);
    std_msgs::Float64MultiArray msg_head;
    msg_head.data.resize(2);
    msg_head.data[0] = 0;
    msg_head.data[1] = -1.5;
    pub.publish(msg_head);
    while(ros::ok()){
        ros::spinOnce();
        loop.sleep();
        if(info==0){
            std::cout<<"Moviendo cámara"<<std::endl;
            pub.publish(msg_head);
        }
            //continue;
        //cv::imshow("Visualizar",pc_dest);
        //if(cv::waitKey(3)==27)
            //break;
    }
};
