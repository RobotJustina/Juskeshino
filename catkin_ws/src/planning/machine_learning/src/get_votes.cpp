#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/PointCloud2.h"
#include "pcl_ros/transforms.h"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Int32MultiArray.h"
#include "actionlib_msgs/GoalStatus.h"

//sensor_msgs::PointCloud2::Ptr point_cloud_ptr2;
sensor_msgs::PointCloud2 pc_msg;
cv::Mat bgr_dest;
cv::Mat pc_dest;
bool cam_ready=false;
// Variable that wait for goal_reached
//int votes[3][3]={};
tf::TransformListener* tf_listener;
ros::Publisher pub_votes;
ros::Publisher pub_ready;
int edo=1;
int count=0;
//Create message global message msg_head-->head goal
std_msgs::Float64MultiArray msg_head;
//Global array to save the last votes
std::vector<std::vector<int>> Last_votes = {
    {0, 0, 0},
    {0, 0, 0},
    {0, 0, 0}
};

std::vector<std::vector<int>> Pointcloud_to_Array(sensor_msgs::PointCloud2 msg) {
    std::vector<std::vector<int>> Array(3, std::vector<int>(3, 0));
    float xmin=-0.249;
    float ymin=-0.749;
    float d=0.5;
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
                //votes[r][int(2-c)]+=1;
                Array[r][int(2-c)]+=1;

        }
    for(int i=0; i<=2; i++)
        for(int j=0; j<=2; j++){
            if(Array[i][j]>=10)
                Array[i][j]=1;
            else
                Array[i][j]=0;
    }
    //std::cout << Array[0][0]<<""<< Array[0][1]<<""<< Array[0][0] << std::endl;
    return Array;
}


void callback_pointcloud(sensor_msgs::PointCloud2 msg){
    if(cam_ready && (edo==1 || edo==2 || edo==3 || edo==4) ){
        //std::cout<<edo<<std::endl;
        std::vector<std::vector<int>> votes = Pointcloud_to_Array(msg);
        //Last_votes=
        for(int i=0; i<=2; i++)
            for(int j=0; j<=2; j++)
                if(Last_votes[i][j]==1 || votes[i][j]==1)
                    Last_votes[i][j]=1;
        if(Last_votes[0][0]==0 && edo==1 && Last_votes[0][2]==0)
            edo=2;
        else if(Last_votes[0][2]==0 && edo==1)
            edo=4;
        else if(Last_votes[0][0]==0 && edo==1)
            edo=3;
        else if( (Last_votes[0][0]==1 && edo==1 && Last_votes[0][2]==1) || edo==3)
            edo=5;
        else
            edo+=1;
        cam_ready=false;
        if(edo==5){
            std::cout<<"Publicar topicos"<<std::endl;
            std_msgs::Int32MultiArray msg_votes;
            msg_votes.data.resize(6);
            msg_votes.data[0] = Last_votes[0][0];
            msg_votes.data[1] = Last_votes[0][1];
            msg_votes.data[2] = Last_votes[0][2];
            msg_votes.data[3] = Last_votes[1][0];
            msg_votes.data[4] = Last_votes[1][1];
            msg_votes.data[5] = Last_votes[1][2];
            pub_votes.publish(msg_votes);
            Last_votes = { {0, 0, 0},{0, 0, 0},{0, 0, 0} };
        }
    }

}

void callback_goal(actionlib_msgs::GoalStatus msg){
    if(msg.status==3)
        edo=1;
}

void callback_head(std_msgs::Float64MultiArray msg){
    float tol=0.02;
    //msg_head is the goal, if the Error between the goal and the current pose is less than the tolerance info=1
    if (count ==6){
        count=0;
        cam_ready=true;
    }
    else if(msg.data[0]<msg_head.data[0]+tol && msg.data[0]>msg_head.data[0]-tol && msg.data[1]<msg_head.data[1]+tol && msg.data[1]>msg_head.data[1]-tol && edo!=0)
        count+=1;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "get_votes");
    ros::NodeHandle n;
    tf_listener = new tf::TransformListener();
    ros::Subscriber sub = n.subscribe("/hardware/realsense/points", 10, callback_pointcloud);
    ros::Subscriber sub_pose = n.subscribe("/hardware/head/current_pose", 10, callback_head);
    ros::Subscriber sub_goal = n.subscribe("/simple_move/goal_reached", 10, callback_goal);
    ros::Publisher pub =  n.advertise<std_msgs::Float64MultiArray>("/hardware/head/goal_pose", 10);
    pub_ready = n.advertise<actionlib_msgs::GoalStatus>("/ready", 10);
    pub_votes = n.advertise<std_msgs::Int32MultiArray>("/votes", 10);
    ros::Rate loop(20);

    //std_msgs::Float64MultiArray msg_head;
    msg_head.data.resize(2);
    actionlib_msgs::GoalStatus msg_ready;
    //msg_head.data[0] = 0;
    //msg_head.data[1] = -1.15;
    while(ros::ok()){
        ros::spinOnce();
        loop.sleep();
        switch(edo){
            case 1:
                std::cout<<"Centro"<<std::endl;
                msg_head.data[0] = 0;
                msg_head.data[1] = -1.15;
                pub.publish(msg_head);
            break;
            case 2:
                std::cout<<"Derecha estado 2"<<std::endl;
                msg_head.data[0] = -0.9;
                msg_head.data[1] = -1.15;
                pub.publish(msg_head);
            break;
            case 3:
                std::cout<<"Izquierda"<<std::endl;
                msg_head.data[0] = 0.9;
                msg_head.data[1] = -1.15;
                pub.publish(msg_head);
            break;
            case 4:
                std::cout<<"Derecha estado 4"<<std::endl;
                msg_head.data[0] = -0.9;
                msg_head.data[1] = -1.15;
                pub.publish(msg_head);
            break;
            case 5:
                msg_ready.status=3;
                pub_ready.publish(msg_ready);
                edo=0;
            break;
            default: break;
        }
            //continue;
        //cv::imshow("Visualizar",pc_dest);
        //if(cv::waitKey(3)==27)
            //break;
    }
};
