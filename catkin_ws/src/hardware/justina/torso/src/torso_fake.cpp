#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>

#define TH_ERR 0.001


int main(int argc, char ** argv){
    ros::init(argc, argv, "torso_node_fake");
    ros::NodeHandle n;
    ros::Publisher pubJointState = n.advertise<sensor_msgs::JointState>("/joint_states", 1);
    ros::Rate rate(30);
    
    //std::string names[5] = {"spine_connect","waist_connect","shoulders_connect", "shoulders_left_connect", "shoulders_right_connect"};
    //float positions[5] = {0, 0, 0, 0, 0};
    //float deltaPose[5] = {0.0, 0.0, 0.0, 0.0, 0.0};

    sensor_msgs::JointState jointStates;
    jointStates.name.push_back("spine_connect");
    jointStates.position.push_back(0);

    while(ros::ok()){
        jointStates.header.stamp = ros::Time::now();
        jointStates.position[0] = 0.0;
        pubJointState.publish(jointStates);
        rate.sleep();
        ros::spinOnce();
    }
    return 0;
}
