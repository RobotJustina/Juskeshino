#include "ros/ros.h"
#include <ros/package.h>
#include <cv_bridge/cv_bridge.h>
#include "vision_msgs/RecognizeObject.h"
#include "vision_msgs/VisionObject.h"
#include "visualization_msgs/MarkerArray.h"


int main(int argc, char** argv)
{
    std::cout << "INITIALIZING OBJECT POSE AND GRIP NODE BY IBY ****(✿◠‿◠)/**" << std::endl;
    ros::init(argc, argv, "obj_pose_and_grip_node");
    ros::NodeHandle n;

    /*
    ros::ServiceServer srvObjPose     = n.advertiseService("/vision/obj_pose_and_grip/get_obj_pose", callback_get_obj_pose);
    ros::ServiceServer srvOptimunGrip = n.advertiseService("/vision/obj_pose_and_grip/optimun_grip", callback_optimun_grip);
    */

    ros::Rate loop(30);
    
    while(ros::ok() && cv::waitKey(10) != 27)
    {
        ros::spinOnce();
        loop.sleep();
    }
    

}