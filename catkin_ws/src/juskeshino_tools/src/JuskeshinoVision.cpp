#include "juskeshino_tools/JuskeshinoVision.h"

bool JuskeshinoVision::is_node_set = false;
//Sevices for line finding
ros::ServiceClient JuskeshinoVision::cltFindLines;
//Services for object recognition
ros::ServiceClient JuskeshinoVision::cltRecogObjects;
//Services for training objects
ros::ServiceClient JuskeshinoVision::cltTrainObject;

bool JuskeshinoVision::setNodeHandle(ros::NodeHandle* nh)
{
    if(JuskeshinoVision::is_node_set)
        return true;
    if(nh == 0)
        return false;

    std::cout << "JuskeshinoVision.->Setting ros node..." << std::endl;
    //Services for line finding
    JuskeshinoVision::cltFindLines = nh->serviceClient<vision_msgs::FindLines>("/vision/line_finder/find_lines_ransac");
    //Services for object recognition
    JuskeshinoVision::cltRecogObjects = nh->serviceClient<vision_msgs::RecognizeObjects>("/vision/obj_reco/recognize_objects");
    //Services for training objects
    JuskeshinoVision::cltTrainObject = nh->serviceClient<vision_msgs::RecognizeObject>("/vision/obj_reco/train_object");

    JuskeshinoVision::is_node_set = true;

    ros::Rate loop(10);
    int counter = 3;
    while(ros::ok() && counter > 0)
    {
        counter--;
        ros::spinOnce();
        loop.sleep();
    }
    return true;
}


//Methods for line finding
bool JuskeshinoVision::findLine(float& x1, float& y1, float& z1, float& x2, float& y2, float& z2)
{
    std::cout << "JuskeshinoVision.->Trying to find a straight line." << std::endl;
    
    
    return true;
}

//Methods for object recognition
bool JuskeshinoVision::recognizeObjects(std::vector<vision_msgs::VisionObject>& recog_objects)
{
    vision_msgs::RecognizeObjects srv;
    boost::shared_ptr<sensor_msgs::PointCloud2 const> ptr;
    ptr = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/hardware/kinect/rgbd_wrt_robot", ros::Duration(1.0));
    srv.request.point_cloud = *ptr;
    if(!cltRecogObjects.call(srv)) return false;
    recog_objects = srv.response.recog_objects;
    return true;
}

bool JuskeshinoVision::recognizeObject(std::string name, vision_msgs::VisionObject& recog_object)
{
    vision_msgs::RecognizeObjects srv;
    return false;
}

//Methods for object training
bool JuskeshinoVision::trainObject(std::string name)
{
    vision_msgs::RecognizeObject srv;
    boost::shared_ptr<sensor_msgs::PointCloud2 const> ptr;
    ptr = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/hardware/kinect/rgbd_wrt_robot", ros::Duration(1.0));
    srv.request.point_cloud = *ptr;
    srv.request.id = name;
    return cltTrainObject.call(srv);
}
