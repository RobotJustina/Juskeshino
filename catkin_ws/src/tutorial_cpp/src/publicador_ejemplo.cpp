#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>


void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("I heard: [%s]*************", msg->data.c_str());
}


int main(int argc, char** argv){

    ros::init(argc, argv, "talker");   //Inicia el nodo llamado
    ros::NodeHandle n("~");

    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
    // nombre del OBJETO publicador, uso del objeto NodeHandle, metodo "advertise" (ANUNCIAR),
    // <tipo de msg::nombre del mensaje>(nombre de topico publicador , tamano de cola)
    ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);

    ros::Rate loop_rate(10);
    int count = 0;
    ROS_INFO("............");

    while (ros::ok())
    {
        // Creando mensaje String
        std_msgs::String msg;

        std::stringstream ss;
        ss << "hola mundo" << count;
        msg.data = ss.str();
        ROS_INFO("%s", msg.data.c_str());

        chatter_pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }
    

    return 0;
}