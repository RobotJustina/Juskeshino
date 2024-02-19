#include <ros/ros.h>
#include <std_msgs/String.h>
#include <face_recog/RecognizeFace.h>
#include <sensor_msgs/Image.h>
#include <vector>
#include <string>


#define RATE 30

ros::ServiceClient  analyzeFaceSrv;
ros::Subscriber     sub_image;
ros::Subscriber     sub_name;

ros::Publisher      pub_face_res;


sensor_msgs::Image  face_image;
std::string         face_name;


//Caller for analyze face service 
std::string service_caller(const sensor_msgs::Image::ConstPtr& img){

    face_recog::RecognizeFace srv;
    std_msgs::String str_msg;
    str_msg.data = "analyze ";
    srv.request.Ids.ids.push_back(str_msg);
    srv.request.in.image_msgs.push_back(*img);

    if(analyzeFaceSrv.call(srv))
    {
        std::cout<<"Analyze face background service called"<<std::endl;
        //srv.response.
        //std::vector<std::string> results;
        int i = 0;
        std::string pronoun = "she";
        std::string gender;
        std::string age;
        std::string state;
        std::string race;
        for (const auto& chars : srv.response.Ids.ids){

            std::cout<<"Data: "<<chars.data<<std::endl;
            //results.push_back(chars.data);
            switch(i){
            case 0:
                gender = chars.data;
                i++;
                break;
            case 1:
                race = chars.data;
                i++;
                break;
            case 2:
                state = chars.data;
                i++;
                break;
            case 3:
                age = chars.data;
                i++;
                break;
            default:
                break;
            }
        }
        if (gender == "Man")
                pronoun = "he";
	std::string takeshi_line = "";
	if (gender != "NO_FACE"){
        takeshi_line = face_name+ " has arrived... " +pronoun+  " is a " + gender +"... I believe " +pronoun; 
        takeshi_line += " is  around " +age+ " years old... I would say " +pronoun+ " is a bit " +state;
        takeshi_line += ". I might guess "+ pronoun+ " is of " +race+ " descent.";
	}
        return takeshi_line;

        /*if (results.size() > 1){
            std::string pronoun = "she";
            std::string gender  = results[0];
            std::string age     = results[-1];
            std::string state   = results[2];
            std::string race    = results[1];
            if (gender == "Man")
                pronoun = "he";
            std::string takeshi_line = face_name+ " has arrived... " +pronoun+  " is a " + gender +"... I believe " +pronoun; 
            takeshi_line += " is  around " +age+ " years old... I would say he is a bit " +state;
            takeshi_line += ". And I might guess "+ pronoun+ " is of " +race+ "descent.";
            return takeshi_line;
        }
        else
            return "1";*/
        
    }
    else
        return "2";
}


void img_msg_Callback(const sensor_msgs::Image::ConstPtr& msg)
{
    std_msgs::String result;
    //addDescription(face_name, description);
    result.data = service_caller(msg);
    pub_face_res.publish(result);

}
void name_msg_Callback(const std_msgs::String::ConstPtr& msg)
{
    face_name = msg->data;
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "analyze_face_node");
    ros::NodeHandle n;

    // Create a subscriber to receive the image
    //sub_image   = n.subscribe("/image_to_analyze", 10, img_msg_Callback);
    sub_image   = n.subscribe<sensor_msgs::Image>("/image_to_analyze", 10, img_msg_Callback);
    sub_name    = n.subscribe<std_msgs::String>("/name_face", 10, name_msg_Callback);


    // Create a service client to call the analyze_face service
    analyzeFaceSrv = n.serviceClient<face_recog::RecognizeFace>("/analyze_face");

    // Create a publisher to publish the analysis results
    //packPath = ros::package::getPath("config_files");
    pub_face_res= n.advertise<std_msgs::String>("/analyze_result",10);

    ros::Rate loop(RATE);
    while(ros::ok())
    {
        ros::spinOnce();
        loop.sleep();
    }


    return 0;
}
