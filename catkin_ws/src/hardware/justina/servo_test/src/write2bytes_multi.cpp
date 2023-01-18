#include "dynamixel_sdk/dynamixel_sdk.h"
#include "ros/ros.h"
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>

int main(int argc, char **argv)
{
    ros::init(argc, argv,"write2byte");
    ros::NodeHandle node("~");
    int addr,value,baudrate;
    std::vector<int> ids;
    std::string ids_string;
    std::string port;

    if(!node.hasParam("ids")){
        std::cout<<"missing servo IDs"<<std::endl;
        return -1;
    }
    if(!node.hasParam("addr")){
        std::cout<<"missing address"<<std::endl;
        return -1;
    }
    if(!node.hasParam("value")){
        std::cout<<"missing value"<<std::endl;
        return -1;
    }

    if(!node.getParam("ids",ids_string )){
        std::cout<<"Invalid servo IDs"<<std::endl;
        return -1;
    }
    if(!node.getParam("addr",addr)){
        std::cout<<"Invalid address"<<std::endl;
        return -1;
    }
    if(!node.getParam("value",value)){
        std::cout<<"Invalid value"<<std::endl;
        return -1;
    }

    std::vector<std::string> parts;
    boost::split(parts, ids_string, boost::is_any_of(" ,\t\r\n"), boost::token_compress_on);
    for(size_t i=0; i < parts.size(); i++)
    {
        std::stringstream ss(parts[i]);
        int temp_id;
        if(!(ss >> temp_id))
        {
            std::cout<<"Invalid servo IDs"<<std::endl;
            return -1;
        }
        else
            ids.push_back(temp_id);
    }

    //Check if param exist. Otherwise, use the default values
    node.param("baudrate", baudrate,1000000);
    node.param<std::string>("port",port,"/dev/ttyUSB0");


    //Set port, select protocol and set baudrate
    dynamixel::PortHandler   *portHandler   = dynamixel::PortHandler::getPortHandler(port.c_str());
    dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(1.0);    
    portHandler->setBaudRate(baudrate);

    uint8_t  dxl_error_write        = 0;
    int      dxl_comm_result = COMM_TX_FAIL;
    uint16_t info;

    ros::Time start_time = ros::Time::now();
    for(int i=0; i<ids.size(); i++)
    {
        //Write value of 2 bytes
        dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, ids[i], addr,value, &dxl_error_write);

        if(dxl_comm_result != COMM_SUCCESS)
        {
            std::cout<<"Comunication error"<<std::endl;
            return -1;
        }
        if(dxl_error_write & 0x01)
        {
            std::cout<<"Input voltage error"<<std::endl;     
        }
        if(dxl_error_write & 0x02)
        {
            std::cout<<"Angle limit error"<<std::endl;
        }
        if(dxl_error_write & 0x04)
        {
            std::cout<<"Overheating error"<<std::endl;     
        }
        if(dxl_error_write & 0x08)
        {
            std::cout<<"Range error"<<std::endl;     
        }
        if(dxl_error_write & 0x10)
        {
            std::cout<<"CheckSum error"<<std::endl;     
        }
        if(dxl_error_write & 0x20)
        {
            std::cout<<"Overload error"<<std::endl;     
        }
        if(dxl_error_write & 0x40)
        {
            std::cout<<"Instruction error"<<std::endl;     
        }
        std::cout<<"id: "<<ids[i]<<"\taddress: "<<addr<<"\tvalue: "<<value<<"\tBaudRate: "<<baudrate<<"\tPort: "<<port
                 <<"\tError code: "<<int(dxl_error_write)<<std::endl;
    }
    double millisecs = 1000*(ros::Time::now() - start_time).toSec();
    std::cout << "Communication time: " << millisecs << " milliseconds." << std::endl;
    
    portHandler->closePort();
    return 0;
}
