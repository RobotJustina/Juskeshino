#include "dynamixel_sdk/dynamixel_sdk.h"
#include "ros/ros.h"
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "read1byte");
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
    if(!node.getParam("ids",ids_string )){
        std::cout<<"Invalid servo IDs"<<std::endl;
        return -1;
    }
    if(!node.getParam("addr",addr)){
        std::cout<<"Invalid address"<<std::endl;
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
    dynamixel::GroupBulkRead groupBulkRead(portHandler, packetHandler);
    portHandler->setBaudRate(baudrate);

    int      dxl_comm_result = COMM_TX_FAIL;
    bool     dxl_addparam_result = false;
    bool     dxl_getdata_result  = false;
    std::vector<int> data;
    data.resize(ids.size());

    //Setting parameters for bulk read. It is assumed we are going to read the same address and number of bytes for all servos
    for(int i=0; i<ids.size(); i++)
    {
        dxl_addparam_result = groupBulkRead.addParam(ids[i], addr, 1);
        if(!dxl_addparam_result)
            std::cout << "Cannot add bulk read param addr=" << addr << "  len=1 for servo id=" << ids[i] << std::endl;
    }

    //Sending bulk read package and getting the results
    ros::Time start_time = ros::Time::now();
    dxl_comm_result = groupBulkRead.txRxPacket();
    if(dxl_comm_result != COMM_SUCCESS)
    {
        std::cout<<"Comunication error while trying to bulk reading." <<std::endl;
        return -1;
    }
    for(int i=0; i<ids.size(); i++)
    {
        if(groupBulkRead.isAvailable(ids[i], addr, 1))
        {
            data[i] = groupBulkRead.getData(ids[i], addr, 1);
            std::cout<<"Data: "<<data[i]<<"\tid: "<<ids[i]<<"\taddress: "<<addr<<"\tBaudRate: "<<baudrate<<"\tPort: "<<port<<std::endl;
        }
        else
            std::cout << "Cannot get data for servo " << ids[i] << " while trying to bulk read" << std::endl;
            
    }
    
    double millisecs = 1000*(ros::Time::now() - start_time).toSec();
    std::cout << "Communication time: " << millisecs << " milliseconds." << std::endl;
    
    portHandler->closePort();

    return 0;
    
}
