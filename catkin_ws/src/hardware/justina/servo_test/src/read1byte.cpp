#include "dynamixel_sdk/dynamixel_sdk.h"
#include "ros/ros.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "read1byte");
    ros::NodeHandle node("~");
    int id,addr,value,baudrate;
    std::string port;

    if(!node.hasParam("id")){
        std::cout<<"missing servo id"<<std::endl;
        return -1;
    }
    if(!node.hasParam("addr")){
        std::cout<<"missing address"<<std::endl;
        return -1;
    }
    if(!node.getParam("id",id )){
        std::cout<<"Invalid servo id"<<std::endl;
        return -1;
    }
    if(!node.getParam("addr",addr)){
        std::cout<<"Invalid address"<<std::endl;
        return -1;
    }

    //Default baudrate and port
    node.param("baudrate", baudrate,1000000);
    node.param<std::string>("port",port,"/dev/ttyUSB0");
    
    //Set port, select protocol and set baudrate
    dynamixel::PortHandler   *portHandler   = dynamixel::PortHandler::getPortHandler(port.c_str());
    dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(1.0);
    portHandler->setBaudRate(baudrate);

    uint8_t  dxl_error_read      = 0;
    int      dxl_comm_result = COMM_TX_FAIL;
    uint8_t info;

    //Read value of 1 Byte
    dxl_comm_result = packetHandler->read1ByteTxRx(portHandler, id,addr,&info, &dxl_error_read);
    
    if(dxl_comm_result != COMM_SUCCESS)
    {
        std::cout<<"Comunication error"<<std::endl;
        return -1;
    }

    if(dxl_error_read & 0x01)
    {
        std::cout<<"Input voltage error"<<std::endl;
    }
    if(dxl_error_read & 0x02)
    {
        std::cout<<"Angle limit error"<<std::endl;
    }
    if(dxl_error_read & 0x04)
    {
        std::cout<<"Overheating error"<<std::endl;
    }
    if(dxl_error_read & 0x08)
    {
        std::cout<<"Range error"<<std::endl;
    }
    if(dxl_error_read & 0x10)
    {
        std::cout<<"CheckSum error"<<std::endl;
    }
    if(dxl_error_read & 0x20)
    {
        std::cout<<"Overload error"<<std::endl;
    }
    if(dxl_error_read & 0x40)
    {
        std::cout<<"Instruction error"<<std::endl;
    }

    std::cout<<"Data: "<<int(info)<<"\tid: "<<id<<"\taddress: "<<addr<<"\tBaudRate: "<<baudrate<<"\tPort: "<<port
    <<"\tError code: "<<int(dxl_error_read)<<std::endl;
    
    portHandler->closePort();
    return 0;
}
