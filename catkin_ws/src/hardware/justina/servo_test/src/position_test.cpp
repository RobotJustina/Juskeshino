#include "dynamixel_sdk/dynamixel_sdk.h"
#include "ros/ros.h"

#define ID_CM730 200
#define ADDR_CM730_WAKE_UP 24

int main(int argc, char **argv){
    ros::init(argc, argv, "position_test");
    ros::NodeHandle node("~");
    int baudrate;
    int id;
    std::string port;

    if(!node.hasParam("id")){
        std::cout << "missing servo id" << std::endl;
        return -1;
    }
    if(!node.getParam("id",id)){
        std::cout << "Invalid servo id" << std::endl;
        return -1;
    }

    //Default baudrate and port
    node.param("baudrate", baudrate, 1000000);
    node.param<std::string>("port",port,"/dev/ttyUSB0");

    //Set port, select protocol and set baudrate
    dynamixel::PortHandler      *portHandler    = dynamixel::PortHandler::getPortHandler(port.c_str());
    dynamixel::PacketHandler    *packetHandler  = dynamixel::PacketHandler::getPacketHandler(1.0);
    portHandler->setBaudRate(baudrate);

    uint8_t dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;
    uint16_t info;
    int present_position_addr=36;

    /*---------------------------------------
    |      WAKE UP CM730 (ADDR 24)          |
    -----------------------------------------*/
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, 
                                                    ID_CM730, 
                                                    ADDR_CM730_WAKE_UP , 
                                                    1, 
                                                    &dxl_error);
    if(dxl_comm_result != COMM_SUCCESS)
        std::cout << "CM730.->Commnunication problem while turning on dynamixel power." << std::endl;
    if(dxl_error != 0){
        std::cout << "CM730.->Status error after turning on dynamixel power: " << int(dxl_error) << std::endl;
        return -1;
    }
    std::cout << "\t Servo ID: " << id << std::endl;

    while(ros::ok()){
        /*---------------------------------------
        |      READ ID POSITION (ADDR 36)        |
        -----------------------------------------*/
        dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, 
                                                       id,
                                                       present_position_addr,
                                                       &info,
                                                       &dxl_error);
        if(dxl_comm_result != COMM_SUCCESS){
            std::cout << "Communication Error" << std::endl;
            return -1;
        }
        std::cout << "\r\t Position: " << int(info) << "  " << std::flush;

    }
    std::cout << std::endl;
    portHandler->closePort();
    return 0;
}