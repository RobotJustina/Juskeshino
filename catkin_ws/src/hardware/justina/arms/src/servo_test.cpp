#ifdef __linux__
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#elif defined(_WIN32) || defined(_WIN64)
#include <conio.h>
#endif

#include <stdlib.h>
#include <stdio.h>
#include "ros/ros.h"

#include "dynamixel_sdk/dynamixel_sdk.h"


#include "std_msgs/Bool.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include "sensor_msgs/JointState.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "tf/transform_broadcaster.h"
#include "iostream"
#include <cmath>

#define MX_CURRENT_POSITION 36
#define MX_GOAL_TORQUE 71
#define MX_TORQUE_ENABLE 24
#define MX_TORQUE_CTL_ENABLE 70
#define MX_BITS_PER_RADIAN 651.739491961 //=4095/360*180/PI
#define MX_BITS_PER_A 222.222222222
#define MX_64_K 1.149
#define MX_106_K 1.408  //for ID:0,1,3
#define MX_CURRENT_VOLTAGE 42

std::string prompt;
std::vector<int> servo_arm_ids;            //Servo IDs for each joint of the arm
std::vector<int> servo_arm_zeros;          //Bits corresponding to the 0 rad value
std::vector<int> servo_arm_directions;     //1 for clockwise, -1 for counter clockwise
std::vector<int> servo_gripper_ids;        //Servo IDs for each joint of the gripper
std::vector<int> servo_gripper_zeros;      //Bits corresponding to the 0 rad value
std::vector<int> servo_gripper_directions; //1 for clockwise, -1 for counter clockwise
std::vector<int> servo_ids;                // = arm_ids + gripper_ids
std::vector<int> servo_zeros;              // = arm zeros  + gripper zeros
std::vector<int> servo_directions;         // = arm directions +  gripper directions
std::vector<int> goal_torque_arm_bits;
std::vector<int> goal_pose_arm_bits;       //***********************************************************
std::vector<int> goal_pose_gripper_bits;
std::vector<std::vector<int> >   goal_trajectory_bits;
trajectory_msgs::JointTrajectory goal_trajectory;

bool new_arm_torque     = false;
bool new_arm_pose       = false;
bool new_trajectory     = false;
bool new_gripper_pose   = false;
bool new_gripper_torque = false;




//#include "dynamixel_sdk.h"                                  // Uses DYNAMIXEL SDK library

// Control table address
#define ADDR_MX_TORQUE_ENABLE           24                  // Control table address is different in Dynamixel model
#define ADDR_MX_GOAL_POSITION           30
#define ADDR_MX_PRESENT_POSITION        36

#define ADDR_MX_GOAL_TORQUE             71
#define ADDR_MX_TORQUE_CTL_ENABLE       70

// Protocol version
#define PROTOCOL_VERSION                1.0                 // See which protocol version is used in the Dynamixel

// Default setting
#define DXL_ID                          5                   // Dynamixel ID: 1
#define BAUDRATE                        1000000
#define DEVICENAME                      "/dev/justinaLeftArm"      // Check which port is being used on your controller
                                                            // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0"

#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
#define DXL_MINIMUM_POSITION_VALUE      100                 // Dynamixel will rotate between this value
#define DXL_MAXIMUM_POSITION_VALUE      4000                // and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
#define DXL_MOVING_STATUS_THRESHOLD     10                  // Dynamixel moving status threshold

#define ESC_ASCII_VALUE                 0x1b


 // Initialize PortHandler instance
  // Set the port path
  // Get methods and members of PortHandlerLinux or PortHandlerWindows
  dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

  // Initialize PacketHandler instance
  // Set the protocol version
  // Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
  dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  int dxl_comm_result = COMM_TX_FAIL;             // Communication result
  int dxl_goal_position[2] = {DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE};         // Goal position

  uint8_t dxl_error = 0;                          // Dynamixel error
  uint16_t dxl_present_position = 0;              // Present position

int getch()
{
#ifdef __linux__
  struct termios oldt, newt;
  int ch;
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  ch = getchar();
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  return ch;
#elif defined(_WIN32) || defined(_WIN64)
  return _getch();
#endif
}

int kbhit(void)
{
#ifdef __linux__
  struct termios oldt, newt;
  int ch;
  int oldf;

  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

  ch = getchar();

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);

  if (ch != EOF)
  {
    ungetc(ch, stdin);
    return 1;
  }

  return 0;
#elif defined(_WIN32) || defined(_WIN64)
  return _kbhit();
#endif
}


//write goal torque in bits in the control table********************************************************
bool write_goal_torque_bits()//dynamixel::PortHandler* port, dynamixel::PacketHandler* packet, std::vector<int>& ids, std::vector<int> torque)
{
  /*
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID, ADDR_MX_GOAL_TORQUE , 2, &dxl_error);
    //rospy.sleep(1);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf(":(");
      //packetHandler->printTxRxResult(dxl_comm_result);
    }
    else if (dxl_error != 0)
    {
        printf(":(");
      //packetHandler->printRxPacketError(dxl_error);
    }
        // Write goal torque
    //dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID, ADDR_MX_GOAL_TORQUE , 0, &dxl_error);
    printf("enviando par");
    */
    
}



int main()
{

  // Open port
  if (portHandler->openPort())
  {
    printf("Succeeded to open the port!\n");
  }
  else
  {
    printf("Failed to open the port!\n");
    printf("Press any key to terminate...\n");
    getch();
    return 0;
  }

  // Set port baudrate
  if (portHandler->setBaudRate(BAUDRATE))
  {
    printf("Succeeded to change the baudrate!\n");
  }
  else
  {
    printf("Failed to change the baudrate!\n");
    printf("Press any key to terminate...\n");
    getch();
    return 0;
  }


  // Enable DXL Torque
  dxl_comm_result = packetHandler->write1ByteTxOnly(portHandler, DXL_ID, ADDR_MX_TORQUE_ENABLE, 1);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    //packetHandler->printTxRxResult(dxl_comm_result);
    printf("No se pudo habilitar par");
  }
  else if (dxl_error != 0)
  {
    printf("No se pudo habilitar par");
  }
  else
  {
    printf("Dynamixel has been successfully connected \n");
  }


  dxl_comm_result = packetHandler->write1ByteTxOnly(portHandler, DXL_ID, ADDR_MX_TORQUE_CTL_ENABLE, 1);
    if (dxl_comm_result != COMM_SUCCESS)
  {
    //packetHandler->printTxRxResult(dxl_comm_result);
    printf("No se pudo habilitar control  por par");
  }
  else if (dxl_error != 0)
  {
    //packetHandler->printRxPacketError(dxl_error);
    printf("No se pudo habilitar control  por par");
  }
  else
  {
    printf("Dynamixel torque ctl has been successfully connected \n");
  }

  while(1)
  {
    printf("Press any key to continue! (or press ESC to quit!)\n");


    //write_goal_torque_bits();

   dxl_comm_result = packetHandler->write2ByteTxOnly(portHandler, DXL_ID, ADDR_MX_GOAL_TORQUE , 2);

    //rospy.sleep(1);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf(":(");
      //packetHandler->printTxRxResult(dxl_comm_result);
    }
    else if (dxl_error != 0)
    {
        printf(":(");
      //packetHandler->printRxPacketError(dxl_error);
    }
        // Write goal torque
    //dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID, ADDR_MX_GOAL_TORQUE , 0, &dxl_error);
    printf("enviando par");

    if (getch() == ESC_ASCII_VALUE)
    break;



  }

  // Disable Dynamixel Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, 5, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    //packetHandler->printTxRxResult(dxl_comm_result);
  }
  else if (dxl_error != 0)
  {
    //packetHandler->printRxPacketError(dxl_error);
  }

  // Close port
  portHandler->closePort();

  return 0;
}
