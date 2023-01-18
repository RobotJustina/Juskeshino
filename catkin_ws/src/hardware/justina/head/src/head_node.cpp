#include "dynamixel_sdk/dynamixel_sdk.h"
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include "sensor_msgs/JointState.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"

#define MX_CURRENT_POSITION 36
#define MX_GOAL_POSITION 30
#define MX_MOVING_SPEED 32
#define MX_TORQUE_ENABLE 24
#define MX_BITS_PER_RADIAN 651.739491961 //=4095/360*180/PI

#define SM_INIT                      10
#define SM_WRITE_HEAD_POSITION       20 
#define SM_START_TRAJECTORY          45
#define SM_WRITE_TRAJECTORY_POINT    50
#define SM_WAIT_FOR_GOAL_REACHED     55 
#define SM_FINISH_TASK               60

std::string prompt;
std::vector<int> servo_ids;        
std::vector<int> servo_zeros;      
std::vector<int> servo_directions; 
std::vector<int> goal_pose_bits;
std::vector<std::vector<int> >   goal_trajectory_bits;
trajectory_msgs::JointTrajectory goal_trajectory;
bool new_pose       = false;
bool new_trajectory     = false;

std::vector<double> positions_bits_to_radians(std::vector<int>& positions_bits, std::vector<int>& centers, std::vector<int>& directions)
{
    std::vector<double> positions_radians;
    positions_radians.resize(positions_bits.size());
    for(int i=0; i<positions_bits.size(); i++)
        positions_radians[i] = directions[i]*(positions_bits[i] - centers[i])/MX_BITS_PER_RADIAN;
    return positions_radians;
}

std::vector<int> positions_radians_to_bits(const std::vector<double>& positions_rads, std::vector<int>& centers, std::vector<int>& directions)
{
    std::vector<int> positions_bits;
    positions_bits.resize(positions_rads.size());
    for(int i=0; i<positions_rads.size(); i++)
        positions_bits[i] = directions[i]*positions_rads[i]*MX_BITS_PER_RADIAN + centers[i];
    return positions_bits;
}

void callback_goal_pose(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    goal_pose_bits = positions_radians_to_bits(msg->data, servo_zeros, servo_directions);
    new_pose = true;
}

void callback_q_trajectory(const trajectory_msgs::JointTrajectory::ConstPtr& msg)
{
    std::cout << prompt << "Received new goal trajectory with " << msg->points.size() << " points." << std::endl;
    goal_trajectory = *msg;
    goal_trajectory_bits.resize(msg->points.size());
    for(int i=0; i< msg->points.size(); i++)
        goal_trajectory_bits[i] = positions_radians_to_bits(msg->points[i].positions, servo_zeros, servo_directions);
    new_trajectory = true;
}

bool get_current_position_bits(dynamixel::GroupBulkRead& groupBulkRead, std::vector<int>& ids, std::vector<int>& current_positions)
{
    if(groupBulkRead.txRxPacket() != COMM_SUCCESS)
        return false;
    for(int i=0; i<ids.size(); i++)
        if(groupBulkRead.isAvailable(ids[i], MX_CURRENT_POSITION, 2))
            current_positions[i] = groupBulkRead.getData(ids[i], MX_CURRENT_POSITION, 2);
        else
            return false;
    return true;
}

bool write_goal_position_bits(dynamixel::PortHandler* port, dynamixel::PacketHandler* packet, std::vector<int>& ids, std::vector<int> positions)
{
    for(int i=0; i < ids.size(); i++)
        if(packet->write2ByteTxOnly(port, ids[i], MX_GOAL_POSITION, positions[i]) != COMM_SUCCESS)
            return false;
    return true;
}

bool write_moving_speed_bits(dynamixel::PortHandler* port, dynamixel::PacketHandler* packet, std::vector<int>& ids, int moving_speed)
{
    for(int i=0; i < ids.size(); i++)
        if(packet->write2ByteTxOnly(port, ids[i], MX_MOVING_SPEED, moving_speed) != COMM_SUCCESS)
            return false;
    return true;
}

bool on_shutting_down(dynamixel::PortHandler* port, dynamixel::PacketHandler* packet, std::vector<int>& ids, std::vector<int>& zeros)
{
    for(int i=0; i < ids.size(); i++)
        packet->write1ByteTxOnly(port, ids[i], MX_TORQUE_ENABLE, 0);
    return true;
}

int main(int argc, char **argv)
{
    bool correct_params = true;
    int baudrate = 1000000;
    bool torque_enable = false;
    int max_speed;
    std::string port_name;
    std::vector<std::string> joint_names;
    
    ros::init(argc, argv, "head_node");    
    ros::NodeHandle n("~");
    prompt = ros::this_node::getName() + ".->";
    std::cout << "INTIALIZING " << ros::this_node::getName() << " NODE BY MARCOSOFT..." << std::endl;
    if(!n.getParam("port", port_name))
    {
        std::cout<<prompt<<"Missing port name. Specifying a port with param 'port' is mandatory."<<std::endl;
        correct_params = false;
    }
    if(!n.getParam("baudrate", baudrate))
    {
        std::cout<<prompt<<"Missing port baudrate. Specifying a port with param 'baudrate' is mandatory."<<std::endl;
        correct_params = false;
    }
    if(!n.getParam("servo_ids",servo_ids))
    {
        std::cout<<prompt<<"Missing servo IDs. Specifying servo IDs with param 'servo_ids' is mandatory."<<std::endl;
        correct_params = false;
    }
    if(!n.getParam("servo_zeros",servo_zeros))
    {
        std::cout<<prompt<<"Missing servo zeros. Specifying servo centers with param 'servo_zeros' is mandatory."<<std::endl;
        correct_params = false;
    }
    if(!n.getParam("servo_directions",servo_directions))
    {
        std::cout<<prompt<<"Missing directions. Specifying moving directions with param 'servo_directions' is mandatory."<<std::endl;
        correct_params = false;
    }
    if(!n.getParam("joint_names", joint_names))
    {
        std::cout<<prompt<<"Missing joint names. Specifying joint names with param 'joint_names' is mandatory."<<std::endl;
        correct_params = false;
    }
    if(!correct_params)
        return -1;
    n.param("torque_enable", torque_enable, true);
    n.param("max_speed"    , max_speed    , 150);
    
    std::cout<<prompt << "Servo IDs: ";
    for(size_t i=0; i<servo_ids.size(); i++) std::cout << servo_ids[i] << "   ";
    std::cout << std::endl;
    std::cout<<prompt << "Servo zeros: ";
    for(size_t i=0; i<servo_zeros.size(); i++) std::cout << servo_zeros[i] << "   ";
    std::cout << std::endl;
    std::cout<<prompt << "Servo directions: ";
    for(size_t i=0; i<servo_directions.size(); i++) std::cout << servo_directions[i] << "   ";
    std::cout << std::endl;
    std::cout<<prompt << "Joint names: ";
    for(size_t i=0; i<joint_names.size(); i++) std::cout << joint_names[i] << "   ";
    std::cout << std::endl;
    std::cout<<prompt << "Torque enable: " << (torque_enable? "True" : "False") << std::endl;
    std::cout<<prompt << "Max moving speed: " << max_speed << std::endl;
    std::cout<<prompt << "Trying to open port " << port_name << " at " << baudrate << std::endl;

    //Set port, select protocol, set baudrate and create objects for bulk reading and writing
    dynamixel::PortHandler   *portHandler   = dynamixel::PortHandler::getPortHandler(port_name.c_str());
    dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(1.0);
    dynamixel::GroupBulkRead groupBulkRead(portHandler, packetHandler);
    portHandler->setBaudRate(baudrate);
    //Setting parameters for bulk reading current head position. 
    for(int i=0; i<servo_ids.size(); i++)
        if(!groupBulkRead.addParam(servo_ids[i], MX_CURRENT_POSITION, 2))
        {
            std::cout<<prompt << "Cannot add bulk read param for id=" << servo_ids[i] << std::endl;
            return -1;
        }

    //Reading startup servo positions and setting them as a goal position
    std::vector<int>  current_position_bits;
    current_position_bits.resize(servo_ids.size());
    std::cout<<prompt << "Trying to get initial servo positions..." << std::endl;
    if(!get_current_position_bits(groupBulkRead, servo_ids, current_position_bits))
    {
        std::cout<<prompt << "Cannot get head initial position..." << std::endl;
        return -1;
    }
    else
    {
        std::cout<<prompt << "Initial head position: ";
        for(int i=0; i<current_position_bits.size(); i++) std::cout << current_position_bits[i] << "  ";
        std::cout << std::endl;
    }
    goal_pose_bits.resize(servo_ids.size());
    for(int i=0; i< servo_ids.size(); i++) goal_pose_bits[i] = current_position_bits[i];
    //If torque is enabled, send current position as servo goal position
    if(torque_enable)
        if(!write_goal_position_bits(portHandler, packetHandler, servo_ids, goal_pose_bits))
        {
            std::cout << prompt << "Cannot set initial head position as goal position" << std::endl;
            return -1;
        }
    if(!write_moving_speed_bits(portHandler, packetHandler, servo_ids, max_speed))
    {
        std::cout << prompt << "Cannot write moving speed to head servos" << std::endl;
        return -1;
    }

    ros::Subscriber sub_goal_pose      = n.subscribe("/hardware/head/goal_pose"     , 1, callback_goal_pose     );
    ros::Subscriber sub_q_trajectory   = n.subscribe("/manipulation/hd_q_trajectory"  , 1, callback_q_trajectory  );
    ros::Publisher pub_joint_state     = n.advertise<sensor_msgs::JointState>("/joint_states", 1);
    ros::Publisher pub_current_pose    = n.advertise<std_msgs::Float64MultiArray>("/hardware/head/current_pose", 1);
    ros::Publisher pub_battery         = n.advertise<std_msgs::Float64>("/hardware/robot_state/head_battery", 1);
    ros::Publisher pub_goal_reached    = n.advertise<std_msgs::Bool>("/manipulation/head/goal_reached", 1);
    ros::Rate rate(40);
    sensor_msgs::JointState joint_states;
    std_msgs::Float64MultiArray msg_current_pose;
    joint_states.name.insert(joint_states.name.end(), joint_names.begin(), joint_names.end());
    joint_states.position = positions_bits_to_radians(current_position_bits, servo_zeros, servo_directions);
    msg_current_pose.data.resize(servo_ids.size());

    int state = SM_INIT;
    int trajectory_idx = 0;
    ros::Time start_time;
    while(ros::ok())
    {
        switch(state)
        {
        case SM_INIT:
            if(new_pose)
                state = SM_WRITE_HEAD_POSITION;
            else if(new_trajectory)
                state = SM_START_TRAJECTORY;
            break;

        case SM_WRITE_HEAD_POSITION:
            new_pose = false;
            if(!write_goal_position_bits(portHandler, packetHandler, servo_ids, goal_pose_bits))
                std::cout << prompt << "Cannot write goal position to head servos" << std::endl;
            state = SM_WAIT_FOR_GOAL_REACHED;
            break;

        case SM_START_TRAJECTORY:
            new_trajectory = false;
            std::cout << prompt << "Starting execution of joint trajectory" << std::endl;
            trajectory_idx = 0;
            start_time = ros::Time::now();
            state = SM_WRITE_TRAJECTORY_POINT;
            break;

        case SM_WRITE_TRAJECTORY_POINT:
            if((ros::Time::now() - start_time) > goal_trajectory.points[trajectory_idx].time_from_start)
            {                
                if(!write_goal_position_bits(portHandler, packetHandler, servo_ids, goal_trajectory_bits[trajectory_idx]))
                    std::cout << prompt << "Cannot write trajectory point to head servos" << std::endl;
                if(++trajectory_idx >=goal_trajectory.points.size())
                    state = SM_FINISH_TASK;
            }
            break;

        case SM_WAIT_FOR_GOAL_REACHED:
            state = SM_FINISH_TASK;
            break;

        case SM_FINISH_TASK:
            std::cout << prompt << "Task finished. " << std::endl;
            state = SM_INIT;
            break;
        }

        //Get current servo position and publish the corresponding topics 
        if(!get_current_position_bits(groupBulkRead, servo_ids, current_position_bits))
            std::cout<<prompt << "Cannot get head current position..." << std::endl;
        joint_states.position = positions_bits_to_radians(current_position_bits, servo_zeros, servo_directions);
        joint_states.header.stamp = ros::Time::now();
        msg_current_pose.data = joint_states.position;
        pub_joint_state.publish(joint_states);
        pub_current_pose.publish(msg_current_pose);
        
        ros::spinOnce();
        rate.sleep();
    }
    on_shutting_down(portHandler, packetHandler, servo_ids, servo_zeros);
}
