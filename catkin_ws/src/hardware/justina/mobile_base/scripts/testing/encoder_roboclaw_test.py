#!/usr/bin/env python
import serial, time, sys, math
import rospy
from std_msgs.msg import Empty
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Twist
from roboclaw_3 import Roboclaw
import tf


def check_encoders(s_left, s_right, s_front, s_rear):




def main(port_name_1, port_name_2):

    #ROS CONNECTION
    rospy.init_node("encoder_test")
    rate = rospy.Rate(30)

    #ROBOCLAW CONNECTION
    rc_frontal  = roboclaw.Roboclaw(port_name_1, 250000);
    rc_lateral  = roboclaw.Roboclaw(port_name_2, 250000);
    rc_address1 = 0x80
    rc_address2 = 0x80
    if rc_frontal.Open() != 1:
        print "MobileBase.-> Cannot open port " + port_name_1;
        return;
    #if rc_lateral.Open() != 1:
    #    print "MobileBase.-> Cannot open port " + port_name_2;
    #    return;
    
    rc_frontal.ResetEncoders(rc_address1);
    #rc_lateral.ResetEncoders(rc_address2);+

    #ROBOCLAW CONFIGURATION CONSTANTS
    m_left_pos_PID  = rc_frontal.ReadM1PositionPID(rc_address1);
    m_right_pos_PID = rc_frontal.ReadM2PositionPID(rc_address1);
    #m_front_pos_PID = rc_lateral.ReadM1PositionPID(rc_address2);
    #m_rear_pos_PID  = rc_lateral.ReadM2PositionPID(rc_address2);
    m_left_vel_PID  = rc_frontal.ReadM1VelocityPID(rc_address1);
    m_right_vel_PID = rc_frontal.ReadM2VelocityPID(rc_address1);
    #m_front_vel_PID = rc_lateral.ReadM1VelocityPID(rc_address2);
    #m_rear_vel_PID  = rc_lateral.ReadM2VelocityPID(rc_address2);
    print "MobileBase.->Position PID constants:  Success   P   I   D   MaxI  Deadzone  MinPos  MaxPos";
    print "MobileBase.->Left Motor: "  + str(m_left_pos_PID);
    print "MobileBase.->Right Motor: " + str(m_right_pos_PID);
    #print "MobileBase.->Front Motor: " + str(m_front_pos_PID);
    #print "MobileBase.->Rear Motor: "  + str(m_rear_pos_PID);
    print "MobileBase.->Velocity PID constants:  Success  P   I   D   QPPS"; #QPPS = speed in ticks/s when motor is at full speed
    print "MobileBase.->Left Motor: "  + str(m_left_vel_PID);
    print "MobileBase.->Right Motor: " + str(m_right_vel_PID);
    #print "MobileBase.->Front Motor: " + str(m_front_vel_PID);
    #print "MobileBase.->Left Motor: "  + str(m_left_vel_PID);
    QPPS_M_LEFT  = m_left_vel_PID[4];
    QPPS_M_RIGHT = m_right_vel_PID[4];
    QPPS_M_FRONT = 1000;#m_front_vel_PID[4];
    QPPS_M_REAR  = 1000;#m_rear_vel_PID[4];
    if QPPS_M_LEFT != QPPS_M_RIGHT or QPPS_M_REAR != QPPS_M_FRONT:
        print "MobileBase.->WARNING: Motors have different max speeds!! Stall condition may occur."
    if rc_frontal.ReadPWMMode(rc_address1)[1] == 1:
        print "MobileBase.->PWM Mode for left and right motors: Sign magnitude"
    else:
        print "MobileBase.->PWM Mode for left and right motors: Locked antiphase"
    #if rc_lateral.ReadPWMMode(rc_address2) == 1:
    #    print "MobileBase.->PWM Mode for front and rear motors: Sign magnitude"
    #else:
    #    print "MobileBase.->PWM Mode for front and rear motors: Locked antiphase"

    #Variable initialization
    M_LR_ACCELERATION = 15000;          #Acceleration for speed control of left and right motors
    M_FR_ACCELERATION = 30000;          #Acceleration for speed control of front and rear motors
    global new_data;
    global speed_left; 
    global speed_right;
    global speed_front;
    global speed_rear;
    speed_left  = 0;    
    speed_right = 0;    
    speed_front = 0;    
    speed_rear  = 0;    
    new_data    = False;
    no_new_data_counter = 10;  #If no new speed data arrive after 5 cycles, motors are stopped.
    robot_x = 0;
    robot_y = 0;
    robot_t = 0;
    encoder_left_last  = 0;
    encoder_right_last = 0;
    encoder_front_last = 0;
    encoder_rear_last  = 0;
    
    while not rospy.is_shutdown():
        if new_data:
            new_data = False;
            no_new_data_counter = 10;
            speed_left  = int(speed_left *16.0/35.0*QPPS_M_LEFT); #Factor 16/35 is due to the different motor reductions
            speed_right = int(speed_right*16.0/35.0*QPPS_M_RIGHT); #Factor 16/35 is due to the different motor reductions
            speed_front = int(speed_front*QPPS_M_FRONT);
            speed_rear  = int(speed_rear *QPPS_M_REAR);
            try:
                rc_frontal.SpeedAccelM1M2(rc_address1, M_LR_ACCELERATION, speed_left, speed_right);
                #rc_lateral.SpeedAccelM1M2(rc_address2, M_FR_ACCELERATION, speed_front, speed_rear);
            except:
                print "MobileBase.->Error while sending speed to left-right Roboclaw";
        else:
            no_new_data_counter -= 1;
            if no_new_data_counter == 0:
                rc_frontal.ForwardM1(rc_address1, 0);
                rc_frontal.ForwardM2(rc_address1, 0);
                #rc_lateral.ForwardM1(rc_address2, 0);
                #rc_lateral.ForwardM2(rc_address2, 0);
            if no_new_data_counter < -1:
                no_new_data_counter = -1;
        encoder_left  = rc_frontal.ReadEncM1(rc_address1)[1];
        encoder_right = rc_frontal.ReadEncM2(rc_address1)[1];
        encoder_front = 0;#rc_lateral.ReadEncM1(rc_address2)[1];
        encoder_rear  = 0;#rc_lateral.ReadEncM2(rc_address2)[1];
        delta_left  = encoder_left  - encoder_left_last;
        delta_right = encoder_right - encoder_right_last;
        delta_front = encoder_front - encoder_front_last;
        delta_rear  = encoder_rear  - encoder_rear_last;
        encoder_left_last  = encoder_left;
        encoder_right_last = encoder_right;
        encoder_front_last = encoder_front;
        encoder_rear_last  = encoder_rear;
        if(math.fabs(delta_left) < 10000 and math.fabs(delta_right) < 10000
               and math.fabs(delta_front) < 10000 and math.fabs(delta_rear) < 10000):
            (robot_x,robot_y,robot_t)=calculate_odometry(robot_x,robot_y,robot_t,delta_left,delta_right,delta_front,delta_rear);
        else:
            print "MobileBase.->Invalid encoder readings.";

        quaternion = tf.transformations.quaternion_from_euler(0, 0, robot_t);
        br.sendTransform((robot_x, robot_y, 0), quaternion, rospy.Time.now(), "base_link", "odom");
        pubBattery.publish(Float32(rc_frontal.ReadMainBatteryVoltage(rc_address1)[1]));
        rate.sleep();

    print "MobileBase.->Stopping motors..."
    rc_frontal.ForwardM1(rc_address1, 0);
    rc_frontal.ForwardM2(rc_address1, 0);
    #rc_lateral.ForwardM1(rc_address2, 0);
    #rc_lateral.ForwardM2(rc_address2, 0);
    
if __name__ == '__main__':
    if "--help" in sys.argv or "-h" in sys.argv:
        print_help();
        sys.exit();
    
    port_1 = "/dev/ttyACM0";
    port_2 = "/dev/ttyACM1";
    if "--port1" in sys.argv:
        port_1 = sys.argv[sys.argv.index("--port1") + 1];
    if "--port2" in sys.argv:
        port_2 = sys.argv[sys.argv.index("--port2") + 1];
    main(port_1, port_2);
