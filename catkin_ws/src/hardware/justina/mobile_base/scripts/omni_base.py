#!/usr/bin/env python
import rospy
import tf
import math
from geometry_msgs.msg import Twist
from roboclaw_3 import Roboclaw

TICKS_PER_METER = 365063.24
BASE_WIDTH = 0.5
SPEED_GAIN = 127.0/1.5  #THIS IS A TEMP VALUE. THIS GAIN IS USED TO MAP FROM METERS-PER-SECOND TO [0-127]
MAX_SPEED = 1.5  #METERS PER SECOND

def read_delta_encoders(last_readings, rc_frontal, rc_lateral):
    last_left, last_right, last_front, last_rear = last_readings
    address = 0x80
    enc_left  = rc_frontal.ReadEncM1(address)
    enc_right = rc_frontal.ReadEncM2(address)
    enc_rear  = rc_lateral.ReadEncM2(address)
    enc_front = rc_lateral.ReadEncM1(address)
	
    if enc_left[0] == 0 or enc_right[0] == 0 or enc_rear[0] == 0 or enc_front[0] == 0:
        print("Error while trying to read encoders")
        return [0,0,0,0]
 
    delta_left = enc_left[1]   - last_left
    delta_right = enc_right[1] - last_right
    delta_front = enc_front[1] - last_front
    delta_rear = enc_rear[1]   - last_rear

    last_left  = enc_left[1]
    last_right = enc_right[1]
    last_front = enc_front[1]
    last_rear  = enc_rear[1]
    return [delta_left, delta_right, -delta_front, -delta_rear, [last_left, last_right, last_front, last_rear]]

def calculate_odometry(robot_x, robot_y, robot_a, delta_left, delta_right, delta_front, delta_rear):
    dist_left  = delta_left  / TICKS_PER_METER
    dist_right = delta_right / TICKS_PER_METER
    dist_front = delta_front / TICKS_PER_METER
    dist_rear  = delta_rear  / TICKS_PER_METER

    delta_theta = (dist_right - dist_left + dist_front - dist_rear)/BASE_WIDTH / 2.0
    if abs(delta_theta) >= 0.00001:
        rg_x = (dist_right + dist_left)  / (2.0 * delta_theta)
        rg_y = (dist_rear  + dist_front) / (2.0 * delta_theta)
        delta_x = rg_x * math.sin(delta_theta)       + rg_y * (1 - math.cos(delta_theta))
        delta_y = rg_x * (1 - math.cos(delta_theta)) + rg_y * math.sin(delta_theta)
    else: 
        delta_x = (dist_left + dist_right) / 2
        delta_y = (dist_front + dist_rear) / 2

    robot_x += delta_x * math.cos(robot_a) - delta_y * math.sin(robot_a)
    robot_y += delta_x * math.sin(robot_a) + delta_y * math.cos(robot_a) 
    robot_a  = ((robot_a + delta_theta) + math.pi) % (2*math.pi) - math.pi

    return [robot_x, robot_y, robot_a]

def broadcast_odom(robot_x, robot_y, robot_a, br):
    quat = tf.transformations.quaternion_from_euler(0, 0, robot_a)
    current_time = rospy.Time.now()
    br.sendTransform((robot_x, robot_y, 0), quat, rospy.Time.now(), "base_link", "odom")

def check_speed_ranges(s_left, s_right, s_front, s_rear): #speeds: left, right, front and rear
    max_value = max(max(abs(s_left), abs(s_right)), max(abs(s_front), abs(s_rear)))
    if max_value > MAX_SPEED:
        print("MobileBase.->Warning! motor speeds should not be greater than" + str(MAX_SPEED) + ". Normalized speeds used instead")
        s_left  /= max_value;
        s_right /= max_value;
        s_front /= max_value;
        s_rear  /= max_value;
    return (s_left, s_right, s_front, s_rear);

def callback_cmd_vel(msg):
    global speed_left, speed_right, speed_front, speed_rear, new_cmd_vel;
    speed_left  = msg.linear.x - msg.angular.z * BASE_WIDTH/2.0
    speed_right = msg.linear.x + msg.angular.z * BASE_WIDTH/2.0
    speed_front = msg.linear.y + msg.angular.z * BASE_WIDTH/2.0
    speed_rear  = msg.linear.y - msg.angular.z * BASE_WIDTH/2.0
    (speed_left, speed_right, speed_front, speed_rear) = check_speed_ranges(speed_left, speed_right, speed_front, speed_rear)
    new_cmd_vel = True;

def main():
    global speed_left, speed_right, speed_front, speed_rear, new_cmd_vel
    print("INITIALIZING MOBILE BASE BY MARCOSOFT...")
    print("PLEASE DON'T TRY TO OPERATE JUSTINA IF YOU ARE NOT QUALIFIED ENOUGH.")
    rospy.init_node("mobile_base")

    port_frontal = "/dev/justinaRC30"
    port_lateral = "/dev/justinaRC15"
    baud_frontal = 115200
    baud_lateral = 115200
    
    if rospy.has_param('~port1'):
        port_frontal = rospy.get_param('~port1')
    if rospy.has_param('~port2'):
        port_lateral = rospy.get_param('~port2')
    if rospy.has_param('~baud1'):
        baud_frontal = rospy.get_param('~baud1')
    if rospy.has_param('~baud2'):
        baud_lateral = rospy.get_param('~baud2')

    rospy.Subscriber("/hardware/mobile_base/cmd_vel", Twist, callback_cmd_vel)
    br = tf.TransformBroadcaster()
    loop = rospy.Rate(10)
        
    rc_frontal = Roboclaw(port_frontal,115200)
    rc_lateral = Roboclaw(port_lateral,115200)
    rc_frontal.Open()
    rc_lateral.Open()
    version_frontal = rc_frontal.ReadVersion(0x80)
    version_lateral = rc_lateral.ReadVersion(0x80)
    if not version_frontal[0] or not version_lateral[0]:
        print ("MobileBase.->ERROR!!! Cannot read Roboclaw versions. Exiting program")
        exit(-1)

    last_readings = [0,0,0,0]
    robot_x, robot_y, robot_a = 0,0,0
    speed_left, speed_right, speed_front, speed_rear = 0,0,0,0
    new_cmd_vel = False
    no_new_cmd_vel_counter = 0
    read_delta_encoders(last_readings, rc_frontal, rc_lateral)
    while not rospy.is_shutdown():
        [s_left, s_right, s_front, s_rear, last_readings] = read_delta_encoders(last_readings, rc_frontal, rc_lateral)
        robot_x, robot_y, robot_a = calculate_odometry(robot_x, robot_y, robot_a, s_left, s_right, s_front, s_rear)
        broadcast_odom(robot_x, robot_y, robot_a, br)
        if new_cmd_vel:
            new_cmd_vel = False
            no_new_cmd_vel_counter = 0
        else:
            no_new_cmd_vel_counter = min(no_new_cmd_vel_counter + 1, 5)
        if no_new_cmd_vel_counter >= 5:
            speed_left, speed_right, speed_front, speed_rear = 0,0,0,0

        motor_left  = int(speed_left *SPEED_GAIN)
        motor_right = int(speed_right*SPEED_GAIN)
        motor_front = int(-speed_front*SPEED_GAIN)
        motor_rear  = int(-speed_rear *SPEED_GAIN)

        #print([motor_left, motor_right, motor_front, motor_rear])

        if motor_left > 0:
            rc_frontal.ForwardM1(0x80, motor_left)
        else:
            rc_frontal.BackwardM1(0x80, -motor_left)
        if motor_right > 0:
            rc_frontal.ForwardM2(0x80, motor_right)
        else:
            rc_frontal.BackwardM2(0x80, -motor_right)

        if motor_front > 0:
            rc_lateral.ForwardM1(0x80, motor_front)
        else:
            rc_lateral.BackwardM1(0x80, -motor_front)
        if motor_rear > 0:
            rc_lateral.ForwardM2(0x80, motor_rear)
        else:
            rc_lateral.BackwardM2(0x80, -motor_rear)
        loop.sleep()

if __name__ == "__main__":
    main()
