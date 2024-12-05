
import time
import math
from roboclaw_3 import Roboclaw


rc1 = Roboclaw("/dev/justinaRC30",115200)
rc2 = Roboclaw("/dev/justinaRC15",115200)
last_enc_left = 0
last_enc_right = 0
last_enc_front = 0
last_enc_rear = 0

TICKS_PER_METER = 365063.24
BASE_WIDTH = 0.5


def read_delta_encoders():
    global last_enc_left, last_enc_right, last_enc_front, last_enc_rear
    address = 0x80
    enc_left = rc1.ReadEncM1(address)
    enc_right = rc1.ReadEncM2(address)
    enc_rear = rc2.ReadEncM2(address)
    enc_front = rc2.ReadEncM1(address)
	
    if enc_left[0] == 0 or enc_right[0] == 0 or enc_rear[0] == 0 or enc_front[0] == 0:
        print("Error while trying to read encoders")
        return [0,0,0,0]
 
    delta_left = enc_left[1] - last_enc_left
    delta_right = enc_right[1] - last_enc_right
    delta_front = enc_front[1] - last_enc_front
    delta_rear = enc_rear[1] - last_enc_rear

    last_enc_left = enc_left[1]
    last_enc_right = enc_right[1]
    last_enc_front = enc_front[1]
    last_enc_rear = enc_rear[1]

    return [delta_left, delta_right, -delta_front, -delta_rear]

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


robot_x = 0
robot_y = 0
robot_a = 0


rc1.Open()
rc2.Open()

[delta_left, delta_right, delta_front, delta_rear] = read_delta_encoders()

while(1):
    [delta_left, delta_right, delta_front, delta_rear] = read_delta_encoders()
    robot_x, robot_y, robot_a = calculate_odometry(robot_x, robot_y, robot_a, delta_left, delta_right, delta_front, delta_rear)
    print([robot_x, robot_y, robot_a])
    time.sleep(0.1)



