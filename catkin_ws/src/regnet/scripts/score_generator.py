#!/usr/bin/env python3

import rospy 
import rospkg 
import random
from gazebo_msgs.msg import ModelState, ContactsState 
from gazebo_msgs.srv import SetModelState
from std_msgs.msg import String, Float64MultiArray
from geometry_msgs.msg import Pose  
from manip_msgs.srv import DataCapture


