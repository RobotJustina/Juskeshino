#!/usr/bin/env python
import rospy
import math
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray


def callbackJoy(msg):
    global speedX
    global speedY
    global yaw
    global panPos
    global tiltPos


    ### Control of head with left Stick 
    leftStickX = msg.axes[0]
    leftStickY = msg.axes[1]
    
    magnitudLeft = math.sqrt(leftStickX*leftStickX + leftStickY*leftStickY)
    if magnitudLeft > 0.1:
        panPos = leftStickX
        tiltPos = leftStickY
    else:
        panPos = 0
        tiltPos = 0
    ### Control of mobile-base with right Stick
    rightStickX = msg.axes[3]
    rightStickY = msg.axes[4]
    magnitudRight = math.sqrt(rightStickX*rightStickX + rightStickY*rightStickY)
    if magnitudRight > 0.1:
        speedX = 0.45*rightStickY
        yaw = 0.8*rightStickX
    else:
        speedX = 0
        yaw = 0

    if msg.axes[2] == 1.0 and msg.axes[5] != 1.0:
        leftTigger = msg.axes[2]
        rightTigger = -(msg.axes[5] - 1) 

    elif msg.axes[5] == 1.0 and msg.axes[2] != 1.0:
        leftTigger = -(msg.axes[2] - 1)
        rightTigger = msg.axes[5] 

    elif msg.axes[5] == 1.0 and msg.axes[2] == 1.0:
        leftTigger = 0
        rightTigger = 0
    else:
        leftTigger = -(msg.axes[2] - 1)
        rightTigger = -(msg.axes[5] - 1) 


    #print "leftTigger: " + str(leftTigger) +" rightTigger: " + str(rightTigger)

    ### Tigger button for speed y componente

    magnitudTiggerDiference = math.sqrt((leftTigger*leftTigger) + (rightTigger*rightTigger))
    #print "diference: " + str(magnitudTiggerDiference)
    if magnitudTiggerDiference > 0.15:
        speedY = (leftTigger - rightTigger)/4
    else:
        speedY = 0

def main():
    
    global speedX
    global speedY
    global yaw
    global panPos
    global tiltPos
    speedY = 0
    speedX = 0
    yaw = 0
    panPos = 0
    tiltPos = 0

    msgTwist = Twist()
    msgHeadPos = Float64MultiArray()

    #print("INITIALIZING JOYSTICK TELEOP BY MARCOSOFT... :)")
    rospy.init_node("dualshock_teleop")

    rospy.Subscriber("/hardware/joy", Joy, callbackJoy)
    pubTwist = rospy.Publisher("/hardware/mobile_base/cmd_vel", Twist, queue_size =1)
    pubHeadPos = rospy.Publisher("/hardware/head/goal_pose", Float64MultiArray, queue_size=1)


    
    loop = rospy.Rate(10)
    while not rospy.is_shutdown():
        if math.fabs(speedX) > 0 or math.fabs(speedY) > 0 or math.fabs(yaw) > 0:
            msgTwist.linear.x = speedX
            msgTwist.linear.y = speedY/2.0
            msgTwist.linear.z = 0
            msgTwist.angular.z = yaw
            print("x: " + str(msgTwist.linear.x) + "  y: " + str(msgTwist.linear.y) + " yaw: " + str(msgTwist.angular.z))
            pubTwist.publish(msgTwist)
        if math.fabs(panPos) > 0 or math.fabs(tiltPos) > 0:
            msgHeadPos.data = [panPos, tiltPos]
            pubHeadPos.publish(msgHeadPos)
            
        loop.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass