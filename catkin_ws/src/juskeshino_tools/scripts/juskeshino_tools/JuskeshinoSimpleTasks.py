import math
import rospy
from juskeshino_tools.JuskeshinoHardware import JuskeshinoHardware
from juskeshino_tools.JuskeshinoNavigation import JuskeshinoNavigation
from juskeshino_tools.JuskeshinoVision import JuskeshinoVision
from juskeshino_tools.JuskeshinoHRI import JuskeshinoHRI

class JuskeshinoSimpleTasks:
    def setNodeHandle():
        print("JuskeshinoSimpleTasks.->Setting ros node...")
        return True

    def waitForTheDoorToBeOpen(timeout):
        attempts = int(timeout/0.1)
        loop = rospy.Rate(10)
        door_closed = JuskeshinoNavigation.isThereObstacleInFront()
        while(not rospy.is_shutdown() and (door_closed is None or door_closed) and attempts >= 0):
            door_closed = JuskeshinoNavigation.isThereObstacleInFront()
            loop.sleep()
            attempts -=1
        return (door_closed is not None and not door_closed)

    def waitForSentenceAndConfirm(timeout):
        cmd = None
        while not rospy.is_shutdown() and cmd is None:
            cmd = JuskeshinoHRI.waitForNewSentence(timeout)
            if cmd is None or cmd.lower() == "justina yes" or cmd.lower() == "justina no":
                JuskeshinoHRI.say("I cannot hear you. Can you repeat your command?")
                cmd = None
        JuskeshinoHRI.say("Do you want me to " + cmd.lower() + "?. Please answer hoostina yes or hoostina no")
        answer = None
        while not rospy.is_shutdown() and answer is None:
            answer = JuskeshinoHRI.waitForNewSentence(timeout)
            if answer is None:
                JuskeshinoHRI.say("I cannot hear you. Please answer hoostina yes or hoostina no")
            if answer.lower() != "justina yes" and answer.lower() != "justina no":
                answer = None
        return [answer.lower() == "justina yes", cmd]

    def waitForSentenceUntilConfirmed(timeout):
        JuskeshinoHRI.say("Please tell me what do you want me to do.")
        [answer, cmd] = JuskeshinoSimpleTasks.waitForSentenceAndConfirm(timeout)
        while not answer:
            JuskeshinoHRI.say("Ok. Please tell me again what do you want me to do.")
            [answer, cmd] = JuskeshinoSimpleTasks.waitForSentenceAndConfirm(timeout)
        JuskeshinoHRI.say("Ok. I'm going to " +  cmd)
        return cmd
            
    def alignWithTable():
        if not JuskeshinoHardware.moveHead(0,-1,3000):
            print("JuskeshinoSimpleTasks.->Cannot move head")
            return False
        edge = JuskeshinoVision.findTableEdge()
        if edge is None:
            print("JuskeshinoSimpleTasks.->Cannot find table edge")
            return False
        A = edge[0].y - edge[1].y
        B = edge[1].x - edge[0].x
        error_a = math.pi/2 - math.atan2(A,B)%math.pi
        error_a = 0 if abs(error_a) < 0.03 else error_a
        error_d = abs(A*edge[0].x + B*edge[0].y)/math.sqrt(A**2 + B**2) - 0.25
        error_d = 0 if error_d < 0.07 else error_d
        timeout = ((abs(error_a) + abs(error_d))/0.5 + 1)
        print("JuskeshinoSimpleTasks.->Moving to align with table d="+str(error_d) + " and theta="+str(error_a))
        return JuskeshinoNavigation.moveDistAngle(error_d, error_a, timeout)

    
