import math
import rospy
from juskeshino_tools.JuskeshinoHardware import JuskeshinoHardware
from juskeshino_tools.JuskeshinoNavigation import JuskeshinoNavigation
from juskeshino_tools.JuskeshinoVision import JuskeshinoVision

class JuskeshinoSimpleTasks:
    def setNodeHandle():
        print("JuskeshinoSimpleTasks.->Setting ros node...")
        return True

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
