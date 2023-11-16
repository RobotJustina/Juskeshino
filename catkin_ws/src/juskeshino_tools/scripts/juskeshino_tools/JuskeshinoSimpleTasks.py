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
            return False
        edge = JuskeshinoVision.findTableEdge()
        if edge is None:
            return False
        A = edge[0].y - edge[1].y
        B = edge[0].x - edge[1].x
        theta = math.atan2(B, A)
        dist  = abs(A*edge[0].x + B*edge[0].y)/math.sqrt(A**2 + B**2)
        print([dist, theta])
        return True
