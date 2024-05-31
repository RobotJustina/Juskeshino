import rospy 
import yaml
import math
from planning_msgs.msg import *
from planning_msgs.srv import *

class JuskeshinoKnowledge:
    def setNodeHandle():
        print("JuskeshinoKnowledge.->Setting ros node...")
        clt_known_locs = rospy.ServiceProxy("/planning/get_known_location", GetLocation)
        #JuskeshinoKnowledge.locations = {}

        loop = rospy.Rate(10)
        counter = 3;
        while not rospy.is_shutdown() and counter > 0:
            counter-=1
            loop.sleep()
        return True

    # def loadLocations(file_name):
    #     with open(file_name, "r") as stream:
    #         try:
    #             JuskeshinoKnowledge.locations = yaml.safe_load(stream)
    #         except:
    #             print("Cannot load known locations file: " + file_name)

    def getKnownLocation(loc):
        req = GetLocationRequest()
        req.name = loc
        try:
            resp = clt(req)
            p = resp.location.pose.position
            q = resp.location.pose.orientation
            a = math.atan2(q.z, q.w)*2
            return p.x, p,y, a
        except:
            return None
