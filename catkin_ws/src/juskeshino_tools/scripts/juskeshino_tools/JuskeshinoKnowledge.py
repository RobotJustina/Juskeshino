import rospy 
import yaml

class JuskeshinoKnowledge:
    def setNodeHandle():
        print("JuskeshinoKnowledge.->Setting ros node...")
        JuskeshinoKnowledge.locations = {}
        loop = rospy.Rate(10)
        counter = 3;
        while not rospy.is_shutdown() and counter > 0:
            counter-=1
            loop.sleep()
        return True

    def loadLocations(file_name):
        with open(file_name, "r") as stream:
            try:
                JuskeshinoKnowledge.locations = yaml.safe_load(stream)
            except:
                print("Cannot load known locations file: " + file_name)

    def getKnownLocation(loc):
        try:
            return JuskeshinoKnowledge.locations[loc]
        except:
            return None
