import rospy
from std_msgs.msg import Empty

class JuskeshinoNavigation:
    n = None
    def __init__(self):
        return

    def callbackStop(msg):
        JuskeshinoNavigation._stop = True
        return

    def setNodeHandle():
        print("JuskeshinoNavigation.->Setting ros node...")
        rospy.Subscriber("/stop"                    , Empty, JuskeshinoNavigation.callbackStop);
        # subNavigationStop      = nh->subscribe("/navigation/stop"         , 10, &JuskeshinoNavigation::callbackNavigationStop);
        # subNavigationStatus    = nh->subscribe("/navigation/status"       , 10, &JuskeshinoNavigation::callbackNavigationStatus);
        # subSimpleMoveStatus    = nh->subscribe("/simple_move/goal_reached", 10, &JuskeshinoNavigation::callbackSimpleMoveStatus);
        # pubSimpleMoveDist      = nh->advertise<std_msgs::Float32          >("/simple_move/goal_dist", 10);
        # pubSimpleMoveDistAngle = nh->advertise<std_msgs::Float32MultiArray>("/simple_move/goal_dist_angle", 10);
        # pubSimpleMoveLateral   = nh->advertise<std_msgs::Float32          >("/simple_move/goal_dist_lateral", 10);
        # pubMvnPlnGetCloseXYA   = nh->advertise<geometry_msgs::PoseStamped >("/move_base_simple/goal", 10);
        # pubNavigationStop      = nh->advertise<std_msgs::Empty>            ("/navigation/stop", 10);
        # tf_listener = new tf::TransformListener();
