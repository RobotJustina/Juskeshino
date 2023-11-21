import math
import rospy
import tf
from std_msgs.msg import Empty, Float32, Float32MultiArray
from std_srvs.srv import Trigger, TriggerRequest
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalStatus

class JuskeshinoNavigation:
    def setNodeHandle():
        print("JuskeshinoNavigation.->Setting ros node...")
        rospy.Subscriber("/stop"                    , Empty,      JuskeshinoNavigation.callbackStop)
        rospy.Subscriber("/navigation/stop"         , Empty,      JuskeshinoNavigation.callbackNavigationStop)
        rospy.Subscriber("/navigation/status"       , GoalStatus, JuskeshinoNavigation.callbackNavigationStatus)
        rospy.Subscriber("/simple_move/goal_reached", GoalStatus, JuskeshinoNavigation.callbackSimpleMoveStatus)
        JuskeshinoNavigation.pubSimpleMoveDist      = rospy.Publisher("/simple_move/goal_dist", Float32, queue_size=10)
        JuskeshinoNavigation.pubSimpleMoveDistAngle = rospy.Publisher("/simple_move/goal_dist_angle", Float32MultiArray, queue_size=10)
        JuskeshinoNavigation.pubSimpleMoveLateral   = rospy.Publisher("/simple_move/goal_dist_lateral", Float32, queue_size=10)
        JuskeshinoNavigation.pubMvnPlnGetCloseXYA   = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)
        JuskeshinoNavigation.pubNavigationStop      = rospy.Publisher("/navigation/stop", Empty, queue_size=10)
        JuskeshinoNavigation.tfListener = tf.TransformListener()
        JuskeshinoNavigation._stop = False;
        JuskeshinoNavigation._navigation_status = GoalStatus()
        JuskeshinoNavigation._simple_move_status = GoalStatus()
        JuskeshinoNavigation._navigation_status.status  = GoalStatus.PENDING;
        JuskeshinoNavigation._simple_move_status.status = GoalStatus.PENDING;
        
        loop = rospy.Rate(10)
        counter = 3;
        while not rospy.is_shutdown() and counter > 0:
            counter-=1
            loop.sleep()
        return True

    #Methods for checking if goal position is reached.
    def isLocalGoalReached():
        return JuskeshinoNavigation._simple_move_status.status == GoalStatus.SUCCEEDED

    def isGlobalGoalReached():
        return JuskeshinoNavigation._navigation_status.status == GoalStatus.SUCCEEDED;

    def waitForLocalGoalReached(timeout):
        JuskeshinoNavigation._stop = False;
        JuskeshinoNavigation._simple_move_status.status = GoalStatus.PENDING;
        attempts = int(timeout/0.1);
        loop = rospy.Rate(10)
        while (not rospy.is_shutdown() and JuskeshinoNavigation._simple_move_status.status == GoalStatus.PENDING and
               not JuskeshinoNavigation._stop and attempts >= 0):
            loop.sleep()
            attempts -= 1
    
        while (not rospy.is_shutdown() and JuskeshinoNavigation._simple_move_status.status == GoalStatus.ACTIVE and
               not JuskeshinoNavigation._stop and attempts >= 0):
            loop.sleep()
            attempts -=1

        JuskeshinoNavigation._stop = False; #This flag is set True in the subscriber callback
        return JuskeshinoNavigation._simple_move_status.status == GoalStatus.SUCCEEDED

    def waitForGlobalGoalReached(timeout):
        JuskeshinoNavigation._stop = False
        JuskeshinoNavigation._navigation_status.status = GoalStatus.PENDING
        attempts = int(timeout/0.1)
        loop = rospy.Rate(10)
        while (not rospy.is_shutdown() and JuskeshinoNavigation._navigation_status.status == GoalStatus.PENDING and
               not JuskeshinoNavigation._stop and attempts >= 0):
            loop.sleep()
            attempts -= 1

        while (not rospy.is_shutdown() and JuskeshinoNavigation._navigation_status.status == GoalStatus.ACTIVE and
               not JuskeshinoNavigation._stop and attempts >= 0):
            loop.sleep()
            attempts -= 1
            
        JuskeshinoNavigation._stop = False
        return JuskeshinoNavigation._navigation_status.status == GoalStatus.SUCCEEDED


    #Methods for robot localization
    def getRobotPoseWrtMap():
        ([x, y, z], rot) = listener.lookupTransform('map', 'base_link', rospy.Time(0))
        a = 2*math.atan2(rot[2], rot[3])
        a = a - 2*math.pi if a > math.pi else a
        return [x, y, a]

    def getRobotPoseWrtOdom():
        ([x, y, z], rot) = listener.lookupTransform('odom', 'base_link', rospy.Time(0))
        a = 2*math.atan2(rot[2], rot[3])
        a = a - 2*math.pi if a > math.pi else a
        return [x, y, a]
        
    #These methods use the simple_move node
    def startMoveDist(distance):
        msg = Float32()
        msg.data = distance
        JuskeshinoNavigation._simple_move_status.status = GoalStatus.PENDING;
        JuskeshinoNavigation.pubSimpleMoveDist.publish(msg)
        rospy.sleep(0.1)

    def startMoveDistAngle(distance, angle):
        msg = Float32MultiArray()
        msg.data.append(distance)
        msg.data.append(angle)
        JuskeshinoNavigation._simple_move_status.status = GoalStatus.PENDING
        JuskeshinoNavigation.pubSimpleMoveDistAngle.publish(msg)
        rospy.sleep(0.1)

    def startMoveLateral(distance):
        msg = Float32()
        msg.data = distance
        JuskeshinoNavigation._simple_move_status.status = GoalStatus.PENDING
        JuskeshinoNavigation.pubSimpleMoveLateral.publish(msg)
        rospy.sleep(0.1)

    def moveDist(distance, timeout):
        JuskeshinoNavigation.startMoveDist(distance)
        return JuskeshinoNavigation.waitForLocalGoalReached(timeout)


    def moveDistAngle(distance, angle, timeout):
        JuskeshinoNavigation.startMoveDistAngle(distance, angle)
        return JuskeshinoNavigation.waitForLocalGoalReached(timeout)


    def moveLateral(distance, timeout):
        JuskeshinoNavigation.startMoveLateral(distance)
        return JuskeshinoNavigation.waitForLocalGoalReached(timeout)


    #These methods use the mvn_pln node.
    def startGetCloseXYA(x, y, angle):
        msg = PoseStamped()
        msg.header.frame_id = "map";
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = 0
        msg.pose.orientation.x = 0
        msg.pose.orientation.y = 0
        msg.pose.orientation.z = math.sin(angle/2)
        msg.pose.orientation.w = math.cos(angle/2)
        JuskeshinoNavigation._navigation_status.status = GoalStatus.PENDING;
        JuskeshinoNavigation.pubMvnPlnGetCloseXYA.publish(msg)
        rospy.sleep(0.1)


    def startGetClose(location):
        return None
    
    def getCloseXYA(x, y, angle, timeout):
        JuskeshinoNavigation.startGetCloseXYA(x,y,angle)
        return JuskeshinoNavigation.waitForGlobalGoalReached(timeout)


    def getClose(location, timeout):
        JuskeshinoNavigation.startGetClose(location)
        return JuskeshinoNavigation.waitForGlobalGoalReached(timeout)

    def isThereObstacleInFront():
        clt = rospy.ServiceProxy("/navigation/obs_detector/obstacle_in_front", Trigger)
        req = TriggerRequest()
        try:
            resp = clt(req)
            return resp.success
        except:
            return None

    def stopNavigation():
        msg = Empty()
        JuskeshinoNavigation.pubNavigationStop.publish(msg)

    def callbackStop(msg):
        JuskeshinoNavigation._stop = True
        return

    def callbackNavigationStop(msg):
        JuskeshinoNavigation._stop = True

    def callbackNavigationStatus(msg):
        JuskeshinoNavigation._navigation_status = msg;

    def callbackSimpleMoveStatus(msg):
        JuskeshinoNavigation._simple_move_status = msg;
