import math
import rospy
import tf
import numpy
from std_msgs.msg import Empty, Float32, Float32MultiArray
from std_srvs.srv import Trigger, TriggerRequest
from nav_msgs.srv import GetMap
from geometry_msgs.msg import PoseStamped, PointStamped
from actionlib_msgs.msg import GoalStatus
from planning_msgs.srv import *
from planning_msgs.msg import *

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
        JuskeshinoNavigation.cltKnownLocation       = rospy.ServiceProxy("/planning/get_known_location", GetLocation)
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
        listener = tf.TransformListener()
        listener.waitForTransform('map', 'base_link', rospy.Time(0), rospy.Duration(1.0))
        ([x, y, z], rot) = listener.lookupTransform('map', 'base_link', rospy.Time(0))
        a = 2*math.atan2(rot[2], rot[3])
        a = a - 2*math.pi if a > math.pi else a
        return [x, y, a]

    def getRobotPoseWrtOdom():
        listener = tf.TransformListener()
        listener.waitForTransform('odom', 'base_link', rospy.Time(0), rospy.Duration(1.0))
        ([x, y, z], rot) = listener.lookupTransform('odom', 'base_link', rospy.Time(0))
        a = 2*math.atan2(rot[2], rot[3])
        a = a - 2*math.pi if a > math.pi else a
        return [x, y, a]
        
    #These methods use the simple_move node
    def startMoveDist(distance):
        msg = Float32()
        msg.data = distance
        JuskeshinoNavigation._simple_move_status.status = GoalStatus.PENDING
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
        msg.header.frame_id = "map"
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = 0
        msg.pose.orientation.x = 0
        msg.pose.orientation.y = 0
        msg.pose.orientation.z = math.sin(angle/2)
        msg.pose.orientation.w = math.cos(angle/2)
        JuskeshinoNavigation._navigation_status.status = GoalStatus.PENDING
        JuskeshinoNavigation.pubMvnPlnGetCloseXYA.publish(msg)
        rospy.sleep(0.1)


    def startGetClose(location):
        req = GetLocationRequest()
        req.name = location
        try:
            resp = JuskeshinoNavigation.cltKnownLocation(req)
            p = resp.location.pose.position
            q = resp.location.pose.orientation
            a = math.atan2(q.z, q.w)*2
        except:
            print("JuskeshinoNavigation.->Cannot get position for location " + location)
            return False
        JuskeshinoNavigation.startGetCloseXYA(p.x, p.y, a)
        return True
    


    def getCloseSuitableGripPositionLa(position_object, timeout):
        # La posición del objeto debe ser una lista [x,y,z] en coordenadas de 'map'
        l_threshold_la       = 0.25
        r_threshold_la       = 0.11

        if position_object[1] > l_threshold_la:     # Objeto a la izquierda
            mov_izq = (position_object[1] - l_threshold_la) 
            print("mov izq", mov_izq)

            JuskeshinoNavigation.moveLateral(mov_izq , 5.0)
            print("Movimiento lateral", mov_izq )
            return False, mov_izq

        if position_object[1] < r_threshold_la:     # Objeto a la derecha
            mov_der = position_object[1] - r_threshold_la
            print("mov der", mov_der)

            JuskeshinoNavigation.moveLateral(mov_der, 10.0)
            print("Movimiento lateral", mov_der)
            return False, mov_der
            
        return False, 0
    


    def getCloseXYA(x, y, angle, timeout):
        JuskeshinoNavigation.startGetCloseXYA(x,y,angle)
        return JuskeshinoNavigation.waitForGlobalGoalReached(timeout)


    def getClose(location, timeout):
        if JuskeshinoNavigation.startGetClose(location):
            return JuskeshinoNavigation.waitForGlobalGoalReached(timeout)
        return False

    def isThereObstacleInFront():
        clt = rospy.ServiceProxy("/navigation/obs_detector/obstacle_in_front", Trigger)
        req = TriggerRequest()
        try:
            resp = clt(req)
            return resp.success
        except:
            return None

    def isPointInFreeSpace(x,y):
        rospy.wait_for_service('/map_augmenter/get_augmented_map')
        grid_map = rospy.ServiceProxy("/map_augmenter/get_augmented_map", GetMap)().map
        map_info = grid_map.info
        width, height, res = map_info.width, map_info.height, map_info.resolution
        grid_map = numpy.reshape(numpy.asarray(grid_map.data, dtype='int'), (height, width))
        cx = int((x - map_info.origin.position.x)/res)
        cy = int((y - map_info.origin.position.y)/res)
        return grid_map[cy, cx] >= 0  and grid_map[cy, cx] < 40

    def findSuitableNearPointInFreeSpace(x,y):
        #Finds a point in free space nearest to x,y
        [rx, ry, ra] = JuskeshinoNavigation.getRobotPoseWrtMap()
        a = math.atan2(ry - y, rx - x)
        N = 10
        theta_candidates = [a + 2*math.pi/N*i for i in range(N)]
        r_candidates = [1.0, 1.5, 2.0]

        rospy.wait_for_service('/map_augmenter/get_augmented_map')
        grid_map = rospy.ServiceProxy("/map_augmenter/get_augmented_map", GetMap)().map
        map_info = grid_map.info
        width, height, res = map_info.width, map_info.height, map_info.resolution
        grid_map = numpy.reshape(numpy.asarray(grid_map.data, dtype='int'), (height, width))
        print("JuskeshinoNavigation.->Finding suitable pose for ", x, y)
        for r in r_candidates:
            for theta in theta_candidates:
                xg = x + r*math.cos(theta)
                yg = y + r*math.sin(theta)
                cx = int((xg - map_info.origin.position.x)/res)
                cy = int((yg - map_info.origin.position.y)/res)
                print("Testing: ", xg, yg, cx, cy, grid_map[cy, cx] >= 0  and grid_map[cy, cx] < 40)
                if grid_map[cy, cx] >= 0  and grid_map[cy, cx] < 40:
                    return xg, yg, (2*math.pi + theta)%(2*math.pi)-math.pi 
        return None, None, None
            
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
