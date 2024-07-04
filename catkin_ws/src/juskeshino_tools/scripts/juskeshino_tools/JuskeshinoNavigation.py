import math
import rospy
import tf
from std_msgs.msg import Empty, Float32, Float32MultiArray
from std_srvs.srv import Trigger, TriggerRequest
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
    


    def getCloseSuitableGripPositionLa(location, position_object, timeout):
        # La posición del objeto debe ser una lista [x,y,z] en coordenadas de 'map'
        l_threshold_la       = 0.22
        r_threshold_la       = 0.11
        # Se extrae la orientacion de la locacion
        req = GetLocationRequest()
        req.name = location
        try:
            resp = JuskeshinoNavigation.cltKnownLocation(req)
            q = resp.location.pose.orientation
            a = math.atan2(q.z, q.w)*2
        except:
            print("JuskeshinoNavigation.->Cannot get position for location " + location)

        if position_object[1] > l_threshold_la:     # Objeto a la izquierda
            mov_izq = (position_object[1] - l_threshold_la) 
            print("mov izq", mov_izq)
            if abs(mov_izq) > 0.28:
                new_point = [0 - 0.1, position_object[1]-0.2, 0]  # xyz
                #print("new point", new_point)

                listener = tf.TransformListener()
                point_msg = PointStamped()  
                point_msg.header.frame_id = 'base_link'   # frame de origen
                point_msg.header.stamp = rospy.Time() # la ultima transformacion
                point_msg.point.x = new_point[0]
                point_msg.point.y = new_point[1]
                point_msg.point.z = new_point[2]
                #listener.waitForTransform('base_link' , 'map', rospy.Time(), rospy.Duration())
                point_target_frame = listener.transformPoint('map', point_msg)
                new_point_nav = point_target_frame.point.x, point_target_frame.point.y, point_target_frame.point.z
                #print("new point nav izq", new_point_nav)
                JuskeshinoNavigation.moveDist(-0.3, 5.0)
                JuskeshinoNavigation.getCloseXYA(new_point_nav[0] , new_point_nav[1] , a, timeout)
                return True, mov_izq
            else:
                JuskeshinoNavigation.moveLateral(mov_izq + 0.03, 5.0)
                print("Movimiento lateral", mov_izq + 0.03)
                return False, mov_izq

        if position_object[1] < r_threshold_la:     # Objeto a la derecha
            mov_der = position_object[1] - r_threshold_la
            print("mov der", mov_der)
            if abs(mov_der) > 0.28:
                new_point = [0 - 0.1, position_object[1] - 0.2, 0]  # xyz
                #print("new point", new_point)
                listener = tf.TransformListener()
                point_msg = PointStamped()  
                point_msg.header.frame_id = 'base_link'   # frame de origen
                point_msg.header.stamp = rospy.Time() # la ultima transformacion
                point_msg.point.x = new_point[0]
                point_msg.point.y = new_point[1]
                point_msg.point.z = new_point[2]
                listener.waitForTransform('base_link' , 'map', rospy.Time(0), rospy.Duration(1.0))
                point_target_frame = listener.transformPoint('map', point_msg)
                new_point_nav = [point_target_frame.point.x, point_target_frame.point.y, point_target_frame.point.z]
                #print("new point nav der", new_point_nav)
                JuskeshinoNavigation.moveDist(-0.3, 5.0)
                JuskeshinoNavigation.getCloseXYA(new_point_nav[0] , new_point_nav[1] , a, timeout)
                return True, mov_der
            else:
                JuskeshinoNavigation.moveLateral(mov_der - 0.03 , 10.0)
                print("Movimiento lateral", mov_der -0.03)
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
