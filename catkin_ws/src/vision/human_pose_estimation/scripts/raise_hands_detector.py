#!/usr/bin/env python3
# coding:UTF-8

import rospy
import smach

import traceback

# import my custom msg
from geometry_msgs.msg import Pose
from vision_msgs.msg import Keypoint 
from vision_msgs.msg  import HumanCoordinates 
from vision_msgs.msg  import HumanCoordinatesArray

class RaiseHandsDetector(smach.State):
    
    def __init__(self,timeout=60.):
        smach.State.__init__(self,
                             outcomes=['success','failure','timeout'],
                             output_keys=['human_coordinate'])

        self.human_coordinate = None

        self.timeout = timeout

        self.trace_keypoint = 0
        self.trace_counter = 0
        self.timeout = timeout
        self.frame = 0

    def callback(self, data):

        now_time = rospy.Time.now()
        if (now_time - data.header.stamp).to_sec() > 1.5: 
            print(now_time - data.header.stamp).to_sec()
            print("time is far from now")
            return

        people = data.coordinates_array
        #print("how many people = ", data.number_of_people)

        if people is None:
            return

        try:

            for person in people:
                person_id = person.person_id
                
                keypoints = {kpt.keypoint_name:kpt for kpt in person.keypoints_array if kpt.keypoint_name in ["neck","r_wri","l_wri","r_hip","l_hip"]}
    
                neck  = [keypoints["neck"].keypoint_coordinates.position.x,
                         keypoints["neck"].keypoint_coordinates.position.y, 
                         keypoints["neck"].keypoint_coordinates.position.z] 

                r_wri = [keypoints["r_wri"].keypoint_coordinates.position.x,
                         keypoints["r_wri"].keypoint_coordinates.position.y, 
                         keypoints["r_wri"].keypoint_coordinates.position.z]

                l_wri = [keypoints["l_wri"].keypoint_coordinates.position.x,
                         keypoints["l_wri"].keypoint_coordinates.position.y,
                         keypoints["l_wri"].keypoint_coordinates.position.z]

                r_hip = [keypoints["r_hip"].keypoint_coordinates.position.x,
                         keypoints["r_hip"].keypoint_coordinates.position.y,
                         keypoints["r_hip"].keypoint_coordinates.position.z]

                l_hip = [keypoints["l_hip"].keypoint_coordinates.position.x,
                         keypoints["l_hip"].keypoint_coordinates.position.y,
                         keypoints["l_hip"].keypoint_coordinates.position.z]

                if (neck and r_wri and l_wri and r_hip and l_hip):
                    # add c_hip
                    c_hip_x = (r_hip[0] + l_hip[0]) / 2
                    c_hip_y = (r_hip[1] + l_hip[1]) / 2
                    c_hip_z = (r_hip[2] + l_hip[2]) / 2
#                    print("r_hip",r_hip)
#                    print("l_hip",l_hip)
#                    print("c_hip",c_hip)
    
                    #TODO add if state using distance
                    

                    if self.trace_counter >= 5: 
                        self.trace_keypoint = 0
                        self.trace_counter = 0
                        self.frame = 0
        
                    if self.frame > 0:
                        if self.trace_keypoint != 0:
                            pass
                        else:
                            self.trace_counter += 1
                            continue
    
                    # for waving hand detection
                    #if (neck[1] > r_wri[1] or neck[1] > l_wri[1]) and  (c_hip_y - neck[1] > 2):
                    if neck[1] > r_wri[1] or neck[1] > l_wri[1]:
                        self.frame += 1
                        print("count")
                        self.trace_keypoint = neck[0] 
    
                        if (self.frame >= 3):
                            print("waving hand detected")
                            self.human_coordinate = [c_hip_x, c_hip_y, c_hip_z]
    
                            self.frame = 0
    
                    else:
                        self.frame = 0
                        break
    
        except KeyError:
            pass

        except IndexError:
            pass

    def execute(self,userdata):
        try:
            sub = rospy.Subscriber('/human_coordinates_array', HumanCoordinatesArray, self.callback) 
            start_time = rospy.Time.now()
            while not rospy.is_shutdown():
                if self.human_coordinate:
    
                    userdata.human_coordinate = self.human_coordinate
                    print("to userdata variable ->> ",self.human_coordinate)
                    sub.unregister()
                    return 'success'
    
                if (rospy.Time.now() - start_time).to_sec() > self.timeout:
                    sub.unregister()
                    return 'timeout'
        except:
            rospy.logger(traceback.format_exc())
            sub.unregister()
            return 'failure'

if __name__ == '__main__':
        rospy.init_node('waving_hand_detection_state')
        sm = smach.StateMachine(outcomes=['success','failure'])

        with sm:
            smach.StateMachine.add('DEBUG', RaiseHandsDetector(),
                                   transitions={'success': 'success',
                                                'timeout': 'failure',
                                                'failure': 'failure'})
        sm.execute()
