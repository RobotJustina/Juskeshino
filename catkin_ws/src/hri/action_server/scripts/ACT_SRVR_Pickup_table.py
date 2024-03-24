#!/usr/bin/env python3
from utils_action import *
from smach_ros import ActionServerWrapper
from action_server.msg import PickUpAction

##### Define state INITIAL #####

# --------------------------------------------------
class Initial(smach.State):

    def __init__(self):

        smach.State.__init__(self, outcomes=['succ', 'failed', 'tries'],input_keys=['goal'])
        self.tries = 0
        
    def execute(self, userdata):
        
        self.tries += 1
        if self.tries == 1:
            return 'failed'
        if 'goal' not in userdata:
            rospy.logwarn("No goal received yet.")
            return 'failed'

        global target_object
        target_object= userdata.goal.target.data #'apple'   ### GET FROM REQUEST
        
        rospy.loginfo('STATE : INITIAL')
        print('Robot neutral pose')
        print(f'Try {self.tries} of 5 attempts')
        head.set_joint_values([0.0,0.0])
        rospy.sleep(0.4)
        talk (f'ready to take {target_object} in table')
        print (f'ready to take {target_object} in table')
        rospy.sleep(0.8)

        return 'succ'


# --------------------------------------------------
class Wait_push_hand(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed', 'tries'])
        self.tries = 0

    def execute(self, userdata):

        rospy.loginfo('STATE : Wait for Wait_push_hand')
        print('Waiting for hand to be pushed')
        self.tries += 1
        print(f'Try {self.tries} of 4 attempts')
        if self.tries == 4:
            self.tries=0
            return 'tries'
        talk('Gently... push my hand to continue')
        succ = wait_for_push_hand(100)

        if succ:
            return 'succ'
        else:
            return 'failed'
#########################################################################################################
class Find_object(smach.State):  # ADD KNONW LOCATION DOOR
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed', 'tries'])
        self.tries = 0

    def execute(self, userdata):

        rospy.loginfo('STATE : Segment and tfing object' )
        talk (f'Using Yolo to find {target_object}')
        print (f'Using Yolo to find {target_object}')

        print(f'Try {self.tries} of 3 attempts')
        self.tries += 1
        if self.tries == 4:
            self.tries=0
            talk("I failed finding the object, please help")
            talk("Ending action")
            return 'tries'

        head.set_joint_values([0.0,-0.65])
        arm.set_named_target('go')
        arm.go()
        rospy.sleep(0.6)
        image= cv2.cvtColor(rgbd.get_image(), cv2.COLOR_RGB2BGR)
        img_msg  = bridge.cv2_to_imgmsg(image)
        req      = classify_client.request_class()
        req.in_.image_msgs.append(img_msg)
        res      = classify_client(req)
        
        for i  in range(len(res.names)):
        
            if target_object==res.names[i].data[4:] or 'bowl'==res.names[i].data[4:] :
                talk (f'{target_object} found')
                tf_man.pub_static_tf(pos=[res.poses[i].position.x ,res.poses[i].position.y,res.poses[i].position.z], 
                                     rot=[0,0,0,1], ref="head_rgbd_sensor_rgb_frame", point_name=target_object )   
                rospy.sleep(0.3)
                tf_man.change_ref_frame_tf(target_object)
                return 'succ'
                
        talk (f'I could not find the {target_object}')
        return 'failed'
# --------------------------------------------------
class Get_close_to_object(smach.State):  # ADD KNONW LOCATION DOOR
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed', 'tries'])
        self.tries = 0

    def execute(self, userdata):
        global target_object
        rospy.loginfo('STATE : Navigate to known location')
        print (target_object)
        talk (f'Getting close to {target_object} using move D to ')


        print('getting close to Target')
        res = omni_base.move_d_to(0.66,target_object)
        head.set_named_target('neutral')

        _,rot= tf_man.getTF("base_link",ref_frame='map')
        original_rot=tf.transformations.euler_from_quaternion(rot)[2]
    
        succ = False
        start_time = rospy.get_time()          
        timeout=30.0
        while not succ  and rospy.get_time() - start_time < timeout: 
            
            _,rot= tf_man.getTF("base_link",ref_frame='map')
            trans,_=tf_man.getTF(target_object,ref_frame="base_link")

            trans
            eX, eY, eZ = trans
            
            eX+= -0.4
            eY+= -.06
            
            eT= tf.transformations.euler_from_quaternion(rot)[2] - original_rot #Original 
            print (eT)
            if eT > np.pi: eT=-2*np.pi+eT
            if eT < -np.pi: eT= 2*np.pi+eT
            rospy.loginfo("error: {:.2f}, {:.2f}, angle {:.2f}, target obj frame {}".format(eX, eY , eT,target_object))
            X, Y, Z = trans
            rospy.loginfo("Pose: {:.2f}, {:.2f}, angle {:.2f}, target obj frame {}".format(X, Y , eT,target_object))
            
            if abs(eX) <=0.05 :
                print ('here')
                eX = 0
            if abs(eY) <=0.05  :
                eY = 0
            if abs(eT   ) < 0.1:
                eT = 0
            succ =  eX == 0 and eY == 0 and eT==0
            omni_base.tiny_move( velX=0.2*+eX, velY=0.3*eY, velT=-eT, std_time=0.2, MAX_VEL=0.3) 
        
        if succ:return 'succ'
        return 'failed'   ### ADD TIMEOUT


class Move_arm_pregrasp(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed'])
        self.tries = 0

    def execute(self, userdata):

        rospy.loginfo('STATE : FK to pregrasp from above floor')
        

        self.tries += 1
        print(f'Try {self.tries} of 4 attempts')
        trans,_=tf_man.getTF(target_object,ref_frame="base_link")
        pickup_pose=[min(trans[2],0.66),-1.2,0.0,-1.9, 0.0, 0.0]
        #pickup_pose=[0.65,-1.2,0.0,-1.9, 0.0, 0.0]
        succ= arm.go(pickup_pose)
        gripper.open()

        
        if succ:
            return 'succ'
        else:
            return 'failed'
#########################################################################################################
        
# --------------------------------------------------
class Move_arm_grasp(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed', 'tries'])
        self.tries = 0

    def execute(self, userdata):

        rospy.loginfo('STATE : fine adjustments using hand cam yolo')
        talk ('fine adjustments using hand cam yolo')
        #####################################################
        if target_object=='pear':common_misid='tennis_ball'
        if target_object=='apple':common_misid='orange'
        #########################################################

        eX,eY= 0.3,0.3 

        while abs(eX) > 0.08 or abs(eY) > 0.08:
            image= cv2.cvtColor(hand_cam.get_image(), cv2.COLOR_RGB2BGR)
            img_msg  = bridge.cv2_to_imgmsg(image)
            req      = classify_client.request_class()
            req.in_.image_msgs.append(img_msg)
            res      = classify_client(req)


            for i in range(len(res.names)):
                if res.names[i].data[4:]== target_object or res.names[i].data[4:]== common_misid:#'013_apple' or res.names[i].data== '017_orange':
                    bbM=res.pt_min.data[2*i:2*i+2]
                    bbm=res.pt_max.data[2*i:2*i+2]
                    bbx=[bbM,bbm]
            if 'bbx' not in locals():print ('not yoloed')
            else:
                eY=(bbx[1][1]-215)/215    #experience value
                print(f'bbx {bbx}')
                #eX=(bbx[0][0]-323)/323
                eY=(bbx[1][1]-239)/239
                eX=(bbx[0][0]-312)/312
                #eY=(bbx[1][1]-215)/215    #experience value

                print(f'error relative to gripper cam ex={eX} ey={eY}')
                omni_base.tiny_move(velX=0.015*eX, velY=0.06*-eY, std_time=0.2,MAX_VEL=0.05) 
                print(f'error relative to gripper cam ex={eX} ey={eY}:::::: Error within tolerance')
        talk ('ready to grasp')
        clear_octo_client()
        av=arm.get_current_joint_values()
        
        pose,_=tf_man.getTF(target_frame='hand_palm_link',ref_frame=target_object)
        av[0]+=0.07-pose[2]

        #av[0]+= -0.17###LAB
        #av[0]+= -0.35###GAZ
        arm.go(av)
        rospy.sleep(0.5)
        gripper.close(force=0.06)
        succ=brazo.check_grasp()
        if succ:
            av=arm.get_current_joint_values()
            av[0]+= 0.15
            arm.go(av)
            omni_base.tiny_move(velX=-0.3, std_time=4.0)
            arm.set_named_target('go')
            arm.go()
            return 'succ'
        return 'failed'
        
#########################################################################################################
        
#########################################################################################################
        
# --------------------------------------------------
class Plan_arm(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed', 'tries'])
        self.tries = 0

    def execute(self, userdata):

        rospy.loginfo('STATE : plan arm using IK')
        
        self.tries += 1
        print(f'Try {self.tries} of 10 attempts')
        if self.tries == 10:
            self.tries=0
            return 'tries'  
        #t=tfBuffer.lookup_transform('hand_palm_link', 'Target',rospy.Time())
        clear_octo_client()
        pose, quat=tf_man.getTF('Target')

        wb_gp=whole_body.get_current_pose()
        wb_gp.pose.position.x= pose[0]
        wb_gp.pose.position.y= pose[1]
        wb_gp.pose.position.z= pose[2]+0.35

        whole_body.set_pose_target(wb_gp)
        plan=whole_body.plan()
        
        if plan[0]:
            self.tries=0
            return 'succ'
        else:
            return 'failed'
#########################################################################################################
        
                
# --------------------------------------------------
class Execute_plan(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed', 'tries'])
        self.tries = 0

    def execute(self, userdata):

        rospy.loginfo('STATE : EXECUTE arm using IK')
        

        self.tries += 1
        print(f'Try {self.tries} of 10 attempts')
        if self.tries == 10:
            self.tries=0
            return 'tries'  
        #t=tfBuffer.lookup_transform('hand_palm_link', 'Target',rospy.Time())
        
        succ=whole_body.go()

        pose, quat=tf_man.getTF('Target',   )
        pos,rot=tf_man.getTF(target_frame='hand_palm_link',ref_frame='Target')
        print (pos,rot,'pos,rot')

        if succ:
            return 'succ'
        else:
            clear_octo_client()
            return 'failed'
#########################################################################################################
        
        
        

# --------------------------------------------------
class Goto_exit(smach.State):  # ADD KNONW LOCATION DOOR
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed', 'tries'])
        self.tries = 0

    def execute(self, userdata):

        rospy.loginfo('STATE : Navigate to known location')

        print(f'Try {self.tries} of 3 attempts')
        self.tries += 1
        if self.tries == 3:
            return 'tries'
        if self.tries == 1: talk('Navigating to, exit')
        #print ('navigating to room'+str(self.next_room))
        res = omni_base.move_base(known_location='exit')
        print(res)

        if res == 3:
            talk('test done! thanks for your attention!')
            return 'succ'

        else:
            talk('Navigation Failed, retrying')
            return 'failed'


# --------------------------------------------------
def init(node_name):
    print('smach ready')

# --------------------------------------------------
# Entry point
if __name__ == '__main__':
    
    print("Takeshi STATE MACHINE...")
    init("takeshi_smach")
    # State machine, final state "END"
    sm = smach.StateMachine(outcomes=['succeeded','preempted','failed'], input_keys=['goal'])

    
    
    # sm.userdata.clear = False
    sis = smach_ros.IntrospectionServer('SMACH_VIEW_SERVER', sm, '/SM_PICKUP_table')
    sis.start()

    with sm:
        # State machine for Restaurant
        smach.StateMachine.add("INITIAL",       Initial(),      
                               transitions={'failed': 'preempted', 'succ': 'FIND_OBJECT', 'tries': 'succeeded'} ,remapping={'goal': 'goal'} )
        #################################################################################################################################################################
        smach.StateMachine.add("FIND_OBJECT", Find_object(),   
                               transitions={'failed': 'FIND_OBJECT', 'succ': 'GET_CLOSE_TO_OBJECT', 'tries': 'failed'})
        smach.StateMachine.add("GET_CLOSE_TO_OBJECT", Get_close_to_object(),   
                               transitions={'failed': 'GET_CLOSE_TO_OBJECT', 'succ': 'MOVE_ARM_PREGRASP', 'tries': 'INITIAL'})
        smach.StateMachine.add("MOVE_ARM_PREGRASP", Move_arm_pregrasp(),   
                               transitions={'failed': 'MOVE_ARM_PREGRASP', 'succ': 'MOVE_ARM_GRASP'})
        smach.StateMachine.add("MOVE_ARM_GRASP", Move_arm_grasp(),   
                               transitions={'failed': 'MOVE_ARM_PREGRASP', 'succ': 'succeeded', 'tries': 'failed'})
        smach.StateMachine.add("PLAN_ARM", Plan_arm(),      
                               transitions={'failed': 'PLAN_ARM', 'succ': 'EXECUTE_PLAN', 'tries': 'FIND_OBJECT'})
        smach.StateMachine.add("EXECUTE_PLAN", Execute_plan(),   
                               transitions={'failed': 'EXECUTE_PLAN', 'succ': 'succeeded','tries': 'failed'})
        smach.StateMachine.add("WAIT_HAND", Wait_push_hand(),       
                               transitions={'failed': 'WAIT_HAND','succ': 'GOTO_EXIT', 'tries': 'WAIT_HAND'})
        smach.StateMachine.add("GOTO_EXIT", Goto_exit(),     
                               transitions={'failed': 'succeeded','succ': 'succeeded', 'tries': 'succeeded'})
        ##################################################################################################################################################################
        
    asw = ActionServerWrapper(
        'grasp_table_act_server', PickUpAction,
       
        wrapped_container = sm,
        succeeded_outcomes = ['succeeded'],
        aborted_outcomes = ['failed'],
        preempted_outcomes = ['preempted'],
        goal_key='goal',
        result_key = 'egad_its_a_result' )
    asw.run_server()

    

    outcome = sm.execute()

