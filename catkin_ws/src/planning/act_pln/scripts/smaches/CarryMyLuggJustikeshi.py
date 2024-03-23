#!/usr/bin/env python3

#from smach_utils2 import *  # TODO: Delete
from smach_utils_justina_carry import *
from smach_ros import SimpleActionState

#########################################################################################################
class Initial(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed', 'tries'])
        self.tries = 0

    def execute(self, userdata):

        
        rospy.loginfo('STATE : INITIAL')
        print('Robot neutral pose')
        self.tries += 1
        print(f'Try {self.tries} of 5 attempts')
        if self.tries == 3:
            return 'tries'
        
        #READ YAML ROOMS XYS

        # TODO: delete---->       
        # global arm ,  hand_rgb
        # hand_rgb = HAND_RGB()

        # arm = moveit_commander.MoveGroupCommander('arm')
        # head.set_named_target('neutral')

        # rospy.sleep(0.8)
        # arm.set_named_target('go')
        # arm.go()
        # rospy.sleep(0.3)
        # <---

        #gripper.open()
        #rospy.sleep(0.3)

        #gripper.close()
        #rospy.sleep(0.3)


        
        global arm ,  hand_rgb
        
        #TAKESHI ONLY
        #arm = moveit_commander.MoveGroupCommander('arm')
        #head.set_named_target('neutral')
        #rospy.sleep(0.8)
        #arm.set_named_target('go')
        #arm.go()
        #rospy.sleep(0.3)       
        #TAKESHI ONLY

        
        return 'succ'

#########################################################################################################
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
            return 'tries'
        voice.talk('Gently... push my hand to begin')

        
        
        #succ = wait_for_push_hand(100) # NOT JUSTIKESHINABLE #WAIT FOR DOOR
        return 'succ'## REMOVE
        if succ:
            return 'succ'
        else:
            return 'failed'

#########################################################################################################
class Goto_living_room(smach.State):  
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed', 'tries'])
        self.tries = 0

    def execute(self, userdata):

        rospy.loginfo('STATE : Navigate to known location')

        print(f'Try {self.tries} of 3 attempts')
        self.tries += 1
        if self.tries == 3:
            return 'tries'

        if self.tries == 1: voice.talk('Navigating to, living room')



        res = omni_base.move_base(known_location='living_room', time_out=200)
        print(res)
        if res:
            return 'succ'
        else:

            voice.talk('Navigation Failed, retrying')

            return 'failed'

#########################################################################################################
class Find_human(smach.State):
    def __init__(self):
        smach.State.__init__(
            self, outcomes=['succ', 'failed', 'tries'])
        self.tries = 0

    def execute(self, userdata):
        rospy.loginfo('State : Find_human')

        voice.talk('Scanning the room for humans')
        #talk('Scanning the room for humans')

        self.tries += 1
        if self.tries >= 4:
            self.tries = 0
            return'tries'
        #if self.tries==1:head.set_joint_values([ 0.0, 0.0])# Looking ahead
        #if self.tries==2:head.set_joint_values([ 0.5, 0.1])#looking left
        #if self.tries==3:head.set_joint_values([-0.5, 0.1])#looking right        
        humanpose=detect_human_to_tf()  #make sure service is running (pointing detector server now hosts this service)
        if humanpose== False:
            print ('no human ')
            return 'failed'

        #talk('Please start pointing at the bag. In three')
        rospy.sleep(1.0)
        #talk('Two')
        rospy.sleep(1.0)
        #talk('One')
        rospy.sleep(1.5)
        res=pointing_detect_server.call()
        if (res.x_r+res.y_r)==0 and  (res.x_l+res.y_l)==0  :
            #talk('I did not find a pointing arm, I will try again')
            print ('no pointing ')
            return 'failed'
        #talk('Ok')

        rospy.sleep(0.8)
        if res.x_r ==-1: tf_man.pub_static_tf(pos=[res.x_l, res.y_l,0], rot =[0,0,0,1], point_name='pointing_')
        else: tf_man.pub_static_tf(pos=[res.x_r, res.y_r,0], rot =[0,0,0,1], point_name='pointing_')
        #if (res.x_l+res.y_l)!=0:tf_man.pub_static_tf(pos=[res.x_l, res.y_l,0], rot =[0,0,0,1], point_name='pointing_left')
        return 'succ'# There is now a "human TF with humans aprrox location(face)"

#########################################################################################################
class Scan_floor(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed', 'tries'])
        self.tries = 0
        

    def execute(self, userdata):

        rospy.loginfo('STATE : Scan estimated pointing area')
        self.tries+=1  
        if self.tries==1:head.to_tf('pointing_')
        #if self.tries==2:head.to_tf('pointing_left')     
        if self.tries==3:
            self.tries=0
            return 'tries'
        
        
        ##### Segment and analyze
        #img=rgbd.get_image()
        print ('got image for segmentation')
        ##### Segment and analyze
        rospy.sleep(1.5)
        request= segmentation_server.request_class() 
        request.height.data=0.00
        res=segmentation_server.call(request)
        img=bridge.imgmsg_to_cv2(res.im_out.image_msgs[0])
        save_image(img)  # TODO: Delete?
        print (res.poses.data)
        #res=segmentation_server.call()
        origin_map_img=[round(img_map.shape[0]*0.5) ,round(img_map.shape[1]*0.5)]

        if len(res.poses.data)==0:
            #talk('no Objects in area....')
            return 'failed'
        else:
            succ=seg_res_tf_pointing(res)   # Cambia ya que depende de tf pointing_
            return 'succ'
            #print ('poses',res.poses_corr.data,res.poses.data)
            #poses=np.asarray(res.poses.data)
            #quats=np.asarray(res.quats.data)
            #poses=poses.reshape((int(len(poses)/3) ,3     )      )  
            #quats=quats.reshape((int(len(quats)/4) ,4     )      )  
            #num_objs=len(poses)
            #
            #for i,pose in enumerate(poses):
            #    
            #    quat=  quats[i]/np.linalg.norm(quats[i])
            #    point_name=f'object_{i}'
            #    print ('point_name',point_name, pose, quat)
            #    print (np.rad2deg(tf.transformations.euler_from_quaternion(quat)))
            #    print( f'################## {point_name} estimated rotation PCA {np.rad2deg(tf.transformations.euler_from_quaternion(quat))[0]}') #np.sign(np.rad2deg(tf.transformations.euler_from_quaternion(quat))[0])*np.rad2deg(tf.transformations.euler_from_quaternion(quat))[1]}')
            #    axis=[0,0,1]
            #    angle = tf.transformations.euler_from_quaternion(quat)[0]
            #    rotation_quaternion = tf.transformations.quaternion_about_axis(angle, axis)
            #    print ('rot quat',rotation_quaternion)
            #    tf_man.pub_static_tf(pos=pose, rot =[0,0,0,1], point_name=point_name+'_norot', ref='head_rgbd_sensor_rgb_frame')## which object to choose   #TODO
            #    tf_man.change_ref_frame_tf(point_name=point_name+'_norot', new_frame='map')
            #    tf_man.pub_static_tf(pos=pose, rot =rotation_quaternion, point_name=point_name, ref='head_rgbd_sensor_rgb_frame')## which object to choose   #TODO
            #    rospy.sleep(0.3)
            #    tf_man.change_ref_frame_tf(point_name=point_name,rotational=rotation_quaternion , new_frame='map')
            #    rospy.sleep(0.3)
            #    pose,_= tf_man.getTF(point_name)
            #    print (f'Occupancy map at point object {i}-> pixels ',origin_map_img[1]+ round(pose[1]/pix_per_m),origin_map_img[0]+ round(pose[0]/pix_per_m), img_map[origin_map_img[1]+ round(pose[1]/pix_per_m),origin_map_img[0]+ round(pose[0]/pix_per_m)])
            #    if img_map[origin_map_img[1]+ round(pose[1]/pix_per_m),origin_map_img[0]+ round(pose[0]/pix_per_m)]!=0:#### Yes axes seem to be "flipped" !=0:
            #        print ('reject point suggested ( for floor), most likely part of arena, occupied inflated map')
            #    print (f"object found at map coords.{pose} ")
            #    return 0
            #    return 'succ'

#########################################################################################################
class Scan_table(smach.State):
    def __init__(self):
        smach.State.__init__(
            self, outcomes=['succ', 'failed', 'tries'])
        self.tries = 0
        #self.target= '024_bowl'
        self.target= '003_cracker_box'
    def execute(self, userdata):
        rospy.loginfo('State : Scanning_table')

        #talk('Scanning table')

        self.tries += 1
        if self.tries >= 4:self.tries = 0
        if self.tries==1:head.set_joint_values([ 0.0, -0.7])
        if self.tries==2:head.set_joint_values([ 0.2, -0.7])
        if self.tries==3:head.set_joint_values([-0.2, -0.7])
        rospy.sleep(1.3)
        img_msg  = bridge.cv2_to_imgmsg(rgbd.get_image())
        req      = classify_client.request_class()
        req.in_.image_msgs.append(img_msg)
        res      = classify_client(req)
        print (res.poses, res.names)
        if len (res.poses)==0:return 'failed'
        #for pose in res.poses:
        for pose, point_name in zip(res.poses, res.names):
            
            if point_name.data == self.target :
                print ([pose.position.x,pose.position.y,pose.position.z],[pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w],point_name.data ,'*******************************')
                tf_man.pub_static_tf(pos=[pose.position.x,pose.position.y,pose.position.z], rot =[pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w] , point_name='static'+point_name.data, ref='head_rgbd_sensor_depth_frame') #head_rgbd_sensor_rgb_frame')## which object to choose   #TODO
                rospy.sleep(0.3)
                succ=tf_man.change_ref_frame_tf('static'+point_name.data)
                rospy.sleep(0.3)
                print (f'{self.target} found')
                self.target='003_cracker_box'
                return 'succ'
        print( self.target , 'not found')
        return 'failed'

#########################################################################################################
class Pre_pickup(smach.State):
    def __init__(self):
        smach.State.__init__(
            self, outcomes=['succ', 'failed', 'tries'])
        self.target= 'object_0'    
        
    def execute(self, userdata):
        rospy.loginfo(f'State : Pre PICKUP  {self.target} ')

        #talk(f'Picking up {self.target}, please stay where you are')

        rospy.sleep(0.7)
        global target_object, pca_angle
        head.set_named_target('neutral')
        rospy.sleep(0.5)
        clear_octo_client()  # TODO: ???
        res = omni_base.move_d_to(0.55,self.target)

        gripper.open()
        if res:return 'succ'
        return 0
        
        
        
        succ= arm.go(pickup_pose)
        rospy.sleep(1.0)
        

        if succ:
            
            succ = False
            
            while not succ:
                #trans,rot = tf_man.getTF(target_frame='static003_cracker_box', ref_frame='hand_palm_link')
                trans,_ = tf_man.getTF(target_frame=target_object, ref_frame='hand_palm_link')
                _,rot = tf_man.getTF(target_frame='base_link', ref_frame='map')

                if type(trans) is bool:
                    trans,_ = tf_man.getTF(target_frame='static029_plate', ref_frame='hand_palm_link')
                    _,rot = tf_man.getTF(target_frame='base_link', ref_frame='map')
                    print('no tf')
                    return 'failed'
                if type(trans) is not bool:
                    eX, eY, eZ = trans
                    eY+=-0.05
                    eT= tf.transformations.euler_from_quaternion(rot)[2] - 0.5*np.pi    #(known loc pickup change to line finder?)   NO! THERE ARE  ROUND TABLES !
                    rospy.loginfo("Distance to goal: {:.2f}, {:.2f}, angle {:.2f}, target obj frame {}".format(eX, eY , eT,target_object))
                    if abs(eX) < 0.03:
                        eX = 0
                    if abs(eY) < 0.01:
                        eY = 0
                    if abs(eT   ) < 0.05:
                        eT = 0
                    succ =  eX == 0 and eY == 0 and eT==0
                        # grasp_base.tiny_move(velY=-0.4*trans[1], std_time=0.2, MAX_VEL=0.3)
                    omni_base.tiny_move(velX=0.13*eX, velY=-0.4*eY, velT=-0.3*eT, std_time=0.2, MAX_VEL=0.3) #Pending test
            
            print(tf_man.getTF(ref_frame=target_object,target_frame='base_link'), 'tf obj base######################################')

            
            return 'succ'
            
        return 'failed'

#########################################################################################################
class Pickup(smach.State):
    def __init__(self):
        smach.State.__init__(
            self, outcomes=['succ', 'failed', 'tries'])
        self.tries = 0
        self.target='object_0'
    def execute(self, userdata):
        rospy.loginfo('State :  PICKUP ')
        clear_octo_client()  # TODO: ???
        self.tries += 1
        target_object=self.target

        if self.tries >= 4:
            self.tries = 0
            return'tries'
        
        clear_octo_client()

        pickup_pose=[0.0,-1.4,0.0,-1.74, 0.0, 0.0]
        succ= arm.go(pickup_pose)
        


        succ = False
        
        '''while not succ:
            #trans,rot = tf_man.getTF(target_frame='static003_cracker_box', ref_frame='hand_palm_link')
            trans,_ = tf_man.getTF(target_frame=target_object, ref_frame='hand_palm_link')
            _,rot = tf_man.getTF(target_frame='base_link', ref_frame='map')

            
            if type(trans) is not bool:
                eX, eY, eZ = trans
                eY+=-0.05
                eT= tf.transformations.euler_from_quaternion(rot)[2] - 0.5*np.pi    #(known loc pickup change to line finder?)   NO! THERE ARE  ROUND TABLES !
                rospy.loginfo("Distance to goal: {:.2f}, {:.2f}, angle {:.2f}, target obj frame {}".format(eX, eY , eT,target_object))
                if abs(eX) < 0.03:
                    eX = 0
                if abs(eY) < 0.04:
                    eY = 0
                if abs(eT   ) < 0.05:
                    eT = 0
                succ =  eX == 0 and eY == 0 and eT==0
                    # grasp_base.tiny_move(velY=-0.4*trans[1], std_time=0.2, MAX_VEL=0.3)
                omni_base.tiny_move(velX=0.13*eX, velY=-0.4*eY, velT=0.3*eT, std_time=0.2, MAX_VEL=0.3) #Pending test'''
        brazo.move_hand_to_target(target_frame= target_object)
        gripper.close()





        arm.set_named_target('go')
        arm.go()
        res  = omni_base.move_base(known_location='living_room')
        succ=brazo.check_grasp()
        if res:
            return 'succ'
        else:
            return 'failed'

#########################################################################################################
class Pickup_two(smach.State):
    def __init__(self):
        smach.State.__init__(
            self, outcomes=['succ', 'failed', 'tries'])
        self.tries = 0
        self.target='object_0'
    def execute(self, userdata):
        rospy.loginfo('State :  PICKUP ')
        clear_octo_client()
        self.tries += 1
        target_object=self.target

        if self.tries >= 4:
            self.tries = 0
            return'tries'
        
        clear_octo_client()

        _,rot= tf_man.getTF("base_link",ref_frame='map')
        original_rot=tf.transformations.euler_from_quaternion(rot)[2]
        target_object=self.target


        succ = False
                    
        while not succ:
            
            _,rot= tf_man.getTF("base_link",ref_frame='map')
            trans,_=tf_man.getTF(target_object,ref_frame="base_link")

            #trans
            eX, eY, eZ = trans
            
            eX+= -0.3
            eY+= -.1
            
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

            
            omni_base.tiny_move( velX=0.2*+eX,velY=0.3*eY, velT=-eT,std_time=0.2, MAX_VEL=0.3) 

        rospy.sleep(2.0)
        clear_octo_client()
        rospy.sleep(1.0)
        print("MOVING ARM")    
        floor_pose=[0.05,-1.6,0.0,-1.41,0.0,0.0]
        arm.set_joint_value_target(floor_pose)
        arm.go()
        rospy.sleep(1.0)
        print("closing hand")
        gripper.close(0.04)
        rospy.sleep(3.0)
        succ=brazo.check_grasp()
        if succ:
            brazo.set_named_target('neutral')         
            return 'succ'

        #talk (' I think I missed the object, I will retry ')

        return 'failed'
        

        

#########################################################################################################
class Post_Pickup(smach.State):
    def __init__(self):
        smach.State.__init__(
            self, outcomes=['succ', 'failed', 'tries'])
        self.tries = 0    
    def execute(self, userdata):
        rospy.loginfo('State :  PICKUP ')
        clear_octo_client()
        self.tries += 1
        #talk('Rotating to previous human location found')
        rospy.sleep(0.7)
        """
        _,rot= tf_man.getTF("base_link",ref_frame='map')
        trans,_=tf_man.getTF("human",ref_frame="base_link")
        eX, eY, eZ = trans
        eT= tf.transformations.euler_from_quaternion(rot)[2] 
            
        omni_base.tiny_move( velX=0.1*+eX,velY=0.1*eY, velT=-eT,std_time=0.2, MAX_VEL=0.2)

        """
        
        omni_base.move_d_to(target_distance= 1.0, target_link='human')
        
        return 'succ'

#########################################################################################################
class Pickup_cereal(smach.State):
    def __init__(self):
        smach.State.__init__(
            self, outcomes=['succ', 'failed', 'tries'])
        self.tries = 0
    def execute(self, userdata):
        rospy.loginfo('State :  PICKUP ')
        clear_octo_client()  # TODO: Delete is TAKE
        self.tries += 1


        if self.tries >= 4:
            self.tries = 0
            return'tries'
        

        print ( 'x,y,',np.cos(np.deg2rad(pca_angle)), np.sin(np.deg2rad(pca_angle)) )
        #omni_base.tiny_move(velX=-0.2 *np.cos(np.deg2rad(pca_angle)) , velY= 0.1 * np.sin(np.deg2rad(pca_angle)) ,std_time=4.2, MAX_VEL=0.3) 

      













        clear_octo_client()
        print(pca_angle,max(min((pca_angle+90),-45),45),"####################AnGLE WITHIN LIMITS")
        #print(max(-np.deg2rad(pca_angle),-90),"####################ACNLGE WITHIN LIMITS")
        pickup_pose=[0.077,np.deg2rad(-70),np.deg2rad(-90), np.deg2rad(max(min((pca_angle+90),-45),45)),np.deg2rad(90), 0.0]
        print (pickup_pose)
        succ= arm.go(pickup_pose)
        rospy.sleep(1.0)
        succ=False
        while not succ:
            #trans,rot = tf_man.getTF(target_frame='static003_cracker_box', ref_frame='hand_palm_link')
            trans,_ = tf_man.getTF(target_frame='hand_palm_link', ref_frame='static003_cracker_box')
            _,rot = tf_man.getTF(target_frame='base_link', ref_frame='map')
            print (trans)
            print ('#############')
            if type(trans) is bool:return 'failed'
            if type(trans) is not bool:
                eX, eY, eZ = trans
                eT= tf.transformations.euler_from_quaternion(rot)[2] - 0.5*np.pi    #(known loc pickup change to line finder?)   NO! THERE ARE  ROUND TABLES !
                rospy.loginfo("Distance to goal: {:.2f}, {:.2f}, angle {:.2f}, target obj frame {}".format(eX, eY , eT,'cereal'))
                
                if abs(eX) < 0.05:
                    eX = 0
                if abs(eY) < 0.15:
                    eY = 0
                if abs(eT   ) < 0.1:
                    eT = 0
                succ =  eX == 0 and eY == 0 and eT==0
                    # grasp_base.tiny_move(velY=-0.4*trans[1], std_time=0.2, MAX_VEL=0.3)
                omni_base.tiny_move(velX=-0.1*eY, velY=0.1*eX, velT=-0.3*eT, std_time=0.2, MAX_VEL=0.3) #Pending test

        gripper.close()
        av= arm.get_current_joint_values()
        av[-2]+=-0.2
        arm.go(av)
        rospy.sleep(1.0)
 
        
              
        if succ:
            
            trans,_ = tf_man.getTF(target_frame='static003_cracker_box', ref_frame='hand_palm_link')
            
            print ('#############')
            print ('#############')
            print ('#############')
            print ('#############')
            print (trans, 'hand')
            print ('#############')
            print ('#############')
            print ('#############')
            print ('#############')
            print ('#############')
            
            return 'succ'

            
        return 'failed'

#########################################################################################################
"""
class Post_pickup(smach.State):
    def __init__(self):
        smach.State.__init__(
            self, outcomes=['succ', 'failed', 'tries'])
        self.tries = 0
    def execute(self, userdata):
        rospy.loginfo('State : Pre PICKUP ')
        clear_octo_client()
        self.tries += 1
        if self.tries >= 4:
            self.tries = 0
            return'tries'
        
        clear_octo_client()

        av=arm.get_current_joint_values()
        print (av,'av')
        av[0]=0.31
        succ=arm.go(av)
        omni_base.tiny_move(velX=-0.2, std_time=1.0, MAX_VEL=0.3) 
        
       
        if succ:
            return 'succ'
            
        return 'failed'
"""
#########################################################################################################
class Goto_table(smach.State):  
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed', 'tries'])
        self.tries = 0

    def execute(self, userdata):

        rospy.loginfo('STATE : Navigate to known location')
        arm.set_named_target('go')
        arm.go()
        


        self.tries += 1
        if self.tries == 3:
            return 'tries'

        if self.tries == 1: print("talk('Navigating to, pickup')")

        res = omni_base.move_base(known_location='table', time_out=200)
        print(res)

        if res:
            return 'succ'
        else:

            #talk('Navigation Failed, retrying')

            return 'failed'

#########################################################################################################
class Place_table(smach.State):
    def __init__(self):
        smach.State.__init__(
            self, outcomes=['succ', 'failed', 'tries'])
        self.tries = 0
    def execute(self, userdata):
        rospy.loginfo('State : PLACE TABLE ')
        clear_octo_client()
        self.tries += 1
        if self.tries >= 4:
            self.tries = 0
            return'tries'
        place_pose=[0.5, -0.9, 0.0, -1.9, 0.0, 0.0]
        succ=arm.go(place_pose)
        
        if succ:
            omni_base.tiny_move(velX=0.3, std_time=0.7, MAX_VEL=0.3) 
            
            shake=[0.45, -1.1    , 0.0, -1.9, 0.0, 0.0]
            arm.go(shake)                 
            rospy.sleep(0.3)           
            gripper.open()             
            rospy.sleep(0.3)           
            succ=arm.go(place_pose)
            omni_base.tiny_move(velX=-0.2, std_time=0.7, MAX_VEL=0.4) 
            res = omni_base.move_base(known_location='pca_table', time_out=200)
            

            return 'succ'
            
        return 'failed'

#########################################################################################################
class Goto_next_room(smach.State):  # ADD KNONW LOCATION DOOR
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed', 'tries'])
        self.tries = 0
        self.next_room=1

    def execute(self, userdata):

        rospy.loginfo('STATE : Navigate to known location')

       
        self.tries += 1
        
        
        
        if self.tries   <= 1: 

                    #talk('Navigating to bed room')
                    next_room='bedroom'
            
        elif self.tries == 2: 
                    #talk('Navigating to kitchen')
                    next_room='kitchen'
        elif self.tries == 3: 
                    #talk('Navigating to  living room')
                    next_room='living_room'
            
        elif self.tries == 4: 
                    #talk('Navigating to dining room')

                    next_room='dining_room'
                    self.tries=0
       

        res = omni_base.move_base(known_location=next_room)
        print(res)

        if res :


            return 'succ'

        else:

            #talk('Navigation Failed, retrying')

            self.tries +=-1
            return 'failed'

    
class Goto_human(smach.State):  
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed', 'tries'])
        self.tries = 0

    def execute(self, userdata):

        rospy.loginfo('STATE : Navigate to known location')

        print(f'Try {self.tries} of 3 attempts')
        self.tries += 1
        if self.tries == 3:
            return 'tries'
        if self.tries == 1: print("talk('Navigating to, living room')")
        res = omni_base.move_base(known_location='living_room', time_out=200)
        print(res)
        if res:
            return 'succ'
        else:
            #talk('Navigation Failed, retrying')
            return 'failed'

#########################################################################################################
###########################################################################################################

# --------------------------------------------------
def init(node_name):
   
    print('smach ready')
   

# --------------------------------------------------
# Entry point
if __name__ == '__main__':
    print("Takeshi STATE MACHINE...")
    init("takeshi_smach")
    # State machine, final state "END"
    sm = smach.StateMachine(outcomes=['END'])

    # sm.userdata.clear = False
    sis = smach_ros.IntrospectionServer('SMACH_VIEW_SERVER', sm, '/SM_STICKLER')
    sis.start()

    with sm:
        # State machine STICKLER
        smach.StateMachine.add("INITIAL",           Initial(),              transitions={'failed': 'INITIAL',           
                                                                                         'succ': 'WAIT_PUSH_HAND',   
                                                                                         'tries': 'END'})
        smach.StateMachine.add("WAIT_PUSH_HAND",    Wait_push_hand(),       transitions={'failed': 'WAIT_PUSH_HAND',    
                                                                                         'succ': 'GOTO_LIVING_ROOM',       
                                                                                         'tries': 'END'})
        smach.StateMachine.add("GOTO_LIVING_ROOM",  Goto_living_room(),     transitions={'failed': 'GOTO_LIVING_ROOM',        
                                                                                         'succ': 'FIND_HUMAN',   
                                                                                         'tries': 'INITIAL'})  
        smach.StateMachine.add("FIND_HUMAN",         Find_human(),          transitions={'failed': 'FIND_HUMAN',        
                                                                                         'succ': 'SCAN_FLOOR',   
                                                                                         'tries': 'GOTO_LIVING_ROOM'})
        smach.StateMachine.add("SCAN_FLOOR",         Scan_floor(),          transitions={'failed': 'SCAN_FLOOR',     
                                                                                         'succ': 'PRE_PICKUP',    
                                                                                         'tries': 'END'})   
        smach.StateMachine.add("PRE_PICKUP",        Pre_pickup(),           transitions={'failed': 'PRE_PICKUP',        
                                                                                         'succ': 'PICKUPTWO',  
                                                                                         'tries': 'END'})        
        smach.StateMachine.add("PICKUP",            Pickup(),               transitions={'failed': 'PICKUP',        
                                                                                         'succ': 'GOTO_HUMAN',   
                                                                                         'tries': 'END'})        
        smach.StateMachine.add("PICKUPTWO",         Pickup_two(),           transitions={'failed': 'PICKUPTWO',        
                                                                                         'succ': 'POST_PICKUP',   
                                                                                         'tries': 'END'})    
        smach.StateMachine.add("POST_PICKUP",       Post_Pickup(),          transitions={'failed': 'END',        
                                                                                         'succ': 'END',   
                                                                                         'tries': 'END'})   
        smach.StateMachine.add("GOTO_HUMAN",         Goto_human(),          transitions={'failed': 'GOTO_HUMAN',        'succ': 'END' ,   'tries': 'END' })    
    
        #smach.StateMachine.add("GOTO_HUMAN", SimpleActionState('follow_server', FollowAction) ,transitions={'aborted': 'GOTO_HUMAN',        
        #                                                                                                  'succeeded': 'END' ,   
        #                                                                                                  'preempted': 'END' })
        #smach.StateMachine.add("PICKUP_CEREAL",     Pickup_cereal(),        transitions={'failed': 'PICKUP_CEREAL',        'succ': 'POST_PICKUP'    ,   'tries': 'END'})        
        #smach.StateMachine.add("POST_PICKUP",        Post_pickup(),         transitions={'failed': 'POST_PICKUP',        'succ': 'GOTO_TABLE'    ,   'tries': 'END'})        
        #smach.StateMachine.add("GOTO_TABLE",        Goto_table(),           transitions={'failed': 'GOTO_TABLE',        'succ': 'PLACE_TABLE'    ,   'tries': 'END'})        
        #smach.StateMachine.add("PLACE_TABLE",     Place_table(),      transitions={'failed': 'GOTO_TABLE',    'succ': 'SCAN_TABLE'    ,   'tries': 'INITIAL'})

        #smach.StateMachine.add("FIND_HUMAN",         Find_human(),          transitions={'failed': 'FIND_HUMAN',        'succ': 'DETECT_DRINK'    ,   'tries': 'GOTO_NEXT_ROOM', 'forbidden':'GOTO_HUMAN'})
        #smach.StateMachine.add("GOTO_HUMAN",         Goto_human(),          transitions={'failed': 'GOTO_HUMAN',        'succ': 'ANALYZE_HUMAN' ,   'tries': 'FIND_HUMAN' , 'forbidden':'LEAD_TO_LIVING_ROOM'})
        #smach.StateMachine.add("LEAD_TO_LIVING_ROOM",Lead_to_living_room(), transitions={'failed': 'LEAD_TO_LIVING_ROOM','succ': 'GOTO_NEXT_ROOM',  'tries': 'END'})
        #smach.StateMachine.add("ANALYZE_HUMAN",      Analyze_human(),       transitions={'failed': 'FIND_HUMAN',        'succ': 'ANALYZE_AREA'})
        #smach.StateMachine.add("ANALYZE_AREA",      Analyze_area(),         transitions={'failed': 'ANALYZE_AREA' ,     'succ': 'GOTO_NEXT_ROOM' ,    'tries': 'GOTO_NEXT_ROOM'})   #
        ##################################################################################################################################################################
        #smach.StateMachine.add("DETECT_DRINK",         Detect_drink(),      transitions={'failed': 'GOTO_NEXT_ROOM',    'succ': 'GOTO_HUMAN'  , 'tries': 'FIND_HUMAN'})
        ##################################################################################################################################################################
        ###################################################################################################################################################################
        


    outcome = sm.execute()
