#!/usr/bin/env python3
from smach_utils_justina import *  # from smach_utils2 import *
# from smach_ros import SimpleActionState


def categorize_objs(name):
    # 'fork', 'knife', 'mug', 'sponge', 'b_cups', 'c_cups', 'e_cups', 'f_cups', 'dice', 'marker', 'rubiks_cube'
    kitchen = ['bowl', 'fork', 'knife', 'mug', 'plate', 'spatula', 'sponge', 'spoon', 'b_cups', 'c_cups', 'e_cups', 'f_cups']
    # 'extra_large_clamp', 'large_clamp', 'small_clamp', 'medium_clamp'
    tools = ['adjustable_wrench', 'flat_screwdriver', 'phillips_screwdriver', 'wood_block']
    # 'softball', 'a_mini_soccer_ball', 'racquetball', 'golf_ball',
    balls = ['tennis_ball', 'baseball']
    fruits = ['apple', 'banana', 'lemon', 'pear']  # 'plum', 'orange'
    food = ['chips_can', 'mustard_bottle', 'potted_meat_can', 'tomato_soup_can', 'tuna_fish_can', 'pudding_box', 'cracker_box']  # 'master_chef_can', 'sugar_box'
    other = ['dice', 'marker', 'rubiks_cube']
    if name in kitchen:
        return 'kitchen'
    elif name in tools:
        return 'tools'
    elif name in balls:
        return 'balls'
    elif name in fruits:
        return 'fruits'
    elif name in food:
        return 'food'
    elif name in other:
        return 'other'
    return 'unknown'


class Initial(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed', 'tries'],
                             output_keys=['l_arm_home'], input_keys=['l_arm_home'])
        self.tries = 0

    def execute(self, userdata):
        print('\n> STATE <: INITIAL')
        self.tries += 1
        print(f'Try {self.tries} of 3 attempts')
        userdata.l_arm_home = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        if self.tries == 3:
            rospy.logerr("Error in state INITIAL")
            return 'tries'

        # READ YAML ROOMS XYS   objs csv, known initial object locations
        global arm, objs
        rospack = rospkg.RosPack()
        file_path = rospack.get_path('config_files')
        print(file_path)
        objs = pd.read_csv(file_path + '/objects.csv')
        objs = objs.drop(columns='Unnamed: 0')
        print(objs)

        print('Robot neutral pose')
        #arm = moveit_commander.MoveGroupCommander('arm')  # TODO: Delete
        JuskeshinoHardware.moveLeftArmWithTrajectory(userdata.l_arm_home, 6)
        head.set_named_target('neutral')
        rospy.sleep(0.8)

        return 'succ'


class Wait_push_hand(smach.State):
    pass


class Goto_pickup(smach.State):
    def __init__(self):
        smach.State.__init__(
            self, outcomes=['succ', 'failed', 'tries', 'pickup'])
        self.tries = 0
        self.time_out = 30
        self.attempts = 3

    def execute(self, userdata):
        
        self.tries += 1
        
        if self.tries == 1:
            self.time_out = 30
            print("\n")
            rospy.logwarn('--> STATE <: Navigate to pickup')
            print("Navigating to, pickup")
            voice.talk('Navigating to, pickup')
        
        print(f'Try {self.tries} of {self.attempts} attempts')
        if self.tries == 3:
            rospy.logerr('Navigation Failed, I can not reach the objective')
            voice.talk('Navigation Failed, I can not reach the objective')
            self.tries = 0
            return 'failed'

        res = omni_base.move_base(known_location='pickup', time_out=self.time_out)
        # First TIme Only go

        if res == 3:  # Success
            self.tries = 0
            return 'succ'
        elif res == 1 or res == 8:
            if self.tries < self.attempts-1:
                rospy.logerr('Navigation Failed, retrying')
                voice.talk('Navigation Failed, retrying')
            self.time_out = 25
            return 'tries'
        else:
            rospy.logerr('Navigation Failed, I can not reach the objective')
            voice.talk('Navigation Failed, I can not reach the objective')
            return 'failed'


        # if self.tries == 1:
        #     print('Navigating to, pickup')
        #     voice.talk('Navigating to, pickup')
        #     if res:
        #         return 'succ'
        #     else:
        #         print('Navigation Failed, retrying')
        #         voice.talk('Navigation Failed, retrying')
        #         return 'failed'

        # if self.tries > 1:
        #     if res:
        #         return 'pickup'  # TODO: Is needed?
        #     else:
        #         print('Navigation Failed, retrying')
        #         voice.talk('Navigation Failed, retrying')
        #         return 'failed'


class Scan_table(smach.State):
    def __init__(self):
        smach.State.__init__(
            self, outcomes=['succ', 'failed', 'tries'])
        self.tries = 0

    def execute(self, userdata):
        self.tries += 1
        if self.tries == 1:
            print("\n")
            rospy.logwarn('--> STATE <: Scanning table')
            head.set_joint_values([0.0, -0.5])
            voice.talk('Scanning table')
            return 'failed'  # TODO: Remove
        
        if self.tries == 2:
            head.set_joint_values([0.2, -0.7])
        
        if self.tries >= 3:
            self.tries = 0
            return 'succ'
        global objs
        rospy.sleep(3.0)
        img_msg = bridge.cv2_to_imgmsg(rgbd.get_image())
        req = classify_client.request_class()
        req.in_.image_msgs.append(img_msg)
        res = classify_client(req)
        objects = detect_object_yolo('all', res)
        # <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
        if len(objects) != 0:
            for i in range(len(res.poses)):
                # tf_man.getTF("head_rgbd_sensor_rgb_frame")
                position = [res.poses[i].position.x,
                            res.poses[i].position.y, res.poses[i].position.z]
                print('position,name', position, res.names[i].data[4:])
                ##########################################################
                object_point = PointStamped()
                object_point.header.frame_id = "head_rgbd_sensor_rgb_frame"
                object_point.point.x = position[0]
                object_point.point.y = position[1]
                object_point.point.z = position[2]
                position_map = tfBuffer.transform(
                    object_point, "map", timeout=rospy.Duration(1))
                print('position_map', position_map)
                tf_man.pub_static_tf(pos=[position_map.point.x, position_map.point.y, position_map.point.z], rot=[
                                     0, 0, 0, 1], ref="map", point_name=res.names[i].data[4:])
                new_row = {'x': position_map.point.x, 'y': position_map.point.y,
                           'z': position_map.point.z, 'obj_name': res.names[i].data[4:]}
                objs.loc[len(objs)] = new_row
                ###########################################################

        else:
            print('Objects list empty')
            return 'failed'
        rospack = rospkg.RosPack()
        file_path = rospack.get_path('config_files')+'/regions'
        regions = {'shelves': np.load(
            file_path+'/shelves_region.npy'), 'pickup': np.load(file_path+'/pickup_region.npy')}

        def is_inside(x, y): return ((area_box[:, 1].max() > y) and (area_box[:, 1].min(
        ) < y)) and ((area_box[:, 0].max() > x) and (area_box[0, 0].min() < x))
        for name in regions:
            in_region = []
            area_box = regions[name]
            for index, row in objs[['x', 'y']].iterrows():
                in_region.append(is_inside(row.x, row.y))
            objs[name] = pd.Series(in_region)
        cats = []
        for name in objs['obj_name']:
            cats.append(categorize_objs(name))
        objs['category'] = cats

        return 'failed'


class Pickup(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed', 'tries'])
        self.tries = 0

    def execute(self, userdata):
        global cat, target_object
        rospy.loginfo('STATE : PICKUP')
        rob_pos, _ = tf_man.getTF('base_link')
        pickup_objs = objs[objs['pickup'] == True]
        pickup_objs = pickup_objs[pickup_objs['z'] > 0.65]
        print('pickup_objs', pickup_objs)
        ix = np.argmin(np.linalg.norm(
            rob_pos-pickup_objs[['x', 'y', 'z']]  .values, axis=1))
        name, cat = pickup_objs[['obj_name', 'category']].iloc[ix]
        print('closest', name, cat)
        print(objs)
        voice.talk(f'closest pickup object is {name,cat}')
        ##################################################
        clear_octo_client()
        _, rot = tf_man.getTF("base_link", ref_frame='map')
        original_rot = tf.transformations.euler_from_quaternion(rot)[2]
        succ = False
        target_object = name
        while not succ and not rospy.is_shutdown():

            _, rot = tf_man.getTF("base_link", ref_frame='map')
            trans, _ = tf_man.getTF(target_object, ref_frame="base_link")

            # trans
            eX, eY, eZ = trans

            eX += -0.41  # REAL
            # eX+= -0.46  #GAZEBO

            eY += -.06  # REAL
            # eY+= -.1 #GAZEBO

            eT = tf.transformations.euler_from_quaternion(
                rot)[2] - original_rot  # Original
            print(eT)
            if eT > np.pi:
                eT = -2*np.pi+eT
            if eT < -np.pi:
                eT = 2*np.pi+eT
            rospy.loginfo("error: {:.2f}, {:.2f}, angle {:.2f}, target obj frame {}".format(
                eX, eY, eT, target_object))
            X, Y, Z = trans
            rospy.loginfo("Pose: {:.2f}, {:.2f}, angle {:.2f}, target obj frame {}".format(
                X, Y, eT, target_object))

            if abs(eX) <= 0.05:
                print('here')
                eX = 0
            if abs(eY) <= 0.05:
                eY = 0
            if abs(eT) < 0.1:
                eT = 0
            succ = eX == 0 and eY == 0 and eT == 0
            omni_base.tiny_move(velX=0.2*+eX, velY=0.3*eY,
                                velT=-eT, std_time=0.2, MAX_VEL=0.3)

        trans, _ = tf_man.getTF(target_object, ref_frame="base_link")
        pickup_pose = [min(trans[2], 0.66), -1.2, 0.0, -1.9, 0.0, 0.0]
        # pickup_pose=[0.65,-1.2,0.0,-1.9, 0.0, 0.0]
        succ = arm.go(pickup_pose)
        gripper.open()
        clear_octo_client()
        av = arm.get_current_joint_values()
        pose, _ = tf_man.getTF(
            target_frame='hand_palm_link', ref_frame=target_object)
        av[0] += 0.07-pose[2]
        # arm.go(av)
        # print (av)
        brazo.set_joint_values(av[:-1])
        rospy.sleep(0.5)
        gripper.close(force=0.06)
        succ = brazo.check_grasp()
        if succ:
            av = arm.get_current_joint_values()
            av[0] += 0.15
            arm.go(av)
            omni_base.tiny_move(velX=-0.3, std_time=4.0)
            arm.set_named_target('go')
            arm.go()
            objs.drop(objs[objs['obj_name'] ==
                      target_object].index, inplace=True)
        return 'succ'


class Goto_shelf(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed', 'tries'])
        self.tries = 0

    def execute(self, userdata):

        rospy.loginfo('STATE : Navigate to known location')

        print(f'Try {self.tries} of 3 attempts')
        self.tries += 1
        if self.tries == 3:
            return 'tries'
        if self.tries == 1:
            voice.talk('Navigating to, shelf')
        res = omni_base.move_base(known_location='shelf', time_out=200)
        print(res)

        if res:
            return 'succ'
        else:
            voice.talk('Navigation Failed, retrying')
            return 'failed'


class Goto_place_shelf(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed', 'tries'])
        self.tries = 0

    def execute(self, userdata):

        rospy.loginfo('STATE : Navigate to known location')

        print(f'Try {self.tries} of 3 attempts')
        self.tries += 1
        if self.tries == 3:
            return 'tries'
        arm.set_named_target('neutral')
        arm.set_named_target('go')
        arm.go()
        if self.tries == 1:
            voice.talk('Navigating to, shelf')
        res = omni_base.move_base(known_location='place_shelf', time_out=200)
        print(res)

        if res:
            return 'succ'
        else:
            voice.talk('Navigation Failed, retrying')
            return 'failed'


class Place_shelf(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed', 'tries'])
        self.tries = 0

    def execute(self, userdata):

        rospy.loginfo('STATE : Placing in shelf')

        print(f'Try {self.tries} of 3 attempts')
        self.tries += 1

        print(f'shelves_cats{shelves_cats}, object picked up cat {cat}')
        high_shelf_place = [0.1505, -0.586, 0.0850, -0.934, 0.0220, 0.0]
        mid_shelf_place = [0.2556, -1.6040, -0.0080, -0.0579, 0.0019, 0.0]
        low_shelf_place = [0.0457, -2.2625, 0.0, 0.7016, -0.0, 0.0]
        placing_poses = np.asarray(
            (high_shelf_place, mid_shelf_place, low_shelf_place))
        placing_places = np.asarray(
            ('placing_area_high_shelf1', 'placing_area_mid_shelf1', 'placing_area_low_shelf1'))

        if cat in shelves_cats:
            ind = np.argmax(shelves_cats == cat)
            placing_pose = placing_poses[ind]
            placing_place = placing_places[ind]
        else:
            print('No category found, placing at random')
            ind = np.random.randint(0, len(placing_poses))
            placing_pose = placing_poses[ind]
            placing_place = placing_places[ind]
        clear_octo_client()
        succ = arm.go(placing_pose)
        intended_placing_area = tf_man.getTF(placing_place)
        print(
            f'#placing_poses {placing_poses}, index{ind} , placing_places{placing_places} , placing_place {placing_place}')
        print('################################')
        print('################################')
        print('################################')
        print('################################')
        print('################################')
        print('################################')
        print(
            f'###########intended_placing_area{intended_placing_area}#####################')
        base_grasp_D(tf_name=placing_place, d_x=0.76, d_y=0.0, timeout=30)
        succ = arm.go(placing_pose)
        base_grasp_D(tf_name=placing_place, d_x=0.6, d_y=0.0, timeout=30)
        gripper.open()
        rospy.sleep(1.0)

        if succ:
            return 'succ'
        return 'failed'


class Scan_shelf(smach.State):
    def __init__(self):
        smach.State.__init__(
            self, outcomes=['succ', 'failed', 'tries'])
        self.tries = 0

    def execute(self, userdata):
        global shelves_cats
        rospy.loginfo('State : Scanning_shelf')
        voice.talk('Scanning shelf')
        print("talk('Scanning shelf')")
        request = segmentation_server.request_class()
        area_number = 0

        self.tries += 1
        if self.tries >= 4:
            self.tries = 0
            # REMOVE THIS LINE; ONLY FOR DEBGNG PURPOSES
            # regions={'shelves':np.load('/home/roboworks/Documents/shelf_sim.npy'),'pickup':np.load('/home/roboworks/Documents/pickup_sim.npy')}
            rospack = rospkg.RosPack()
            file_path = rospack.get_path('config_files')
            regions = {'shelves': np.load(
                file_path+'/shelves_region.npy'), 'pickup': np.load(file_path+'/pickup_region.npy')}

            def is_inside(x, y): return ((area_box[:, 1].max() > y) and (area_box[:, 1].min(
            ) < y)) and ((area_box[:, 0].max() > x) and (area_box[0, 0].min() < x))
            for name in regions:
                in_region = []
                area_box = regions[name]
                for index, row in objs[['x', 'y']].iterrows():
                    in_region.append(is_inside(row.x, row.y))
                objs[name] = pd.Series(in_region)
            cats = []
            for name in objs['obj_name']:
                cats.append(categorize_objs(name))
            objs['category'] = cats
            print('################################')
            shelf_objs = objs[objs['shelves'] == True]

            sh_o_btm = shelf_objs[shelf_objs['z'] < 0.15]
            # sh_o_mdl=shelf_objs[(shelf_objs['z']>0.2 )& (shelf_objs['z']<0.5) ] #SIM
            sh_o_mdl = shelf_objs[(shelf_objs['z'] > 0.2)
                                  & (shelf_objs['z'] < 0.7)]
            sh_o_top = shelf_objs[(shelf_objs['z'] > 0.7)]

            print('sh_o_btm,sh_o_mdl,sh_o_top', sh_o_btm, sh_o_mdl, sh_o_top)
            shelves_cats = []

            a = sh_o_btm['category'].value_counts()
            if 'other' in sh_o_btm['category'].values:
                a.drop('other', inplace=True)
            if len(a.values) != 0:
                print(f'btm shelf category {a.index[a.argmax()]}')
                shelves_cats.append(a.index[a.argmax()])
            a = sh_o_mdl['category'].value_counts()
            if 'other' in sh_o_mdl['category'].values:
                a.drop('other', inplace=True)
            if len(a.values) != 0:
                print(f'middle shelf category {a.index[a.argmax()]}')
                shelves_cats.append(a.index[a.argmax()])
            a = sh_o_top['category'].value_counts()
            if 'other' in sh_o_top['category'].values:
                a.drop('other', inplace=True)
            if len(a.values) != 0:
                print(f'top shelf category {a.index[a.argmax()]}')
                shelves_cats.append(a.index[a.argmax()])

            shelves_cats = np.asarray(shelves_cats)
            objs.to_csv('/home/roboworks/Documents/objs.csv')
            print('################################')
            print('################################')
            print('################################')
            print('################################')
            print('################################')
            print('################################')
            print('################################')
            print('################################')
            ################################
            return 'succ'
        if self.tries == 1:
            head.set_named_target('neutral')
            av = arm.get_current_joint_values()
            av[0] = 0.5
            av[1] = -0.5
            arm.go(av)
            head.set_joint_values([-np.pi/2, -0.5])
            request.height.data = 0.87  # TOP SHELF FOR PLACING
            area_name_numbered = 'high_shelf'
            rospy.sleep(1.3)
        if self.tries == 2:
            av = arm.get_current_joint_values()
            av[0] = 0.05
            arm.go(av)
            request.height.data = 0.41  # MiD SHELF FOR PLACING
            area_name_numbered = 'mid_shelf'
            rospy.sleep(1.3)
        if self.tries == 3:
            head.set_joint_values([-np.pi/2, -0.8])
            av = arm.get_current_joint_values()
            av[0] = 0.0
            av[1] = -0.5
            arm.go(av)
            request.height.data = 0.00  # BTM SHELF FOR PLACING
            area_name_numbered = 'low_shelf'
            rospy.sleep(1.3)
        rospy.sleep(1.3)
        # 3

        res = placing_finder_server.call(request)
        # succ=seg_res_tf(res)
        print(f'Placing Area at {res.poses.data}')
        area_number += 1
        area_name_numbered += str(area_number)
        tf_man.pub_static_tf(pos=[res.poses.data[0], res.poses.data[1], res.poses.data[2]], rot=[
                             0, 0, 0, 1], point_name=f'placing_area_{area_name_numbered}')
        ############### OBJECTS YOLO#######################
        img_msg = bridge.cv2_to_imgmsg(rgbd.get_image())
        req = classify_client.request_class()
        req.in_.image_msgs.append(img_msg)
        res = classify_client(req)
        objects = detect_object_yolo('all', res)
        # <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
        if len(objects) != 0:
            for i in range(len(res.poses)):
                # tf_man.getTF("head_rgbd_sensor_rgb_frame")
                position = [res.poses[i].position.x,
                            res.poses[i].position.y, res.poses[i].position.z]
                print('position,name', position, res.names[i].data[4:])
                ##########################################################
                object_point = PointStamped()
                object_point.header.frame_id = "head_rgbd_sensor_rgb_frame"
                object_point.point.x = position[0]
                object_point.point.y = position[1]
                object_point.point.z = position[2]
                position_map = tfBuffer.transform(
                    object_point, "map", timeout=rospy.Duration(1))
                print('position_map', position_map)
                tf_man.pub_static_tf(pos=[position_map.point.x, position_map.point.y, position_map.point.z], rot=[
                                     0, 0, 0, 1], ref="map", point_name=res.names[i].data[4:])
                new_row = {'x': position_map.point.x, 'y': position_map.point.y,
                           'z': position_map.point.z, 'obj_name': res.names[i].data[4:]}
                objs.loc[len(objs)] = new_row
                ###########################################################

        else:
            print('Objects list empty')
            return 'failed'
        # >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
        if len(objects) != 0:
            for i in range(len(res.poses)):
                tf_man.getTF("head_rgbd_sensor_rgb_frame")
                position = [res.poses[i].position.x,
                            res.poses[i].position.y, res.poses[i].position.z]
                tf_man.pub_static_tf(pos=position, rot=[
                                     0, 0, 0, 1], ref="head_rgbd_sensor_rgb_frame", point_name=res.names[i].data[4:])
                rospy.sleep(0.3)
                tf_man.change_ref_frame_tf(res.names[i].data[4:])
                rospy.sleep(0.3)
                pose, _ = tf_man.getTF(res.names[i].data[4:])
                if type(pose) != bool:
                    new_row = {
                        'x': pose[0], 'y': pose[1], 'z': pose[2], 'obj_name': res.names[i].data[4:]}
                    objs.loc[len(objs)] = new_row
        else:
            print('Objects list empty')
            return 'failed'
        # >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
        print(objs)

        return 'failed'


class Check_grasp(smach.State):
    def __init__(self):
        smach.State.__init__(self, output_keys=['target_pose'], outcomes=[
                             'succ', 'failed', 'tries'])
        self.tries = 0

    def execute(self, userdata):
        self.tries += 1
        if self.tries >= 4:
            self.tries = 0
            return 'tries'
        rospy.loginfo('STATE : Check Grasp')
        omni_base.tiny_move(velX=-0.2, std_time=4.2)

        arm.set_named_target('go')

        arm.go()
        head.to_tf(target_object)
        rospy.sleep(1.0)
        img_msg = bridge.cv2_to_imgmsg(cv2.cvtColor(
            rgbd.get_image(), cv2.COLOR_RGB2BGR))  # GAZEBO BGR!?!??!
        req = classify_client.request_class()
        req.in_.image_msgs.append(img_msg)
        res = classify_client(req)
        objects = detect_object_yolo('all', res)

        def check_if_grasped(pose_target, test_pt, tolerance=0.05): return np.linalg.norm(
            pose_target-test_pt) < tolerance
        ##############################
        pose_target, _ = tf_man.getTF(target_object)
        #########################

        if len(objects) != 0:
            for i in range(len(res.poses)):

                position = [res.poses[i].position.x,
                            res.poses[i].position.y, res.poses[i].position.z]

                object_point = PointStamped()
                object_point.header.frame_id = "head_rgbd_sensor_rgb_frame"
                object_point.point.x = position[0]
                object_point.point.y = position[1]
                object_point.point.z = position[2]
                position_map = tfBuffer.transform(
                    object_point, "map", timeout=rospy.Duration(1))

                tf_man.pub_static_tf(pos=[position_map.point.x, position_map.point.y, position_map.point.z], rot=[
                                     0, 0, 0, 1], ref="map", point_name=res.names[i].data[4:])
                new_row = {'x': position_map.point.x, 'y': position_map.point.y,
                           'z': position_map.point.z, 'obj_name': res.names[i].data[4:]}
                objs.loc[len(objs)] = new_row

                test_pt = np.asarray(
                    (position_map.point.x, position_map.point.y, position_map.point.z))
                print(np.linalg.norm(pose_target-test_pt))
                if check_if_grasped(pose_target, test_pt):
                    print(
                        f'Centroid found in area {test_pt}, obj_name: {res.names[i].data[4:]}')
                    print('Grasping May have failed')
                    print(
                        f'recalling grasp action on coordinates{test_pt} wrt map, converting to odom and action goal slot  ')
                    pos, _ = tf_man.getTF(
                        target_frame=target_object, ref_frame='odom')
                    target_pose = Float32MultiArray()
                    pos[2] += 0.03
                    target_pose.data = pos
                    userdata.target_pose = target_pose
                    ###################
                    head.set_named_target('neutral')
                    rospy.sleep(0.5)
                    clear_octo_client()
                    return 'failed'
        objs.drop(objs[objs['obj_name'] == target_object].index, inplace=True)
        self.tries = 0
        return 'succ'


# --------------------------------------------------
if __name__ == '__main__':
    print("Justina STATE MACHINE...")
    # State machine, final state "END"
    sm = smach.StateMachine(outcomes=['END'])

    # sm.userdata.clear = False
    sis = smach_ros.IntrospectionServer('SMACH_VIEW_SERVER', sm, '/SM_STORING')
    sis.start()

    with sm:
        # State machine STICKLER
        smach.StateMachine.add("INITIAL", Initial(),
                               transitions={'failed':'INITIAL', 'succ':'GOTO_PICKUP', 'tries':'END'})

        # smach.StateMachine.add("WAIT_PUSH_HAND", Wait_push_hand(),
        #                        transitions={'failed':'WAIT_PUSH_HAND', 'succ':'GOTO_PICKUP', 'tries':'END'})

        smach.StateMachine.add("GOTO_PICKUP", Goto_pickup(),
                               transitions={'failed':'END', 'succ':'SCAN_TABLE', 'tries':'GOTO_PICKUP', 'pickup':'PICKUP'})

        smach.StateMachine.add("SCAN_TABLE", Scan_table(),
                               transitions={'failed':'SCAN_TABLE', 'succ':'GOTO_PICKUP', 'tries':'GOTO_PICKUP'})
                               #transitions={'failed':'SCAN_TABLE', 'succ':'GOTO_PICKUP', 'tries':'GOTO_PICKUP'})

        smach.StateMachine.add("GOTO_SHELF", Goto_shelf(),
                               transitions={'failed':'GOTO_SHELF', 'succ':'SCAN_SHELF', 'tries':'GOTO_SHELF'})

        smach.StateMachine.add("GOTO_PLACE_SHELF", Goto_place_shelf(),
                               transitions={'failed':'GOTO_PLACE_SHELF', 'succ':'PLACE_SHELF', 'tries':'GOTO_SHELF'})

        smach.StateMachine.add("PLACE_SHELF", Place_shelf(),
                               transitions={'failed':'SCAN_SHELF', 'succ':'GOTO_PICKUP', 'tries':'GOTO_SHELF'})

        smach.StateMachine.add("SCAN_SHELF", Scan_shelf(),
                               transitions={'failed':'SCAN_SHELF', 'succ':'GOTO_PLACE_SHELF', 'tries':'END'})

        smach.StateMachine.add("PICKUP", Pickup(),
                               transitions={'failed':'PICKUP', 'succ':'CHECK_GRASP', 'tries':'GOTO_PICKUP'})

        smach.StateMachine.add("CHECK_GRASP", Check_grasp(),
                               transitions={'failed':'PICKUP', 'succ':'GOTO_SHELF', 'tries':'SCAN_TABLE'})
                               

    outcome = sm.execute()
