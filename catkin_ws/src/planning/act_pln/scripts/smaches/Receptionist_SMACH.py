#!/usr/bin/env python3
from smach_utils_justina import *
# from smach_utils_receptionist import *

##### Define state INITIAL #####

# --------------------------------------------------
guest_name = ""  # TODO: Delete

class Initial(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed', 'tries'])
        self.tries = 0

    def execute(self, userdata):

        rospy.loginfo('STATE : INITIAL')
        print('Robot neutral pose')
        self.tries += 1
        print(f'Try {self.tries} of 5 attempts')

        # clean_knowledge()
        # places_2_tf()

        # -----INIT GRAMMAR FOR VOSK
        # -----Use with get_keywords_speech()
        # -----------SPEECH REC
        # drinks=['coke','juice','beer', 'water', 'soda', 'wine', 'i want a', 'i would like a']
        drinks = ['coke', 'juice', 'milk', 'water', 'soda', 'wine', 'i want a',
                  'i would like a', 'tea', 'icedtea', 'cola', 'redwine', 'orangejuice', 'tropicaljuice']
        # names=['rebeca','ana','jack', 'michael', ' my name is' , 'i am','george','mary','ruben','oscar','yolo','mitzi']
        names = [' my name is', 'i am', 'adel', 'angel', 'axel', 'charlie',
                 'jane', 'john', 'jules', 'morgan', 'paris', 'robin', 'simone']
        confirmation = ['yes', 'no', 'robot yes',
                        'robot no', 'not', 'now', 'nope', 'yeah']
        gram = drinks+names+confirmation
        if self.tries == 1:
            # set_grammar(gram)  ##PRESET DRINKS
            print('SMACH STARTING')
        elif self.tries == 3:
            return 'tries'

        print(drinks)

        # rospy.sleep(0.8)
        return 'succ'

# --------------------------------------------------


class Wait_push_hand(smach.State):
    pass

# --------------------------------------------------


class Wait_door_opened(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed', 'tries'])
        self.tries = 0

    def execute(self, userdata):
        rospy.loginfo('STATE : Wait for door to be opened')
        print('Waiting for door to be opened')

        self.tries += 1
        print(f'Try {self.tries} of 4 attempts')

        if self.tries == 100:
            return 'tries'
        voice.talk('I am ready for receptionist task.')
        rospy.sleep(0.8)
        voice.talk('I am waiting for the door to be opened')
        msg = rospy.wait_for_message('/hardware/scan', LaserScan)
        if np.mean(msg.ranges[(int)(len(msg.ranges)*0.5 - 10):(int)(len(msg.ranges)*0.5 + 10)]) < 1.0:
            return 'failed'  # DOOR
        self.tries = 0
        return 'succ'


class Goto_door(smach.State):  # ADD KNONW LOCATION DOOR
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed', 'tries'])
        self.tries = 0

    def execute(self, userdata):

        rospy.loginfo('STATE : Navigate to known location: Door')

        print(f'Try {self.tries} of 3 attempts')
        self.tries += 1
        if self.tries == 3:
            return 'tries'
        if self.tries == 1:
            print("Navigating to, door")
            voice.talk('Navigating to, door')
        res = omni_base.move_base(known_location='door')
        print(res)

        if res:
            self.tries = 0
            return 'succ'
        else:
            voice.talk('Navigation Failed, retrying')
            return 'failed'

# --------------------------------------------------


class Scan_face(smach.State):
    def __init__(self):
        smach.State.__init__(
            self, outcomes=['succ', 'unknown', 'failed', 'tries'])
        self.tries = 0

    def execute(self, userdata):
        global img_face, name_face

        rospy.loginfo('State : Scan face')
        head.set_named_target('face_to_face')
        rospy.sleep(0.3)
        # head.set_joint_values([0.0, 0.3])  # TODO: delete
        print("voice.talk('Scanning for faces, look at me, please')")
        voice.talk('Scanning for faces, look at me, please')
        self.tries += 1
        if self.tries >= 4:
            self.tries = 0
            return 'tries'
        name_face = ""
        res, img_face = wait_for_face()  # default 10 secs
        rospy.sleep(0.7)
        print(res)

        print('Checking for faces')
        print(
            "voice.talk('When confirmation is needed, please say: robot yes or robot no')")
        voice.talk(
            'When confirmation is needed. please say, robot yes or robot no')
        rospy.sleep(0.3)
        print("voice.talk('Please only talk, when my leds are white')")
        rospy.sleep(0.3)
        if res != None:
            name = res.Ids.ids

            print('RESPONSE', name)
            if name == 'NO_FACE':
                print('No face Found, Keep scanning')
                print("voice.talk('I did not see you, I will try again')")
                voice.talk('I did not see you, I will try again')
                return 'failed'

            elif name == 'unknown':
                print('A face was found.')
                print("voice.talk('I believe we have not met. ')")
                voice.talk('I believe we have not met. ')
                self.tries = 0
                return 'unknown'

            else:
                print("voice.talk(f'I found you, I Think you are, {name}.')")
                voice.talk(f'I found you, I Think you are, {name}.')
                print("voice.talk('Is it correct?')")
                voice.talk('Is it correct?')
                print("voice.talk('Robot yes or robot no')")
                voice.talk('Robot yes or robot no')
                rospy.sleep(2.5)
                print('LEDS WHITE')
                confirmation = get_keywords_speech(10)
                print(confirmation)

                if confirmation not in ['yes', 'jack', 'juice', 'takeshi yes', 'yeah']:
                    return 'unknown'
                elif confirmation == "timeout":
                    print(
                        "voice.talk('I could not hear you, lets try again, please speak louder.')")
                    voice.talk(
                        'I could not hear you, lets try again, please speak louder.')
                    return "failed"
                else:
                    name_face = name
                    self.tries = 0
                    global guest_name  # TODO: delete 
                    guest_name = name  # TODO: delete
                    return 'succ'
        else:
            return 'failed'

# --------------------------------------------------


class New_face(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed', 'tries'])
        self.new_name = ''
        self.num_faces = 0
        self.tries = 0

    def execute(self, userdata):
        global name_face
        self.tries += 1

        rospy.loginfo('STATE : NEW_FACE')

        # If name is not recognized 3 times, guest may be registered as a "someone"
        if self.tries == 3:
            print(
                "voice.talk ('I am having trouble understanding your name, lets keep going')")
            voice.talk(
                'I am having trouble understanding your name, lets keep going')
            name = 'someone'
            name_face = name
            train_face(img_face, name)
            self.tries = 0
            return 'succ'

        # Asking for name
        print("voice.talk('Please, tell me your name')")
        voice.talk('Please, tell me your name')
        rospy.sleep(1.0)
        print('LEDS WHITE')
        speech = get_keywords_speech(10)
        # in case things like I am , my name is . etc
        if len(speech.split(' ')) > 1:
            name = (speech.split(' ')[-1])
        else:
            name = speech

        name_face = name
        print(name)

        if name == 'timeout':
            print(
                "voice.talk('I could not hear you , lets try again, please speak louder.')")
            voice.talk(
                'I could not hear you , lets try again, please speak louder.')
            # voice.talk ('lets try again')
            return 'failed'

        print("voice.talk(f'Is", name, "your name?')")
        voice.talk(f'Is {name} your name?')
        rospy.sleep(2.0)
        print('LEDS WHITE')
        confirmation = get_keywords_speech(10)
        print(confirmation)

        confirmation = confirmation.split(' ')
        confirm = match_speech(confirmation, ['yes', 'yeah', 'jack', 'juice'])
        if confirm:
            print("Nice to Meet You", name)
            voice.talk(f'Nice to Meet You {name}')
            train_face(img_face, name)
            self.tries = 0
            global guest_name  # TODO: delete
            guest_name = name
            return 'succ'
        else:
            print("voice.talk ('lets try again')")
            voice.talk('lets try again')
            return 'failed'


# --------------------------------------------------

class Get_drink(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed', 'tries'])
        self.new_name = ''
        self.num_faces = 0
        self.tries = 0

    def execute(self, userdata):
        global name_face, drink
        self.tries += 1

        rospy.loginfo('STATE : GET DRINK')

        if self.tries == 3:
            print("voice.talk ('I am having trouble understanding you, lets keep going')")
            voice.talk('I am having trouble understanding you, lets keep going')
            drink = 'something'
            self.tries = 0
            # add_guest(name_face, drink)                       #KNOWLEDGE UTILS RUBEN
            # analyze_face_background(img_face, name_face)      #KNOWLEDGE UTILS RUBEN takeshi git atm
            return 'succ'
        # Asking for drink
        print("voice.talk('What would you like to drink?')")
        voice.talk('What would you like to drink?')
        rospy.sleep(2.0)
        print('LEDS WHITE')
        drink = get_keywords_speech(10)

        if len(drink.split(' ')) > 1:
            drink = (drink.split(' ')[-1])
        print(drink)
        rospy.sleep(0.5)
        if drink == 'timeout':
            print(
                "voice.talk('sorry I could not hear you, lets try again, please speak louder.')")
            voice.talk(
                'sorry I could not hear you, lets try again, please speak louder.')
            return 'failed'
        print("voice.talk(f'Did you say {drink}?')")
        voice.talk(f'Did you say {drink}?')

        rospy.sleep(4.0)
        print('LEDS WHITE')
        confirmation = get_keywords_speech(10)

        '''if len(confirmation.split(' ')) > 1: confirmation=
        print(confirmation)
            
        if confirmation not in['yes','jack','juice','yeah']:
            return 'failed'  '''

        confirmation = confirmation.split(' ')
        confirm = match_speech(confirmation, ['yes', 'yeah', 'jack', 'juice'])
        if not confirm:
            return 'failed'

        # add_guest(name_face, drink)                     ##  Knowledge utils takeshi
        # analyze_face_background(img_face, name_face)    ##
        print("voice.talk('Nice')")
        voice.talk('Nice')
        self.tries = 0
        return 'succ'

# --------------------------------------------------


class Lead_to_living_room(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed', 'tries'])
        self.tries = 0

    def execute(self, userdata):

        rospy.loginfo('STATE : navigate to known location')

        print('Try', self.tries, 'of 3 attempts')
        self.tries += 1
        if self.tries == 3:
            return 'tries'
        # _,guest_name = get_waiting_guests()   #KNOWLEDGE UTILS TAKESHI
        print(
            "voice.talk(f'{guest_name}... I will lead you to the living room, please follow me')")
        voice.talk(
            f'{guest_name}... I will lead you to the living room, please follow me')
        print("voice.talk('Navigating to ,living room')")
        voice.talk('Navigating to ,living room')
        res = omni_base.move_base(known_location='living_room')
        if res:
            self.tries = 0
            return 'succ'
        else:
            # voice.talk('Navigation Failed, retrying')
            voice.talk('Navigation Failed, retrying')
            return 'failed'

# --------------------------------------------------


class Find_sitting_place(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed', 'tries'])
        self.tries = 0
        self.last_choice = ""

    def execute(self, userdata):

        rospy.loginfo('STATE : Looking for a place to sit')

        print('Try', self.tries, 'of 3 attempts')
        self.tries += 1
        # voice.talk('I am looking for a place to sit')
        voice.talk('I am looking for a place to sit')
        try:
            place, loc = find_empty_places(self.last_choice)
            self.last_choice = place
            print(place, loc)
        except:
            # voice.talk('Sorry, there is no more places to sit')
            voice.talk('Sorry, there is no more places to sit')
            return 'succ'

        tf_name = place.replace('_', '_face')
        print(tf_name)
        rospy.sleep(1.0)
        trans, _ = tf_man.getTF(target_frame=tf_name)
        print('Transformada es : ', trans)
        head.to_tf(tf_name)

        # commented detect human
        # res = detect_human_to_tf()
        # voice.talk('I will check if this place is empty')
        voice.talk('I will check if this place is empty')
        res, _ = wait_for_face()  # seconds
        if res == None:

            print("Place is: ", place)
            _, guest = get_waiting_guests()
            head.set_named_target('neutral')
            rospy.sleep(0.8)
            # head.turn_base_gaze(tf=place)
            # to be tested
            place_face = place.replace("_", "_face")
            trans, _ = tf_man.getTF(
                target_frame=place_face, ref_frame='base_link')
            distance = np.linalg.norm(np.array(trans))
            print("Distance to face is : ", distance)
            omni_base.move_d_to(distance * 0.9, place_face)

            ########################################################################

            # head.turn_base_gaze(tf=place, to_gaze='arm_flex_link')
            brazo.set_named_target('neutral')
            # voice.talk(f'{guest}, Here is a place to sit')
            voice.talk(f'{guest}, Here is a place to sit')

            assign_occupancy(who=guest, where=place)
            self.tries = 0
            return 'succ'

        else:
            occupant_name = res.Ids.ids
            # print(occupant_name)
            # occupant_name = "someone"
            if occupant_name == 'unknown':
                occupant_name = 'someone'
                # host_name, loc = find_host()
                # occupant_name = host_name
            update_occupancy(found=occupant_name, place=place)
            # voice.talk(f'I am sorry, here is {occupant_name}, I will find another place for you')
            voice.talk(
                f'I am sorry, here is {occupant_name}, I will find another place for you')
            return 'failed'

# --------------------------------------------------


class Find_host(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed', 'tries'])
        self.tries = 0
        self.tries = 1

    def execute(self, userdata):
        global name_like_host
        rospy.loginfo('STATE : Find host')
        # Using ANALYZE FACE SERVICE (DEEP FACE SERVER  on FACE_RECOG  PKG)
        # analyze = rospy.ServiceProxy('analyze_face', RecognizeFace)
        # self.tries = 1
        self.tries += 1
        print('Try', self.tries, 'of 3 attempts')

        host_name, loc = find_host()

        if loc == "None":
            # host location is not known
            num_places = len(return_places())
            # print(f'num_places: {num_places}')
            name_like_host = ""
            for i in range(1, num_places + 1):
                guest_loc = get_guest_location(name_face)
                guest_face = guest_loc.replace('_', '_face')
                tf_name = f'Place_face{i}'
                if guest_face != tf_name:
                    head.to_tf(tf_name)
                    rospy.sleep(1.0)
                    # voice.talk(f'looking for host on sit {i}')
                    voice.talk(f'looking for host on sit {i}')

                    res, _ = 
                    ()
                    if res is not None:
                        name = res.Ids.ids
                        print('Found: ', name)

                        if (self.tries == 1) and (name == host_name):
                            # self.tries = 0
                            self.tries = 1
                            name_like_host = host_name
                            assign_occupancy(who=host_name, where=f'Place_{i}')
                            return 'succ'
                        elif (self.tries >= 2) and (name != 'NO_FACE'):
                            if name != 'unknown':
                                name_like_host = name
                            # self.tries = 0
                            self.tries = 1
                            return 'succ'
                        # else:
                        #    continue

            # if the host is not found
            return 'failed'

        else:
            # Host location is known
            name_like_host = host_name
            tf_name = loc.replace('_', '_face')
            res, _ = wait_for_face()
            if res is not None:
                name = res.Ids.ids
                if name != host_name:
                    reset_occupancy(who=host_name)
                    return 'failed'

            # voice.talk(f'Host is on seat {tf_name.replace("Place_face","")}')
            # rospy.sleep(0.8)

                else:
                    head.to_tf(tf_name)
                    # self.tries == 0
                    self.tries = 1
                    return 'succ'
            else:
                reset_occupancy(who=host_name)
                return 'failed'

# --------------------------------------------------


class Introduce_guest(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed', 'tries'])
        self.tries = 0

    def execute(self, userdata):
        global name_like_host
        rospy.loginfo('STATE : Find host')

        self.tries += 1
        print('Try', self.tries, 'of 3 attempts')
        # voice.talk(f'Host like name is {name_like_host}')
        voice.talk(f'Host like name is {name_like_host}')

        takeshi_line = get_guest_description(name_face)
        drink = get_guest_drink(name_face)
        if drink == 'something':
            drink_line = ""
        else:
            drink_line = f'And likes {drink}'

        if takeshi_line != 'None':
            print("Description found")
            speech = f'{name_like_host}, {takeshi_line}, {drink_line}'
            timeout = 14.0

        else:
            print('No description found')
            speech = f'{name_like_host}, {name_face} has arrived, {drink_line}'
            timeout = 7.0

        # voice.talk(speech, timeout)
        voice.talk(speech, timeout)
        return 'succ'


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
    sis = smach_ros.IntrospectionServer(
        'SMACH_VIEW_SERVER', sm, '/SM_RECEPTIONIST')
    sis.start()

    with sm:
        # State machine for Receptionist task

        smach.StateMachine.add("INITIAL",           Initial(),              transitions={
                               'failed': 'INITIAL',           'succ': 'WAIT_DOOR_OPENED',   'tries': 'END'})
        smach.StateMachine.add("WAIT_DOOR_OPENED",  Wait_door_opened(),     transitions={
                               'failed': 'WAIT_DOOR_OPENED',  'succ': 'GOTO_DOOR',        'tries': 'END'})
        # smach.StateMachine.add("WAIT_PUSH_HAND",    Wait_push_hand(),       transitions={
        #                        'failed': 'WAIT_PUSH_HAND',    'succ': 'GOTO_DOOR',        'tries': 'WAIT_PUSH_HAND'})

        smach.StateMachine.add("SCAN_FACE",         Scan_face(),    transitions={
                               'failed': 'SCAN_FACE',     'succ': 'GET_DRINK',            'tries': 'GOTO_DOOR', 'unknown': 'NEW_FACE', })
        smach.StateMachine.add("NEW_FACE",          New_face(),     transitions={
                               'failed': 'NEW_FACE',      'succ': 'GET_DRINK',            'tries': 'NEW_FACE'})
        smach.StateMachine.add("GET_DRINK",         Get_drink(),    transitions={
                               'failed': 'GET_DRINK',     'succ': 'LEAD_TO_LIVING_ROOM',  'tries': 'GET_DRINK'})

        smach.StateMachine.add("GOTO_DOOR",             Goto_door(),            transitions={
                               'failed': 'GOTO_DOOR',             'succ': 'SCAN_FACE',            'tries': 'SCAN_FACE'})
        smach.StateMachine.add("LEAD_TO_LIVING_ROOM",   Lead_to_living_room(),  transitions={
                               'failed': 'LEAD_TO_LIVING_ROOM',   'succ': 'FIND_SITTING_PLACE',   'tries': 'END'})
        smach.StateMachine.add("FIND_SITTING_PLACE",    Find_sitting_place(),   transitions={
                               'failed': 'FIND_SITTING_PLACE',    'succ': 'FIND_HOST',            'tries': 'END'})
        smach.StateMachine.add("FIND_HOST",             Find_host(),            transitions={
                               'failed': 'FIND_HOST',             'succ': 'INTRODUCE_GUEST',       'tries': 'FIND_HOST'})
        smach.StateMachine.add("INTRODUCE_GUEST",       Introduce_guest(),      transitions={
                               'failed': 'INTRODUCE_GUEST',       'succ': 'GOTO_DOOR',        'tries': 'GOTO_DOOR'})

    outcome = sm.execute()
