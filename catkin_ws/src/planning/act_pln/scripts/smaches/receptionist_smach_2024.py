#!/usr/bin/env python3
from smach_utils_justina import *
# from smach_utils_receptionist import *

##### Define state INITIAL #####
# --------------------------------------------------
class Initial(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed'], 
                             input_keys=['l_arm_home', 'confirm_list', 'negation_list', 'speech_time'],
                             output_keys = ['l_arm_home', 'confirm_list', 'negation_list', 'speech_time'])
        self.tries = 0
        global camera_enable
        if robot_real:
            print("Cam enable")
            camera_enable = False
        else:
            print("Cam disable")
            camera_enable = True

    def execute(self, userdata):
        print('\n> STATE <: INITIAL')
        global vosk_enable
        if vosk_enable:
            print("VOSK ENABLE -->")
            userdata.confirm_list = ['yes', 'robot yes', 'jack', 'juice', 'justina yes', 'yeah', 'correct', 'affirmative']
            userdata.negation_list = ['no', 'robot no','not','now','nope','justina no', 'incorrect', 'negative']
            userdata.speech_time = 6
            print("speech timeout: ", userdata.speech_time)
        else:
            userdata.confirm_list = ["YES", "YEAH", "ROBOT YES", "JUSTINA YES", "JUICE"]
            userdata.negation_list = ['NO', 'ROBOT NO','NOPE','JUSTINA NO']

        self.tries += 1
        print(f'Try {self.tries}')
        userdata.l_arm_home = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        party.clean_knowledge(host_name="Jack", host_location="Place_2")
        places_2_tf()

        # Set neutral pose
        JuskeshinoHardware.moveLeftArmWithTrajectory(userdata.l_arm_home, 6)

        # -----INIT GRAMMAR FOR VOSK
        # -----Use with get_keywords_speech()
        # -----------SPEECH REC
        drinks = ['coke','juice','milk', 'water', 'soda', 'wine', 
                  'I want a', 'I would like a', 'tea', 'iced tea', 'cola', 'red wine', 'orange juice', 'tropical juice']
       
        names = [' my name is' , 'i am','adel', 'angel', 'axel', 
                 'charlie', 'jane', 'john', 'jules', 'morgan', 'paris', 'robin', 'simone', 'jack']                    
        gram = drinks + names + userdata.confirm_list + userdata.negation_list

        print("** confirmation list: ") 
        print(userdata.confirm_list)  
        print("** negation list: ") 
        print(userdata.negation_list)  

        head.publish_tfs()
        if self.tries == 1:
            set_grammar(gram)
            print("drinks:")
            print(drinks)
            print("-->")
            self.tries = 0
            return 'succ'
        
        elif self.tries == 3:
            rospy.logerr('Failed! some error ocurred')
            return 'failed'

# --------------------------------------------------
class Wait_push_hand(smach.State):
    pass

# --------------------------------------------------
class Wait_door_opened(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed'])
        self.tries = 0

    def execute(self, userdata):
        self.tries += 1
        if self.tries == 1:
            print("\n")
            rospy.logwarn('--> STATE <: Wait for door to be opened')
            print('I am ready for receptionist task.')
            voice.talk('I am ready for receptionist task.')

        print(f'Try {self.tries}')
        print('I am waiting for the door to be opened')
        voice.talk('I am waiting for the door to be opened')
        rospy.sleep(2.5)

        msg = rospy.wait_for_message('/hardware/scan', LaserScan)
        if np.mean(msg.ranges[(int)(len(msg.ranges)*0.5 - 10):(int)(len(msg.ranges)*0.5 + 10)]) < 1.0:
            return 'failed'  # DOOR
        
        self.tries = 0
        return 'succ'


class Goto_door(smach.State):  # ADD KNONW LOCATION DOOR
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed', 'tries'])
        self.tries = 0
        self.time_out = 30
        self.attempts = 2

    def execute(self, userdata):
        self.tries += 1
        if self.tries == 1:
            self.time_out = 30
            print("\n")
            rospy.logwarn('--> STATE <: Navigate to Door')
            print("Navigating door")
            voice.talk('Navigating door')

        if self.tries == self.attempts + 1: 
            rospy.logerr('Navigation Failed, I can not reach the door')
            voice.talk('Navigation Failed, I can not reach the door')
            return 'failed'
        
        print(f'Try {self.tries} of {self.attempts} attempts')
        res = omni_base.move_base(known_location='door', time_out=self.time_out)
        print("res:", res)

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
            return 'failed'

# --------------------------------------------------
class Scan_face(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes = ['succ', 'failed'], 
                             output_keys = ['name', 'face_img'])
        self.tries = 0

    def execute(self, userdata):
        global camera_enable
        self.tries += 1
        if self.tries == 1:
            print("\n")
            rospy.logwarn('--> STATE <: Scan face')

        head.set_joint_values([0.0, -0.1])  # Ensure head is up before scan 
        rospy.sleep(0.3)
        head.set_joint_values([0.0, -0.1])
        print('Scanning for faces')
        voice.talk('Scanning for faces')
        
        # For simulation use camera_enable = True
        res, userdata.face_img = wait_for_face(lap_camera=camera_enable)  # default 10 secs
        if res != None:
            userdata.name = res.Ids.ids
            return 'succ'
        else:
            return 'failed'


# Decide face STATE: Decide if the new guest is known, unknown or can't be decided
class Decide_face(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['succ', 'failed', 'unknown'], 
                             input_keys=['name', 'face_img', 'confirm_list', 'speech_time'],
                             output_keys=['name', 'face_img'])
        self.tries = 0
    def execute(self, userdata):
        print("\n")
        rospy.logwarn('--> STATE <: Decide face')

        if userdata.name == 'NO_FACE':
            print('I did not see you, trying again')
            voice.talk('I did not see you, trying try again')
            return 'failed'

        elif userdata.name == 'unknown':
            print('I find you.')
            voice.talk('I find you.')
            return 'unknown'

        else:
            print(f'I found you, I Think you are, {userdata.name}.')
            print('Is it correct?')
            voice.talk(f'I found you, I Think you are, {userdata.name}.')
            voice.talk('Is it correct?')

            if vosk_enable:
                rospy.logwarn('Listening now (Cc )')
                confirmation = get_keywords_speech(userdata.speech_time)

            else: 
                JuskeshinoHRI.getLastRecognizedSentence()
                rospy.sleep(0.3)
                confirmation = JuskeshinoHRI.waitForNewSentence(userdata.speech_time) # 10 is to much?
            
            print(confirmation)
            if confirmation not in userdata.confirm_list:
                return 'unknown'
            elif confirmation == "timeout":
                print('I could not hear you, lets try again, please speak louder.')
                voice.talk('I could not hear you, lets try again, please speak louder.')
                return "failed"
            else:
                party.add_guest(userdata.name)
                return 'succ'


# --------------------------------------------------
class New_face(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['succ', 'failed'],
                             input_keys=['name', 'face_img', 'confirm_list', 'speech_time'],
                             output_keys=['name', 'face_img'])
        self.tries = 0

    def execute(self, userdata):
        self.tries += 1
        if self.tries == 1:
            print("\n")
            rospy.logwarn('--> STATE <: new face')

        #If name is not recognized 3 times, guest will be registered as a "someone"
        if self.tries == 3:
            print("I didn't undestand your name, lets continue")
            voice.talk ('I didnt undestand your name, lets continue')
            userdata.name = 'someone'
            train_face(userdata.face_img, userdata.name)
            party.add_guest(userdata.name)
            self.tries = 0
            return 'succ'
        
        #Asking for name
        print('Please, tell me your name')
        voice.talk('Please, tell me your name')

        if vosk_enable:
            rospy.logwarn('Listening now (Cc )')
            speech = get_keywords_speech(userdata.speech_time)
        else: 
            JuskeshinoHRI.getLastRecognizedSentence()
            rospy.sleep(0.3)
            speech = JuskeshinoHRI.waitForNewSentence(userdata.speech_time)
            speech.lower()

        # in case thinks like I am , my name is . etc
        if len(speech.split(' ')) > 1: 
            name = (speech.split(' ')[-1])
        else: 
            name = speech

        if userdata.name == 'timeout':
            print('Please repeat it and speak louder.')
            voice.talk('Please repeat it and speak louder.')
            return 'failed'

        print(f'Is {name} your name?')
        voice.talk(f'Is {name} your name?')

        if vosk_enable:
            rospy.logwarn('Listening now (Cc )')
            confirmation = get_keywords_speech(userdata.speech_time)
        else: 
            JuskeshinoHRI.getLastRecognizedSentence()
            rospy.sleep(0.3)
            confirmation = JuskeshinoHRI.waitForNewSentence(userdata.speech_time)

        print(confirmation)
        confirmation = confirmation.split(' ')
        confirm = match_speech(confirmation, userdata.confirm_list)
        if confirm:
            userdata.name = name
            print(f'Nice to Meet You {userdata.name}')
            voice.talk (f'Nice to Meet You {userdata.name}')
            party.add_guest(userdata.name)
            train_face(userdata.face_img, userdata.name)
            self.tries = 0
            return 'succ'
        else:
            print('lets try again')
            voice.talk('lets try again')
            return 'failed'


# --------------------------------------------------
class Get_drink(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['succ', 'failed', 'tries'],
                             input_keys=['name', 'face_img', 'confirm_list', 'speech_time'])
        self.tries = 0
        self.attempts = 3

    def execute(self, userdata):
        self.tries += 1
        
        if self.tries == 1:
            print("\n")
            rospy.logwarn('--> STATE <: get drink')
            analyze_face_background(userdata.face_img, party.get_active_guest_name())

        print(f'Try {self.tries} of {self.attempts} attempts')
        if self.tries == 3:
            voice.talk ('I am having trouble understanding you, lets keep going')
            drink = 'something'
            self.tries = 0
            party.add_guest_drink(drink)
            analyze_face_background(userdata.face_img, userdata.name)
            return 'failed'
        
        #Asking for drink
        print('What would you like to drink?')
        voice.talk('What would you like to drink?')
        rospy.sleep(0.3)

        if vosk_enable:
            rospy.logwarn('Listening now (Cc )')
            drink = get_keywords_speech(userdata.speech_time)
        else: 
            JuskeshinoHRI.getLastRecognizedSentence()
            drink = JuskeshinoHRI.waitForNewSentence(userdata.speech_time)
            drink.lower()

        print(drink)
        rospy.sleep(0.5)
        if drink=='timeout':
            print("Sorry, couldn't hear you. Please speak louder.")
            voice.talk("Sorry, couldn't hear you. Please speak louder.")
            return 'tries'
        print(f'Did you say {drink}?')
        voice.talk(f'Did you say {drink}?')
        
        rospy.sleep(0.3)
        # TODO: TEST VOICE BEFORE START
        if vosk_enable:
            rospy.logwarn('Listening now (Cc )')
            confirmation = get_keywords_speech(userdata.speech_time)
        else: 
            JuskeshinoHRI.getLastRecognizedSentence()
            confirmation = JuskeshinoHRI.waitForNewSentence(userdata.speech_time)

        print(confirmation)
        confirm = match_speech(confirmation, userdata.confirm_list)
        if not confirm: 
            return 'tries' 

        party.add_guest_drink(drink)
        print("Nice")
        voice.talk("Nice")
        self.tries = 0
        return 'succ'


# --------------------------------------------------
class Lead_to_living_room(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed', 'tries'])
        self.tries = 0
        self.known_location = 'living_room'
        self.known_location_talk = 'living room'

    def execute(self, userdata):
        self.tries += 1
        print('Try', self.tries)
        if self.tries == 1:
            print("\n")
            rospy.logwarn('--> STATE <: navigate to ' + self.known_location_talk)

        print(f'{party.get_active_guest_name()}... I will lead you to the living room, please follow me')
        voice.talk(f'{party.get_active_guest_name()}... I will lead you to the living room, please follow me')
        print('Navigating to ' + self.known_location_talk)
        voice.talk('Navigating to, '+ self.known_location_talk)

        res = omni_base.move_base(known_location=self.known_location)
        if res == 3:
            self.tries = 0
            return 'succ'
        elif res == 1 or res == 8:
            rospy.logerr('Navigation Failed, retrying')
            voice.talk('Navigation Failed, retrying')
            return 'tries'
        else:
            rospy.logerr('Navigation Failed, I can not reach the living room')
            voice.talk('Navigation Failed, I can not reach the living room')
            self.tries = 0
            return 'failed'


# --------------------------------------------------
class Find_sitting_place(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed', 'tries'],
                             input_keys=['l_arm_home'])
        self.tries = 0
        self.failed = False
        self.n_sits = 3  # TODO: SET THIS BEFORE START
        self.l_arm_offerSit = [0.3, 0.2, -0.4, 1.7, 0.0, -0.4, 1.5, 0.2]

    def execute(self, userdata):
        global camera_enable

        self.tries += 1
        if self.tries == 1:
            print("\n")
            rospy.logwarn('--> STATE <: looking for a place to sit')

        print('Try', self.tries, 'of', self.n_sits, 'attempts')
        if self.tries == self.n_sits:
            print('Sorry, there is no more places to sit')
            voice.talk('Sorry, there is no more places to sit')
            self.failed = False
            return 'failed'

        print("I am looking for a place to sit")
        voice.talk('I am looking for a place to sit')
        isPlace, place = party.get_active_seat()

        if self.failed and self.tries < self.n_sits:
            place = party.get_any_available_seat()
            if place != None:
                isPlace = True

        print("isPlace:", isPlace)
        if isPlace:
            tf_name = place.replace('_', '_face')
            print("tf_name", tf_name)
            head.to_tf(tf_name)
        else:  
            rospy.logerr('Sorry there are not more empty sits')
            voice.talk('Sorry, there are not more empty sits')
            return 'failed'


        print('I will check if this place is empty')
        voice.talk('I will check if this place is empty')
        # For simulation use camera_enable = True
        res , _ = wait_for_face(lap_camera=camera_enable)  # seconds
        if res == None:
            print("Place is: ", place)
            guest = party.get_active_guest_name()
            head.turn_base_gaze(tf=place, to_gaze='base_link') #'arm_flex_link'
            head.set_named_target('neutral')
            JuskeshinoHardware.moveLeftArmWithTrajectory(self.l_arm_offerSit, 6)
            rospy.sleep(0.8)
            print(f'{guest}, Here is a place to sit')
            voice.talk(f'{guest}, Here is a place to sit')
            JuskeshinoHardware.moveLeftArmWithTrajectory(userdata.l_arm_home, 6)
            party.seat_confirmation()
            self.tries = 0
            self.failed = False
            return 'succ'

        else:
            occupant_name = res.Ids.ids
            if occupant_name == 'unknown':
                occupant_name = 'someone'
            party.seat_confirmation(occupant_name)
            print(f'I am sorry, here is {occupant_name}, I will find another place for you')
            voice.talk(f'I am sorry, here is {occupant_name}, I will find another place for you')
            self.failed = True
            return 'tries'
        
# --------------------------------------------------
class Find_host_alternative(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes= ['succ', 'failed', 'tries'],
                             output_keys=['name_like_host'])
        self.tries = 0
    def execute(self, userdata):
        global camera_enable
        self.tries += 1

        host_name = ""
        host_loc = ""
        dont_compare = False

        # First try: looks for the real host
        # Next tries: looks for anyone
        if self.tries == 1:
            print("\n")
            rospy.logwarn('--> STATE <: find host alternative')
            host_name, host_loc = party.get_host_info()
            if host_loc == 'None':
                return 'failed'
        else:
            seats = party.get_guests_seat_assignments()
            print("seats: ", seats)
            for place, guest in seats.items():
                if guest != party.get_active_guest():
                    host_loc = place
                    dont_compare = True
                    break

        if self.tries == 3:
            print(f'I am sorry, I can not find the host, lets keep going')
            voice.talk(f'I am sorry, I can not find the host, lets keep going')
            userdata.name_like_host, _ = party.get_host_info()
            return 'failed'
        
        print(f'looking for host on: {host_loc}')
        host_place_say = host_loc.replace('_', ' ')
        voice.talk(f'looking for host on: {host_place_say}')
        tf_host = host_loc.replace('_', '_face')
        head.to_tf(tf_host)

        # For simulation use camera_enable = True
        res, _ = wait_for_face(lap_camera=camera_enable)
        if res is not None:
            person_name = res.Ids.ids
            if (person_name == host_name) or dont_compare:
                userdata.name_like_host = person_name
                return 'succ'
            else:
                return 'tries'
        else:
            return 'tries'


# --------------------------------------------------
class Introduce_guest(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed', 'tries'],
                             input_keys=['name_like_host'])
        self.tries = 0
        self.n_guest = 2  # TODO: SET THIS BEFORE START

    def execute(self, userdata):
        self.tries += 1
        if self.tries == 1:
            print("\n")
            rospy.logwarn('--> STATE <: introduce guest')

        print('Try', self.tries, 'of 3 attempts')
        print(f'Host like name is {userdata.name_like_host}')
        voice.talk(f'Host like name is {userdata.name_like_host}')

        active_guest = party.get_active_guest_name()
        justina_line = party.get_active_guest_description()                    
        drink = party.get_active_guest_drink()

        if drink == 'something':
            drink_line = ""
        else:
            drink_line = f'And likes {drink}'

        if justina_line != 'None':
            print("Description found")
            speech = f'{userdata.name_like_host}, {justina_line}, {drink_line}'
            timeout = 14.0
        else:
            print('No description found')
            speech = f'{userdata.name_like_host}, {active_guest} has arrived, {drink_line}'
            timeout = 7.0

        print(speech)
        voice.talk(speech, timeout)
        
        if self.tries < self.n_guest:
            return 'succ'
        else:
            print('Task completed, thanks for watching')
            voice.talk("Task completed, thanks for watching")
            return 'tries'


# --------------------------------------------------
# Entry point
if __name__ == '__main__':
    print("Justina STATE MACHINE...")
    # State machine, final state "END"
    sm = smach.StateMachine(outcomes=['END'])
    # sm.userdata.clear = False
    sis = smach_ros.IntrospectionServer(
        'SMACH_VIEW_SERVER', sm, '/SM_RECEPTIONIST')
    sis.start()

    with sm:
        # State machine for Receptionist task
        # Initial states routine
        smach.StateMachine.add("INITIAL", Initial(),              
                               transitions={'failed':'INITIAL', 'succ':'WAIT_DOOR_OPENED'})
        
        # smach.StateMachine.add("WAIT_PUSH_HAND", Wait_push_hand(),       
        #                        transitions={'failed': 'WAIT_PUSH_HAND', 'succ': 'GOTO_DOOR'})

        smach.StateMachine.add("WAIT_DOOR_OPENED", Wait_door_opened(),     
                               transitions={'failed':'WAIT_DOOR_OPENED', 'succ':'GOTO_DOOR'})
        
        smach.StateMachine.add("GOTO_DOOR", Goto_door(),            
                               transitions={'failed':'SCAN_FACE', 'tries':'GOTO_DOOR', 'succ':'SCAN_FACE'})
        
        # Guest recognition states
        smach.StateMachine.add("SCAN_FACE", Scan_face(),    
                               transitions={'failed':'SCAN_FACE', 'succ':'DECIDE_FACE'})
        
        smach.StateMachine.add("DECIDE_FACE", Decide_face(),
                               transitions={'failed':'SCAN_FACE', 'succ':'GET_DRINK', 'unknown':'NEW_FACE'})
        
        smach.StateMachine.add("NEW_FACE", New_face(),     
                               transitions={'failed':'NEW_FACE', 'succ':'GET_DRINK'})
        
        smach.StateMachine.add("GET_DRINK", Get_drink(),    
                               transitions={'tries':'GET_DRINK', 'failed':'LEAD_TO_LIVING_ROOM', 'succ':'LEAD_TO_LIVING_ROOM'})

        # Final states
        smach.StateMachine.add("LEAD_TO_LIVING_ROOM", Lead_to_living_room(),  
                               transitions={'tries':'LEAD_TO_LIVING_ROOM', 'failed':'FIND_SITTING_PLACE', 'succ':'FIND_SITTING_PLACE'})
        
        smach.StateMachine.add("FIND_SITTING_PLACE", Find_sitting_place(),
                               transitions={'tries':'FIND_SITTING_PLACE', 'failed':'FIND_HOST', 'succ':'FIND_HOST'})
        
        smach.StateMachine.add("FIND_HOST", Find_host_alternative(),
                               transitions={'tries':'FIND_HOST', 'failed':'INTRODUCE_GUEST', 'succ':'INTRODUCE_GUEST'})
        
        smach.StateMachine.add("INTRODUCE_GUEST", Introduce_guest(),
                               transitions={'failed':'INTRODUCE_GUEST', 'succ':'GOTO_DOOR', 'tries':'END'})

    outcome = sm.execute()