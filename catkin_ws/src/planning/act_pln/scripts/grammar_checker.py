class Command:
    def __init__(self):
        self.sentence = ''
        self.actions = []

_takeVerb = ['TAKE', 'GET', 'GRASP', 'FETCH']
_placeVerb = ['PUT', 'PLACE']
_deliverVerb = ['BRING', 'GIVE', 'DELIVER']
_bringVerb = ['BRING', 'GIVE']
_goVerb = ['GO', 'NAVIGATE']
_findVerb = ['FIND', 'LOCATE', 'LOOK FOR']
_talkVerb = ['TELL', 'SAY']
_answerVerb = ['ANSWER']
_meetVerb = ['MEET']
_tellVerb = ['TELL']
_greetVerb = ['GREET', 'SALUTE', 'SAY HELLO TO', 'INTRODUCE YOURSELF TO']
_countVerb = ['TELL ME HOW MANY']
_followVerb = ['FOLLOW']
_guideVerb = ['GUIDE', 'ESCORT', 'TAKE', 'LEAD']

_toLocPrep = ['TO']
_art = ['A', 'AN']
_onLocPrep = ['ON']
_inLocPrep = ['IN']
_deliverPrep = ['TO']
_talkPrep = ['TO']
_ofPrsPrep = ['OF']
_fromLocPrep = ['FROM']
_atLocPrep = ['AT']

_gesturePersonList = ['WAVING PERSON', 'PERSON RAISING THEIR LEFT ARM', 'PERSON RAISING THEIR RIGHT ARM', 'PERSON POINTING TO THE LEFT', 'PERSON POINTING TO THE RIGHT']
_posePersonList = ['SITTING PERSON', 'STANDING PERSON', 'LYING PERSON']
_gesturePersonPluralList = ['WAVING PERSONS', 'PERSONS RAISING THEIR LEFT ARM', 'PERSONS RAISING THEIR RIGHT ARM', 'PERSONS POINTING TO THE LEFT', 'PERSONS POINTING TO THE RIGHT']
_posePersonPluralList = ['SITTING PERSONS', 'STANDING PERSONS', 'LYING PERSONS']
_personInfoList = ['NAME', 'POSE', 'GESTURE']
_objCompList = ['BIGGEST', 'LARGEST', 'SMALLEST', 'HEAVIEST', 'LIGHTEST', 'THINNEST']
_talkList = ['SOMETHING ABOUT YOURSELF', 'THE TIME', 'WHAT DAY IS TODAY', 'WHAT DAY IS TOMORROW', 'YOUR TEAMS NAME', 'YOUR TEAMS COUNTRY', 'YOUR TEAMS AFFILIATION', 'THE DAY OF THE WEEK', 'THE DAY OF THE MONTH']
_questionList = ['QUESTION', 'QUIZ']
_colorClotheList = ['BLUE T SHIRT', 'BLUE SHIRT', 'BLUE BLOUSE', 'BLUE SWEATER', 'BLUE COAT', 'BLUE JACKET', 'YELLOW T SHIRT', 'YELLOW SHIRT', 'YELLOW BLOUSE', 'YELLOW SWEATER', 'YELLOW COAT', 'YELLOW JACKET', 'BLACK T SHIRT', 'BLACK SHIRT', 'BLACK BLOUSE', 'BLACK SWEATER', 'BLACK COAT', 'BLACK JACKET', 'WHITE T SHIRT', 'WHITE SHIRT', 'WHITE BLOUSE', 'WHITE SWEATER', 'WHITE COAT', 'WHITE JACKET', 'RED T SHIRT', 'RED SHIRT', 'RED BLOUSE', 'RED SWEATER', 'RED COAT', 'RED JACKET', 'ORANGE T SHIRT', 'ORANGE SHIRT', 'ORANGE BLOUSE', 'ORANGE SWEATER', 'ORANGE COAT', 'ORANGE JACKET', 'GRAY T SHIRT', 'GRAY SHIRT', 'GRAY BLOUSE', 'GRAY SWEATER', 'GRAY COAT', 'GRAY JACKET']
_colorClothesList = ['BLUE T SHIRTS', 'BLUE SHIRTS', 'BLUE BLOUSES', 'BLUE SWEATERS', 'BLUE COATS', 'BLUE JACKETS', 'YELLOW T SHIRTS', 'YELLOW SHIRTS', 'YELLOW BLOUSES', 'YELLOW SWEATERS', 'YELLOW COATS', 'YELLOW JACKETS', 'BLACK T SHIRTS', 'BLACK SHIRTS', 'BLACK BLOUSES', 'BLACK SWEATERS', 'BLACK COATS', 'BLACK JACKETS', 'WHITE T SHIRTS', 'WHITE SHIRTS', 'WHITE BLOUSES', 'WHITE SWEATERS', 'WHITE COATS', 'WHITE JACKETS', 'RED T SHIRTS', 'RED SHIRTS', 'RED BLOUSES', 'RED SWEATERS', 'RED COATS', 'RED JACKETS', 'ORANGE T SHIRTS', 'ORANGE SHIRTS', 'ORANGE BLOUSES', 'ORANGE SWEATERS', 'ORANGE COATS', 'ORANGE JACKETS', 'GRAY T SHIRTS', 'GRAY SHIRTS', 'GRAY BLOUSES', 'GRAY SWEATERS', 'GRAY COATS', 'GRAY JACKETS']

_locationNames = ['BED', 'BEDSIDE TABLE', 'SHELF', 'TRASHBIN', 'DISHWASHER', 'POTTED PLANT', 'KITCHEN TABLE', 'CHAIRS', 'PANTRY', 'REFRIGERATOR', 'SINK', 'CABINET', 'COATRACK', 'DESK', 'ARMCHAIR', 'DESK LAMP', 'WASTE BASKET', 'TV STAND', 'STORAGE RACK', 'LAMP', 'SIDE TABLES', 'SOFA', 'BOOKSHELF', 'ENTRANCE', 'EXIT']
_roomNames = ['BEDROOM', 'KITCHEN', 'OFFICE', 'LIVING ROOM', 'BATHROOM']
_objNames = ['PRINGLES', 'CHEEZIT', 'CORNFLAKES', 'SPONGE', 'CLEANSER', 'SUGAR', 'TOMATO SOUP', 'CHOCOLATE JELLO', 'STRAWBERRY JELLO', 'MUSTARD', 'TUNA', 'SPAM', 'COFFEE GROUNDS', 'ORANGE', 'BANANA', 'STRAWBERRY', 'PEACH', 'PEAR', 'APPLE', 'PLUM', 'LEMON', 'FORK', 'SPOON', 'BOWL', 'KNIFE', 'PLATE', 'CUP', 'MILK', 'TROPICAL JUICE', 'ICED TEA', 'ORANGE JUICE', 'JUICE PACK', 'RED WINE', 'COLA', 'BASEBALL', 'TENNIS BALL', 'DICE', 'RUBIKS CUBE', 'SOCCER BALL']
_singCategories = ['SNACK', 'CLEANING SUPPLY', 'FOOD', 'FRUIT', 'DISH', 'DRINK', 'TOY']
_personNames = ['ADEL', 'ANGEL', 'AXEL', 'CHARLIE', 'JANE', 'JULES', 'MORGAN', 'PARIS', 'ROBIN', 'SIMONE']
_placementLocNames = ['BED', 'BEDSIDE TABLE', 'SHELF', 'DISHWASHER', 'KITCHEN TABLE', 'PANTRY', 'REFRIGERATOR', 'SINK', 'CABINET', 'DESK', 'TV STAND', 'STORAGE RACK', 'SIDE TABLES', 'SOFA', 'BOOKSHELF']
_pluralCategories = ['SNACKS', 'CLEANING SUPPLIES', 'FOOD', 'FRUITS', 'DISHES', 'DRINKS', 'TOYS']

talkListAnswers = {'SOMETHING ABOUT YOURSELF': 'MY NAME IS JUSTINA AND I AM FROM MEXICO', 'THE TIME': 'IT IS AROUND THREE O CLOCK', 'WHAT DAY IS TODAY':'TODAY IS THURSDAY', 'WHAT DAY IS TOMORROW':'TOMORROW IS FRIDAY', 'YOUR TEAMS NAME':'MY TEAM NAME IS PUMAS', 'YOUR TEAMS COUNTRY':'MY TEAM COUNTRY IS MEXICO', 'YOUR TEAMS AFFILIATION':'MY TEAM IS WITH THE NATIONAL AUTONOMOUS UNIVERSITY OF MEXICO', 'THE DAY OF THE WEEK':'TODAY IS WEDNESDAY', 'THE DAY OF THE MONTH':'TODAY IS JULY SEVENTEENTH'}

def has_pattern(cmd, word_list):
    # print(cmd.sentence)
    # print(word_list)
    for w in word_list:
        if w in cmd.sentence.split():
            #cmd.sentence = cmd.sentence.replace(w,'', 1)
            return True
    for w in word_list:
        if w in cmd.sentence:
            #cmd.sentence = cmd.sentence.replace(w,'', 1)
            return True
    return False

def get_pattern(cmd, word_list, start_idx=0, end_idx=-1):
    sp = cmd.sentence.split()
    triples = []
    for i in range(2, len(sp)):
        triples.append(sp[i-2] + ' ' + sp[i-1] + ' ' + sp[i])
    for w in word_list:
        if w in triples:
            return w

    sp = cmd.sentence.split()
    pairs = []
    for i in range(1, len(sp)):
        pairs.append(sp[i-1] + ' ' + sp[i])
    for w in word_list:
        if w in pairs:
            return w
        
    for w in word_list:
        if w in cmd.sentence.split():
            return w
    
    for w in word_list:
        if w in cmd.sentence:
            return w
    return ''

def get_pattern_from_string(cmd, word_list, start_idx=0, end_idx=-1):
    sp = cmd.split()
    triples = []
    for i in range(2, len(sp)):
        triples.append(sp[i-2] + ' ' + sp[i-1] + ' ' + sp[i])
    for w in word_list:
        if w in triples:
            return w

    sp = cmd.split()
    pairs = []
    for i in range(1, len(sp)):
        pairs.append(sp[i-1] + ' ' + sp[i])
    for w in word_list:
        if w in pairs:
            return w
        
    for w in word_list:
        if w in cmd.split():
            return w
    
    for w in word_list:
        if w in cmd:
            return w
    return ''

def takeVerb(command):
    return has_pattern(command, _takeVerb)
    
def placeVerb(command):
    return has_pattern(command, _placeVerb)

def deliverVerb(command):
    return has_pattern(command, _deliverVerb)

def bringVerb(command):
    return has_pattern(command, _bringVerb)

def goVerb(command):
    return has_pattern(command, _goVerb)

def findVerb(command):
    return has_pattern(command, _findVerb)

def talkVerb(command):
    return has_pattern(command, _talkVerb)

def answerVerb(command):
    return has_pattern(command, _answerVerb)

def meetVerb(command):
    return has_pattern(command, _meetVerb)

def tellVerb(command):
    return has_pattern(command, _tellVerb)

def greetVerb(command):
    return has_pattern(command, _greetVerb)

def countVerb(command):
    return has_pattern(command, _countVerb)

def followVerb(command):
    return has_pattern(command, _followVerb)

def guideVerb(command):
    return has_pattern(command, _guideVerb)


def toLocPrep(command):
    return has_pattern(command, _toLocPrep)

def art(command):
    return has_pattern(command, _art)

def onLocPrep(command):
    return has_pattern(command, _onLocPrep)

def inLocPrep(command):
    return has_pattern(command, _inLocPrep)

def deliverPrep(command):
    return has_pattern(command, _deliverVerb)

def talkPrep(command):
    return has_pattern(command, _talkVerb)

def ofPrsPrep(command):
    return has_pattern(command, _ofPrsPrep)

def fromLocPrep(command):
    return has_pattern(command, _fromLocPrep)

def atLocPrep(command):
    return has_pattern(command, _atLocPrep)

                         
def gesturePersonList(command):
    return has_pattern(command, _gesturePersonList)

def posePersonList(command):
    return has_pattern(command, _posePersonList)

def gesturePersonPluralList(command):
    return has_pattern(command, _gesturePersonPluralList)

def posePersonPluralList(command):
    return has_pattern(command, _posePersonPluralList)

def personInfoList(command):
    return has_pattern(command, _personInfoList)

def objCompList(command):
    return has_pattern(command, _objCompList)

def talkList(command):
    return has_pattern(command, _talkList)

def questionList (command):
    return has_pattern(command, _questionList)

def colorClotheList(command):
    return has_pattern(command, _colorClotheList)

def colorClothesList(command):
    return has_pattern(command, _colorClothesList)

                         
def locationNames(command):
    return has_pattern(command, _locationNames)

def roomNames(command):
    return has_pattern(command, _roomNames)

def objNames(command):
    return has_pattern(command, _objNames)

def singCategories(command):
    return has_pattern(command, _singCategories)

def personNames(command):
    return has_pattern(command, _personNames)

def placementLocNames(command):
    return has_pattern(command, _placementLocNames)

def pluralCategories(command):
    return has_pattern(command, _pluralCategories)



def atLoc(cmd):
    return 'AT THE' in cmd.sentence and locationNames(cmd);

def inRoom(cmd):
    return 'IN THE' in cmd.sentence and roomNames(cmd);

def inRoom_atLoc(cmd):
    return inRoom(cmd) or atLoc(cmd)

def gestPersPlur_posePersPlur(cmd):
    return gesturePersonPluralList(cmd) or posePersonPluralList(cmd)

def loc_room(cmd):
    return locationNames(cmd) or roomNames(cmd)

def obj_singCat(cmd):
    return objNames(cmd) or singCategories(cmd)

def gestPers_posePers(cmd):
    return gesturePersonList(cmd) or posePersonList(cmd)

#
# The following functions can represent commands.
# Possible robot actions:
# navigate, follow, say, find_person, find_gesture, leave_object, take, find_objects
def guidePrsToBeacon(cmd):
    prev_actions = cmd.actions.copy()
    success = guideVerb(cmd) and 'THEM' in cmd.sentence and toLocPrep(cmd) and 'THE' in cmd.sentence and loc_room(cmd)
    if success:
        goal_location = get_pattern(cmd, (_locationNames + _roomNames))
        actions = [['navigate', goal_location]]
        cmd.actions += actions
    else:
        cmd.actions = prev_actions
    return success

def followPrsToRoom(cmd):
    prev_actions = cmd.actions.copy()
    success = followVerb(cmd) and 'THEM' in cmd.sentence and toLocPrep(cmd) and 'THE' in cmd.sentence and loc_room(cmd)
    if success:
        goal_location = get_pattern(cmd, (_locationNames + _roomNames))
        actions = [['follow', goal_location]]
        cmd.actions += actions
    else:
        cmd.actions = prev_actions
    return success

def followPrs(cmd):
    prev_actions = cmd.actions.copy()
    success = followVerb(cmd) and 'THEM' in cmd.sentence
    if success:
        actions = [['follow', '']]
        cmd.actions += actions
    else:
        cmd.actions = prev_actions
    return success

def answerQuestion(cmd):
    prev_actions = cmd.actions.copy()
    success = answerVerb(cmd)  and 'A' in cmd.sentence and questionList(cmd)
    if success:
        actions = [['say', 'I will answer a question']]
        cmd.actions += actions
    else:
        cmd.actions = prev_actions
    return success

def talkInfo(cmd):
    prev_actions = cmd.actions.copy()
    success = talkVerb(cmd) and talkList(cmd)
    if success:
        None
    else:
        cmd.actions = prev_actions
    return success

def deliverObjToNameAtBeac(cmd):
    prev_actions = cmd.actions.copy()
    success = deliverVerb(cmd) and 'IT' in cmd.sentence and deliverPrep(cmd) and personNames(cmd) and inLocPrep(cmd) and 'THE' in cmd.sentence and roomNames(cmd)
    if success:
        goal_room = get_pattern(cmd, _roomNames)
        person_name = get_pattern(cmd, _personNames)
        actions = [['navigate', goal_room], ['find_person', person_name], ['leave_object','']]
        cmd.actions += actions
    else:
        cmd.actions = prev_actions
    return success

def deliverObjToPrsInRoom(cmd):
    prev_actions = cmd.actions.copy()
    success = deliverVerb(cmd) and 'IT' in cmd.sentence and deliverPrep(cmd) and 'THE' in cmd.sentence and gestPers_posePers(cmd) \
        and inLocPrep(cmd) and 'THE' in cmd.sentence and roomNames(cmd)
    if success:
        goal_room = get_pattern(cmd, _roomNames)
        gesture = get_pattern(cmd, (_gesturePersonList + _posePersonList))
        actions = [['navigate', goal_room], ['find_gesture', gesture], ['leave_object','']]
        cmd.actions += actions
    else:
        cmd.actions = prev_actions
    return success

def deliverObjToMe(cmd):
    prev_actions = cmd.actions.copy()
    success = deliverVerb(cmd) and 'IT TO ME' in cmd.sentence
    if success:
        actions = [['navigate','start'], ['leave_object', '']]
        cmd.actions += actions
    else:
        cmd.actions = prev_actions
    return success

def placeObjOnPlcmt(cmd):
    prev_actions = cmd.actions.copy()
    success = placeVerb(cmd) and 'IT' in cmd.sentence and onLocPrep(cmd) and 'THE' in cmd.sentence and placementLocNames(cmd)
    if success:
        goal_location = get_pattern(cmd, _placementLocNames)
        actions = [['navigate', goal_location], ['leave_object', '']]
        cmd.actions += actions
    else:
        cmd.actions = prev_actions
    return success

def hasObj(cmd):
    return placeObjOnPlcmt(cmd) or deliverObjToMe(cmd) or deliverObjToPrsInRoom(cmd) or deliverObjToNameAtBeac(cmd)

def foundPers(cmd):
    return talkInfo(cmd) or answerQuestion(cmd) or followPrsToRoom(cmd) or followPrs(cmd) or guidePrsToBeacon(cmd)

def findObj(cmd):
    prev_actions = cmd.actions.copy()
    success = findVerb(cmd) and art(cmd) and obj_singCat(cmd) and 'AND' in cmd.sentence and  takeVerb(cmd) and 'IT AND' in cmd.sentence and  hasObj(cmd)
    if success:
        goal_object = get_pattern(cmd, (_objNames + _singCategories))
        actions = [['take', goal_object]]
        cmd.actions = actions + cmd.actions
    else:
        cmd.actions = prev_actions
    return success

def meetName(cmd):
    prev_actions = cmd.actions.copy()
    success = meetVerb(cmd) and personNames(cmd) and 'AND' in cmd.sentence and foundPers(cmd)
    if success:
        person_name = get_pattern(cmd, _personNames)
        actions = [['find_person', person_name]]
        cmd.actions = actions + cmd.actions
    else:
        cmd.actions = prev_actions
    return success

def findPrs(cmd):
    prev_actions = cmd.actions.copy()
    success = findVerb(cmd) and 'THE' in cmd.sentence and gestPers_posePers(cmd) and 'AND' in cmd.sentence and foundPers(cmd)
    if success:
        gesture = get_pattern(cmd, (_gesturePersonList + _posePersonList))
        actions = [['find_gesture', gesture]]
        cmd.actions = actions + cmd.actions
    else:
        cmd.actions = prev_actions
    return success


def followUpFoundObj(cmd):
    prev_actions = cmd.actions.copy()
    success = takeVerb(cmd) and 'IT AND' in cmd.sentence and hasObj(cmd)
    if success:
        actions = [['take', 'it']]
        cmd.actions = actions + cmd.actions
    else:
        cmd.actions = prev_actions
    return success

def followUpFoundPers(cmd):
    return foundPers(cmd)

def followUpAtLoc(cmd):
    return findPrs(cmd) or meetName(cmd) or findObj(cmd)

def tellCatPropOnPlcmt(cmd):
    prev_actions = cmd.actions.copy()
    success = tellVerb(cmd) and 'ME WHAT IS THE' in cmd.sentence and objCompList(cmd) and singCategories(cmd) and onLocPrep(cmd) \
        and 'THE' in cmd.sentence and placementLocNames(cmd)
    if success:
        goal_location = get_pattern(cmd, _placementLocNames)
        actions = [['navigate', goal_location], ['find_objects', ''], ['say', 'random_object']]
        cmd.actions = actions
    else:
        cmd.actions = prev_actions
    return success

def bringMeObjFromPlcmt(cmd):
    prev_actions = cmd.actions.copy()
    success = bringVerb(cmd) and 'ME' in cmd.sentence and art(cmd) and objNames(cmd) and fromLocPrep(cmd) and 'THE' in cmd.sentence and placementLocNames(cmd)
    if success:
        goal_location = get_pattern(cmd, _placementLocNames)
        goal_object = get_pattern(cmd, _objNames)
        actions = [['navigate', goal_location], ['take', goal_object], ['navigate', 'start'], ['leave_object','']]
        cmd.actions = actions
    else:
        cmd.actions = prev_actions
    return success

def tellObjPropOnPlcmt(cmd):
    prev_actions = cmd.actions.copy()
    success = tellVerb(cmd) and 'ME WHAT IS THE' in cmd.sentence and objCompList(cmd) and 'OBJECT' in cmd.sentence and  onLocPrep(cmd) \
        and 'THE' in cmd.sentence and placementLocNames(cmd)
    if success:
        goal_location = get_pattern(cmd, _placementLocNames)
        actions = [['navigate', goal_location], ['find_objects', ''], ['say', 'random_object']]
        cmd.actions = actions
    else:
        cmd.actions = prev_actions
    return success

def countObjOnPlcmt(cmd):
    prev_actions = cmd.actions.copy()
    success = countVerb(cmd) and pluralCategories(cmd) and 'THERE ARE' in cmd.sentence and onLocPrep(cmd) and 'THE' in cmd.sentence and  placementLocNames(cmd)
    if success:
        goal_location = get_pattern(cmd, _placementLocNames)
        actions = [['navigate', goal_location], ['find_objects', ''], ['say', 'random_object']]
        cmd.actions = actions
    else:
        cmd.actions = prev_actions
    return success

def findObjInRoom(cmd):
    prev_actions = cmd.actions.copy()
    success = findVerb(cmd) and art(cmd) and obj_singCat(cmd) and inLocPrep(cmd) and 'THE' in cmd.sentence and  roomNames(cmd) \
        and 'THEN' in cmd.sentence and followUpFoundObj(cmd)
    if success:
        goal_room = get_pattern(cmd, _roomNames)
        goal_object = get_pattern(cmd, (_objNames + _singCategories))
        actions = [['navigate', goal_room],['take', goal_object]]
        cmd.actions = actions + cmd.actions
    else:
        cmd.actions = prev_actions
    return success

def takeObjFromPlcmt(cmd):
    prev_actions = cmd.actions.copy()
    success = takeVerb(cmd) and art(cmd) and obj_singCat(cmd) and fromLocPrep(cmd) and 'THE' in cmd.sentence and placementLocNames(cmd) \
        and 'AND' in cmd.sentence and hasObj(cmd)
    if success:
        goal_location = get_pattern(cmd, _placementLocNames)
        goal_object = get_pattern(cmd, (_objNames + _singCategories))
        actions = [['navigate', goal_location], ['take', goal_object]]
        cmd.actions = actions + cmd.actions
    else:
        cmd.actions = prev_actions
    return success


def followPrsAtLoc(cmd):
    prev_actions = cmd.actions.copy()
    success = followVerb(cmd) and 'THE' in cmd.sentence and gestPers_posePers(cmd) and  inRoom_atLoc(cmd)
    if success:
        goal_location = get_pattern(cmd, (_roomNames + _locationNames))
        gesture = get_pattern(cmd, (_gesturePersonList + _posePersonList))
        actions = [['navigate', goal_location], ['find_gesture', gesture], ['follow','']]
        cmd.actions += actions
    else:
        cmd.actions = prev_actions
    return success

def tellPrsInfoAtLocToPrsAtLoc(cmd):
    prev_actions = cmd.actions.copy()
    success = tellVerb(cmd) and 'THE' in cmd.sentence and personInfoList(cmd) and 'OF THE PERSON' and atLocPrep(cmd) and 'THE' in cmd.sentence and \
        locationNames(cmd) and 'TO THE PERSON' in cmd.sentence and atLocPrep(cmd) and 'THE' in cmd.sentence and locationNames(cmd)
    if success:
        parts = cmd.sentence.split('TO THE PERSON')
        goal_location1 = get_pattern_from_string(parts[0], _locationNames)
        goal_location2 = get_pattern_from_string(parts[1], _locationNames)
        actions = [['navigate', goal_location1], ['say', 'I am looking for person info'], ['navigate', goal_location2], ['find_person', ''],\
                   ['say','the person at the ' + goal_location1 + ' is doing something']]
        cmd.actions = actions
    else:
        cmd.actions = prev_actions
    return success

def countClothPrsInRoom(cmd):
    prev_actions = cmd.actions.copy()
    success = countVerb(cmd) and 'PEOPLE' in cmd.sentence and inLocPrep(cmd) and 'THE' in cmd.sentence and roomNames(cmd) \
        and 'ARE WEARING' in cmd.sentence and colorClothesList(cmd)
    if success:
        goal_location = get_pattern(cmd, _roomNames)
        goal_clothe = get_pattern(cmd, _colorClotheList)
        actions = [['navigate', goal_location], ['find_clothes', goal_clothe]]
        cmd.actions = actions
    else:
        cmd.actions = prev_actions
    return success

def meetNameAtLocThenFindInRm(cmd):
    prev_actions = cmd.actions.copy()
    success = meetVerb(cmd) and personNames(cmd) and atLocPrep(cmd) and 'THE' in cmd.sentence and locationNames(cmd) and \
        'THEN' in cmd.sentence and findVerb(cmd) and 'THEM' in cmd.sentence and inLocPrep(cmd) and 'THE' in cmd.sentence and roomNames(cmd)
    if success:
        parts = cmd.sentence.split('THEN')
        goal_location = get_pattern_from_string(parts[0], _locationNames)
        goal_room = get_pattern_from_string(parts[1], _roomNames)
        goal_person = get_pattern_from_string(parts[0], _personNames)
        actions = [['navigate', goal_location], ['find_person', goal_person], ['navigate', goal_room],['find_person', goal_person]]
        cmd.actions = actions
    else:
        cmd.actions = prev_actions
    return success

def greetNameInRm(cmd):
    prev_actions  = cmd.actions.copy()
    prev_sentence = cmd.sentence[:]
    success = greetVerb(cmd) and personNames(cmd) and inLocPrep(cmd) and 'THE' in cmd.sentence and roomNames(cmd) and 'AND' in cmd.sentence
    parts = []
    if success:
        parts = cmd.sentence.split(' AND ')
        cmd.sentence = parts[1]
    success = success and followUpFoundPers(cmd)
    if success:
        goal_room = get_pattern_from_string(parts[0], _roomNames)
        goal_person = get_pattern_from_string(parts[0], _personNames)
        actions = [['navigate', goal_room], ['find_person', goal_person]]
        cmd.actions = actions + cmd.actions
    else:
        cmd.actions = prev_actions
    cmd.sentence = prev_sentence
    return success

def greetClothDscInRm(cmd):
    prev_actions = cmd.actions.copy()
    prev_sentence = cmd.sentence[:]
    success = greetVerb(cmd) and 'THE PERSON WEARING' in cmd.sentence and art(cmd) and colorClotheList(cmd) and inLocPrep(cmd) \
        and 'THE' in cmd.sentence and roomNames(cmd) and 'AND' in cmd.sentence
    parts = []
    if success:
        parts = cmd.sentence.split(' AND ')
        cmd.sentence = parts[1]
    success = success and followUpFoundPers(cmd)
    if success:
        goal_room = get_pattern_from_string(parts[0], _roomNames)
        goal_clothes = get_pattern_from_string(parts[0], _colorClotheList)
        actions = [['navigate', goal_room], ['find_clothes', goal_clothes]]
        cmd.actions = actions + cmd.actions
    else:
        cmd.actions = prev_actions
    cmd.sentence = prev_sentence
    return success

def guideClothPrsFromBeacToBeac(cmd):
    prev_actions = cmd.actions.copy()
    success = guideVerb(cmd) and 'THE PERSON WEARING A' in cmd.sentence and  colorClotheList(cmd) and fromLocPrep(cmd) and 'THE' \
        and locationNames(cmd) and toLocPrep(cmd) and 'THE' in cmd.sentence and loc_room(cmd)
    if success:
        parts = cmd.sentence.split(' TO THE ')
        goal_location1 = get_pattern_from_string(parts[0], _locationNames)
        goal_location2 = get_pattern_from_string(parts[1], (_locationNames + _roomNames))
        goal_clothes = get_pattern_from_string(parts[0], _colorClotheList)
        actions = [['navigate', goal_location1],['find_clothes', goal_clothes], ['say', 'human, please follow me'], ['navigate', goal_location2]]
        cmd.actions = actions
    else:
        cmd.actions = prev_actions
    return success

def guidePrsFromBeacToBeac(cmd):
    prev_actions = cmd.actions.copy()
    success = guideVerb(cmd) and 'THE' in cmd.sentence and gestPers_posePers(cmd) and fromLocPrep(cmd) and 'THE' in cmd.sentence and locationNames(cmd) \
        and toLocPrep(cmd) and 'THE' in cmd.sentence and loc_room(cmd)
    if success:
        parts = cmd.sentence.split(' TO THE ')
        goal_location1 = get_pattern_from_string(parts[0], _locationNames)
        goal_location2 = get_pattern_from_string(parts[1], (_locationNames + _roomNames))
        goal_gesture = get_pattern_from_string(parts[0], (_gesturePersonList + _posePersonList))
        actions = [['navigate', goal_location1],['find_gesture', goal_gesture], ['say', 'human, please follow me'], ['navigate', goal_location2]]
        cmd.actions = actions
    else:
        cmd.actions = prev_actions
    return success

def guideNameFromBeacToBeac(cmd):
    prev_actions = cmd.actions.copy()
    success = guideVerb(cmd) and personNames(cmd) and fromLocPrep(cmd) and 'THE' and locationNames(cmd) and toLocPrep(cmd) and 'THE' in cmd.sentence and loc_room(cmd)
    if success:
        parts = cmd.sentence.split(' TO THE ')
        goal_location1 = get_pattern_from_string(parts[0], _locationNames)
        goal_location2 = get_pattern_from_string(parts[1], (_locationNames + _roomNames))
        goal_person = get_pattern_from_string(parts[0], _personNames)
        actions = [['navigate', goal_location1],['find_person', goal_person], ['say', 'human, please follow me'], ['navigate', goal_location2]]
        cmd.actions = actions
    else:
        cmd.actions = prev_actions
    return success

def followNameFromBeacToRoom(cmd):
    prev_actions = cmd.actions.copy()
    success = followVerb(cmd) and personNames(cmd) and fromLocPrep(cmd) and 'THE' in cmd.sentence and locationNames(cmd) \
        and toLocPrep(cmd) and 'THE' in cmd.sentence and roomNames(cmd)
    if success:
        parts = cmd.sentence.split(' TO THE ')
        goal_location1 = get_pattern_from_string(parts[0], _locationNames)
        goal_location2 = get_pattern_from_string(parts[1], _roomNames)
        goal_person = get_pattern_from_string(parts[0], _personNames)
        actions = [['navigate', goal_location1],['find_person', goal_person], ['say', 'human, please follow me'], ['navigate', goal_location2]]
        cmd.actions = actions
    else:
        cmd.actions = prev_actions
    return success

def answerToGestPrsInRoom(cmd):
    prev_actions = cmd.actions.copy()
    success = answerVerb(cmd) and 'THE' in cmd.sentence and questionList(cmd) and ofPrsPrep(cmd) and 'THE' in cmd.sentence and gesturePersonList(cmd) and\
        inLocPrep(cmd) and 'THE' in cmd.sentence and roomNames(cmd)
    if success:
        goal_room = get_pattern(cmd, _roomNames)
        goal_gesture = get_pattern(cmd, _gesturePersonList)
        actions = [['navigate', goal_room], ['find_gesture', goal_gesture], ['say', 'I will answer the question']]
        cmd.actions = actions
    else:
        cmd.actions = prev_actions
    return success

def talkInfoToGestPrsInRoom(cmd):
    prev_actions = cmd.actions.copy()
    success = talkVerb(cmd) and talkList(cmd) and talkPrep(cmd) and 'THE' in cmd.sentence and gesturePersonList(cmd) and inLocPrep(cmd) \
        and 'THE' in cmd.sentence and roomNames(cmd)
    if success:
        goal_room = get_pattern(cmd, _roomNames)
        goal_gesture = get_pattern(cmd, _gesturePersonList)
        question = get_pattern(cmd, _talkList)
        answer = talkListAnswers[question]
        actions = [['navigate', goal_room], ['find_gesture', goal_gesture], ['say', 'I will give the information'], ['say', answer]]
        cmd.actions = actions
    else:
        cmd.actions = prev_actions
    return success

def tellPrsInfoInLoc(cmd):
    prev_actions = cmd.actions.copy()
    success = tellVerb(cmd) and 'ME THE' in cmd.sentence and personInfoList(cmd) and 'OF THE PERSON' in cmd.sentence and inRoom_atLoc(cmd)
    if success:
        goal_location = get_pattern(cmd, (_roomNames + _locationNames))
        actions = [['navigate', goal_location], ['find_person', ''], ['say', 'the person is doing something']]
        cmd.actions = actions
    else:
        cmd.actions = prev_actions
    return success

def countPrsInRoom(cmd):
    prev_actions = cmd.actions.copy()
    success = countVerb(cmd) and gestPersPlur_posePersPlur(cmd) and ' ARE' in cmd.sentence and  inLocPrep(cmd) and 'THE' in cmd.sentence and roomNames(cmd)
    if success:
        goal_room = get_pattern(cmd, _roomNames)
        actions = [['navigate', goal_room],['find_gesture', ''],['say', 'I found two people']]
        cmd.actions = actions
    else:
        cmd.actions = prev_actions
    return success

def meetPrsAtBeac(cmd):
    prev_actions  = cmd.actions.copy()
    prev_sentence = cmd.sentence[:]
    success = meetVerb(cmd) and personNames(cmd) and inLocPrep(cmd) and 'THE' in cmd.sentence and roomNames(cmd) and 'AND' in cmd.sentence
    if success:
        parts = cmd.sentence.split(' AND ')
        cmd.sentence = parts[1]
    success = success and followUpFoundPers(cmd)
    if success:
        goal_room = get_pattern_from_string(parts[0], _roomNames)
        goal_person = get_pattern_from_string(parts[0], _personNames)
        actions = [['navigate', goal_room],['find_person', goal_person]]
        cmd.actions = actions + cmd.actions
    else:
        cmd.actions = prev_actions
    cmd.sentence = prev_sentence
    return success

def findPrsInRoom(cmd):
    prev_actions  = cmd.actions.copy()
    prev_sentence = cmd.sentence[:]
    success = findVerb(cmd) and 'A' in cmd.sentence and gestPers_posePers(cmd) and  inLocPrep(cmd) and 'THE' in cmd.sentence \
        and roomNames(cmd) and 'AND' in cmd.sentence
    if success:
        parts = cmd.sentence.split(' AND ')
        cmd.sentence = parts[1]
    success = success and followUpFoundPers(cmd)
    if success:
        goal_room = get_pattern_from_string(parts[0], _roomNames)
        goal_gesture = get_pattern_from_string(parts[0], (_gesturePerson + _posePersonList))
        actions = [['navigate', goal_room],['find_gesture', goal_gesture]]
        cmd.actions = actions + cmd.actions
    else:
        cmd.actions = prev_actions
    cmd.sentence = prev_sentence
    return success

def goToLoc(cmd):
    prev_actions  = cmd.actions.copy()
    prev_sentence = cmd.sentence[:]
    success = goVerb(cmd) and toLocPrep(cmd) and 'THE' in cmd.sentence and loc_room(cmd) and 'THEN' in cmd.sentence
    parts = []
    if success:
        parts = cmd.sentence.split('THEN')
        cmd.sentence = parts[1]
    success = success and followUpAtLoc(cmd)
    if success:
        goal_location = get_pattern_from_string(parts[0], (_locationNames + _roomNames))
        actions = [['navigate', goal_location]]
        cmd.actions = actions + cmd.actions
    else:
        cmd.actions = prev_actions
    cmd.sentence = prev_sentence
    return success

def getCommnandType(cmd):
    if goToLoc(cmd):
        return 'goToLoc'
    if findPrsInRoom(cmd):
        return 'findPrsInRoom'
    if meetPrsAtBeac(cmd):
        return 'meetPrsAtBeac'
    if countPrsInRoom(cmd):
        return 'countPrsInRoom'
    if tellPrsInfoInLoc(cmd):
        return 'tellPrsInfoInLoc'
    if talkInfoToGestPrsInRoom(cmd):
        return 'talkInfoToGestPrsInRoom'
    if answerToGestPrsInRoom(cmd):
        return 'answerToGestPrsInRoom'
    if followNameFromBeacToRoom(cmd):
        return 'followNameFromBeacToRoom'
    if guideNameFromBeacToBeac(cmd):
        return 'guideNameFromBeacToBeac'
    if guidePrsFromBeacToBeac(cmd):
        return 'guidePrsFromBeacToBeac'
    if guideClothPrsFromBeacToBeac(cmd):
        return 'guideClothPrsFromBeacToBeac'
    if greetClothDscInRm(cmd):
        return 'greetClothDscInRm'
    if greetNameInRm(cmd):
        return 'greetNameInRm'
    if meetNameAtLocThenFindInRm(cmd):
        return 'meetNameAtLocThenFindInRm'
    if countClothPrsInRoom(cmd):
        return 'countClothPrsInRoom'
    if tellPrsInfoAtLocToPrsAtLoc(cmd):
        return 'tellPrsInfoAtLocToPrsAtLoc'
    if followPrsAtLoc(cmd):
        return 'followPrsAtLoc'
    if takeObjFromPlcmt(cmd):
        return 'takeObjFromPlcmt'
    if findObjInRoom(cmd):
        return 'findObjInRoom'
    if countObjOnPlcmt(cmd):
        return 'countObjOnPlcmt'
    if tellObjPropOnPlcmt(cmd):
        return 'tellObjPropOnPlcmt'
    if bringMeObjFromPlcmt(cmd):
        return 'bringMeObjFromPlcmt'
    if tellCatPropOnPlcmt(cmd):
        return 'tellCatPropOnPlcmt'
    return None
