
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

def has_pattern(command, word_list):
    cmd = command
    for w in word_list:
        if w in cmd:
            return True#, cmd.replace(w, '')
    return False

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
    return 'AT THE' in cmd and locationNames(cmd);

def inRoom(cmd):
    return 'IN THE' and roomNames(cmd);

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

def guidePrsToBeacon(cmd):
    return guideVerb(cmd) and 'THEM' in cmd and toLocPrep(cmd) and 'THE' in cmd and loc_room(cmd)

def followPrsToRoom(cmd):
    return followVerb(cmd) and 'THEM' in cmd and toLocPrep(cmd) and 'THE' in cmd and loc_room(cmd)

def followPrs(cmd):
    return followVerb(cmd) and 'THEM' in cmd

def answerQuestion(cmd):
    return answerVerb(cmd)  and 'A' in cmd and questionList(cmd)

def talkInfo(cmd):
    return talkVerb(cmd) and talkList(cmd)

def deliverObjToNameAtBeac(cmd):
    return deliverVerb(cmd) and 'IT' in cmd and deliverPrep(cmd) and personNames(cmd) and inLocPrep(cmd) and 'THE' in cmd and roomNames(cmd)

def deliverObjToPrsInRoom(cmd):
    return deliverVerb(cmd) and 'IT' in cmd and deliverPrep(cmd) and 'THE' in cmd and gestPers_posePers(cmd) and inLocPrep(cmd) and 'THE' in cmd and roomNames(cmd)

def deliverObjToMe(cmd):
    return deliverVerb(cmd) and 'IT TO ME' in cmd

def placeObjOnPlcmt(cmd):
    return placeVerb(cmd) and 'IT' in cmd and onLocPrep(cmd) and 'THE' in cmd and placementLocNames(cmd)

def hasObj(cmd):
    return placeObjOnPlcmt(cmd) or deliverObjToMe(cmd) or deliverObjToPrsInRoom(cmd) or deliverObjToNameAtBeac(cmd)

def foundPers(cmd):
    return talkInfo(cmd) or answerQuestion(cmd) or followPrs(cmd) or followPrsToRoom(cmd) or guidePrsToBeacon(cmd)

def findObj(cmd):
    return findVerb(cmd) and art(cmd) and obj_singCat(cmd) and 'AND' in cmd and  takeVerb(cmd) and 'IT AND' in cmd and  hasObj(cmd)

def meetName(cmd):
    return meetVerb(cmd) and personNames(cmd) and 'AND' in cmd and foundPers(cmd)

def findPrs(cmd):
    return findVerb(cmd) and 'THE' in cmd and gestPers_posePers(cmd) and 'AND' in cmd and foundPers(cmd)


def followUpFoundObj(cmd):
    return takeVerb(cmd) and 'IT AND' in cmd and hasObj(cmd)

def followUpFoundPers(cmd):
    return foundPers(cmd)

def followUpAtLoc(cmd):
    return findPrs(cmd) or meetName(cmd) or findObj(cmd)

def tellCatPropOnPlcmt(cmd):
    return tellVerb(cmd) and 'ME WHAT IS THE' in cmd and objCompList(cmd) and singCategories(cmd) and onLocPrep(cmd) and 'THE' in cmd and placementLocNames(cmd)

def bringMeObjFromPlcmt(cmd):
    return bringVerb(cmd) and 'ME' in cmd and art(cmd) and objNames(cmd) and fromLocPrep(cmd) and 'THE' in cmd and placementLocNames(cmd)

def tellObjPropOnPlcmt(cmd):
    return tellVerb(cmd) and 'ME WHAT IS THE' in cmd and objCompList(cmd) and 'OBJECT' in cmd and  onLocPrep(cmd) and 'THE' in cmd and placementLocNames(cmd)

def countObjOnPlcmt(cmd):
    return countVerb(cmd) and pluralCategories(cmd) and 'THERE ARE' in cmd and onLocPrep(cmd) and 'THE' in cmd and  placementLocNames(cmd)

def findObjInRoom(cmd):
    return findVerb(cmd) and art(cmd) and obj_singCat(cmd) and inLocPrep(cmd) and 'THE' in cmd and  roomNames(cmd) and 'THEN' in cmd and followUpFoundObj(cmd)

def takeObjFromPlcmt(cmd):
    return takeVerb(cmd) and art(cmd) and obj_singCat(cmd) and fromLocPrep(cmd) and 'THE' in cmd and placementLocNames(cmd) and 'AND' in cmd and hasObj(cmd)


def followPrsAtLoc(cmd):
    return followVerb(cmd) and 'THE' in cmd and gestPers_posePers(cmd) and  inRoom_atLoc(cmd)

def tellPrsInfoAtLocToPrsAtLoc(cmd):
    return tellVerb(cmd) and 'THE' in cmd and personInfoList(cmd) and 'OF THE PERSON' and atLocPrep(cmd) and 'THE' in cmd and \
        locationNames(cmd) and 'TO THE PERSON' in cmd and atLocPrep(cmd) and 'THE' in cmd and locationNames(cmd)

def countClothPrsInRoom(cmd):
    return countVerb(cmd) and 'PEOPLE' in cmd and inLocPrep(cmd) and 'THE' in cmd and roomNames(cmd) and 'ARE WEARING' in cmd and colorClothesList(cmd)

def meetNameAtLocThenFindInRm(cmd):
    return meetVerb(cmd) and personNames(cmd) and atLocPrep(cmd) and 'THE' in cmd and locationNames(cmd) and \
        'THEN' in cmd and findVerb(cmd) and 'THEM' in cmd and inLocPrep(cmd) and 'THE' in cmd and roomNames(cmd)

def greetNameInRm(cmd):
    return greetVerb(cmd) and personNames(cmd) and inLocPrep(cmd) and 'THE' in cmd and roomNames(cmd) and 'AND' in cmd and followUpFoundPers(cmd)

def greetClothDscInRm(cmd):
    return greetVerb(cmd) and 'THE PERSON WEARING' in cmd and art(cmd) and colorClotheList(cmd) and inLocPrep(cmd) \
        and 'THE' in cmd and roomNames(cmd) and 'AND' in cmd and followUpFoundPers(cmd)

def guideClothPrsFromBeacToBeac(cmd):
    return guideVerb(cmd) and 'THE PERSON WEARING A' in cmd and  colorClotheList(cmd) and fromLocPrep(cmd) and 'THE' \
        and locationNames(cmd) and toLocPrep(cmd) and 'THE' in cmd and loc_room(cmd)

def guidePrsFromBeacToBeac(cmd):
    return guideVerb(cmd) and 'THE' in cmd and gestPers_posePers(cmd) and fromLocPrep(cmd) and 'THE' in cmd and locationNames(cmd) \
        and toLocPrep(cmd) and 'THE' in cmd and loc_room(cmd)

def guideNameFromBeacToBeac(cmd):
    return guideVerb(cmd) and personNames(cmd) and fromLocPrep(cmd) and 'THE' and locationNames(cmd) and toLocPrep(cmd) and 'THE' in cmd and loc_room(cmd)

def followNameFromBeacToRoom(cmd):
    return followVerb(cmd) and personNames(cmd) and fromLocPrep(cmd) and 'THE' in cmd and locationNames(cmd) and toLocPrep(cmd) and 'THE' in cmd and roomNames(cmd)

def answerToGestPrsInRoom(cmd):
    return answerVerb(cmd) and 'THE' in cmd and questionList(cmd) and ofPrsPrep(cmd) and 'THE' in cmd and gesturePersonList(cmd) and\
        inLocPrep(cmd) and 'THE' in cmd and roomNames(cmd)

def talkInfoToGestPrsInRoom(cmd):
    return talkVerb(cmd) and talkList(cmd) and talkPrep(cmd) and 'THE' in cmd and gesturePersonList(cmd) and inLocPrep(cmd) and 'THE' in cmd and roomNames(cmd)

def tellPrsInfoInLoc(cmd):
    return tellVerb(cmd) and 'ME THE' in cmd and personInfoList(cmd) and 'OF THE PERSON' in cmd and inRoom_atLoc(cmd)

def countPrsInRoom(cmd):
    return countVerb(cmd) and gestPersPlur_posePersPlur(cmd) and ' ARE' in cmd and  inLocPrep(cmd) and 'THE' in cmd and roomNames(cmd)

def meetPrsAtBeac(cmd):
    return meetVerb(cmd) and personNames(cmd) and inLocPrep(cmd) and 'THE' in cmd and roomNames(cmd) and 'AND' in cmd and followUpFoundPers(cmd)

def findPrsInRoom(cmd):
    return findVerb(cmd) and 'A' in cmd and gestPers_posePers(cmd) and  inLocPrep(cmd) and 'THE' in cmd and roomNames(cmd) and 'AND' in cmd and followUpFoundPers(cmd)

def goToLoc(cmd):
    return goVerb(cmd) and toLocPrep(cmd) and 'THE' in cmd and loc_room(cmd) and 'THEN' in cmd and followUpAtLoc(cmd)

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
