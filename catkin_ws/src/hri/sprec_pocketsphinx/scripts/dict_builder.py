def build_dict(word_list, cmudict):
    print("Reading CMU dictionary...")
    f = open(cmudict, 'r')
    lines = f.readlines()
    words = {}
    for l in lines:
        parts = l.split()
        words[parts[0]] = ' '.join(parts[1:])
    print("Checking word list..")
    grammar_dic = []
    for w in word_list:
        if w.upper() in words.keys():
            grammar_dic.append(w.upper() + " " + words[w.upper()])
            #print("Added " + w)
        else:
            print("Cannot find word " + w.upper() + " in CMU dict")
    print("Writing to file gpsr.dic")
    with open('gpsr.dic', 'w') as f:
        for line in grammar_dic:
            f.write(f"{line}\n")

def get_command_word_list():
    verb_dict = {
        "take": ["take", "get", "grasp", "fetch"],
        "place": ["put", "place"],
        "deliver": ["bring", "give", "deliver"],
        "bring": ["bring", "give"],
        "go": ["go", "navigate"],
        "find": ["find", "locate", "look for"],
        "talk": ["tell", "say"],
        "answer": ["answer"],
        "meet": ["meet"],
        "tell": ["tell"],
        "greet": ["greet", "salute", "say hello to", "introduce yourself to"],
        "remember": ["meet", "contact", "get to know", "get acquainted with"],
        "count": ["tell me how many"],
        "describe": ["tell me how", "describe"],
        "offer": ["offer"],
        "follow": ["follow"],
        "guide": ["guide", "escort", "take", "lead"],
        "accompany": ["accompany"]
    }

    prep_dict = {
        "deliverPrep": ["to"],
        "placePrep": ["on"],
        "inLocPrep": ["in"],
        "fromLocPrep": ["from"],
        "toLocPrep": ["to"],
        "atLocPrep": ["at"],
        "talkPrep": ["to"],
        "locPrep": ["in", "at"],
        "onLocPrep": ["on"],
        "arePrep": ["are"],
        "ofPrsPrep": ["of"]
    }

    connector_list = ["and"]
    gesture_person_list = ["waving person", "person raising their left arm", "person raising their right arm",
                           "person pointing to the left", "person pointing to the right"]
    pose_person_list = ["sitting person", "standing person", "lying person"]
    # Ugly...
    gesture_person_plural_list = ["waving persons", "persons raising their left arm", "persons raising their right arm",
                                  "persons pointing to the left", "persons pointing to the right"]
    pose_person_plural_list = ["sitting persons", "standing persons", "lying persons"]

    person_info_list = ["name", "pose", "gesture"]
    object_comp_list = ["biggest", "largest", "smallest", "heaviest", "lightest", "thinnest"]

    talk_list = ["something about yourself", "the time", "what day is today", "what day is tomorrow", "your teams name",
                 "your teams country", "your teams affiliation", "the day of the week", "the day of the month"]
    question_list = ["question", "quiz"]

    color_list = ["blue", "yellow", "black", "white", "red", "orange", "gray"]
    clothe_list = ["t shirt", "shirt", "blouse", "sweater", "coat", "jacket"]
    clothes_list = ["t shirts", "shirts", "blouses", "sweaters", "coats", "jackets"]

    cmd_lists = connector_list + gesture_person_list + pose_person_list + gesture_person_plural_list + pose_person_plural_list
    cmd_lists+= person_info_list + object_comp_list + talk_list + question_list + color_list + clothe_list + clothes_list

    names = ['Adel', 'Angel', 'Axel', 'Charlie', 'Jane', 'Jules', 'Morgan', 'Paris', 'Robin', 'Simone', 'bed',
             'bedside table', 'shelf', 'trashbin', 'dishwasher', 'potted plant', 'kitchen table', 'chairs', 'pantry',
             'refrigerator', 'sink', 'cabinet', 'coatrack', 'desk', 'armchair', 'desk lamp', 'waste basket', 'tv stand',
             'storage rack', 'lamp', 'side tables', 'sofa', 'bookshelf', 'entrance', 'exit', 'bed', 'bedside table',
             'shelf', 'dishwasher', 'kitchen table', 'pantry', 'refrigerator', 'sink', 'cabinet', 'desk', 'tv stand',
             'storage rack', 'side tables', 'sofa', 'bookshelf','bedroom', 'kitchen', 'office', 'living room', 'bathroom',
             'pringles', 'cheezit', 'cornflakes', 'sponge', 'cleanser', 'sugar', 'tomato soup', 'chocolate jello',
             'strawberry jello', 'mustard', 'tuna', 'spam', 'coffee grounds', 'orange', 'banana', 'strawberry', 'peach',
             'pear', 'apple', 'plum', 'lemon', 'fork', 'spoon', 'bowl', 'knife', 'plate', 'cup', 'milk', 'tropical juice',
             'iced tea', 'orange juice', 'juice pack', 'red wine', 'cola', 'baseball', 'tennis ball', 'dice', 'rubiks cube',
             'soccer ball','snacks', 'cleaning supplies', 'food', 'fruits', 'dishes', 'drinks', 'toys','snack',
             'cleaning supply', 'food', 'fruit', 'dish', 'drink', 'toy']


    words = []
    for k in verb_dict.keys():
        for v in verb_dict[k]:
            parts = v.split()
            for p in parts:
                if p not in words:
                    words.append(p)
    for k in prep_dict.keys():
        for p in prep_dict[k]:
            if p not in words:
                words.append(p)
    for c in cmd_lists:
        parts = c.split()
        for p in parts:
            if p not in words:
                words.append(p)

    for n in names:
        parts = n.split()
        for p in parts:
            if p not in words:
                words.append(p)
    return words

if __name__ == '__main__':
    words = get_command_word_list()
    print("Total words: ", len(words))
    build_dict(words, "cmudict.dic")
