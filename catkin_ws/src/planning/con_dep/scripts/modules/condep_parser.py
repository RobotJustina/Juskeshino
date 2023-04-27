# Import the package
import numpy as np
import spacy
import rospy
import traceback
import os

CD_structures = { "go" : "PTRANS",
"navigate" : "PTRANS",
"walk" : "PTRANS",
"lead" : "PTRANS",
"guide" : "PTRANS",
#===================
"bring" : "ATRANS",
"give" : "ATRANS",
"deliver" : "ATRANS",
#===================
"take" : "GRAB",
"grasp" : "GRAB",
#===================
"find" : "ATTEND",
"look" : "ATTEND",
"meet" : "ATTEND",
#===================
"deposit" : "RELEASE",
#===================
"open" : "PROPEL",
"close" : "PROPEL",
#===================
"tell" : "SPEAK",
"say" : "SPEAK",
#===================
"remind" : "MTRANS",
#===================
"follow" : "FTRANS",
    }

def CondepParser(text):
    
    print("=====================================================================")
    print("|                          SPACY PARSER                              |")
    print("=====================================================================")
    
    dependencies_list = []
    
    verb_list = []
    prim_list = []
    pron_list = []
    pos_list = []
    
    print("Input sentence: %s"%text)
    
    #Remove the word "Robot" in order to obtain a clean separation of the types of "obj"
    text = text.replace('Robot, ', '')
    
    #Extracting tokens from the sentence
    nlp = None
    try:
        nlp = spacy.load("en_core_web_sm")
    except:
        os.system("python -m spacy download en_core_web_sm")
        nlp = spacy.load("en_core_web_sm")

    res = text.split()
    verbs = ["go", "walk", "lead", "guide","follow", "bring", "give", "deliver", "take", "grasp", "find", "look", "meet", "deposit", "open", "close", "tell", "say", "remind", "follow"]
    for x in res:
        if x.lower() in verbs:
            text = text.replace(x, x.lower())
    
    doc = nlp(text)
    
    try:
        #Creating the list of nouns and positions
        text_list = [token.text for token in doc]
        pos_list1 = [token.pos_ for token in doc]
        pron_list = [chunk.text for chunk in doc.noun_chunks]
        pos_list = [[t.pos_ for t in chunk]for chunk in doc.noun_chunks]
        verb_list = [token.lemma_ for token in doc if token.pos_ == "VERB"]
        
        #print("=====================================================================")
        #============================================================================
        #                          WHICH QUESTION?                                    
        #============================================================================
        question_list = []
        for qu in text_list:
            qu = qu.lower()
            if qu in ["where", "what", "who"]:
                question_list.append(qu)
        #print(question_list) #<------------------------NOTE
        
        #elif text_list[-1] == '?':
            #verb_list = text_list[0]
        
        #If doesn't exist verb or nouns then send a request for a new sentece
        if len(verb_list) == 0 and len(question_list) == 0:
            print("I can't understand you, give me a valid command.")
        else:
            print("FINDING THE SENTENCES: ")
            txt = text
            #Sentences where her o him are included, ex: ... and give an apple to her
            #Check for the words "her" or "him" and replace them with the last PROPN
            acus_list = []
            pers_list = []
            for pron in range(len(pron_list)):
                if pron_list[pron] == 'her' or pron_list[pron] == "him":
                    acus_list.append(pron)
                for p in pos_list[pron]:
                    if p == "PROPN":
                        pers_list.append(pron)
            
        	#print(acus_list)
        	#print(pers_list)
            
            d = []
            for f in range(len(acus_list)):
                dis = np.array(pers_list) - acus_list[f]
                a=-1000
                for poc in range(len(dis)):
                    if(dis[poc]>a and dis[poc]<0):
                        a=dis[poc]
                        ind = poc
                txt = txt.replace(pron_list[acus_list[f]],pron_list[pers_list[ind]])
            
            txt = txt.replace(", and", ".")
            txt = txt.replace(" and", ".")
            txt = txt.replace(",", ".")
            
            doc = nlp(txt)
            assert doc.has_annotation("SENT_START")
            list_sentences =[sent.text for sent in doc.sents]
            for s in range(len(list_sentences)):
                list_sentences[s] = list_sentences[s].replace('.', '')
            print(list_sentences)
            
            #print("=====================================================================")
            
            location_list = []
            object_list = []
            person_list = []
            
            for sen in list_sentences:
                doc1 = nlp(sen)
                #Creating the list of nouns and positions
                text_list_sen = [token.text for token in doc1]
                pos_list_sen = [token.pos_ for token in doc1]
                pron_list_sen = [chunk.text for chunk in doc1.noun_chunks]
                pos_list_sen1 = [[t.pos_ for t in chunk]for chunk in doc1.noun_chunks]
                verb_list_sen = [token.lemma_ for token in doc1 if token.pos_ == "VERB"]
                
                #========================================================================
                #                          VERB OR QUESTION?                             
                #========================================================================
                
                verb_list_sen = [token.lemma_ for token in doc1 if token.pos_ == "VERB"]
                flag1 = 0
                for rem in verb_list_sen:
                    if rem == "remind":
                        flag1 = 1
                        break
                
                if len(verb_list_sen) != 0 and len(question_list) == 0:
                    if flag1 == 0:
                        prim = CD_structures[verb_list_sen[-1]]
                        #print(verb_list_sen)
                    else:
                        verb_list_sen.remove(verb_list_sen[-1])
                        prim = CD_structures[verb_list_sen[-1]]
                
                elif len(verb_list_sen) == 0 and len(question_list) != 0:
                    prim = "QTRANS"
                elif len(verb_list_sen) != 0 and len(question_list) != 0:
                    prim = "QTRANS"
                
                check_prim = len(pron_list_sen)
                
                verb_list_sen = [token.lemma_ for token in doc1 if token.pos_ == "VERB"]
                flag1 = 0
                for rem in verb_list_sen:
                    if rem == "remind": 
                        flag1 = 1
                        break
                
                print("=====================================================================")
                print("                    CONCEPTUAL DEPENDENCIES                          ")
                print("====================================================================")
                
                #==============================PTRANS=======================================
                if prim == 'PTRANS':
                    person = "robot"
                    for pron in range(len(pron_list_sen)):
                        if pos_list_sen1[pron][-1] == 'PROPN' or pron_list[pron] == "me":
                            person=pron_list_sen[pron]
                            pron_list_sen.remove(person)
                            break

                    loc2 = "nil"
                    for ex in pron_list_sen:
                        chop_noun3 = ex.split()
                        len_noun4 = len(chop_noun3)
                        f_noun4 = chop_noun3[-1]
                        idx5 = text_list_sen.index(f_noun4)
                        idx6 = idx5-len_noun4
                        if text_list_sen[idx6] == "to" and ex not in ["the", "a", "an"]:
                            loc2 = ex
                            pron_list_sen.remove(ex)

                    loc1 = "nil"
                    for ex in pron_list_sen:
                        chop_noun3 = ex.split()
                        len_noun4 = len(chop_noun3)
                        f_noun4 = chop_noun3[-1]
                        idx5 = text_list_sen.index(f_noun4)
                        idx6 = idx5-len_noun4
                        if text_list_sen[idx6] == "from" and ex not in ["the", "a", "an"]:
                            loc1 = ex
                            pron_list_sen.remove(ex)


                    if verb_list_sen[-1] in ["go", "walk", "navigate"] and len(pron_list_sen)!= 0:
                        if person == "nil" and loc1 == "nil":
                            person = "Robot"
                            loc1 = "Robot place"
                        elif person == "nil" and loc1 != "nil":
                            person = "Robot"


                    
                    dependencies_list.append(prim+'((ACTOR Robot)(OBJ '+person+')(FROM '+loc1+')(TO '+loc2+'))')
                    #print(prim+'((ACTOR Robot)(OBJ '+person+')(FROM '+loc1+')(TO '+loc2+'))')

                
                #==============================ATRANS=======================================
                #ATRANS ------> a / an [object], [person]
                elif prim == 'ATRANS':
                    if len(pron_list_sen) != 0:
                        person='nil'
                        for pron in range(len(pron_list_sen)):
                            if pos_list_sen1[pron][-1] == 'PROPN' or pron_list[pron] == "me":
                                person=pron_list_sen[pron]
                                pron_list_sen = np.delete(pron_list_sen, pron)
                                break
                        
                        if person != 'nil': #There is a person
                            if len(pron_list_sen) != 0 and pron_list_sen[-1] not in ["an", "a", "the"]: #There is a person and a valid object
                                obj = pron_list_sen[-1]
                                dependencies_list.append(prim+'((ACTOR Robot)(OBJ '+obj+')(TO '+person+'))')
                                #print(prim+'((ACTOR Robot)(OBJ '+obj+')(TO '+person+'))') 
                            else: #There is a person but no an object
                                dependencies_list.append(prim+'((ACTOR Robot)(OBJ nil)(TO '+person+'))')
                                #print(prim+'((ACTOR Robot)(OBJ nil)(TO '+person+'))')
                        else:
                            if len(pron_list_sen) != 0 and pron_list_sen[-1] in ["an", "a", "the"]: #There is no person and the obj in an/a/the
                                dependencies_list.append(prim+'((ACTOR Robot)(OBJ nil)(TO nil))')
                                #print(prim+'((ACTOR Robot)(OBJ nil)(FROM nil)(TO nil))')
                            elif len(pron_list_sen) == 0  and pron_list_sen[-1] not in ["an", "a", "the"]:
                                obj = pron_list_sen[-1]
                                dependencies_list.append(prim+'((ACTOR Robot)(OBJ '+obj+')(TO nil))')
                            elif len(pron_list_sen) >1:
                                for ex in pron_list_sen:
                                    chop_noun3 = ex.split()
                                    len_noun4 = len(chop_noun3)
                                    f_noun4 = chop_noun3[-1]
                                    idx5 = text_list_sen.index(f_noun4)
                                    idx6 = idx5-len_noun4
                                    if text_list_sen[idx6] in ["on", "at", "over", "in", "to"]:
                                        place = ex
                                        pron_list_sen.remove(ex)
                                        
                                        if pron_list_sen[-1] not in ["an", "a", "the"]:
                                                obj = pron_list_sen[-1]
                                        else:
                                                obj = "nil"
                                dependencies_list.append(prim+'((ACTOR Robot)(OBJ '+obj+')(FROM '+obj+' place)(TO '+place+'))')
                                #print(prim+'((ACTOR Robot)(OBJ '+obj+')(FROM '+obj+' place)(TO '+place+'))')	
                    
                    else:
                        dependencies_list.append(prim+'((ACTOR Robot)(OBJ nil)(TO nil))') #There is no person and no obj
                        #print(prim+'((ACTOR Robot)(OBJ nil)(TO nil))')
                
                #==============================GRAB=======================================
                elif prim == "GRAB":
                    #GRAB -----> [an object]
                    if len(pron_list_sen) == 0:
                        dependencies_list.append(prim+'((ACTOR Robot)(OBJ nil))')
                        #print(prim+'((ACTOR Robot)(OBJ nil))')
                    else:
                        if pron_list_sen[-1] in ["a", "an", "the"]:
                            dependencies_list.append(prim+'((ACTOR Robot)(OBJ nil))')
                            #print(prim+'((ACTOR Robot)(OBJ nil))')
                        else:
                            obj = pron_list_sen[-1]
                            dependencies_list.append(prim+'((ACTOR Robot)(OBJ '+obj+'))')
                            #print(prim+'((ACTOR Robot)(OBJ '+obj+'))')
                
                #==============================ATTEND=======================================
                #ATTEND -----> [an object] / [a person]
                elif prim == "ATTEND":
                    at = "nil"
                    for ex in pron_list_sen:
                                    chop_noun3 = ex.split()
                                    len_noun4 = len(chop_noun3)
                                    f_noun4 = chop_noun3[-1]
                                    idx5 = text_list_sen.index(f_noun4)
                                    idx6 = idx5-len_noun4
                                    #print(idx6)
                                    if text_list_sen[idx6] in ["on", "at","in", "to"]:
                                        at = ex
                                        pron_list_sen.remove(ex)
                    obj = "nil"
                    #person = "nil"
                    if len(pron_list_sen) != 0 and pron_list_sen[-1] not in ["a", "an", "the"]:
                        obj = pron_list_sen[-1]
                        dependencies_list.append(prim+'((ACTOR Robot)(OBJ '+obj+')(AT '+at+'))')
                        print(prim+'((ACTOR Robot)(OBJ '+obj+')(AT '+at+'))')
                    else:
                        dependencies_list.append(prim+'((ACTOR Robot)(OBJ nil)(AT '+at+'))')
                        print(prim+'((ACTOR Robot)(OBJ nil)(AT '+at+'))')

                    
                #==============================RELEASE=======================================
                elif prim == "RELEASE":
                    #print(pron_list_sen)
                    for serc_noun in pron_list_sen:
                        chop_noun = serc_noun.split()
                        len_noun = len(chop_noun)
                        #print(len_noun)
                        f_noun = chop_noun[-1]
                        idx3 = text_list_sen.index(f_noun) 
                        idx4 = idx3-len_noun
                        #print(text_list_sen[idx4])
                        
                        if text_list_sen[idx4] in ["on", "at", "over", "in", "to", "into"]:
                            place = serc_noun
                            pron_list_sen.remove(serc_noun)
                            #print("place: ", place)
                            #print(pron_list_sen)
                            #break
                        else: 
                            place = "nil"
                    
                    if len(pron_list_sen) != 0 and pron_list_sen[-1] not in ["an", "a", "the"]:
                        obj = pron_list_sen[-1]
                    elif len(pron_list_sen) != 0 and pron_list_sen[-1] in ["an", "a", "the"]:
                        obj = "nil"
                    else:
                        obj = "nil"
                    
                    #print("place: ", place)
                    #print("object: ", obj)
                    dependencies_list.append(prim+'((ACTOR Robot)(OBJ '+obj+')(TO '+place+'))')
                    #print(prim+'((ACTOR Robot)(OBJ '+obj+')(TO '+place+'))')
                
                #==============================PROPEL=======================================
                elif prim == "PROPEL":
                    #PROPEL -----> [an object]
                    action = verb_list_sen[-1]
                    if len(pron_list_sen) == 0:
                        dependencies_list.append(prim+'((ACTION '+action+')(ACTOR Robot)(OBJ nil))')
                        #print(prim+'((ACTION '+action+')((ACTOR Robot)(OBJ nil))')
                    else:
                        if pron_list_sen[-1] in ["a", "an", "the"]:
                            dependencies_list.append(prim+'((ACTION '+action+')((ACTOR Robot)(OBJ nil))')
                            #print(prim+'((ACTION '+action+')((ACTOR Robot)(OBJ nil))')
                        else:
                            obj = pron_list_sen[-1]
                            dependencies_list.append(prim+'((ACTION '+action+')((ACTOR Robot)(OBJ '+obj+'))')
                            #print(prim+'((ACTION '+action+')((ACTOR Robot)(OBJ '+obj+'))')
                
                #==============================QTRANS=======================================
                elif prim == "QTRANS":
                    for qsen in text_list_sen:
                        qsen = qsen.lower()
                        if qsen in ["where", "what", "who"]:
                            question = qsen
                            #print(question)
                    if len(pron_list_sen) == 0:
                        dependencies_list.append(prim+'((OBJ nil)(QUESTION '+question+'))')
                        #print(prim+'((OBJ nil)(OBJ'+question+'))')
                    elif len(pron_list_sen) != 0 and pron_list_sen[-1] in ["a", "an", "the"]:
                        dependencies_list.append(prim+'((OBJ nil)(QUESTION '+question+'))')
                        #print(prim+'((OBJ nil)(OBJ '+question+'))') 
                    else:
                        obj = pron_list_sen[-1]
                        dependencies_list.append(prim+'((OBJ '+obj+')(QUESTION '+question+'))')
                        #print(prim+'((OBJ '+obj+')(QUESTION '+question+'))')
                
                #==============================SPEAK=======================================
                elif prim == "SPEAK": 
                    #tell [to someone] [something]
                    #tell hello? -> not correct
                    #print(len(text_list_sen))
                    if verb_list_sen[-1] == "tell" and len(text_list_sen)==1: 
                        sent = "nil"
                        dependencies_list.append(prim+'((MSG nil)(TO nil))')
                        #print(prim+'((MSG nil)(TO nil))')
                    elif verb_list_sen[-1] == "tell" and len(text_list_sen)>1: 
                        #print(pos_list_sen[1])
                        #print(text_list_sen[1])
                        if pos_list_sen[1] == "PROPN" or pos_list_sen[1] == "PRON":
                            person = text_list_sen[1]
                            sent = sen.replace((text_list_sen[0])+" ", "")
                            sent = sent.replace((text_list_sen[1])+" ", "")
                            sent = sent.strip()
                            #print(len(sent))
                            #print(sent)
                            if len(sent)!=0:
                                dependencies_list.append(prim+'((MSG '+sent+')(TO '+person+'))')
                                #print(prim+'((MSG '+sent+')(TO '+person+'))')
                            else:
                                dependencies_list.append(prim+'((MSG nil)(TO '+person+'))')
                                #print(prim+'((MSG nil)(TO '+person+'))')
                        else:
                            sent = sen.replace(text_list_sen[0] , "")
                            sent = sent.strip()
                            dependencies_list.append(prim+'((MSG '+sent+')(TO nil))')
                            #print(prim+'((MSG '+sent+')(TO nil))')
                    
                    elif verb_list_sen[-1] == "say" and len(text_list_sen)<=1:
                        #say [something]
                        sent = "nil"
                        dependencies_list.append(prim+'((MSG '+sent+'))')
                        #print(prim+'((MSG '+sent+'))')
                    else:
                        sent = sen.replace(text_list_sen[0] , "")
                        sent = sent.strip()
                        dependencies_list.append(prim+'((MSG '+sent+'))')
                        #print(prim+'((MSG '+sent+'))')		    
                
                #==============================MTRANS=======================================
                elif prim == "MTRANS":
                    #(MTRANS (ACTOR Robot)(MSG sentence)(FROM source)(TO goal))
                    #"Robot, please remind to call John"
                    #(mtrans (actor robot)(msg call John)(from human)(to robot's memory))
                    #"Robot, please remind me to call John"
                    #(mtrans (actor robot)(msg call John)(from robots memory)(to humans memory))
                    ind = 1000
                    for w in range(len(text_list_sen)):
                        if text_list_sen[w] == "to":
                            ind2 = w
                            break
                    
                    if pos_list1[w-1] == "PRON" or pos_list1[w-1] == "PROPN":
                        goal = text_list_sen[w-1]
                        source = "robot"
                    
                    else:
                        goal = "robot"
                        source = "human"
                    
                    if w != 1000:
                        sent2 = sen
                        for wd in range(ind2):
                            sent2 = sent2.replace((text_list_sen[wd])+" ", "")
                    
                    sent2 = sent.strip()
                    dependencies_list.append(prim+'((ACTOR Robot)(MSG '+sent2+')(FROM '+source+')(TO '+goal+'))')
                    #print(prim+'((ACTOR Robot)(MSG '+sent2+')(FROM '+source+')(TO '+goal+'))')
                
                #============================FTRANS==FOLLOW=================================
                elif prim == "FTRANS":
                    #print(text_list_sen)
                    #print(pron_list_sen)
                    loc2 = "nil"
                    for ex in pron_list_sen:
                                    chop_noun3 = ex.split()
                                    len_noun4 = len(chop_noun3)
                                    f_noun4 = chop_noun3[-1]
                                    idx5 = text_list_sen.index(f_noun4)
                                    idx6 = idx5-len_noun4
                                    #print(idx6)
                                    if text_list_sen[idx6] == "to":
                                        loc2 = ex
                                        pron_list_sen.remove(ex)

                    loc1 = "nil"
                    for ex in pron_list_sen:
                                    chop_noun3 = ex.split()
                                    len_noun4 = len(chop_noun3)
                                    f_noun4 = chop_noun3[-1]
                                    idx5 = text_list_sen.index(f_noun4)
                                    idx6 = idx5-len_noun4
                                    #print(idx6)
                                    if text_list_sen[idx6] == "from":
                                        loc1 = ex
                                        pron_list_sen.remove(ex)
                    #print(loc1, loc2)

                    if len(pron_list_sen) != 0 and pron_list_sen[-1] not in ["an", "a", "the"]:
                        obj = pron_list_sen[-1]
                        dependencies_list.append(prim+'((ACTOR Robot)(OBJ '+ obj+')(FROM '+loc1+')(TO '+loc2+'))')
                        print(prim+'((ACTOR Robot)(OBJ '+ obj+')(FROM '+loc1+')(TO '+loc2+'))')
                    else:
                        dependencies_list.append(prim+'((ACTOR Robot)(OBJ nil)(FROM '+loc1+')(TO '+loc2+'))')
                        print(prim+'((ACTOR Robot)(OBJ nil)(FROM '+loc1+')(TO '+loc2+'))')
            
            print("====================================================================")
        
        for i in range(len(dependencies_list)):
            #FORMAT
            dependencies_list[i] = dependencies_list[i].lower().replace('the ', '')
            dependencies_list[i] = dependencies_list[i].lower().replace('an ', '')
            dependencies_list[i] = dependencies_list[i].lower().replace('a ', '')
    except:
        dependencies_list = []
        rospy.logerr(traceback.print_exc())    
    #print(dependencies_list)
    
    return dependencies_list
