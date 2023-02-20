# Import the package
import numpy as np
import spacy

import os

CD_structures = { "go" : "PTRANS",
"navigate" : "PTRANS",
"walk" : "PTRANS",
"lead" : "PTRANS",
"guide" : "PTRANS",
#to somewhere
"bring" : "ATRANS",
"give" : "ATRANS",
"deliver" : "ATRANS",
#the object #to a person
"take" : "GRAB",
"grasp" : "GRAB",
"find" : "ATTEND",
"look" : "ATTEND",
#someone/at something
"deposit" : "RELEASE",
#something, somewhere
"open" : "PROPEL",
"close" : "PROPEL",
#something
    }

def CondepParser(txt):
    """print("=====================================================================")
    print("|                       ORIGINAL COMMAND                            |")
    print("=====================================================================")
    print (text)"""
    
    print("=====================================================================")
    print("|                          SPACY PARSER                              |")
    print("=====================================================================")
    
    dependencies_list = []
    
    verb_list = []
    prim_list = []
    pron_list = []
    pos_list = []
    obj = []
    loc = []
    
    #Remove the word "Robot" in order to obtain a clean separation of the types of "obj"
    txt = txt.replace('Robot,', '')
    
    #Extracting tokens from the sentence
    nlp = None
    try:
        nlp = spacy.load("en_core_web_sm")
    except:
        os.system("python -m spacy download en_core_web_sm")
        nlp = spacy.load("en_core_web_sm")
    
    doc = nlp(txt)
    
    #Creating the list of nouns and positions
    for token in doc:
        text_list = [token.text for token in doc]
        pos_list1 = [token.pos_ for token in doc]
    print(text_list, pos_list1)
    
    pron_list = [chunk.text for chunk in doc.noun_chunks]
    pos_list = [[t.pos_ for t in chunk]for chunk in doc.noun_chunks]
    print(pron_list, pos_list)
    
    verb_list = [token.lemma_ for token in doc if token.pos_ == "VERB"]
    print(verb_list)
    
#=======================================================================
#If it doesn't exist verb or nouns then send a request for a new sentece
    if len(verb_list) == 0: #or len(pron_list) == 0:
        print("I can't understand you, give me a valid command.")
    else:
    	print("FINDING THE SENTENCES: ")
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
    	#print(txt)

    	"""if len(pers_list) == 1:
            txt = txt.replace(pron_list[acus_list[-1]], pron_list[pers_list[-1]])
            #note: person_list < acus_list ?
    	elif len(pers_list) == len(acus_list):
            for t in range(len(pers_list)):
                #print(pron_list[pers_list[t]])
                #print(pron_list[acus_list[t]])
                txt = txt.replace(pron_list[acus_list[t]], pron_list[pers_list[t]])
    	#print(txt)"""

        #Finding the sentences
    	for pos in range(len(pos_list1)):
    		if pos_list1[pos] == "VERB" and text_list[pos-1] == "and":
    			txt = txt.replace(" and", ".")
    			#print(txt)

    	txt = txt.replace(",", ".")
    	#print(txt)
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
            #print("INITIAL LIST: ")
            #print(text_list_sen)
            #print(pos_list_sen)
            pron_list_sen = [chunk.text for chunk in doc1.noun_chunks]
            pos_list_sen = [[t.pos_ for t in chunk]for chunk in doc1.noun_chunks]
            #print("PRONOUN LIST: ")
            #print(pron_list_sen)
            #print("POSITION LIST: ")
            #print(pos_list_sen)

            verb_list_sen = [token.lemma_ for token in doc1 if token.pos_ == "VERB"]
            #print("VERB LIST: ")
            #print(verb_list_sen)
            prim = CD_structures[verb_list_sen[0]]
            check_prim = len(pron_list_sen)
            print("=====================================================================")
            print("                    CONCEPTUAL DEPENDENCIES                          ")
            print("====================================================================")

            if prim == 'PTRANS':
                if check_prim != 0 and pron_list_sen[-1] != "the":
                    l = len(pos_list_sen[-1])
                    sn = pron_list_sen[-1].split()
                    sn = sn[-1]

                    index = text_list_sen.index(sn)
		    		#print(text_list_sen[index])
                    if text_list_sen[index-l] == "to":
                        location = pron_list_sen[-1]
                        dependencies_list.append(prim+'((ACTOR Robot)(OBJ Robot)(FROM robot place)(TO '+location+'))')
                        #print(prim+'((ACTOR Robot)(OBJ Robot)(FROM robot place)(TO '+location+'))')
                else:
                    #QTRANS (obj ?obj)(question where))
                    #print('QTRANS((OBJ nil)(question where))')
                    
                    dependencies_list.append(prim+'((ACTOR Robot)(OBJ Robot)(FROM robot place)(TO nil))')
                    #print(prim+'((ACTOR Robot)(OBJ Robot)(FROM robot place)(TO nil))')
    	    	#ATRANS ------> a / an [object], [person]
            
            elif prim == 'ATRANS':
                if len(pron_list_sen) != 0:
                    person='nil'
                    for pron in range(len(pron_list_sen)):
                        if pos_list_sen[pron][-1] == 'PROPN':
                            person=pron_list_sen[pron]
                            #print(person)
                            pron_list_sen = np.delete(pron_list_sen, pron)
                            break
                            #print(person)
                    #print(person)
                    if person != 'nil': #There is a person
                        if len(pron_list_sen) != 0 and pron_list_sen[-1] not in ["an", "a", "the"]: #There is a person and a valid object
                            obj = pron_list_sen[-1]
                            dependencies_list.append(prim+'((ACTOR Robot)(OBJ '+obj+')(FROM '+obj+' place)(TO '+person+'))')
                            #print(prim+'((ACTOR Robot)(OBJ '+obj+')(FROM '+obj+' place)(TO '+person+'))') 
                        else: #There is a person but no an object
                            #QTRANS (obj ?obj)(question what))
                            #print('QTRANS((OBJ nil)(question what))')
                            
                            dependencies_list.append(prim+'((ACTOR Robot)(OBJ nil)(FROM nil)(TO '+person+'))')
                            #print(prim+'((ACTOR Robot)(OBJ nil)(FROM nil)(TO '+person+'))')
                    else:
                        if len(pron_list_sen) != 0 and pron_list_sen[-1] in ["an", "a", "the"]: #There is no person and the obj in an/a/the
                            #QTRANS (obj ?obj)(question what))
                            #print('QTRANS((OBJ nil)(question what))')
                            
                            #QTRANS (obj ?obj)(question who))
                            #print('QTRANS((OBJ nil)(question who))')
                            
                            dependencies_list.append(prim+'((ACTOR Robot)(OBJ nil)(FROM nil)(TO nil))')
                            #print(prim+'((ACTOR Robot)(OBJ nil)(FROM nil)(TO nil))')
                        elif len(pron_list_sen) !=0  and pron_list_sen[-1] not in ["an", "a", "the"]:
                            #QTRANS (obj ?obj)(question who))
                            #print('QTRANS((OBJ nil)(question who))')
                            
                            obj = pron_list_sen[-1]
                            dependencies_list.append(prim+'((ACTOR Robot)(OBJ '+obj+')(FROM '+obj+' place)(TO nil))')
                            #print(prim+'((ACTOR Robot)(OBJ '+obj+')(FROM '+obj+' place)(TO nil))')
                else:
                        #QTRANS (obj ?obj)(question what))
                        #print('QTRANS((OBJ nil)(question what))')
                        
                        #QTRANS (obj ?obj)(question who))
                        #print('QTRANS((OBJ nil)(question who))')
                        
                        dependencies_list.append(prim+'((ACTOR Robot)(OBJ nil)(FROM nil)(TO nil))') #There is no person and no obj
                        #print(prim+'((ACTOR Robot)(OBJ nil)(FROM nil)(TO nil))')
            
            elif prim == "GRAB":
                #GRAB -----> [an object]
                      if len(pron_list_sen) == 0:
                          #QTRANS (obj ?obj)(question what))
                          #print('QTRANS((OBJ nil)(question what))')
                          
                          dependencies_list.append(prim+'((ACTOR Robot)(OBJ nil))')
                          #print(prim+'((ACTOR Robot)(OBJ nil))')
                      else:
                          if pron_list_sen[-1] in ["a", "an", "the"]:
                              #QTRANS (obj ?obj)(question what))
                              #print('QTRANS((OBJ nil)(question what))')
                              
                              dependencies_list.append(prim+'(ACTOR Robot)(OBJ nil))')
                              #print(prim+'((ACTOR Robot)(OBJ nil))')
                          else:
                              obj = pron_list_sen[-1]
                              dependencies_list.append(prim+'((ACTOR Robot)(OBJ '+obj+'))')
                              #print(prim+'((ACTOR Robot)(OBJ '+obj+'))')
            
            elif prim == "ATTEND":
                       #ATTEND -----> [an object] / [a person]
                       if len(pron_list_sen) == 0:
                           #QTRANS (obj ?obj)(question what))
                           #print('QTRANS((OBJ nil)(question what))')
                           
                           dependencies_list.append(prim+'((ACTOR Robot)(OBJ nil))')
                           #print(prim+'((ACTOR Robot)(OBJ nil))')
                       else:
                           if pron_list_sen[-1] in ["a", "an", "the"]:
                               #QTRANS (obj ?obj)(question what))
                               #print('QTRANS((OBJ nil)(question what))')
                               
                               dependencies_list.append(prim+'(ACTOR Robot)(OBJ nil))')
                               #print(prim+'((ACTOR Robot)(OBJ nil))')
                           else:
                               obj = pron_list_sen[-1]
                               dependencies_list.append(prim+'((ACTOR Robot)(OBJ '+obj+'))')
                               #print(prim+'((ACTOR Robot)(OBJ '+obj+'))')
            
            elif prim == "RELEASE":
                       #RELEASE -----> [an object]
                       if len(pron_list_sen) == 0:
                           #QTRANS (obj ?obj)(question what))
                           #print('QTRANS((OBJ nil)(question what))')
                           
                           dependencies_list.append(prim+'((ACTOR Robot)(OBJ nil))')
                           #print(prim+'((ACTOR Robot)(OBJ nil))')
                       else:
                           if pron_list_sen[-1] in ["a", "an", "the"]:
                               #QTRANS (obj ?obj)(question what))
                               #print('QTRANS((OBJ nil)(question what))')
                               
                               dependencies_list.append(prim+'((ACTOR Robot)(OBJ nil))')
                               #print(prim+'((ACTOR Robot)(OBJ nil))')
                           else:
                               obj = pron_list_sen[-1]
                               dependencies_list.append(prim+'((ACTOR Robot)(OBJ '+obj+'))')
                               #print(prim+'((ACTOR Robot)(OBJ '+obj+'))')
            
            elif prim == "PROPEL":
                       #PROPEL -----> [an object]
                       if len(pron_list_sen) == 0:
                           #QTRANS (obj ?obj)(question what))
                           #print('QTRANS((OBJ nil)(question what))')
                           
                           dependencies_list.append(prim+'((ACTOR Robot)(OBJ nil))')
                           #print(prim+'((ACTOR Robot)(OBJ nil))')
                       else:
                           if pron_list_sen[-1] in ["a", "an", "the"]:
                               #QTRANS (obj ?obj)(question what))
                               #print('QTRANS((OBJ nil)(question what))')
                               
                               dependencies_list.append(prim+'((ACTOR Robot)(OBJ nil))')
                               #print(prim+'((ACTOR Robot)(OBJ nil))')
                           else:
                               obj = pron_list_sen[-1]
                               dependencies_list.append(prim+'((ACTOR Robot)(OBJ '+obj+'))')
                               #print(prim+'((ACTOR Robot)(OBJ '+obj+'))')
            print("====================================================================")
    
    for i in range(len(dependencies_list)):
        mySeparator = " "

        #FORMAT
        dependencies_list[i] = dependencies_list[i].lower().replace('the ', '')
        dependencies_list[i] = dependencies_list[i].lower().replace('an ', '')
        dependencies_list[i] = dependencies_list[i].lower().replace('a ', '')
    
    #print(dependencies_list)
    
    return dependencies_list
