;****************************************
;*                                      *
;*  gpsr_rules.clp                      *
;*                                      *
;*          University of Mexico        *
;*          Jesus Savage-Carmona        *
;*          November 2022               *
;*                                      *
;*          Tamagawa University         *
;*          Luis Contreras              *
;*          March 2023                  *
;*                                      *
;****************************************

(defglobal ?*plan_number* = 0)
(defglobal ?*plan_number_new* = 0)

;****************************************
;*                                      *
;*               Rules                  *
;*                                      *
;****************************************

;#######################################
;      Global Functions
(deffunction oneof (?v $?values)
    (if (member$ ?v ?values)
        then ?v
        else (not ?v)
    )
)

;#######################################
;      Global Rules


(defrule update-ownership-object
    (item (type Objects) (name ?obj)(possession ?human))
    ?f1 <- (item (type Human) (name ?human)(objs ?obj))
    ?f2 <- (item (type Human) (name ?human1&:(neq ?human1 ?human))(objs ?obj))
    =>
    (modify ?f2 (objs nil))
)


(defrule delete-num-sentences-ROS
    (declare (salience -200))
    ?f <- (num-sentences ?n)
    =>
    (retract ?f)
)


;#######################################
;      Question rules

;;;{"where is", "who has"}
;;;(QTRANS((OBJ ?obj)(QUESTION ?word))) #where  #who
(defrule qtrans-where-ROS
    ?f  <- (num-sentences 1)
    ?f1 <- (qtrans (obj ?target)(question where))
    (item (type ?type)(name ?target)(room ?room-target)(zone ?zone-target))
    =>
    ; it answer where is the target
    (retract ?f ?f1)
    (bind ?*plan_number* (+ 1 ?*plan_number*))
    (bind ?*plan_number_new* (+ 1 ?*plan_number_new*))
    ;(assert (attempt (name answer)(id ?*plan_number* )(question where)(zone ?zone-target)(number 1 )))
    
    (switch ?type
        (case Objects
            then
            (printout t "(plan (name state)(id " ?*plan_number* ")(number 1)(actions answer where object " ?target " " ?room-target " " ?zone-target "))" crlf)
        )
        (case Human
            then
            (printout t "(plan (name state)(id " ?*plan_number* ")(number 1)(actions answer where human " ?target " " ?room-target " " ?zone-target "))" crlf)
        )
    )
)


   (switch ?student
      (case (oneof ?student stu1 stu2 stu3)
         then B-)
      (case stu4
         then A+)))


(defrule qtrans-who-object-ROS
    ?f  <- (num-sentences 1)
    ?f1 <- (qtrans (obj ?obj)(question who))
    (item (type Objects)(name ?obj)(possession ?human)(zone ?zone-object))
    =>
    ; it answer who has the object
    (retract ?f ?f1)
    (bind ?*plan_number* (+ 1 ?*plan_number*))
    (bind ?*plan_number_new* (+ 1 ?*plan_number_new*))
    ;(assert (attempt (name answer)(id ?*plan_number* )(question who)(zone ?zone-object)(number 1 )))
    
    (printout t "(plan (name state)(id " ?*plan_number* ")(number 1)(actions answer who " ?obj " " ?human " " ?zone-object "))" crlf)
)




;#######################################
;      General purpose rules


;;;;;;;;;;;;
; Single conceptual dependencies

;;;{"go", "navigate", "walk", "lead", "guide"} #to somewhere
;;;(PTRANS((ACTOR Robot)(OBJ Robot)(TO ?location)))
(defrule exec-ptrans-target-ROS
    ?f   <- (num-sentences 1)
    ?f1  <- (ptrans (actor ?actor)(obj ?actor)(to ?target))
    (item (type Robot)(name ?actor))
    
    (or
        (room (type ?type)(name ?target)(room ?room-target)(zone ?zone-target))
        (item (type ?type)(name ?target)(room ?room-target&:(neq ?room-target nil))(zone ?zone-target&:(neq ?zone-target nil)))
    )
    =>
    (retract ?f ?f1)
    ; it sends the robot to ?target
    (bind ?*plan_number* (+ 1 ?*plan_number*))
    (bind ?*plan_number_new* (+ 1 ?*plan_number_new*))
    ;(assert (attempt (name ptrans)(id ?*plan_number*)(move robot)(room ?room-target)(zone ?zone-target)(on room)(number 0 )))
    
    (printout t "(plan (name ptrans)(id " ?*plan_number* ")(number 1)(actions goto " ?room-target " " ?zone-target "))" crlf)
)


;;;{"find", "look"} #someone/at something
;;;(ATTEND((ACTOR Robot)(OBJ ?obj)))
(defrule exec-attend-target-ROS
    ?f   <- (num-sentences 1)
    ?f1  <- (attend (actor ?actor)(obj ?target))
    (item (type Robot)(name ?actor))
    (item (type ?type)(name ?target)(room ?room-target&:(neq ?room-target nil))(zone ?zone-target&:(neq ?zone-target nil))(upper nothing))
    =>
    ; it sends the robot to the ?target location and finds the ?target
    (retract ?f ?f1)
    (bind ?*plan_number* (+ 1 ?*plan_number*))
    (bind ?*plan_number_new* (+ 1 ?*plan_number_new*))
    ;(assert (attempt (name attend)(id ?*plan_number* )(move ?actor)(room ?room-target)(zone ?zone-target)(to ?target)(number 0 )))
    
    (switch ?type
        (case Objects
            then
            (printout t "(plan (name atrans)(id " ?*plan_number* ")(number 1)(actions goto " ?room-target " " ?zone-target"))" crlf)
            (printout t "(plan (name atrans)(id " ?*plan_number* ")(number 2)(actions find-object " ?target "))" crlf)
        )
        (case Human
            then
            (printout t "(plan (name ptrans)(id " ?*plan_number* ")(number 1)(actions goto " ?room-target " " ?zone-target "))" crlf)
            (printout t "(plan (name ptrans)(id " ?*plan_number* ")(number 2)(actions find-human " ?target "))" crlf)
        )
    )
)


;;;{"bring", "give", deliver"} #the object #to a person
;;;(ATRANS((ACTOR Robot)(OBJ ?obj)(TO ?person)))
(defrule exec-atrans-target-ROS
    ?f  <- (num-sentences 1)
    ?f1 <- (atrans (actor ?actor)(obj ?obj)(to ?target))
    (item (type Robot)(name ?actor))
    (item (type Objects)(name ?obj)(room ?room-object&:(neq ?room-object nil))(zone ?zone-object&:(neq ?zone-object nil))(upper nothing))
    
    (or
        (room (type ?type)(name ?target)(room ?room-target)(zone ?zone-target))
        (item (type ?type)(name ?target)(room ?room-target&:(neq ?room-target nil))(zone ?zone-target&:(neq ?zone-target nil)))
    )
    =>
    ; it deliver ?obj to ?target
    (retract ?f ?f1)
    (bind ?*plan_number* (+ 1 ?*plan_number*))
    (bind ?*plan_number_new* (+ 1 ?*plan_number_new*))
    ;(assert (attempt (name atrans)(id ?*plan_number* )(move ?obj)(room ?room-target)(zone ?zone-target)(to ?target)(number 0 )))
    
    (printout t "(plan (name atrans)(id " ?*plan_number* ")(number 1)(actions goto " ?room-object " " ?zone-object"))" crlf)
    (printout t "(plan (name atrans)(id " ?*plan_number* ")(number 2)(actions find-object " ?obj "))" crlf)
    (printout t "(plan (name atrans)(id " ?*plan_number* ")(number 3)(actions goto " ?room-target " " ?zone-target"))" crlf)
    
    (switch ?type
        (case Room
            then
            (printout t "(plan (name ptrans)(id " ?*plan_number* ")(number 4)(actions find-space))" crlf)
        )
        (case Human
            then
            (printout t "(plan (name atrans)(id " ?*plan_number* ")(number 4)(actions deliver-object " ?target "))" crlf)
        )
    )
)


;;;{"take", "grasp"} 
;;;(GRAB((ACTOR Robot)(OBJ ?obj)))
(defrule exec-grab-target-ROS
    ?f   <- (num-sentences 1)
    ?f1  <- (grab (actor ?actor)(obj ?obj))
    (not (attend))
    (item (type Robot)(name ?actor))
    (item (type Objects)(name ?obj)(room ?room-obj&:(neq ?room-obj nil))(zone ?zone-obj&:(neq ?zone-obj nil))(upper nothing))
    =>
    ; it sends the robot to the ?target location and finds the ?target
    (retract ?f ?f1)
    (bind ?*plan_number* (+ 1 ?*plan_number*))
    (bind ?*plan_number_new* (+ 1 ?*plan_number_new*))
    ;(assert (attempt (name grab)(id ?*plan_number* )(move ?actor)(room ?room-obj)(zone ?zone-obj)(to ?obj)(number 0 )))
    
    (printout t "(plan (name atrans)(id " ?*plan_number* ")(number 1)(actions goto " ?room-obj " " ?zone-obj"))" crlf)
    (printout t "(plan (name atrans)(id " ?*plan_number* ")(number 2)(actions find-object " ?obj "))" crlf)
)


;;;{"deposit"} #something, somewhere
;;;(RELEASE((ACTOR Robot)(OBJ ?obj)(TO ?place)))
(defrule exec-release-target-ROS
    ?f  <- (num-sentences 1)
    ?f1 <- (release (actor ?actor)(obj ?obj)(to ?place))
    (not (atrans))
    (item (type Robot)(name ?actor))
    (item (type Objects)(name ?obj)(room ?room-object&:(neq ?room-object nil))(zone ?zone-object&:(neq ?zone-object nil))(upper nothing))
    (room (type Room)(name ?place)(room ?room-place)(zone ?zone-place))
    =>
    ; it deliver ?obj to ?place
    (retract ?f ?f1)
    (bind ?*plan_number* (+ 1 ?*plan_number*))
    (bind ?*plan_number_new* (+ 1 ?*plan_number_new*))
    ;(assert (attempt (name release)(id ?*plan_number* )(move ?obj)(room ?room-target)(zone ?zone-target)(to ?target)(number 0 )))
    
    (printout t "(plan (name atrans)(id " ?*plan_number* ")(number 1)(actions goto " ?room-object " " ?zone-object"))" crlf)
    (printout t "(plan (name atrans)(id " ?*plan_number* ")(number 2)(actions find-object " ?obj "))" crlf)
    (printout t "(plan (name atrans)(id " ?*plan_number* ")(number 3)(actions goto " ?room-place " " ?zone-place"))" crlf)
    (printout t "(plan (name ptrans)(id " ?*plan_number* ")(number 4)(actions find-space))" crlf)
)


;;;{"tell", "say"}
;;;(SPEAK((MSG ?text)(TO ?human))
(defrule exec-speak-message-ROS
    ?f  <- (num-sentences 1)
    ?f1 <- (speak (msg $?text)(to nil))
    =>
    ; it says a ?text
    (retract ?f ?f1)
    (bind ?*plan_number* (+ 1 ?*plan_number*))
    (bind ?*plan_number_new* (+ 1 ?*plan_number_new*))
    ;(assert (attempt (name speak)(id ?*plan_number* )(say text)(number 0 )))
    
    (printout t "(plan (name speak)(id " ?*plan_number* ")(number 1)(actions say-string " ?text "))" crlf)
)

(defrule exec-speak-message-to-ROS
    ?f  <- (num-sentences 1)
    ?f1 <- (speak (msg $?text)(to ?human))
    (item (type Human)(name ?human)(room ?room-human&:(neq ?room-human nil))(zone ?zone-human&:(neq ?zone-human nil)))
    =>
    ; it says a ?text to ?human
    (retract ?f ?f1)
    (bind ?*plan_number* (+ 1 ?*plan_number*))
    (bind ?*plan_number_new* (+ 1 ?*plan_number_new*))
    ;(assert (attempt (name speak)(id ?*plan_number* )(say ?text)(room ?room-human)(zone ?zone-human)(to ?human)(number 0 )))
    
    (printout t "(plan (name atrans)(id " ?*plan_number* ")(number 1)(actions goto " ?room-human " " ?zone-human"))" crlf)
    (printout t "(plan (name ptrans)(id " ?*plan_number* ")(number 2)(actions find-human " ?human "))" crlf)
    (printout t "(plan (name speak)(id " ?*plan_number* ")(number 3)(actions say-string " ?text "))" crlf)
)


;;;{"remind"}
;;;(MTRANS((ACTOR Robot)(MSG ?msg)(from ?human)(to ?robot))) #remind (from ?human)(to ?robot)
;;;(MTRANS((ACTOR Robot)(MSG ?msg)(from ?robot)(to ?human))) #remind me/somebody (from ?robot)(to ?human)


;;;{"open", "close"} #something
;;;(PROPEL((ACTOR Robot)(OBJ ?obj)))



;;;;;;;;;;;;
; Consecutive conceptual dependencies

; Go to the studio, find mother, and give her a book
; (ptrans (actor robot)(obj robot)(to studio))
; (attend (actor robot)(obj mother)(from studio))
; (atrans (actor robot)(obj book)(to mother))
(defrule exec-ptrans-attend-atrans-ROS
    ;(declare (salience 200))
    ?f  <- (num-sentences 3)
    
    ?f1 <- (ptrans (actor ?actor)(obj ?actor)(to ?place))
    (item (type Robot) (name ?actor))
    (room (name ?place)(room ?room-place)(zone ?zone-place))
    
    ?f2 <- (attend (actor ?actor)(obj ?human))
    (item (type Human) (name ?human)(room ?room-human)(zone ?zone-human))
    
    ?f3 <- (atrans (actor ?actor)(obj ?obj)(to ?human))
    (item (type Objects)(name ?obj)(room ?room-object&:(neq ?room-object nil))(zone ?zone-object&:(neq ?zone-object nil))(upper nothing))
    =>
    (retract ?f ?f1 ?f2 ?f3)
    (bind ?*plan_number* (+ 1 ?*plan_number*))
    (bind ?*plan_number_new* (+ 1 ?*plan_number_new*))
    ;(assert (attempt (name ptrans-attend-atrans)(id ?*plan_number* )(move ?obj)(room ?room-human)(zone ?zone-human)(on ?human)(number 0 )))
    
    (printout t "(plan (name ptrans)(id " ?*plan_number* ")(number 1)(actions goto " ?room-object " " ?zone-object"))" crlf)
    (printout t "(plan (name ptrans)(id " ?*plan_number* ")(number 2)(actions find-object " ?obj "))" crlf)
    (printout t "(plan (name ptrans)(id " ?*plan_number* ")(number 3)(actions goto " ?room-place " " ?zone-place"))" crlf)
    (printout t "(plan (name ptrans)(id " ?*plan_number* ")(number 4)(actions find-human " ?human "))" crlf)
    (printout t "(plan (name ptrans)(id " ?*plan_number* ")(number 5)(actions deliver-object " ?human "))" crlf)
)




;;;;;;;;;;;;
; Incomplete conceptual dependencies

(defrule exec-atrans-no-recipient-ROS
    ?f <- (num-sentences 1)
    (atrans (actor ?actor)(to nil))
    (item (type Robot)(name ?actor))
    =>
    ; it asks who is the human that receives the object
    (retract ?f)
    (bind ?*plan_number* (+ 1 ?*plan_number*))
    (bind ?*plan_number_new* (+ 1 ?*plan_number_new*))
    ;(assert (attempt (name state)(id ?*plan_number* )(state recipient)(human unknown)(number 0 )))
    
    (printout t "(plan (name state)(id " ?*plan_number* ")(number 1)(actions ask human recipient))" crlf)
)

(defrule state-atrans-recipient-ROS
    (declare (salience 300))
    (num-sentences 1)
    ?f1 <- (atrans (actor ?actor)(to nil))
    ?f2 <- (state (attribute recipient)(value ?human))
    =>
    (retract ?f2)
    ; it modifies the atrans recipient
    (modify ?f1 (to ?human))
)


(defrule exec-atrans-no-object-location-ROS
    ?f  <- (num-sentences 1)
    ?f1 <- (atrans (actor ?actor)(obj ?obj)(to ?place))
    (item (type Robot) (name ?actor))
    (item (type Objects)(name ?obj)(zone nil))
    =>
    ; it asks where is the object
    (retract ?f)
    (bind ?*plan_number* (+ 1 ?*plan_number*))
    (bind ?*plan_number_new* (+ 1 ?*plan_number_new*))
    ;(assert (attempt (name state)(id ?*plan_number* ) (state ?obj)(room unknown)(zone any)(number 0 )))
    
    (printout t "(plan (name state)(id " ?*plan_number* ")(number 1)(actions ask object location " ?obj "))" crlf)
)

(defrule state-object-location-ROS
    (declare (salience 300))
    ?f  <- (num-sentences 1)
    ?f2 <-(state (attribute location)(obj ?object)(value ?place))
    ?f1 <- (item (type Objects)(name ?object))
    (room (name ?place)(room ?room-place)(zone ?zone-place))
    =>
    (retract ?f2)
    ; it modifies the object location
    (modify ?f1 (room ?room-place)(zone ?zone-place))
)
