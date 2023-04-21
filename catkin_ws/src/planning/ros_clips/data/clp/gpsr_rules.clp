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
(defglobal ?*mem_number* = 0)

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
;;;(QTRANS((OBJ ?obj)(QUESTION ?word))) #where
(defrule qtrans-where-ROS
    ?f  <- (num-sentences 1)
    ?f1 <- (qtrans (obj ?target)(question where))
    (item (type ?type)(name ?target)(room ?room-target)(zone ?zone-target))
    =>
    ; it answer where is the target
    (retract ?f ?f1)
    (bind ?*plan_number* (+ 1 ?*plan_number*))
    ;(assert (attempt (name answer)(id ?*plan_number* )(question where)(zone ?zone-target)(number 1 )))
    
    (switch ?type
        (case Objects
            then
            (printout t "(plan (name answer)(id " ?*plan_number* ")(number 1)(actions answer where object " ?target " " ?room-target " " ?zone-target "))" crlf)
        )
        (case Human
            then
            (printout t "(plan (name answer)(id " ?*plan_number* ")(number 1)(actions answer where human " ?target " " ?room-target " " ?zone-target "))" crlf)
        )
    )
)

(defrule qtrans-who-object-ROS
    ?f  <- (num-sentences 1)
    ?f1 <- (qtrans (obj ?obj)(question who))
    (item (type Objects)(name ?obj)(possession ?human)(zone ?zone-object))
    =>
    ; it answer who has the object
    (retract ?f ?f1)
    (bind ?*plan_number* (+ 1 ?*plan_number*))
    ;(assert (attempt (name answer)(id ?*plan_number* )(question who)(zone ?zone-object)(number 1 )))
    
    (printout t "(plan (name answer)(id " ?*plan_number* ")(number 1)(actions answer who " ?obj " " ?human " " ?zone-object "))" crlf)
)




;#######################################
;      General purpose rules

;;;;;;;;;;;;
; Single conceptual dependencies


;;;{"go", "navigate", "walk"} #to somewhere
;;;(PTRANS((ACTOR Robot)(OBJ Robot)(TO ?location)))
(defrule exec-ptrans-robot-target-ROS
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
    ;(assert (attempt (name ptrans)(id ?*plan_number*)(move robot)(room ?room-target)(zone ?zone-target)(on room)(number 0 )))
    
    (printout t "(plan (name ptrans)(id " ?*plan_number* ")(number 1)(actions goto " ?room-target " " ?zone-target "))" crlf)
)

;;;{"lead", "guide"} #to somewhere
;;;(PTRANS((ACTOR Robot)(OBJ ?human)(TO ?location)))
(defrule exec-ptrans-person-target-ROS
    ?f   <- (num-sentences 1)
    ?f1  <- (ptrans (actor ?actor)(obj ?human)(to ?target))
    (item (type Robot)(name ?actor))
    (item (type Human)(name ?human)(room ?room-human)(zone ?zone-human))
    
    (or
        (room (type ?type)(name ?target)(room ?room-target)(zone ?zone-target))
        (item (type ?type)(name ?target)(room ?room-target&:(neq ?room-target nil))(zone ?zone-target&:(neq ?zone-target nil)))
    )
    =>
    (retract ?f ?f1)
    ; guides human to ?target
    (bind ?*plan_number* (+ 1 ?*plan_number*))
    ;(assert (attempt (name ptrans)(id ?*plan_number*)(guide ?human)(room ?room-target)(zone ?zone-target)(on room)(number 0 )))
    
    (printout t "(plan (name ptrans)(id " ?*plan_number* ")(number 1)(actions find-human " ?human "))" crlf)
    (printout t "(plan (name ptrans)(id " ?*plan_number* ")(number 2)(actions guide-human " ?room-target " " ?zone-target "))" crlf)
)





;;;{"find", "look"} #someone/at something
;;;(ATTEND((ACTOR Robot)(OBJ ?obj)(AT nil)))
(defrule exec-attend-target-nil-ROS
    ?f   <- (num-sentences 1)
    ?f1  <- (attend (actor ?actor)(obj ?target)(at nil))
    (item (type Robot)(name ?actor))
    (item (type ?type)(name ?target)(room ?room-target)(zone ?zone-target)(upper nothing))
    =>
    ; it sends the robot to the ?target or ?place location and finds the ?target
    (retract ?f ?f1)
    (bind ?*plan_number* (+ 1 ?*plan_number*))
    ;(assert (attempt (name attend)(id ?*plan_number* )(move ?actor)(room ?room-target)(zone ?zone-target)(to ?target)(number 0 )))
    
    (switch ?type
        (case Objects
            then
			(if (neq ?room-target nil)
			   then
               (printout t "(plan (name attend)(id " ?*plan_number* ")(number 1)(actions goto " ?room-target " " ?zone-target"))" crlf)
               (printout t "(plan (name attend)(id " ?*plan_number* ")(number 2)(actions find-object " ?target "))" crlf)
			   else
               (printout t "(plan (name attend)(id " ?*plan_number* ")(number 1)(actions find-object " ?target "))" crlf)
			)
        )
        (case Human
            then
			(if (neq ?room-target nil)
			   then
               (printout t "(plan (name attend)(id " ?*plan_number* ")(number 1)(actions goto " ?room-target " " ?zone-target"))" crlf)
               (printout t "(plan (name attend)(id " ?*plan_number* ")(number 2)(actions find-human " ?target "))" crlf)
			   else
               (printout t "(plan (name attend)(id " ?*plan_number* ")(number 1)(actions find-human " ?target "))" crlf)
			)
        )
    )
)

;;;(ATTEND((ACTOR Robot)(OBJ ?obj)(AT ?place)))
(defrule exec-attend-target-place-ROS
    ?f   <- (num-sentences 1)
    ?f1  <- (attend (actor ?actor)(obj ?target)(at ?place))
    (item (type Robot)(name ?actor))
    (item (type ?type)(name ?target)(room ?room-target)(zone ?zone-target)(upper nothing))
	(room (type Room)(name ?place)(room ?room-place&:(neq room-place nil))(zone ?zone-place&:(neq ?zone-place nil)))
    =>
    ; it sends the robot to the ?target or ?place location and finds the ?target
    (retract ?f ?f1)
    (bind ?*plan_number* (+ 1 ?*plan_number*))
    ;(assert (attempt (name attend)(id ?*plan_number* )(move ?actor)(room ?room-place)(zone ?zone-place)(to ?target)(number 0 )))
    
    (switch ?type
        (case Objects
            then
            (printout t "(plan (name attend)(id " ?*plan_number* ")(number 1)(actions goto " ?room-place " " ?zone-place"))" crlf)
            (printout t "(plan (name attend)(id " ?*plan_number* ")(number 2)(actions find-object " ?target "))" crlf)
        )
        (case Human
            then
            (printout t "(plan (name attend)(id " ?*plan_number* ")(number 1)(actions goto " ?room-place " " ?zone-place"))" crlf)
            (printout t "(plan (name attend)(id " ?*plan_number* ")(number 2)(actions find-human " ?target "))" crlf)
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
    ;(assert (attempt (name atrans)(id ?*plan_number* )(move ?obj)(room ?room-target)(zone ?zone-target)(to ?target)(number 0 )))
    
    (printout t "(plan (name atrans)(id " ?*plan_number* ")(number 1)(actions goto " ?room-object " " ?zone-object"))" crlf)
    (printout t "(plan (name atrans)(id " ?*plan_number* ")(number 2)(actions find-object " ?obj "))" crlf)
    (printout t "(plan (name atrans)(id " ?*plan_number* ")(number 3)(actions goto " ?room-target " " ?zone-target"))" crlf)
    
    (switch ?type
        (case Room
            then
            (printout t "(plan (name atrans)(id " ?*plan_number* ")(number 4)(actions find-space " ?target "))" crlf)
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
    (item (type Robot)(name ?actor))
    (item (type Objects)(name ?obj)(room ?room-obj&:(neq ?room-obj nil))(zone ?zone-obj&:(neq ?zone-obj nil))(upper nothing))
    =>
    ; it sends the robot to the ?target location and finds the ?target
    (retract ?f ?f1)
    (bind ?*plan_number* (+ 1 ?*plan_number*))
    ;(assert (attempt (name grab)(id ?*plan_number* )(move ?actor)(room ?room-obj)(zone ?zone-obj)(to ?obj)(number 0 )))
    
    (printout t "(plan (name grab)(id " ?*plan_number* ")(number 1)(actions goto " ?room-obj " " ?zone-obj"))" crlf)
    (printout t "(plan (name grab)(id " ?*plan_number* ")(number 2)(actions find-object " ?obj "))" crlf)
)





;;;{"deposit"} #something, somewhere
;;;(RELEASE((ACTOR Robot)(OBJ ?obj)(TO ?place)))
(defrule exec-release-target-ROS
    ?f  <- (num-sentences 1)
    ?f1 <- (release (actor ?actor)(obj ?obj)(to ?place))
    (item (type Robot)(name ?actor))
    (item (type Objects)(name ?obj)(room ?room-object&:(neq ?room-object nil))(zone ?zone-object&:(neq ?zone-object nil))(upper nothing))
    (room (type Room)(name ?place)(room ?room-place)(zone ?zone-place))
    =>
    ; it deliver ?obj to ?place
    (retract ?f ?f1)
    (bind ?*plan_number* (+ 1 ?*plan_number*))
    ;(assert (attempt (name release)(id ?*plan_number* )(move ?obj)(room ?room-target)(zone ?zone-target)(to ?target)(number 0 )))
    
    (printout t "(plan (name release)(id " ?*plan_number* ")(number 1)(actions goto " ?room-object " " ?zone-object"))" crlf)
    (printout t "(plan (name release)(id " ?*plan_number* ")(number 2)(actions find-object " ?obj "))" crlf)
    (printout t "(plan (name release)(id " ?*plan_number* ")(number 3)(actions goto " ?room-place " " ?zone-place"))" crlf)
    (printout t "(plan (name release)(id " ?*plan_number* ")(number 4)(actions find-space " ?place "))" crlf)
)





;;;{"tell", "say"}
;;;(SPEAK((MSG ?text))
(defrule exec-speak-message-ROS
    ?f  <- (num-sentences 1)
    ?f1 <- (speak (msg $?text)(to nil))
    =>
    ; it says a ?text
    (retract ?f ?f1)
    (bind ?*plan_number* (+ 1 ?*plan_number*))
    ;(assert (attempt (name speak)(id ?*plan_number* )(say ?text)(number 0 )))
    
    (printout t "(plan (name speak)(id " ?*plan_number* ")(number 1)(actions say-string [" (implode$ ?text) "]))" crlf)
)

;;;(SPEAK((MSG ?text)(TO ?human))
(defrule exec-speak-message-to-ROS
    ?f  <- (num-sentences 1)
    ?f1 <- (speak (msg $?text)(to ?human))
    (item (type Human)(name ?human)(room ?room-human&:(neq ?room-human nil))(zone ?zone-human&:(neq ?zone-human nil)))
    =>
    ; it says a ?text to ?human
    (retract ?f ?f1)
    (bind ?*plan_number* (+ 1 ?*plan_number*))
    ;(assert (attempt (name speak)(id ?*plan_number* )(say ?text)(room ?room-human)(zone ?zone-human)(to ?human)(number 0 )))
    
    (printout t "(plan (name speak)(id " ?*plan_number* ")(number 1)(actions goto " ?room-human " " ?zone-human"))" crlf)
    (printout t "(plan (name speak)(id " ?*plan_number* ")(number 2)(actions find-human " ?human "))" crlf)
    (printout t "(plan (name speak)(id " ?*plan_number* ")(number 3)(actions say-string [" (implode$ ?text) "]))" crlf)
)





;;;{"remind"}
;;;(MTRANS((ACTOR Robot)(MSG ?msg)(to ?robot))) #remind (from ?human)(to ?robot)
(defrule exec-mtrans-message-ROS
    ?f  <- (num-sentences 1)
    ?f1 <- (mtrans (msg $?text)(from nil)(to ?robot))
    (item (type Robot)(name ?robot))
    =>
    ; it saves a ?text
    (retract ?f ?f1)
    (bind ?*plan_number* (+ 1 ?*plan_number*))
    (bind ?*mem_number* (+ 1 ?*mem_number*))
    ;(assert (attempt (name mtrans)(id ?*plan_number* )(remind ?text)(from nil)(to robot)(number 0 )))
    
    (assert (memory (msg ?text)(source unknown)(target robot)(num ?*mem_number* )))
    (printout t "(plan (name mtrans)(id " ?*plan_number* ")(number 1)(actions remind from nil to robot num " ?*mem_number* " [" (implode$ ?text) "]))" crlf)
)

;;;(MTRANS((ACTOR Robot)(MSG ?msg)(from ?human)(to ?robot))) #remind (from ?human)(to ?robot)
(defrule exec-mtrans-message-from-ROS
    ?f  <- (num-sentences 1)
    ?f1 <- (mtrans (msg $?text)(from ?human)(to ?robot))
    (item (type Robot)(name ?robot))
    (item (type Human)(name ?human)(room ?room-human&:(neq ?room-human nil))(zone ?zone-human&:(neq ?zone-human nil)))
    =>
    ; it saves a ?text from ?human
    (retract ?f ?f1)
    (bind ?*plan_number* (+ 1 ?*plan_number*))
    (bind ?*mem_number* (+ 1 ?*mem_number*))
    ;(assert (attempt (name mtrans)(id ?*plan_number* )(remind ?text)(from ?human)(to robot)(number 0 )))
    
    (assert (memory (msg ?text)(source ?human)(target robot)(num ?*mem_number* )))
    (printout t "(plan (name mtrans)(id " ?*plan_number* ")(number 1)(actions remind from " ?human " to robot num " ?*mem_number* " [" (implode$ ?text) "]))" crlf)
)

;;;(MTRANS((ACTOR Robot)(MSG ?msg)(from ?robot)(to ?human))) #remind me/somebody (from ?robot)(to ?human)
(defrule exec-mtrans-message-to-ROS
    ?f  <- (num-sentences 1)
    ?f1 <- (mtrans (msg $?text)(from ?robot)(to ?human))
    (item (type Robot)(name ?robot))
    (item (type Human)(name ?human)(room ?room-human&:(neq ?room-human nil))(zone ?zone-human&:(neq ?zone-human nil)))
    =>
    ; it says a saved ?text to ?human
    (retract ?f ?f1)
    (bind ?*plan_number* (+ 1 ?*plan_number*))
    (bind ?*mem_number* (+ 1 ?*mem_number*))
    ;(assert (attempt (name mtrans)(id ?*plan_number* )(remind ?text)(from ?human)(to robot)(number 0 )))
    
    (assert (memory (msg ?text)(source robot)(target ?human)(num ?*mem_number* )))
    (printout t "(plan (name mtrans)(id " ?*plan_number* ")(number 1)(actions remind from robot to " ?human " num " ?*mem_number* " [" (implode$ ?text) "]))" crlf)
)





;;;{"open", "close"} #something
;;;(PROPEL((ACTOR Robot)(OBJ ?target)(ACTION ?action)))
(defrule exec-propel-obj-ROS
    ?f  <- (num-sentences 1)
    ?f1 <- (propel (actor ?actor)(obj ?target)(action ?action))
    (item (type Robot)(name ?actor))
    (room (type Room)(name ?target)(room ?room-target&:(neq ?room-target nil))(zone ?zone-target&:(neq ?zone-target nil)))
    =>
    ; it propels ?action to ?target
    (retract ?f ?f1)
    (bind ?*plan_number* (+ 1 ?*plan_number*))
    (bind ?*mem_number* (+ 1 ?*mem_number*))
    ;(assert (attempt (name propel)(id ?*plan_number* )(propel ?obj)(room ?room-obj)(zone ?zone-obj)(action ?action)(number 0 )))
    
    (printout t "(plan (name propel)(id " ?*plan_number* ")(number 1)(actions goto " ?room-target " " ?zone-target"))" crlf)
    (printout t "(plan (name propel)(id " ?*plan_number* ")(number 2)(actions propel " ?action " " ?target "))" crlf)
)





;;;{follow} #someone from place to destination
;;;(FTRANS((ACTOR Robot)(obj ?human)(from nil)(to nil)))
(defrule exec-ftrans-person-ROS
    ?f  <- (num-sentences 1)
    ?f1 <- (ftrans (actor ?robot)(obj ?human)(from nil)(to nil))
    (item (type Robot)(name ?robot))
    (item (type Human)(name ?human)(room ?room-human)(zone ?zone-human))
    =>
    ; it follows ?human
    (retract ?f ?f1)
    (bind ?*plan_number* (+ 1 ?*plan_number*))
    ;(assert (attempt (name ftrans)(id ?*plan_number* )(follow ?human)))
    
	(if (and (neq ?room-human nil)
	         (neq ?zone-human nil))
	   then
       (printout t "(plan (name ftrans)(id " ?*plan_number* ")(number 1)(actions goto " ?room-human " " ?zone-human"))" crlf)
       (printout t "(plan (name ftrans)(id " ?*plan_number* ")(number 2)(actions find-human " ?human "))" crlf)
       (printout t "(plan (name ftrans)(id " ?*plan_number* ")(number 3)(actions follow " ?human "))" crlf)
	   else
       (printout t "(plan (name ftrans)(id " ?*plan_number* ")(number 1)(actions find-human " ?human "))" crlf)
       (printout t "(plan (name ftrans)(id " ?*plan_number* ")(number 2)(actions follow " ?human "))" crlf)
	)
)

;;;(FTRANS((ACTOR Robot)(obj ?human)(from nil)(to destination)))
(defrule exec-ftrans_to-person-ROS
    ?f  <- (num-sentences 1)
    ?f1 <- (ftrans (actor ?robot)(obj ?human)(from nil)(to ?destination))
    (item (type Robot)(name ?robot))
    (item (type Human)(name ?human)(room ?room-human)(zone ?zone-human))
    (room (type Room)(name ?destination)(room ?room-dst)(zone ?zone-dst))
    =>
    ; it follows ?human
    (retract ?f ?f1)
    (bind ?*plan_number* (+ 1 ?*plan_number*))
    ;(assert (attempt (name ftrans)(id ?*plan_number* )(follow ?human)(to ?destination)))
    
	(if (and (neq ?room-human nil)
	         (neq ?zone-human nil))
	   then
       (printout t "(plan (name ftrans)(id " ?*plan_number* ")(number 1)(actions goto " ?room-human " " ?zone-human"))" crlf)
       (printout t "(plan (name ftrans)(id " ?*plan_number* ")(number 2)(actions find-human " ?human "))" crlf)
       (printout t "(plan (name ftrans)(id " ?*plan_number* ")(number 3)(actions say-string [Please, take me to " ?destination ".]))" crlf)
       (printout t "(plan (name ftrans)(id " ?*plan_number* ")(number 4)(actions follow " ?human "))" crlf)
	   else
       (printout t "(plan (name ftrans)(id " ?*plan_number* ")(number 1)(actions find-human " ?human "))" crlf)
       (printout t "(plan (name ftrans)(id " ?*plan_number* ")(number 2)(actions say-string [Please, take me to " ?destination ".]))" crlf)
       (printout t "(plan (name ftrans)(id " ?*plan_number* ")(number 3)(actions follow " ?human "))" crlf)
	)
) 

;;;(FTRANS((ACTOR Robot)(obj ?human)(from ?place)(to nil)))
(defrule exec-ftrans_from-person-ROS
    ?f  <- (num-sentences 1)
    ?f1 <- (ftrans (actor ?robot)(obj ?human)(from ?place)(to nil))
    (item (type Robot)(name ?robot))
    (item (type Human)(name ?human)(room ?room-human)(zone ?zone-human))
    (room (type Room)(name ?place)(room ?room-place)(zone ?zone-place))
    =>
    ; it follows ?human
    (retract ?f ?f1)
    (bind ?*plan_number* (+ 1 ?*plan_number*))
    ;(assert (attempt (name ftrans)(id ?*plan_number* )(follow ?human)(from ?place)(to ?destination)))
    
    (printout t "(plan (name ftrans)(id " ?*plan_number* ")(number 1)(actions goto " ?room-place " " ?zone-place"))" crlf)
    (printout t "(plan (name ftrans)(id " ?*plan_number* ")(number 2)(actions find-human " ?human "))" crlf)
    (printout t "(plan (name ftrans)(id " ?*plan_number* ")(number 3)(actions follow " ?human "))" crlf)
)

;;;(FTRANS((ACTOR Robot)(obj ?human)(from ?place)(to ?destination)))
(defrule exec-ftrans_from_to-person-ROS
    ?f  <- (num-sentences 1)
    ?f1 <- (ftrans (actor ?robot)(obj ?human)(from ?place)(to ?destination))
    (item (type Robot)(name ?robot))
    (item (type Human)(name ?human)(room ?room-human)(zone ?zone-human))
    (room (type Room)(name ?place)(room ?room-place)(zone ?zone-place))
    (room (type Room)(name ?destination)(room ?room-dst)(zone ?zone-dst))
    =>
    ; it follows ?human
    (retract ?f ?f1)
    (bind ?*plan_number* (+ 1 ?*plan_number*))
    ;(assert (attempt (name ftrans)(id ?*plan_number* )(follow ?human)(from ?place)(to ?destination)))
    
    (printout t "(plan (name ftrans)(id " ?*plan_number* ")(number 1)(actions goto " ?room-place " " ?zone-place"))" crlf)
    (printout t "(plan (name ftrans)(id " ?*plan_number* ")(number 2)(actions find-human " ?human "))" crlf)
    (printout t "(plan (name ftrans)(id " ?*plan_number* ")(number 3)(actions say-string [Please, take me to " ?destination ".]))" crlf)
    (printout t "(plan (name ftrans)(id " ?*plan_number* ")(number 4)(actions follow " ?human "))" crlf)
)





;;;;;;;;;;;;
; Consecutive conceptual dependencies

;Go to the shelf and take the chips
; (PTRANS (ACTOR robot)(OBJ robot)(TO ?place))
; (GRAB (ACTOR Robot)(OBJ ?obj))
(defrule exec-ptrans-grab-ROS
    ?f   <- (num-sentences 2)
    
	?f1 <- (ptrans (actor ?actor)(obj ?actor)(to ?place))
    (item (type Robot) (name ?actor))
    (room (name ?place)(room ?room-place)(zone ?zone-place))
    
	?f2  <- (grab (actor ?actor)(obj ?obj))
    (item (type Objects)(name ?obj)(room ?room-obj)(zone ?zone-obj))
    =>
    ; it sends the robot to the ?target location and finds the ?target
    (retract ?f ?f1 ?f2)
    (bind ?*plan_number* (+ 1 ?*plan_number*))
    ;(assert (attempt (name ptrans-grab)(id ?*plan_number* )(move ?actor)(room ?room-place)(zone ?zone-place)(to ?obj)(number 0 )))
    
    (printout t "(plan (name ptrans-grab)(id " ?*plan_number* ")(number 1)(actions goto " ?room-place " " ?zone-place"))" crlf)
    (printout t "(plan (name ptrans-grab)(id " ?*plan_number* ")(number 2)(actions find-object " ?obj "))" crlf)
)





; Find Luis and give him some chips
; (ATTEND (ACTOR robot)(OBJ ?human)(AT ?place))
; (ATRANS (ACTOR robot)(OBJ ?obj)(TO ?human))
(defrule exec-attend-atrans-ROS
    ;(declare (salience 200))
    ?f  <- (num-sentences 2)
    
    ?f1 <- (attend (actor ?actor)(obj ?human)(at nil))
    (item (type Robot) (name ?actor))
    (item (type Human) (name ?human)(room ?room-human&:(neq ?room-human nil))(zone ?zone-human&:(neq ?zone-human nil)))
    
    ?f2 <- (atrans (actor ?actor)(obj ?obj)(to ?human))
    (item (type Objects)(name ?obj)(room ?room-object&:(neq ?room-object nil))(zone ?zone-object&:(neq ?zone-object nil))(upper nothing))
    =>
	;it gives ?obj to ?human 
    (retract ?f ?f1 ?f2)
    (bind ?*plan_number* (+ 1 ?*plan_number*))
    ;(assert (attempt (name attend-atrans)(id ?*plan_number* )(move ?obj)(room ?room-human)(zone ?zone-human)(on ?human)(number 0 )))
    
    (printout t "(plan (name attend-atrans)(id " ?*plan_number* ")(number 1)(actions goto " ?room-object " " ?zone-object"))" crlf)
    (printout t "(plan (name attend-atrans)(id " ?*plan_number* ")(number 2)(actions find-object " ?obj "))" crlf)
	(printout t "(plan (name attend-atrans)(id " ?*plan_number* ")(number 3)(actions goto " ?room-human " " ?zone-human"))" crlf)
    (printout t "(plan (name attend-atrans)(id " ?*plan_number* ")(number 4)(actions find-human " ?human "))" crlf)
    (printout t "(plan (name attend-atrans)(id " ?*plan_number* ")(number 5)(actions deliver-object " ?human "))" crlf)
)





; Find Luis at the studio and give him some chips
; (ATTEND (ACTOR robot)(OBJ ?human)(AT ?place))
; (ATRANS (ACTOR robot)(OBJ ?obj)(TO ?human))
(defrule exec-attend_place-atrans-ROS
    ;(declare (salience 200))
    ?f  <- (num-sentences 2)
    
    ?f1 <- (attend (actor ?actor)(obj ?human)(at ?place))
    (item (type Robot) (name ?actor))
    (item (type Human) (name ?human)(room ?room-human)(zone ?zone-human))
	(room (type Room)(name ?place)(room ?room-place)(zone ?zone-place))
    
    ?f2 <- (atrans (actor ?actor)(obj ?obj)(to ?human))
    (item (type Objects)(name ?obj)(room ?room-object&:(neq ?room-object nil))(zone ?zone-object&:(neq ?zone-object nil))(upper nothing))
    =>
	;it gives ?obj to ?human 
    (retract ?f ?f1 ?f2)
    (bind ?*plan_number* (+ 1 ?*plan_number*))
    ;(assert (attempt (name attend-atrans)(id ?*plan_number* )(move ?obj)(room ?room-human)(zone ?zone-human)(on ?human)(number 0 )))
    
    (printout t "(plan (name attend-atrans)(id " ?*plan_number* ")(number 1)(actions goto " ?room-object " " ?zone-object"))" crlf)
    (printout t "(plan (name attend-atrans)(id " ?*plan_number* ")(number 2)(actions find-object " ?obj "))" crlf)
	(printout t "(plan (name attend-atrans)(id " ?*plan_number* ")(number 3)(actions goto " ?room-place " " ?zone-place"))" crlf)
    (printout t "(plan (name attend-atrans)(id " ?*plan_number* ")(number 4)(actions find-human " ?human "))" crlf)
    (printout t "(plan (name attend-atrans)(id " ?*plan_number* ")(number 5)(actions deliver-object " ?human "))" crlf)
)





; Go to the studio and give Luis some chips
; (PTRANS (ACTOR robot)(OBJ robot)(TO ?place))
; (ATRANS (ACTOR robot)(OBJ ?obj)(TO ?human))
(defrule exec-ptrans-atrans-ROS
    ;(declare (salience 200))
    ?f  <- (num-sentences 2)
    
    ?f1 <- (ptrans (actor ?actor)(obj ?actor)(to ?place))
    (item (type Robot) (name ?actor))
    (room (name ?place)(room ?room-place)(zone ?zone-place))
    
    ?f2 <- (atrans (actor ?actor)(obj ?obj)(to ?human))
    (item (type Human) (name ?human)(room ?room-human)(zone ?zone-human))
    (item (type Objects)(name ?obj)(room ?room-object&:(neq ?room-object nil))(zone ?zone-object&:(neq ?zone-object nil))(upper nothing))
    =>
	;it gives ?obj to ?human 
    (retract ?f ?f1 ?f2)
    (bind ?*plan_number* (+ 1 ?*plan_number*))
    ;(assert (attempt (name ptrans-attend-atrans)(id ?*plan_number* )(move ?obj)(room ?room-human)(zone ?zone-human)(on ?human)(number 0 )))
    
    (printout t "(plan (name ptrans-attend-atrans)(id " ?*plan_number* ")(number 1)(actions goto " ?room-object " " ?zone-object"))" crlf)
    (printout t "(plan (name ptrans-attend-atrans)(id " ?*plan_number* ")(number 2)(actions find-object " ?obj "))" crlf)
    (printout t "(plan (name ptrans-attend-atrans)(id " ?*plan_number* ")(number 3)(actions goto " ?room-place " " ?zone-place"))" crlf)
    (printout t "(plan (name ptrans-attend-atrans)(id " ?*plan_number* ")(number 4)(actions find-human " ?human "))" crlf)
    (printout t "(plan (name ptrans-attend-atrans)(id " ?*plan_number* ")(number 5)(actions deliver-object " ?human "))" crlf)
)





; Go to the studio, find Luis, and give him some chips
; (PTRANS (ACTOR robot)(OBJ robot)(TO ?place))
; (ATTEND (ACTOR robot)(OBJ ?human))
; (ATRANS (ACTOR robot)(OBJ ?obj)(TO ?human))
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
	;it gives ?obj to ?human 
    (retract ?f ?f1 ?f2 ?f3)
    (bind ?*plan_number* (+ 1 ?*plan_number*))
    ;(assert (attempt (name ptrans-attend-atrans)(id ?*plan_number* )(move ?obj)(room ?room-human)(zone ?zone-human)(on ?human)(number 0 )))
    
    (printout t "(plan (name ptrans-attend-atrans)(id " ?*plan_number* ")(number 1)(actions goto " ?room-object " " ?zone-object"))" crlf)
    (printout t "(plan (name ptrans-attend-atrans)(id " ?*plan_number* ")(number 2)(actions find-object " ?obj "))" crlf)
    (printout t "(plan (name ptrans-attend-atrans)(id " ?*plan_number* ")(number 3)(actions goto " ?room-place " " ?zone-place"))" crlf)
    (printout t "(plan (name ptrans-attend-atrans)(id " ?*plan_number* ")(number 4)(actions find-human " ?human "))" crlf)
    (printout t "(plan (name ptrans-attend-atrans)(id " ?*plan_number* ")(number 5)(actions deliver-object " ?human "))" crlf)
)





; Find Luis and guide him to the office
; (ATTEND (ACTOR robot)(OBJ ?human))
; (PTRANS (ACTOR robot)(OBJ ?human)(TO ?destination))
(defrule exec-attend-ptrans-ROS
    ;(declare (salience 200))
    ?f  <- (num-sentences 2)
    
    ?f1 <- (attend (actor ?actor)(obj ?human)(at nil))
    (item (type Robot) (name ?actor))
    (item (type Human) (name ?human)(room ?room-human)(zone ?zone-human))
    
    ?f2 <- (ptrans (actor ?actor)(obj ?human)(to ?destination))
    (room (name ?destination)(room ?room-dst)(zone ?zone-dst))
    =>
	;it guides ?human to ?destination
    (retract ?f ?f1 ?f2)
    (bind ?*plan_number* (+ 1 ?*plan_number*))
    ;(assert (attempt (name attend-ptrans)(id ?*plan_number* )(move ?human)(room ?room-human)(zone ?zone-human)(to ?destination)(number 0 )))
    
	(if (neq ?room-human nil)
	   then
       (printout t "(plan (name attend-ptrans)(id " ?*plan_number* ")(number 1)(actions goto " ?room-human " " ?zone-human"))" crlf)
       (printout t "(plan (name attend-ptrans)(id " ?*plan_number* ")(number 2)(actions find-human " ?human "))" crlf)
       (printout t "(plan (name attend-ptrans)(id " ?*plan_number* ")(number 3)(actions guide-human " ?room-dst " " ?zone-dst "))" crlf)
	   else
       (printout t "(plan (name attend-ptrans)(id " ?*plan_number* ")(number 1)(actions find-human " ?human "))" crlf)
       (printout t "(plan (name attend-ptrans)(id " ?*plan_number* ")(number 2)(actions guide-human " ?room-dst " " ?zone-dst "))" crlf)
	)
)





; Find Luis at the studio and guide him to the office
; (ATTEND (ACTOR robot)(OBJ ?human))
; (PTRANS (ACTOR robot)(OBJ ?human)(TO ?destination))
(defrule exec-attend_place-ptrans-ROS
    ;(declare (salience 200))
    ?f  <- (num-sentences 2)
    
    ?f1 <- (attend (actor ?actor)(obj ?human)(at ?place))
    (item (type Robot) (name ?actor))
    (item (type Human) (name ?human)(room ?room-human)(zone ?zone-human))
	(room (type Room)(name ?place)(room ?room-place)(zone ?zone-place))
    
    ?f2 <- (ptrans (actor ?actor)(obj ?human)(to ?destination))
    (room (name ?destination)(room ?room-dst)(zone ?zone-dst))
    =>
	;it guides ?human to ?destination
    (retract ?f ?f1 ?f2)
    (bind ?*plan_number* (+ 1 ?*plan_number*))
    ;(assert (attempt (name attend-ptrans)(id ?*plan_number* )(move ?human)(room ?room-place)(zone ?zone-place)(to ?destination)(number 0 )))
    
    (printout t "(plan (name attend-ptrans)(id " ?*plan_number* ")(number 1)(actions goto " ?room-place " " ?zone-place"))" crlf)
    (printout t "(plan (name attend-ptrans)(id " ?*plan_number* ")(number 2)(actions find-human " ?human "))" crlf)
    (printout t "(plan (name attend-ptrans)(id " ?*plan_number* ")(number 3)(actions guide-human " ?room-dst " " ?zone-dst "))" crlf)
)





; Go to the studio and guide Luis to the office
; (PTRANS (ACTOR robot)(OBJ robot)(TO ?place))
; (PTRANS (ACTOR robot)(OBJ ?human)(TO ?destination))
(defrule exec-ptrans-ptrans-ROS
    ;(declare (salience 200))
    ?f  <- (num-sentences 2)
    
    ?f1 <- (ptrans (actor ?actor)(obj ?actor)(to ?place))
    (item (type Robot) (name ?actor))
    (room (name ?place)(room ?room-place)(zone ?zone-place))
    
    ?f2 <- (ptrans (actor ?actor)(obj ?human)(to ?destination))
    (item (type Human) (name ?human)(room ?room-human)(zone ?zone-human))
    (room (name ?destination)(room ?room-dst)(zone ?zone-dst))
    =>
	;it guides ?human to ?destination
    (retract ?f ?f1 ?f2)
    (bind ?*plan_number* (+ 1 ?*plan_number*))
    ;(assert (attempt (name ptrans-ptrans)(id ?*plan_number* )(move ?human)(room ?room-human)(zone ?zone-dst)(to ?destination)(number 0 )))
    
    (printout t "(plan (name ptrans-attend-atrans)(id " ?*plan_number* ")(number 1)(actions goto " ?room-place " " ?zone-place"))" crlf)
    (printout t "(plan (name ptrans-attend-atrans)(id " ?*plan_number* ")(number 2)(actions find-human " ?human "))" crlf)
    (printout t "(plan (name ptrans-attend-ptrans)(id " ?*plan_number* ")(number 3)(actions guide-human " ?room-dst " " ?zone-dst "))" crlf)
)





; Go to the studio, find Luis, and guide him to the office
; (PTRANS (ACTOR robot)(OBJ robot)(TO ?place))
; (ATTEND (ACTOR robot)(OBJ ?human))
; (PTRANS (ACTOR robot)(OBJ ?human)(TO ?destination))
(defrule exec-ptrans-attend-ptrans-ROS
    ;(declare (salience 200))
    ?f  <- (num-sentences 3)
    
    ?f1 <- (ptrans (actor ?actor)(obj ?actor)(to ?place))
    (item (type Robot) (name ?actor))
    (room (name ?place)(room ?room-place)(zone ?zone-place))
    
    ?f2 <- (attend (actor ?actor)(obj ?human))
    (item (type Human) (name ?human)(room ?room-human)(zone ?zone-human))
    
    ?f3 <- (ptrans (actor ?actor)(obj ?human)(to ?destination))
    (room (name ?destination)(room ?room-dst)(zone ?zone-dst))
    =>
	;it guides ?human to ?destination
    (retract ?f ?f1 ?f2 ?f3)
    (bind ?*plan_number* (+ 1 ?*plan_number*))
    ;(assert (attempt (name ptrans-attend-ptrans)(id ?*plan_number* )(move ?human)(room ?room-human)(zone ?zone-dst)(to ?destination)(number 0 )))
    
    (printout t "(plan (name ptrans-attend-atrans)(id " ?*plan_number* ")(number 1)(actions goto " ?room-place " " ?zone-place"))" crlf)
    (printout t "(plan (name ptrans-attend-atrans)(id " ?*plan_number* ")(number 2)(actions find-human " ?human "))" crlf)
    (printout t "(plan (name ptrans-attend-ptrans)(id " ?*plan_number* ")(number 3)(actions guide-human " ?room-dst " " ?zone-dst "))" crlf)
)





; Find Luis [at the studio] and follow him [from the studio] [to the office]
; (ATTEND (ACTOR robot)(OBJ ?human)(AT ?location))
; (FTRANS((ACTOR Robot)(obj ?human)(FROM ?place)(TO ?destination)))
(defrule exec-attend-ftrans-ROS
    ?f  <- (num-sentences 2)
    
    ?f1 <- (attend (actor ?robot)(obj ?human)(at nil))
    ?f2 <- (ftrans (actor ?robot)(obj ?human)(from nil)(to nil))
    
    (item (type Robot)(name ?robot))
    (item (type Human)(name ?human)(room ?room-human)(zone ?zone-human))
    =>
    ; it follows ?human
    (retract ?f ?f1 ?f2)
    (bind ?*plan_number* (+ 1 ?*plan_number*))
    ;(assert (attempt (name ftrans)(id ?*plan_number* )(follow ?human)))
    
	(if (and (neq ?room-human nil)
	         (neq ?zone-human nil))
	   then
       (printout t "(plan (name attend-ftrans)(id " ?*plan_number* ")(number 1)(actions goto " ?room-human " " ?zone-human"))" crlf)
       (printout t "(plan (name attend-ftrans)(id " ?*plan_number* ")(number 2)(actions find-human " ?human "))" crlf)
       (printout t "(plan (name attend-ftrans)(id " ?*plan_number* ")(number 3)(actions follow " ?human "))" crlf)
	   else
       (printout t "(plan (name attend-ftrans)(id " ?*plan_number* ")(number 1)(actions find-human " ?human "))" crlf)
       (printout t "(plan (name attend-ftrans)(id " ?*plan_number* ")(number 2)(actions follow " ?human "))" crlf)
	)
)

(defrule exec-attend_at-ftrans-ROS
    ?f  <- (num-sentences 2)
    
    ?f1 <- (attend (actor ?robot)(obj ?human)(at ?location))
    ?f2 <- (ftrans (actor ?robot)(obj ?human)(to nil))
    
    (item (type Robot)(name ?robot))
    (item (type Human)(name ?human)(room ?room-human)(zone ?zone-human))
	(room (type Room)(name ?location)(room ?room-loc&:(neq ?room-loc nil))(zone ?zone-loc&:(neq ?zone-loc nil)))
    =>
    ; it follows ?human
    (retract ?f ?f1 ?f2)
    (bind ?*plan_number* (+ 1 ?*plan_number*))
    ;(assert (attempt (name ftrans)(id ?*plan_number* )(follow ?human)))
    
    (printout t "(plan (name attend-ftrans)(id " ?*plan_number* ")(number 1)(actions goto " ?room-loc " " ?zone-loc"))" crlf)
    (printout t "(plan (name attend-ftrans)(id " ?*plan_number* ")(number 2)(actions find-human " ?human "))" crlf)
    (printout t "(plan (name attend-ftrans)(id " ?*plan_number* ")(number 3)(actions follow " ?human "))" crlf)
)

(defrule exec-attend-ftrans_from-ROS
    ?f  <- (num-sentences 2)
    
    ?f1 <- (attend (actor ?robot)(obj ?human))
    ?f2 <- (ftrans (actor ?robot)(obj ?human)(from ?place)(to nil))
    
    (item (type Robot)(name ?robot))
    (item (type Human)(name ?human)(room ?room-human)(zone ?zone-human))
	(room (type Room)(name ?place)(room ?room-place&:(neq ?room-place nil))(zone ?zone-place&:(neq ?zone-place nil)))
    =>
    ; it follows ?human
    (retract ?f ?f1 ?f2)
    (bind ?*plan_number* (+ 1 ?*plan_number*))
    ;(assert (attempt (name ftrans)(id ?*plan_number* )(follow ?human)))
    
    (printout t "(plan (name attend-ftrans)(id " ?*plan_number* ")(number 1)(actions goto " ?room-place " " ?zone-place"))" crlf)
    (printout t "(plan (name attend-ftrans)(id " ?*plan_number* ")(number 2)(actions find-human " ?human "))" crlf)
    (printout t "(plan (name attend-ftrans)(id " ?*plan_number* ")(number 3)(actions follow " ?human "))" crlf)
)

(defrule exec-attend-ftrans_to-ROS
    ?f  <- (num-sentences 2)
    
    ?f1 <- (attend (actor ?robot)(obj ?human)(at nil))
    ?f2 <- (ftrans (actor ?robot)(obj ?human)(from nil)(to ?destination))
    
    (item (type Robot)(name ?robot))
    (item (type Human)(name ?human)(room ?room-human)(zone ?zone-human))
    (room (type Room)(name ?destination)(room ?room-dst)(zone ?zone-dst))
    =>
    ; it follows ?human
    (retract ?f ?f1 ?f2)
    (bind ?*plan_number* (+ 1 ?*plan_number*))
    ;(assert (attempt (name ftrans)(id ?*plan_number* )(follow ?human)))
    
	(if (and (neq ?room-human nil)
	         (neq ?zone-human nil))
	   then
       (printout t "(plan (name attend-ftrans)(id " ?*plan_number* ")(number 1)(actions goto " ?room-human " " ?zone-human"))" crlf)
       (printout t "(plan (name attend-ftrans)(id " ?*plan_number* ")(number 2)(actions find-human " ?human "))" crlf)
       (printout t "(plan (name attend-ftrans)(id " ?*plan_number* ")(number 3)(actions say-string [Please, take me to " ?destination ".]))" crlf)
       (printout t "(plan (name attend-ftrans)(id " ?*plan_number* ")(number 3)(actions follow " ?human "))" crlf)
	   else
       (printout t "(plan (name attend-ftrans)(id " ?*plan_number* ")(number 1)(actions find-human " ?human "))" crlf)
       (printout t "(plan (name attend-ftrans)(id " ?*plan_number* ")(number 3)(actions say-string [Please, take me to " ?destination ".]))" crlf)
       (printout t "(plan (name attend-ftrans)(id " ?*plan_number* ")(number 2)(actions follow " ?human "))" crlf)
	)
)

(defrule exec-attend_at-ftrans_to-ROS
    ?f  <- (num-sentences 2)
    
    ?f1 <- (attend (actor ?robot)(obj ?human)(at ?location))
    ?f2 <- (ftrans (actor ?robot)(obj ?human)(to ?destination))
    
    (item (type Robot)(name ?robot))
    (item (type Human)(name ?human)(room ?room-human)(zone ?zone-human))
	(room (type Room)(name ?location)(room ?room-loc&:(neq ?room-loc nil))(zone ?zone-loc&:(neq ?zone-loc nil)))
    (room (type Room)(name ?destination)(room ?room-dst)(zone ?zone-dst))
    =>
    ; it follows ?human
    (retract ?f ?f1 ?f2)
    (bind ?*plan_number* (+ 1 ?*plan_number*))
    ;(assert (attempt (name ftrans)(id ?*plan_number* )(follow ?human)))
    
    (printout t "(plan (name attend-ftrans)(id " ?*plan_number* ")(number 1)(actions goto " ?room-loc " " ?zone-loc"))" crlf)
    (printout t "(plan (name attend-ftrans)(id " ?*plan_number* ")(number 2)(actions find-human " ?human "))" crlf)
    (printout t "(plan (name attens-ftrans)(id " ?*plan_number* ")(number 3)(actions say-string [Please, take me to " ?destination ".]))" crlf)
    (printout t "(plan (name attend-ftrans)(id " ?*plan_number* ")(number 3)(actions follow " ?human "))" crlf)
)

(defrule exec-attend-ftrans_from_to-ROS
    ?f  <- (num-sentences 2)
    
    ?f1 <- (attend (actor ?robot)(obj ?human))
    ?f2 <- (ftrans (actor ?robot)(obj ?human)(from ?place)(to ?destination))
    
    (item (type Robot)(name ?robot))
    (item (type Human)(name ?human)(room ?room-human)(zone ?zone-human))
	(room (type Room)(name ?place)(room ?room-place&:(neq ?room-place nil))(zone ?zone-place&:(neq ?zone-place nil)))
    (room (type Room)(name ?destination)(room ?room-dst)(zone ?zone-dst))
    =>
    ; it follows ?human
    (retract ?f ?f1 ?f2)
    (bind ?*plan_number* (+ 1 ?*plan_number*))
    ;(assert (attempt (name ftrans)(id ?*plan_number* )(follow ?human)))
    
    (printout t "(plan (name attend-ftrans)(id " ?*plan_number* ")(number 1)(actions goto " ?room-place " " ?zone-place"))" crlf)
    (printout t "(plan (name attend-ftrans)(id " ?*plan_number* ")(number 2)(actions find-human " ?human "))" crlf)
    (printout t "(plan (name attend-ftrans)(id " ?*plan_number* ")(number 3)(actions say-string [Please, take me to " ?destination ".]))" crlf)
    (printout t "(plan (name attend-ftrans)(id " ?*plan_number* ")(number 3)(actions follow " ?human "))" crlf)
)





; Go to the studio and follow Luis
; (PTRANS (ACTOR robot)(OBJ ?robot)(TO ?place))
; (FTRANS((ACTOR Robot)(obj ?human)))
(defrule exec-ptrans-ftrans-ROS
    ?f  <- (num-sentences 2)
    
    ?f1 <- (ptrans (actor ?robot)(obj ?robot)(to ?place))
    (item (type Robot) (name ?robot))
    (room (name ?place)(room ?room-place)(zone ?zone-place))
    
    ?f2 <- (ftrans (actor ?robot)(obj ?human))
    (item (type Human)(name ?human)(room ?room-human)(zone ?zone-human))
    =>
    ; it follows ?human
    (retract ?f ?f1 ?f2)
    (bind ?*plan_number* (+ 1 ?*plan_number*))
    ;(assert (attempt (name ftrans)(id ?*plan_number* )(follow ?human)))
    
    (printout t "(plan (name ptrans-ftrans)(id " ?*plan_number* ")(number 1)(actions goto " ?room-place " " ?zone-place"))" crlf)
    (printout t "(plan (name ptrans-ftrans)(id " ?*plan_number* ")(number 2)(actions find-human " ?human "))" crlf)
    (printout t "(plan (name ptrans-ftrans)(id " ?*plan_number* ")(number 3)(actions follow " ?human "))" crlf)
)

(defrule exec-ptrans-ftrans_to-ROS
    ?f  <- (num-sentences 2)
    
    ?f1 <- (ptrans (actor ?robot)(obj ?robot)(to ?place))
    (item (type Robot) (name ?robot))
    (room (name ?place)(room ?room-place)(zone ?zone-place))
    
    ?f2 <- (ftrans (actor ?robot)(obj ?human)(to ?destination))
    (item (type Human)(name ?human)(room ?room-human)(zone ?zone-human))
    (room (type Room)(name ?destination)(room ?room-dst)(zone ?zone-dst))
    =>
    ; it follows ?human
    (retract ?f ?f1 ?f2)
    (bind ?*plan_number* (+ 1 ?*plan_number*))
    ;(assert (attempt (name ftrans)(id ?*plan_number* )(follow ?human)))
    
    (printout t "(plan (name ptrans-ftrans)(id " ?*plan_number* ")(number 1)(actions goto " ?room-place " " ?zone-place"))" crlf)
    (printout t "(plan (name ptrans-ftrans)(id " ?*plan_number* ")(number 2)(actions find-human " ?human "))" crlf)
    (printout t "(plan (name ptrans-ftrans)(id " ?*plan_number* ")(number 3)(actions say-string [Please, take me to " ?destination ".]))" crlf)
    (printout t "(plan (name ptrans-ftrans)(id " ?*plan_number* ")(number 4)(actions follow " ?human "))" crlf)
)





; Go to the studio, find Luis, and follow him
; (PTRANS (ACTOR robot)(OBJ robot)(TO ?place))
; (ATTEND (ACTOR robot)(OBJ ?human))
; (FTRANS((ACTOR Robot)(obj ?human)))
(defrule exec-ptrans-attend-ftrans-ROS
    ?f  <- (num-sentences 3)
    
    ?f1 <- (ptrans (actor ?robot)(obj ?robot)(to ?place))
    (item (type Robot) (name ?robot))
    (room (name ?place)(room ?room-place)(zone ?zone-place))
    
    ?f2 <- (attend (actor ?robot)(obj ?human))
    ?f3 <- (ftrans (actor ?robot)(obj ?human)(to nil))
    (item (type Human)(name ?human)(room ?room-human)(zone ?zone-human))
    =>
    ; it follows ?human
    (retract ?f ?f1 ?f2 ?f3)
    (bind ?*plan_number* (+ 1 ?*plan_number*))
    ;(assert (attempt (name ftrans)(id ?*plan_number* )(follow ?human)))
    
    (printout t "(plan (name ptrans-attend-ftrans)(id " ?*plan_number* ")(number 1)(actions goto " ?room-place " " ?zone-place"))" crlf)
    (printout t "(plan (name ptrans-attend-ftrans)(id " ?*plan_number* ")(number 2)(actions find-human " ?human "))" crlf)
    (printout t "(plan (name ptrans-attend-ftrans)(id " ?*plan_number* ")(number 3)(actions follow " ?human "))" crlf)
)

(defrule exec-ptrans-attend-ftrans_to-ROS
    ?f  <- (num-sentences 3)
    
    ?f1 <- (ptrans (actor ?robot)(obj ?robot)(to ?place))
    (item (type Robot) (name ?robot))
    (room (name ?place)(room ?room-place)(zone ?zone-place))
    
    ?f2 <- (attend (actor ?robot)(obj ?human))
    ?f3 <- (ftrans (actor ?robot)(obj ?human)(to ?destination))
    (item (type Human)(name ?human)(room ?room-human)(zone ?zone-human))
    (room (type Room)(name ?destination)(room ?room-dst)(zone ?zone-dst))
    =>
    ; it follows ?human
    (retract ?f ?f1 ?f2 ?f3)
    (bind ?*plan_number* (+ 1 ?*plan_number*))
    ;(assert (attempt (name ftrans)(id ?*plan_number* )(follow ?human)))
    
    (printout t "(plan (name ptrans-attend-ftrans)(id " ?*plan_number* ")(number 1)(actions goto " ?room-place " " ?zone-place"))" crlf)
    (printout t "(plan (name ptrans-attend-ftrans)(id " ?*plan_number* ")(number 2)(actions find-human " ?human "))" crlf)
    (printout t "(plan (name ptrans-attend-ftrans)(id " ?*plan_number* ")(number 3)(actions say-string [Please, take me to " ?destination ".]))" crlf)
    (printout t "(plan (name ptrans-attend-ftrans)(id " ?*plan_number* ")(number 4)(actions follow " ?human "))" crlf)
)





;;;;;;;;;;;;
; Incomplete conceptual dependencies

;;;(ATRANS (ACTOR robot)(TO nil))
(defrule exec-atrans-no-recipient-ROS
    ?f <- (num-sentences 1)
    (atrans (actor ?actor)(to nil))
    (item (type Robot)(name ?actor))
    =>
    ; it asks who is the human that receives the object
    (retract ?f)
    (bind ?*plan_number* (+ 1 ?*plan_number*))
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


;;;(ATRANS (ACTOR robot)(OBJ ?obj)(TO ?place))
;;;(item (type Objects)(name ?obj)(zone nil))
(defrule exec-atrans-no-object-location-ROS
    ?f  <- (num-sentences 1)
    ?f1 <- (atrans (actor ?actor)(obj ?obj)(to ?place))
    (item (type Robot) (name ?actor))
    (item (type Objects)(name ?obj)(zone nil))
    =>
    ; it asks where is the object
    (retract ?f)
    (bind ?*plan_number* (+ 1 ?*plan_number*))
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
