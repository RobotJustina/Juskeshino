;****************************************
;*                                      *
;*  gpsr_rules.clp                      *
;*                                      *
;*          University of Mexico        *
;*          Jesus Savage-Carmona        *
;*                                      *
;*          11/22/22                    *
;*                                      *
;****************************************

(defglobal ?*plan_number* = 0)
(defglobal ?*plan_time* = 30000)
(defglobal ?*plan_number_new* = 0)

;****************************************
;*                                      *
;*               Rules                  *
;*                                      *
;****************************************


;#######################################
;      Clear stack rules


(defrule move-to-free-space-ROS
    (declare (salience 100))
    (num-sentences 1)
    ?goal <- (goal (move ?obj1) (on freespace))
    (ptrans (actor ?actor)(obj ?obj)(to ?place))
    ?f1 <- (item (type Objects) (name ?obj)(room ?room)(zone ?zone))
    (item (type Robot) (name ?actor))
    ?f2 <- (item (type Objects) (name ?obj1)(room ?room)(zone ?zone)(upper nothing)(lower ?obj))
    =>
    (retract ?goal)
    (modify ?f1 (upper nothing))
    (modify ?f2 (lower base))
    (bind ?*plan_number* (+ 1 ?*plan_number*))
    (bind ?*plan_number_new* (+ 1 ?*plan_number_new*))
    ;(printout t ?obj1 " will be moved onto free space in room " ?room crlf)
    ;(assert (plan (name ptrans)(id ?*plan_number*)(number 1 )(actions goto ?room ?zone)(duration ?*plan_time*)) )
    ;(assert (plan (name ptrans)(id ?*plan_number*)(number 2 )(actions find-object ?obj1)(duration ?*plan_time*)) )
    ;(assert (plan (name ptrans)(id ?*plan_number*)(number 3 )(actions grab ?obj1 )(duration ?*plan_time*)) )
    ;(assert (plan (name ptrans)(id ?*plan_number*)(number 4 )(actions find-object freespace)(duration ?*plan_time*)) )
    ;(assert (plan (name ptrans)(id ?*plan_number*)(number 5 )(actions go freespace)(duration ?*plan_time*)) )
    ;(assert (plan (name ptrans)(id ?*plan_number*)(number 6 )(actions drop ?obj1 )(duration ?*plan_time*)) )
    ;(assert (attempt (name ptrans)(id ?*plan_number*)(move ?obj1)(room ?room)(zone ?zone)(on freespace)(number 6 ) ))
    
    (printout t "(attempt (name ptrans)(id " ?*plan_number* ")(move " ?obj1 ")(room " ?room ")(zone " ?zone")(on freespace))" crlf)
)


(defrule atrans-move-to-free-space-ROS
    (declare (salience 100))
    ?goal <- (goal (move ?obj1) (on freespace))
    (atrans (actor ?actor)(obj ?obj)(to ?place))
    ?f1 <- (item (type Objects) (name ?obj)(room ?room)(zone ?zone))
    (item (type Robot) (name ?actor))
    ?f2 <- (item (type Objects) (name ?obj1)(room ?room)(zone ?zone)(upper nothing)(lower ?obj))
    =>
    (retract ?goal)
    (modify ?f1 (upper nothing))
    (modify ?f2 (lower base))
    (bind ?*plan_number* (+ 1 ?*plan_number*))
    (bind ?*plan_number_new* (+ 1 ?*plan_number_new*))
    ;(printout t ?obj1 " will be moved onto free space in room " ?room crlf)
    ;(assert (plan (name atrans)(id ?*plan_number*)(number 1 )(actions goto ?room ?zone)(duration ?*plan_time*)) )
    ;(assert (plan (name atrans)(id ?*plan_number*)(number 2 )(actions find-object ?obj1)(duration ?*plan_time*)) )
    ;(assert (plan (name atrans)(id ?*plan_number*)(number 3 )(actions grab ?obj1 )(duration ?*plan_time*)) )
    ;(assert (plan (name atrans)(id ?*plan_number*)(number 4 )(actions find-object freespace)(duration ?*plan_time*)) )
    ;(assert (plan (name atrans)(id ?*plan_number*)(number 5 )(actions go freespace)(duration ?*plan_time*)) )
    ;(assert (plan (name atrans)(id ?*plan_number*)(number 6 )(actions drop ?obj1 )(duration ?*plan_time*)) )
    ;(assert (attempt (name atrans)(id ?*plan_number*)(move ?obj1)(room ?room)(zone ?zone)(on freespace)(number 6 ) ))
    
    (printout t "(attempt (name atrans)(id " ?*plan_number* ")(move " ?obj1 ")(room " ?room ")(zone " ?zone")(on freespace))" crlf)
)


(defrule move-to-free-space-objects-ROS
    (declare (salience 100))
    ?goal <- (goal (move ?obj1) (on freespace))
    ?goal2 <- (goal (move ?obj2) (on freespace))
    (ptrans (actor ?actor)(obj ?obj)(to ?place))
    (item (type Robot) (name ?actor))
    ?f2 <- (item (type Objects) (name ?obj1)(room ?room)(zone ?zone)(upper nothing)(lower ?obj2))
    ?f1 <- (item (type Objects) (name ?obj2)(room ?room)(zone ?zone))
    =>
    (retract ?goal)
    (modify ?f1 (upper nothing))
    (modify ?f2 (lower base))
    (bind ?*plan_number* (+ 1 ?*plan_number*))
    (bind ?*plan_number_new* (+ 1 ?*plan_number_new*))
    ;(printout t ?obj1 " will be moved onto free space in room " ?room crlf)
    ;(assert (plan (name ptrans)(id ?*plan_number*)(number 1 )(actions goto ?room ?zone)(duration ?*plan_time*)) )
    ;(assert (plan (name ptrans)(id ?*plan_number*)(number 2 )(actions find-object ?obj1)(duration ?*plan_time*)) )
    ;(assert (plan (name ptrans)(id ?*plan_number*)(number 3 )(actions grab ?obj1 )(duration ?*plan_time*)) )
    ;(assert (plan (name ptrans)(id ?*plan_number*)(number 4 )(actions find-object freespace)(duration ?*plan_time*)) )
    ;(assert (plan (name ptrans)(id ?*plan_number*)(number 5 )(actions go freespace)(duration ?*plan_time*)) )
    ;(assert (plan (name ptrans)(id ?*plan_number*)(number 6 )(actions drop ?obj1 )(duration ?*plan_time*)) )
    ;(assert (attempt (name ptrans)(id ?*plan_number*)(move ?obj1)(room ?room)(zone ?zone)(on freespace)(number 6 ) ))
    
    (printout t "(attempt (name ptrans)(id " ?*plan_number* ")(move " ?obj1 ")(room " ?room ")(zone " ?zone")(on freespace))" crlf)
)


(defrule atrans-move-to-free-space-objects-ROS
    (declare (salience 100))
    ?goal <- (goal (move ?obj1) (on freespace))
    ?goal2 <- (goal (move ?obj2) (on freespace))
    (atrans (actor ?actor)(obj ?obj)(to ?place))
    (item (type Robot) (name ?actor))
    ?f2 <- (item (type Objects) (name ?obj1)(room ?room)(zone ?zone)(upper nothing)(lower ?obj2))
    ?f1 <- (item (type Objects) (name ?obj2)(room ?room)(zone ?zone))
    =>
    (retract ?goal)
    (modify ?f1 (upper nothing))
    (modify ?f2 (lower base))
    (bind ?*plan_number* (+ 1 ?*plan_number*))
    (bind ?*plan_number_new* (+ 1 ?*plan_number_new*))
    ;(printout t ?obj1 " will be moved onto free space in room " ?room crlf)
    ;(assert (plan (name atrans)(id ?*plan_number*)(number 1 )(actions goto ?room ?zone)(duration ?*plan_time*)) )
    ;(assert (plan (name atrans)(id ?*plan_number*)(number 2 )(actions find-object ?obj1)(duration ?*plan_time*)) )
    ;(assert (plan (name atrans)(id ?*plan_number*)(number 3 )(actions grab ?obj1 )(duration ?*plan_time*)) )
    ;(assert (plan (name atrans)(id ?*plan_number*)(number 4 )(actions find-object freespace)(duration ?*plan_time*)) )
    ;(assert (plan (name atrans)(id ?*plan_number*)(number 5 )(actions go freespace)(duration ?*plan_time*)) )
    ;(assert (plan (name atrans)(id ?*plan_number*)(number 6 )(actions drop ?obj1 )(duration ?*plan_time*)) )
    ;(assert (attempt (name atrans)(id ?*plan_number*)(move ?obj1)(room ?room)(zone ?zone)(on freespace)(number 6 ) ))
    
    (printout t "(attempt (name atrans)(id " ?*plan_number* ")(move " ?obj1 ")(room " ?room ")(zone " ?zone")(on freespace))" crlf)
)


(defrule clear-first-upper-obj-ROS
    (declare (salience 100))
    (num-sentences 1)
    (ptrans (actor ?actor)(obj ?obj)(to ?place))
    (item (type Objects) (name ?obj)(room ?room)(zone ?zone)(upper ?upper-obj&:(neq ?upper-obj nothing)))
    =>
    (assert (goal (move ?upper-obj)(on freespace)))
)


(defrule clear-atrans-first-upper-obj-ROS
    (declare (salience 100))
    (num-sentences 1)
    (atrans (actor ?actor)(obj ?obj)(to ?place))
    (item (type Objects) (name ?obj)(room ?room)(zone ?zone)(upper ?upper-obj&:(neq ?upper-obj nothing)))
    =>
    (assert (goal (move ?upper-obj)(on freespace)))
)


(defrule clear-upper-objs-ROS
    (declare (salience 200))
    (num-sentences 1)
    (ptrans (actor ?actor)(obj ?obj)(to ?place))
    ?goal <- (goal (move ?obj1) (on freespace))
    ?f1 <- (item (type Objects) (name ?obj1)(room ?room)(zone ?zone)(upper ?upper-obj&:(neq ?upper-obj nothing)))
    =>
    (assert (goal (move ?upper-obj)(on freespace)))
    ;(modify ?f1 (upper nothing))
)


(defrule clear-atrans-upper-objs-ROS
    (declare (salience 200))
    (num-sentences 1)
    (atrans (actor ?actor)(obj ?obj)(to ?place))
    ?goal <- (goal (move ?obj1) (on freespace))
    ?f1 <- (item (type Objects) (name ?obj1)(room ?room)(zone ?zone)(upper ?upper-obj&:(neq ?upper-obj nothing)))
    =>
    (assert (goal (move ?upper-obj)(on freespace)))
    ;(modify ?f1 (upper nothing))
)

