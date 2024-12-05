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

;****************************************
;*                                      *
;*               Rules                  *
;*                                      *
;****************************************




;#######################################
;      Global Rules


(defrule delete-goto-ROS
    ?f  <- (plans ?id ?num ?success goto)
    ?f1 <- (plan (id ?id)(number ?num )(actions goto ?room ?zone))
    ?f2 <- (item (type Robot) (name ?actor))
    =>
    (retract ?f ?f1)
    ; it modifies the robot position
    (if (eq ?success 1) then
        (modify ?f2 (room ?room)) 
    )
)


(defrule delete-grab-ROS
    ?f  <- (plans ?id ?num ?success grab)
    ?f1 <- (plan (id ?id)(number ?num )(actions grab ?object))
    ?f2 <- (item (type Robot) (name ?actor))
    ?f3 <- (item (type Objects)(name ?object))
    =>
    (retract ?f ?f1)
    ; it modifies the robot position
    (if (eq ?success 1) then
        (modify ?f2 (hands ?object))
        (modify ?f3 (room robot))
    )
)


(defrule delete-drop-ROS
    ?f  <- (plans ?id ?num ?success dropped)
    ?f1 <- (plan (id ?id)(number ?num )(actions drop ?object))
    ?f2 <- (item (type Robot) (name ?actor)(room ?zone-robot))
    ?f3 <- (item (type Objects)(name ?object))
    =>
    (retract ?f ?f1)
    ; it modifies the robot position
    (if (eq ?success 1) then
        (modify ?f2 (hands nil))
        (modify ?f3 (room ?zone-robot))
    )
)


(defrule delete-find-object-ROS
    ?f  <- (plans ?id ?num ?success find-object)
    ?f1 <- (plan (id ?id)(number ?num )(actions find-object ?object))
    =>
    (retract ?f ?f1)
)


(defrule delete-mv-ROS
    ?f  <- (plans ?id ?num ?success mv) 
    ?f1 <- (plan (id ?id)(number ?num )(actions mv ?object))
    =>
    (retract ?f ?f1)
)


(defrule delete-go-place-ROS
    ?f  <- (plans ?id ?num ?success go) 
    ?f1 <- (plan (id ?id)(number ?num )(actions go ?place))
    =>
    (retract ?f ?f1)
)


(defrule delete-attempt-atrans-ROS
    (declare (salience 100))
    ?f  <- (attempts ?id ?success)
    ?f1 <- (attempt (name atrans) (id ?id )(move ?obj)(room ?room-human)(zone ?zone-human)(on ?human))
    ?f2 <- (item (type Objects) (name ?obj)(room ?room-obj)(zone ?zone))
    ?f3 <- (item (type Human) (name ?human)(room ?room-human)(zone ?zone-human))
    =>
    (retract ?f)
    (if (eq ?success 1) then
        (retract ?f1)
        (modify ?f2 (room ?room-human)(zone ?zone-human)(possession ?human))
        (modify ?f3 (objs ?obj))
    else
        (assert (fail-plan ?id))
        (retract ?f1) ; modify this later
    )
)


(defrule delete-attempt-generic-ROS
    ?f  <- (attempts ?id ?success)
    ?f1 <- (attempt (id ?id))
    =>
    (retract ?f)
    (if (eq ?success 1) then
        (retract ?f1)
    else
        (assert (fail-plan ?id))
        (retract ?f1) ; modify this later
    )
)


