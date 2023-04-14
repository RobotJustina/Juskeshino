;****************************************
;*                                      *
;* gpsr_defactions.clp                  *
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

;(PTRANS((ACTOR Robot)(OBJ Robot)(TO ?location)))
(deftemplate ptrans
    (field actor
        (type SYMBOL)
        (default nil)
    )
    (field obj
        (type SYMBOL)
        (default nil)
    )
    (field from
        (type SYMBOL)
        (default nil)
    )
    (field to
        (type SYMBOL)
        (default nil)
    )
)


;(ATRANS((ACTOR Robot)(OBJ ?obj)(TO ?person)))
(deftemplate atrans
    (field actor
        (type SYMBOL)
        (default nil)
    )
    (field obj
        (type SYMBOL)
        (default nil)
    )
    (field from
        (type SYMBOL)
        (default nil)
    )
    (field to
        (type SYMBOL)
        (default nil)
    )
)


;(SPEAK((MSG ?msg)(TO ?human))
(deftemplate speak
    (field actor
        (type SYMBOL)
        (default nil)
    )
    (multifield msg
        (type SYMBOL)
        (default nil)
    )
    (field from
        (type SYMBOL)
        (default nil)
    )
    (field to
        (type SYMBOL)
        (default nil)
    )
)


;(MTRANS((ACTOR Robot)(MSG ?msg)(from ?source)(to ?target)))
(deftemplate mtrans
    (field actor
        (type SYMBOL)
        (default nil)
    )
    (multifield msg
        (type SYMBOL)
        (default nil)
    )
    (field from
        (type SYMBOL)
        (default nil)
    )
    (field to
        (type SYMBOL)
        (default nil)
    )
)


;(ATTEND((ACTOR Robot)(OBJ ?obj)))
(deftemplate attend
    (field actor
        (type SYMBOL)
        (default nil)
    )
    (field obj
        (type SYMBOL)
        (default nil)
    )
    (field from
        (type SYMBOL)
        (default nil)
    )
    (field to
        (type SYMBOL)
        (default nil)
    )
)


;(GRAB((ACTOR Robot)(OBJ ?obj)))
(deftemplate grab
    (field actor
        (type SYMBOL)
        (default nil)
    )
    (field obj
        (type SYMBOL)
        (default nil)
    )
    (field from
        (type SYMBOL)
        (default nil)
    )
    (field to
        (type SYMBOL)
        (default nil)
    )
)


;(RELEASE((ACTOR Robot)(OBJ ?obj)(TO ?place)))
(deftemplate release
    (field actor
        (type SYMBOL)
        (default nil)
    )
    (field obj
        (type SYMBOL)
        (default nil)
    )
    (field from
        (type SYMBOL)
        (default nil)
        )
    (field to
        (type SYMBOL)
        (default nil)
    )
)
                                                                                                                         

;(QTRANS((OBJ ?obj)(QUESTION ?word))) 
(deftemplate qtrans
    (field actor
        (type SYMBOL)
        (default nil)
    )
    (field question
        (type SYMBOL)
        (default nil)
    )
    (field obj
        (type SYMBOL)
        (default nil)
    )
)


;(PROPEL((ACTOR Robot)(OBJ ?obj)(ACTION ?action)))
(deftemplate propel
    (field actor
        (type SYMBOL)
        (default nil)
    )
    (field obj
        (type SYMBOL)
        (default nil)
    )
    (field action
        (type SYMBOL)
        (allowed-symbols open close nil)
        (default nil)
    )
)


;(FOLLOW((ACTOR Robot)(OBJ ?obj)))
(deftemplate follow
    (field actor
        (type SYMBOL)
        (default nil)
    )
    (field obj
        (type SYMBOL)
        (default nil)
    )
)
