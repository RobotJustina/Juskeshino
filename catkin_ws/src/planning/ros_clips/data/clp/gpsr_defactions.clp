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
                                                                                                                         

(deftemplate qtrans
    (field actor
        (type SYMBOL)
        (default nil)
    )
    (field question
        (type SYMBOL)
        (default nil)
    )
    (field aux-verb
        (type SYMBOL)
        (default nil)
    )
    (field verb
        (type SYMBOL)
        (default nil)
    )
    (field human
        (type SYMBOL)
        (default nil)
    )
    (field obj
        (type SYMBOL)
        (default nil)
    )
    (multifield answer
        (type SYMBOL)
        (default nil)
    )
)


