;****************************************
;*                                      *
;* gpsr_deftemplates.clp                *
;*                                      *
;*          University of Mexico        *
;*          Jesus Savage-Carmona        *
;*                                      *
;*          20 Dec 2022                 *
;*                                      *
;****************************************

(deftemplate item
	(field type
		(type SYMBOL)
		(default nil)
	)
	(field name
		(type SYMBOL)
		(default nil)
	)
	(field room
		(type SYMBOL)
		(default nil)
	)
	(field zone
		(type SYMBOL)
		(default nil)
	)
	(multifield attributes
		(type SYMBOL)
		(default nil)
	)
	(multifield pose
		(type NUMBER)
		(default 0 0 0)
	)
	(field lower
		(type SYMBOL)
		(default base)
	)
	(field upper
		(type SYMBOL)
		(default nothing)
	)
	(field grasp
		(type SYMBOL)
		(default nil)
	)
	(field possession
		(type SYMBOL)
		(default nobody)
	)
	(multifield status
		(type SYMBOL)
		(default nil)
	)
	(field num
		(type NUMBER)
		(default 1)
	)
)


(deftemplate room
	(field type
		(type SYMBOL)
		(default Room)
	)
	(field name
		(type SYMBOL)
		(default nil)
	)
	(field room
		(type SYMBOL)
		(default nil)
	)
	(field zone
		(type SYMBOL)
		(default nil)
	)
	(multifield zones
		(type SYMBOL)
		(default nil)
	)
	(multifield attributes
		(type SYMBOL)
		(default nil)
	)
	(multifield center
		(type NUMBER)
		(default 0 0 0 0)
	)
	(field num
		(type NUMBER)
		(default 1)
	)
)


(deftemplate arm
	(field type
		(type SYMBOL)
		(default Arm)
	)
	(field name
		(type SYMBOL)
		(default nil)
	)
	(field obj
		(type SYMBOL)
		(default nil)
	)
	(field status
		(type SYMBOL)
		(default nil)
	)
	(field num
		(type NUMBER)
		(default 1)
	)
)


(deftemplate state
	(field type
		(type SYMBOL)
		(default State)
	)
	(field attribute
		(type SYMBOL)
		(default nil)
	)
	(field obj
		(type SYMBOL)
		(default nil)
	)
	(field human 
		(type SYMBOL)
		(default nil)
	)
	(field value
		(type SYMBOL)
		(default nil)
	)
)

