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
	(multifield status
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
	(field grasp
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
	(field possession
		(type SYMBOL)
		(default nobody)
	)
	(field image
		(type SYMBOL)
		(default nil)
	)
	(field script
		(type SYMBOL)
		(default nil)
	)
	(field num
		(type NUMBER)
		(default 1)
	)
	(field shared
		(type SYMBOL)
		(default false)
	)
	(multifield zones
		(type SYMBOL)
		(default nil)
	)
	(multifield hands
		(type SYMBOL)
		(default nil)
	)
	(multifield objs
		(type SYMBOL)
		(default nil)
	)
	(field lower
		(type SYMBOL)
		(default base)
	)
	(field upper
		(type SYMBOL)
		(default nothing)
	)
)


(deftemplate Room
	(field name
		(type SYMBOL)
		(default nil)
	)
	(field status
		(type SYMBOL)
		(default nil)
	)
	(multifield attributes
		(type SYMBOL)
		(default nil)
	)
	(field grasp
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
	(field possession
		(type SYMBOL)
		(default nobody)
	)
	(field image
		(type SYMBOL)
		(default nil)
	)
	(field script
		(type SYMBOL)
		(default nil)
	)
	(field num
		(type NUMBER)
		(default 1)
	)
	(field shared
		(type SYMBOL)
		(default false)
	)
	(multifield center
		(type NUMBER)
		(default 0 0 0 0)
	)
)


(deftemplate Arm
	(field name
		(type SYMBOL)
		(default nil)
	)
	(field status
		(type SYMBOL)
		(default nil)
	)
	(multifield attributes
		(type SYMBOL)
		(default nil)
	)
	(field possession
		(type SYMBOL)
		(default nobody)
	)
	(field grasp
		(type SYMBOL)
		(default nil)
	)
	(field num
		(type NUMBER)
		(default 1)
	)
	(field flag
		(type SYMBOL)
		(default nil)
	)
)


(deftemplate state
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

