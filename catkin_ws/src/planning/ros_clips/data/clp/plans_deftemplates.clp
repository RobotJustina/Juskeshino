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

(deftemplate plan
	(field name
		(type SYMBOL)
		(default nil)
	)
	(field id
		(type NUMBER)
		(default 1)
	)
	(field number
		(type NUMBER)
		(default 1)
	)
		(multifield actions
		(type SYMBOL)
	)
	(field duration
		(type NUMBER)
		(default 1)
	)
	(field status
		(type SYMBOL)
		(default inactive)
	)
	(
	field statusTwo
		(type SYMBOL)
		(default active)
	)
)


(deftemplate goal 
	(slot move)
	(slot room)
	(slot zone)
	(slot on)
)


(deftemplate attempt 
	(field name
		(type SYMBOL)
		(default nil)
	)
	(field id
		(type NUMBER)
		(default 1)
	)
	(field status 
		(type SYMBOL)
		(default nil)
	)
	
	(slot move)
	(slot state)
	(slot room)
	(slot zone)
	(slot on)
	(slot number) 
)

