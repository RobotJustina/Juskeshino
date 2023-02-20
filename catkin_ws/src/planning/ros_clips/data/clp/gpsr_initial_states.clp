;****************************************
;*                                      *
;* gpsr_initial_states.clp              *
;*                                      *
;*          University of Mexico        *
;*          Jesus Savage-Carmona        *
;*                                      *
;*          20 Dec 2022                 *
;*                                      *
;****************************************

(deffacts Initial-state-objects-rooms-zones-actors

; Objects definitions
    ( item (type Objects) (name mail_box)(room corridor)(image table)( attributes no-pick brown)(pose 0.25 1.00 0.0) )
    ( item (type Objects) (name cold_storage)(room kitchen)(image fridge)( attributes no-pick white)(pose 1.5 1.66 0.0) )
    ( item (type Objects) (name tools_storage)(room deposit)(image shelf)( attributes no-pick brown)(pose 0.4 1.63 0.0) )
    ( item (type Objects) (name library)(room studio)(image shelf)( attributes no-pick brown)(pose 0.2 0.16 0.0))
    ( item (type Objects) (name service_table)(room service)(image table)( attributes no-pick brown)(pose 1.65 0.35 0.0) )
    ( item (type Objects) (name desk)(room studio)(image desk)( attributes no-pick brown)(pose 0.25 0.74 0.0) )
    ( item (type Objects) (name bed)(room bedroom)(image bed)( attributes no-pick white)(pose 0.95 0.36 0.0) )

    ( item (type Objects) (name apple)(room corridor)(zone deposit)(image apple)(attributes pick)(pose 0.40 1.0 0.0) )
    ( item (type Objects) (name sushi)(room corridor)(zone deposit)(image sushi)(attributes pick)(pose 0.30 1.0 0.0) )
    ( item (type Objects) (name milk)(room corridor)(zone deposit)(image milk)(attributes pick)(pose 0.20 1.0 0.0) )

    ( item (type Objects) (name soap)(room corridor)(zone deposit)(image soap)(attributes pick)(pose 0.50 1.1 0.0) )
    ( item (type Objects) (name perfume)(room corridor)(zone deposit)(image perfume)(attributes pick)(pose 0.37 1.1 0.0) )
    ( item (type Objects) (name shampoo)(room corridor)(zone deposit)(image shampoo)(attributes pick)(pose 0.20 1.1 0.0) )

    ( item (type Objects) (name book)(room studio)(zone library)(image book)(attributes pick)(pose 0.2 1.50 0.0) )
    ( item (type Objects) (name hammer)(room deposit)(zone tools_storage)(image hammer)(attributes pick)(pose 1.6 0.4 0.0) )

    ( item (type Objects) (name freespace)(room any)(zone any)(image none)(attributes none)(pose 0.0 0.0 0.0) )

; Rooms definitions
    ( Room (name deposit)(zone any)(zones dummy1 frontexit frontentrance storage dummy2)(center 0.70 1.51 0.0) )
    ( Room (name corridor)(zone mail_box)(zones dummy1 frontexit frontentrance storage dummy2)(center 0.6 1.0 0.0) )
    ( Room (name studio)(zone any)(zones dummy1 frontexit desk storage dummy2)(center 0.25 1.45 0.0) )
    ( Room (name service)(zone service_table)(zones dummy1 frontexit frontentrance storage dummy2)(center 1.65 0.55 0.0) )
    ( Room (name kitchen)(zone cold_storage)(zones dummy1 frontexit frontentrance deposit dummy2)(center 1.60 1.40 0.0) )
    ( Room (name bedroom)(zone desk)(zones dummy1 frontexit frontentrance deposit dummy2)(center 1.0 0.55 0.0) )

; Humans definitions
    ( item (type Human) (name mother)(room studio)(zone desk)(pose 1.048340 1.107002 0.0) )
    ( item (type Human) (name father)(room kitchen)(zone stove)(pose 1.048340 1.107002 0.0) )

; Robots definitions
    ( item (type Robot) (name robot)(zone frontexit)(pose 1.048340 1.107002 0.0) )

; Furniture definitions
    ( item (type Furniture) (name fridge)(zone kitchen)(image fridge)( attributes no-pick white)(pose 1.50 1.436 0.0) )
    ( item (type Furniture) (name table)(zone service)(image table)( attributes no-pick brown)(pose 1.65 0.35 0.0) )

; Doors definitions
    ( item (type Door) (name outsidedoor) (status closed) )

; Arm definition
    ( Arm (name left))

;Stacks definitions
    ;( stack corridor mail_box apple sushi milk )
    ;( stack corridor mail_box soap perfume shampoo )
    ;( stack service tools_storage hammer )
    ;( stack studio library book )

    ;( real-stack corridor mail_box apple sushi milk )
    ;( real-stack corridor mail_box soap perfume shampoo )
    ;( real-stack service tools_storage hammer )
    ;( real-stack studio library book )

    ;( goal-stack 1 service service_table soap)
    ;( goal-stack 4 studio library hammer)
    ;( goal-stack 3 bedroom bed book)
    ;( goal-stack 2 service service_table soap perfume shampoo)
    ;( goal-stack 1 kitchen cold_storage sushi apple milk)

    ;( ptrans (actor robot)(obj robot)(to service) )
    ;( ptrans (actor robot)(obj robot)(to kitchen) )
    ;( ptrans (actor robot)(obj book)(to bedroom) )
    ;( ptrans (actor robot)(obj book)(to bedroom) )
    ;( ptrans (actor robot)(obj robot)(to mother) )
    ;( atrans (actor robot)(obj book)(to father) )
    ;( attempt (name cubes)(id 0)(number 0) )

)

