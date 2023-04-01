;****************************************
;*                                      *
;* gpsr_initial_states.clp              *
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

(deffacts Initial-state-objects-rooms-zones-actors

    ( item (type Objects) (name chips_can) (room living_room) (zone large_table) (attributes food_items) (pose 0.000000 0.000000 0.000000) (num 1) )
    ( item (type Objects) (name banana) (room nil) (zone nil) (attributes food_items) (pose 0.000000 0.000000 0.000000) (num 1) )
    ( item (type Objects) (name apple) (room living_room) (zone large_table) (attributes food_items) (pose 0.000000 0.000000 0.000000) (num 1) )

    ( item (type Human) (name paola) (room office) (zone office_unknown) (pose 0.000000 0.000000 0.000000) (num 1) )
    ( item (type Human) (name luis) (room living_room) (zone living_room_unknown) (pose 0.000000 0.000000 0.000000) (num 1) )

    ( room (type Room) (name front_entrance) (room living_room) (zone front_entrance) (center 0.000000 0.000000 0.000000 0.000000) )
    ( room (type Room) (name office_unknown) (room office) (zone office_unknown) (center 0.000000 0.000000 0.000000 0.000000) )
    ( room (type Room) (name living_room_unknown) (room living_room) (zone living_room_unknown) (center 0.000000 0.000000 0.000000 0.000000) )
    ( room (type Room) (name large_table) (room living_room) (zone large_table) (center 0.000000 0.000000 0.000000 0.000000) )

    ( item (type Robot) (name robot) (zone front_entrance) (pose 0.000000 0.000000 0.000000) )
    ( arm (type Arm) (name left) (num 1) )
)

