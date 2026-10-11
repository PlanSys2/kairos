(define (domain cooking)
(:requirements :strips :typing :adl :fluents :durative-actions)

;; Types ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(:types
ingredient
dish
zone
robot
);; end Types ;;;;;;;;;;;;;;;;;;;;;;;;;

;; Predicates ;;;;;;;;;;;;;;;;;;;;;;;;;
(:predicates

(robot_at ?r - robot ?z - zone)
(robot_free ?r - robot)            ; not moving, carrying, cooking or recharging

(is_egg ?i - ingredient)
(is_oil ?i - ingredient)
(is_salt ?i - ingredient)
(is_flour ?i - ingredient)
(is_sugar ?i - ingredient)
(is_pasta ?i - ingredient)
(is_water ?i - ingredient)

(is_omelette ?d - dish)
(is_cake ?d - dish)
(is_spaghetti ?d - dish)

(is_cooking_zone ?z - zone)
(is_pantry_zone ?z - zone)
(is_fridge_zone ?z - zone)
(is_watertap_zone ?z - zone)
(is_recharge_zone ?z - zone)

(stove_free ?z - zone)             ; one dish at a time per cooking zone

(dish_prepared ?d - dish)

);; end Predicates ;;;;;;;;;;;;;;;;;;;;


;; Functions ;;;;;;;;;;;;;;;;;;;;;;;;;
(:functions

(stock ?i - ingredient ?z - zone)  ; units of an ingredient in a zone
(battery ?r - robot)               ; 0..100

);; end Functions ;;;;;;;;;;;;;;;;;;;;
;; Actions ;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(:durative-action move
    :parameters (?r - robot ?z1 ?z2 - zone)
    :duration ( = ?duration 5)
    :condition (and
        (at start (robot_free ?r))
        (at start (robot_at ?r ?z1))
        (at start (>= (battery ?r) 5)))
    :effect (and
        (at start (not (robot_free ?r)))
        (at start (not (robot_at ?r ?z1)))
        (at start (decrease (battery ?r) 5))
        (at end (robot_at ?r ?z2))
        (at end (robot_free ?r))
    )
)

; One unit at a time, slower and more costly than moving empty
(:durative-action transport
    :parameters (?r - robot ?i - ingredient ?z1 ?z2 - zone)
    :duration ( = ?duration 8)
    :condition (and
        (at start (robot_free ?r))
        (at start (robot_at ?r ?z1))
        (at start (>= (stock ?i ?z1) 1))
        (at start (>= (battery ?r) 10))
    )
    :effect (and
        (at start (not (robot_free ?r)))
        (at start (not (robot_at ?r ?z1)))
        (at start (decrease (stock ?i ?z1) 1))
        (at start (decrease (battery ?r) 10))
        (at end (robot_at ?r ?z2))
        (at end (increase (stock ?i ?z2) 1))
        (at end (robot_free ?r))
    )
)

(:durative-action recharge
    :parameters (?r - robot ?z - zone)
    :duration ( = ?duration 10)
    :condition (and
        (at start (robot_free ?r))
        (at start (is_recharge_zone ?z))
        (at start (< (battery ?r) 100))
        (over all (robot_at ?r ?z))
      )
    :effect (and
        (at start (not (robot_free ?r)))
        (at end (assign (battery ?r) 100))
        (at end (robot_free ?r))
    )
)

; Cooking uses one unit of each ingredient, the stove and the robot
(:durative-action cook_omelette
    :parameters (?r - robot ?z - zone ?i1 ?i2 ?i3 - ingredient ?d - dish)
    :duration ( = ?duration 10)
    :condition (and
        (at start (robot_free ?r))
        (at start (stove_free ?z))
        (at start (is_cooking_zone ?z))
        (over all (robot_at ?r ?z))
        (at start (>= (battery ?r) 10))
        (at start (is_egg ?i1))
        (at start (is_oil ?i2))
        (at start (is_salt ?i3))
        (at start (>= (stock ?i1 ?z) 1))
        (at start (>= (stock ?i2 ?z) 1))
        (at start (>= (stock ?i3 ?z) 1))
        (at start (is_omelette ?d))
    )
    :effect (and
        (at start (not (robot_free ?r)))
        (at start (not (stove_free ?z)))
        (at start (decrease (battery ?r) 10))
        (at start (decrease (stock ?i1 ?z) 1))
        (at start (decrease (stock ?i2 ?z) 1))
        (at start (decrease (stock ?i3 ?z) 1))
        (at end (dish_prepared ?d))
        (at end (stove_free ?z))
        (at end (robot_free ?r))
    )
)

(:durative-action cook_cake
    :parameters (?r - robot ?z - zone ?i1 ?i2 ?i3 - ingredient ?d - dish)
    :duration ( = ?duration 20)
    :condition (and
        (at start (robot_free ?r))
        (at start (stove_free ?z))
        (at start (is_cooking_zone ?z))
        (over all (robot_at ?r ?z))
        (at start (>= (battery ?r) 10))
        (at start (is_flour ?i1))
        (at start (is_egg ?i2))
        (at start (is_sugar ?i3))
        (at start (>= (stock ?i1 ?z) 1))
        (at start (>= (stock ?i2 ?z) 1))
        (at start (>= (stock ?i3 ?z) 1))
        (at start (is_cake ?d))
    )
    :effect (and
        (at start (not (robot_free ?r)))
        (at start (not (stove_free ?z)))
        (at start (decrease (battery ?r) 10))
        (at start (decrease (stock ?i1 ?z) 1))
        (at start (decrease (stock ?i2 ?z) 1))
        (at start (decrease (stock ?i3 ?z) 1))
        (at end (dish_prepared ?d))
        (at end (stove_free ?z))
        (at end (robot_free ?r))
    )
)

(:durative-action cook_spaghetti
    :parameters (?r - robot ?z - zone ?i1 ?i2 ?i3 ?i4 - ingredient ?d - dish)
    :duration ( = ?duration 15)
    :condition (and
        (at start (robot_free ?r))
        (at start (stove_free ?z))
        (at start (is_cooking_zone ?z))
        (over all (robot_at ?r ?z))
        (at start (>= (battery ?r) 10))
        (at start (is_water ?i1))
        (at start (is_pasta ?i2))
        (at start (is_oil ?i3))
        (at start (is_salt ?i4))
        (at start (>= (stock ?i1 ?z) 1))
        (at start (>= (stock ?i2 ?z) 1))
        (at start (>= (stock ?i3 ?z) 1))
        (at start (>= (stock ?i4 ?z) 1))
        (at start (is_spaghetti ?d))
    )
    :effect (and
        (at start (not (robot_free ?r)))
        (at start (not (stove_free ?z)))
        (at start (decrease (battery ?r) 10))
        (at start (decrease (stock ?i1 ?z) 1))
        (at start (decrease (stock ?i2 ?z) 1))
        (at start (decrease (stock ?i3 ?z) 1))
        (at start (decrease (stock ?i4 ?z) 1))
        (at end (dish_prepared ?d))
        (at end (stove_free ?z))
        (at end (robot_free ?r))
    )
)

);; end Domain ;;;;;;;;;;;;;;;;;;;;;;;;
