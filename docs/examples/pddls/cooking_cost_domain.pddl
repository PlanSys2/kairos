; Cooking with flexible recipes and priced ingredients: each recipe has fixed ingredients and
; slots that several ingredients can fill (e.g. the fat of a cake: butter or oil). Cooking uses
; one unit of each and pays its price; the problems minimize what is spent.
(define (domain cooking_cost)
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
(robot_free ?r - robot)            ; not moving, carrying or cooking

; Fixed ingredients
(is_egg ?i - ingredient)
(is_flour ?i - ingredient)
(is_pasta ?i - ingredient)
(is_water ?i - ingredient)

; Slots with alternatives
(is_fat ?i - ingredient)           ; oil, butter...
(is_seasoning ?i - ingredient)     ; salt, pepper, herbs...
(is_sweetener ?i - ingredient)     ; sugar, honey...
(is_sauce ?i - ingredient)         ; tomato, pesto, cream...

(is_omelette ?d - dish)            ; egg + fat + seasoning
(is_cake ?d - dish)                ; flour + egg + sweetener + fat
(is_spaghetti ?d - dish)           ; pasta + water + sauce + seasoning

(is_cooking_zone ?z - zone)
(stove_free ?z - zone)             ; one dish at a time per cooking zone

(dish_prepared ?d - dish)

);; end Predicates ;;;;;;;;;;;;;;;;;;;;


;; Functions ;;;;;;;;;;;;;;;;;;;;;;;;;
(:functions

(stock ?i - ingredient ?z - zone)  ; units of an ingredient in a zone
(price ?i - ingredient)            ; cost of using one unit
(total-cost)

);; end Functions ;;;;;;;;;;;;;;;;;;;;
;; Actions ;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(:durative-action move
    :parameters (?r - robot ?z1 ?z2 - zone)
    :duration ( = ?duration 5)
    :condition (and
        (at start (robot_free ?r))
        (at start (robot_at ?r ?z1)))
    :effect (and
        (at start (not (robot_free ?r)))
        (at start (not (robot_at ?r ?z1)))
        (at end (robot_at ?r ?z2))
        (at end (robot_free ?r))
    )
)

; One unit at a time, slower than moving empty
(:durative-action transport
    :parameters (?r - robot ?i - ingredient ?z1 ?z2 - zone)
    :duration ( = ?duration 8)
    :condition (and
        (at start (robot_free ?r))
        (at start (robot_at ?r ?z1))
        (at start (>= (stock ?i ?z1) 1))
    )
    :effect (and
        (at start (not (robot_free ?r)))
        (at start (not (robot_at ?r ?z1)))
        (at start (decrease (stock ?i ?z1) 1))
        (at end (robot_at ?r ?z2))
        (at end (increase (stock ?i ?z2) 1))
        (at end (robot_free ?r))
    )
)

(:durative-action cook_omelette
    :parameters (?r - robot ?z - zone ?egg ?fat ?seasoning - ingredient ?d - dish)
    :duration ( = ?duration 10)
    :condition (and
        (at start (robot_free ?r))
        (at start (stove_free ?z))
        (at start (is_cooking_zone ?z))
        (over all (robot_at ?r ?z))
        (at start (is_omelette ?d))
        (at start (is_egg ?egg))
        (at start (is_fat ?fat))
        (at start (is_seasoning ?seasoning))
        (at start (>= (stock ?egg ?z) 1))
        (at start (>= (stock ?fat ?z) 1))
        (at start (>= (stock ?seasoning ?z) 1))
    )
    :effect (and
        (at start (not (robot_free ?r)))
        (at start (not (stove_free ?z)))
        (at start (decrease (stock ?egg ?z) 1))
        (at start (decrease (stock ?fat ?z) 1))
        (at start (decrease (stock ?seasoning ?z) 1))
        (at start (increase (total-cost) (+ (price ?egg) (+ (price ?fat) (price ?seasoning)))))
        (at end (dish_prepared ?d))
        (at end (stove_free ?z))
        (at end (robot_free ?r))
    )
)

(:durative-action cook_cake
    :parameters (?r - robot ?z - zone ?flour ?egg ?sweetener ?fat - ingredient ?d - dish)
    :duration ( = ?duration 20)
    :condition (and
        (at start (robot_free ?r))
        (at start (stove_free ?z))
        (at start (is_cooking_zone ?z))
        (over all (robot_at ?r ?z))
        (at start (is_cake ?d))
        (at start (is_flour ?flour))
        (at start (is_egg ?egg))
        (at start (is_sweetener ?sweetener))
        (at start (is_fat ?fat))
        (at start (>= (stock ?flour ?z) 1))
        (at start (>= (stock ?egg ?z) 1))
        (at start (>= (stock ?sweetener ?z) 1))
        (at start (>= (stock ?fat ?z) 1))
    )
    :effect (and
        (at start (not (robot_free ?r)))
        (at start (not (stove_free ?z)))
        (at start (decrease (stock ?flour ?z) 1))
        (at start (decrease (stock ?egg ?z) 1))
        (at start (decrease (stock ?sweetener ?z) 1))
        (at start (decrease (stock ?fat ?z) 1))
        (at start (increase (total-cost)
            (+ (+ (price ?flour) (price ?egg)) (+ (price ?sweetener) (price ?fat)))))
        (at end (dish_prepared ?d))
        (at end (stove_free ?z))
        (at end (robot_free ?r))
    )
)

(:durative-action cook_spaghetti
    :parameters (?r - robot ?z - zone ?pasta ?water ?sauce ?seasoning - ingredient ?d - dish)
    :duration ( = ?duration 15)
    :condition (and
        (at start (robot_free ?r))
        (at start (stove_free ?z))
        (at start (is_cooking_zone ?z))
        (over all (robot_at ?r ?z))
        (at start (is_spaghetti ?d))
        (at start (is_pasta ?pasta))
        (at start (is_water ?water))
        (at start (is_sauce ?sauce))
        (at start (is_seasoning ?seasoning))
        (at start (>= (stock ?pasta ?z) 1))
        (at start (>= (stock ?water ?z) 1))
        (at start (>= (stock ?sauce ?z) 1))
        (at start (>= (stock ?seasoning ?z) 1))
    )
    :effect (and
        (at start (not (robot_free ?r)))
        (at start (not (stove_free ?z)))
        (at start (decrease (stock ?pasta ?z) 1))
        (at start (decrease (stock ?water ?z) 1))
        (at start (decrease (stock ?sauce ?z) 1))
        (at start (decrease (stock ?seasoning ?z) 1))
        (at start (increase (total-cost)
            (+ (+ (price ?pasta) (price ?water)) (+ (price ?sauce) (price ?seasoning)))))
        (at end (dish_prepared ?d))
        (at end (stove_free ?z))
        (at end (robot_free ?r))
    )
)

);; end Domain ;;;;;;;;;;;;;;;;;;;;;;;;
