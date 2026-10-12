; Robots that patrol waypoints with a battery that drains while they move and charges while
; they recharge (continuous effects, #t). A recharge lasts as long as the planner wants, up
; to a full battery (a duration inequality, with ?duration deciding the charge).
(define (domain battery_patrol)
(:requirements :strips :typing :fluents :durative-actions :duration-inequalities
               :continuous-effects)

(:types robot waypoint)

(:predicates
  (robot_at ?r - robot ?w - waypoint)
  (connected ?a ?b - waypoint)
  (charger_at ?w - waypoint)
  (visited ?w - waypoint)
  (robot_free ?r - robot)          ; not moving nor recharging
)

(:functions
  (battery ?r - robot)             ; 0..capacity
  (capacity ?r - robot)
  (discharge_rate ?r - robot)      ; per second while moving
  (charge_rate ?r - robot)         ; per second while recharging
  (distance ?a ?b - waypoint)      ; metres
  (speed ?r - robot)               ; metres per second
)

(:durative-action move
  :parameters (?r - robot ?from ?to - waypoint)
  :duration (= ?duration (/ (distance ?from ?to) (speed ?r)))
  :condition (and
    (at start (robot_at ?r ?from))
    (at start (robot_free ?r))
    (over all (connected ?from ?to))
    (over all (>= (battery ?r) 0)))
  :effect (and
    (at start (not (robot_at ?r ?from)))
    (at start (not (robot_free ?r)))
    (at end (robot_at ?r ?to))
    (at end (robot_free ?r))
    (at end (visited ?to))
    (decrease (battery ?r) (* #t (discharge_rate ?r)))))

; As long as wanted, from 1 s up to a full battery
(:durative-action recharge
  :parameters (?r - robot ?w - waypoint)
  :duration (and
    (>= ?duration 1)
    (<= ?duration (/ (- (capacity ?r) (battery ?r)) (charge_rate ?r))))
  :condition (and
    (at start (robot_at ?r ?w))
    (at start (robot_free ?r))
    (over all (charger_at ?w))
    (over all (robot_at ?r ?w)))
  :effect (and
    (at start (not (robot_free ?r)))
    (at end (robot_free ?r))
    (increase (battery ?r) (* #t (charge_rate ?r)))))
)
