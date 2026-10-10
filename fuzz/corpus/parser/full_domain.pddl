; Every syntactic construct Kairos parses, in one domain. Not meant to make sense.
(define (domain Full-Syntax)
  (:requirements :strips :typing :adl :fluents :durative-actions :duration-inequalities
                 :derived-predicates :timed-initial-literals :preferences :constraints
                 :continuous-effects :object-fluents :time)
  (:types robot gripper - object
          room corridor - place
          charger - (either room corridor))
  (:constants dock - room home)
  (:predicates
    (at ?r - robot ?p - place)
    (free ?g - gripper)
    (connected ?a ?b - place)
    (charging ?r - robot)
    (reachable ?a ?b - place)
    (busy))
  (:functions
    (battery ?r - robot)
    (distance ?a ?b - place) - number
    (speed ?r - robot)
    (energy)
    (holder ?g - gripper) - robot)

  (:constraints (and
    (always (>= (energy) 0))
    (sometime (at r1 dock))
    (within 10.5 (busy))
    (at-most-once (busy))
    (sometime-after (busy) (free g1))
    (sometime-before (free g1) (busy))
    (always-within 5 (busy) (free g1))
    (hold-during 2 8 (busy))
    (hold-after 3 (busy))
    (preference p-end (at end (at r1 home)))))

  (:action move
    :parameters (?r - robot ?from ?to - place)
    :precondition (and (at ?r ?from)
                       (or (connected ?from ?to) (connected ?to ?from))
                       (not (= ?from ?to))
                       (imply (busy) (charging ?r))
                       (exists (?c - charger) (connected ?to ?c))
                       (forall (?g - gripper) (free ?g))
                       (>= (battery ?r) (* 2 (distance ?from ?to)))
                       (< (energy) 100) (<= 1 2) (> 3 -4.5)
                       (= (holder g1) ?r)
                       (preference keep-free (free g1)))
    :effect (and (not (at ?r ?from)) (at ?r ?to)
                 (decrease (battery ?r) (* 2 (distance ?from ?to)))
                 (increase (energy) (+ 1 2 3))
                 (assign (holder g1) ?r)
                 (scale-up (speed ?r) 2) (scale-down (speed ?r) 0.5)
                 (forall (?g - gripper) (when (free ?g) (not (free ?g))))))

  (:action noop
    :parameters ())

  (:durative-action drive
    :parameters (?r - robot ?from ?to - place)
    :duration (and (>= ?duration (/ (distance ?from ?to) (speed ?r)))
                   (<= ?duration (* 2 (/ (distance ?from ?to) (speed ?r)))))
    :condition (and (at start (at ?r ?from))
                    (over all (connected ?from ?to))
                    (at end (> (battery ?r) 0))
                    (at start (preference fast (> (speed ?r) 1))))
    :effect (and (at start (not (at ?r ?from)))
                 (at end (at ?r ?to))
                 (at end (decrease (battery ?r) (* ?duration 1.5)))
                 (decrease (battery ?r) (* #t 0.1))
                 (when (at start (busy)) (at end (free g1)))))

  (:durative-action charge
    :parameters (?r - robot)
    :duration (= ?duration 10)
    :condition (at start (at ?r dock))
    :effect (at end (assign (battery ?r) 100)))

  (:derived (reachable ?a ?b - place)
    (or (connected ?a ?b)
        (exists (?c - place) (and (connected ?a ?c) (reachable ?c ?b)))))

  (:process discharge
    :parameters (?r - robot)
    :precondition (charging ?r)
    :effect (increase (battery ?r) (* #t (- (^ (speed ?r) 2)))))

  (:event full
    :parameters (?r - robot)
    :precondition (and (charging ?r) (>= (battery ?r) 100))
    :effect (not (charging ?r)))

  (:constraint safe-battery
    :parameters (?r - robot)
    :condition (>= (battery ?r) 0)))
