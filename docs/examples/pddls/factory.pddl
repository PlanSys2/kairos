( define ( domain factory )
( :requirements :strips :adl :typing :durative-actions :fluents )
( :types
	robot - object
	zone - object
	piece - object
	car - object
)
( :predicates
	( robot_available ?robot0 - robot )
	( robot_at ?robot0 - robot ?zone1 - zone )
	( piece_at ?piece0 - piece ?zone1 - zone )
	( piece_is_wheel ?piece0 - piece )
	( piece_is_body_car ?piece0 - piece )
	( piece_is_steering_wheel ?piece0 - piece )
	( is_assembly_zone ?zone0 - zone )
	( assembly_zone_free ?zone0 - zone )
	( car_assembled ?car0 - car )
	( car_at ?car0 - car ?zone1 - zone )
)
( :durative-action move
  :parameters ( ?robot0 - robot ?zone1 - zone ?zone2 - zone )
  :duration ( = ?duration 5 )
  :condition
	( and
		( at start ( robot_available ?robot0 ) )
		( at start ( robot_at ?robot0 ?zone1 ) )
	)
  :effect
	( and
		( at start ( not ( robot_at ?robot0 ?zone1 ) ) )
		( at start ( not ( robot_available ?robot0 ) ) )
		( at end ( robot_at ?robot0 ?zone2 ) )
		( at end ( robot_available ?robot0 ) )
	)
)
; Carrying a piece is slower than moving empty
( :durative-action transport
  :parameters ( ?robot0 - robot ?piece1 - piece ?zone2 - zone ?zone3 - zone )
  :duration ( = ?duration 8 )
  :condition
	( and
		( at start ( robot_available ?robot0 ) )
		( at start ( robot_at ?robot0 ?zone2 ) )
		( at start ( piece_at ?piece1 ?zone2 ) )
	)
  :effect
	( and
		( at start ( not ( robot_at ?robot0 ?zone2 ) ) )
		( at start ( not ( piece_at ?piece1 ?zone2 ) ) )
		( at start ( not ( robot_available ?robot0 ) ) )
		( at end ( robot_at ?robot0 ?zone3 ) )
		( at end ( piece_at ?piece1 ?zone3 ) )
		( at end ( robot_available ?robot0 ) )
	)
)
; The pieces become part of the car: they leave the zone for good. One car at a time per
; assembly zone, and the robot stays there until the car is done.
( :durative-action assemble
  :parameters ( ?robot0 - robot ?zone1 - zone ?piece2 - piece ?piece3 - piece ?piece4 - piece ?car5 - car )
  :duration ( = ?duration 5 )
  :condition
	( and
		( at start ( robot_available ?robot0 ) )
		( at start ( is_assembly_zone ?zone1 ) )
		( at start ( assembly_zone_free ?zone1 ) )
		( over all ( robot_at ?robot0 ?zone1 ) )
		( at start ( piece_at ?piece2 ?zone1 ) )
		( at start ( piece_at ?piece3 ?zone1 ) )
		( at start ( piece_at ?piece4 ?zone1 ) )
		( at start ( piece_is_wheel ?piece2 ) )
		( at start ( piece_is_body_car ?piece3 ) )
		( at start ( piece_is_steering_wheel ?piece4 ) )
	)
  :effect
	( and
		( at start ( not ( piece_at ?piece2 ?zone1 ) ) )
		( at start ( not ( piece_at ?piece3 ?zone1 ) ) )
		( at start ( not ( piece_at ?piece4 ?zone1 ) ) )
		( at start ( not ( robot_available ?robot0 ) ) )
		( at start ( not ( assembly_zone_free ?zone1 ) ) )
		( at end ( car_assembled ?car5 ) )
		( at end ( car_at ?car5 ?zone1 ) )
		( at end ( robot_available ?robot0 ) )
		( at end ( assembly_zone_free ?zone1 ) )
	)
)
)
