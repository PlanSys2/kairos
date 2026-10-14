# Examples

A set of PDDL domains to learn Kairos with, in [examples/pddls](examples/pddls) (`docs/examples/pddls` in the repository). Most come
from ROS 2 robotics (PlanSys2 tests and tutorials) and are temporal: durative actions, numeric
fluents, resources that run out, metrics. They are small enough to read in a few minutes and to
try options on.

Every command below is run from `docs/examples/pddls`. `kairos DOMAIN PROBLEM` plans for 30
seconds by default, printing every better plan it finds (anytime); add `--no-anytime` to stop at
the first one, `-v` to see progress and the time of the first plan, and validate a plan with
`kairos validate DOMAIN PROBLEM PLAN`. See [cli.md](cli.md) for every option.

- [Cooking](#cooking)
- [Cooking with costs](#cooking-with-costs)
- [Factory](#factory)
- [Battery patrol](#battery-patrol)
- [Elevators](#elevators)
- [Road trip](#road-trip)
- [SUAVE](#suave)
- [Simple (talk to a person)](#simple-talk-to-a-person)
- [Domains without a problem](#domains-without-a-problem)
- [Results](#results)

## Cooking

[cooking_domain.pddl](examples/pddls/cooking_domain.pddl). Robots carry ingredients from the
fridge, pantry and water tap to kitchens, one unit per trip, and cook omelettes (egg, oil,
salt), cakes (flour, egg, sugar) and spaghetti (water, pasta, oil, salt).

- Types: `robot`, `zone`, `ingredient`, `dish`. Actions (all durative): `move` (5),
  `transport` (8), `recharge` (10), `cook_omelette` (10), `cook_cake` (20),
  `cook_spaghetti` (15).
- Numeric fluents as consumable resources: `(stock ?i ?z)` counts units of an ingredient in a
  zone; `transport` moves one unit, cooking consumes one of each. `(battery ?r)` drops with every
  action and only `recharge` (in the recharge zone, `assign` to 100) restores it.
- Mutual exclusion through predicates: a robot does one thing at a time (`robot_free`) and a
  kitchen cooks one dish at a time (`stove_free`). With several robots and kitchens, a good plan
  runs them in parallel.

| Problem | Size |
|---------|------|
| [cooking_problem_simple](examples/pddls/cooking_problem_simple.pddl) | 1 robot, 1 kitchen, 1 omelette; battery at 30, so it must recharge on the way |
| [cooking_problem_long](examples/pddls/cooking_problem_long.pddl) | 2 robots, 2 kitchens, 6 dishes (2 of each) |
| [cooking_problem_huge](examples/pddls/cooking_problem_huge.pddl) | 4 robots, 3 kitchens, 18 dishes: about 60 units to carry and many recharges |

```bash
kairos cooking_domain.pddl cooking_problem_simple.pddl
kairos cooking_domain.pddl cooking_problem_long.pddl
kairos cooking_domain.pddl cooking_problem_huge.pddl
```

## Cooking with costs

[cooking_cost_domain.pddl](examples/pddls/cooking_cost_domain.pddl). The cooking domain without
battery, but with flexible recipes: each dish has fixed ingredients and slots that several
ingredients can fill (fat: oil or butter; seasoning: salt, pepper or herbs; sweetener: sugar or
honey; sauce: tomato, pesto or cream). Every ingredient has a `(price ?i)` and cooking adds the
prices of what it uses to `(total-cost)`.

- The problems say `(:metric minimize (total-cost))`: the cheapest plan, not the shortest, is
  the goal. The cheap ingredients are scarce, so the planner has to decide which dish gets them.
  Anytime search shows up here: the first plan is usually not the cheapest, and later plans
  improve it.
- Each problem states its minimum cost in a comment, so you can check how close a plan is.

| Problem | Size | Minimum cost |
|---------|------|-------------:|
| [cooking_cost_problem_simple](examples/pddls/cooking_cost_problem_simple.pddl) | 1 robot, 1 kitchen, 1 omelette | 3 |
| [cooking_cost_problem_medium](examples/pddls/cooking_cost_problem_medium.pddl) | 1 robot, omelette + cake + spaghetti; one unit of oil, salt and sugar | 15 |
| [cooking_cost_problem_long](examples/pddls/cooking_cost_problem_long.pddl) | 2 robots, 2 kitchens, 6 dishes; oil, salt, sugar and tomato run short | 34 |

```bash
kairos cooking_cost_domain.pddl cooking_cost_problem_simple.pddl
kairos cooking_cost_domain.pddl cooking_cost_problem_medium.pddl
kairos cooking_cost_domain.pddl cooking_cost_problem_long.pddl --target-cost 34   # stop at the optimum
```

## Factory

[factory.pddl](examples/pddls/factory.pddl). Robots carry pieces (wheel, body, steering wheel)
to an assembly zone and assemble cars there.

- Types: `robot`, `zone`, `piece`, `car`. Durative actions: `move` (5), `transport` (8, slower
  than moving empty), `assemble` (5).
- `assemble` consumes the three pieces (they leave the zone for good), needs the robot in the
  zone `over all` the action, and takes the zone (`assembly_zone_free`), so cars are assembled
  one at a time while pieces are carried in parallel by the other robots.

| Problem | Size |
|---------|------|
| [factory_problem_1](examples/pddls/factory_problem_1.pddl) | 3 robots, 9 pieces, 3 cars |

```bash
kairos factory.pddl factory_problem_1.pddl
```

`factory2.pddl`, `factory3.pddl` and `factory4.pddl` are earlier variants, see
[Domains without a problem](#domains-without-a-problem).

## Battery patrol

[battery_domain.pddl](examples/pddls/battery_domain.pddl). Robots visit waypoints with a
battery that drains while they move and charges while they recharge.

- Types: `robot`, `waypoint`. Durative actions: `move` (distance / speed) and `recharge`.
- **Continuous effects**: `(decrease (battery ?r) (* #t (discharge_rate ?r)))` while moving,
  `(increase (battery ?r) (* #t (charge_rate ?r)))` while recharging; `move` needs
  `(over all (>= (battery ?r) 0))`.
- **Duration inequality**: a recharge lasts from 1 s up to a full battery,
  `(<= ?duration (/ (- (capacity ?r) (battery ?r)) (charge_rate ?r)))`.
- How Kairos plans it: engines M2 and M3 try each recharge at its bounds (the minimum and until
  full), with each continuous change as its total at the safest instant for the conditions
  that read the battery (a drain at the start of the move, a charge at the end of the
  recharge); if that finds no plan, steps of time try durations in between (see
  [pddl-support.md](pddl-support.md)). Every plan is checked with the real continuous
  semantics.

| Problem | Size |
|---------|------|
| [battery_problem_simple](examples/pddls/battery_problem_simple.pddl) | 1 robot, 4 waypoints in a line, 50 of battery for 90 m: it must recharge first |
| [battery_problem_medium](examples/pddls/battery_problem_medium.pddl) | 2 robots, a ring of 8 waypoints, two chargers |
| [battery_problem_long](examples/pddls/battery_problem_long.pddl) | 3 robots, a 4x4 grid, chargers in three corners |

```bash
kairos battery_domain.pddl battery_problem_simple.pddl
kairos battery_domain.pddl battery_problem_long.pddl
```

## Elevators

[elevator_domain.pddl](examples/pddls/elevator_domain.pddl). Passengers ride elevators between
floors.

- Types: `elevator`, `passenger`, `num` (floors, ordered with `next`). Durative actions:
  `move-up`, `move-down`, `board`, `leave`.
- Durations come from numeric fluents: a move lasts `floor_distance / elevator_speed`, boarding
  and leaving last `person_speed`. Boarding and leaving need the elevator at the floor
  `over all` the action, so the elevator must wait for the passenger.
- The problem minimizes `(total-time)`: the makespan, so using both elevators at once pays off.

| Problem | Size |
|---------|------|
| [elevator_problem](examples/pddls/elevator_problem.pddl) | 5 floors, 2 elevators, 3 passengers to floor 1 |

```bash
kairos elevator_domain.pddl elevator_problem.pddl
```

## Road trip

[road_trip_domain.pddl](examples/pddls/road_trip_domain.pddl). A car visits waypoints and
refuels at stations.

- Types: `car`, `waypoint`. Durative actions: `move`, `visit`, `refuel`.
- Fuel is a continuous resource: `move` needs `fuel_level > min_fuel_level + distance/range`
  and spends `distance/range`. Durations depend on the state: `move` lasts `distance/speed`,
  and `refuel` lasts `(1 - fuel_level) / fuel_rate`, so the emptier the tank, the longer it
  takes (`variable-duration` in `kairos check`).

| Problem | Size |
|---------|------|
| [road_trip_problem](examples/pddls/road_trip_problem.pddl) | 1 car, 4 waypoints and 2 fuel stations, all connected; visit 3 waypoints |

```bash
kairos road_trip_domain.pddl road_trip_problem.pddl
```

## SUAVE

[suave_domain.pddl](examples/pddls/suave_domain.pddl). From
[SUAVE](https://github.com/kas-lab/suave), a self-adaptive underwater vehicle: it searches for a
pipeline and inspects it, reconfiguring its function designs (search spiral, thrusters) first.

- Classical (no durations): `start_robot`, `reconfigure1`, `reconfigure2`, `search_pipeline`,
  `inspect_pipeline`.
- The interesting part is the 55 derived predicates (`:derived`), a knowledge base translated
  from an ontology: which function designs are realisable, which has better utility, which
  functions are active. Preconditions use `exists`, `or`, negation and equality over them.
- Object names such as `0.0_decimal` start with a digit, which strict PDDL does not allow;
  Kairos accepts them.

| Problem | Size |
|---------|------|
| [suave_problem](examples/pddls/suave_problem.pddl) | 1 robot, 1 pipeline: start, search, inspect |

```bash
kairos suave_domain.pddl suave_problem.pddl
```

## Simple (talk to a person)

[domain_simple.pddl](examples/pddls/domain_simple.pddl). A robot moves between rooms, approaches
a person and talks to them. Three durative actions of 5 time units: `move`, `approach` (robot
and person in the same room `over all`), `talk` (robot near the person `over all`). Its
problems are tests of PlanSys2 and include two that are wrong on purpose:

| Problem | What it tests |
|---------|---------------|
| [domain_simple_problem_1](examples/pddls/domain_simple_problem_1.pddl) | Solvable: move to the bedroom, approach Jack, talk |
| [domain_simple_problem_2](examples/pddls/domain_simple_problem_2.pddl) | Unsolvable: Jack is in no room, so the robot can never approach him |
| [domain_simple_problem_3](examples/pddls/domain_simple_problem_3.pddl) | Parse error: `:goa` instead of `:goal` and a missing `)` |

```bash
kairos domain_simple.pddl domain_simple_problem_1.pddl
kairos domain_simple.pddl domain_simple_problem_2.pddl     # exit code 10: unsolvable
kairos check domain_simple.pddl domain_simple_problem_3.pddl   # exit code 2: shows the errors
```

## Domains without a problem

These domains have no problem file. `kairos check DOMAIN` checks them; write a problem to plan.

| Domain | What it models |
|--------|----------------|
| [matchcellar.pddl](examples/pddls/matchcellar.pddl) | MatchCellar (IPC 2011): a fuse can only be mended while a match is lit, so `mend_fuse` must run inside `light_match`: required concurrency, which planners that only sequence actions cannot solve |
| [stn_test.pddl](examples/pddls/stn_test.pddl) | MatchCellar plus a robot with a numeric battery and a non-durative `switch_on`: mixes instantaneous and durative actions |
| [domain_charging.pddl](examples/pddls/domain_charging.pddl) | A robot patrols waypoints; moving spends `state_of_charge` proportional to the distance, `charge` restores it (`assign` 100); move duration is `distance/speed` |
| [domain_simple_2.pddl](examples/pddls/domain_simple_2.pddl) | A robot patrols connected rooms; it can only move while `battery_full` holds `over all`, and `askcharge` teleports it to a charger. [simple_example.pddl](examples/pddls/simple_example.pddl) is the same file |
| [simple_move_example.pddl](examples/pddls/simple_move_example.pddl) | Domain `replan`: one durative `move` between connected waypoints, the smallest temporal domain here |
| [factory2.pddl](examples/pddls/factory2.pddl) | Factory variant: pieces are marked used instead of removed, assembly zones are not exclusive, and `move` has an `at end` condition |
| [factory3.pddl](examples/pddls/factory3.pddl) | factory2 where moving needs `battery_full` `over all` |
| [factory4.pddl](examples/pddls/factory4.pddl) | factory3 as a classical domain (`:action`, no durations) |

`factory_problem_1.pddl` does not fit factory2, 3 or 4: it uses `assembly_zone_free`, which
only `factory.pddl` declares.

```bash
kairos check matchcellar.pddl
```

## Results

Kairos 0.3.0 with its defaults (anytime, 30 s) and POPF (the version PlanSys2 uses, 60 s), one
problem at a time with 4 GB, on a 16-core laptop. Every plan was checked with
`kairos validate`. The first-plan time is the planner's own; the best plan is the last one found
within the time.

| Problem | Kairos: first plan | Kairos: best plan | POPF |
|---------|-------------------:|-------------------|------|
| cooking_problem_simple | 5 ms | 8 actions, makespan 59 | makespan 74 |
| cooking_problem_long | 18 ms | 52 actions, makespan 343 | makespan 373 |
| cooking_problem_huge | 148 ms | 160 actions, makespan 903 | no plan |
| cooking_cost_problem_simple | 6 ms | cost 3 (the minimum) | cost 7 |
| cooking_cost_problem_medium | 7 ms | cost 15 (the minimum) | cost 24 |
| cooking_cost_problem_long | 28 ms | cost 34 (the minimum) | no plan |
| factory_problem_1 | 6 ms | 21 actions, makespan 54 | makespan 54 |
| battery_problem_simple | 1 ms | 7 actions, makespan 94 | makespan 94 (one 4 s recharge) |
| battery_problem_medium | 4 ms | makespan 199 | no plan |
| battery_problem_long | 6 ms | makespan 195 | no plan |
| elevator_problem | 1 ms | 12 actions, makespan 9 | makespan 9 |
| road_trip_problem | 2 ms | 8 actions, makespan 529.8 | invalid as printed (*) |
| suave_problem | 23 ms | 7 actions | no plan (derived predicates) |
| domain_simple_problem_1 | 1 ms | 3 actions, makespan 15 | makespan 15 |

The first plan is not always the best: in the factory it comes in 6 ms with makespan 132 (one
robot does most of the work), then 70, and the anytime search reaches 54 after about 14 s. Use
`--timeout` to choose how long to improve, or `--no-improvement` to stop when plans stop
getting better.

(*) Road trip has durations such as 121.9775. A move then ends exactly when the next visit
starts, and with times printed with 3 decimals the visit appears to start before the move ends.
Kairos prints such plans with as many decimals as they need to stay valid (6 here); POPF prints
3, so its plan fails validation.

