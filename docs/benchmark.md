# Benchmark: Kairos and other planners

How Kairos compares with the planners it is meant to replace or match, on public benchmarks
and on robotics problems. The tables are regenerated as Kairos improves; each one says which
version produced it.

## Summary

Kairos is meant to answer in milliseconds, so each suite counts the problems solved within
**100 ms**, **1 s** and the whole limit (30 s and 3 GB per problem; B6: 60 s and 2 GB), all
planners under the same limits. Every plan is checked by Kairos's validator: a planner that
prints an invalid plan does not get the problem.

The short columns need the time of the first plan. Kairos reports it; planners that stop at
their first plan take that long (the agile configurations of the IPC 2023 winners, lama-first,
POPF, ENHSP, Metric-FF, LNM-Plan, DecStar, Dual-BFWS). Anytime runs (LAMA, OPTIC, TFD, LPG-td,
NextFLAP, the satisficing IPC 2023 configurations) only show when they stopped: "-".

### Classical (220)

| Planner | 100 ms | 1 s | whole limit |
|---|---:|---:|---:|
| **Kairos** | 67 | 103 | 153 |
| LAMA (anytime) | - | - | 151 |
| lama-first | 0 | 68 | 150 |
| Scorpion Maidu | - | - | 145 |
| Scorpion Maidu agile | 0 | 65 | 132 |
| Levitron | - | - | 146 |
| Levitron agile | 0 | 72 | 131 |
| FD Stone Soup 2023 | - | - | 147 |
| FD Stone Soup 2023 agile | 0 | 66 | 132 |
| DecStar 2023 (agile) | 0 | 67 | 136 |
| Dual-BFWS | 51 | 78 | 123 |

### B3: numeric (420)

| Planner | 100 ms | 1 s | whole limit |
|---|---:|---:|---:|
| **Kairos** | 152 | 203 | 289 |
| ENHSP (current) | 0 | 128 | 208 |
| ENHSP-2020 | 0 | 41 | 103 |
| LNM-Plan | 22 | 69 | 117 |
| Metric-FF | 46 | 51 | 57 |

### B1: temporal (225)

| Planner | 100 ms | 1 s | whole limit |
|---|---:|---:|---:|
| **Kairos** | 101 | 146 | 178 |
| POPF | 53 | 84 | 104 |
| OPTIC | - | - | 107 |
| TFD | - | - | 110 |
| LPG-td | - | - | 125 |
| NextFLAP | - | - | 80 |

### B2: required concurrency, temporal numeric (55)

| Planner | 100 ms | 1 s | whole limit |
|---|---:|---:|---:|
| **Kairos** | 25 | 32 | 33 |
| POPF | 14 | 19 | 24 |
| OPTIC | - | - | 17 |
| TFD | - | - | 24 |
| LPG-td | - | - | 0 |
| NextFLAP | - | - | 2 |

### Robotics examples

| Planner | 100 ms | 1 s | whole limit |
|---|---:|---:|---:|
| **Kairos** | 13 | 14 | 14 |
| POPF | 4 | 5 | 6 (of 11) |
| OPTIC | - | - | 8 (of 11) |
| TFD | - | - | 5 (of 11) |
| LPG-td | - | - | 2 (of 11) |
| NextFLAP | - | - | 4 |

### PDDL3: IPC 2006 preferences and constraints (125)

| Planner | 100 ms | 1 s | whole limit |
|---|---:|---:|---:|
| **Kairos** | 58 | 67 | 84 |
| POPF | 13 | 15 | 16 |
| OPTIC | - | - | 45 |
| NextFLAP | - | - | 10 |

### B6: PlanSys2 problems at scale (144; 60 s, 2 GB)

| Planner | 100 ms | 1 s | whole limit |
|---|---:|---:|---:|
| **Kairos** | 60 | 77 | 92 |
| POPF | 23 | 45 | 57 |
| OPTIC | - | - | 57 |
| TFD | - | - | 58 |
| TAMER (unified-planning) | 0 | 0 | 21 |
| Aries (unified-planning) | - | - | 9 |

- **Classical**: Kairos solves 153, more than LAMA (151) and than the winners of the IPC 2023
  satisficing track (Scorpion Maidu 145, Levitron 146, Fast Downward Stone Soup 2023 147) with
  the same 30 s, and the most within 100 ms (67; Dual-BFWS 51, the others none) and 1 s (103;
  the best of the others 78).
- **Numeric** (B3): Kairos 289, the current ENHSP 208 and LNM-Plan (IPC 2023 numeric winner) 117.
- **Temporal** (B1, B2, robotics, PDDL3): Kairos solves the most at every limit.
- **PlanSys2 problems at scale** (B6): Kairos 92 of 144, the others at most 58. Kairos keeps the
  2 GB limit from the start of grounding: the largest patrol problems (537 680 actions) need
  more than that just to be prepared.

## Method

- **Machine**: a 16-core laptop, 60 GB of RAM, 8 planner processes at a time (measured on
  2026-10-13; the details below are older).
- **Limits**: 30 s of wall time and 3 GB of memory per problem (a cgroup, so no planner can
  exceed them), one CPU per planner.
- **Kairos**: the release build of the command (`kairos DOMAIN PROBLEM --timeout 30 --memory
  3072`), anytime: its result is the best plan found within the time. Its first-plan time is
  its agile time. One thread (`--threads 1`) unless a row says otherwise: the command uses up
  to 4 by default (its search phases at once), while the other planners use one core each.
- **Other planners**: planutils 0.10.25 (https://github.com/AI-Planning/planutils) in a
  container: OPTIC, TFD (Temporal Fast Downward), POPF, LPG-td (`-quality`), ENHSP-2020
  (`-planner sat-hmrphj`), Metric-FF and LAMA (Fast Downward). Built from source at pinned
  commits, in release mode (the workspace's `bench/sota/Dockerfile`): the IPC 2023 classical
  winners as submitted (Scorpion Maidu, Levitron, Fast Downward Stone Soup 2023, DecStar 2023,
  with their satisficing and agile configurations), lama-first, Dual-BFWS (LAPKT), LNM-Plan
  (IPC 2023 numeric winner), the current ENHSP and NextFLAP. Their time is measured inside the
  container, without Docker's start-up. Their last plan is taken (anytime planners keep
  improving until the timeout).
- **Packaging is not the planner's**: planutils runs POPF and ENHSP-2020 in Apptainer, started
  cold for each problem; their times lose 0.189 s (POPF's whole time on a one-action problem,
  the most favourable reading) and 0.084 s (Apptainer's cold start; the JVM is ENHSP's). Kairos's
  first-plan time leaves out its process start (about 5 ms).
- **Validation**: every plan of every planner is checked by `kairos validate` against the
  original domain and problem; an invalid plan counts as not solved and is listed.
- **Quality**: the IPC score. For each problem, the best cost among the planners that solved it
  divided by the planner's own cost (1 for the best plan, 0 if not solved), summed over the
  problems. The cost is the metric of the problem, or the makespan, or the number of actions.
- **Agile score** (only for planners that stop at their first plan): 1 if solved within 1 s,
  else 1 - log(t) / log(30), as in the IPC agile track.

## Suites

| Suite | Problems | What it is |
|---|---:|---|
| Robotics examples | 11 | Problems of [examples.md](examples.md): robots that cook with consumable ingredients and battery, cooking with ingredient costs, a car factory, PlanSys2 examples (the battery patrol problems, with continuous effects, came later: see examples.md) |
| B1 | 225 | Temporal domains of the IPC 2002 to 2014, 5 instances per domain (from easy to hard) |
| B2 | 55 | Domains that need concurrent actions (match-cellar, turn-and-open, temporal-machine-shop), time windows compiled away, and temporal numeric domains of time2processes, 5 each |
| B3 | 420 | Numeric domains of the IPC 2002, 2023 and 2026, 10 instances per domain |
| Classical | 220 | Classical domains of the IPC 1998 to 2023, 10 instances per domain |
| B6 | 144 | PlanSys2 domains with problems scaled from 10 to 20 000 objects (below) |
| PDDL3 | 125 | IPC 2006 domains with preferences (simple, qualitative, complex) and trajectory constraints, 5 instances each (`coverage.py` suite `prefs`) |

The instances come from public collections (potassco pddl-instances, the IPC 2023 and 2026
datasets, time2processes), fetched at pinned commits by `tools/fetch_corpus.sh`.

## Details (2026-10-11, Kairos 0.3.0 at `023fe98`; the other planners)

### docs/examples (robotics) (11 problems)

| planner | solved | quality (IPC score) | invalid plans |
|---|---:|---:|---:|
| kairos | 11 | 9.2 | 0 |
| lpg-td | 2 | 2.0 | 1 |
| optic | 8 | 8.0 | 0 |
| popf | 6 | 5.4 | 1 |
| tfd | 5 | 3.8 | 0 |

### B2: required concurrency, temporal numeric (55 problems)

| planner | solved | quality (IPC score) | invalid plans |
|---|---:|---:|---:|
| kairos | 32 | 30.6 | 0 |
| lpg-td | 0 | 0.0 | 0 |
| optic | 17 | 16.6 | 0 |
| popf | 24 | 23.6 | 5 |
| tfd | 24 | 22.0 | 0 |

Domains where another planner solves more than Kairos:

| domain | Kairos | better |
|---|---:|---|
| ipc-2004/airport-temporal-time-windows-compiled-strips | 2 | popf 3 |
| ipc-2011/temporal-machine-shop-temporal-satisficing | 0 | popf 1 |
| ipc-2011/turn-and-open-temporal-satisficing | 2 | tfd 5 |
| ipc-2014/turn-and-open-temporal-satisficing | 2 | tfd 3 |

### B1: temporal, IPC 2002-2014 (225 problems)

| planner | solved | quality (IPC score) | invalid plans |
|---|---:|---:|---:|
| kairos | 161 | 140.0 | 0 |
| lpg-td | 125 | 109.6 | 14 |
| optic | 107 | 99.1 | 2 |
| popf | 104 | 91.2 | 2 |
| tfd | 110 | 80.2 | 1 |

Domains where another planner solves more than Kairos:

| domain | Kairos | better |
|---|---:|---|
| ipc-2004/airport-temporal-strips | 4 | tfd 5 |
| ipc-2004/satellite-time-time-windows-strips | 1 | lpg-td 3, optic 2 |
| ipc-2006/pathways-metric-time | 2 | lpg-td 3 |
| ipc-2006/storage-time | 3 | lpg-td 4 |
| ipc-2006/trucks-time-constraints-timed-initial-literals | 1 | lpg-td 2, optic 2, popf 2 |
| ipc-2008/elevator-temporal-satisficing-strips | 4 | tfd 5 |
| ipc-2008/parc-printer-temporal-satisficing-strips | 2 | tfd 4 |
| ipc-2011/floor-tile-temporal-satisficing | 3 | lpg-td 4 |
| ipc-2011/parc-printer-temporal-satisficing | 0 | tfd 3 |
| ipc-2011/parking-temporal-satisficing | 4 | popf 5 |
| ipc-2014/floor-tile-temporal-satisficing | 0 | lpg-td 5 |
| ipc-2014/parking-temporal-satisficing | 3 | lpg-td 5, tfd 5, optic 4 |
| ipc-2014/satellite-temporal-satisficing | 4 | lpg-td 5 |

### B3: numeric, IPC 2002/2023/2026 (420 problems)

| planner | solved | quality (IPC score) | invalid plans |
|---|---:|---:|---:|
| kairos | 252 | 249.3 | 0 |
| enhsp-2020 | 103 | 89.2 | 28 |
| metric-ff | 57 | 49.5 | 10 |

Domains where another planner solves more than Kairos:

| domain | Kairos | better |
|---|---:|---|
| ipc-2002/depots-numeric-automatic | 2 | metric-ff 9 |
| ipc-2002/driverlog-numeric-automatic | 9 | enhsp-2020 10 |
| ipc-2002/settlers-numeric-automatic | 0 | metric-ff 3 |

### classical, IPC 1998-2023 (220 problems)

| planner | solved | quality (IPC score) | invalid plans |
|---|---:|---:|---:|
| kairos | 120 | 113.5 | 10 |
| lama | 151 | 148.9 | 0 |

Domains where another planner solves more than Kairos:

| domain | Kairos | better |
|---|---:|---|
| ipc-2000/blocks-strips-typed | 6 | lama 8 |
| ipc-2002/depots-strips-automatic | 8 | lama 9 |
| ipc-2008/sokoban-sequential-satisficing-strips | 3 | lama 9 |
| ipc-2011/barman-sequential-satisficing | 0 | lama 10 |
| ipc-2011/parking-sequential-satisficing | 7 | lama 10 |
| ipc-2014/hiking-sequential-satisficing | 6 | lama 10 |
| ipc2023/folding | 1 | lama 3 |
| ipc2023/recharging-robots | 0 | lama 2 |
| ipc2023/ricochet-robots | 0 | lama 1 |
| ipc2023/rubiks-cube | 6 | lama 10 |

## B6: PlanSys2 problems at scale (2026-10-12, Kairos 0.4.0)

How each planner copes as PlanSys2 problems grow. Eight domains of PlanSys2's packages, used
unchanged: `patrol`, `simple` (with battery), `charging` and `road_trip` (energy and fuel as
numeric fluents), `elevator`, `cooking`, `factory` (classical) and `suave` (derived predicates).
A generator writes problems at six levels, by number of objects: XS ~10, S ~50, M ~200,
L ~1000, XL ~5000 and XXL ~20 000, 3 seeds per level (144 problems). Goals grow with the
objects: places to patrol, passengers, dishes, cars, pipelines. Every problem comes with a
reference plan built along with it, so all are solvable (the reference plans are validated
too).

Limits: 60 s and 2 GB per problem, one CPU per planner, plans validated by Kairos. Kairos runs
for its first plan (`--no-anytime`), one thread. TAMER and Aries are those
of unified-planning (`up-tamer` 1.1.6, `up-aries` 0.5.0).

| level | problems | Kairos | OPTIC | TFD | POPF | TAMER | Aries |
|---|---|---|---|---|---|---|---|
| XS | 24 | 24 | 18 | 18 | 18 | 12 | 6 |
| S | 24 | 24 | 18 | 18 | 18 | 6 | 3 |
| M | 24 | 21 | 13 | 15 | 13 | 3 | 0 |
| L | 24 | 17 | 6 | 7 | 6 | 0 | 0 |
| XL | 24 | 9 | 2 | 0 | 2 | 0 | 0 |
| XXL | 24 | 0 | 0 | 0 | 0 | 0 | 0 |
| **total** | **144** | **95** | **57** | **58** | **57** | **21** | **9** |

- Kairos solves every problem up to S and most up to L, and is the only planner with plans at
  L and XL in most families. The others fail on suave (derived predicates) at every size, and
  OPTIC, TFD and POPF on factory from XS (its `assemble` takes any three pieces).
- What stops Kairos: instantiation out of memory (factory from M: any wheel, body and steering
  wheel for any car is millions of actions; everything at XXL), energy and fuel in charging and
  road_trip from L (a robot left without charge far from a charger), and elevator from L (one
  lift for ~1000 passengers: solved at L in about 50 s, out of memory at XL).
- Targets of the plan: first plan at L in under 10 s and at XL in under 60 s with 2 GB. Met at L
  in every family Kairos solves there but elevator (median 0.1-0.6 s: patrol, simple, cooking,
  suave and 2 of 3 charging; elevator 51 s), and at XL for patrol (4 s), cooking (2.9 s) and
  simple (8 s). These come from the incremental goals (one goal at a time when there
  are many); before them, patrol L had no plan in 100 s.

Per family (solved of 3, and Kairos's median time of the solved ones and its peak memory):

| family | level | Kairos | OPTIC | TFD | POPF | TAMER | Aries | Kairos time (median s) | Kairos peak MB |
|---|---|---|---|---|---|---|---|---|---|
| charging | XS | 3/3 | 3/3 | 3/3 | 3/3 | 3/3 | 0/3 | 0.02 | 7 |
| charging | S | 3/3 | 3/3 | 3/3 | 3/3 | 3/3 | 0/3 | 0.07 | 8 |
| charging | M | 3/3 | 0/3 | 3/3 | 0/3 | 0/3 | 0/3 | 0.14 | 13 |
| charging | L | 2/3 | 0/3 | 0/3 | 0/3 | 0/3 | 0/3 | 0.49 | 102 |
| charging | XL | 0/3 | 0/3 | 0/3 | 0/3 | 0/3 | 0/3 | - | 1445 |
| charging | XXL | 0/3 | 0/3 | 0/3 | 0/3 | 0/3 | 0/3 | - | 1078 |
| cooking | XS | 3/3 | 3/3 | 3/3 | 3/3 | 0/3 | 0/3 | 0.03 | 7 |
| cooking | S | 3/3 | 3/3 | 3/3 | 3/3 | 0/3 | 0/3 | 0.03 | 7 |
| cooking | M | 3/3 | 3/3 | 3/3 | 3/3 | 0/3 | 0/3 | 0.03 | 8 |
| cooking | L | 3/3 | 3/3 | 3/3 | 3/3 | 0/3 | 0/3 | 0.13 | 13 |
| cooking | XL | 3/3 | 0/3 | 0/3 | 0/3 | 0/3 | 0/3 | 2.91 | 78 |
| cooking | XXL | 0/3 | 0/3 | 0/3 | 0/3 | 0/3 | 0/3 | - | - |
| elevator | XS | 3/3 | 3/3 | 3/3 | 3/3 | 0/3 | 0/3 | 0.03 | 6 |
| elevator | S | 3/3 | 3/3 | 3/3 | 3/3 | 0/3 | 0/3 | 0.03 | 9 |
| elevator | M | 3/3 | 3/3 | 0/3 | 3/3 | 0/3 | 0/3 | 0.79 | 26 |
| elevator | L | 3/3 | 0/3 | 0/3 | 0/3 | 0/3 | 0/3 | 50.79 | 219 |
| elevator | XL | 0/3 | 0/3 | 0/3 | 0/3 | 0/3 | 0/3 | - | 717 |
| elevator | XXL | 0/3 | 0/3 | 0/3 | 0/3 | 0/3 | 0/3 | - | - |
| factory | XS | 3/3 | 0/3 | 0/3 | 0/3 | 3/3 | 3/3 | 0.03 | 6 |
| factory | S | 3/3 | 0/3 | 0/3 | 0/3 | 0/3 | 0/3 | 0.48 | 37 |
| factory | M | 0/3 | 0/3 | 0/3 | 0/3 | 0/3 | 0/3 | - | 784 |
| factory | L | 0/3 | 0/3 | 0/3 | 0/3 | 0/3 | 0/3 | - | - |
| factory | XL | 0/3 | 0/3 | 0/3 | 0/3 | 0/3 | 0/3 | - | - |
| factory | XXL | 0/3 | 0/3 | 0/3 | 0/3 | 0/3 | 0/3 | - | - |
| patrol | XS | 3/3 | 3/3 | 3/3 | 3/3 | 3/3 | 3/3 | 0.03 | 6 |
| patrol | S | 3/3 | 3/3 | 3/3 | 3/3 | 0/3 | 3/3 | 0.03 | 7 |
| patrol | M | 3/3 | 3/3 | 3/3 | 3/3 | 0/3 | 0/3 | 0.03 | 12 |
| patrol | L | 3/3 | 0/3 | 1/3 | 0/3 | 0/3 | 0/3 | 0.17 | 77 |
| patrol | XL | 3/3 | 0/3 | 0/3 | 0/3 | 0/3 | 0/3 | 4.01 | 1369 |
| patrol | XXL | 0/3 | 0/3 | 0/3 | 0/3 | 0/3 | 0/3 | - | 439 |
| road_trip | XS | 3/3 | 3/3 | 3/3 | 3/3 | 0/3 | 0/3 | 0.02 | 7 |
| road_trip | S | 3/3 | 3/3 | 3/3 | 3/3 | 0/3 | 0/3 | 0.03 | 8 |
| road_trip | M | 3/3 | 1/3 | 3/3 | 1/3 | 0/3 | 0/3 | 0.11 | 13 |
| road_trip | L | 0/3 | 0/3 | 0/3 | 0/3 | 0/3 | 0/3 | - | 92 |
| road_trip | XL | 0/3 | 0/3 | 0/3 | 0/3 | 0/3 | 0/3 | - | 835 |
| road_trip | XXL | 0/3 | 0/3 | 0/3 | 0/3 | 0/3 | 0/3 | - | 1237 |
| simple | XS | 3/3 | 3/3 | 3/3 | 3/3 | 3/3 | 0/3 | 0.03 | 6 |
| simple | S | 3/3 | 3/3 | 3/3 | 3/3 | 3/3 | 0/3 | 0.03 | 7 |
| simple | M | 3/3 | 3/3 | 3/3 | 3/3 | 3/3 | 0/3 | 0.03 | 9 |
| simple | L | 3/3 | 3/3 | 3/3 | 3/3 | 0/3 | 0/3 | 0.07 | 35 |
| simple | XL | 3/3 | 2/3 | 0/3 | 2/3 | 0/3 | 0/3 | 8.32 | 620 |
| simple | XXL | 0/3 | 0/3 | 0/3 | 0/3 | 0/3 | 0/3 | - | 596 |
| suave | XS | 3/3 | 0/3 | 0/3 | 0/3 | 0/3 | 0/3 | 0.03 | 7 |
| suave | S | 3/3 | 0/3 | 0/3 | 0/3 | 0/3 | 0/3 | 0.03 | 7 |
| suave | M | 3/3 | 0/3 | 0/3 | 0/3 | 0/3 | 0/3 | 0.05 | 8 |
| suave | L | 3/3 | 0/3 | 0/3 | 0/3 | 0/3 | 0/3 | 0.60 | 14 |
| suave | XL | 0/3 | 0/3 | 0/3 | 0/3 | 0/3 | 0/3 | - | 543 |
| suave | XXL | 0/3 | 0/3 | 0/3 | 0/3 | 0/3 | 0/3 | - | 965 |

The generator, the reference plans and the runners are in the workspace's `bench/b6`
(`gen.py`, `check.py`, `run_others.sh`, `report.py`).

## Notes

- The classical suite has 10 bogus entries: in quantum-layout every problem has its own domain
  file, and the suite takes those domain files as problems. They count against every planner.
- ENHSP does not read some IPC 2002 domains ("Depot not found") and Java reaches the 3 GB limit
  in some problems; Metric-FF does not read most IPC 2023 and 2026 numeric domains. Their
  numbers are below what their best configurations reach.
- LPG-td sometimes announces a plan file it never writes; runs that fail at once are retried.
- POPF here is planutils's build; the one PlanSys2 ships may differ slightly.
- Robotics examples, per problem: see the results table of [examples.md](examples.md).
