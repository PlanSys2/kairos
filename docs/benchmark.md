# Benchmark: Kairos and other planners

How Kairos compares with the planners it is meant to replace or match, on public benchmarks
and on robotics problems. The tables are regenerated as Kairos improves; each one says which
version produced it.

## Summary

Problems solved in 30 s with 3 GB per problem; every plan checked by Kairos's validator
(planners that print invalid plans do not get the problem).

| Suite | Kairos | OPTIC | TFD | POPF | LPG-td | ENHSP | Metric-FF | LAMA |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| Robotics examples (11) | **11** | 8 | 5 | 6 | 2 | | | |
| B2: required concurrency, temporal numeric (55) | **32** | 17 | 24 | 24 | 0 | | | |
| B1: temporal, IPC 2002-2014 (225) | **172** (**180** with 3 threads) | 107 | 110 | 104 | 125 | | | |
| B3: numeric, IPC 2002/2023/2026 (420) | **252** | | | | | 103 | 57 | |
| Classical, IPC 1998-2023 (220) | 120 | | | | | | | **151** |

The other planners were measured on 2026-10-11 with Kairos 0.3.0 at `023fe98`; the Kairos
column is the current version (B1: 161 at `023fe98`, 172 since `cf886fc`). The robotics row is
the 11 problems of that day; the 3 battery patrol problems added since (continuous effects) are
solved by Kairos, POPF solves the simplest and OPTIC none (see [examples.md](examples.md)). Blank cells: the
planner does not handle that kind of problem, or was not run on it.

- **Temporal planning** (B1, B2, robotics): Kairos solves the most problems and has the best
  plan quality in every suite.
- **Numeric planning** (B3): Kairos solves more than twice as many as ENHSP and Metric-FF as
  run here (both do not read part of the domains; see the notes).
- **Classical planning**: Fast Downward's LAMA solves more (barman, sokoban, parking, hiking,
  rubiks-cube); Kairos is not yet at its level there.

## Method

- **Machine**: a 16-core laptop, 60 GB of RAM, 8 planner processes at a time.
- **Limits**: 30 s of wall time and 3 GB of memory per problem (a cgroup, so no planner can
  exceed them), one CPU per planner.
- **Kairos**: the release build of the command (`kairos DOMAIN PROBLEM --timeout 30 --memory
  3072`), anytime: its result is the best plan found within the time. Its first-plan time is
  its agile time. One thread (`--threads 1`) unless a row says otherwise: the command uses up
  to 4 by default (its search phases at once), while the other planners use one core each.
- **Other planners**: planutils 0.10.25 (https://github.com/AI-Planning/planutils) in a
  container: OPTIC, TFD (Temporal Fast Downward), POPF, LPG-td (`-quality`), ENHSP-2020
  (`-planner sat-hmrphj`), Metric-FF and LAMA (Fast Downward). Their time is measured inside the
  container, without Docker's start-up. Their last plan is taken (anytime planners keep
  improving until the timeout).
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

The instances come from public collections (potassco pddl-instances, the IPC 2023 and 2026
datasets, time2processes), fetched at pinned commits by `tools/fetch_corpus.sh`.

## Details (2026-10-11, Kairos 0.3.0 at `023fe98`)

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

## Notes

- The classical suite has 10 bogus entries: in quantum-layout every problem has its own domain
  file, and the suite takes those domain files as problems. They count against every planner.
- ENHSP does not read some IPC 2002 domains ("Depot not found") and Java reaches the 3 GB limit
  in some problems; Metric-FF does not read most IPC 2023 and 2026 numeric domains. Their
  numbers are below what their best configurations reach.
- LPG-td sometimes announces a plan file it never writes; runs that fail at once are retried.
- POPF here is planutils's build; the one PlanSys2 ships may differ slightly.
- Robotics examples, per problem: see the results table of [examples.md](examples.md).

## Kairos over time

| Date | Version | B1 | B2 | B3 | Classical | What changed |
|---|---|---:|---:|---:|---:|---|
| 2026-10-11 | `023fe98` | 161 | 32 | 252 | 120 | First comparison with the planners of planutils |
| 2026-10-12 | `cf886fc` | 172 | 32 | 252 | 120 | Novelty memory cap, unit costs for the first plan of M2, FF when LAMA has no plan |
| 2026-10-12 | step 18 | 172 | 32 | 252 | 120 | Duration inequalities and continuous effects (battery patrol examples: 3 more robotics problems, all solved); cost + 1 for the first plan of temporal numeric tasks |
| 2026-10-12 | step 19, 1 thread | 172 | 32 | 252 | 120 | Phases of the search in parallel; with one thread nothing changes |
| 2026-10-12 | step 19, 3 threads | **180** | 32 | | | The phases at once, each with the whole time: 8 more temporal problems, better plans (quality 166 -> 180), first plans in B2 much sooner (agile 19 -> 31) |
