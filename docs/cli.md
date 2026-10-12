# The `kairos` command

`kairos` plans, validates plans and checks PDDL files from the shell. It is built on the public
library only, so it behaves as the C++ API does.

- [Overview](#overview)
- [solve (the default)](#solve-the-default)
  - [Output](#output)
  - [Exit codes](#exit-codes-of-solve)
  - [Options](#options-of-solve)
  - [Engines](#engines---engine)
  - [Search presets](#search-presets---search)
  - [How a search ends](#how-a-search-ends)
- [validate](#validate)
- [check](#check)
- [--version, --help](#--version---help)
- [--completion](#--completion)
- [How arguments are parsed](#how-arguments-are-parsed)
- [Recipes](#recipes)

The samples below were produced with `kairos 0.3.0` on PDDL files from
[tests/data/plansys2](../tests/data/plansys2): the elevator domain of `plansys2_executor`
(temporal) and the suave domain (sequential), copied as `domain.pddl` and `problem.pddl`.

## Overview

```text
kairos [solve] DOMAIN PROBLEM [options]     plan; the plan goes to stdout
kairos validate DOMAIN PROBLEM PLAN [--epsilon E]
kairos check DOMAIN [PROBLEM]               check a domain, or a problem against it
kairos check --problem PROBLEM              check a problem alone
kairos --version | --help
kairos --completion bash|zsh|fish           shell completion script
```

General rules:

- **stdout** gets only results: plans (`solve`), the usage text (`--help`), the version and
  completion scripts. **stderr** gets everything else: diagnostics, progress, statistics, the
  verdicts of `check` and `validate`. Piping stdout to a file gives a clean plan.
- Diagnostics show the file, line and column, the offending source line, and a code in brackets
  (for example `[parse.unexpected_token]`).
- Exit codes are shared by all commands:

| Code | Meaning |
|-----:|---------|
| 0 | success: a plan was found, the plan is valid, the file is valid |
| 1 | internal error; also an invalid plan in `validate` (see [validate](#validate)) |
| 2 | invalid input: bad arguments, unreadable file, PDDL errors |
| 10 | unsolvable |
| 11 | timeout without a plan |
| 12 | out of memory without a plan (also: grounding too large) |
| 13 | unsupported: the task uses something Kairos cannot plan with yet |

## solve (the default)

```text
kairos [solve] DOMAIN PROBLEM [options]
```

The word `solve` is optional: when the first argument is not `solve`, `validate` or `check`, the
command is `solve`. Exactly two positional arguments are needed (the domain file, then the
problem file); otherwise the usage text goes to stderr and the exit code is 2.

```console
$ kairos domain.pddl problem.pddl        # suave: a sequential domain
; cost = 7
(start_robot bluerov)
(reconfigure1 f_maintain_motion fd_all_thrusters)
(reconfigure1 f_generate_search_path fd_spiral_high)
(search_pipeline pipeline bluerov)
(reconfigure1 f_follow_pipeline fd_follow_pipeline)
(reconfigure2 f_generate_search_path fd_spiral_high fd_unground)
(inspect_pipeline pipeline bluerov)
```

What `solve` does, in order:

1. Parses the options. Any bad option or value: a message on stderr, exit 2, nothing planned.
2. Reads and parses the domain, then the problem. Errors: printed on stderr, exit 2 (exit 0 with
   `--format popf`). Warnings and notes of parsing are printed only with `-v`.
3. Plans (anytime by default, see [How a search ends](#how-a-search-ends)). Every plan is
   validated against the original task before it is printed.
4. Prints the diagnostics of the search (notes, warnings, errors) on stderr, then the status
   line if the task was not solved or `-v` was given, then statistics with `-v`.

`Ctrl-C` (SIGINT) or SIGTERM stops the search cleanly: the best plan found so far is printed and
the exit code is 0. If no plan was found yet, the status is `cancelled` and the exit code is 1.

### Output

The plan text depends on `--format`; where it goes depends on `--output` and on anytime.

**On stdout (no `--output`)**, with anytime (the default) each better plan is printed as soon as
it is found. From the second plan on, a comment line separates them:

```console
$ kairos domain.pddl problem.pddl        # elevator: a temporal domain
; cost = 12.667666666666666
; makespan = 12.668
0.000: (move-down e2 n5 n4) [1.000]
1.000: (board p3 n4 e2) [2.000]
...
9.668: (leave p1 n1 e2) [2.000]
; better plan 2
; cost = 9
; makespan = 9.000
0.000: (move-up e1 n1 n2) [1.500]
0.000: (move-down e2 n5 n4) [1.000]
...
6.669: (leave p3 n1 e2) [2.000]
```

The last plan printed is the best one. Each plan is better than the previous one (lower cost,
see [the cost](#the-cost-of-a-plan)).

**With `--output FILE`**, nothing goes to stdout: with anytime each plan goes to its own file,
`FILE.1`, `FILE.2`, ... (the highest number is the best); with `--no-anytime` the single plan goes
to `FILE`.

**The status line** goes to stderr when the task was not solved, or always with `-v`:

```text
kairos: STATUS (STOP REASON)[, cost C]
```

`STATUS` is one of `solved`, `unsolvable`, `timeout`, `cancelled`, `out of memory`,
`unsupported`, `invalid input`, `grounding too large`, `internal error`. `STOP REASON` is one of
`exhausted`, `proven optimal`, `timeout`, `no improvement`, `target reached`, `stopped`,
`memory limit` (see [How a search ends](#how-a-search-ends)).

```console
$ kairos domain.pddl problem.pddl        # factory2 of plansys2_executor
kairos: unsolvable (exhausted)
$ echo $?
10
```

### Exit codes of solve

| Code | Status | When |
|-----:|--------|------|
| 0 | `solved` | at least one valid plan (also when stopped by timeout, memory, `Ctrl-C`, `--target-cost` or `--no-improvement` after a plan) |
| 10 | `unsolvable` | no plan exists (proved, or with M3 strong evidence, see [Engines](#engines---engine)) |
| 11 | `timeout` | out of time before the first plan |
| 12 | `out of memory`, `grounding too large` | memory limit reached before the first plan |
| 13 | `unsupported` | features Kairos cannot plan with yet, `--engine m1` on a temporal task, M2 exhausted without proof |
| 2 | `invalid input` | bad options, unreadable files, PDDL errors |
| 1 | `cancelled`, `internal error` | stopped by a signal before any plan; a bug (e.g. every plan found failed validation) |

**With `--format popf` the exit code is always 0**, whatever happens (no plan, PDDL errors,
timeout): PlanSys2's POPF plugin expects 0 and looks for the `;;;; Solution Found` marker on
stdout instead. Without a plan, stdout is empty.

### Options of solve

| Option | Value (default) | Short description |
|--------|-----------------|-------------------|
| [`--timeout`](#--timeout-s) | seconds (30) | time budget of the search |
| [`--anytime`, `--no-anytime`](#--anytime---no-anytime) | flag (anytime) | keep improving after the first plan, or not |
| [`--engine`](#engines---engine) | `auto` `m1` `m2` `m3` (`auto`) | planning engine |
| [`--search`](#search-presets---search) | `auto` `ff` `lama` `eager-lama` `novelty` `lama-novelty` `ehc` `goals` (`auto`) | search configuration |
| [`--format`](#--format-ipcpopfjson) | `ipc` `popf` `json` (`ipc`) | plan format |
| [`--output`](#--output-file) | file (stdout) | write plans to files |
| [`--memory`](#--memory-mb) | MB (half of what is available) | memory limit |
| [`--threads`](#--threads-n) | count (cores, up to 4) | phases of the search at once |
| [`--epsilon`](#--epsilon-e) | seconds (0.001) | separation between interfering happenings |
| [`--seed`](#--seed-n) | integer (0) | reserved: no effect yet |
| [`--target-cost`](#--target-cost-c) | number (none) | stop at a plan this good |
| [`--no-improvement`](#--no-improvement-s) | seconds (none) | stop when no better plan comes for this long |
| [`-v`](#-v) | flag | progress and statistics on stderr |
| [`--debug`](#--debugchannels) | `[=CHANNELS]` (all) | debug messages on stderr |
| [`--dump`, `--dump-dir`](#--dumpwhat---dump-dir-dir) | `[=WHAT]`, directory | write intermediate structures |

Numeric values must be plain non-negative numbers (`30`, `2.5`); `1s`, `-1` or `abc` give
`kairos: bad value for --timeout: 1s` and exit 2.

#### `--timeout S`

Time budget in seconds (default 30; fractions allowed, millisecond resolution). It covers
normalization, grounding, translation and search, not the parsing of the files. When it expires
the search stops: with a plan, the best one is kept (status `solved`, reason `timeout`, exit 0);
without one, the status is `timeout` (exit 11).

```bash
kairos domain.pddl problem.pddl --timeout 5      # best plan found in 5 s
kairos domain.pddl problem.pddl --timeout 0.5
```

With anytime, the search runs until the timeout unless it ends earlier (optimality proved, search
space exhausted, `--target-cost`, `--no-improvement`). For small tasks that is usually long
before the timeout.

#### `--anytime`, `--no-anytime`

Anytime is the default: after the first plan, Kairos keeps searching for better ones (restarts
with weighted A* of decreasing weight, each needing a strictly cheaper plan, as LAMA) until the
timeout or a stop condition. `--anytime` only restates the default; if both flags are given,
`--no-anytime` wins.

`--no-anytime` stops improving after the first plan, which is printed once at the end:

```console
$ kairos domain.pddl problem.pddl --no-anytime        # elevator
; cost = 12.667666666666666
; makespan = 12.668
0.000: (move-down e2 n5 n4) [1.000]
...
```

Interactions:

- With M3 and `--search auto`, the portfolio's three phases (see [Engines](#engines---engine))
  still run one after the other with `--no-anytime`; only the improving restarts inside each
  phase are disabled. On elevator with `--engine m3 --no-anytime`, the first plan came at 4 ms and
  the search ended at 1.5 s.
- On a temporal task with `--engine auto`, `--no-anytime` keeps the M2 plan: the M3 phase that
  improves the makespan runs only with anytime.
- `--no-improvement` and `--target-cost` only matter with anytime.
- `--format popf` on stdout prints only the best plan at the end, anytime or not.

#### `--format ipc|popf|json`

| Format | Use | Sequential plan | Temporal plan |
|--------|-----|-----------------|---------------|
| `ipc` (default) | VAL, IPC tools, `kairos validate` | `; cost = 7` then `(a x y)` per line | `; cost`, `; makespan`, then `0.000: (a x y) [5.000]` |
| `popf` | PlanSys2's POPF plugin | `;;;; Solution Found`, `; Cost: 7.000`, then `0.000: (a x)  [0.000]` with the position as time | `;;;; Solution Found`, `; Cost: 9.000`, then `0.000: (a x)  [1.500]` |
| `json` | scripts | one line per plan | one line per plan |

Times and durations have 3 decimals in `ipc` and `popf`, unless rounding to 3 decimals would
make the plan invalid: then 6 (or 9), as many as it needs. That happens with durations such as
121.9775 when an action starts exactly as another ends (the road trip example). The `ipc` cost
comment has full precision.

The examples below are complete outputs (`--no-anytime`, one plan) for two problems of
[examples/pddls](examples/pddls): a temporal one (`domain_simple.pddl` with
`domain_simple_problem_1.pddl`: a robot moves, approaches a person and talks) and a sequential
one (`suave_domain.pddl` with `suave_problem.pddl`). Run them from `docs/examples/pddls`.

**`ipc`**, temporal: start time, action, duration.

```console
$ kairos domain_simple.pddl domain_simple_problem_1.pddl --no-anytime
; cost = 15
; makespan = 15.000
0.000: (move leia kitchen bedroom) [5.000]
5.000: (approach leia bedroom jack) [5.000]
10.000: (talk leia jack jack m1) [5.000]
```

**`ipc`**, sequential: one action per line, in order.

```console
$ kairos suave_domain.pddl suave_problem.pddl --no-anytime
; cost = 7
(start_robot bluerov)
(reconfigure1 f_maintain_motion fd_all_thrusters)
(reconfigure1 f_generate_search_path fd_spiral_high)
(search_pipeline pipeline bluerov)
(reconfigure1 f_follow_pipeline fd_follow_pipeline)
(reconfigure2 f_generate_search_path fd_spiral_high fd_unground)
(inspect_pipeline pipeline bluerov)
```

**`popf`**, temporal: what POPF prints, so that PlanSys2's POPF plugin can read it. The marker
`;;;; Solution Found` comes first, then the cost and the steps (two spaces before the duration).

```console
$ kairos domain_simple.pddl domain_simple_problem_1.pddl --format popf --no-anytime
;;;; Solution Found
; Cost: 15.000
0.000: (move leia kitchen bedroom)  [5.000]
5.000: (approach leia bedroom jack)  [5.000]
10.000: (talk leia jack jack m1)  [5.000]
```

**`popf`**, sequential: the position of each action is its time, and every duration is 0.

```console
$ kairos suave_domain.pddl suave_problem.pddl --format popf --no-anytime
;;;; Solution Found
; Cost: 7.000
0.000: (start_robot bluerov)  [0.000]
1.000: (reconfigure1 f_maintain_motion fd_all_thrusters)  [0.000]
2.000: (reconfigure1 f_generate_search_path fd_spiral_high)  [0.000]
3.000: (search_pipeline pipeline bluerov)  [0.000]
4.000: (reconfigure1 f_follow_pipeline fd_follow_pipeline)  [0.000]
5.000: (reconfigure2 f_generate_search_path fd_spiral_high fd_unground)  [0.000]
6.000: (inspect_pipeline pipeline bluerov)  [0.000]
```

**`json`**: one object per plan, on a single line.

```console
$ kairos domain_simple.pddl domain_simple_problem_1.pddl --format json --no-anytime
{"temporal":true,"cost":15,"makespan":15,"steps":[{"time":0,"duration":5,"action":"(move leia kitchen bedroom)"},{"time":5,"duration":5,"action":"(approach leia bedroom jack)"},{"time":10,"duration":5,"action":"(talk leia jack jack m1)"}]}
```

The same object, indented (`| python3 -m json.tool`):

```json
{
    "temporal": true,
    "cost": 15,
    "makespan": 15,
    "steps": [
        {"time": 0, "duration": 5, "action": "(move leia kitchen bedroom)"},
        {"time": 5, "duration": 5, "action": "(approach leia bedroom jack)"},
        {"time": 10, "duration": 5, "action": "(talk leia jack jack m1)"}
    ]
}
```

| Field | Meaning |
|-------|---------|
| `temporal` | `true` if the plan has times and durations (durative actions), `false` if it is a sequence |
| `cost` | the value of the problem metric; without a metric, the makespan (temporal) or the number of actions (sequential) |
| `makespan` | end of the last action; the number of steps for a sequential plan |
| `steps` | the actions sorted by start time: `time`, `duration` (0 for instantaneous actions) and `action` |

Sequential plans have `"temporal":false`, the position as `time` and `"duration":0`:

```console
$ kairos suave_domain.pddl suave_problem.pddl --format json --no-anytime
{"temporal":false,"cost":7,"makespan":7,"steps":[{"time":0,"duration":0,"action":"(start_robot bluerov)"},{"time":1,"duration":0,"action":"(reconfigure1 f_maintain_motion fd_all_thrusters)"},{"time":2,"duration":0,"action":"(reconfigure1 f_generate_search_path fd_spiral_high)"},{"time":3,"duration":0,"action":"(search_pipeline pipeline bluerov)"},{"time":4,"duration":0,"action":"(reconfigure1 f_follow_pipeline fd_follow_pipeline)"},{"time":5,"duration":0,"action":"(reconfigure2 f_generate_search_path fd_spiral_high fd_unground)"},{"time":6,"duration":0,"action":"(inspect_pipeline pipeline bluerov)"}]}
```

With anytime (the default) `json` prints one line per better plan (JSON Lines), so a script can
read the stream line by line and keep the last one:

```console
$ kairos elevator_domain.pddl elevator_problem.pddl --format json --timeout 3 | tail -n 1 | python3 -c 'import json,sys; p=json.load(sys.stdin); print(p["makespan"], len(p["steps"]))'
9 12
```

Interactions:

- `popf` on stdout: no anytime stream. The search still improves (anytime is not turned off), but
  only the best plan is printed, when the search ends. Bound the wait with `--timeout` or
  `--no-improvement`.
- `popf` with `--output FILE`: anytime files `FILE.1`, `FILE.2`, ... as with the other formats.
- `popf` always exits with 0 (see [Exit codes](#exit-codes-of-solve)).
- `json` on stdout with anytime: one JSON object per line, one line per plan (JSON Lines), with
  no `; better plan` separator.

#### `--output FILE`

Writes plans to files instead of stdout (stdout stays empty).

| Mode | Files written |
|------|---------------|
| anytime (default) | `FILE.1`, `FILE.2`, ..., one per better plan, as they are found |
| `--no-anytime` | `FILE` |

```console
$ kairos domain.pddl problem.pddl --output plan       # elevator
$ ls plan*
plan.1  plan.2
$ head -3 plan.2
; cost = 9
; makespan = 9.000
0.000: (move-up e1 n1 n2) [1.500]
```

Existing files with those names are overwritten. Only the files of the plans found in this run
are written: an older `FILE.3` from a previous run stays. The best plan is the highest numbered
file of the run (the status line with `-v` tells its cost).

#### `--memory MB`

Memory limit in megabytes. Default (also `--memory 0`): half of the smallest of the available
RAM, the cgroup limit and the address-space rlimit. The limit is checked during grounding and
search; reaching it stops the search: with a plan, the best one is kept (reason `memory limit`,
exit 0); without one, exit 12 (`out of memory`, or `grounding too large` when it happens while
grounding).

```bash
kairos domain.pddl problem.pddl --memory 2048
```

With `-v` the limit in use is logged: `[memory] memory limit 22469 MB`.

#### `--threads N`

How many phases of the search run at once. By default, the cores of the machine, up to 4;
Kairos never uses more threads than phases, so a classical task (one phase) uses one.

- With one thread the phases run one after another, sharing the time: M2 until its first plan,
  then the M3 portfolio (`lama` 20 %, `eager-lama` 37.5 %, `lama-novelty` the rest); numeric
  tasks `lama` then `lama-novelty`. Runs are reproducible.
- With several, the phases start together and each has the whole time: the first plan comes
  from whichever finds one first, and the one that improves it best has all the time. A phase
  that proves there is no plan, or that its plan is optimal, stops the others. Which phase
  finds a plan first depends on the machine and its load, so runs may differ.
- The memory limit (`--memory`) is shared by all the threads.

Measured on the IPC temporal suite (225 problems, 30 s): 172 solved with one thread, 180 with
three, with better plans (see [benchmark.md](benchmark.md)).

```console
$ kairos factory.pddl factory_problem_1.pddl --threads 1 --timeout 20 | grep makespan | tail -1
; makespan = 70.010
$ kairos factory.pddl factory_problem_1.pddl --threads 3 --timeout 20 | grep makespan | tail -1
; makespan = 54.008
```

#### `--epsilon E`

Minimum separation, in seconds, between interfering happenings of temporal plans (default
0.001, as POPF and VAL). The scheduler of temporal plans and the validation of each plan found use
it. It has no visible effect on sequential plans.

```bash
kairos domain.pddl problem.pddl --epsilon 0.01
```

#### `--seed N`

Accepted and stored, but no part of the planner uses it yet: results do not depend on it.

#### `--target-cost C`

Stops the search as soon as a plan with cost `<= C` is found (reason `target reached`, exit 0).
Useful with anytime when "good enough" is known.

```console
$ kairos domain.pddl problem.pddl --target-cost 10 -v 2>&1 >/dev/null | grep '^kairos: solved'
kairos: solved (target reached), cost 9                # elevator
```

#### `--no-improvement S`

Stops the search when `S` seconds pass without a better plan (reason `no improvement`, exit 0).
The clock starts at the first plan and restarts at each better one: it never stops a search that
has not found a plan yet (that is what `--timeout` is for).

```bash
kairos domain.pddl problem.pddl --timeout 60 --no-improvement 5
```

#### `-v`

Progress and statistics on stderr: the warnings and notes of parsing, the planner's info
messages with their time and channel, the status line, and two lines of statistics.

```console
$ kairos domain.pddl problem.pddl -v > /dev/null      # elevator
[   0.000s] [memory] memory limit 22469 MB
[   0.001s] [ground] grounded in 0.1 ms: 31 atoms, 0 numeric fluents, 0 actions, 76 durative actions, 0 axioms (194 Datalog facts, 12 rules, 0 instances pruned)
[   0.001s] [sas] SAS+ in 0.1 ms: 5 variables (5 from 5 mutex groups, 0 binary), 2 invariants from 10 candidates, 0 axiom layers
[   0.001s] [orchestrator] engine M2: 76 durative actions compressed
[   0.001s] [orchestrator] search: lama (auto)
[   0.001s] [orchestrator] engine M2: 5 variables, 76 actions, search lama, costs durations
[   0.002s] [search] plan 0 with 10 actions, cost 12.667666666666666
[   0.002s] [orchestrator] engine M3: 81 variables, 152 actions, search lama, costs makespan
[   0.005s] [search] plan 1 with 12 actions, cost 9
[   0.091s] [orchestrator] engine M3: 81 variables, 152 actions, search eager-lama, costs makespan
[   0.171s] [orchestrator] engine M3: 81 variables, 152 actions, search lama-novelty, costs makespan
[   0.270s] [orchestrator] solved (exhausted) in 269.4 ms
kairos: solved (exhausted), cost 9
kairos: parse 0.1 ms, normalize 0.1 ms, ground 0.2 ms, sas 0.2 ms, search 268.7 ms, total 269.4 ms, first plan 1.3 ms
kairos: 31 atoms, 0 actions, 5 variables (5 mutex groups), 132357 states, 14699 expanded, 15476 evaluated, 2 plans, 61.2 bytes per state, 10 MB peak
```

(Some `[search]` lines were removed.) In the statistics, `first plan -1.0 ms` means no plan was
found; `actions` counts instantaneous ground actions (durative ones are in the `[ground]` line).

#### `--debug[=CHANNELS]`

Debug messages on stderr, with the same `[time] [channel]` prefix as `-v`. Without a value, all
channels; with `=CHANNELS`, a comma-separated list of channel names, or `all`. The `=` is
required: `--debug search` would take `search` as a file name.

| Channel | Messages from |
|---------|---------------|
| `general` | general messages |
| `parse` | the PDDL parser |
| `semantic` | the semantic checks |
| `normalize` | normalization of the lifted task |
| `ground` | grounding |
| `sas` | translation to SAS+ (mutex groups, invariants) |
| `search` | the search engines (new best heuristic values, plans) |
| `heuristic` | the heuristics |
| `temporal` | the temporal engines |
| `postprocess` | plan post-processing |
| `validate` | the validator |
| `orchestrator` | engine and search choices, phases, final status |
| `memory` | the memory limit |

`--debug` sets the level to debug, so it also shows every info message (of all channels); the
channel list filters only the debug messages. The status line and the statistics still need `-v`.
An unknown channel gives `kairos: unknown debug channel in foo` and exit 2.

```console
$ kairos domain.pddl problem.pddl --debug=search 2>&1 >/dev/null | head -8     # suave
[   0.001s] [memory] memory limit 22437 MB
[   0.004s] [ground] grounded in 3.2 ms: 79 atoms, 0 numeric fluents, 41 actions, 0 durative actions, 7950 axioms (8760 Datalog facts, 146 rules, 0 instances pruned)
[   0.006s] [sas] SAS+ in 1.3 ms: 78 variables (1 from 4 mutex groups, 11 binary), 4 invariants from 13 candidates, 1 axiom layers
[   0.006s] [orchestrator] search: lama (auto)
[   0.006s] [orchestrator] search: 78 variables, 41 actions, search lama, costs unit
[   0.007s] [search] engine 0: heuristics ff,lmcount
[   0.009s] [search] new best h 5 (1 expanded, 2 evaluated)
[   0.009s] [search] new best h 4 (2 expanded, 3 evaluated)
```

#### `--dump[=WHAT]`, `--dump-dir DIR`

Writes the intermediate structures of the pipeline to files. `WHAT` is a comma-separated list of:

| Name | Files | Contents |
|------|-------|----------|
| `ast` | `ast-domain.pddl`, `ast-problem.pddl` | the parsed domain and problem printed back as PDDL |
| `lifted` | `lifted.txt` | the normalized lifted task |
| `ground` | `ground.txt` | the ground atoms and actions |
| `sas` | `sas.txt` | the SAS+ task: variables, mutex groups, actions |
| `plan-trace` | `plan-trace.txt` | the best plan step by step, with the facts that hold after each step |

| Given | Written | Where |
|-------|---------|-------|
| `--dump` | everything | `DIR` if `--dump-dir DIR`, else the current directory |
| `--dump=sas,plan-trace` | those | `DIR` if `--dump-dir DIR`, else the current directory |
| `--dump-dir DIR` alone | everything | `DIR` |

`DIR` is created if needed. A file that cannot be written gives a warning on stderr; planning goes
on. An unknown name gives `kairos: unknown dump foo` and exit 2.

```console
$ kairos domain.pddl problem.pddl --dump=sas,plan-trace --dump-dir out > /dev/null    # suave
$ ls out
plan-trace.txt  sas.txt
$ head -3 out/plan-trace.txt
; initial state
  (robot_not_started bluerov)
  (inferred-fdbetterutility fd_all_thrusters fd_recover_thrusters)
$ head -3 out/sas.txt
;; SAS+ task suave / example-auv
(:variable v0 group (robot_not_started bluerov) | (robot_started bluerov) | (none-of v0))
(:variable v1 binary (pipeline_found pipeline) | (not (pipeline_found pipeline)))
```

### Engines (`--engine`)

| Value | What it is | Use it when |
|-------|-----------|-------------|
| `auto` (default) | chooses from the task (below) | almost always |
| `m1` | sequential and numeric planning: no durative actions, no timed initial literals | you want to be told if a task is temporal |
| `m2` | temporal planning by compression: each durative action becomes one instantaneous action, then the plan is scheduled | temporal tasks where actions never need to overlap; fastest first plan |
| `m3` | temporal planning with snap actions (start and end of each durative action) over a temporal network, as POPF | tasks that need actions to run concurrently, or timed initial literals |

What `auto` does:

- **No durative actions and no timed initial literals**: one search on the task (M1). `m2` and
  `m3` have no effect on these tasks either.
- **Temporal, "simple" (no action needs to overlap another) and without timed initial
  literals**: M2 for the first plan. With anytime, `--search auto` and a makespan metric (the
  default when the problem has no `:metric`, or `(:metric minimize (total-time))`), the M3
  portfolio then improves the makespan, since M2's improvements lower the sum of durations, which
  says little about the makespan. That is the elevator sample above: plan 1 from M2 (12.668),
  plan 2 from M3 (9).
- **Otherwise**: M3.

The M3 portfolio (`--search auto` on M3): `lama` for 20% of the time left, `eager-lama` for 37.5%
of the time left then, `lama-novelty` for the rest. No configuration wins everywhere. With an
explicit `--search`, M3 runs that single configuration.

Forcing an engine:

- `--engine m1` on a temporal task: `error: engine M1 plans without durative actions or timed
  initial literals: use auto, m2 or m3 [unsupported.engine]`, exit 13.
- `--engine m2` on a task that may need overlapping actions: a warning (`engine M2 forced on a
  task that may need actions to overlap: plans may be missed`). If M2 then exhausts its space
  without a plan and that is not a proof (variable durations, possible required concurrency), the
  status is `unsupported` (exit 13). On a simple task without variable durations, it is a proof:
  `unsolvable`.
- `--engine m3`: when M3 exhausts its space without a plan the status is `unsolvable` (exit 10)
  with a note: like POPF, M3 keeps one partial plan per state, so this is strong evidence, not a
  proof. Constructs M3 cannot handle yet give `unsupported.m3` and exit 13.

```bash
kairos domain.pddl problem.pddl --engine m3 -v      # see the phases in the [orchestrator] lines
```

### Search presets (`--search`)

The search configuration of every engine (M2 and M3 search a transformed task with the same
engines). After the first plan, anytime restarts use weighted A* with weights 5, 3, 2, 1 and the
heuristic of the preset.

| Value | First plan | Use it when |
|-------|-----------|-------------|
| `auto` (default) | with 20 goals or more, all of them facts, `goals` first (half of the time); then `lama`; with numeric conditions, `lama` for a quarter of the time and then `lama-novelty` (below); the portfolio on M3 | almost always |
| `ff` | lazy greedy best-first search, FF heuristic, preferred operators | simple tasks; a cheap baseline |
| `lama` | lazy greedy best-first search, FF and landmark-count heuristics in alternation, preferred operators (as LAMA) | classical tasks |
| `eager-lama` | as `lama`, with eager search | when lazy search wastes evaluations (many children per state) |
| `novelty` | eager greedy search, FF heuristic plus a novelty queue (width 2) | numeric tasks; plateaus where the heuristic does not guide |
| `lama-novelty` | eager greedy search, FF and landmark count plus a novelty queue (width 2) | numeric tasks, M3 tasks |
| `ehc` | enforced hill climbing with FF and helpful actions; falls back to lazy greedy search if it fails | tasks where greedy progress is reliable |
| `goals` | incremental goals: the nearest goal fact not yet true, by a short lazy greedy search (FF) that keeps the ones already true, then the next; falls back to lazy greedy search if it fails | many goals that are facts: patrolling, visiting, delivering |

With many goals `auto` starts with `goals`, alone even with several threads: one search for all
of them meets plateaus that grow with the robots and the goals (a patrol of 1000 places, 200 of
them to patrol and 6 robots: no plan in 100 s with `lama`, 0.2 s with `goals`). It is greedy:
it may spend a robot's energy one goal needs next, so the other searches follow if it fails.

With numeric conditions `auto` starts with lazy `lama`, which finds most first plans at once
(robots carrying units of a resource: [the cooking examples](examples.md)), and moves on to
`lama-novelty` if `lama` has no plan after a quarter of the time. On a temporal task headed for
M3 (the M2 first plan, then the M3 portfolio), `lama-novelty` also improves the M2 plan for 40%
of the time left before M3 starts: on large tasks M3 seldom improves in time, while improving in
M2 and rescheduling does.

When some actions cost 0 for the metric (moving or carrying, when only ingredients cost money),
the heuristics see unit costs for the first plan and cost + 1 for the improving searches, as LAMA
does; plans are still compared by the real metric.

With `-v`, `auto` shows its choice: `[orchestrator] search: lama (auto)`, or
`search: lama, then lama-novelty (auto)` with numeric conditions.

```bash
kairos domain.pddl problem.pddl --search lama-novelty
kairos domain.pddl problem.pddl --search ehc --no-anytime
```

### How a search ends

#### The cost of a plan

Each plan gets a cost, printed in the plan and used to decide whether a plan is better:

- the value of the problem's `:metric`, when there is one;
- otherwise, the makespan for temporal plans and the number of actions for sequential ones.

Only plans better than the previous one are printed. When the metric cannot be expressed as
action costs, the search minimizes the number of actions and says so with a note
(`[orchestrator.metric]`); plans are still compared by the metric.

#### Stop reasons

| Reason | Meaning |
|--------|---------|
| `proven optimal` | sequential task: a restart exhausted every cheaper plan; nothing better exists |
| `exhausted` | the search space was exhausted: no plan at all (unsolvable), or on temporal tasks the searches ended (no optimality proof for the makespan) |
| `timeout` | `--timeout` expired |
| `no improvement` | `--no-improvement` expired, or no strategy left to improve the plan (also the usual reason with `--no-anytime`) |
| `target reached` | a plan with cost `<= --target-cost` |
| `stopped` | `Ctrl-C` or SIGTERM |
| `memory limit` | `--memory` reached |

When grounding shows that no action can reach some goal atom, the result is immediate:
`note: the goal needs atoms that no action can reach [orchestrator.unreachable_goal]`,
`unsolvable`, exit 10.

## validate

```text
kairos validate DOMAIN PROBLEM PLAN [--epsilon E]
```

Checks that `PLAN` solves `PROBLEM`: every step is applicable at its time, interfering
happenings are separated, and the goal holds at the end. Nothing goes to stdout; diagnostics and
the verdict go to stderr.

Accepted plan files:

- timed lines `t: (a x y) [d]` (IPC, VAL, TFD, OPTIC), duration optional;
- sequential lines `(a x y)` or `n: (a x y)` (Fast Downward's `sas_plan` and similar);
- the whole stdout of POPF or OPTIC (or `kairos --format popf`): everything before the last
  `;;;; Solution Found` is ignored, so the last (best) plan is read;
- comments from `;` to the end of the line, and blank lines.

| Option | Value (default) | Meaning |
|--------|-----------------|---------|
| `--epsilon E` | seconds (0.001) | minimum separation between interfering happenings; closer ones give a warning, not an error (as VAL) |

```console
$ kairos domain.pddl problem.pddl --format popf > plan.popf
$ kairos validate domain.pddl problem.pddl plan.popf
plan valid, makespan 9, metric 9
```

```console
$ kairos validate domain.pddl problem.pddl plan.txt
plan.txt:3: error: step 3 (board p3 n3 e2) at 1.000: at start condition false: (passenger_at p3 n3) is false [validate.precondition_failed]
 3 | 1.000: (board p3 n3 e2)  [2.000]
plan invalid
```

| Code | Verdict |
|-----:|---------|
| 0 | `plan valid` (with makespan, and metric when the problem has one) |
| 1 | `plan invalid` or `plan unsupported` (the plan uses something the validator cannot check yet) |
| 2 | wrong number of arguments, unreadable file, bad `--epsilon`, plan syntax error, or errors in the domain or problem (`plan bad input`) |

> [!NOTE]
> The anytime stdout of `solve` in `ipc` format holds several plans one after the other; read as
> one file, their steps add up and the "plan" is invalid. To validate a plan from `solve`, write
> it with `--no-anytime`, `--output FILE` (then validate the last `FILE.N`) or `--format popf`.

## check

```text
kairos check DOMAIN               check a domain
kairos check DOMAIN PROBLEM       check a problem against its domain (the domain too)
kairos check --problem PROBLEM    check a problem alone
```

Checks PDDL without planning: syntax, declarations, types, arities, variables. Errors say where
and often suggest a fix. A problem checked alone cannot have its types and arities checked, and
names not in `:objects` are warnings (they may be domain constants). Nothing goes to stdout.

The output on stderr: the diagnostics, then a verdict, then the PDDL features the files use:

| Verdict | Meaning |
|---------|---------|
| `valid, plannable` | no errors, and Kairos can plan with everything used |
| `valid, not plannable yet` | correct PDDL that uses something Kairos cannot plan with yet (always the case for a problem checked alone) |
| `invalid` | errors |

```console
$ kairos check domain.pddl problem.pddl                # elevator
valid, plannable
features: typing numeric-fluents durative-actions metric metric-total-time
```

```console
$ kairos check broken_domain.pddl
broken_domain.pddl:49:1: error: expected durative action keyword, found '(' [parse.unexpected_token]
 49 | (:durative-action approach
    | ^
broken_domain.pddl:49:1: note: a new section starts here: the previous one was probably not closed [parse.resuming]
 49 | (:durative-action approach
    | ^
invalid
```

| Code | Meaning |
|-----:|---------|
| 0 | valid (plannable or not) |
| 2 | invalid, unreadable file, or wrong arguments (none, or more than two positional) |

`--problem` takes precedence: with it, positional arguments are ignored.

## `--version`, `--help`

```console
$ kairos --version
kairos 0.4.0
```

`--version` must be the first argument. It prints to stdout and exits with 0.

`--help` (or `-h`) anywhere on the command line prints the usage text to stdout and exits with 0,
whatever the other arguments are. `kairos` with no arguments prints the same text to stderr and
exits with 2.

## `--completion`

```text
kairos --completion bash|zsh|fish
```

Prints a completion script to stdout (exit 0). It completes the commands, the options of each
command, the values of `--engine`, `--search`, `--format` and `--completion`, the lists of
`--debug=` and `--dump=` (after the `=`, comma by comma), files for `--output` and `--problem`,
directories for `--dump-dir`, and PDDL files (or any file if there is none) for positional
arguments. It is generated from the parser's own lists, so it never offers an option the parser
rejects.

`--completion` must be the first argument and take exactly one shell name; otherwise `kairos:
--completion takes bash, zsh or fish`, exit 2.

| Shell | Setup |
|-------|-------|
| bash | `eval "$(kairos --completion bash)"` in `~/.bashrc` (needs no bash-completion package) |
| zsh | `eval "$(kairos --completion zsh)"` in `~/.zshrc` (uses `bashcompinit`; does nothing in non-interactive shells) |
| fish | `kairos --completion fish \| source` in `~/.config/fish/config.fish` |

In a colcon workspace nothing is needed for bash and zsh: `source install/setup.bash` (or
`setup.zsh`) sources the completion that the build installed in `share/kairos/completion`.
Installed in a system prefix, the bash script goes to `share/bash-completion/completions/kairos`
and the fish one to `share/fish/vendor_completions.d`, where those shells find them.

## How arguments are parsed

- The command is the first argument if it is `solve`, `validate` or `check`; otherwise `solve`.
- Options and positional arguments can be mixed in any order.
- Options with a value take it as the next argument or after `=`: `--timeout 5` and
  `--timeout=5` are the same.
- `--debug` and `--dump` take a value only after `=`: `--debug=search`, `--dump=sas`.
- `--anytime` and `--no-anytime` take no value (`--anytime=1` is an error).
- `-v` is the only short option (`-h` is handled as `--help`). Any other word that does not start
  with `--` is a positional argument.
- An unknown option (`kairos: unknown option --foo`) or a missing value (`kairos: missing value
  for --timeout`) gives exit 2.
- Repeating an option: the last value wins.
- Every known option is accepted after any command, but each command uses only its own: `kairos
  validate ... --timeout 5` ignores `--timeout`.

## Recipes

**A first plan, fast.**

```bash
kairos domain.pddl problem.pddl --no-anytime
```

**The best plan within a time budget.** Anytime is the default; the last plan printed (or the
highest `plan.N`) is the best:

```bash
kairos domain.pddl problem.pddl --timeout 60 --output plan
kairos domain.pddl problem.pddl --timeout 60 --no-improvement 10 > plans.txt   # stop early if stuck
kairos domain.pddl problem.pddl --timeout 60 --target-cost 42                 # or when good enough
```

**Plans for PlanSys2.** PlanSys2's POPF plugin reads POPF's output: use `--format popf`, which
prints only the best plan and always exits with 0. Bound the search, since the plugin waits for
the end:

```bash
kairos domain.pddl problem.pddl --format popf --timeout 5
```

[tools/plansys2/popf_overlay.sh](../tools/plansys2/popf_overlay.sh) installs Kairos as PlanSys2's
`popf` executable with exactly these options (see the [README](../README.md#with-plansys2)).

**Debugging a domain.**

```bash
kairos check domain.pddl                         # syntax, types, arities
kairos check domain.pddl problem.pddl            # the problem against the domain
kairos domain.pddl problem.pddl -v               # parsing warnings, engine, search, statistics
kairos domain.pddl problem.pddl -v --debug=ground,sas,orchestrator
kairos domain.pddl problem.pddl --dump=lifted,ground,sas,plan-trace --dump-dir dump
```

`ground.txt` shows which actions were instantiated (missing ones point to unreachable
preconditions), and `plan-trace.txt` the state after each step of the plan.

**Validating a plan from another planner.** POPF or OPTIC stdout, Fast Downward's `sas_plan`, or
any IPC/VAL plan file:

```bash
popf domain.pddl problem.pddl > popf.out
kairos validate domain.pddl problem.pddl popf.out
kairos validate domain.pddl problem.pddl sas_plan --epsilon 0.01
```

**Shell completion.**

```bash
echo 'eval "$(kairos --completion bash)"' >> ~/.bashrc      # bash
echo 'eval "$(kairos --completion zsh)"' >> ~/.zshrc        # zsh
echo 'kairos --completion fish | source' >> ~/.config/fish/config.fish
source install/setup.bash                                   # colcon: bash and zsh, nothing else
```

**Using the exit code in scripts.**

```bash
kairos domain.pddl problem.pddl --timeout 10 --output plan
case $? in
  0)  echo "solved" ;;
  10) echo "no plan exists" ;;
  11) echo "no plan in time" ;;
  *)  echo "error" ;;
esac
```
