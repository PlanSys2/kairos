# PDDL support: Kairos and other planners

Which PDDL features each planner actually plans with, checked by running it: one small,
solvable domain and problem per feature, each planner with 10 s, and every plan validated by
`kairos validate`. A cell is ✅ only if the planner returned a valid plan.

Checked on 2026-10-12 (Kairos's PDDL3 rows on 2026-10-13, its PDDL+ row on 2026-10-14): Kairos (current `main`), and OPTIC, TFD, POPF, LPG-td, ENHSP-2020,
Metric-FF and LAMA (Fast Downward) from planutils 0.10.25. This page is updated as Kairos gains
features; `kairos --features` prints Kairos's own table, with the limits of what it supports in
part ([cli.md](cli.md#--features)).

| Feature | Kairos | OPTIC | TFD | POPF | LPG-td | ENHSP | Metric-FF | LAMA |
|---|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|
| STRIPS and types | ✅ | ✅ | ❌ | ✅ | ✅ | ✅ | ✅ | ✅ |
| `either` types | ✅ | ✅ | ❌ | ✅ | ✅ | ❌ | ✅ | ❌ |
| Domain constants | ✅ | ✅ | ❌ | ✅ | ✅ | ✅ | ✅ | ✅ |
| Negative preconditions | ✅ | ❌ | ❌ | ❌ | ✅ | ✅ | ✅ | ✅ |
| Equality | ✅ | ✅ | ❌ | ✅ | ✅ | ✅ | ✅ | ✅ |
| Disjunctive preconditions (`or`) | ✅ | ❌ | ❌ | ❌ | ✅ | ✅ | ✅ | ✅ |
| Quantified preconditions (`forall`, `exists`) | ✅ | ✅ | ❌ | ✅ | ❌ | ✅ | ❌ | ✅ |
| Conditional effects (`when`) | ✅ | ❌ | ❌ | ❌ | ❌ | ✅ | ❌ | ✅ |
| Negative goals | ✅ | ❌ | ❌ | ❌ | ✅ | ✅ | ✅ | ✅ |
| Derived predicates (recursive) | ✅ | ❌ | ❌ | ❌ | ✅ | ❌ | ❌ | ✅ |
| Numeric fluents | ✅ | ✅ | ❌ | ✅ | ✅ | ⚠️ invalid plan | ✅ | ❌ |
| Action costs (`total-cost`) | ✅ | ✅ | ❌ | ✅ | ❌ | ❌ | ❌ | ✅ |
| Metric over numeric fluents | ✅ | ✅ | ❌ | ✅ | ❌ | ✅ | ❌ | ❌ |
| Durative actions | ✅ | ✅ | ✅ | ✅ | ✅ | ❌ | ❌ | ❌ |
| Required concurrency | ✅ | ✅ | ✅ | ✅ | ❌ | ❌ | ❌ | ❌ |
| Duration inequalities with `?duration` in effects | ✅ | ✅ | ❌ | ✅ | ❌ | ❌ | ❌ | ❌ |
| Continuous effects (`#t`) | ✅ | ❌ | ❌ | ✅ | ❌ | ❌ | ❌ | ❌ |
| Timed initial literals | ✅ | ✅ | ❌ | ✅ | ❌ | ❌ | ❌ | ❌ |
| Preferences (PDDL3) | ✅ | ✅ | ❌ | ✅ | ❌ | ✅ | ❌ | ❌ |
| Trajectory constraints (PDDL3) | ✅ | ✅ | ❌ | ⚠️ invalid plan | ❌ | ❌ | ❌ | ❌ |
| Object fluents (PDDL 3.1) | ✅ | ❌ | ❌ | ❌ | ❌ | ❌ | ❌ | ❌ |
| Processes (PDDL+) | ✅ | ❌ | ❌ | ❌ | ❌ | ❌ | ❌ | ❌ |

✅ valid plan · ❌ error, unsupported or no plan in 10 s · ⚠️ a plan that fails validation

## What Kairos does not plan with yet

Nothing: `kairos --features` lists every feature as `full`.

**Durations in between their bounds.** Engines M2 and M3 try each duration of an inequality at
its bounds, with each linear continuous change at the safest instant for the conditions that
read it. When they find no plan, the task is also planned in steps of time (below): an action
then lasts until a deadline, a step of 0.1 s, or the instant where ending it makes a condition
true (an oven that adds 2 degrees per second of heating ends when the temperature reaches the
range wanted), with `?duration` in its end effects as the time it ran. On problems that need a
duration in between (`bench/durations`: a one-shot recharge inside a closing window, an oven,
filling a tank, a shared charger, drilling to a depth range; 25 problems), Kairos solves all 25
within 100 ms, POPF 17 (and 8 invalid plans), OPTIC and NextFLAP 5 each.

## How Kairos plans PDDL+

Processes, events, global constraints (`:constraint`, as ENHSP), and durative actions and timed
initial literals with them, are planned in steps of time: an action that lets 0.1 s pass, one
that lets time pass until some numeric condition is about to change, and, with durative actions,
one that lets epsilon pass. The active processes are integrated with Runge-Kutta; events fire
between steps, and nothing else happens while one can; a durative action runs as start, a clock
and end (its end, and each timed literal, at their exact time). Every plan is then validated in
continuous time: the validator integrates exactly and fires each event at the instant its
condition crosses. A plan the steps reach but continuous time rejects is discarded and the
search goes on; a plan that needs finer timing than the steps is not found. The time itself is
kept only with timed initial literals, until the last one, and epsilon passes only right after
a happening, so that states repeat and small tasks have small spaces.

`always-within` in temporal plans is planned the same way (a clock of the delay, as a process),
and preferences are planned anywhere: compiled where the search sees their cost (goals,
`forall` included; the top of a precondition, four per action; over constraints), counted
exactly elsewhere (more, nested under a `forall` of a precondition or inside an `or`).

## Tasks too large to ground

Kairos grounds tasks (instantiates every action) before searching, like most planners. Some
tasks have millions of ground actions: they do not fit in memory, or preparing the search takes
longer than solving. Classical tasks (types, equality, negative preconditions, action costs)
are then searched as they are, without grounding, as Powerlifted does: successors come from
joining the preconditions over the state, guided by novelty and a lifted FF heuristic with
preferred operators. A short grounding (200 ms) tells whether the task is large; if it is, the
lifted search runs beside the whole grounding (several threads, the first plan wins) or, with
one thread, goes first for a moment. On the hard-to-ground benchmarks (287 problems) Kairos
solves 215, Powerlifted 190.

## Kairos and imperfect PDDL

An extra `)` after the end of a domain or problem is a warning, not an error
(`parse.extra_parentheses`): VAL, and with it POPF and OPTIC, accept such files, and PlanSys2
domains that work with POPF should work with Kairos. Fast Downward rejects them. Any other text
after the definition is an error.

What ENHSP and SMTPlan+ accept, and the PDDL+ benchmarks written for them use, is read the same
way, each with a warning:

- a function without arguments written without parentheses: `(* #t rate)` is `(* #t (rate))`
  (`dialect.bare_function`);
- `_` for `-` or the reverse in a predicate or function name, when only one declared name fits:
  `air_flow` for `air-flow` (`dialect.underscore`);
- a function declared again with other parameter types: one function, whose parameters take
  the types of both (`dialect.function_redeclared`);
- an initial fact with another number of arguments than its predicate, or of a predicate the
  domain does not declare: ignored (`dialect.init_arity`, `semantic.unknown_predicate`);
- `? x` for the variable `?x` (`parse.spaced_variable`);
- a function that only continuous effects change and nothing assigns, left without a value:
  0 (`dialect.uninitialized`, SMTPlan's benchmarks).

In PDDL+, as ENHSP: a condition on a value that is undefined is false (a process that reads it
does not run), `(not (= a b))` holds when either is undefined, and a continuous change of an
undefined fluent leaves it undefined.

## Notes on the other planners

- **TFD** plans only domains with durative actions: the classical rows fail at its translator.
- **OPTIC and POPF** (VAL's parser) reject a numeric effect or condition that is not inside
  `(and ...)`: `:effect (increase (fuel) 4)` is a syntax error, `:effect (and (increase (fuel)
  4))` is fine. The test domains use the second form, the most common one. They have no
  negative preconditions, disjunctions or conditional effects.
- **ENHSP** returned an invalid plan on the numeric test (two refuels of 4 for a fuel of at
  least 10) and does not accept `either`.
- **Metric-FF** ran with `-s 0` (its classical search): its default search needs a metric.
- **LAMA** (Fast Downward) rejects `either` in action parameters and has no numeric fluents.
- **LPG-td** ran with `-quality`; it has no quantified preconditions nor conditional effects.

## How it is checked

The test domains and the script live outside the repository, in the maintainers' workspace
(`bench/features`: `make.py` writes the 22 domain/problem pairs, `run.py` runs every planner
and prints this table). Kairos runs with `--no-anytime --threads 1`; the others as planutils
runs them, in a container with 3 GB and one CPU.

Beyond that one problem per feature, every feature has a bank of domains, from easy to hard,
with problems that have no plan: the `features` suite of
[kairos-benchmarks](https://github.com/PlanSys2/kairos-benchmarks). It takes up to six families
per feature from the public collections (chosen by the features `kairos check` reports) and adds
families written for what none of them has: object fluents, timed initial fluents and a robotic
warehouse with up to 400 robots. Kairos answers "unsolvable" only with a proof; when the
engines that apply are not complete for the task and run out of states, it answers
`no plan found` (exit code 14). On the bank it gives no wrong answer: no "unsolvable" for a
problem with a plan, no plan for one without.
