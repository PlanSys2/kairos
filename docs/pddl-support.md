# PDDL support: Kairos and other planners

Which PDDL features each planner actually plans with, checked by running it: one small,
solvable domain and problem per feature, each planner with 10 s, and every plan validated by
`kairos validate`. A cell is ✅ only if the planner returned a valid plan.

Checked on 2026-10-12: Kairos (current `main`), and OPTIC, TFD, POPF, LPG-td, ENHSP-2020,
Metric-FF and LAMA (Fast Downward) from planutils 0.10.25. This page is updated as Kairos gains
features.

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
| Preferences (PDDL3) | ❌ | ✅ | ❌ | ✅ | ❌ | ✅ | ❌ | ❌ |
| Trajectory constraints (PDDL3) | ❌ | ✅ | ❌ | ⚠️ invalid plan | ❌ | ❌ | ❌ | ❌ |
| Object fluents (PDDL 3.1) | ✅ | ❌ | ❌ | ❌ | ❌ | ❌ | ❌ | ❌ |
| Processes (PDDL+) | ❌ | ❌ | ❌ | ❌ | ❌ | ❌ | ❌ | ❌ |

✅ valid plan · ❌ error, unsupported or no plan in 10 s · ⚠️ a plan that fails validation

## What Kairos does not plan with yet

- **Preferences and trajectory constraints** (PDDL3): phase F4. `kairos check` reports them as
  valid but not plannable.
- **Processes and events** (PDDL+): phase F5.
- **Duration inequalities and continuous effects** are planned without a linear program: each
  duration is tried at its bounds, and a continuous change is applied at the safest instant
  for the conditions that read it. Plans are always valid, but a plan that needs a duration in
  between is not found (POPF, with its LP, finds those). See [examples.md](examples.md),
  battery patrol.

## Kairos and imperfect PDDL

An extra `)` after the end of a domain or problem is a warning, not an error
(`parse.extra_parentheses`): VAL, and with it POPF and OPTIC, accept such files, and PlanSys2
domains that work with POPF should work with Kairos. Fast Downward rejects them. Any other text
after the definition is an error.

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
