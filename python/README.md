# kairos-planner

[Kairos](https://github.com/PlanSys2/kairos) in Python: a temporal and numeric PDDL planner
(PDDL 2.1 to 3.1 and PDDL+), with its validator. The wheel also installs the `kairos` command.

```bash
pip install kairos-planner
```

```python
import kairos

domain = kairos.load_domain("domain.pddl")
problem = kairos.load_problem("problem.pddl", domain)

result = kairos.Planner().solve(problem, timeout=10)
if result.status == kairos.Status.SOLVED:
    print(result.plan)                     # IPC format; result.plan.steps for the steps
    print(kairos.validate(problem, result.plan).valid)
elif result.status == kairos.Status.UNSOLVABLE:
    print("no plan exists (proved)")
```

- **Anytime**: `Planner().plans(problem, timeout=30)` yields each better plan as it comes;
  `Planner().start(problem, on_plan=...)` runs the search in its own thread (`wait_first`,
  `next_plan`, `best`, `stop`, `wait`).
- **Options** as keywords (`timeout`, `memory_limit_mb`, `anytime`, `engine="m3"`, `threads`,
  `epsilon`, `stop_after_no_improvement`, `target_cost`, ...) or as a `SolveOptions` object.
- **Statuses**: `SOLVED`, `UNSOLVABLE` (proved), `NO_PLAN_FOUND` (no engine that applies is
  complete for the task: likely none exists, but nothing proves it), `TIMEOUT`,
  `OUT_OF_MEMORY`, `UNSUPPORTED`, ... `result.diagnostics` says why there is no plan.
- **PDDL errors** raise `kairos.PddlError`; each of its `diagnostics` has a stable `code`, a
  `message` and a `location` (file, line, column).
- **Checking and validation**: `check_domain`, `check_problem`, `validate`, `validate_plan`,
  `parse_plan`, `pddl_support()`.
- Planning releases the GIL (several threads can plan at once) and Ctrl-C stops it.
