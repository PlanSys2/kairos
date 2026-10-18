^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package kairos
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.2 (2026-10-18)
------------------
* Proofs that a temporal task has no plan: the timed initial literals take away what the goal
  needs too soon (from the initial state, ignoring deletes, with the shortest durations), or the
  task without durations nor times has no path to the goal (when M3 runs out of states).
* Steps of time: the time is kept only with timed initial literals, until the last one, and
  epsilon passes only right after a happening; tiny tasks that ran to the timeout end in ms.
* With several threads, the phases for when there is no plan stop once another finds one.
* M3 out of states while the steps of time run out of time is a timeout, not "no plan found".
* The deadlines of M3 no longer take durations that change as bounds, nor over all conditions
  an action's start gives itself as needed before it.
* Contributors: Francisco Martín Rico

1.0.1 (2026-10-17)
------------------
* Builds on Ubuntu 22.04 and ROS 2 Humble: CMake 3.22 is enough (nothing needed 3.24), and the
  CI builds the Humble packages as the buildfarm does.
* The style test is disabled, with a warning, when uncrustify is older than its configuration
  (0.78).
* Why there is no plan: an unreachable goal is now traced to the static fact that is really
  missing (the initial facts that grounding leaves out of the task count as reachable).
* Contributors: Francisco Martín Rico

1.0.0 (2026-10-14)
------------------
* PDDL+: processes, events, global constraints and continuous effects (linear and non-linear),
  planned in steps of time and validated in continuous time.
* Durations between their bounds and non-linear numeric conditions in the heuristics.
* Planning without grounding: a lifted engine for tasks with millions of ground actions.
* Status ``NoPlanFound`` (exit code 14): ``Unsolvable`` now needs a proof.
* A bank of benchmarks per PDDL feature (https://github.com/PlanSys2/kairos-benchmarks), and the
  bugs it found fixed: object fluents without a value, timed initial fluents in M2 and M3,
  required concurrency between two variables, deadlines with derived atoms, waits longer than
  2000 s, incremental goals that must break a goal true from the start.
* Every PDDL feature that ``kairos --features`` lists is fully supported.
* Without a plan, a note says why and which option could change it (``orchestrator.why``).
* Builds as a ROS 2 package for Jazzy, Kilted and Lyrical on every buildfarm target (Ubuntu
  amd64 and arm64, RHEL 9 and 10, Fedora): no ``<format>`` (GCC 11), no whole-archive linking
  (CMake 3.28), the library in ``lib`` under ``/opt/ros`` and the command with a relative
  RUNPATH; the tests follow ``BUILD_TESTING``. A CI job builds the packages as the buildfarm.
* The library's SONAME is ``libkairos.so.1``: the ABI is stable within a major version.
* Contributors: Francisco Martín Rico

0.5.0 (2026-10-13)
------------------
* Ahead of LAMA and of the winners of the IPC 2023 on classical planning.
* PDDL3 preferences and trajectory constraints; changing object fluents everywhere.
* Resources in the schedule; temporal landmarks and invariants.
* The memory limit kept while grounding, and the real peak in the statistics.
* Contributors: Francisco Martín Rico

0.4.0 (2026-10-12)
------------------
* Temporal planning: engine M2 (durative actions compressed and rescheduled) and engine M3 (snap
  actions over a temporal network, as POPF); duration inequalities and continuous effects.
* The phases of the search in parallel; scaling to large PlanSys2 problems.
* Why the goal cannot be reached; shell completion; documentation of the command and examples.
* Conda package with pixi.
* Contributors: Francisco Martín Rico

0.3.0 (2026-10-11)
------------------
* Landmarks, novelty and other heuristics in the search; test coverage in the CI.
* Contributors: Francisco Martín Rico

0.2.0 (2026-10-11)
------------------
* PDDL lexer, parser, printer, semantic checks and diagnostics; plan validator.
* Normalization, grounding in Datalog, SAS+ translation with invariant synthesis.
* Packed states, heuristics (hmax, hadd, FF), searches (GBFS, WA*, EHC, anytime).
* The planner API, the orchestrator and the ``kairos`` executable.
* Contributors: Francisco Martín Rico

0.1.0 (2026-10-10)
------------------
* Project skeleton and core infrastructure.
* Contributors: Francisco Martín Rico
