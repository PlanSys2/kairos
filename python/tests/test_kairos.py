# Copyright 2026 Intelligent Robotics Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import gc
import os
import signal
import threading
import time
from datetime import timedelta
from pathlib import Path

import pytest

import kairos

ROBOTS = """
(define (domain robots) (:requirements :strips :typing)
 (:types robot room)
 (:predicates (at ?r - robot ?l - room) (connected ?a ?b - room))
 (:action move :parameters (?r - robot ?a ?b - room)
  :precondition (and (at ?r ?a) (connected ?a ?b))
  :effect (and (not (at ?r ?a)) (at ?r ?b))))
"""

ROBOTS_PROBLEM = """
(define (problem p) (:domain robots)
 (:objects r1 - robot kitchen corridor hall - room)
 (:init (at r1 kitchen) (connected kitchen corridor) (connected corridor hall))
 (:goal (at r1 hall)))
"""

# Two routes: one flight of 10, or six drives of 1 (anytime improves from the flight)
ROUTES = """
(define (domain routes) (:requirements :strips :action-costs)
 (:predicates (at ?l) (road ?a ?b) (highway ?a ?b))
 (:functions (total-cost))
 (:action drive :parameters (?a ?b) :precondition (and (at ?a) (road ?a ?b))
  :effect (and (not (at ?a)) (at ?b) (increase (total-cost) 1)))
 (:action fly :parameters (?a ?b) :precondition (and (at ?a) (highway ?a ?b))
  :effect (and (not (at ?a)) (at ?b) (increase (total-cost) 10))))
"""

ROUTES_PROBLEM = """
(define (problem p) (:domain routes) (:objects a b c d e f g)
 (:init (at a) (highway a g) (road a b) (road b c) (road c d) (road d e) (road e f) (road f g))
 (:goal (at g)) (:metric minimize (total-cost)))
"""

CELLAR = """
(define (domain cellar) (:requirements :typing :durative-actions)
 (:types match fuse)
 (:predicates (unused ?m - match) (light ?m - match) (mended ?f - fuse) (handfree))
 (:durative-action light-match :parameters (?m - match) :duration (= ?duration 8)
  :condition (at start (unused ?m))
  :effect (and (at start (not (unused ?m))) (at start (light ?m)) (at end (not (light ?m)))))
 (:durative-action mend :parameters (?f - fuse ?m - match) :duration (= ?duration 5)
  :condition (and (at start (handfree)) (over all (light ?m)))
  :effect (and (at start (not (handfree))) (at end (handfree)) (at end (mended ?f)))))
"""

CELLAR_PROBLEM = """
(define (problem p) (:domain cellar) (:objects m1 - match f1 - fuse)
 (:init (unused m1) (handfree)) (:goal (mended f1)))
"""

# M3 runs out of states, the steps of time never do: a search that only ends by a stop
ENDLESS = """
(define (domain endless) (:requirements :typing :durative-actions :numeric-fluents
 :duration-inequalities) (:types robot)
 (:predicates (free ?r - robot) (done ?r - robot))
 (:functions (battery ?r - robot))
 (:durative-action charge :parameters (?r - robot) :duration (>= ?duration 1)
  :condition (at start (free ?r))
  :effect (and (at start (not (free ?r))) (at end (increase (battery ?r) (* 10 ?duration)))))
 (:durative-action work :parameters (?r - robot) :duration (= ?duration 1)
  :condition (at start (and (>= (battery ?r) 30) (<= (battery ?r) 20)))
  :effect (at end (done ?r))))
"""

ENDLESS_PROBLEM = """
(define (problem p) (:domain endless) (:objects r - robot)
 (:init (free r) (= (battery r) 0)) (:goal (done r)))
"""


def problem(domain_text, problem_text):
    return kairos.parse_problem(problem_text, kairos.parse_domain(domain_text))


# --- Parsing -------------------------------------------------------------------------------------

def test_version():
    assert kairos.__version__ == kairos.version()
    assert kairos.__version__.count(".") == 2


def test_parse_domain_and_problem():
    d = kairos.parse_domain(ROBOTS)
    assert d.name == "robots"
    assert "typing" in d.features()
    p = kairos.parse_problem(ROBOTS_PROBLEM, d)
    assert p.name == "p"
    assert p.domain.name == "robots"


def test_errors_carry_their_position():
    with pytest.raises(kairos.PddlError) as e:
        kairos.parse_domain("(define (domain d)\n (:action a :parameters ()\n :effect (p)))",
                            "broken.pddl")
    errors = [x for x in e.value.diagnostics if x.severity == kairos.Severity.ERROR]
    assert errors
    assert errors[0].location.file == "broken.pddl"
    assert errors[0].location.line >= 1
    assert errors[0].code
    assert "broken.pddl" in str(e.value)
    assert isinstance(e.value, ValueError)
    # Formatted with the source line, as compilers do
    assert "|" in errors[0].format("(define (domain d)\n (:action a :parameters ()\n :effect (p)))")


def test_problem_errors_against_the_domain():
    d = kairos.parse_domain(ROBOTS)
    with pytest.raises(kairos.PddlError):
        kairos.parse_problem("(define (problem p) (:domain robots) (:objects r1 - robot)"
                             " (:init (at r1 nowhere)) (:goal (at r1 nowhere)))", d)


def test_load_from_files(tmp_path):
    (tmp_path / "d.pddl").write_text(ROBOTS)
    (tmp_path / "p.pddl").write_text(ROBOTS_PROBLEM)
    d = kairos.load_domain(tmp_path / "d.pddl")
    p = kairos.load_problem(str(tmp_path / "p.pddl"), d)
    assert p.name == "p"
    with pytest.raises(kairos.PddlError):
        kairos.load_domain(tmp_path / "missing.pddl")


def test_check():
    report = kairos.check_domain(ROBOTS)
    assert report.valid and report.plannable
    assert "typing" in report.features
    bad = kairos.check_problem("(define (problem p) (:domain robots) (:objects r1 - robot)"
                               " (:init (at r1 nowhere)) (:goal (at r1 nowhere)))", ROBOTS)
    assert not bad.valid
    assert bad.diagnostics
    alone = kairos.check_problem(ROBOTS_PROBLEM)
    assert alone.valid and not alone.plannable_known


def test_pddl_support_lists_every_feature():
    support = kairos.pddl_support()
    names = {f.feature for f in support}
    assert {"durative-actions", "processes", "preferences", "object-fluents"} <= names
    assert all(f.level == kairos.SupportLevel.FULL for f in support)


# --- Plans ---------------------------------------------------------------------------------------

def test_plans_read_written_and_compared():
    plan = kairos.parse_plan("0.000: (light-match m1) [8.000]\n0.001: (mend f1 m1) [5.000]\n")
    assert plan.temporal
    assert len(plan) == 2
    assert plan.steps[1].name == "mend"
    assert plan.steps[1].args == ["f1", "m1"]
    assert plan.steps[1].action() == "(mend f1 m1)"
    assert plan.makespan() == pytest.approx(8.0)
    assert kairos.parse_plan(str(plan)) == plan
    assert '"steps"' in plan.to_string(kairos.PlanFormat.JSON)
    assert ";;;; Solution Found" in plan.to_string(kairos.PlanFormat.POPF)
    built = kairos.Plan()
    built.steps = [kairos.PlanStep("move", ["r1", "kitchen", "corridor"]),
                   kairos.PlanStep("move", ["r1", "corridor", "hall"], time=1.0)]
    assert built.steps[0].action() == "(move r1 kitchen corridor)"
    with pytest.raises(kairos.PddlError):
        kairos.parse_plan("0.000: (light-match m1 [8.000]\n")


def test_validate():
    p = problem(ROBOTS, ROBOTS_PROBLEM)
    good = kairos.parse_plan("(move r1 kitchen corridor)\n(move r1 corridor hall)\n")
    report = kairos.validate(p, good)
    assert report.valid and bool(report)
    assert report.status == kairos.ValidationStatus.VALID
    bad = kairos.parse_plan("(move r1 kitchen corridor)\n(move r1 kitchen hall)\n")
    report = kairos.validate(p, bad)
    assert not report.valid
    assert report.failed_step == 1
    assert report.diagnostics
    short = kairos.parse_plan("(move r1 kitchen corridor)\n")
    assert kairos.validate(p, short).status == kairos.ValidationStatus.INVALID
    # From the texts, with options
    o = kairos.ValidateOptions()
    o.check_goal = False
    assert kairos.validate_plan(ROBOTS, ROBOTS_PROBLEM, short, o).valid
    assert kairos.validate_plan("(define", ROBOTS_PROBLEM, short).status == \
        kairos.ValidationStatus.BAD_INPUT


def test_validate_temporal_and_metric():
    p = problem(CELLAR, CELLAR_PROBLEM)
    ok = kairos.parse_plan("0.000: (light-match m1) [8.000]\n0.001: (mend f1 m1) [5.000]\n")
    assert kairos.validate(p, ok).valid
    late = kairos.parse_plan("0.000: (light-match m1) [8.000]\n4.000: (mend f1 m1) [5.000]\n")
    assert not kairos.validate(p, late).valid  # the match goes out while mending
    r = problem(ROUTES, ROUTES_PROBLEM)
    report = kairos.validate(r, kairos.parse_plan("(fly a g)\n"))
    assert report.metric == pytest.approx(10.0)


# --- Planning ------------------------------------------------------------------------------------

def test_solve_sequential():
    result = kairos.Planner().solve(problem(ROBOTS, ROBOTS_PROBLEM), timeout=10)
    assert result.status == kairos.Status.SOLVED
    assert [s.action() for s in result.plan.steps] == \
        ["(move r1 kitchen corridor)", "(move r1 corridor hall)"]
    assert result.stats.total_ms > 0
    assert result.stats.first_plan_ms is not None
    assert result.stats.plans >= 1


def test_solve_temporal_with_required_concurrency():
    p = problem(CELLAR, CELLAR_PROBLEM)
    result = kairos.Planner().solve(p, timeout=timedelta(seconds=10))
    assert result.status == kairos.Status.SOLVED
    assert result.plan.temporal
    assert kairos.validate(p, result.plan).valid


def test_anytime_proves_the_optimum():
    result = kairos.Planner().solve(problem(ROUTES, ROUTES_PROBLEM), timeout=10)
    assert result.status == kairos.Status.SOLVED
    assert result.plan.cost == pytest.approx(6.0)
    assert result.stop_reason == kairos.StopReason.PROVEN_OPTIMAL


def test_unsolvable_and_no_plan_found():
    unreachable = ROBOTS_PROBLEM.replace("(connected corridor hall)", "")
    r = kairos.Planner().solve(problem(ROBOTS, unreachable), timeout=10)
    assert r.status == kairos.Status.UNSOLVABLE
    assert r.plan is None
    assert any(d.code.startswith("orchestrator.") for d in r.diagnostics)
    endless = kairos.Planner().solve(problem(ENDLESS, ENDLESS_PROBLEM), timeout=0.5, threads=4)
    assert endless.status == kairos.Status.TIMEOUT


def test_options_as_object_and_keywords():
    p = problem(CELLAR, CELLAR_PROBLEM)
    o = kairos.SolveOptions()
    o.timeout = 5
    o.engine = kairos.Engine.M3
    o.anytime = False
    assert o.timeout == pytest.approx(5.0)
    r = kairos.Planner().solve(p, o)
    assert r.status == kairos.Status.SOLVED
    assert r.stop_reason == kairos.StopReason.NO_IMPROVEMENT  # first plan only
    # Keywords over the object; strings for the enums
    r = kairos.Planner().solve(p, o, engine="m1")
    assert r.status == kairos.Status.UNSUPPORTED
    assert o.engine == kairos.Engine.M3  # the caller's object is not changed
    # SearchOptions for a solve: what SolveOptions has
    so = kairos.SearchOptions()
    so.timeout = 4
    so.target_cost = 1.0
    assert kairos.Planner().solve(p, so).status == kairos.Status.SOLVED
    with pytest.raises(TypeError):
        kairos.Planner().solve(p, timeot=5)
    with pytest.raises(KeyError):
        kairos.Planner().solve(p, engine="m9")


def test_solve_from_texts_and_files(tmp_path):
    assert kairos.solve(ROBOTS, ROBOTS_PROBLEM, timeout=5).status == kairos.Status.SOLVED
    (tmp_path / "d.pddl").write_text(ROBOTS)
    (tmp_path / "p.pddl").write_text(ROBOTS_PROBLEM)
    r = kairos.solve(tmp_path / "d.pddl", str(tmp_path / "p.pddl"), timeout=5)
    assert r.status == kairos.Status.SOLVED


def test_log_callback():
    messages = []
    kairos.Planner().solve(problem(ROBOTS, ROBOTS_PROBLEM), timeout=5,
                           on_log=lambda level, channel, text: messages.append((level, channel)))
    assert messages
    assert all(isinstance(level, kairos.LogLevel) for level, _ in messages)
    assert any(channel == kairos.LogChannel.ORCHESTRATOR for _, channel in messages)


@pytest.mark.filterwarnings("ignore::pytest.PytestUnraisableExceptionWarning")
def test_a_callback_that_raises_does_not_stop_planning():
    def boom(*args):
        raise RuntimeError("boom")
    r = kairos.Planner().solve(problem(ROBOTS, ROBOTS_PROBLEM), timeout=5, on_log=boom)
    assert r.status == kairos.Status.SOLVED


# --- Background searches -------------------------------------------------------------------------

def test_background_search_with_callback():
    updates = []
    search = kairos.Planner().start(
        problem(ROUTES, ROUTES_PROBLEM), timeout=10,
        on_plan=lambda plan, index, elapsed, optimal: updates.append(
            (plan.cost, index, elapsed, optimal)))
    first = search.wait_first(10)
    assert first is not None
    result = search.wait()
    assert result.status == kairos.Status.SOLVED
    assert not search.running()
    assert updates[0][1] == 0
    costs = [u[0] for u in updates]
    assert costs == sorted(costs, reverse=True)  # each better
    assert updates[-1][0] == pytest.approx(6.0)
    assert search.best().cost == pytest.approx(6.0)
    assert search.stop_reason == kairos.StopReason.PROVEN_OPTIMAL


def test_plans_iterator():
    costs = [p.cost for p in kairos.Planner().plans(problem(ROUTES, ROUTES_PROBLEM), timeout=10)]
    assert costs
    assert costs[-1] == pytest.approx(6.0)
    assert costs == sorted(costs, reverse=True)


def test_an_iterator_without_plans_ends_with_the_search():
    started = time.monotonic()
    assert list(kairos.Planner().plans(problem(ENDLESS, ENDLESS_PROBLEM), timeout=1)) == []
    assert time.monotonic() - started < 5


def test_leaving_the_iterator_stops_the_search():
    # A first plan at once, then improving for up to 30 s if nobody stops it
    p = problem(ENDLESS.replace("(<= (battery ?r) 20)", ""), ENDLESS_PROBLEM)
    started = time.monotonic()
    for plan in kairos.Planner().plans(p, timeout=30, threads=2):
        assert plan.steps
        break
    gen = kairos.Planner().plans(p, timeout=30)
    next(gen)
    gen.close()
    assert time.monotonic() - started < 10


def test_stop_and_drop_a_running_search():
    p = problem(ENDLESS, ENDLESS_PROBLEM)
    search = kairos.Planner().start(p, timeout=30, threads=2)
    time.sleep(0.2)
    assert search.running()
    started = time.monotonic()
    search.stop()
    result = search.wait()
    assert time.monotonic() - started < 5
    assert result.status == kairos.Status.CANCELLED
    # Dropped while running, with a callback: stopped without a deadlock
    other = kairos.Planner().start(p, timeout=30, on_plan=lambda *a: None,
                                   on_log=lambda *a: None)
    time.sleep(0.1)
    del other
    gc.collect()


def test_ctrl_c_stops_solve():
    p = problem(ENDLESS, ENDLESS_PROBLEM)
    threading.Timer(0.3, lambda: os.kill(os.getpid(), signal.SIGINT)).start()
    started = time.monotonic()
    with pytest.raises(KeyboardInterrupt):
        kairos.Planner().solve(p, timeout=30)
    assert time.monotonic() - started < 5


def test_several_threads_plan_at_once():
    p = problem(CELLAR, CELLAR_PROBLEM)
    results = []

    def run():
        results.append(kairos.Planner().solve(p, timeout=10).status)
    threads = [threading.Thread(target=run) for _ in range(4)]
    for t in threads:
        t.start()
    for t in threads:
        t.join()
    assert results == [kairos.Status.SOLVED] * 4


def test_the_examples_of_the_documentation():
    examples = Path(os.environ.get(
        "KAIROS_EXAMPLES_DIR", Path(__file__).parents[2] / "docs" / "examples" / "pddls"))
    d = kairos.load_domain(examples / "cooking_domain.pddl")
    p = kairos.load_problem(examples / "cooking_problem_simple.pddl", d)
    r = kairos.Planner().solve(p, timeout=10, anytime=False)
    assert r.status == kairos.Status.SOLVED
    assert kairos.validate(p, r.plan).valid
