// Copyright 2026 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gtest/gtest.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <stdexcept>
#include <set>
#include <string>
#include <tuple>
#include <thread>
#include <vector>

#include "kairos/kairos.hpp"
#include "support/pddl_helpers.hpp"

namespace kairos
{
namespace
{

// Links only against libkairos: planning through the public API

using namespace std::chrono_literals;
namespace fs = std::filesystem;

const char * kRobots =
  "(define (domain robots)\n"
  " (:requirements :strips :typing)\n"
  " (:types robot room)\n"
  " (:predicates (at ?r - robot ?l - room) (connected ?a ?b - room))\n"
  " (:action move :parameters (?r - robot ?a ?b - room)\n"
  "  :precondition (and (at ?r ?a) (connected ?a ?b))\n"
  "  :effect (and (not (at ?r ?a)) (at ?r ?b))))\n";

const char * kRobotsProblem =
  "(define (problem p) (:domain robots)\n"
  " (:objects r1 - robot kitchen corridor hall attic - room)\n"
  " (:init (at r1 kitchen) (connected kitchen corridor) (connected corridor hall))\n"
  " (:goal (at r1 hall)))\n";

// Two routes: one expensive flight, or six cheap drives
const char * kRoutes =
  "(define (domain routes)\n"
  " (:requirements :strips :action-costs)\n"
  " (:predicates (at ?l) (road ?a ?b) (highway ?a ?b))\n"
  " (:functions (total-cost))\n"
  " (:action drive :parameters (?a ?b) :precondition (and (at ?a) (road ?a ?b))\n"
  "  :effect (and (not (at ?a)) (at ?b) (increase (total-cost) 1)))\n"
  " (:action fly :parameters (?a ?b) :precondition (and (at ?a) (highway ?a ?b))\n"
  "  :effect (and (not (at ?a)) (at ?b) (increase (total-cost) 10))))\n";

const char * kRoutesProblem =
  "(define (problem p) (:domain routes)\n"
  " (:objects a b c d e f g)\n"
  " (:init (at a) (highway a g) (road a b) (road b c) (road c d) (road d e) (road e f)\n"
  "  (road f g))\n"
  " (:goal (at g))\n"
  " (:metric minimize (total-cost)))\n";

// Five blocks: greedy finds a long plan first, anytime improves it (IPC 2000, BLOCKS-5-1)
const char * kBlocks =
  "(define (domain blocks) (:requirements :strips)\n"
  " (:predicates (on ?x ?y) (ontable ?x) (clear ?x) (handempty) (holding ?x))\n"
  " (:action pick-up :parameters (?x) :precondition (and (clear ?x) (ontable ?x) (handempty))\n"
  "  :effect (and (not (ontable ?x)) (not (clear ?x)) (not (handempty)) (holding ?x)))\n"
  " (:action put-down :parameters (?x) :precondition (holding ?x)\n"
  "  :effect (and (not (holding ?x)) (clear ?x) (handempty) (ontable ?x)))\n"
  " (:action stack :parameters (?x ?y) :precondition (and (holding ?x) (clear ?y))\n"
  "  :effect (and (not (holding ?x)) (not (clear ?y)) (clear ?x) (handempty) (on ?x ?y)))\n"
  " (:action unstack :parameters (?x ?y) :precondition (and (on ?x ?y) (clear ?x) (handempty))\n"
  "  :effect (and (holding ?x) (clear ?y) (not (clear ?x)) (not (handempty)) (not (on ?x ?y)))))";

const char * kBlocksProblem =
  "(define (problem blocks-5-1) (:domain blocks) (:objects a d c e b)\n"
  " (:init (clear b) (clear e) (clear c) (ontable d) (ontable e) (ontable c) (on b a) (on a d)\n"
  "  (handempty))\n"
  " (:goal (and (on d c) (on c b) (on b a) (on a e))))";

const char * kDurative =
  "(define (domain timed)\n"
  " (:requirements :strips :durative-actions)\n"
  " (:predicates (p) (q))\n"
  " (:durative-action a :parameters () :duration (= ?duration 1)\n"
  "  :condition (at start (p)) :effect (at end (q))))\n";

// PDDL+: a process fills, an action stops when full
const char * kProcess =
  "(define (domain flow) (:requirements :fluents :time) (:predicates (on) (done))"
  " (:functions (level))"
  " (:process fill :parameters () :precondition (on) :effect (increase (level) (* #t 2)))"
  " (:action stop :parameters () :precondition (>= (level) 10) :effect (done)))\n";

const char * kProcessProblem =
  "(define (problem x) (:domain flow) (:init (on) (= (level) 0)) (:goal (done)))\n";

// PDDL+ with a continuous effect that reads ?duration: not plannable yet (found when planning,
// not from the features)
const char * kProcessDurative =
  "(define (domain mixed) (:requirements :fluents :time :durative-actions)"
  " (:predicates (on) (done)) (:functions (level))"
  " (:process fill :parameters () :precondition (on) :effect (increase (level) (* #t 2)))"
  " (:durative-action stop :parameters () :duration (= ?duration 1)"
  "  :condition (at start (>= (level) 10))"
  "  :effect (and (at end (done)) (decrease (level) (* #t ?duration)))))\n";

const char * kProcessDurativeProblem =
  "(define (problem x) (:domain mixed) (:init (on) (= (level) 0)) (:goal (done)))\n";

const char * kDurativeProblem =
  "(define (problem x) (:domain timed) (:init (p)) (:goal (q)))";

// Pigeons into holes, one per hole: relaxed reachable, so only search can prove it when there
// are more pigeons than holes, and the state space grows fast
const char * kHoles =
  "(define (domain holes)\n"
  " (:requirements :strips :typing)\n"
  " (:types pigeon hole)\n"
  " (:predicates (out ?p - pigeon) (in ?p - pigeon) (free ?h - hole))\n"
  " (:action put :parameters (?p - pigeon ?h - hole)\n"
  "  :precondition (and (out ?p) (free ?h))\n"
  "  :effect (and (not (out ?p)) (in ?p) (not (free ?h)))))\n";

std::string holes_problem(int pigeons, int holes)
{
  std::string objects, init, goal;
  for (int i = 0; i < pigeons; ++i) {
    objects += " p" + std::to_string(i);
    init += " (out p" + std::to_string(i) + ")";
    goal += " (in p" + std::to_string(i) + ")";
  }
  objects += " - pigeon";
  for (int i = 0; i < holes; ++i) {
    objects += " h" + std::to_string(i);
    init += " (free h" + std::to_string(i) + ")";
  }
  objects += " - hole";
  return "(define (problem x) (:domain holes) (:objects" + objects + ") (:init" + init +
         ") (:goal (and" + goal + ")))";
}

Problem problem(const std::string & domain_text, const std::string & problem_text)
{
  auto d = parse_domain(domain_text);
  if (!d.ok()) {
    throw std::runtime_error("bad domain: " + to_string(d.diagnostics().at(0)));
  }
  auto p = parse_problem(problem_text, *d);
  if (!p.ok()) {
    throw std::runtime_error("bad problem: " + to_string(p.diagnostics().at(0)));
  }
  return *p;
}

SolveOptions quick()
{
  SolveOptions o;
  o.timeout = 10s;
  return o;
}

bool has_code(const std::vector<Diagnostic> & ds, const std::string & prefix)
{
  for (const auto & d : ds) {
    if (d.code.rfind(prefix, 0) == 0) {
      return true;
    }
  }
  return false;
}

fs::path temp_dir(const std::string & name)
{
  fs::path dir = fs::temp_directory_path() /
    ("kairos_planner_test_" + name + "_" + std::to_string(::testing::UnitTest::GetInstance()->
    random_seed()));
  fs::remove_all(dir);
  fs::create_directories(dir);
  return dir;
}

// --- Parsing and loading ----------------------------------------------------------------------

TEST(PlannerApiTest, ParsesDomainsAndProblems)
{
  auto d = parse_domain(kRobots);
  ASSERT_TRUE(d.ok());
  EXPECT_EQ(d->name(), "robots");
  auto features = d->features();
  EXPECT_NE(std::find(features.begin(), features.end(), "typing"), features.end());
  auto p = parse_problem(kRobotsProblem, *d);
  ASSERT_TRUE(p.ok());
  EXPECT_EQ(p->name(), "p");
  EXPECT_EQ(p->domain().name(), "robots");
}

TEST(PlannerApiTest, ParseErrorsCarryTheFileName)
{
  auto d = parse_domain("(define (domain d) (:predicates (p))", "mine.pddl");
  ASSERT_FALSE(d.ok());
  ASSERT_FALSE(d.diagnostics().empty());
  EXPECT_EQ(d.diagnostics()[0].location->file, "mine.pddl");

  auto good = parse_domain(kRobots);
  ASSERT_TRUE(good.ok());
  auto p = parse_problem(
    "(define (problem p) (:domain robots) (:objects r1 - robot) (:init (at r1 r1)))", *good,
    "prob.pddl");
  ASSERT_FALSE(p.ok());
  EXPECT_EQ(p.diagnostics()[0].location->file, "prob.pddl");
}

TEST(PlannerApiTest, LoadsFromFiles)
{
  fs::path dir = temp_dir("load");
  std::ofstream(dir / "domain.pddl") << kRobots;
  std::ofstream(dir / "problem.pddl") << kRobotsProblem;
  auto d = load_domain(dir / "domain.pddl");
  ASSERT_TRUE(d.ok());
  auto p = load_problem(dir / "problem.pddl", *d);
  ASSERT_TRUE(p.ok());
  EXPECT_EQ(Planner().solve(*p, quick()).status, Status::Solved);

  auto missing = load_domain(dir / "nope.pddl");
  ASSERT_FALSE(missing.ok());
  EXPECT_EQ(missing.diagnostics().at(0).code, "io.cannot_read");
  auto missing_problem = load_problem(dir / "nope.pddl", *d);
  ASSERT_FALSE(missing_problem.ok());
  EXPECT_EQ(missing_problem.diagnostics().at(0).code, "io.cannot_read");
  fs::remove_all(dir);
}

TEST(PlannerApiTest, ChecksProblemsAgainstAParsedDomain)
{
  auto d = parse_domain(kRobots);
  ASSERT_TRUE(d.ok());
  CheckReport ok = check_problem(kRobotsProblem, *d);
  EXPECT_TRUE(ok.valid);
  EXPECT_TRUE(ok.plannable);

  CheckReport bad = check_problem(
    "(define (problem p) (:domain robots) (:objects r1 - robot) (:init (at r1 r1)))", *d,
    "bad.pddl");
  EXPECT_FALSE(bad.valid);
  EXPECT_FALSE(bad.plannable);
  EXPECT_EQ(bad.diagnostics.at(0).location->file, "bad.pddl");

  auto timed = parse_domain(kDurative);
  ASSERT_TRUE(timed.ok());
  EXPECT_TRUE(check_problem(kDurativeProblem, *timed).plannable);
  auto flow = parse_domain(kProcess);
  ASSERT_TRUE(flow.ok());
  CheckReport t = check_problem(kProcessProblem, *flow);
  EXPECT_TRUE(t.valid);
  EXPECT_TRUE(t.plannable);
  EXPECT_FALSE(has_code(t.diagnostics, "unsupported."));
}

// --- solve() ----------------------------------------------------------------------------------

TEST(PlannerApiTest, SolvesAndValidates)
{
  Problem p = problem(kRobots, kRobotsProblem);
  SolveResult r = Planner().solve(p, quick());
  ASSERT_EQ(r.status, Status::Solved);
  ASSERT_TRUE(r.plan);
  ASSERT_EQ(r.plan->steps.size(), 2u);
  EXPECT_EQ(r.plan->steps[0].action(), "(move r1 kitchen corridor)");
  EXPECT_EQ(r.plan->steps[1].action(), "(move r1 corridor hall)");
  EXPECT_EQ(r.plan->cost, 2.0);
  // A plan of unit actions explored completely below its length is optimal
  EXPECT_EQ(r.stop_reason, StopReason::ProvenOptimal);
  EXPECT_TRUE(validate(p, *r.plan).valid());

  const Statistics & s = r.stats;
  EXPECT_GT(s.atoms, 0u);
  EXPECT_EQ(s.actions, 2u);  // only the reachable moves
  EXPECT_GT(s.variables, 0u);
  EXPECT_GT(s.expanded, 0u);
  EXPECT_GE(s.generated, s.expanded);
  EXPECT_GT(s.states, 0u);
  EXPECT_EQ(s.plans, 1u);
  ASSERT_TRUE(s.first_plan_ms);
  EXPECT_LE(*s.first_plan_ms, s.total_ms);
  EXPECT_GE(s.total_ms, s.search_ms);
  EXPECT_GE(s.parse_ms, 0.0);
  EXPECT_GT(s.memory_limit_bytes, 0u);
  EXPECT_GT(s.bytes_per_state, 0.0);
}

TEST(PlannerApiTest, AnInvalidPlanIsReportedByValidate)
{
  Problem p = problem(kRobots, kRobotsProblem);
  auto plan = parse_plan("(move r1 kitchen hall)\n");
  ASSERT_TRUE(plan.ok());
  ValidationReport r = validate(p, *plan);
  EXPECT_FALSE(r.valid());
  ASSERT_FALSE(r.diagnostics.empty());
}

TEST(PlannerApiTest, AlreadySolvedProblemGivesAnEmptyPlan)
{
  Problem p = problem(
    kRobots,
    "(define (problem p) (:domain robots) (:objects r1 - robot k - room)"
    " (:init (at r1 k)) (:goal (at r1 k)))");
  SolveResult r = Planner().solve(p, quick());
  ASSERT_EQ(r.status, Status::Solved);
  ASSERT_TRUE(r.plan);
  EXPECT_TRUE(r.plan->steps.empty());
  EXPECT_EQ(r.plan->cost, 0.0);
}

TEST(PlannerApiTest, UsesTheMetricForActionCosts)
{
  Problem p = problem(kRoutes, kRoutesProblem);
  SolveResult r = Planner().solve(p, quick());
  ASSERT_EQ(r.status, Status::Solved);
  ASSERT_TRUE(r.plan);
  // Anytime ends with the six cheap drives
  EXPECT_EQ(r.plan->steps.size(), 6u);
  EXPECT_EQ(r.plan->cost, 6.0);
  EXPECT_EQ(r.stop_reason, StopReason::ProvenOptimal);

  SolveOptions first = quick();
  first.anytime = false;
  SolveResult f = Planner().solve(p, first);
  ASSERT_EQ(f.status, Status::Solved);
  EXPECT_EQ(f.stats.plans, 1u);
  EXPECT_TRUE(validate(p, *f.plan).valid());
}

TEST(PlannerApiTest, UnreachableGoalIsUnsolvable)
{
  Problem p = problem(
    kRobots,
    "(define (problem p) (:domain robots) (:objects r1 - robot kitchen attic - room)"
    " (:init (at r1 kitchen)) (:goal (at r1 attic)))");
  SolveResult r = Planner().solve(p, quick());
  EXPECT_EQ(r.status, Status::Unsolvable);
  EXPECT_FALSE(r.plan);
  EXPECT_EQ(r.stop_reason, StopReason::Exhausted);
}

TEST(PlannerApiTest, SearchProvesUnsolvability)
{
  // Relaxed reachable: only the exhausted search shows four pigeons do not fit in three holes
  Problem p = problem(kHoles, holes_problem(4, 3));
  SolveResult r = Planner().solve(p, quick());
  EXPECT_EQ(r.status, Status::Unsolvable);
  EXPECT_EQ(r.stop_reason, StopReason::Exhausted);
  EXPECT_GT(r.stats.expanded, 1u);
}

TEST(PlannerApiTest, ContinuousEffectsAndDurationInequalities)
{
  // Filling at 2 per second, for as long as wanted up to 10 s: at least 10 needed
  const char * domain =
    "(define (domain tank) (:requirements :durative-actions :fluents :continuous-effects"
    " :duration-inequalities) (:predicates (idle)) (:functions (level))"
    " (:durative-action fill :parameters () :duration (and (>= ?duration 1) (<= ?duration 10))"
    "  :condition (at start (idle))"
    "  :effect (and (at start (not (idle))) (at end (idle)) (increase (level) (* #t 2)))))";
  Problem p = problem(domain,
      "(define (problem x) (:domain tank) (:init (idle) (= (level) 0)) (:goal (>= (level) 15)))");
  EXPECT_TRUE(check_problem(
      "(define (problem x) (:domain tank) (:init (idle) (= (level) 0)) (:goal (>= (level) 15)))",
      p.domain()).plannable);
  SolveResult r = Planner().solve(p, quick());
  ASSERT_EQ(r.status, Status::Solved);
  EXPECT_TRUE(validate(p, *r.plan).valid());
  EXPECT_TRUE(r.plan->temporal);
}

// The note that explains an unreachable goal
std::string why(const std::string & domain, const std::string & problem_text)
{
  SolveResult r = Planner().solve(problem(domain, problem_text), quick());
  EXPECT_EQ(r.status, Status::Unsolvable);
  for (const auto & d : r.diagnostics) {
    if (d.code == "orchestrator.unreachable_goal") {
      return d.message;
    }
  }
  ADD_FAILURE() << "no explanation";
  return "";
}

const char * kTalk =
  "(define (domain talk) (:requirements :typing) (:types robot person room message)"
  " (:predicates (robot_at ?r - robot ?o - room) (person_at ?p - person ?o - room)"
  "  (near ?r - robot ?p - person) (told ?r - robot ?m - message ?p - person) (busy ?r - robot))"
  " (:action move :parameters (?r - robot ?a ?b - room)"
  "  :precondition (robot_at ?r ?a) :effect (and (not (robot_at ?r ?a)) (robot_at ?r ?b)))"
  " (:action approach :parameters (?r - robot ?o - room ?p - person)"
  "  :precondition (and (robot_at ?r ?o) (person_at ?p ?o)) :effect (near ?r ?p))"
  " (:action talk :parameters (?r - robot ?p - person ?m - message)"
  "  :precondition (and (near ?r ?p) (not (busy ?r))) :effect (told ?r ?m ?p)))";

TEST(PlannerApiTest, ExplainsAnUnreachableGoal)
{
  // Jack is in no room: told needs near, near needs person_at, which nothing adds
  std::string w = why(kTalk,
      "(define (problem p) (:domain talk) (:objects leia - robot jack - person"
      " kitchen bedroom - room m1 - message) (:init (robot_at leia kitchen))"
      " (:goal (told leia m1 jack)))");
  EXPECT_NE(w.find("(told leia m1 jack) can never be reached"), std::string::npos) << w;
  EXPECT_NE(w.find("talk adds it, but needs (near leia jack)"), std::string::npos) << w;
  EXPECT_NE(w.find("approach adds it, but needs (person_at jack ?o - room)"), std::string::npos)
    << w;
  EXPECT_NE(w.find("no action adds it, and the initial state does not have it"),
    std::string::npos) << w;
}

TEST(PlannerApiTest, ExplainsOnlyTheUnreachableGoalFacts)
{
  // (robot_at leia bedroom) is reachable; the other two are not, for different reasons
  std::string w = why(kTalk,
      "(define (problem p) (:domain talk) (:objects leia - robot jack ann - person"
      " kitchen bedroom - room m1 - message) (:init (robot_at leia kitchen))"
      " (:goal (and (robot_at leia bedroom) (told leia m1 jack) (person_at ann kitchen))))");
  EXPECT_EQ(w.find("(robot_at leia bedroom)"), std::string::npos) << w;
  EXPECT_NE(w.find("(told leia m1 jack) can never be reached"), std::string::npos) << w;
  EXPECT_NE(w.find("(person_at ann kitchen) can never be reached:\n"
    "  no action adds it"), std::string::npos) << w;
}

TEST(PlannerApiTest, ExplainsThroughDerivedPredicates)
{
  const char * domain =
    "(define (domain d) (:requirements :derived-predicates) (:predicates (a) (b) (c) (done))"
    " (:derived (b) (a))"
    " (:action finish :parameters () :precondition (b) :effect (done)))";
  std::string w = why(domain, "(define (problem p) (:domain d) (:init (c)) (:goal (done)))");
  EXPECT_NE(w.find("finish adds it, but needs (b)"), std::string::npos) << w;
  EXPECT_NE(w.find("b adds it, but needs (a)"), std::string::npos) << w;
}

const char * kDeliver =
  "(define (domain deliver) (:requirements :typing) (:types robot room item)"
  " (:predicates (at ?r - robot ?x - room) (in ?i - item ?x - room)"
  "  (holding ?r - robot ?i - item) (connected ?x ?y - room) (door-open ?x ?y - room))"
  " (:action move :parameters (?r - robot ?x ?y - room)"
  "  :precondition (and (at ?r ?x) (connected ?x ?y) (door-open ?x ?y))"
  "  :effect (and (not (at ?r ?x)) (at ?r ?y)))"
  " (:action pick :parameters (?r - robot ?i - item ?x - room)"
  "  :precondition (and (at ?r ?x) (in ?i ?x)) :effect (and (not (in ?i ?x)) (holding ?r ?i)))"
  " (:action drop :parameters (?r - robot ?i - item ?x - room)"
  "  :precondition (and (at ?r ?x) (holding ?r ?i))"
  "  :effect (and (not (holding ?r ?i)) (in ?i ?x))))";

std::string deliver_why(const std::string & statics)
{
  return why(kDeliver,
           "(define (problem p) (:domain deliver)"
           " (:objects r1 - robot kitchen hall office - room cup - item)"
           " (:init (at r1 hall) (in cup kitchen) (connected hall kitchen)"
           "  (connected kitchen hall) (door-open hall kitchen) (door-open kitchen hall) " +
           statics + ") (:goal (in cup office)))");
}

TEST(PlannerApiTest, ExplainsTheStaticFactThatIsMissing)
{
  // The room is connected but its door is closed: the reason is the door, not the connection
  std::string w = deliver_why("(connected hall office)");
  EXPECT_NE(w.find("move adds it, but needs (door-open ?x - room office)"), std::string::npos)
    << w;
  EXPECT_EQ(w.find("needs (connected"), std::string::npos) << w;
}

TEST(PlannerApiTest, ExplainsTheFirstStaticFactWhenItIsTheMissingOne)
{
  // The door is open but nothing connects the room: the connection
  std::string w = deliver_why("(door-open hall office)");
  EXPECT_NE(w.find("move adds it, but needs (connected ?x - room office)"), std::string::npos)
    << w;
  EXPECT_EQ(w.find("needs (door-open"), std::string::npos) << w;
}

TEST(PlannerApiTest, ExplainsAStaticFactThatHoldsOnlyForOtherObjects)
{
  // Both static predicates have facts, but none for the office: the first one is named, and the
  // chain ends at a fact no action adds
  std::string w = deliver_why("");
  EXPECT_NE(w.find("move adds it, but needs (connected ?x - room office)"), std::string::npos)
    << w;
  EXPECT_NE(w.find("no action adds it, and the initial state does not have it"),
    std::string::npos) << w;
}

TEST(PlannerApiTest, ExplainsAnActionThatNeverApplies)
{
  // The adder's positive preconditions hold: what blocks it is another kind of condition
  const char * domain =
    "(define (domain d) (:requirements :negative-preconditions) (:predicates (p) (q) (done))"
    " (:action a :parameters () :precondition (and (p) (not (p))) :effect (done)))";
  std::string w = why(domain, "(define (problem x) (:domain d) (:init (p)) (:goal (done)))");
  EXPECT_FALSE(w.empty());
}

TEST(PlannerApiTest, ExplanationsStayShort)
{
  // A chain of 10 facts: only the first levels are explained
  std::string domain = "(define (domain chain) (:predicates";
  for (int i = 0; i <= 10; ++i) {
    domain += " (f" + std::to_string(i) + ")";
  }
  domain += ")";
  for (int i = 1; i <= 10; ++i) {
    domain += " (:action a" + std::to_string(i) + " :parameters () :precondition (f" +
      std::to_string(i - 1) + ") :effect (f" + std::to_string(i) + "))";
  }
  domain += ")";
  std::string w = why(domain, "(define (problem p) (:domain chain) (:init) (:goal (f10)))");
  EXPECT_NE(w.find("a10 adds it, but needs (f9)"), std::string::npos) << w;
  EXPECT_EQ(w.find("a1 adds it"), std::string::npos) << w;
  EXPECT_LT(std::count(w.begin(), w.end(), '\n'), 10) << w;
}

TEST(PlannerApiTest, UnsupportedFeaturesAreReported)
{
  Problem p = problem(kProcessDurative, kProcessDurativeProblem);
  SolveResult r = Planner().solve(p, quick());
  EXPECT_EQ(r.status, Status::Unsupported);
  EXPECT_TRUE(has_code(r.diagnostics, "unsupported."));
}

TEST(PlannerApiTest, PlansWithProcesses)
{
  // The level reaches 10 at 5 s: the plan waits until then, stops, and ends there
  Problem p = problem(kProcess, kProcessProblem);
  SolveResult r = Planner().solve(p, quick());
  ASSERT_EQ(r.status, Status::Solved);
  ASSERT_TRUE(r.plan);
  ASSERT_EQ(r.plan->steps.size(), 1u);
  EXPECT_EQ(r.plan->steps[0].name, "stop");
  EXPECT_NEAR(r.plan->steps[0].time, 5.0, 1e-6);
  ASSERT_TRUE(r.plan->end);
  EXPECT_NEAR(*r.plan->end, 5.0, 1e-6);
  EXPECT_NEAR(r.plan->makespan(), 5.0, 1e-6);
}

TEST(PlannerApiTest, PddlPlusEventsFireOnTheirOwn)
{
  // The overflow at level 10 (5 s) floods, and the siren follows at once: no action does it
  const char * domain =
    "(define (domain tank) (:requirements :fluents :time :negative-preconditions)"
    " (:predicates (open) (flooded) (alarm)) (:functions (level))"
    " (:process filling :parameters () :precondition (open)"
    "  :effect (increase (level) (* #t 2)))"
    " (:event overflow :parameters () :precondition (and (>= (level) 10) (not (flooded)))"
    "  :effect (flooded))"
    " (:event siren :parameters () :precondition (and (flooded) (not (alarm))) :effect (alarm))"
    " (:action open-valve :parameters () :precondition (not (open)) :effect (open)))";
  Problem p = problem(domain,
      "(define (problem q) (:domain tank) (:init (= (level) 0)) (:goal (alarm)))");
  SolveResult r = Planner().solve(p, quick());
  ASSERT_EQ(r.status, Status::Solved);
  ASSERT_EQ(r.plan->steps.size(), 1u);
  EXPECT_EQ(r.plan->steps[0].name, "open-valve");
  ASSERT_TRUE(r.plan->end);
  EXPECT_NEAR(*r.plan->end, 5.0, 1e-6);
}

TEST(PlannerApiTest, PddlPlusPlansKeepTheGlobalConstraints)
{
  // At most 2 m/s: the car must stop accelerating in time, and still cover 10 m and stop
  const char * domain =
    "(define (domain car) (:requirements :fluents :time :negative-preconditions)"
    " (:predicates (running) (stopped)) (:functions (d) (v) (a))"
    " (:constraint limit :parameters () :condition (<= (v) 2))"
    " (:process move :parameters () :precondition (running)"
    "  :effect (and (increase (d) (* #t (v))) (increase (v) (* #t (a)))))"
    " (:action accelerate :parameters () :precondition (and (running) (< (a) 1))"
    "  :effect (increase (a) 1))"
    " (:action decelerate :parameters () :precondition (and (running) (> (a) -1))"
    "  :effect (decrease (a) 1))"
    " (:action stop :parameters () :precondition (and (running) (< (v) 0.1) (> (v) -0.1)"
    "  (= (a) 0)) :effect (and (not (running)) (stopped) (assign (v) 0))))";
  Problem p = problem(domain,
      "(define (problem q) (:domain car) (:init (running) (= (d) 0) (= (v) 0) (= (a) 0))"
      " (:goal (and (stopped) (>= (d) 10))))");
  SolveResult r = Planner().solve(p, quick());
  ASSERT_EQ(r.status, Status::Solved);  // solve validates: v never above 2
  EXPECT_GT(r.plan->makespan(), 5.0);  // 10 m at no more than 2 m/s
}

TEST(PlannerApiTest, PddlPlusIntegratesNonLinearDynamics)
{
  // Heating towards 30 degrees, faster the further: 20 -> 29 takes ln(10) / 0.5 = 4.6 s. Euler
  // steps of 0.1 s would say 4.5 s (too early: the continuous check rejects it)
  const char * domain =
    "(define (domain room) (:requirements :fluents :time) (:predicates (heating) (warm))"
    " (:functions (temp))"
    " (:process heat :parameters () :precondition (heating)"
    "  :effect (increase (temp) (* #t (* 0.5 (- 30 (temp))))))"
    " (:action on :parameters () :precondition () :effect (heating))"
    " (:action done :parameters () :precondition (>= (temp) 29) :effect (warm)))";
  Problem p = problem(domain,
      "(define (problem q) (:domain room) (:init (= (temp) 20)) (:goal (warm)))");
  SolveResult r = Planner().solve(p, quick());
  ASSERT_EQ(r.status, Status::Solved);
  ASSERT_EQ(r.plan->steps.size(), 2u);
  EXPECT_NEAR(r.plan->steps[1].time, 4.7, 1e-6);  // the first step of 0.1 s past 4.605 s
}

TEST(PlannerApiTest, PddlPlusFluentsWithoutValueWaitForOne)
{
  // (clock) is increased by a process from the start, but has no value until reset
  const char * domain =
    "(define (domain timer) (:requirements :fluents :time) (:predicates (rang))"
    " (:functions (clock))"
    " (:process tick :parameters () :precondition () :effect (increase (clock) (* #t 1)))"
    " (:action reset :parameters () :precondition () :effect (assign (clock) 0))"
    " (:action ring :parameters () :precondition (>= (clock) 1) :effect (rang)))";
  Problem p = problem(domain,
      "(define (problem q) (:domain timer) (:init) (:goal (rang)))");
  SolveResult r = Planner().solve(p, quick());
  ASSERT_EQ(r.status, Status::Solved);
  ASSERT_EQ(r.plan->steps.size(), 2u);
  EXPECT_EQ(r.plan->steps[0].name, "reset");
  EXPECT_NEAR(r.plan->steps[1].time - r.plan->steps[0].time, 1.0, 1e-6);
}

TEST(PlannerApiTest, PddlPlusWithDurativeActions)
{
  // The tank fills at 2 per second; closing takes 1 s and needs 10: from 5 s to 6 s
  Problem p = problem(
    "(define (domain mixed) (:requirements :fluents :time :durative-actions)"
    " (:predicates (on) (done)) (:functions (level))"
    " (:process fill :parameters () :precondition (on) :effect (increase (level) (* #t 2)))"
    " (:durative-action close :parameters () :duration (= ?duration 1)"
    "  :condition (at start (>= (level) 10)) :effect (and (at start (not (on))) (at end (done)))))",
    "(define (problem x) (:domain mixed) (:init (on) (= (level) 0)) (:goal (done)))");
  SolveResult r = Planner().solve(p, quick());
  ASSERT_EQ(r.status, Status::Solved);
  ASSERT_EQ(r.plan->steps.size(), 1u);
  EXPECT_NEAR(r.plan->steps[0].time, 5.0, 1e-6);
  EXPECT_NEAR(r.plan->steps[0].duration, 1.0, 1e-6);
  EXPECT_NEAR(r.plan->makespan(), 6.0, 1e-6);
}

TEST(PlannerApiTest, PddlPlusDurativeActionsWithContinuousEffectsAndBounds)
{
  // Pumping adds 1 per second, for 2 to 4 s: 3 needs 3 s. The over all condition (the valve
  // open) holds throughout
  Problem p = problem(
    "(define (domain pump) (:requirements :fluents :time :durative-actions"
    " :duration-inequalities) (:predicates (valve) (watching)) (:functions (level) (t))"
    " (:process clock :parameters () :precondition (watching) :effect (increase (t) (* #t 1)))"
    " (:durative-action pump :parameters () :duration (and (>= ?duration 2) (<= ?duration 4))"
    "  :condition (over all (valve)) :effect (increase (level) (* #t 1))))",
    "(define (problem x) (:domain pump) (:init (valve) (watching) (= (level) 0) (= (t) 0))"
    " (:goal (>= (level) 3)))");
  SolveResult r = Planner().solve(p, quick());
  ASSERT_EQ(r.status, Status::Solved);
  ASSERT_EQ(r.plan->steps.size(), 1u);
  EXPECT_GE(r.plan->steps[0].duration, 3.0 - 1e-6);
  EXPECT_LE(r.plan->steps[0].duration, 4.0 + 1e-6);
}

// Sanitizers and coverage run several times slower: time limits scale
#if defined(__SANITIZE_ADDRESS__) || defined(__SANITIZE_THREAD__) || defined(KAIROS_COVERAGE_BUILD)
constexpr int kSlowBuild = 20;
#elif defined(__has_feature)
#if __has_feature(address_sanitizer) || __has_feature(thread_sanitizer)
constexpr int kSlowBuild = 20;
#else
constexpr int kSlowBuild = 1;
#endif
#else
constexpr int kSlowBuild = 1;
#endif

// SMTPlan's linear generator: it runs for 1000 s, burning 1 per second; each tank refuels it at
// 2 per second for 10 s
std::string generator_domain()
{
  return "(define (domain gen) (:requirements :fluents :durative-actions :typing)"
         " (:types generator tank)"
         " (:predicates (ran) (available ?t - tank) (refueling ?g - generator))"
         " (:functions (fuel ?g - generator) (capacity ?g - generator))"
         " (:durative-action generate :parameters (?g - generator) :duration (= ?duration 1000)"
         "  :condition (over all (>= (fuel ?g) 0))"
         "  :effect (and (decrease (fuel ?g) (* #t 1)) (at end (ran))))"
         " (:durative-action refuel :parameters (?g - generator ?t - tank)"
         "  :duration (= ?duration 10)"
         "  :condition (and (at start (available ?t)) (over all (< (fuel ?g) (capacity ?g))))"
         "  :effect (and (at start (refueling ?g)) (increase (fuel ?g) (* #t 2))"
         "   (at start (not (available ?t))) (at end (not (refueling ?g))))))";
}

std::string generator_problem(int tanks, int fuel)
{
  std::string objects, init;
  for (int i = 1; i <= tanks; ++i) {
    objects += " t" + std::to_string(i);
    init += " (available t" + std::to_string(i) + ")";
  }
  return "(define (problem p) (:domain gen) (:objects g - generator" + objects + " - tank)"
         " (:init (= (fuel g) " + std::to_string(fuel) + ") (= (capacity g) 1000)" + init + ")"
         " (:goal (ran)))";
}

TEST(PlannerApiTest, PddlPlusSymmetricObjectsAreTriedInOneOrder)
{
  // Eight tanks the task does not tell apart, four refuels needed (950 + 4 * 20 - 1000 > 0):
  // one order of the tanks, not 8 * 7 * 6 * 5. The plan is found at once and is valid
  Problem p = problem(generator_domain(), generator_problem(8, 950));
  auto start = std::chrono::steady_clock::now();
  SolveResult r = Planner().solve(p, quick());
  ASSERT_EQ(r.status, Status::Solved);
  EXPECT_LT(std::chrono::steady_clock::now() - start, kSlowBuild * std::chrono::seconds(2));
  EXPECT_TRUE(validate(p, *r.plan).valid());
  int refuels = 0;
  for (const auto & s : r.plan->steps) {
    refuels += s.name == "refuel";
  }
  EXPECT_GE(refuels, 3);
}

TEST(PlannerApiTest, PddlPlusObjectsThatDifferAreNotSymmetric)
{
  // One tank was already used: the others are interchangeable, not that one
  std::string prob = generator_problem(3, 980);
  prob.replace(prob.find(" (available t1)"), std::string(" (available t1)").size(), "");
  Problem p = problem(generator_domain(), prob);
  SolveResult r = Planner().solve(p, quick());
  ASSERT_EQ(r.status, Status::Solved);
  EXPECT_TRUE(validate(p, *r.plan).valid());
  for (const auto & s : r.plan->steps) {
    if (s.name == "refuel") {
      EXPECT_NE(s.args.at(1), "t1");
    }
  }
}

TEST(PlannerApiTest, PddlPlusWaitsLongerThanOneLongWait)
{
  // A trip of 2500 s while a process runs: a long wait takes at most 20000 steps of 0.1 s
  // (2000 s), and one that ran them all was not applicable, so the trip never ended
  Problem p = problem(
    "(define (domain trip) (:requirements :fluents :time :durative-actions)"
    " (:predicates (on) (arrived)) (:functions (age))"
    " (:process aging :parameters () :precondition (on) :effect (increase (age) (* #t 1)))"
    " (:durative-action travel :parameters () :duration (= ?duration 2500)"
    "  :condition (at start (on)) :effect (at end (arrived))))",
    "(define (problem x) (:domain trip) (:init (on) (= (age) 0)) (:goal (arrived)))");
  SolveResult r = Planner().solve(p, quick());
  ASSERT_EQ(r.status, Status::Solved);
  EXPECT_TRUE(validate(p, *r.plan).valid());
}

TEST(PlannerApiTest, PddlPlusWithTimedInitialLiterals)
{
  // The gate opens at 3; the level (1 per second) must reach 3.5 too: through at 3.5
  Problem p = problem(
    "(define (domain gate) (:requirements :fluents :time :timed-initial-literals)"
    " (:predicates (gate) (through) (on)) (:functions (level))"
    " (:process fill :parameters () :precondition (on) :effect (increase (level) (* #t 1)))"
    " (:action pass :parameters () :precondition (and (gate) (>= (level) 3.5))"
    "  :effect (through)))",
    "(define (problem x) (:domain gate) (:init (on) (= (level) 0) (at 3 (gate)))"
    " (:goal (through)))");
  SolveResult r = Planner().solve(p, quick());
  ASSERT_EQ(r.status, Status::Solved);
  ASSERT_EQ(r.plan->steps.size(), 1u);
  EXPECT_NEAR(r.plan->steps[0].time, 3.5, 1e-6);
  // Before the gate opens: unsolvable at 2 (the literal at 3 comes too late)
  Problem late = problem(
    "(define (domain gate) (:requirements :fluents :time :timed-initial-literals)"
    " (:predicates (gate) (through) (on) (closed)) (:functions (level))"
    " (:process fill :parameters () :precondition (on) :effect (increase (level) (* #t 1)))"
    " (:action pass :parameters () :precondition (and (gate) (not (closed)))"
    "  :effect (through)))",
    "(define (problem x) (:domain gate) (:init (on) (= (level) 0) (at 3 (gate))"
    " (at 2 (closed))) (:goal (through)))");
  EXPECT_NE(Planner().solve(late, quick()).status, Status::Solved);
}

// --- Temporal planning (engine M2) -----------------------------------------------------------

const char * kTwoRobots =
  "(define (domain robots) (:requirements :typing :durative-actions)"
  " (:types robot room)"
  " (:predicates (at ?r - robot ?l - room) (connected ?a ?b - room) (cleaned ?l - room))"
  " (:durative-action move :parameters (?r - robot ?a ?b - room) :duration (= ?duration 5)"
  "  :condition (and (at start (at ?r ?a)) (over all (connected ?a ?b)))"
  "  :effect (and (at start (not (at ?r ?a))) (at end (at ?r ?b))))"
  " (:durative-action clean :parameters (?r - robot ?l - room) :duration (= ?duration 3)"
  "  :condition (over all (at ?r ?l)) :effect (at end (cleaned ?l))))";

const char * kTwoRobotsProblem =
  "(define (problem p) (:domain robots) (:objects r1 r2 - robot a b c d - room)"
  " (:init (at r1 a) (at r2 c) (connected a b) (connected c d))"
  " (:goal (and (cleaned b) (cleaned d))))";

TEST(PlannerApiTest, DurativeActionsWithEngineM2)
{
  Problem p = problem(kDurative, kDurativeProblem);
  SolveResult r = Planner().solve(p, quick());
  ASSERT_EQ(r.status, Status::Solved);
  ASSERT_TRUE(r.plan);
  EXPECT_TRUE(r.plan->temporal);
  ASSERT_EQ(r.plan->steps.size(), 1u);
  EXPECT_EQ(r.plan->steps[0].action(), "(a)");
  EXPECT_EQ(r.plan->steps[0].time, 0.0);
  EXPECT_EQ(r.plan->steps[0].duration, 1.0);
  EXPECT_TRUE(validate(p, *r.plan).valid());
  // M2 does not prove makespans optimal
  EXPECT_NE(r.stop_reason, StopReason::ProvenOptimal);
}

TEST(PlannerApiTest, IndependentWorkOverlaps)
{
  // Each robot moves and cleans: 8 time units each, both at once
  Problem p = problem(kTwoRobots, kTwoRobotsProblem);
  SolveResult r = Planner().solve(p, quick());
  ASSERT_EQ(r.status, Status::Solved);
  ASSERT_EQ(r.plan->steps.size(), 4u);
  EXPECT_TRUE(validate(p, *r.plan).valid());
  EXPECT_NEAR(r.plan->makespan(), 8.0, 1e-9);  // clean starts when the move ends (over all)
  EXPECT_NEAR(*r.plan->cost, 8.0, 1e-9);       // no metric: the makespan
}

TEST(PlannerApiTest, EnginesAndDurativeActions)
{
  Problem p = problem(kDurative, kDurativeProblem);
  SolveOptions m1 = quick();
  m1.engine = Engine::M1;
  SolveResult r1 = Planner().solve(p, m1);
  EXPECT_EQ(r1.status, Status::Unsupported);
  EXPECT_TRUE(has_code(r1.diagnostics, "unsupported.engine"));
  SolveOptions m2 = quick();
  m2.engine = Engine::M2;
  EXPECT_EQ(Planner().solve(p, m2).status, Status::Solved);
}

TEST(PlannerApiTest, RequiredConcurrencyWithEngineM3)
{
  // The fuse can only be mended while the match burns
  const char * cellar =
    "(define (domain cellar) (:requirements :typing :durative-actions)"
    " (:types match fuse)"
    " (:predicates (unused ?m - match) (light ?m - match) (mended ?f - fuse) (handfree))"
    " (:durative-action light-match :parameters (?m - match) :duration (= ?duration 8)"
    "  :condition (at start (unused ?m))"
    "  :effect (and (at start (not (unused ?m))) (at start (light ?m))"
    "   (at end (not (light ?m)))))"
    " (:durative-action mend :parameters (?f - fuse ?m - match) :duration (= ?duration 5)"
    "  :condition (and (at start (handfree)) (over all (light ?m)))"
    "  :effect (and (at start (not (handfree))) (at end (handfree)) (at end (mended ?f)))))";
  Problem p = problem(cellar,
      "(define (problem p) (:domain cellar) (:objects m1 - match f1 - fuse)"
      " (:init (unused m1) (handfree)) (:goal (mended f1)))");
  SolveResult r = Planner().solve(p, quick());
  ASSERT_EQ(r.status, Status::Solved);
  EXPECT_TRUE(validate(p, *r.plan).valid());
  // Mending happens while the match burns
  ASSERT_EQ(r.plan->steps.size(), 2u);
  const auto & light = r.plan->steps[0];
  const auto & mend = r.plan->steps[1];
  EXPECT_EQ(light.action(), "(light-match m1)");
  EXPECT_EQ(mend.action(), "(mend f1 m1)");
  EXPECT_GE(mend.time, light.time);
  EXPECT_LE(mend.time + mend.duration, light.time + light.duration);
  // Forcing M2: no plan, and that proves nothing
  SolveOptions m2 = quick();
  m2.engine = Engine::M2;
  SolveResult f = Planner().solve(p, m2);
  EXPECT_EQ(f.status, Status::NoPlanFound);
  EXPECT_TRUE(has_code(f.diagnostics, "orchestrator.forced_m2"));
  EXPECT_TRUE(has_code(f.diagnostics, "orchestrator.m2_incomplete"));
}

// Two robots lift a table: each one grips at start and needs the other's grip at its end. No
// value is transient, so the task is simple by value, yet only overlapping plans exist
const char * kLiftTable =
  "(define (domain lift) (:requirements :typing :durative-actions :negative-preconditions)"
  " (:types robot)"
  " (:predicates (partner ?r ?o - robot) (grip ?r - robot) (ready ?r - robot))"
  " (:durative-action prepare :parameters (?r ?o - robot) :duration (= ?duration 10)"
  "  :condition (and (at start (partner ?r ?o)) (at start (not (grip ?r)))"
  "   (at end (grip ?o)))"
  "  :effect (and (at start (grip ?r)) (at end (ready ?r)))))";

TEST(PlannerApiTest, SimpleByValueTaskThatNeedsOverlapIsSolved)
{
  Problem p = problem(kLiftTable,
      "(define (problem p) (:domain lift) (:objects r1 r2 - robot)"
      " (:init (partner r1 r2) (partner r2 r1)) (:goal (and (ready r1) (ready r2))))");
  // M2 runs out of states: M3 follows, in every mode
  for (int mode = 0; mode < 4; ++mode) {
    SolveOptions o = quick();
    o.anytime = mode % 2 == 0;
    o.threads = mode < 2 ? 1 : 4;
    o.search = mode == 1 ? "ff" : "auto";
    SolveResult r = Planner().solve(p, o);
    ASSERT_EQ(r.status, Status::Solved) << "mode " << mode;
    ASSERT_EQ(r.plan->steps.size(), 2u);
    EXPECT_TRUE(validate(p, *r.plan).valid());
    const auto & a = r.plan->steps[0];
    const auto & b = r.plan->steps[1];
    EXPECT_LT(a.time, b.time + b.duration);  // they overlap
    EXPECT_LT(b.time, a.time + a.duration);
  }
  // Forcing M2: no plan, and that proves nothing
  SolveOptions m2 = quick();
  m2.engine = Engine::M2;
  SolveResult f = Planner().solve(p, m2);
  EXPECT_EQ(f.status, Status::NoPlanFound);
  EXPECT_FALSE(has_code(f.diagnostics, "orchestrator.forced_m2"));  // simple by value
  EXPECT_TRUE(has_code(f.diagnostics, "orchestrator.m2_incomplete"));
}

TEST(PlannerApiTest, SimpleByValueTaskWithoutPlanIsUnsolvableWithoutTimes)
{
  // Each robot grips only if the other has not: no order of the happenings reaches the goal
  std::string d = kLiftTable;
  d.replace(d.find("(at end (grip ?o))"), 18, "(at start (not (grip ?o))) (at end (grip ?o))");
  Problem p = problem(d.c_str(),
      "(define (problem p) (:domain lift) (:objects r1 r2 - robot)"
      " (:init (partner r1 r2) (partner r2 r1)) (:goal (and (ready r1) (ready r2))))");
  for (unsigned threads : {1u, 4u}) {
    SolveOptions o = quick();
    o.threads = threads;
    SolveResult r = Planner().solve(p, o);
    EXPECT_EQ(r.status, Status::Unsolvable) << threads << " threads";
    EXPECT_FALSE(r.plan);
  }
}

TEST(PlannerApiTest, OverAllConditionsHoldInTheOpenInterval)
{
  // Time windows compiled as in IPC 2004's airport: a wrapper of 10 gives a phase marker over
  // all; idle (4) then work (6) must fill it exactly. Work needs idle's end only over all, so it
  // starts at that instant, and it ends with the wrapper: with epsilon separations no plan exists
  const char * domain =
    "(define (domain w) (:requirements :durative-actions)"
    " (:predicates (go) (global) (idle0) (idle1) (done))"
    " (:durative-action wrapper :parameters () :duration (= ?duration 10)"
    "  :condition (at start (go))"
    "  :effect (and (at start (not (go))) (at start (global)) (at start (idle0))"
    "   (at end (not (global)))))"
    " (:durative-action idle :parameters () :duration (= ?duration 4)"
    "  :condition (and (over all (idle0)) (over all (global)))"
    "  :effect (and (at end (idle1)) (at end (not (idle0)))))"
    " (:durative-action work :parameters () :duration (= ?duration 6)"
    "  :condition (and (over all (idle1)) (over all (global)))"
    "  :effect (and (at end (done)) (at end (not (idle1))))))";
  Problem p = problem(domain, "(define (problem x) (:domain w) (:init (go)) (:goal (done)))");
  SolveResult r = Planner().solve(p, quick());
  ASSERT_EQ(r.status, Status::Solved) << (r.diagnostics.empty() ? "" : r.diagnostics[0].message);
  EXPECT_TRUE(validate(p, *r.plan).valid());
  EXPECT_NEAR(r.plan->makespan(), 10.0, 1e-9);
}

TEST(PlannerApiTest, TimedInitialLiteralsWithEngineM3)
{
  const char * domain =
    "(define (domain w) (:requirements :durative-actions :timed-initial-literals)"
    " (:predicates (open) (done))"
    " (:durative-action work :parameters () :duration (= ?duration 4)"
    "  :condition (over all (open)) :effect (at end (done))))";
  // Open from 3 to 10: work starts when it opens
  Problem p = problem(domain,
      "(define (problem x) (:domain w) (:init (at 3 (open)) (at 10 (not (open))))"
      " (:goal (done)))");
  SolveResult r = Planner().solve(p, quick());
  ASSERT_EQ(r.status, Status::Solved);
  ASSERT_EQ(r.plan->steps.size(), 1u);
  EXPECT_NEAR(r.plan->steps[0].time, 3.0, 1e-9);
  EXPECT_TRUE(validate(p, *r.plan).valid());
  // Open from 3 to 6: no room for 4
  Problem q = problem(domain,
      "(define (problem x) (:domain w) (:init (at 3 (open)) (at 6 (not (open))))"
      " (:goal (done)))");
  SolveResult n = Planner().solve(q, quick());
  EXPECT_EQ(n.status, Status::Unsolvable);  // even ignoring deletes, it ends after 6
  EXPECT_TRUE(has_code(n.diagnostics, "orchestrator.deadlines"));
}

// --- Tasks from PlanSys2's test bank (kairos 1.0.0) ---------------------------------------------

const char * kClosingDoors =
  "(define (domain closing_doors)"
  " (:requirements :strips :typing :durative-actions :timed-initial-literals)"
  " (:types robot room door)"
  " (:predicates (robot_at ?r - robot ?l - room) (door_between ?d - door ?a - room ?b - room)"
  "  (door_open ?d - door))"
  " (:durative-action pass_door :parameters (?r - robot ?d - door ?from - room ?to - room)"
  "  :duration (= ?duration 4)"
  "  :condition (and (at start (robot_at ?r ?from)) (over all (door_between ?d ?from ?to))"
  "   (over all (door_open ?d)))"
  "  :effect (and (at start (not (robot_at ?r ?from))) (at end (robot_at ?r ?to)))))";

TEST(PlannerApiTest, ADoorThatClosesBeforeThePassageEndsIsUnsolvable)
{
  // Closes at 3, passing takes 4: it ran to the timeout (3 million states) without a proof
  Problem p = problem(kClosingDoors,
      "(define (problem p) (:domain closing_doors)"
      " (:objects r1 - robot corridor office - room office_door - door)"
      " (:init (robot_at r1 corridor) (door_open office_door)"
      "  (door_between office_door corridor office) (at 3 (not (door_open office_door))))"
      " (:goal (robot_at r1 office)))");
  for (unsigned threads : {1u, 4u}) {
    SolveOptions o = quick();
    o.threads = threads;
    auto start = std::chrono::steady_clock::now();
    SolveResult r = Planner().solve(p, o);
    EXPECT_EQ(r.status, Status::Unsolvable) << threads;
    EXPECT_TRUE(has_code(r.diagnostics, "orchestrator.deadlines")) << threads;
    EXPECT_LT(std::chrono::steady_clock::now() - start, kSlowBuild * 1s) << threads;
  }
}

TEST(PlannerApiTest, ADoorThatClosesLaterWithTwoRobotsEndsWithItsPlan)
{
  // The first plan in 2 ms, then the steps of time (for when there is no plan) went on for
  // 300 ms beside it with several threads
  Problem p = problem(kClosingDoors,
      "(define (problem p) (:domain closing_doors)"
      " (:objects r1 r2 - robot corridor office lab - room office_door lab_door - door)"
      " (:init (robot_at r1 corridor) (robot_at r2 corridor) (door_open office_door)"
      "  (door_between office_door corridor office) (door_between lab_door office lab)"
      "  (door_open lab_door) (at 9 (not (door_open office_door))))"
      " (:goal (and (robot_at r1 lab) (robot_at r2 office))))");
  for (unsigned threads : {1u, 4u}) {
    SolveOptions o = quick();
    o.threads = threads;
    auto start = std::chrono::steady_clock::now();
    SolveResult r = Planner().solve(p, o);
    ASSERT_EQ(r.status, Status::Solved) << threads;
    EXPECT_TRUE(validate(p, *r.plan).valid());
    EXPECT_NEAR(r.plan->makespan(), 8.001, 1e-6);
    EXPECT_EQ(r.stop_reason, StopReason::Exhausted) << threads;  // not stopped
    EXPECT_LT(r.stats.states, 5000u) << threads;  // 15 000 and more before
    EXPECT_LT(std::chrono::steady_clock::now() - start, kSlowBuild * 1s) << threads;
  }
}

TEST(PlannerApiTest, AChargeTooShortForThreeMovesEndsBeforeTheTimeout)
{
  // The longest charge (8 s) gives 80, three moves need 90. In steps of time the time grew for
  // ever and epsilon waits chained: the timeout, 2.5 million states. Now the steps run out of
  // states at once; not a proof (they may miss a plan that needs finer timing)
  Problem p = problem(
    "(define (domain recharge) (:requirements :strips :typing :durative-actions"
    " :numeric-fluents :duration-inequalities)"
    " (:types robot room charger)"
    " (:predicates (robot_at ?r - robot ?l - room) (connected ?a - room ?b - room)"
    "  (charger_at ?c - charger ?l - room) (charger_free ?c - charger))"
    " (:functions (battery ?r - robot))"
    " (:durative-action move :parameters (?r - robot ?from - room ?to - room)"
    "  :duration (= ?duration 5)"
    "  :condition (and (at start (robot_at ?r ?from)) (at start (>= (battery ?r) 30))"
    "   (over all (connected ?from ?to)))"
    "  :effect (and (at start (not (robot_at ?r ?from))) (at start (decrease (battery ?r) 30))"
    "   (at end (robot_at ?r ?to))))"
    " (:durative-action charge :parameters (?r - robot ?c - charger ?l - room)"
    "  :duration (and (>= ?duration 1) (<= ?duration 8))"
    "  :condition (and (at start (charger_free ?c)) (over all (charger_at ?c ?l))"
    "   (over all (robot_at ?r ?l)))"
    "  :effect (and (at start (not (charger_free ?c)))"
    "   (at end (increase (battery ?r) (* 10 ?duration))))))",
    "(define (problem p) (:domain recharge)"
    " (:objects r1 - robot dock hall office lab - room c1 - charger)"
    " (:init (robot_at r1 dock) (= (battery r1) 0) (charger_at c1 dock) (charger_free c1)"
    "  (connected dock hall) (connected hall office) (connected office lab))"
    " (:goal (robot_at r1 lab)))");
  for (unsigned threads : {1u, 4u}) {
    SolveOptions o = quick();
    o.timeout *= kSlowBuild;  // the coverage build ran out of 10 s
    o.threads = threads;
    auto start = std::chrono::steady_clock::now();
    SolveResult r = Planner().solve(p, o);
    EXPECT_EQ(r.status, Status::NoPlanFound) << threads;
    EXPECT_TRUE(has_code(r.diagnostics, "orchestrator.m3_exhausted")) << threads;
    EXPECT_TRUE(has_code(r.diagnostics, "orchestrator.steps_exhausted")) << threads;
    EXPECT_LT(std::chrono::steady_clock::now() - start, kSlowBuild * 2s) << threads;
  }
}

TEST(PlannerApiTest, StepsOfTimeThatRunOutOfTimeAreATimeout)
{
  // M3 runs out of states (charges of 1 s: 10 each), the steps of time do not (a charge of
  // any length): "no plan, more time does not help" was wrong, more time may help
  Problem p = problem(
    "(define (domain endless) (:requirements :typing :durative-actions :numeric-fluents"
    " :duration-inequalities) (:types robot)"
    " (:predicates (free ?r - robot) (done ?r - robot))"
    " (:functions (battery ?r - robot))"
    " (:durative-action charge :parameters (?r - robot) :duration (>= ?duration 1)"
    "  :condition (at start (free ?r))"
    "  :effect (and (at start (not (free ?r)))"
    "   (at end (increase (battery ?r) (* 10 ?duration)))))"
    " (:durative-action work :parameters (?r - robot) :duration (= ?duration 1)"
    "  :condition (at start (and (>= (battery ?r) 30) (<= (battery ?r) 20)))"
    "  :effect (at end (done ?r))))",
    "(define (problem p) (:domain endless) (:objects r - robot)"
    " (:init (free r) (= (battery r) 0)) (:goal (done r)))");
  for (unsigned threads : {1u, 4u}) {
    SolveOptions o;
    o.timeout = 1s;
    o.threads = threads;
    SolveResult r = Planner().solve(p, o);
    EXPECT_EQ(r.status, Status::Timeout) << threads;
    EXPECT_TRUE(has_code(r.diagnostics, "orchestrator.m3_exhausted")) << threads;
    // Stopped by the caller meanwhile: cancelled (it was "no plan found")
    SearchOptions so;
    so.timeout = 30s;
    so.threads = threads;
    Search s = Planner().start(p, so);
    std::this_thread::sleep_for(300ms);
    s.stop();
    EXPECT_EQ(s.wait().status, Status::Cancelled) << threads;
  }
}

TEST(PlannerApiTest, RequiredConcurrencyThatALoneRobotCannotGiveIsUnsolvable)
{
  // Holding the door needs the robot in the room over all, passing takes it out: no plan.
  // M3 ran out of states, which proves nothing; without durations nor times it does
  const char * domain =
    "(define (domain door_holding) (:requirements :strips :typing :durative-actions)"
    " (:types robot room door)"
    " (:predicates (robot_at ?r - robot ?l - room) (door_between ?d - door ?a - room ?b - room)"
    "  (door_open ?d - door))"
    " (:durative-action hold_door :parameters (?r - robot ?d - door ?l - room ?other - room)"
    "  :duration (= ?duration 10)"
    "  :condition (and (over all (robot_at ?r ?l)) (over all (door_between ?d ?l ?other)))"
    "  :effect (and (at start (door_open ?d)) (at end (not (door_open ?d)))))"
    " (:durative-action pass_door :parameters (?r - robot ?d - door ?from - room ?to - room)"
    "  :duration (= ?duration 4)"
    "  :condition (and (at start (robot_at ?r ?from)) (over all (door_between ?d ?from ?to))"
    "   (over all (door_open ?d)))"
    "  :effect (and (at start (not (robot_at ?r ?from))) (at end (robot_at ?r ?to)))))";
  auto task = [&](const std::string & robots, const std::string & at) {
      return problem(domain, "(define (problem p) (:domain door_holding)"
               " (:objects " + robots + " - robot corridor office - room office_door - door)"
               " (:init " + at + " (door_between office_door corridor office)"
               "  (door_between office_door office corridor))"
               " (:goal (robot_at walker office)))");
    };
  Problem alone = task("walker", "(robot_at walker corridor)");
  for (unsigned threads : {1u, 4u}) {
    SolveOptions o = quick();
    o.threads = threads;
    SolveResult r = Planner().solve(alone, o);
    EXPECT_EQ(r.status, Status::Unsolvable) << threads;
    auto why = std::find_if(r.diagnostics.begin(), r.diagnostics.end(),
        [](const Diagnostic & d) {return d.code == "orchestrator.why";});
    ASSERT_NE(why, r.diagnostics.end()) << threads;
    EXPECT_NE(why->message.find("without durations and times"), std::string::npos);
    EXPECT_FALSE(has_code(r.diagnostics, "orchestrator.m3_exhausted")) << threads;
  }
  // With a teammate to hold it: a plan
  Problem two = task("walker holder", "(robot_at walker corridor) (robot_at holder corridor)");
  SolveResult r = Planner().solve(two, quick());
  ASSERT_EQ(r.status, Status::Solved);
  EXPECT_TRUE(validate(two, *r.plan).valid());
}

TEST(PlannerApiTest, TheGoalHoldsAfterTheLastTimedLiteral)
{
  // As VAL checks it: a literal at 1 sets the value back, so the task must come after it (UPF's
  // basic_numeric_with_timed_effect: M3 planned it at 0 and every plan failed validation, "a bug
  // in Kairos"). A goal state of M3 counts once its plan is valid
  const char * domain =
    "(define (domain d) (:requirements :numeric-fluents :timed-initial-literals)"
    " (:predicates (p)) (:functions (value))"
    " (:action task :parameters () :precondition (= (value) 1) :effect (assign (value) 2)))";
  Problem later = problem(domain, "(define (problem x) (:domain d)"
      " (:init (= (value) 1) (at 1 (= (value) 1))) (:goal (= (value) 2)))");
  for (unsigned threads : {1u, 4u}) {
    SolveOptions o = quick();
    o.threads = threads;
    SolveResult r = Planner().solve(later, o);
    ASSERT_EQ(r.status, Status::Solved) << threads;
    EXPECT_TRUE(validate(later, *r.plan).valid());
    ASSERT_FALSE(r.plan->steps.empty());
    EXPECT_GT(r.plan->steps.back().time, 1.0);  // the last task after the literal
  }
  // A literal after any plan takes the goal away for good: no plan, proved
  Problem never = problem(domain, "(define (problem x) (:domain d)"
      " (:init (p) (= (value) 1) (at 5 (not (p)))) (:goal (p)))");
  EXPECT_EQ(Planner().solve(never, quick()).status, Status::Unsolvable);
}

TEST(PlannerApiTest, DeadlinesKnowWhatAStartGivesItselfAndDurationsThatChange)
{
  // Busy over all, set by work's own start; and a duration (10) that tune shortens to 1: both
  // fit before the light goes at 5, and were taken as too late
  Problem busy = problem(
    "(define (domain s) (:requirements :durative-actions :timed-initial-literals)"
    " (:predicates (light) (busy) (done))"
    " (:durative-action work :parameters () :duration (= ?duration 2)"
    "  :condition (and (at start (light)) (over all (busy)))"
    "  :effect (and (at start (busy)) (at end (done)))))",
    "(define (problem x) (:domain s) (:init (light) (at 5 (not (light)))) (:goal (done)))");
  SolveResult b = Planner().solve(busy, quick());
  ASSERT_EQ(b.status, Status::Solved);
  EXPECT_TRUE(validate(busy, *b.plan).valid());
  Problem tune = problem(
    "(define (domain v) (:requirements :durative-actions :numeric-fluents"
    " :timed-initial-literals)"
    " (:predicates (open) (done)) (:functions (len))"
    " (:action tune :parameters () :precondition () :effect (assign (len) 1))"
    " (:durative-action work :parameters () :duration (= ?duration (len))"
    "  :condition (over all (open)) :effect (at end (done))))",
    "(define (problem x) (:domain v) (:init (open) (= (len) 10) (at 5 (not (open))))"
    " (:goal (done)))");
  SolveResult t = Planner().solve(tune, quick());
  ASSERT_EQ(t.status, Status::Solved);
  EXPECT_TRUE(validate(tune, *t.plan).valid());
}

// Timed initial fluents read where M2 and M3 keep them apart from the snap actions: a duration
// (rush hour triples a travel time) and an over all condition (power only in windows). The
// states dropped them as unread, so the travel kept its old time (a plan the validator
// rejected) and the power never came (a false "unsolvable")
TEST(PlannerApiTest, TimedInitialFluentsInDurationsAndOverAllConditions)
{
  Problem traffic = problem(
    "(define (domain traffic) (:requirements :typing :durative-actions :numeric-fluents"
    " :timed-initial-literals) (:types car place)"
    " (:predicates (at ?c - car ?p - place) (road ?a ?b - place))"
    " (:functions (travel ?a ?b - place))"
    " (:durative-action drive :parameters (?c - car ?a ?b - place)"
    "  :duration (= ?duration (travel ?a ?b))"
    "  :condition (and (at start (at ?c ?a)) (over all (road ?a ?b)))"
    "  :effect (and (at start (not (at ?c ?a))) (at end (at ?c ?b)))))",
    "(define (problem p) (:domain traffic) (:objects c - car x0 x1 x2 - place)"
    " (:init (at c x0) (road x0 x1) (road x1 x2) (= (travel x0 x1) 6) (= (travel x1 x2) 6)"
    "  (at 5 (= (travel x1 x2) 18)))"
    " (:goal (at c x2)))");
  SolveResult r = Planner().solve(traffic, quick());
  ASSERT_EQ(r.status, Status::Solved);
  EXPECT_TRUE(validate(traffic, *r.plan).valid());
  ASSERT_EQ(r.plan->steps.size(), 2u);
  EXPECT_NEAR(r.plan->steps[1].duration, 18.0, 1e-6);  // it starts after rush hour begins
  auto charger = [](const std::string & windows) {
      return problem(
        "(define (domain charger) (:requirements :typing :durative-actions :numeric-fluents"
        " :timed-initial-literals) (:types robot)"
        " (:predicates (done ?r - robot))"
        " (:functions (energy ?r - robot) (power))"
        " (:durative-action charge :parameters (?r - robot) :duration (= ?duration 2)"
        "  :condition (over all (>= (power) 5))"
        "  :effect (at end (increase (energy ?r) 10)))"
        " (:durative-action work :parameters (?r - robot) :duration (= ?duration 3)"
        "  :condition (at start (>= (energy ?r) 5))"
        "  :effect (and (at start (decrease (energy ?r) 5)) (at end (done ?r)))))",
        "(define (problem p) (:domain charger) (:objects r - robot)"
        " (:init (= (energy r) 0) (= (power) 0) " + windows + ")"
        " (:goal (done r)))");
    };
  Problem window = charger("(at 1 (= (power) 8)) (at 7 (= (power) 0))");
  SolveResult w = Planner().solve(window, quick());
  ASSERT_EQ(w.status, Status::Solved);
  EXPECT_TRUE(validate(window, *w.plan).valid());
  EXPECT_GE(w.plan->steps[0].time, 1.0 - 1e-9);  // charging waits for the power
  // A window too short for a charge of 2: no plan
  EXPECT_EQ(Planner().solve(charger("(at 1 (= (power) 8)) (at 2 (= (power) 0))"), quick()).status,
    Status::NoPlanFound);  // M3 and the steps of time: no proof
}

// The pruning of M3 by deadlines (a literal takes a fact away for good) reached only the true
// value of a derived atom: "busy" while holding something could never turn false, and the
// robot that had to put its load down before the window closed was "unsolvable" (airport with
// time windows, compiled with derived atoms)
TEST(PlannerApiTest, DeadlinesLetDerivedAtomsTurnFalse)
{
  auto window = [](const std::string & closes) {
      return problem(
        "(define (domain w) (:requirements :durative-actions :derived-predicates"
        " :negative-preconditions :timed-initial-literals)"
        " (:predicates (holding) (busy) (open) (done))"
        " (:derived (busy) (holding))"
        " (:durative-action put-down :parameters () :duration (= ?duration 2)"
        "  :condition (at start (holding)) :effect (at end (not (holding))))"
        " (:durative-action work :parameters () :duration (= ?duration 3)"
        "  :condition (and (at start (not (busy))) (over all (open)))"
        "  :effect (at end (done))))",
        "(define (problem x) (:domain w) (:init (holding) (open) (at " + closes +
        " (not (open)))) (:goal (done)))");
    };
  // Put down by 2, work from 2.001 to 5.001: the window may close then, at the end of the work
  // (over all holds in the open interval)
  for (const char * closes : {"10", "5.001"}) {
    Problem p = window(closes);
    SolveResult r = Planner().solve(p, quick());
    ASSERT_EQ(r.status, Status::Solved) << closes;
    EXPECT_TRUE(validate(p, *r.plan).valid());
  }
  EXPECT_NE(Planner().solve(window("4"), quick()).status, Status::Solved);
}

TEST(PlannerApiTest, TemporallySimpleIpcDomains)
{
  // M2 on the temporally simple domains of IPC 2011: plans validate, or time runs out; never
  // an invalid plan
  const char * dir = std::getenv("KAIROS_PDDL_CORPUS");
  if (dir == nullptr) {
    GTEST_SKIP() << "KAIROS_PDDL_CORPUS not set";
  }
  fs::path base = fs::path(dir) / "potassco_pddl-instances" / "ipc-2011" / "domains";
  size_t solved = 0;
  for (const char * d : {"crew-planning-temporal-satisficing", "elevator-temporal-satisficing",
      "floor-tile-temporal-satisficing", "openstacks-temporal-satisficing",
      "parc-printer-temporal-satisficing", "parking-temporal-satisficing",
      "peg-solitaire-temporal-satisficing", "sokoban-temporal-satisficing",
      "storage-temporal-satisficing"})
  {
    fs::path folder = base / d;
    fs::path domain = folder / "domain.pddl";
    if (!fs::exists(domain)) {
      domain = folder / "domains" / "domain-1.pddl";
    }
    auto dm = load_domain(domain);
    ASSERT_TRUE(dm.ok()) << d;
    auto pr = load_problem(folder / "instances" / "instance-1.pddl", *dm);
    ASSERT_TRUE(pr.ok()) << d;
    SolveOptions o;
    o.timeout = 20s;
    o.anytime = false;
    SolveResult r = Planner().solve(*pr, o);
    EXPECT_TRUE(r.status == Status::Solved || r.status == Status::Timeout ||
      r.status == Status::OutOfMemory) << d << ": " << to_string(r.status) <<
      (r.diagnostics.empty() ? "" : " " + r.diagnostics[0].message);
    if (r.status == Status::Solved) {
      ++solved;
      EXPECT_TRUE(r.plan->temporal) << d;
      EXPECT_TRUE(validate(*pr, *r.plan).valid()) << d;
    }
  }
  EXPECT_GE(solved, 6u);
}

TEST(PlannerApiTest, TimesOut)
{
  Problem p = problem(kHoles, holes_problem(11, 10));
  SolveOptions o;
  o.timeout = 150ms;
  auto t0 = std::chrono::steady_clock::now();
  SolveResult r = Planner().solve(p, o);
  auto took = std::chrono::steady_clock::now() - t0;
  EXPECT_EQ(r.status, Status::Timeout);
  EXPECT_EQ(r.stop_reason, StopReason::Timeout);
  EXPECT_FALSE(r.plan);
  EXPECT_LT(took, 3s);
}

TEST(PlannerApiTest, StopTokenCancels)
{
  Problem p = problem(kHoles, holes_problem(11, 10));
  std::stop_source source;
  source.request_stop();
  SolveResult before = Planner().solve(p, quick(), source.get_token());
  EXPECT_EQ(before.status, Status::Cancelled);
  EXPECT_EQ(before.stop_reason, StopReason::Stopped);

  std::stop_source later;
  std::jthread stopper([&later]() {
      std::this_thread::sleep_for(100ms);
      later.request_stop();
    });
  auto t0 = std::chrono::steady_clock::now();
  SolveResult r = Planner().solve(p, quick(), later.get_token());
  EXPECT_EQ(r.status, Status::Cancelled);
  EXPECT_EQ(r.stop_reason, StopReason::Stopped);
  EXPECT_LT(std::chrono::steady_clock::now() - t0, 3s);
}

// The sanitizers' shadow memory inflates the RSS the budget watches: a larger limit there
// (ThreadSanitizer's the most: with 8 MB it stopped before the first expansion)
#if defined(__SANITIZE_THREAD__)
constexpr size_t kSearchMemoryMb = 128;
#elif defined(__SANITIZE_ADDRESS__)
constexpr size_t kSearchMemoryMb = 64;
#elif defined(__has_feature)
#if __has_feature(thread_sanitizer)
constexpr size_t kSearchMemoryMb = 128;
#elif __has_feature(address_sanitizer)
constexpr size_t kSearchMemoryMb = 64;
#else
constexpr size_t kSearchMemoryMb = 8;
#endif
#else
constexpr size_t kSearchMemoryMb = 8;
#endif

TEST(PlannerApiTest, MemoryLimitStopsTheSearch)
{
  // Wide (380 actions) and with a cheap heuristic: the limit fills in thousands of states, fast
  // even in unoptimized coverage builds
  Problem p = problem(kHoles, holes_problem(20, 19));
  SolveOptions o;
  o.timeout = 60s;
  o.memory_limit_mb = kSearchMemoryMb;
  o.search = "ff";
  SolveResult r = Planner().solve(p, o);
  EXPECT_EQ(r.status, Status::OutOfMemory);
  EXPECT_EQ(r.stop_reason, StopReason::MemoryLimit);
  EXPECT_EQ(r.stats.memory_limit_bytes, kSearchMemoryMb << 20);
  EXPECT_GT(r.stats.states, 1u);  // stopped in the search (sanitizers reach the limit sooner)
}

TEST(PlannerApiTest, HugeGroundingIsReported)
{
  // 40^6 instances of an action without preconditions
  std::string objects;
  for (int i = 0; i < 40; ++i) {
    objects += " o" + std::to_string(i);
  }
  Problem p = problem(
    "(define (domain big) (:requirements :strips) (:predicates (done ?a ?b ?c ?d ?e ?f))\n"
    " (:action a :parameters (?a ?b ?c ?d ?e ?f) :precondition (and)\n"
    "  :effect (done ?a ?b ?c ?d ?e ?f)))",
    "(define (problem x) (:domain big) (:objects" + objects + ") (:init)"
    " (:goal (done o0 o1 o2 o3 o4 o5)))");
  SolveOptions o;
  o.timeout = 60s;
  o.memory_limit_mb = 16;
  SolveResult r = Planner().solve(p, o);
  EXPECT_EQ(r.status, Status::GroundingTooLarge);
  EXPECT_EQ(r.stop_reason, StopReason::MemoryLimit);
}

TEST(PlannerApiTest, TargetCostStopsAtAGoodEnoughPlan)
{
  Problem p = problem(kRoutes, kRoutesProblem);
  SearchOptions o;
  o.timeout = 10s;
  o.target_cost = 100.0;  // any plan
  SolveResult r = Planner().start(p, o).wait();
  ASSERT_EQ(r.status, Status::Solved);
  EXPECT_EQ(r.stop_reason, StopReason::TargetReached);
  EXPECT_EQ(r.stats.plans, 1u);
}

TEST(PlannerApiTest, StopsWhenNoPlanImproves)
{
  // The first plan comes fast; proving it optimal needs the whole space
  Problem p = problem(kHoles, holes_problem(12, 12));
  SearchOptions o;
  o.timeout = 20s;
  o.stop_after_no_improvement = 200ms;
  auto t0 = std::chrono::steady_clock::now();
  SolveResult r = Planner().start(p, o).wait();
  EXPECT_LT(std::chrono::steady_clock::now() - t0, 10s);
  ASSERT_EQ(r.status, Status::Solved);
  EXPECT_EQ(r.stop_reason, StopReason::NoImprovement);
  EXPECT_EQ(r.plan->steps.size(), 12u);
}

TEST(PlannerApiTest, LogsThroughTheCallback)
{
  Problem p = problem(kRobots, kRobotsProblem);
  std::mutex mutex;
  std::vector<std::pair<LogChannel, std::string>> messages;
  SolveOptions o = quick();
  o.log.level = LogLevel::Debug;
  o.log.callback = [&](LogLevel, LogChannel channel, std::string_view text) {
      std::lock_guard<std::mutex> lock(mutex);
      messages.emplace_back(channel, std::string(text));
    };
  ASSERT_EQ(Planner().solve(p, o).status, Status::Solved);
  bool orchestrator = false, search = false;
  for (const auto & [channel, text] : messages) {
    orchestrator = orchestrator || channel == LogChannel::Orchestrator;
    search = search || channel == LogChannel::Search;
  }
  EXPECT_TRUE(orchestrator);
  EXPECT_TRUE(search);
}

TEST(PlannerApiTest, DumpsIntermediateStructures)
{
  Problem p = problem(kRobots, kRobotsProblem);
  fs::path dir = temp_dir("dump");
  SolveOptions o = quick();
  o.dump_dir = dir;
  ASSERT_EQ(Planner().solve(p, o).status, Status::Solved);
  for (const char * f : {"ast-domain.pddl", "ast-problem.pddl", "lifted.txt", "ground.txt",
      "sas.txt", "plan-trace.txt"})
  {
    EXPECT_TRUE(fs::exists(dir / f)) << f;
    EXPECT_GT(fs::file_size(dir / f), 0u) << f;
  }
  // The dumped PDDL parses again
  std::ifstream in(dir / "ast-domain.pddl");
  std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
  EXPECT_TRUE(parse_domain(text).ok());

  fs::path only = temp_dir("dump_sas");
  o.dump_dir = only;
  o.dump = {"sas"};
  ASSERT_EQ(Planner().solve(p, o).status, Status::Solved);
  EXPECT_TRUE(fs::exists(only / "sas.txt"));
  EXPECT_FALSE(fs::exists(only / "ground.txt"));
  EXPECT_FALSE(fs::exists(only / "ast-domain.pddl"));
  fs::remove_all(dir);
  fs::remove_all(only);
}

// --- Several threads (phases at once) ---------------------------------------------------------

Problem example(const std::string & domain, const std::string & problem_file)
{
  auto d = load_domain(fs::path(KAIROS_EXAMPLES_DIR) / domain);
  EXPECT_TRUE(d.ok());
  auto p = load_problem(fs::path(KAIROS_EXAMPLES_DIR) / problem_file, *d);
  EXPECT_TRUE(p.ok());
  return *p;
}

TEST(PlannerApiTest, PhasesInParallel)
{
  // The factory: M2 first, then the M3 portfolio; with three threads they all run at once
  Problem p = example("factory.pddl", "factory_problem_1.pddl");
  SearchOptions o;
  o.timeout = 60s;
  o.threads = 3;
  o.target_cost = 70.1;
  std::mutex mutex;
  std::vector<uint32_t> indices;
  std::vector<double> costs;
  SolveResult r = Planner().start(p, o, [&](const PlanUpdate & u) {
        std::lock_guard<std::mutex> lock(mutex);
        indices.push_back(u.index);
        costs.push_back(*u.plan.cost);
      }).wait();
  ASSERT_EQ(r.status, Status::Solved);
  EXPECT_EQ(r.stop_reason, StopReason::TargetReached);
  EXPECT_TRUE(validate(p, *r.plan).valid());
  // Delivered in order, numbered without gaps, each better than the one before
  ASSERT_FALSE(indices.empty());
  for (size_t i = 0; i < indices.size(); ++i) {
    EXPECT_EQ(indices[i], i);
    if (i > 0) {
      EXPECT_LT(costs[i], costs[i - 1]);
    }
  }
  EXPECT_EQ(r.stats.plans, indices.size());
}

TEST(PlannerApiTest, AProofInOneThreadStopsTheOthers)
{
  // Numeric, so two phases (lazy LAMA, then LAMA with novelty): 0, 2, 4, 6 never makes 3
  const char * domain =
    "(define (domain c) (:requirements :fluents) (:predicates (go)) (:functions (x))"
    " (:action inc :parameters () :precondition (and (go) (< (x) 6))"
    "  :effect (increase (x) 2)))";
  Problem p = problem(domain,
      "(define (problem q) (:domain c) (:init (go) (= (x) 0)) (:goal (= (x) 3)))");
  SolveOptions o = quick();
  for (unsigned threads : {1u, 2u, 16u}) {
    o.threads = threads;
    auto start = std::chrono::steady_clock::now();
    SolveResult r = Planner().solve(p, o);
    EXPECT_EQ(r.status, Status::Unsolvable) << threads;
    EXPECT_LT(std::chrono::steady_clock::now() - start, 5s) << threads;
  }
}

TEST(PlannerApiTest, StoppingSeveralThreads)
{
  // Pigeons that never fit: every phase searches until stopped
  Problem p = problem(kHoles, holes_problem(12, 11));
  SearchOptions o;
  o.timeout = 60s;
  o.threads = 4;
  o.search = "auto";
  Search s = Planner().start(p, o);
  std::this_thread::sleep_for(200ms);
  auto start = std::chrono::steady_clock::now();
  s.stop();
  SolveResult r = s.wait();
  EXPECT_LT(std::chrono::steady_clock::now() - start, 2s);
  EXPECT_EQ(r.status, Status::Cancelled);
}

TEST(PlannerApiTest, LogFromSeveralThreads)
{
  // The callback is called by one thread at a time
  Problem p = example("factory.pddl", "factory_problem_1.pddl");
  SolveOptions o;
  o.timeout = 3s;
  o.threads = 4;
  std::atomic<int> inside{0};
  std::atomic<bool> overlap{false};
  std::atomic<int> messages{0};
  o.log.level = LogLevel::Info;
  o.log.callback = [&](LogLevel, LogChannel, std::string_view) {
      if (inside.fetch_add(1) > 0) {
        overlap = true;
      }
      std::this_thread::sleep_for(std::chrono::microseconds(50));
      ++messages;
      inside.fetch_sub(1);
    };
  SolveResult r = Planner().solve(p, o);
  EXPECT_EQ(r.status, Status::Solved);
  EXPECT_GT(messages.load(), 5);
  EXPECT_FALSE(overlap);
}

TEST(PlannerApiTest, OneProblemFromSeveralThreads)
{
  Problem p = problem(kRoutes, kRoutesProblem);
  std::vector<SolveResult> results(4);
  {
    std::vector<std::jthread> threads;
    for (auto & r : results) {
      threads.emplace_back([&p, &r]() {r = Planner().solve(p, quick());});
    }
  }
  for (const auto & r : results) {
    ASSERT_EQ(r.status, Status::Solved);
    EXPECT_EQ(r.plan->cost, 6.0);
    EXPECT_EQ(*r.plan, *results[0].plan);
  }
}

// --- start(): anytime in its own thread -------------------------------------------------------

TEST(PlannerApiTest, AnytimeSearchHandsOverEachBetterPlan)
{
  Problem p = problem(kBlocks, kBlocksProblem);
  std::mutex mutex;
  std::vector<double> costs;
  std::vector<uint32_t> indexes;
  Search s = Planner().start(p, {}, [&](const PlanUpdate & u) {
        std::lock_guard<std::mutex> lock(mutex);
        costs.push_back(*u.plan.cost);
        indexes.push_back(u.index);
      });
  auto first = s.wait_first(10s);
  ASSERT_TRUE(first);
  SolveResult r = s.wait();
  EXPECT_FALSE(s.running());
  ASSERT_EQ(r.status, Status::Solved);
  EXPECT_EQ(s.stop_reason(), StopReason::ProvenOptimal);
  EXPECT_EQ(*s.best(), *r.plan);
  // Every plan once, in order, each cheaper than the one before (greedy is not optimal here)
  ASSERT_EQ(costs.size(), r.stats.plans);
  EXPECT_GE(costs.size(), 2u);
  std::vector<Plan> seen;
  while (auto next = s.next_plan(0ms)) {
    seen.push_back(*next);
  }
  ASSERT_EQ(seen.size(), costs.size());
  EXPECT_EQ(seen.front(), *first);
  EXPECT_EQ(seen.back(), *r.plan);
  for (size_t i = 0; i < costs.size(); ++i) {
    EXPECT_EQ(indexes[i], i);
    EXPECT_EQ(*seen[i].cost, costs[i]);
    if (i > 0) {
      EXPECT_LT(costs[i], costs[i - 1]);
    }
  }
  EXPECT_FALSE(s.next_plan(10ms));
  // wait() again gives the same result
  EXPECT_EQ(s.wait().status, Status::Solved);
}

TEST(PlannerApiTest, StopEndsARunningSearch)
{
  Problem p = problem(kHoles, holes_problem(11, 10));
  Search s = Planner().start(p);
  EXPECT_FALSE(s.wait_first(100ms));
  EXPECT_TRUE(s.running());
  EXPECT_FALSE(s.best());
  auto t0 = std::chrono::steady_clock::now();
  s.stop();
  SolveResult r = s.wait();
  EXPECT_LT(std::chrono::steady_clock::now() - t0, 3s);
  EXPECT_EQ(r.status, Status::Cancelled);
  EXPECT_EQ(r.stop_reason, StopReason::Stopped);
  EXPECT_FALSE(s.running());
}

TEST(PlannerApiTest, DestroyingARunningSearchStopsIt)
{
  Problem p = problem(kHoles, holes_problem(11, 10));
  auto t0 = std::chrono::steady_clock::now();
  {
    Search s = Planner().start(p);
    std::this_thread::sleep_for(50ms);
  }
  EXPECT_LT(std::chrono::steady_clock::now() - t0, 3s);
  // Also after stop(), after wait(), and when moved from
  {
    Search s = Planner().start(p);
    s.stop();
  }
  {
    Search s = Planner().start(problem(kRobots, kRobotsProblem));
    s.wait();
  }
  {
    Search a = Planner().start(p);
    Search b = std::move(a);
    b.stop();
    EXPECT_EQ(b.wait().status, Status::Cancelled);
  }
}

TEST(PlannerApiTest, AThrowingCallbackDoesNotStopTheSearch)
{
  Problem p = problem(kRoutes, kRoutesProblem);
  std::atomic<int> calls{0};
  std::atomic<int> errors{0};
  SearchOptions o;
  o.log.callback = [&](LogLevel level, LogChannel, std::string_view) {
      if (level == LogLevel::Error) {
        ++errors;
      }
    };
  Search s = Planner().start(p, o, [&](const PlanUpdate &) {
        ++calls;
        throw std::runtime_error("boom");
      });
  SolveResult r = s.wait();
  ASSERT_EQ(r.status, Status::Solved);
  EXPECT_EQ(r.plan->cost, 6.0);
  EXPECT_EQ(static_cast<size_t>(calls.load()), r.stats.plans);
  EXPECT_EQ(errors.load(), calls.load());
}

TEST(PlannerApiTest, SearchWithoutStartIsInert)
{
  Search s;
  EXPECT_FALSE(s.running());
  EXPECT_FALSE(s.best());
  EXPECT_FALSE(s.wait_first(1ms));
  EXPECT_FALSE(s.next_plan(1ms));
  s.stop();
  SolveResult r = s.wait();
  EXPECT_EQ(r.status, Status::InternalError);
  EXPECT_TRUE(has_code(r.diagnostics, "search.empty"));
}

TEST(PlannerApiTest, UnsolvableInTheBackground)
{
  Problem p = problem(kHoles, holes_problem(4, 3));
  Search s = Planner().start(p);
  EXPECT_FALSE(s.wait_first(10s));  // returns as soon as the search ends
  SolveResult r = s.wait();
  EXPECT_EQ(r.status, Status::Unsolvable);
  EXPECT_FALSE(s.best());
}

TEST(PlannerApiTest, EverySearchPresetSolves)
{
  Problem blocks = problem(kBlocks, kBlocksProblem);
  Problem routes = problem(kRoutes, kRoutesProblem);
  auto names = search_presets();
  EXPECT_GE(names.size(), 5u);
  names.push_back("auto");
  for (const auto & name : names) {
    for (const Problem * p : {&blocks, &routes}) {
      SolveOptions o = quick();
      o.search = name;
      SolveResult r = Planner().solve(*p, o);
      ASSERT_EQ(r.status, Status::Solved) << name;
      EXPECT_TRUE(validate(*p, *r.plan).valid()) << name;
      // Anytime still ends with a proof of optimality
      EXPECT_EQ(r.stop_reason, StopReason::ProvenOptimal) << name;
    }
  }
}

// The search "auto" picked, from the log
std::string auto_choice(const Problem & p)
{
  std::string chosen;
  SolveOptions o = quick();
  o.log.callback = [&](LogLevel, LogChannel, std::string_view text) {
      if (text.find("(auto)") != std::string_view::npos) {
        chosen = std::string(text);
      }
    };
  EXPECT_EQ(Planner().solve(p, o).status, Status::Solved);
  return chosen;
}

// Every "(auto)" choice of the log, in order
std::vector<std::string> auto_choices(const Problem & p, unsigned threads = 1)
{
  std::vector<std::string> chosen;
  SolveOptions o = quick();
  o.threads = threads;
  o.anytime = false;  // the first plan is what the choice is about
  o.log.callback = [&](LogLevel, LogChannel, std::string_view text) {
      if (text.find("(auto)") != std::string_view::npos) {
        chosen.emplace_back(text);
      }
    };
  SolveResult r = Planner().solve(p, o);
  EXPECT_EQ(r.status, Status::Solved);
  if (r.plan) {
    EXPECT_TRUE(validate(p, *r.plan).valid());
  }
  return chosen;
}

// Robots looking at every place of a grid: instant or durative actions
std::pair<std::string, std::string> patrol_grid(int side, bool durative)
{
  std::string domain =
    "(define (domain patrol) (:requirements :strips :typing :durative-actions)"
    " (:types robot place)"
    " (:predicates (at ?r - robot ?p - place) (link ?a ?b - place) (seen ?p - place))";
  if (durative) {
    domain +=
      " (:durative-action move :parameters (?r - robot ?a ?b - place) :duration (= ?duration 5)"
      "  :condition (and (at start (at ?r ?a)) (over all (link ?a ?b)))"
      "  :effect (and (at start (not (at ?r ?a))) (at end (at ?r ?b))))"
      " (:durative-action look :parameters (?r - robot ?p - place) :duration (= ?duration 2)"
      "  :condition (over all (at ?r ?p)) :effect (at end (seen ?p))))";
  } else {
    domain +=
      " (:action move :parameters (?r - robot ?a ?b - place)"
      "  :precondition (and (at ?r ?a) (link ?a ?b)) :effect (and (not (at ?r ?a)) (at ?r ?b)))"
      " (:action look :parameters (?r - robot ?p - place)"
      "  :precondition (at ?r ?p) :effect (seen ?p)))";
  }
  std::string objects, links, goals;
  auto name = [](int x, int y) {return "p" + std::to_string(x) + "_" + std::to_string(y);};
  for (int x = 0; x < side; ++x) {
    for (int y = 0; y < side; ++y) {
      objects += " " + name(x, y);
      goals += " (seen " + name(x, y) + ")";
      for (auto [dx, dy] : {std::pair{1, 0}, std::pair{0, 1}}) {
        if (x + dx < side && y + dy < side) {
          links += " (link " + name(x, y) + " " + name(x + dx, y + dy) + ")";
          links += " (link " + name(x + dx, y + dy) + " " + name(x, y) + ")";
        }
      }
    }
  }
  return {domain, "(define (problem g) (:domain patrol) (:objects r1 r2 - robot" + objects +
    " - place) (:init (at r1 p0_0) (at r2 " + name(side - 1, side - 1) + ")" + links +
    ") (:goal (and" + goals + ")))"};
}

TEST(PlannerApiTest, ManyGoalsAreReachedOneAtATimeFirst)
{
  for (bool durative : {false, true}) {
    auto [d, p] = patrol_grid(5, durative);  // 25 goals
    for (unsigned threads : {1u, 3u}) {
      auto chosen = auto_choices(problem(d, p), threads);
      ASSERT_FALSE(chosen.empty());
      EXPECT_EQ(chosen.front(), "search: incremental goals first (auto)") << durative;
    }
    auto [d4, p4] = patrol_grid(4, durative);  // 16 goals: one search as before
    for (const auto & c : auto_choices(problem(d4, p4))) {
      EXPECT_EQ(c.find("incremental"), std::string::npos) << c;
    }
  }
}

TEST(PlannerApiTest, WithoutAnytimeTheFirstPlanStopsEveryPhase)
{
  // In parallel the other phases went on until the timeout, in order they searched again
  auto [d, p] = patrol_grid(6, true);
  for (unsigned threads : {1u, 3u}) {
    SolveOptions o = quick();
    o.anytime = false;
    o.threads = threads;
    auto t0 = std::chrono::steady_clock::now();
    SolveResult r = Planner().solve(problem(d, p), o);
    ASSERT_EQ(r.status, Status::Solved) << threads;
    EXPECT_EQ(r.stats.plans, 1u) << threads;
    EXPECT_EQ(r.stop_reason, StopReason::NoImprovement) << threads;
    EXPECT_LT(std::chrono::steady_clock::now() - t0, 5s) << threads;
  }
}

TEST(PlannerApiTest, IncrementalGoalsAskedForSolvesAlsoFewGoals)
{
  // As the only search, with fewer goals than auto needs: then improved to a proof
  SolveOptions o = quick();
  o.timeout *= kSlowBuild;  // the proof takes 4 s under ASan, more on the CI runners
  o.search = "goals";
  auto [d, p] = patrol_grid(3, false);
  SolveResult r = Planner().solve(problem(d, p), o);
  ASSERT_EQ(r.status, Status::Solved);
  EXPECT_TRUE(validate(problem(d, p), *r.plan).valid());
  EXPECT_EQ(r.stop_reason, StopReason::ProvenOptimal);
}

// PlanSys2's combat example, reduced: a drone spots a zone only while it runs, the artillery
// bombards it meanwhile, then the infantry moves in and captures it while no enemy is there
const char * kCombat =
  "(define (domain combat) (:requirements :typing :durative-actions :fluents"
  " :negative-preconditions :existential-preconditions)"
  " (:types robot zone - object drone ground - robot infantry artillery tank - ground)"
  " (:predicates (at ?r - robot ?z - zone) (connected ?a ?b - zone) (hostile ?r - robot)"
  "  (spotted ?z - zone) (owned ?z - zone) (destroyed ?r - robot) (available ?r - robot))"
  " (:functions (ammo ?r - artillery) (spent))"
  " (:durative-action spot :parameters (?d - drone ?z - zone) :duration (= ?duration 12)"
  "  :condition (and (at start (available ?d)) (over all (at ?d ?z)))"
  "  :effect (and (at start (not (available ?d))) (at start (spotted ?z))"
  "   (at end (not (spotted ?z))) (at end (available ?d))))"
  " (:durative-action bombard :parameters (?r - artillery ?t - ground ?from ?to - zone)"
  "  :duration (= ?duration 4)"
  "  :condition (and (at start (available ?r)) (at start (hostile ?t)) (at start (at ?t ?to))"
  "   (at start (connected ?from ?to)) (at start (>= (ammo ?r) 1)) (over all (at ?r ?from))"
  "   (over all (spotted ?to)))"
  "  :effect (and (at start (not (available ?r))) (at start (decrease (ammo ?r) 1))"
  "   (at end (not (at ?t ?to))) (at end (destroyed ?t)) (at end (available ?r))"
  "   (at end (increase (spent) 1000))))"
  " (:durative-action move :parameters (?r - infantry ?a ?b - zone) :duration (= ?duration 10)"
  "  :condition (and (at start (available ?r)) (at start (at ?r ?a)) (over all (connected ?a ?b))"
  "   (at start (not (exists (?e - robot) (and (hostile ?e) (at ?e ?b))))))"
  "  :effect (and (at start (not (available ?r))) (at start (not (at ?r ?a)))"
  "   (at end (at ?r ?b)) (at end (available ?r)) (at end (increase (spent) 100))))"
  " (:durative-action capture :parameters (?r - infantry ?z - zone) :duration (= ?duration 5)"
  "  :condition (and (at start (available ?r)) (at start (not (owned ?z))) (over all (at ?r ?z))"
  "   (over all (not (exists (?e - robot) (and (hostile ?e) (at ?e ?z))))))"
  "  :effect (and (at start (not (available ?r))) (at end (owned ?z)) (at end (available ?r))"
  "   (at end (increase (spent) 100)))))";

TEST(PlannerApiTest, StepsOverlapAsFarAsTheResourcesAllow)
{
  // Each job borrows one unit of a capacity of 2 while it runs (umts's pattern): two at once,
  // then the third, makespan 20; ordered by the resource alone they were one after another (30)
  const char * domain =
    "(define (domain r) (:requirements :typing :durative-actions :fluents) (:types job)"
    " (:predicates (todo ?j - job) (done ?j - job)) (:functions (used) (cap))"
    " (:durative-action work :parameters (?j - job) :duration (= ?duration 10)"
    "  :condition (and (at start (todo ?j)) (at start (<= (+ (used) 1) (cap))))"
    "  :effect (and (at start (not (todo ?j))) (at start (increase (used) 1))"
    "   (at end (decrease (used) 1)) (at end (done ?j)))))";
  auto with_cap = [&](int cap) {
      return problem(domain, "(define (problem p) (:domain r) (:objects a b c - job)"
               " (:init (todo a) (todo b) (todo c) (= (used) 0) (= (cap) " +
               std::to_string(cap) + ")) (:goal (and (done a) (done b) (done c))))");
    };
  SolveOptions o = quick();
  o.anytime = false;
  o.threads = 1;
  for (auto [cap, makespan] : {std::pair{1, 30.0}, std::pair{2, 20.0}, std::pair{3, 10.0}}) {
    Problem p = with_cap(cap);
    SolveResult r = Planner().solve(p, o);
    ASSERT_EQ(r.status, Status::Solved) << cap;
    EXPECT_TRUE(validate(p, *r.plan).valid()) << cap;
    EXPECT_NEAR(r.plan->makespan(), makespan, 0.01) << cap;
  }
}

TEST(PlannerApiTest, MetricOfEurosPlusTime)
{
  // spent + 10 * total-time: flying costs 100 euros and 1 s (110), walking 0 euros and 2 s
  // twice (40). With unit costs the search took the one-step flight and stopped
  Problem p = problem(
    "(define (domain m) (:requirements :durative-actions :fluents)"
    " (:predicates (at-a) (at-mid) (at-b)) (:functions (spent))"
    " (:durative-action fly :parameters () :duration (= ?duration 1)"
    "  :condition (at start (at-a)) :effect (and (at start (not (at-a))) (at end (at-b))"
    "  (at end (increase (spent) 100))))"
    " (:durative-action walk1 :parameters () :duration (= ?duration 2)"
    "  :condition (at start (at-a)) :effect (and (at start (not (at-a))) (at end (at-mid))))"
    " (:durative-action walk2 :parameters () :duration (= ?duration 2)"
    "  :condition (at start (at-mid)) :effect (and (at start (not (at-mid))) (at end (at-b)))))",
    "(define (problem p) (:domain m) (:init (at-a) (= (spent) 0)) (:goal (at-b))"
    " (:metric minimize (+ (spent) (* 10 (total-time)))))");
  SolveOptions o = quick();
  o.threads = 1;
  SolveResult r = Planner().solve(p, o);
  ASSERT_EQ(r.status, Status::Solved);
  ASSERT_TRUE(r.plan->cost);
  EXPECT_NEAR(*r.plan->cost, 40.0, 0.1);
  EXPECT_EQ(r.plan->steps.size(), 2u);
  for (const auto & d : r.diagnostics) {
    EXPECT_NE(d.code, "orchestrator.metric") << "the metric guides the search now";
  }
}

// --- Object fluents (PDDL 3.1) --------------------------------------------------------------

TEST(PlannerApiTest, ChangingObjectFluentsAsArgumentsOfAtoms)
{
  // (box-at ?b (loc ?r)), (weight (holding ?r)), (box-at (holding ?r) (loc ?r)): all change
  std::string domain =
    "(define (domain carry) (:requirements :typing :object-fluents :numeric-fluents"
    " :negative-preconditions :action-costs)"
    " (:types robot room box)"
    " (:predicates (connected ?a ?b - room) (box-at ?b - box ?r - room) (free ?r - robot)"
    "  (clean ?r - room))"
    " (:functions (loc ?r - robot) - room (holding ?r - robot) - box (weight ?b - box)"
    "  (total-cost))"
    " (:action move :parameters (?r - robot ?to - room)"
    "  :precondition (connected (loc ?r) ?to)"
    "  :effect (and (assign (loc ?r) ?to) (increase (total-cost) 1)))"
    " (:action pick :parameters (?r - robot ?b - box)"
    "  :precondition (and (free ?r) (box-at ?b (loc ?r)))"
    "  :effect (and (not (free ?r)) (not (box-at ?b (loc ?r))) (assign (holding ?r) ?b)))"
    " (:action drop :parameters (?r - robot)"
    "  :precondition (and (not (free ?r)) (< (weight (holding ?r)) 10))"
    "  :effect (and (free ?r) (box-at (holding ?r) (loc ?r)) (clean (loc ?r))"
    "  (assign (holding ?r) undefined))))";
  std::string problem_text =
    "(define (problem p) (:domain carry)"
    " (:objects r1 - robot a b c - room b1 b2 heavy - box)"
    " (:init (= (loc r1) a) (free r1) (connected a b) (connected b a) (connected b c)"
    "  (connected c b) (box-at b1 a) (box-at b2 c) (box-at heavy b) (= (weight b1) 3)"
    "  (= (weight b2) 4) (= (weight heavy) 12))"
    " (:goal (and (box-at b1 c) (box-at b2 a) (clean (loc r1)) (= (loc r1) b)))"
    " (:metric minimize (total-cost)))";
  Problem p = problem(domain, problem_text);
  for (unsigned threads : {1u, 3u}) {
    SolveOptions o = quick();
    o.threads = threads;
    SolveResult r = Planner().solve(p, o);
    ASSERT_EQ(r.status, Status::Solved) << threads;
    EXPECT_TRUE(validate(p, *r.plan).valid());
    ASSERT_TRUE(r.plan->cost);
    EXPECT_NEAR(*r.plan->cost, 5.0, 1e-6);  // a-b-c, c-b-a, then b
    for (const auto & s : r.plan->steps) {
      EXPECT_FALSE(s.name == "pick" && s.args[1] == "heavy") << "too heavy to drop again";
    }
  }
  // The heavy box at a: picked, it can never be dropped (weight 12)
  std::string heavy = problem_text;
  heavy.replace(heavy.find("(box-at b1 c)"), 13, "(box-at heavy a)");
  EXPECT_EQ(Planner().solve(problem(domain, heavy), quick()).status, Status::Unsolvable);
}

// (not (= f o)) holds while f is undefined, as the validator says: = with an undefined side
// is false, and its negation true (it was compiled to != that was false too: "unsolvable")
TEST(PlannerApiTest, NegatedEqualityHoldsWhileAnObjectFluentIsUndefined)
{
  auto delivery = [](const std::string & goal) {
      return problem(
        "(define (domain delivery) (:requirements :typing :object-fluents"
        " :negative-preconditions)"
        " (:types robot location package)"
        " (:predicates (connected ?a ?b - location))"
        " (:functions (loc ?r - robot) - location (pos ?p - package) - location"
        "  (carrying ?r - robot) - package (owner ?p - package) - robot (twin ?r - robot) - robot)"
        " (:action move :parameters (?r - robot ?from ?to - location)"
        "  :precondition (and (= (loc ?r) ?from) (connected ?from ?to))"
        "  :effect (assign (loc ?r) ?to))"
        " (:action pick :parameters (?r - robot ?p - package ?l - location)"
        "  :precondition (and (= (loc ?r) ?l) (= (pos ?p) ?l) (not (= (carrying ?r) ?p))"
        "   (not (= (owner ?p) ?r)) (not (= (twin ?r) (owner ?p))))"
        "  :effect (and (assign (carrying ?r) ?p) (assign (pos ?p) undefined)))"
        " (:action drop :parameters (?r - robot ?p - package ?l - location)"
        "  :precondition (and (= (loc ?r) ?l) (= (carrying ?r) ?p))"
        "  :effect (and (assign (carrying ?r) undefined) (assign (pos ?p) ?l)))"
        " (:action claim :parameters (?r - robot ?p - package)"
        "  :precondition (= (owner ?p) ?r) :effect (assign (pos ?p) undefined)))",
        "(define (problem deliver) (:domain delivery)"
        " (:objects r1 - robot hall kitchen office - location box - package)"
        " (:init (= (loc r1) hall) (= (pos box) kitchen) (connected hall kitchen)"
        "  (connected kitchen hall) (connected hall office) (connected office hall))"
        " (:goal " + goal + "))");
    };
  // carrying (changing), owner (static) and twin (static, both sides) start undefined
  Problem p = delivery("(= (pos box) office)");
  for (unsigned threads : {1u, 2u}) {
    SolveOptions o = quick();
    o.threads = threads;
    SolveResult r = Planner().solve(p, o);
    ASSERT_EQ(r.status, Status::Solved) << threads;
    EXPECT_EQ(r.plan->steps.size(), 5u);
    EXPECT_TRUE(validate(p, *r.plan).valid());
  }
  // = with an undefined side is false: owner is never assigned, so no plan reaches the goal
  Problem never = delivery("(and (= (pos box) kitchen) (= (owner box) r1))");
  EXPECT_EQ(Planner().solve(never, quick()).status, Status::Unsolvable);
}

TEST(PlannerApiTest, NestedChangingObjectFluentsInTemporalPlans)
{
  // (loc (bearer ?k)): who bears the key and where that one is both change
  Problem p = problem(
    "(define (domain nest) (:requirements :typing :object-fluents :durative-actions :equality)"
    " (:types robot room key)"
    " (:predicates (connected ?a ?b - room) (lit ?r - room))"
    " (:functions (loc ?r - robot) - room (bearer ?k - key) - robot (target ?k - key) - room)"
    " (:durative-action go :parameters (?r - robot ?to - room) :duration (= ?duration 2)"
    "  :condition (at start (connected (loc ?r) ?to))"
    "  :effect (at end (assign (loc ?r) ?to)))"
    " (:durative-action hand :parameters (?k - key ?to - robot) :duration (= ?duration 1)"
    "  :condition (at start (= (loc (bearer ?k)) (loc ?to)))"
    "  :effect (at end (assign (bearer ?k) ?to)))"
    " (:durative-action light :parameters (?k - key) :duration (= ?duration 1)"
    "  :condition (at start (= (loc (bearer ?k)) (target ?k)))"
    "  :effect (at end (lit (loc (bearer ?k))))))",
    "(define (problem p) (:domain nest)"
    " (:objects r1 r2 - robot a b c - room k - key)"
    " (:init (= (loc r1) a) (= (loc r2) a) (= (bearer k) r1) (= (target k) c)"
    "  (connected a b) (connected b a) (connected b c) (connected c b))"
    " (:goal (and (lit c) (lit (loc r2)) (= (bearer k) r2))))");
  SolveResult r = Planner().solve(p, quick());
  ASSERT_EQ(r.status, Status::Solved);
  EXPECT_TRUE(validate(p, *r.plan).valid());
  EXPECT_NEAR(r.plan->makespan(), 5.0, 0.01);  // hand while going, then light at c
}

TEST(PlannerApiTest, DurationsThatReadChangingObjectFluents)
{
  // (dist (loc ?r) ?to): the place at start decides the duration; two short legs beat one long
  Problem p = problem(
    "(define (domain dur) (:requirements :typing :object-fluents :durative-actions"
    " :numeric-fluents)"
    " (:types robot room) (:predicates (connected ?a ?b - room))"
    " (:functions (loc ?r - robot) - room (dist ?a ?b - room))"
    " (:durative-action go :parameters (?r - robot ?to - room)"
    "  :duration (= ?duration (dist (loc ?r) ?to))"
    "  :condition (at start (connected (loc ?r) ?to))"
    "  :effect (at end (assign (loc ?r) ?to))))",
    "(define (problem p) (:domain dur) (:objects r1 - robot a b c - room)"
    " (:init (= (loc r1) a) (connected a b) (connected b c) (connected a c)"
    "  (= (dist a b) 1) (= (dist b c) 1) (= (dist a c) 5))"
    " (:goal (= (loc r1) c)))");
  SolveResult r = Planner().solve(p, quick());
  ASSERT_EQ(r.status, Status::Solved);
  EXPECT_TRUE(validate(p, *r.plan).valid());
  EXPECT_NEAR(r.plan->makespan(), 2.0, 0.01);
}

// --- Preferences (PDDL3) ---------------------------------------------------------------------

const char * kPrefDomain =
  "(define (domain pr) (:requirements :typing :preferences :fluents :action-costs)"
  " (:types item) (:predicates (got ?i - item) (fuel) (done))"
  " (:functions (total-cost))"
  " (:action get :parameters (?i - item) :precondition (and)"
  "  :effect (and (got ?i) (increase (total-cost) 2)))"
  " (:action refuel :parameters () :effect (and (fuel) (increase (total-cost) 1)))"
  " (:action finish :parameters () :precondition (preference full (fuel))"
  "  :effect (done)))";

// Steps of the domain only, and none of the compiled ones
void expect_domain_steps(const Plan & plan)
{
  for (const auto & s : plan.steps) {
    EXPECT_FALSE(s.name.starts_with("kairos-")) << s.action();
  }
}

TEST(PlannerApiTest, GoalPreferencesWorthTheirCost)
{
  // Getting an item costs 2: a is worth it (5), b is not (1). Best: get a, forgo b, cost 3
  Problem p = problem(kPrefDomain,
      "(define (problem p) (:domain pr) (:objects a b - item) (:init (fuel))"
      " (:goal (and (done) (preference pa (got a)) (preference pb (got b))))"
      " (:metric minimize (+ (total-cost) (* 5 (is-violated pa)) (is-violated pb))))");
  for (unsigned threads : {1u, 3u}) {
    SolveOptions o = quick();
    o.threads = threads;
    SolveResult r = Planner().solve(p, o);
    ASSERT_EQ(r.status, Status::Solved) << threads;
    ASSERT_TRUE(r.plan->cost);
    EXPECT_NEAR(*r.plan->cost, 3.0, 1e-6) << threads;
    EXPECT_EQ(r.plan->steps.size(), 2u);  // get a, finish
    expect_domain_steps(*r.plan);
    EXPECT_TRUE(validate(p, *r.plan).valid());
  }
}

TEST(PlannerApiTest, ForallPreferencesCountEachInstance)
{
  // Three items, each violation weighs 3 and getting one costs 2: all of them, cost 6.
  // Weighing 1: none of them, cost 3 (three violations)
  for (auto [weight, cost, steps] : {std::tuple{3, 6.0, 4u}, std::tuple{1, 3.0, 1u}}) {
    Problem p = problem(kPrefDomain,
        "(define (problem p) (:domain pr) (:objects a b c - item) (:init (fuel))"
        " (:goal (and (done) (forall (?i - item) (preference all (got ?i)))))"
        " (:metric minimize (+ (total-cost) (* " + std::to_string(weight) +
        " (is-violated all)))))");
    SolveResult r = Planner().solve(p, quick());
    ASSERT_EQ(r.status, Status::Solved) << weight;
    ASSERT_TRUE(r.plan->cost);
    EXPECT_NEAR(*r.plan->cost, cost, 1e-6) << weight;
    EXPECT_EQ(r.plan->steps.size(), steps) << weight;
    expect_domain_steps(*r.plan);
  }
}

TEST(PlannerApiTest, PreconditionPreferencesCountEachUse)
{
  // finish without fuel violates "full" once: worth refueling (1) only when it weighs more
  for (auto [weight, cost] : {std::pair{4, 1.0}, std::pair{0, 0.0}}) {
    Problem p = problem(kPrefDomain,
        "(define (problem p) (:domain pr) (:objects a - item)"
        " (:goal (done)) (:metric minimize (+ (total-cost) (* " + std::to_string(weight) +
        " (is-violated full)))))");
    SolveResult r = Planner().solve(p, quick());
    ASSERT_EQ(r.status, Status::Solved) << weight;
    ASSERT_TRUE(r.plan->cost);
    EXPECT_NEAR(*r.plan->cost, cost, 1e-6) << weight;
    expect_domain_steps(*r.plan);
    EXPECT_TRUE(validate(p, *r.plan).valid());
  }
}

TEST(PlannerApiTest, PreferencesThatCannotHoldAndHardGoalsThatCannot)
{
  // A preference nothing achieves: forgone. A hard goal nothing achieves: still unsolvable
  Problem soft = problem(kPrefDomain,
      "(define (problem p) (:domain pr) (:objects a - item) (:init (fuel))"
      " (:goal (and (done) (preference never (and (got a) (not (got a))))))"
      " (:metric minimize (* 7 (is-violated never))))");
  SolveResult r = Planner().solve(soft, quick());
  ASSERT_EQ(r.status, Status::Solved);
  EXPECT_NEAR(*r.plan->cost, 7.0, 1e-6);
  EXPECT_EQ(r.plan->steps.size(), 1u);
  Problem hard = problem(
      "(define (domain h) (:requirements :preferences) (:predicates (x) (y))"
      " (:action ax :parameters () :effect (x)))",
      "(define (problem p) (:domain h) (:goal (and (y) (preference px (x))))"
      " (:metric minimize (is-violated px)))");
  EXPECT_EQ(Planner().solve(hard, quick()).status, Status::Unsolvable);
  // Without a metric the preferences weigh nothing: the shortest plan
  Problem none = problem(kPrefDomain,
      "(define (problem p) (:domain pr) (:objects a - item) (:init (fuel))"
      " (:goal (and (done) (preference pa (got a)))))");
  r = Planner().solve(none, quick());
  ASSERT_EQ(r.status, Status::Solved);
  EXPECT_EQ(r.plan->steps.size(), 1u);
}

TEST(PlannerApiTest, PreferencesToMaximize)
{
  // Pipesworld style: maximize what the satisfied preferences are worth
  Problem p = problem(kPrefDomain,
      "(define (problem p) (:domain pr) (:objects a b - item) (:init (fuel))"
      " (:goal (and (done) (preference pa (got a)) (preference pb (got b))))"
      " (:metric maximize (+ (* 9 (- 1 (is-violated pa))) (* 4 (- 1 (is-violated pb))))))");
  SolveResult r = Planner().solve(p, quick());
  ASSERT_EQ(r.status, Status::Solved);
  ASSERT_TRUE(r.plan->cost);
  EXPECT_NEAR(*r.plan->cost, 13.0, 1e-6);
  expect_domain_steps(*r.plan);
}

TEST(PlannerApiTest, TemporalPreferences)
{
  // Pathways style: a goal preference weighed against the makespan. Making b takes 5 s (a,
  // 2 s, at the same time): worth it when its violation weighs 10, not when it weighs 1
  for (auto [weight, cost] : {std::pair{10, 5.0}, std::pair{1, 3.0}}) {
    Problem p = problem(
        "(define (domain t) (:requirements :durative-actions :preferences)"
        " (:predicates (a) (b))"
        " (:durative-action make-a :parameters () :duration (= ?duration 2)"
        "  :condition (and) :effect (at end (a)))"
        " (:durative-action make-b :parameters () :duration (= ?duration 5)"
        "  :condition (and) :effect (at end (b))))",
        "(define (problem p) (:domain t) (:goal (and (a) (preference pb (b))))"
        " (:metric minimize (+ (total-time) (* " + std::to_string(weight) +
        " (is-violated pb)))))");
    SolveResult r = Planner().solve(p, quick());
    ASSERT_EQ(r.status, Status::Solved) << weight;
    ASSERT_TRUE(r.plan->cost);
    EXPECT_NEAR(*r.plan->cost, cost, 0.05) << weight;
    expect_domain_steps(*r.plan);
    EXPECT_TRUE(validate(p, *r.plan).valid());
  }
}

// --- Trajectory constraints (PDDL3) ---------------------------------------------------------

// A line a - b - c - d; c is dangerous to cross, d has a scanner; the package goes from a to b
const char * kRoute =
  "(define (domain route) (:requirements :strips :typing :negative-preconditions :constraints"
  " :preferences :action-costs)"
  " (:types room) (:predicates (at ?x - room) (link ?a ?b - room) (holding) (pkg-at ?x - room)"
  "  (inspected) (delivered) (hurt) (danger ?x - room) (scanner ?x - room) (lit))"
  " (:functions (total-cost))"
  " (:action move :parameters (?a ?b - room) :precondition (and (at ?a) (link ?a ?b))"
  "  :effect (and (not (at ?a)) (at ?b) (increase (total-cost) 1)))"
  " (:action cross :parameters (?a ?b - room) :precondition (and (at ?a) (link ?a ?b) (danger ?b))"
  "  :effect (and (not (at ?a)) (at ?b) (hurt)))"
  " (:action pick :parameters (?x - room) :precondition (and (at ?x) (pkg-at ?x))"
  "  :effect (and (holding) (not (pkg-at ?x))))"
  " (:action inspect :parameters (?x - room) :precondition (and (at ?x) (holding) (scanner ?x))"
  "  :effect (inspected))"
  " (:action drop :parameters (?x - room) :precondition (and (at ?x) (holding))"
  "  :effect (and (not (holding)) (pkg-at ?x) (delivered)))"
  " (:action switch :parameters () :effect (and (lit) (increase (total-cost) 1)))"
  " (:action unswitch :parameters () :effect (and (not (lit)) (increase (total-cost) 1))))";

std::string route_problem(const std::string & goal, const std::string & rest)
{
  return "(define (problem p) (:domain route) (:objects a b c d - room)"
         " (:init (at a) (pkg-at a) (link a b) (link b a) (link b c) (link c b) (link c d)"
         "  (link d c) (danger c) (scanner d))"
         " (:goal " + goal + ") " + rest + ")";
}

// Solves and returns the plan as "(a) (b) ..."
std::string route_plan(const std::string & goal, const std::string & rest, double * cost = nullptr)
{
  Problem p = problem(kRoute, route_problem(goal, rest));
  SolveResult r = Planner().solve(p, quick());
  if (r.status != Status::Solved) {
    return "status " + std::to_string(static_cast<int>(r.status));
  }
  EXPECT_TRUE(validate(p, *r.plan).valid());
  expect_domain_steps(*r.plan);
  if (cost) {
    *cost = r.plan->cost ? *r.plan->cost : -1.0;
  }
  std::string out;
  for (const auto & s : r.plan->steps) {
    out += (out.empty() ? "" : " ") + s.action();
  }
  return out;
}

TEST(PlannerApiTest, AlwaysAndSometimeBefore)
{
  // The user's example: never hurt, and inspected before delivered (b, via d and back)
  std::string plan = route_plan("(delivered)",
      "(:constraints (and (always (not (hurt))) (sometime-before (delivered) (inspected))))");
  EXPECT_NE(plan.find("(inspect d)"), std::string::npos) << plan;
  EXPECT_LT(plan.find("(inspect d)"), plan.find("(drop"));
  EXPECT_EQ(plan.find("cross"), std::string::npos) << plan;
  // Always false from the start: nothing to do about it
  EXPECT_EQ(route_plan("(delivered)", "(:constraints (always (inspected)))"),
    "status " + std::to_string(static_cast<int>(Status::Unsolvable)));
}

TEST(PlannerApiTest, SometimeAndAtEnd)
{
  // Lit at some point, but not at the end: switched on and off again
  double cost = 0;
  std::string plan = route_plan("(pkg-at a)",
      "(:constraints (and (sometime (lit)) (at end (not (lit)))))"
      " (:metric minimize (total-cost))", &cost);
  EXPECT_EQ(plan, "(switch) (unswitch)");
  EXPECT_NEAR(cost, 2.0, 1e-6);
  // Already true at the start counts
  plan = route_plan("(pkg-at a)", "(:constraints (sometime (at a)))");
  EXPECT_EQ(plan, "");
}

TEST(PlannerApiTest, AtMostOnceAndSometimeAfter)
{
  // At b only once: from a to b, back to a, and b again is not allowed; then the package
  // must be dropped at a after going to b
  std::string plan = route_plan("(and (at a) (delivered))",
      "(:constraints (and (at-most-once (at c)) (sometime-after (at b) (holding))))");
  EXPECT_EQ(plan.find("(move b c)"), std::string::npos) << plan;  // not needed
  EXPECT_NE(plan.find("(drop a)"), std::string::npos) << plan;
  // To visit d and come back c is crossed twice: once only is impossible
  EXPECT_EQ(route_plan("(and (at a) (inspected))", "(:constraints (at-most-once (at c)))"),
    "status " + std::to_string(static_cast<int>(Status::Unsolvable)));
  // After b, holding then or later: ending at b without the package never satisfies it
  EXPECT_EQ(route_plan("(and (at b) (not (holding)))",
    "(:constraints (sometime-after (at b) (holding)))"),
    "status " + std::to_string(static_cast<int>(Status::Unsolvable)));
}

TEST(PlannerApiTest, PreferencesOverConstraints)
{
  // Crossing c costs nothing but hurts: worth it only when hurting weighs nothing
  for (auto [weight, expected] : {std::pair{0, 2.0}, std::pair{50, 3.0}}) {
    double cost = 0;
    std::string plan = route_plan("(at d)",
        "(:constraints (preference safe (always (not (hurt)))))"
        " (:metric minimize (+ (total-cost) (* " + std::to_string(weight) +
        " (is-violated safe))))", &cost);
    EXPECT_NEAR(cost, expected, 1e-6) << weight << ": " << plan;
    EXPECT_EQ(plan.find("cross") == std::string::npos, weight == 50) << plan;
  }
  // Inspecting first (d and back, crossing: 3) against delivering at once (1 + the weight)
  for (auto [weight, cost] : {std::pair{10, 3.0}, std::pair{1, 2.0}}) {
    double c = 0;
    std::string plan = route_plan("(and (delivered) (at b))",
        "(:constraints (preference first (sometime-before (delivered) (inspected))))"
        " (:metric minimize (+ (total-cost) (* " + std::to_string(weight) +
        " (is-violated first))))", &c);
    EXPECT_NEAR(c, cost, 1e-6) << weight << ": " << plan;
  }
  // Forall over rooms: each room not visited counts once. All of them: a b, cross to c, d,
  // cross back to c, b, a
  double c = 0;
  route_plan("(at a)",
      "(:constraints (forall (?x - room) (preference visit (sometime (at ?x)))))"
      " (:metric minimize (+ (total-cost) (* 10 (is-violated visit))))", &c);
  EXPECT_NEAR(c, 4.0, 1e-6);
  // Weighing less than a move: none visited but a (3 violations)
  route_plan("(at a)",
      "(:constraints (forall (?x - room) (preference visit (sometime (at ?x)))))"
      " (:metric minimize (+ (* 4 (total-cost)) (is-violated visit)))", &c);
  EXPECT_NEAR(c, 3.0, 1e-6);
}

TEST(PlannerApiTest, ConstraintsOnDurativeActions)
{
  // The quick way hurts: with "always not hurt" the slow one, also as a preference that
  // weighs more than the 4 s it saves
  std::string domain =
    "(define (domain t) (:requirements :durative-actions :constraints :preferences"
    " :negative-preconditions)"
    " (:predicates (there) (hurt))"
    " (:durative-action quick :parameters () :duration (= ?duration 1)"
    "  :condition (and) :effect (and (at end (there)) (at end (hurt))))"
    " (:durative-action slow :parameters () :duration (= ?duration 5)"
    "  :condition (and) :effect (at end (there))))";
  for (std::string rest : {std::string("(:constraints (always (not (hurt))))"),
      std::string("(:constraints (preference safe (always (not (hurt)))))"
      " (:metric minimize (+ (total-time) (* 10 (is-violated safe))))")})
  {
    Problem p = problem(domain, "(define (problem p) (:domain t) (:goal (there)) " + rest + ")");
    SolveResult r = Planner().solve(p, quick());
    ASSERT_EQ(r.status, Status::Solved) << rest;
    ASSERT_EQ(r.plan->steps.size(), 1u) << rest;
    EXPECT_EQ(r.plan->steps[0].name, "slow") << rest;
    EXPECT_TRUE(validate(p, *r.plan).valid());
  }
}

TEST(PlannerApiTest, TimedConstraintsInSequentialPlans)
{
  // State k is at time k (the validator's convention)
  std::string unsolvable = "status " + std::to_string(static_cast<int>(Status::Unsolvable));
  // At c by time 2: through b, and back to a
  std::string plan = route_plan("(at a)", "(:constraints (within 2 (at c)))");
  EXPECT_NE(plan.find("c)"), std::string::npos) << plan;
  EXPECT_EQ(route_plan("(at a)", "(:constraints (within 1 (at c)))"), unsolvable);
  // At d in every state after time 3: d is three steps away, so the plan ends there
  plan = route_plan("(at d)", "(:constraints (hold-after 3 (at d)))");
  EXPECT_FALSE(plan.empty());
  EXPECT_EQ(route_plan("(at d)", "(:constraints (hold-after 1 (at d)))"), unsolvable);
  // Holding at times 1 and 2: picked first, dropped not before the third step
  plan = route_plan("(delivered)", "(:constraints (hold-during 1 3 (holding)))");
  EXPECT_EQ(plan.rfind("(pick a)", 0), 0u) << plan;
  EXPECT_EQ(std::count(plan.begin(), plan.end(), '('), 3) << plan;
  EXPECT_EQ(route_plan("(delivered)", "(:constraints (hold-during 0 1 (holding)))"), unsolvable);
}

TEST(PlannerApiTest, AlwaysWithinInSequentialPlans)
{
  // Delivered within t steps of holding: the drop at most t steps after the pick
  for (int delay : {1, 2, 3}) {
    Problem p = problem(kRoute, route_problem("(and (delivered) (at d))",
        "(:constraints (always-within " + std::to_string(delay) + " (holding) (delivered)))"));
    SolveResult r = Planner().solve(p, quick());
    ASSERT_EQ(r.status, Status::Solved) << delay;
    EXPECT_TRUE(validate(p, *r.plan).valid());
    expect_domain_steps(*r.plan);
    size_t pick = 0, drop = 0;
    for (size_t i = 0; i < r.plan->steps.size(); ++i) {
      pick = r.plan->steps[i].name == "pick" ? i : pick;
      drop = r.plan->steps[i].name == "drop" ? i : drop;
    }
    EXPECT_LE(drop - pick, static_cast<size_t>(delay)) << delay;
  }
  // Inspected (at d, holding) holds the package 4 steps: delivered first (it stays true),
  // then picked again
  std::string plan = route_plan("(and (inspected) (delivered))",
      "(:constraints (always-within 2 (holding) (delivered)))");
  EXPECT_LT(plan.find("(drop"), plan.find("(inspect")) << plan;
  // As preferences, both met the same way
  double cost = -1;
  route_plan("(delivered)",
      "(:constraints (and (preference fast (always-within 2 (holding) (delivered)))"
      " (preference check (sometime (inspected)))))"
      " (:metric minimize (+ (* 3 (is-violated fast)) (* 5 (is-violated check))))", &cost);
  EXPECT_NEAR(cost, 0.0, 1e-6);
  // Inspected within a step of holding is impossible (d is far): holding at all violates it,
  // weighed against not holding
  route_plan("(at a)",
      "(:constraints (and (preference fast (always-within 1 (holding) (inspected)))"
      " (preference carry (sometime (holding)))))"
      " (:metric minimize (+ (* 3 (is-violated fast)) (* 5 (is-violated carry))))", &cost);
  EXPECT_NEAR(cost, 3.0, 1e-6);
}

TEST(PlannerApiTest, WithinInTemporalPlans)
{
  // The cheap way takes 6 s: by time 4 only the expensive one gets there
  std::string domain =
    "(define (domain t) (:requirements :durative-actions :constraints :fluents)"
    " (:predicates (there)) (:functions (spent))"
    " (:durative-action fast :parameters () :duration (= ?duration 2)"
    "  :condition (and) :effect (and (at end (there)) (at end (increase (spent) 10))))"
    " (:durative-action slow :parameters () :duration (= ?duration 6)"
    "  :condition (and) :effect (at end (there))))";
  for (auto [rest, name] : {std::pair{"(:constraints (within 4 (there)))", "fast"},
      std::pair{"", "slow"}})
  {
    Problem p = problem(domain, std::string("(define (problem p) (:domain t) (:init (= (spent) 0))"
        " (:goal (there)) ") + rest + " (:metric minimize (spent)))");
    SolveResult r = Planner().solve(p, quick());
    ASSERT_EQ(r.status, Status::Solved) << rest;
    ASSERT_EQ(r.plan->steps.size(), 1u) << rest;
    EXPECT_EQ(r.plan->steps[0].name, name) << rest;
    EXPECT_TRUE(validate(p, *r.plan).valid());
  }
}

TEST(PlannerApiTest, TimedLiteralsAtTheSameInstant)
{
  // Two literals at 5: their counter made them epsilon apart, an inconsistent STN, and M3
  // answered unsolvable
  std::string domain =
    "(define (domain tl) (:requirements :durative-actions :timed-initial-literals)"
    " (:predicates (a) (b) (x) (y))"
    " (:durative-action ga :parameters () :duration (= ?duration 1)"
    "  :condition (at start (a)) :effect (at end (x)))"
    " (:durative-action gb :parameters () :duration (= ?duration 1)"
    "  :condition (at start (b)) :effect (at end (y))))";
  for (int second : {5, 6}) {
    Problem p = problem(domain, "(define (problem p) (:domain tl) (:init (at 5 (a)) (at " +
        std::to_string(second) + " (b))) (:goal (and (x) (y))))");
    for (unsigned threads : {1u, 3u}) {
      SolveOptions o = quick();
      o.threads = threads;
      SolveResult r = Planner().solve(p, o);
      ASSERT_EQ(r.status, Status::Solved) << second << " " << threads;
      EXPECT_TRUE(validate(p, *r.plan).valid());
      EXPECT_NEAR(r.plan->makespan(), second + 1.0, 0.01) << second;
    }
  }
}

TEST(PlannerApiTest, PreferencesAnywhereInPreconditionsCountTheirViolations)
{
  // Under a forall: one violation per dirty cell when go happens (2); five at the top of a
  // precondition (more than the copies compiled): those that do not hold (p2 and p4)
  Problem p = problem(
    "(define (domain c) (:requirements :preferences :typing :quantified-preconditions)"
    " (:types cell) (:predicates (dirty ?c - cell) (done) (q1) (q2) (q3) (q4) (q5) (ready))"
    " (:action go :parameters ()"
    "  :precondition (forall (?c - cell) (preference clean (not (dirty ?c))))"
    "  :effect (ready))"
    " (:action finish :parameters ()"
    "  :precondition (and (ready) (preference p1 (q1)) (preference p2 (q2))"
    "   (preference p3 (q3)) (preference p4 (q4)) (preference p5 (q5)))"
    "  :effect (done)))",
    "(define (problem x) (:domain c) (:objects a b k - cell)"
    " (:init (dirty a) (dirty b) (q1) (q3) (q5)) (:goal (done))"
    " (:metric minimize (+ (* 10 (is-violated clean)) (is-violated p1) (* 2 (is-violated p2))"
    "  (is-violated p3) (* 3 (is-violated p4)) (is-violated p5))))");
  SolveResult r = Planner().solve(p, quick());
  ASSERT_EQ(r.status, Status::Solved);
  ASSERT_TRUE(r.plan->cost);
  EXPECT_NEAR(*r.plan->cost, 10 * 2 + 2 + 3, 1e-9);
  auto v = validate(p, *r.plan);
  ASSERT_TRUE(v.valid());
  ASSERT_TRUE(v.metric);
  EXPECT_NEAR(*v.metric, *r.plan->cost, 1e-9);
}

TEST(PlannerApiTest, AlwaysWithinInTemporalPlans)
{
  // Holding from the end of pick (1 s); delivered at the end of deliver: within 3 s if it takes
  // 2, not if it takes 5
  auto domain = [](int deliver) {
      return "(define (domain t) (:requirements :durative-actions :constraints)"
             " (:predicates (holding) (delivered))"
             " (:durative-action pick :parameters () :duration (= ?duration 1)"
             "  :condition (and) :effect (at end (holding)))"
             " (:durative-action deliver :parameters () :duration (= ?duration " +
             std::to_string(deliver) + ")"
             "  :condition (at start (holding))"
             "  :effect (and (at end (delivered)) (at end (not (holding))))))";
    };
  const char * hard = "(define (problem p) (:domain t) (:goal (delivered))"
    " (:constraints (always-within 3 (holding) (delivered))))";
  Problem fast = problem(domain(2), hard);
  SolveResult r = Planner().solve(fast, quick());
  ASSERT_EQ(r.status, Status::Solved);
  EXPECT_TRUE(validate(fast, *r.plan).valid());
  // Deliver after pick, early enough: holding at most 3 s
  EXPECT_GE(r.plan->makespan(), 3.0 - 1e-6);
  EXPECT_LE(r.plan->makespan(), 4.0 + 1e-6);
  Problem slow = problem(domain(5), hard);
  EXPECT_NE(Planner().solve(slow, quick()).status, Status::Solved);
  // As a preference: planned, and paid for
  Problem soft = problem(domain(5),
      "(define (problem p) (:domain t) (:goal (delivered))"
      " (:constraints (preference fast (always-within 3 (holding) (delivered))))"
      " (:metric minimize (* 10 (is-violated fast))))");
  SolveResult s = Planner().solve(soft, quick());
  ASSERT_EQ(s.status, Status::Solved);
  ASSERT_TRUE(s.plan->cost);
  EXPECT_NEAR(*s.plan->cost, 10.0, 1e-6);
}

TEST(PlannerApiTest, RequiredConcurrencyThenActionsThatNeedItsResult)
{
  // The end of capture needs "no enemy in the zone", a derived atom being false: the
  // heuristics see it (it was optimistic, and over all conditions were not in the snap task),
  // so the relaxed plan bombards
  // On a 3x3 grid, two infantry units: before, 205 000 expansions and no plan in 10 s
  const char * grid[3][3] = {{"nw", "nm", "ne"}, {"cw", "cm", "ce"}, {"sw", "sm", "se"}};
  std::string zones, links;
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      zones += std::string(" ") + grid[r][c];
      for (auto [dr, dc] : {std::pair{0, 1}, std::pair{1, 0}}) {
        if (r + dr < 3 && c + dc < 3) {
          std::string a = grid[r][c], b = grid[r + dr][c + dc];
          links += " (connected " + a + " " + b + ") (connected " + b + " " + a + ")";
        }
      }
    }
  }
  Problem p = problem(kCombat,
      "(define (problem m6) (:domain combat) (:objects" + zones + " - zone d1 - drone"
      " a1 - artillery i1 i2 - infantry e1 - tank)"
      " (:init (at d1 nm) (at a1 nw) (at i1 nw) (at i2 cw) (at e1 nm) (hostile e1)" + links +
      "  (available d1) (available a1) (available i1) (available i2) (= (ammo a1) 3)"
      "  (= (spent) 0))"
      " (:goal (owned nm)) (:metric minimize (spent)))");
  for (unsigned threads : {1u, 3u}) {
    SolveOptions o = quick();
    o.anytime = false;
    o.threads = threads;
    SolveResult r = Planner().solve(p, o);
    ASSERT_EQ(r.status, Status::Solved) << threads;
    EXPECT_TRUE(validate(p, *r.plan).valid());
    EXPECT_LT(r.stats.expanded, 2000u) << threads;
    EXPECT_EQ(r.plan->steps.size(), 4u);  // spot || bombard, move, capture
  }
}

TEST(PlannerApiTest, FirstPlanSpreadsTheWorkAmongTheRobots)
{
  // PlanSys2's bt_example: three robots assemble cars. The first plan gave every step to one
  // robot (makespan 85 with 3 cars, POPF 45): ties now go to the step that ends first in the
  // schedule, and preferred steps to the robot that is free first
  auto factory = [](int cars) {
      std::string pieces,
    init = "(is_assembly_zone assembly_zone) (is_recharge_zone recharge_zone)";
      for (int i = 1; i <= cars; ++i) {
        for (auto [kind, zone] : {std::pair{"wheel", "wheels_zone"},
            std::pair{"body_car", "body_car_zone"},
            std::pair{"steering_wheel", "steering_wheels_zone"}})
        {
          std::string p = std::string(kind) + "_" + std::to_string(i);
          pieces += " " + p;
          init += " (piece_is_" + std::string(kind) + " " + p + ") (piece_at " + p + " " + zone +
            ") (piece_not_used " + p + ")";
        }
      }
      std::string goal, carlist;
      for (int i = 1; i <= cars; ++i) {
        goal += " (car_assembled car_" + std::to_string(i) + ")";
        carlist += " car_" + std::to_string(i);
      }
      for (const char * r : {"r2d2", "c3po", "bb8"}) {
        init += std::string(" (robot_at ") + r + " wheels_zone) (battery_full " + r +
          ") (robot_available " + r + ")";
      }
      return "(define (problem f) (:domain factory) (:objects r2d2 c3po bb8 - robot"
             " wheels_zone steering_wheels_zone body_car_zone assembly_zone recharge_zone - zone" +
             pieces + " - piece" + carlist + " - car) (:init " + init + ") (:goal (and" + goal +
             ")))";
    };
  std::string domain = kairos::testing::read_file(
    kairos::testing::data_path("plansys2/plansys2_bt_example__bt_example.pddl"));
  for (auto [cars, popf] : {std::pair{2, 30.0}, std::pair{3, 45.0}, std::pair{4, 60.0}}) {
    Problem p = problem(domain, factory(cars));
    SolveOptions o = quick();
    o.anytime = false;
    o.threads = 1;
    SolveResult r = Planner().solve(p, o);
    ASSERT_EQ(r.status, Status::Solved) << cars;
    EXPECT_TRUE(validate(p, *r.plan).valid());
    EXPECT_LE(r.plan->makespan(), popf + 0.1) << cars << " cars";
    std::set<std::string> robots;
    for (const auto & s : r.plan->steps) {
      robots.insert(s.args.at(0));
    }
    EXPECT_EQ(robots.size(), 3u) << cars << " cars";
  }
}

TEST(PlannerApiTest, AutoSearchFollowsTheTask)
{
  // Classical: LAMA, then novelty if no plan; numeric conditions: lazy LAMA, then LAMA with
  // novelty
  EXPECT_EQ(auto_choice(problem(kBlocks, kBlocksProblem)), "search: lama, then novelty (auto)");
  EXPECT_EQ(auto_choice(problem(kRoutes, kRoutesProblem)),  // costs only
    "search: lama, then novelty (auto)");
  // Asked for, no portfolio
  {
    SolveOptions o = quick();
    o.search = "lama";
    SolveResult r = Planner().solve(problem(kBlocks, kBlocksProblem), o);
    ASSERT_EQ(r.status, Status::Solved);
  }
  Problem numeric = problem(
    "(define (domain n) (:requirements :strips :fluents) (:predicates (g))"
    " (:functions (x)) (:action inc :parameters () :precondition () :effect (increase (x) 1))"
    " (:action fin :parameters () :precondition (>= (x) 3) :effect (g)))",
    "(define (problem p) (:domain n) (:init (= (x) 0)) (:goal (g)))");
  EXPECT_EQ(auto_choice(numeric), "search: lama, then lama-novelty (auto)");
}

TEST(PlannerApiTest, UnknownSearchIsInvalidInput)
{
  SolveOptions o = quick();
  o.search = "bogus";
  SolveResult r = Planner().solve(problem(kRobots, kRobotsProblem), o);
  EXPECT_EQ(r.status, Status::InvalidInput);
  EXPECT_TRUE(has_code(r.diagnostics, "orchestrator.unknown_search"));
}

// Grounding five robots' worth of joins over a 60-room corridor is out of reach (60^5
// actions); each real state has a single robot position, so searching it lifted is easy
TEST(PlannerApiTest, TooLargeToGroundIsPlannedWithoutGrounding)
{
  // Rooms 0 to 59 linked in a row, the last link optional
  auto corridor = [](bool last_link) {
      std::string objects;
      std::string init = "(at r0)";
      for (int i = 0; i < 60; ++i) {
        objects += " r" + std::to_string(i);
        if (i > 0 && (i < 59 || last_link)) {
          init += " (link r" + std::to_string(i - 1) + " r" + std::to_string(i) + ")";
        }
      }
      return problem(
        "(define (domain corridor) (:requirements :strips) (:predicates (at ?x) (link ?x ?y))"
        " (:action go :parameters (?a ?b ?c ?d ?e ?n)"
        "  :precondition (and (at ?a) (at ?b) (at ?c) (at ?d) (at ?e) (link ?a ?n))"
        "  :effect (and (not (at ?a)) (at ?n))))",
        "(define (problem p) (:domain corridor) (:objects" + objects + ") (:init " + init +
        ") (:goal (at r59)))");
    };
  Problem p = corridor(true);
  Problem cut = corridor(false);
  // One thread: a short grounding, the lifted engine, then the whole grounding; more: the
  // lifted engine beside the grounding
  for (unsigned threads : {1u, 2u}) {
    SolveOptions o;
    o.timeout = std::chrono::seconds(3 * kSlowBuild);
    o.memory_limit_mb = 512;  // grounding never takes more than this
    o.anytime = false;
    o.threads = threads;
    SolveResult r = Planner().solve(p, o);
    ASSERT_EQ(r.status, Status::Solved) << threads;
    EXPECT_EQ(r.plan->steps.size(), 59u);
    EXPECT_EQ(r.stats.atoms, 0u);  // nothing grounded
    EXPECT_TRUE(validate(p, *r.plan).valid());
    // Without the last link: the lifted engine proves there is no plan
    EXPECT_EQ(Planner().solve(cut, o).status, Status::Unsolvable) << threads;
  }
}

TEST(PlannerApiTest, StatusNames)
{
  EXPECT_EQ(to_string(Status::Solved), "solved");
  EXPECT_EQ(to_string(Status::GroundingTooLarge), "grounding too large");
  EXPECT_EQ(to_string(StopReason::ProvenOptimal), "proven optimal");
  EXPECT_EQ(to_string(StopReason::NoImprovement), "no improvement");
}

}  // namespace
}  // namespace kairos
