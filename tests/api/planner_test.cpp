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
#include <string>
#include <thread>
#include <vector>

#include "kairos/kairos.hpp"

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

// PDDL+ (a process): not plannable until phase F5
const char * kProcess =
  "(define (domain flow) (:requirements :fluents :time) (:predicates (on) (done))"
  " (:functions (level))"
  " (:process fill :parameters () :precondition (on) :effect (increase (level) (* #t 2)))"
  " (:action stop :parameters () :precondition (>= (level) 10) :effect (done)))\n";

const char * kProcessProblem =
  "(define (problem x) (:domain flow) (:init (on) (= (level) 0)) (:goal (done)))\n";

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
  EXPECT_FALSE(t.plannable);
  EXPECT_TRUE(has_code(t.diagnostics, "unsupported."));
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
  Problem p = problem(kProcess, kProcessProblem);
  SolveResult r = Planner().solve(p, quick());
  EXPECT_EQ(r.status, Status::Unsupported);
  EXPECT_TRUE(has_code(r.diagnostics, "unsupported."));
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
  EXPECT_EQ(f.status, Status::Unsupported);
  EXPECT_TRUE(has_code(f.diagnostics, "orchestrator.forced_m2"));
  EXPECT_TRUE(has_code(f.diagnostics, "unsupported.m2_incomplete"));
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
  EXPECT_EQ(n.status, Status::Unsolvable);
  EXPECT_TRUE(has_code(n.diagnostics, "orchestrator.m3_exhausted"));
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

// AddressSanitizer's shadow memory inflates the RSS the budget watches: a larger limit there
#if defined(__SANITIZE_ADDRESS__)
constexpr size_t kSearchMemoryMb = 64;
#elif defined(__has_feature)
#if __has_feature(address_sanitizer)
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

TEST(PlannerApiTest, AutoSearchFollowsTheTask)
{
  // Classical: LAMA; numeric conditions: lazy LAMA, then LAMA with novelty
  EXPECT_EQ(auto_choice(problem(kBlocks, kBlocksProblem)), "search: lama (auto)");
  EXPECT_EQ(auto_choice(problem(kRoutes, kRoutesProblem)), "search: lama (auto)");  // costs only
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

TEST(PlannerApiTest, StatusNames)
{
  EXPECT_EQ(to_string(Status::Solved), "solved");
  EXPECT_EQ(to_string(Status::GroundingTooLarge), "grounding too large");
  EXPECT_EQ(to_string(StopReason::ProvenOptimal), "proven optimal");
  EXPECT_EQ(to_string(StopReason::NoImprovement), "no improvement");
}

}  // namespace
}  // namespace kairos
