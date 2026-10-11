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

#include <atomic>
#include <chrono>
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
  CheckReport t = check_problem(kDurativeProblem, *timed);
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

TEST(PlannerApiTest, UnsupportedFeaturesAreReported)
{
  Problem p = problem(kDurative, kDurativeProblem);
  SolveResult r = Planner().solve(p, quick());
  EXPECT_EQ(r.status, Status::Unsupported);
  EXPECT_TRUE(has_code(r.diagnostics, "unsupported."));

  Problem classic = problem(kRobots, kRobotsProblem);
  SolveOptions m3 = quick();
  m3.engine = Engine::M3;
  SolveResult e = Planner().solve(classic, m3);
  EXPECT_EQ(e.status, Status::Unsupported);
  EXPECT_TRUE(has_code(e.diagnostics, "unsupported.engine"));
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
  for (const char * f : {"domain.pddl", "problem.pddl", "lifted.txt", "ground.txt", "sas.txt",
      "plan-trace.txt"})
  {
    EXPECT_TRUE(fs::exists(dir / f)) << f;
    EXPECT_GT(fs::file_size(dir / f), 0u) << f;
  }
  // The dumped PDDL parses again
  std::ifstream in(dir / "domain.pddl");
  std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
  EXPECT_TRUE(parse_domain(text).ok());

  fs::path only = temp_dir("dump_sas");
  o.dump_dir = only;
  o.dump = {"sas"};
  ASSERT_EQ(Planner().solve(p, o).status, Status::Solved);
  EXPECT_TRUE(fs::exists(only / "sas.txt"));
  EXPECT_FALSE(fs::exists(only / "ground.txt"));
  EXPECT_FALSE(fs::exists(only / "domain.pddl"));
  fs::remove_all(dir);
  fs::remove_all(only);
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

TEST(PlannerApiTest, AutoSearchFollowsTheTask)
{
  // Classical: LAMA; numeric conditions: LAMA with novelty
  EXPECT_EQ(auto_choice(problem(kBlocks, kBlocksProblem)), "search: lama (auto)");
  EXPECT_EQ(auto_choice(problem(kRoutes, kRoutesProblem)), "search: lama (auto)");  // costs only
  Problem numeric = problem(
    "(define (domain n) (:requirements :strips :fluents) (:predicates (g))"
    " (:functions (x)) (:action inc :parameters () :precondition () :effect (increase (x) 1))"
    " (:action fin :parameters () :precondition (>= (x) 3) :effect (g)))",
    "(define (problem p) (:domain n) (:init (= (x) 0)) (:goal (g)))");
  EXPECT_EQ(auto_choice(numeric), "search: lama-novelty (auto)");
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
