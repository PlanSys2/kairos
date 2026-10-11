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
#include <cstdlib>
#include <deque>
#include <filesystem>
#include <map>
#include <memory>
#include <queue>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include "heuristics/heuristic.hpp"
#include "kairos/plan.hpp"
#include "kairos/validate.hpp"
#include "search/search.hpp"
#include "support/pddl_helpers.hpp"
#include "support/pipeline_helpers.hpp"

namespace kairos::search
{
namespace
{

using kairos::testing::data_path;
using kairos::testing::dump;
using kairos::testing::pipeline;
using kairos::testing::read_file;

// A task ready to search, and what came out
struct Run
{
  std::string domain, problem;
  std::unique_ptr<kairos::testing::Pipeline> p;
  std::unique_ptr<Budget> budget;  // before the registry, which gives memory back to it
  std::unique_ptr<state::StateRegistry> reg;
  std::unique_ptr<state::StateSpace> space;
  std::vector<double> costs;
  std::unique_ptr<PlanSink> sink;
  std::unique_ptr<SearchContext> ctx;

  ::testing::AssertionResult valid(const std::vector<uint32_t> & actions) const
  {
    Plan plan = to_plan(p->sas, actions);
    auto r = validate_plan(domain, problem, plan);
    if (!r.valid()) {
      std::string msg;
      for (const auto & d : r.diagnostics) {
        msg += to_string(d) + "\n";
      }
      return ::testing::AssertionFailure() << msg << to_string(plan, PlanFormat::Ipc);
    }
    return ::testing::AssertionSuccess();
  }
};

std::unique_ptr<Run> setup(
  const std::string & domain, const std::string & problem, double seconds = 30.0,
  heuristics::CostModel model = heuristics::CostModel::Metric, std::stop_token stop = {})
{
  auto r = std::make_unique<Run>();
  r->domain = domain;
  r->problem = problem;
  r->p = pipeline(domain, problem);
  BudgetConfig cfg;
  cfg.stop = stop;
  cfg.check_interval = 1;
  cfg.deadline = BudgetConfig::Clock::now() +
    std::chrono::milliseconds(static_cast<int64_t>(seconds * 1000));
  r->budget = std::make_unique<Budget>(cfg);
  r->reg = std::make_unique<state::StateRegistry>(r->p->sas, r->budget.get());
  r->space = std::make_unique<state::StateSpace>(r->p->sas, *r->reg);
  r->costs = heuristics::action_costs(r->p->sas, model);
  r->sink = std::make_unique<PlanSink>();
  r->ctx = std::make_unique<SearchContext>(
    r->p->sas, *r->reg, *r->space, r->costs, *r->sink, r->budget.get());
  r->ctx->prune_with_bound = true;
  return r;
}

// Optimal cost by uniform-cost search over the whole space (small tasks)
double optimal(const Run & r)
{
  using E = std::pair<double, state::StateId>;
  std::priority_queue<E, std::vector<E>, std::greater<E>> q;
  std::map<state::StateId, double> g;
  state::StateId s0 = r.reg->initial();
  g[s0] = 0.0;
  q.emplace(0.0, s0);
  std::vector<uint32_t> ops;
  while (!q.empty()) {
    auto [d, s] = q.top();
    q.pop();
    if (d > g[s]) {
      continue;
    }
    if (r.space->is_goal(s)) {
      return d;
    }
    ops.clear();
    r.space->applicable(s, ops);
    for (uint32_t op : ops) {
      auto n = r.space->successor(s, op);
      if (!n) {
        continue;
      }
      double nd = d + r.costs[op];
      auto it = g.find(*n);
      if (it == g.end() || nd < it->second) {
        g[*n] = nd;
        q.emplace(nd, *n);
      }
    }
  }
  return heuristics::kInfinity;
}

const char * kBlocks =
  "(define (domain blocks) (:requirements :strips)"
  " (:predicates (on ?x ?y) (ontable ?x) (clear ?x) (handempty) (holding ?x))"
  " (:action pick-up :parameters (?x) :precondition (and (clear ?x) (ontable ?x) (handempty))"
  "  :effect (and (not (ontable ?x)) (not (clear ?x)) (not (handempty)) (holding ?x)))"
  " (:action put-down :parameters (?x) :precondition (holding ?x)"
  "  :effect (and (not (holding ?x)) (clear ?x) (handempty) (ontable ?x)))"
  " (:action stack :parameters (?x ?y) :precondition (and (holding ?x) (clear ?y))"
  "  :effect (and (not (holding ?x)) (not (clear ?y)) (clear ?x) (handempty) (on ?x ?y)))"
  " (:action unstack :parameters (?x ?y) :precondition (and (on ?x ?y) (clear ?x) (handempty))"
  "  :effect (and (holding ?x) (clear ?y) (not (clear ?x)) (not (handempty)) (not (on ?x ?y)))))";

const char * kBlocksProblem =
  "(define (problem p) (:domain blocks) (:objects a b c d e)"
  " (:init (clear c) (on c b) (on b a) (ontable a) (clear e) (on e d) (ontable d) (handempty))"
  " (:goal (and (on a b) (on b c) (on c d) (on d e))))";

TEST(SearchTest, EveryEngineFindsValidPlans)
{
  using K = EngineOptions::Kind;
  for (K kind : {K::LazyGreedy, K::EagerGreedy, K::WeightedAStar, K::EnforcedHillClimbing}) {
    for (bool preferred : {true, false}) {
      for (const char * h : {"ff", "hadd", "goalcount"}) {
        auto r = setup(kBlocks, kBlocksProblem);
        EngineOptions o;
        o.kind = kind;
        o.preferred = preferred;
        o.heuristic = h;
        o.weight = 2.0;
        Outcome out = run_engine(*r->ctx, o);
        if (kind == K::EnforcedHillClimbing && out == Outcome::Failed) {
          continue;  // incomplete: allowed to give up
        }
        ASSERT_EQ(out, Outcome::Solved) << static_cast<int>(kind) << " " << h;
        auto best = r->sink->best();
        ASSERT_TRUE(best.has_value());
        EXPECT_TRUE(r->valid(best->actions)) << static_cast<int>(kind) << " " << h;
        EXPECT_EQ(best->cost, static_cast<double>(best->actions.size()));
      }
    }
  }
}

TEST(SearchTest, WeightOneWithAnAdmissibleHeuristicIsOptimal)
{
  auto r = setup(kBlocks, kBlocksProblem);
  EngineOptions o;
  o.kind = EngineOptions::Kind::WeightedAStar;
  o.heuristic = "hmax";
  o.preferred = false;
  ASSERT_EQ(run_engine(*r->ctx, o), Outcome::Solved);
  EXPECT_EQ(r->sink->best()->cost, optimal(*r));
}

TEST(SearchTest, ExhaustingTheSpaceProvesUnsolvability)
{
  // p and q exclude each other but the goal wants both: relaxed reachable, really not
  const std::string domain =
    "(define (domain d) (:predicates (p) (q) (r))"
    " (:action mp :parameters () :precondition (r) :effect (and (p) (not (q))))"
    " (:action mq :parameters () :precondition (r) :effect (and (q) (not (p)))))";
  const std::string problem = "(define (problem pr) (:domain d) (:init (r)) (:goal (and (p) (q))))";
  using K = EngineOptions::Kind;
  for (K kind : {K::LazyGreedy, K::EagerGreedy, K::WeightedAStar}) {
    auto r = setup(domain, problem);
    EngineOptions o;
    o.kind = kind;
    EXPECT_EQ(run_engine(*r->ctx, o), Outcome::Unsolvable);
    EXPECT_FALSE(r->sink->best().has_value());
  }
  auto r = setup(domain, problem);
  EXPECT_EQ(anytime(*r->ctx, AnytimeOptions{}), AnytimeResult::Exhausted);
  // A dead end at the start: no search at all
  auto dead = setup(
    "(define (domain d) (:predicates (p) (q)) (:action a :parameters () :precondition (q)"
    " :effect (p)))", "(define (problem pr) (:domain d) (:init) (:goal (p)))");
  EXPECT_EQ(anytime(*dead->ctx, AnytimeOptions{}), AnytimeResult::Exhausted);
  EXPECT_EQ(dead->ctx->stats.expanded, 0u);
}

TEST(SearchTest, AnytimeImprovesStrictlyAndProvesOptimality)
{
  std::vector<double> seen;
  auto r = setup(kBlocks, kBlocksProblem);
  r->sink = std::make_unique<PlanSink>([&](const PlanSink::Entry & e) {seen.push_back(e.cost);});
  r->ctx = std::make_unique<SearchContext>(
    r->p->sas, *r->reg, *r->space, r->costs, *r->sink, r->budget.get());
  r->ctx->prune_with_bound = true;
  EXPECT_EQ(anytime(*r->ctx, AnytimeOptions{}), AnytimeResult::ProvenOptimal);
  ASSERT_FALSE(seen.empty());
  for (size_t i = 1; i < seen.size(); ++i) {
    EXPECT_LT(seen[i], seen[i - 1]);
  }
  EXPECT_EQ(seen.back(), optimal(*r));
  for (const auto & e : r->sink->all()) {
    EXPECT_TRUE(r->valid(e.actions));
  }
}

TEST(SearchTest, AnytimeWithMetricCosts)
{
  // A cheap long way and an expensive short one
  const std::string domain =
    "(define (domain d) (:requirements :action-costs) (:predicates (at ?l) (link ?a ?b))"
    " (:functions (total-cost) (cost ?a ?b))"
    " (:action go :parameters (?a ?b) :precondition (and (at ?a) (link ?a ?b))"
    "  :effect (and (not (at ?a)) (at ?b) (increase (total-cost) (cost ?a ?b)))))";
  const std::string problem =
    "(define (problem p) (:domain d) (:objects s m1 m2 m3 t)"
    " (:init (at s) (link s t) (link s m1) (link m1 m2) (link m2 m3) (link m3 t)"
    "  (= (cost s t) 10) (= (cost s m1) 1) (= (cost m1 m2) 1) (= (cost m2 m3) 1)"
    "  (= (cost m3 t) 1) (= (total-cost) 0))"
    " (:goal (at t)) (:metric minimize (total-cost)))";
  auto r = setup(domain, problem);
  EXPECT_EQ(anytime(*r->ctx, AnytimeOptions{}), AnytimeResult::ProvenOptimal);
  EXPECT_EQ(r->sink->best()->cost, 4.0);
  EXPECT_TRUE(r->valid(r->sink->best()->actions));
}

TEST(SearchTest, AGoalTrueAtTheStartIsAnOptimalEmptyPlan)
{
  auto r = setup(kBlocks,
      "(define (problem p) (:domain blocks) (:objects a) (:init (ontable a) (clear a)"
      " (handempty)) (:goal (ontable a)))");
  EXPECT_EQ(anytime(*r->ctx, AnytimeOptions{}), AnytimeResult::ProvenOptimal);
  ASSERT_TRUE(r->sink->best().has_value());
  EXPECT_TRUE(r->sink->best()->actions.empty());
  EXPECT_EQ(r->sink->best()->cost, 0.0);
}

TEST(SearchTest, WithoutBoundPruningThereIsNoProof)
{
  auto r = setup(kBlocks, kBlocksProblem);
  r->ctx->prune_with_bound = false;
  EXPECT_EQ(anytime(*r->ctx, AnytimeOptions{}), AnytimeResult::NoMoreImprovement);
  EXPECT_TRUE(r->sink->best().has_value());
}

TEST(SearchTest, EhcFallsBackToGreedySearch)
{
  // The relaxed plan goes through "trap" (cheat looks possible because start is never deleted
  // in the relaxation), so its only helpful action leads to a dead end; the real way is longer
  const std::string domain =
    "(define (domain d) (:predicates (start) (x) (y) (w) (goal))"
    " (:action a :parameters () :precondition (start) :effect (and (not (start)) (x)))"
    " (:action cheat :parameters () :precondition (and (x) (start)) :effect (goal))"
    " (:action b :parameters () :precondition (start) :effect (and (not (start)) (y)))"
    " (:action f1 :parameters () :precondition (y) :effect (w))"
    " (:action f2 :parameters () :precondition (w) :effect (goal)))";
  const std::string problem =
    "(define (problem p) (:domain d) (:init (start)) (:goal (goal)))";
  {
    auto e = setup(domain, problem);
    EngineOptions ehc;
    ehc.kind = EngineOptions::Kind::EnforcedHillClimbing;
    EXPECT_EQ(run_engine(*e->ctx, ehc), Outcome::Failed);
  }
  auto r = setup(domain, problem);
  AnytimeOptions o;
  o.first.kind = EngineOptions::Kind::EnforcedHillClimbing;
  o.improve = false;
  auto res = anytime(*r->ctx, o);
  EXPECT_EQ(res, AnytimeResult::NoMoreImprovement);
  ASSERT_TRUE(r->sink->best().has_value());
  EXPECT_TRUE(r->valid(r->sink->best()->actions));
}

TEST(SearchTest, BudgetAndCancellationStopTheSearch)
{
  // A big space and a deadline already past
  std::string objects;
  for (int i = 0; i < 12; ++i) {
    objects += " b" + std::to_string(i);
  }
  const std::string problem =
    "(define (problem p) (:domain blocks) (:objects" + objects + ")"
    " (:init (handempty)" +[&]() {
    std::string s;
    for (int i = 0; i < 12; ++i) {
      s += " (ontable b" + std::to_string(i) + ") (clear b" + std::to_string(i) + ")";
    }
    return s;
    } () + ") (:goal (and" +[&]() {
    std::string s;
    for (int i = 0; i + 1 < 12; ++i) {
      s += " (on b" + std::to_string(i) + " b" + std::to_string(i + 1) + ")";
    }
    return s;
    } () + ")))";
  auto r = setup(kBlocks, problem, 0.0);
  EngineOptions o;
  o.heuristic = "blind";
  EXPECT_EQ(run_engine(*r->ctx, o), Outcome::Stopped);
  // Cancellation through the stop token
  std::stop_source stop;
  auto c = setup(kBlocks, problem, 30.0, heuristics::CostModel::Metric, stop.get_token());
  stop.request_stop();
  EXPECT_EQ(anytime(*c->ctx, AnytimeOptions{}), AnytimeResult::Stopped);
}

TEST(SearchTest, PlanSinkKeepsOnlyStrictImprovementsAcrossThreads)
{
  std::atomic<int> calls{0};
  PlanSink * sink_ptr = nullptr;
  PlanSink sink([&](const PlanSink::Entry & e) {
      ++calls;
      // The callback may read the sink: it runs outside the lock
      auto b = sink_ptr->best();
      EXPECT_TRUE(b.has_value());
      EXPECT_LE(b->cost, e.cost);
    });
  sink_ptr = &sink;
  std::vector<std::thread> threads;
  for (int t = 0; t < 4; ++t) {
    threads.emplace_back([&sink, t]() {
        for (int c = 100; c > 0; --c) {
          sink.offer({static_cast<uint32_t>(t)}, static_cast<double>(c * 4 + t));
        }
      });
  }
  for (auto & th : threads) {
    th.join();
  }
  EXPECT_EQ(sink.bound(), 4.0);
  auto all = sink.all();
  for (size_t i = 1; i < all.size(); ++i) {
    EXPECT_LT(all[i].cost, all[i - 1].cost);
  }
  EXPECT_EQ(calls.load(), static_cast<int>(all.size()));
  EXPECT_FALSE(sink.offer({0}, 4.0));  // equal is not better
}

TEST(SearchTest, PlansReadAsTheValidatorExpects)
{
  auto r = setup(kBlocks, kBlocksProblem);
  ASSERT_EQ(run_engine(*r->ctx, EngineOptions{}), Outcome::Solved);
  Plan plan = to_plan(r->p->sas, r->sink->best()->actions);
  EXPECT_FALSE(plan.temporal);
  for (size_t i = 0; i < plan.steps.size(); ++i) {
    EXPECT_EQ(plan.steps[i].time, static_cast<double>(i));
    EXPECT_EQ(plan.steps[i].duration, 0.0);
  }
  auto text = to_string(plan, PlanFormat::Ipc);
  auto back = parse_plan(text);
  ASSERT_TRUE(back.ok()) << dump(back.diagnostics());
  EXPECT_EQ(back->steps.size(), plan.steps.size());
}

// --- Coverage ----------------------------------------------------------------------------------

::testing::AssertionResult solves(
  const std::string & domain, const std::string & problem, double seconds, bool * solved = nullptr)
{
  auto r = setup(domain, problem, seconds);
  AnytimeOptions o;
  o.improve = false;
  anytime(*r->ctx, o);
  auto best = r->sink->best();
  if (solved) {
    *solved = best.has_value();
  }
  if (!best) {
    return ::testing::AssertionFailure() << "no plan";
  }
  return r->valid(best->actions);
}

TEST(SearchCoverageTest, PlanSys2InstantDomains)
{
  const std::pair<const char *, const char *> pairs[] = {
    {"plansys2_executor__suave_domain", "plansys2_executor__suave_problem"},
  };
  for (const auto & [d, p] : pairs) {
    EXPECT_TRUE(solves(read_file(data_path(std::string("plansys2/") + d + ".pddl")),
      read_file(data_path(std::string("plansys2/") + p + ".pddl")), 10.0)) << d;
  }
}

TEST(SearchCoverageTest, IpcClassicalAndNumeric)
{
  const char * dir = std::getenv("KAIROS_PDDL_CORPUS");
  if (dir == nullptr) {
    GTEST_SKIP() << "KAIROS_PDDL_CORPUS not set";
  }
  std::filesystem::path base(dir);
  struct Case
  {
    const char * path;
    const char * problem;
  };
  const Case cases[] = {
    {"potassco_pddl-instances/ipc-2000/domains/logistics-strips-typed", "instance-8.pddl"},
    {"potassco_pddl-instances/ipc-2000/domains/blocks-strips-typed", "instance-8.pddl"},
    {"potassco_pddl-instances/ipc-2000/domains/freecell-strips-typed", "instance-8.pddl"},
    {"potassco_pddl-instances/ipc-2002/domains/driverlog-strips-automatic", "instance-8.pddl"},
    {"potassco_pddl-instances/ipc-2002/domains/rovers-strips-automatic", "instance-8.pddl"},
    {"potassco_pddl-instances/ipc-2002/domains/satellite-strips-automatic", "instance-8.pddl"},
    {"potassco_pddl-instances/ipc-2002/domains/zenotravel-numeric-automatic", "instance-5.pddl"},
    {"potassco_pddl-instances/ipc-2002/domains/driverlog-numeric-automatic", "instance-5.pddl"},
    {"potassco_pddl-instances/ipc-2004/domains/psr-small-strips", "instance-8.pddl"},
    {"potassco_pddl-instances/ipc-2006/domains/tpp-propositional-strips", "instance-8.pddl"},
    {"potassco_pddl-instances/ipc-2011/domains/visitall-sequential-satisficing", "instance-3.pddl"},
    {"ipc2023-numeric_ipc2023-dataset/counters", "pfile1.pddl"},
    {"ipc2023-numeric_ipc2023-dataset/fo-counters", "pfile1.pddl"},
    {"ipc2023-numeric_ipc2023-dataset/farmland", "pfile1.pddl"},
    {"ipc2023-numeric_ipc2023-dataset/sailing", "pfile1.pddl"},
    {"ipc2023-numeric_ipc2023-dataset/tpp", "pfile1.pddl"},
    {"ipc2023-numeric_ipc2023-dataset/zenotravel", "pfile1.pddl"},
    {"ipc2023-numeric_ipc2023-dataset/delivery", "pfile1.pddl"},
  };
  size_t solved = 0, tried = 0;
  for (const auto & c : cases) {
    auto d = base / c.path / "domain.pddl";
    auto p = base / c.path / "instances" / c.problem;
    if (!std::filesystem::exists(p) || !std::filesystem::exists(d)) {
      continue;
    }
    ++tried;
    bool ok = false;
    EXPECT_TRUE(solves(read_file(d), read_file(p), 20.0, &ok)) << c.path;
    solved += ok ? 1 : 0;
  }
  EXPECT_GT(tried, 0u);
  EXPECT_EQ(solved, tried);
}

}  // namespace
}  // namespace kairos::search
