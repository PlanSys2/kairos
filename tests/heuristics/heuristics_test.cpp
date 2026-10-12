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

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <deque>
#include <filesystem>
#include <map>
#include <memory>
#include <queue>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "heuristics/heuristic.hpp"
#include "heuristics/relaxed.hpp"
#include "sas/translate.hpp"
#include "state/registry.hpp"
#include "state/successors.hpp"
#include "support/pddl_helpers.hpp"
#include "support/pipeline_helpers.hpp"

namespace kairos::heuristics
{
namespace
{

using kairos::testing::data_path;
using kairos::testing::pipeline;
using kairos::testing::read_file;
using state::StateId;

// A task, its state space and an evaluator for any heuristic at any state
struct Bench
{
  std::unique_ptr<kairos::testing::Pipeline> p;
  std::unique_ptr<state::StateRegistry> reg;
  std::unique_ptr<state::StateSpace> space;
  std::vector<double> costs;

  HeuristicResult eval(Heuristic & h, StateId s) const
  {
    std::vector<uint32_t> v;
    reg->unpack(s, v);
    std::vector<double> n(reg->numbers(s).begin(), reg->numbers(s).end());
    EvalContext ctx{s, v.data(), n.data(), 0.0};
    return h.evaluate(ctx);
  }
  double value(const std::string & name, StateId s) const
  {
    auto h = make_heuristic(name, p->sas, costs);
    return eval(*h, s).value;
  }
  StateId after(StateId s, const std::string & action) const
  {
    for (uint32_t i = 0; i < p->sas.actions.size(); ++i) {
      if (sas::instance_name(p->sas, p->sas.actions[i]) == action) {
        auto n = space->successor(s, i);
        EXPECT_TRUE(n.has_value()) << action;
        return n ? *n : s;
      }
    }
    ADD_FAILURE() << "no action " << action;
    return s;
  }
};

std::unique_ptr<Bench> bench(
  const std::string & domain, const std::string & problem, CostModel model = CostModel::Unit)
{
  auto b = std::make_unique<Bench>();
  b->p = pipeline(domain, problem);
  b->reg = std::make_unique<state::StateRegistry>(b->p->sas);
  b->space = std::make_unique<state::StateSpace>(b->p->sas, *b->reg);
  b->costs = action_costs(b->p->sas, model);
  return b;
}

const char * kWorld =
  "(define (domain w) (:requirements :typing :negative-preconditions :fluents"
  "  :derived-predicates :action-costs)"
  " (:types place) (:constants s0 s1 s2 s3 - place)"
  " (:predicates (at ?p - place) (link ?a ?b - place) (p) (q) (r) (key) (open) (done))"
  " (:functions (x) (total-cost))"
  " (:action move :parameters (?a ?b - place) :precondition (and (at ?a) (link ?a ?b))"
  "  :effect (and (not (at ?a)) (at ?b)))"
  " %s)";

std::string world(const std::string & extra)
{
  char buf[8192];
  std::snprintf(buf, sizeof(buf), kWorld, extra.c_str());
  return buf;
}

std::string problem(const std::string & init, const std::string & goal)
{
  return "(define (problem pr) (:domain w) (:init (link s0 s1) (link s1 s2) (link s2 s3) " +
         init + ") (:goal " + goal + "))";
}

TEST(HeuristicsTest, ChainToOneGoal)
{
  auto b = bench(world(""), problem("(at s0)", "(at s3)"));
  StateId s0 = b->reg->initial();
  EXPECT_EQ(b->value("blind", s0), 1.0);
  EXPECT_EQ(b->value("goalcount", s0), 1.0);
  EXPECT_EQ(b->value("hmax", s0), 3.0);
  EXPECT_EQ(b->value("hadd", s0), 3.0);
  EXPECT_EQ(b->value("ff", s0), 3.0);
  StateId s1 = b->after(s0, "(move s0 s1)");
  EXPECT_EQ(b->value("hmax", s1), 2.0);
  StateId goal = b->after(b->after(s1, "(move s1 s2)"), "(move s2 s3)");
  for (const char * h : {"blind", "goalcount", "hmax", "hadd", "ff"}) {
    EXPECT_EQ(b->value(h, goal), 0.0) << h;
  }
}

TEST(HeuristicsTest, IndependentAndSharedSubgoals)
{
  // p and q need r first: hadd counts r twice, a relaxed plan once
  auto b = bench(world(
      "(:action mr :parameters () :precondition () :effect (r))"
      " (:action mp :parameters () :precondition (r) :effect (p))"
      " (:action mq :parameters () :precondition (r) :effect (q))"),
    problem("(at s0)", "(and (p) (q) (at s2))"));
  StateId s0 = b->reg->initial();
  EXPECT_EQ(b->value("goalcount", s0), 3.0);
  EXPECT_EQ(b->value("hmax", s0), 2.0);
  EXPECT_EQ(b->value("hadd", s0), 2.0 + 2.0 + 2.0);
  EXPECT_EQ(b->value("ff", s0), 3.0 + 2.0);
}

TEST(HeuristicsTest, PreferredOperatorsAreApplicableRelaxedPlanActions)
{
  auto b = bench(world(
      "(:action mr :parameters () :precondition () :effect (r))"
      " (:action mp :parameters () :precondition (r) :effect (p))"),
    problem("(at s0)", "(and (p) (at s1))"));
  auto ff = make_heuristic("ff", b->p->sas, b->costs);
  auto r = b->eval(*ff, b->reg->initial());
  std::set<std::string> names;
  for (uint32_t a : r.preferred) {
    names.insert(sas::instance_name(b->p->sas, b->p->sas.actions[a]));
  }
  EXPECT_EQ(names, (std::set<std::string>{"(move s0 s1)", "(mr)"}));
}

TEST(HeuristicsTest, NegatedConditions)
{
  // wave needs to not be at s0: one move away
  auto b = bench(world("(:action wave :parameters () :precondition (not (at s0)) :effect (p))"),
      problem("(at s0)", "(p)"));
  StateId s0 = b->reg->initial();
  EXPECT_EQ(b->value("hmax", s0), 2.0);
  EXPECT_EQ(b->value("ff", s0), 2.0);
  EXPECT_EQ(b->value("hmax", b->after(s0, "(move s0 s1)")), 1.0);
}

TEST(HeuristicsTest, NumericConditionsNeedRepetitions)
{
  // x from 0 to at least 10 by steps of 3: four steps; hmax gives the bound 10/3
  auto b = bench(world("(:action inc :parameters () :precondition () :effect (increase (x) 3))"),
      problem("(at s0) (= (x) 0)", "(>= (x) 10)"));
  StateId s0 = b->reg->initial();
  EXPECT_EQ(b->value("hadd", s0), 4.0);
  EXPECT_EQ(b->value("ff", s0), 4.0);
  EXPECT_NEAR(b->value("hmax", s0), 10.0 / 3.0, 1e-9);
  EXPECT_EQ(b->value("goalcount", s0), 1.0);
  auto hmax = make_heuristic("hmax", b->p->sas, b->costs);
  EXPECT_TRUE(hmax->admissible());
  // After three steps (x = 9) one more
  StateId s = s0;
  for (int i = 0; i < 3; ++i) {
    s = b->after(s, "(inc)");
  }
  EXPECT_EQ(b->value("hadd", s), 1.0);
}

TEST(HeuristicsTest, StrictConditionsAndTheBestContributor)
{
  // x > 9: with +3 four steps (12), with +5 two (10); the best is two
  auto b = bench(world(
      "(:action inc :parameters () :precondition () :effect (increase (x) 3))"
      " (:action big :parameters () :precondition () :effect (increase (x) 5))"),
    problem("(at s0) (= (x) 0)", "(> (x) 9)"));
  StateId s0 = b->reg->initial();
  EXPECT_EQ(b->value("hadd", s0), 2.0);
  EXPECT_NEAR(b->value("hmax", s0), 9.0 / 5.0, 1e-9);
  auto only3 =
    bench(world("(:action inc :parameters () :precondition () :effect (increase (x) 3))"),
      problem("(at s0) (= (x) 0)", "(> (x) 9)"));
  EXPECT_EQ(only3->value("hadd", only3->reg->initial()), 4.0);  // 3, 6, 9 is not enough
}

TEST(HeuristicsTest, NumericPreconditionsGateActions)
{
  // done needs x >= 6 and the robot at s2
  auto b = bench(world(
      "(:action inc :parameters () :precondition (at s1) :effect (increase (x) 2))"
      " (:action finish :parameters () :precondition (and (at s2) (>= (x) 6)) :effect (done))"),
    problem("(at s0) (= (x) 0)", "(done)"));
  StateId s0 = b->reg->initial();
  // hadd: at s2 costs 2; x >= 6: reach s1 (1) and inc three times (3); finish (1)
  EXPECT_EQ(b->value("hadd", s0), 2.0 + 4.0 + 1.0);
  // relaxed plan: move s0 s1, move s1 s2, inc x3, finish
  EXPECT_EQ(b->value("ff", s0), 6.0);
  // hmax: at s2 costs 2; x >= 6 costs max(1, 6 / 2) = 3; finish max(2, 3) + 1
  EXPECT_EQ(b->value("hmax", s0), 4.0);
}

TEST(HeuristicsTest, MetricCostsFromTotalCost)
{
  auto b = bench(world(
      "(:action slow :parameters () :precondition () :effect (and (p) (increase (total-cost) 5)))"
      " (:action fast :parameters () :precondition ()"
      "  :effect (and (q) (increase (total-cost) 1)))"),
    "(define (problem pr) (:domain w) (:init (at s0) (= (total-cost) 0) (link s0 s1))"
    " (:goal (and (p) (q))) (:metric minimize (total-cost)))", CostModel::Metric);
  EXPECT_TRUE(metric_costs_apply(b->p->sas));
  StateId s0 = b->reg->initial();
  EXPECT_EQ(b->value("hmax", s0), 5.0);
  EXPECT_EQ(b->value("hadd", s0), 6.0);
  // move has no cost effect: 0
  for (uint32_t i = 0; i < b->p->sas.actions.size(); ++i) {
    if (sas::instance_name(b->p->sas, b->p->sas.actions[i]) == "(move s0 s1)") {
      EXPECT_EQ(b->costs[i], 0.0);
    }
  }
  // Unit costs when the metric is not action costs
  auto u = bench(world(""), problem("(at s0)", "(at s3)"), CostModel::Metric);
  EXPECT_FALSE(metric_costs_apply(u->p->sas));
  EXPECT_EQ(u->costs, std::vector<double>(u->p->sas.actions.size(), 1.0));
}

TEST(HeuristicsTest, DerivedGoalsCostTheirSupport)
{
  auto b = bench(world(
      "(:derived (done) (and (at s2) (p)))"
      " (:action mp :parameters () :precondition () :effect (p))"),
    problem("(at s0)", "(done)"));
  StateId s0 = b->reg->initial();
  EXPECT_EQ(b->value("hmax", s0), 2.0);
  EXPECT_EQ(b->value("hadd", s0), 3.0);
  EXPECT_EQ(b->value("ff", s0), 3.0);
}

TEST(HeuristicsTest, ChangingTheGoalOfTheRelaxedGraph)
{
  auto b = bench(world(""), problem("(at s0)", "(at s3)"));
  const auto & t = b->p->sas;
  auto fact = [&](const std::string & name) {
      for (sas::VarId v = 0; v < t.variables.size(); ++v) {
        for (uint32_t x = 0; x < t.variables[v].size(); ++x) {
          if (sas::fact_name(t, {v, x}).find(name) != std::string::npos) {
            return sas::Fact{v, x};
          }
        }
      }
      ADD_FAILURE() << "no fact " << name;
      return sas::Fact{};
    };
  RelaxedGraph g(t, b->costs);
  std::vector<uint32_t> v;
  StateId s0 = b->reg->initial();
  b->reg->unpack(s0, v);
  std::vector<double> n(b->reg->numbers(s0).begin(), b->reg->numbers(s0).end());
  EvalContext ctx{s0, v.data(), n.data(), 0.0};
  EXPECT_EQ(g.run(ctx, RelaxedGraph::Mode::Add), 3.0);
  g.set_goal({fact("at s2")});
  EXPECT_EQ(g.run(ctx, RelaxedGraph::Mode::Add), 2.0);
  EXPECT_EQ(g.relaxed_plan(nullptr), 2.0);
  // Any of them: the nearest, and which one it was
  g.set_goal({fact("at s3"), fact("at s1"), fact("at s2")}, true);
  EXPECT_EQ(g.run(ctx, RelaxedGraph::Mode::Add), 1.0);
  auto f = g.fact_of(g.goal_fact());
  ASSERT_TRUE(f);
  EXPECT_EQ(sas::fact_name(t, {f->first, f->second}).find("at s1") != std::string::npos, true);
  // All of them again (hadd sums them), and a goal true already, and none
  g.set_goal({fact("at s3"), fact("at s1")});
  EXPECT_EQ(g.run(ctx, RelaxedGraph::Mode::Add), 4.0);
  EXPECT_EQ(g.relaxed_plan(nullptr), 3.0);
  g.set_goal({fact("at s0")});
  EXPECT_EQ(g.run(ctx, RelaxedGraph::Mode::Add), 0.0);
  g.set_goal({});
  EXPECT_EQ(g.run(ctx, RelaxedGraph::Mode::Add), 0.0);
  g.set_goal({fact("at s3")}, true);
  EXPECT_EQ(g.run(ctx, RelaxedGraph::Mode::Max), 3.0);
}

TEST(HeuristicsTest, DeadEnds)
{
  // drop loses the key for good: then open is unreachable even relaxed
  auto b = bench(world(
      "(:action drop :parameters () :precondition (key) :effect (not (key)))"
      " (:action op :parameters () :precondition (key) :effect (open))"),
    problem("(at s0) (key)", "(open)"));
  StateId s0 = b->reg->initial();
  StateId lost = b->after(s0, "(drop)");
  for (const char * name : {"hmax", "hadd", "ff"}) {
    auto h = make_heuristic(name, b->p->sas, b->costs);
    EXPECT_FALSE(b->eval(*h, s0).dead_end) << name;
    auto r = b->eval(*h, lost);
    EXPECT_TRUE(r.dead_end) << name;
    EXPECT_EQ(r.value, kInfinity) << name;
  }
  EXPECT_FALSE(b->eval(*make_heuristic("blind", b->p->sas, b->costs), lost).dead_end);
}

TEST(HeuristicsTest, UnknownNames)
{
  auto b = bench(world(""), problem("(at s0)", "(at s3)"));
  EXPECT_EQ(make_heuristic("bogus", b->p->sas, b->costs), nullptr);
  for (const char * n : {"blind", "goalcount", "hmax", "hadd", "ff"}) {
    auto h = make_heuristic(n, b->p->sas, b->costs);
    ASSERT_NE(h, nullptr);
    EXPECT_EQ(h->name(), n);
  }
}

// --- Against optimal costs over whole state spaces ------------------------------------------

// The reachable graph (up to a limit) and h* by Dijkstra back from the goal states
struct Space
{
  bool complete = false;
  std::vector<StateId> states;
  std::map<StateId, double> h_star;
};

Space explore(const Bench & b, size_t limit)
{
  Space out;
  std::map<StateId, std::vector<std::pair<StateId, double>>> back;
  std::deque<StateId> open{b.reg->initial()};
  std::set<StateId> seen{b.reg->initial()};
  std::vector<uint32_t> ops;
  while (!open.empty()) {
    if (seen.size() > limit) {
      return out;
    }
    StateId s = open.front();
    open.pop_front();
    out.states.push_back(s);
    ops.clear();
    b.space->applicable(s, ops);
    for (uint32_t op : ops) {
      auto n = b.space->successor(s, op);
      if (!n) {
        continue;
      }
      back[*n].emplace_back(s, b.costs[op]);
      if (seen.insert(*n).second) {
        open.push_back(*n);
      }
    }
  }
  out.complete = true;
  using E = std::pair<double, StateId>;
  std::priority_queue<E, std::vector<E>, std::greater<E>> q;
  for (StateId s : out.states) {
    if (b.space->is_goal(s)) {
      out.h_star[s] = 0.0;
      q.emplace(0.0, s);
    }
  }
  while (!q.empty()) {
    auto [d, s] = q.top();
    q.pop();
    if (d > out.h_star[s]) {
      continue;
    }
    for (const auto & [prev, c] : back[s]) {
      auto it = out.h_star.find(prev);
      if (it == out.h_star.end() || d + c < it->second) {
        out.h_star[prev] = d + c;
        q.emplace(d + c, prev);
      }
    }
  }
  return out;
}

::testing::AssertionResult properties(
  const Bench & b, size_t limit = 20000, bool * complete = nullptr)
{
  Space sp = explore(b, limit);
  if (complete) {
    *complete = sp.complete;
  }
  if (!sp.complete) {
    return ::testing::AssertionSuccess() << "too big, skipped";
  }
  auto hmax = make_heuristic("hmax", b.p->sas, b.costs);
  auto hadd = make_heuristic("hadd", b.p->sas, b.costs);
  auto ff = make_heuristic("ff", b.p->sas, b.costs);
  size_t checked_fresh = 0;
  for (StateId s : sp.states) {
    auto it = sp.h_star.find(s);
    double star = it == sp.h_star.end() ? kInfinity : it->second;
    auto mx = b.eval(*hmax, s);
    auto ad = b.eval(*hadd, s);
    auto f = b.eval(*ff, s);
    if (hmax->admissible() && mx.value > star + 1e-9) {
      return ::testing::AssertionFailure() << "hmax " << mx.value << " above h* " << star;
    }
    for (const auto * r : {&mx, &ad, &f}) {
      if (r->dead_end && star != kInfinity) {
        return ::testing::AssertionFailure() << "a dead end that is not: h* " << star;
      }
    }
    if (ad.value + 1e-9 < mx.value || f.value + 1e-9 < mx.value) {
      return ::testing::AssertionFailure() << "hadd " << ad.value << " or ff " << f.value <<
             " below hmax " << mx.value;
    }
    // One instance over many states gives what a new one gives (its per-state values are
    // reset lazily)
    if (checked_fresh++ < 300) {
      for (const char * name : {"hmax", "hadd", "ff"}) {
        auto fresh = make_heuristic(name, b.p->sas, b.costs);
        double reused = std::string(name) == "hmax" ? mx.value :
          (std::string(name) == "hadd" ? ad.value : f.value);
        double v = b.eval(*fresh, s).value;
        if (!(v == reused || (std::isinf(v) && std::isinf(reused)))) {
          return ::testing::AssertionFailure() << name << " reused " << reused << ", new " << v;
        }
      }
    }
    std::vector<uint32_t> ops;
    b.space->applicable(s, ops);
    for (uint32_t a : f.preferred) {
      if (std::find(ops.begin(), ops.end(), a) == ops.end()) {
        return ::testing::AssertionFailure() << "a preferred operator that is not applicable";
      }
    }
  }
  return ::testing::AssertionSuccess();
}

TEST(HeuristicsPropertiesTest, SmallDomains)
{
  EXPECT_TRUE(properties(*bench(world(
      "(:action mr :parameters () :precondition () :effect (r))"
      " (:action mp :parameters () :precondition (and (r) (not (at s0))) :effect (p))"
      " (:action unr :parameters () :precondition (p) :effect (and (not (r)) (q)))"
      " (:action drop :parameters () :precondition (key) :effect (not (key)))"
      " (:action op :parameters () :precondition (and (key) (at s3)) :effect (open))"),
    problem("(at s0) (key)", "(and (open) (q))"))));
  EXPECT_TRUE(properties(*bench(world(
      "(:action inc :parameters () :precondition (at s1) :effect (increase (x) 2))"
      " (:action dec :parameters () :precondition (> (x) 0) :effect (decrease (x) 1))"
      " (:action finish :parameters () :precondition (and (at s2) (>= (x) 5)) :effect (done))"),
    problem("(at s0) (= (x) 0)", "(and (done) (at s0))"))));
  EXPECT_TRUE(properties(*bench(world(
      "(:action slow :parameters () :precondition (at s1) :effect (and (p)"
      "   (increase (total-cost) 5)))"
      " (:action fast :parameters () :precondition (at s3) :effect (and (p)"
      "   (increase (total-cost) 1)))"),
    "(define (problem pr) (:domain w) (:init (at s0) (= (total-cost) 0) (link s0 s1)"
    " (link s1 s2) (link s2 s3) (link s3 s0) (link s1 s0)) (:goal (and (p) (at s0)))"
    " (:metric minimize (total-cost)))", CostModel::Metric)));
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

TEST(HeuristicsPropertiesTest, BlocksWorld)
{
  auto b = bench(kBlocks,
      "(define (problem p) (:domain blocks) (:objects a b c d)"
      " (:init (clear c) (on c b) (on b a) (ontable a) (clear d) (ontable d) (handempty))"
      " (:goal (and (on a b) (on b c) (on c d))))");
  EXPECT_TRUE(properties(*b));
  // The textbook values at the start
  StateId s0 = b->reg->initial();
  EXPECT_GT(b->value("ff", s0), 0.0);
  EXPECT_LE(b->value("hmax", s0), b->value("hadd", s0));
}

TEST(HeuristicsPropertiesTest, PlanSys2InstantDomains)
{
  const std::pair<const char *, const char *> pairs[] = {
    {"plansys2_executor__suave_domain", "plansys2_executor__suave_problem"},
    {"plansys2_pddl_parser__dom1", "plansys2_pddl_parser__prob1"},
  };
  for (const auto & [d, p] : pairs) {
    auto b = bench(read_file(data_path(std::string("plansys2/") + d + ".pddl")),
        read_file(data_path(std::string("plansys2/") + p + ".pddl")));
    EXPECT_TRUE(properties(*b)) << d;
  }
}

TEST(HeuristicsPropertiesTest, IpcDomains)
{
  const char * dir = std::getenv("KAIROS_PDDL_CORPUS");
  if (dir == nullptr) {
    GTEST_SKIP() << "KAIROS_PDDL_CORPUS not set";
  }
  std::filesystem::path base = std::filesystem::path(dir) / "potassco_pddl-instances";
  const char * domains[] = {
    "ipc-1998/domains/gripper-round-1-strips", "ipc-2000/domains/blocks-strips-typed",
    "ipc-2000/domains/elevator-strips-simple-typed", "ipc-2000/domains/elevator-adl-full-typed",
    "ipc-2002/domains/depots-strips-automatic", "ipc-2004/domains/psr-small-strips",
    "ipc-2004/domains/promela-dining-philosophers-derived-predicates-adl",
    "ipc-2002/domains/driverlog-numeric-automatic", "ipc-2008/domains/sokoban-sequential-optimal",
  };
  size_t complete = 0;
  for (const char * dom : domains) {
    auto d = base / dom / "domain.pddl";
    auto p = base / dom / "instances" / "instance-1.pddl";
    if (!std::filesystem::exists(d)) {
      d = base / dom / "domains" / "domain-1.pddl";
    }
    if (!std::filesystem::exists(p) || !std::filesystem::exists(d)) {
      continue;
    }
    bool whole = false;
    EXPECT_TRUE(properties(*bench(read_file(d), read_file(p), CostModel::Metric), 30000, &whole))
      << dom;
    complete += whole ? 1 : 0;
  }
  EXPECT_GE(complete, 5u);  // explored whole: the checks are meaningful
}

// IPC 2023 numeric domains (first instances): no false dead ends, hmax never above h*
TEST(HeuristicsPropertiesTest, IpcNumericDomains)
{
  const char * dir = std::getenv("KAIROS_PDDL_CORPUS");
  if (dir == nullptr) {
    GTEST_SKIP() << "KAIROS_PDDL_CORPUS not set";
  }
  std::filesystem::path base = std::filesystem::path(dir) / "ipc2023-numeric_ipc2023-dataset";
  size_t complete = 0;
  for (const char * dom : {"counters", "fo-counters", "farmland", "fo-farmland", "drone",
      "mprime", "block-grouping", "tpp", "delivery"})
  {
    auto d = base / dom / "domain.pddl";
    auto p = base / dom / "instances" / "pfile1.pddl";
    if (!std::filesystem::exists(p) || !std::filesystem::exists(d)) {
      continue;
    }
    bool whole = false;
    EXPECT_TRUE(properties(*bench(read_file(d), read_file(p), CostModel::Metric), 100000, &whole))
      << dom;
    complete += whole ? 1 : 0;
  }
  EXPECT_GE(complete, 2u);  // numeric state spaces grow fast: some are too big to explore
}

TEST(HeuristicsTest, EvaluationIsFast)
{
  // ff on a 30-place line with 9 goals: well under a millisecond per state in release
  std::string objects, links, goals;
  for (int i = 0; i < 30; ++i) {
    objects += " c" + std::to_string(i);
    if (i + 1 < 30) {
      links += " (link c" + std::to_string(i) + " c" + std::to_string(i + 1) + ")";
    }
  }
  for (int i = 1; i <= 9; ++i) {
    goals += " (seen c" + std::to_string(3 * i) + ")";
  }
  auto b = bench(
    "(define (domain l) (:predicates (at ?c) (link ?a ?b) (seen ?c))"
    " (:action go :parameters (?a ?b) :precondition (and (at ?a) (link ?a ?b))"
    "  :effect (and (not (at ?a)) (at ?b) (seen ?b))))",
    "(define (problem p) (:domain l) (:objects" + objects + ") (:init (at c0)" + links +
    ") (:goal (and" + goals + ")))");
  auto ff = make_heuristic("ff", b->p->sas, b->costs);
  StateId s0 = b->reg->initial();
  auto t0 = std::chrono::steady_clock::now();
  double h = 0;
  for (int i = 0; i < 1000; ++i) {
    h = b->eval(*ff, s0).value;
  }
  double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0)
    .count();
  EXPECT_EQ(h, 27.0);  // walk to c27, seeing c3, c6, ..., c27 on the way
  EXPECT_LT(ms, 5000.0);  // generous: debug and sanitizer builds
}

TEST(HeuristicsPropertiesTest, ChangesThatDependOnTheStateAreNeverFalseDeadEnds)
{
  // IPC 2023 fo-counters: x grows by a rate that starts at 0; raising the rate first works
  auto b = bench(world(
      "(:action inc :parameters () :precondition () :effect (increase (x) (total-cost)))"
      " (:action rate :parameters () :precondition () :effect (increase (total-cost) 1))"),
    problem("(at s0) (= (x) 0) (= (total-cost) 0)", "(>= (x) 3)"));
  StateId s0 = b->reg->initial();
  for (const char * name : {"hmax", "hadd", "ff"}) {
    auto h = make_heuristic(name, b->p->sas, b->costs);
    auto r = b->eval(*h, s0);
    EXPECT_FALSE(r.dead_end) << name;
    EXPECT_GE(r.value, 1.0) << name;
  }
  EXPECT_TRUE(properties(*b));
  // An undefined fluent that an action assigns is not a dead end either
  auto u = bench(world(
      "(:action set :parameters () :precondition () :effect (assign (x) 0))"
      " (:action inc :parameters () :precondition () :effect (increase (x) 1))"),
    problem("(at s0)", "(>= (x) 2)"));
  for (const char * name : {"hmax", "hadd", "ff"}) {
    auto h = make_heuristic(name, u->p->sas, u->costs);
    EXPECT_FALSE(u->eval(*h, u->reg->initial()).dead_end) << name;
  }
}

}  // namespace
}  // namespace kairos::heuristics
