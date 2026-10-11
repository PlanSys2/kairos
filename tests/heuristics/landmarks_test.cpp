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
#include <cstdlib>
#include <deque>
#include <filesystem>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "heuristics/landmarks.hpp"
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

struct Bench
{
  std::unique_ptr<kairos::testing::Pipeline> p;
  std::unique_ptr<state::StateRegistry> reg;
  std::unique_ptr<state::StateSpace> space;
  std::vector<double> costs;

  std::vector<uint32_t> values(StateId s) const
  {
    std::vector<uint32_t> v;
    reg->unpack(s, v);
    return v;
  }
  EvalContext context(StateId s, std::vector<uint32_t> & v, std::vector<double> & n) const
  {
    v = values(s);
    n.assign(reg->numbers(s).begin(), reg->numbers(s).end());
    return EvalContext{s, v.data(), n.data(), 0.0};
  }
  LandmarkGraph landmarks() const
  {
    RelaxedGraph g(p->sas, costs);
    std::vector<uint32_t> v;
    std::vector<double> n;
    auto ctx = context(reg->initial(), v, n);
    return LandmarkGraph::build(p->sas, g, ctx, costs);
  }
  std::string name(const Landmark & l) const {return sas::fact_name(p->sas, l.fact);}
  std::set<std::string> names(const LandmarkGraph & lg) const
  {
    std::set<std::string> out;
    for (const auto & l : lg.landmarks()) {
      out.insert(name(l));
    }
    return out;
  }
  // "a < b" for each greedy-necessary ordering
  std::set<std::string> orderings(const LandmarkGraph & lg) const
  {
    std::set<std::string> out;
    for (const auto & l : lg.landmarks()) {
      for (uint32_t q : l.parents) {
        out.insert(name(lg.landmarks()[q]) + " < " + name(l));
      }
    }
    return out;
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
  uint32_t action(const std::string & name) const
  {
    for (uint32_t i = 0; i < p->sas.actions.size(); ++i) {
      if (sas::instance_name(p->sas, p->sas.actions[i]) == name) {
        return i;
      }
    }
    ADD_FAILURE() << "no action " << name;
    return 0;
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

const char * kRooms =
  "(define (domain rooms) (:requirements :strips :typing)"
  " (:types place) (:constants s0 s1 s2 s3 s4 - place)"
  " (:predicates (at ?p - place) (link ?a ?b - place) (key) (open) (done))"
  " (:action move :parameters (?a ?b - place) :precondition (and (at ?a) (link ?a ?b))"
  "  :effect (and (not (at ?a)) (at ?b)))"
  " (:action take :parameters () :precondition (at s1) :effect (key))"
  " (:action unlock :parameters () :precondition (and (key) (at s2)) :effect (open))"
  " (:action finish :parameters () :precondition (and (open) (at s3)) :effect (done)))";

std::string rooms(const std::string & links, const std::string & goal)
{
  return "(define (problem p) (:domain rooms) (:init (at s0) " + links + ") (:goal " + goal +
         "))";
}

TEST(LandmarksTest, ChainOfRooms)
{
  auto b = bench(kRooms, rooms("(link s0 s1) (link s1 s2) (link s2 s3)", "(at s3)"));
  auto lg = b->landmarks();
  EXPECT_EQ(b->names(lg), (std::set<std::string>{"(at s0)", "(at s1)", "(at s2)", "(at s3)"}));
  EXPECT_EQ(
    b->orderings(lg),
    (std::set<std::string>{"(at s0) < (at s1)", "(at s1) < (at s2)", "(at s2) < (at s3)"}));
  for (const auto & l : lg.landmarks()) {
    EXPECT_EQ(l.goal, b->name(l) == "(at s3)");
    EXPECT_EQ(l.initially_true, b->name(l) == "(at s0)");
    EXPECT_EQ(l.cost, b->name(l) == "(at s0)" ? 0.0 : 1.0);  // nothing achieves (at s0)
  }
}

TEST(LandmarksTest, TwoRoutesShareOnlyTheirEnds)
{
  // s0 -> s1 -> s3 or s0 -> s2 -> s3: neither s1 nor s2 is a landmark, and the first
  // achievers of (at s3) share nothing
  auto b = bench(kRooms, rooms("(link s0 s1) (link s1 s3) (link s0 s2) (link s2 s3)", "(at s3)"));
  auto lg = b->landmarks();
  EXPECT_EQ(b->names(lg), (std::set<std::string>{"(at s3)"}));
  EXPECT_TRUE(b->orderings(lg).empty());
}

TEST(LandmarksTest, KeysAndDoors)
{
  // done needs open (at s3); open needs key and at s2; key needs at s1
  auto b = bench(
    kRooms, rooms("(link s0 s1) (link s1 s2) (link s2 s3) (link s1 s0) (link s2 s1)", "(done)"));
  auto lg = b->landmarks();
  auto names = b->names(lg);
  for (const char * n : {"(done)", "(open)", "(key)", "(at s1)", "(at s2)", "(at s3)"}) {
    EXPECT_TRUE(names.count(n)) << n;
  }
  auto ord = b->orderings(lg);
  for (const char * o : {"(open) < (done)", "(at s3) < (done)", "(key) < (open)",
      "(at s2) < (open)", "(at s1) < (key)"})
  {
    EXPECT_TRUE(ord.count(o)) << o;
  }
}

TEST(LandmarksTest, CostsAreTheCheapestAchiever)
{
  auto b = bench(
    "(define (domain c) (:requirements :strips :action-costs) (:predicates (p) (q))"
    " (:functions (total-cost))"
    " (:action cheap :parameters () :precondition () :effect (and (p) (increase (total-cost) 2)))"
    " (:action dear :parameters () :precondition () :effect (and (p) (increase (total-cost) 7)))"
    " (:action goal :parameters () :precondition (p) :effect (and (q)"
    "  (increase (total-cost) 3))))",
    "(define (problem x) (:domain c) (:init (= (total-cost) 0)) (:goal (q))"
    " (:metric minimize (total-cost)))", CostModel::Metric);
  auto lg = b->landmarks();
  std::map<std::string, double> cost;
  for (const auto & l : lg.landmarks()) {
    cost[b->name(l)] = l.cost;
  }
  EXPECT_EQ(cost["(p)"], 2.0);
  EXPECT_EQ(cost["(q)"], 3.0);
}

TEST(LandmarksTest, CountReachedAndNeededAgain)
{
  auto b = bench(
    kRooms, rooms("(link s0 s1) (link s1 s2) (link s2 s3) (link s1 s0) (link s2 s1)",
    "(and (done) (at s0))"));
  auto h = make_landmark_count(b->p->sas, b->costs);
  ASSERT_TRUE(h->path_dependent());
  std::vector<uint32_t> v;
  std::vector<double> n;
  StateId s0 = b->reg->initial();
  auto c0 = b->context(s0, v, n);
  h->initial(c0);
  auto r0 = h->evaluate(c0);
  auto lg = b->landmarks();
  // Not reached: every landmark but (at s0), which is a goal and true now
  EXPECT_EQ(r0.value, static_cast<double>(lg.landmarks().size() - 1));
  // Preferred: the first move, the only achiever of a landmark whose parents are reached
  EXPECT_EQ(r0.preferred, (std::vector<uint32_t>{b->action("(move s0 s1)")}));

  // Along the plan the count never grows, except (at s0) needed again once left
  StateId s = s0;
  double last = r0.value;
  const char * plan[] = {"(move s0 s1)", "(take)", "(move s1 s2)", "(unlock)", "(move s2 s3)",
    "(finish)"};
  for (const char * a : plan) {
    StateId t = b->after(s, a);
    auto ct = b->context(t, v, n);
    h->transition(s, b->action(a), ct);
    auto r = h->evaluate(ct);
    if (std::string(a) == "(move s0 s1)") {
      EXPECT_EQ(r.value, last) << a;  // (at s1) reached, (at s0) needed again
    } else {
      EXPECT_EQ(r.value, last - 1.0) << a;
    }
    last = r.value;
    s = t;
  }
  // Only (at s0) is left: needed again, a goal
  EXPECT_EQ(last, 1.0);
}

TEST(LandmarksTest, AStateReachedTwiceKeepsWhatBothPathsReached)
{
  // s0 -> s1 -> s2 or straight s0 -> s2; (key) is not a landmark of s2, but the first path
  // takes it
  auto b = bench(kRooms, rooms("(link s0 s1) (link s1 s2) (link s0 s2) (link s2 s3)", "(done)"));
  auto h = make_landmark_count(b->p->sas, b->costs);
  std::vector<uint32_t> v;
  std::vector<double> n;
  StateId s0 = b->reg->initial();
  auto c0 = b->context(s0, v, n);
  h->initial(c0);
  // Path 1 to (at s2) without key: s0 -> s2
  StateId a = b->after(s0, "(move s0 s2)");
  auto ca = b->context(a, v, n);
  h->transition(s0, b->action("(move s0 s2)"), ca);
  double direct = h->evaluate(ca).value;
  // Path 2: s0 -> s1 -> s2 reaches (at s1) too, but the same state keeps the intersection
  StateId s1 = b->after(s0, "(move s0 s1)");
  auto c1 = b->context(s1, v, n);
  h->transition(s0, b->action("(move s0 s1)"), c1);
  StateId again = b->after(s1, "(move s1 s2)");
  ASSERT_EQ(again, a);
  auto cg = b->context(again, v, n);
  h->transition(s1, b->action("(move s1 s2)"), cg);
  EXPECT_EQ(h->evaluate(cg).value, direct);
}

// --- Soundness over whole state spaces -------------------------------------------------------

struct Graph
{
  bool complete = false;
  std::vector<StateId> states;
  std::map<StateId, std::vector<std::pair<StateId, uint32_t>>> next;
};

Graph explore(const Bench & b, size_t limit)
{
  Graph g;
  std::deque<StateId> open{b.reg->initial()};
  std::set<StateId> seen{b.reg->initial()};
  std::vector<uint32_t> ops;
  while (!open.empty()) {
    if (seen.size() > limit) {
      return g;
    }
    StateId s = open.front();
    open.pop_front();
    g.states.push_back(s);
    ops.clear();
    b.space->applicable(s, ops);
    for (uint32_t op : ops) {
      auto n = b.space->successor(s, op);
      if (!n) {
        continue;
      }
      g.next[s].emplace_back(*n, op);
      if (seen.insert(*n).second) {
        open.push_back(*n);
      }
    }
  }
  g.complete = true;
  return g;
}

// Every landmark holds at some point of every path to a goal state, and every
// greedy-necessary ordering a < b holds: whenever b first becomes true, a held just before
::testing::AssertionResult sound(const Bench & b, size_t limit = 20000, bool * whole = nullptr)
{
  Graph g = explore(b, limit);
  if (whole) {
    *whole = g.complete;
  }
  if (!g.complete) {
    return ::testing::AssertionSuccess() << "too big, skipped";
  }
  auto lg = b.landmarks();
  StateId s0 = b.reg->initial();
  std::map<StateId, std::vector<uint32_t>> values;
  for (StateId s : g.states) {
    values[s] = b.values(s);
  }
  auto holds = [&](StateId s, const Landmark & l) {return values[s][l.fact.var] == l.fact.value;};
  for (const auto & l : lg.landmarks()) {
    if (holds(s0, l)) {
      continue;
    }
    // Without the states where it holds, no goal state is reachable
    std::deque<StateId> open{s0};
    std::set<StateId> seen{s0};
    while (!open.empty()) {
      StateId s = open.front();
      open.pop_front();
      if (b.space->is_goal(s)) {
        return ::testing::AssertionFailure() << b.name(l) << " is not a landmark";
      }
      for (const auto & [t, op] : g.next[s]) {
        if (!holds(t, l) && seen.insert(t).second) {
          open.push_back(t);
        }
      }
    }
    // Its first achievement, on any path: every parent holds just before
    open = {s0};
    seen = {s0};
    while (!open.empty()) {
      StateId s = open.front();
      open.pop_front();
      for (const auto & [t, op] : g.next[s]) {
        if (holds(t, l)) {
          // A derived landmark holds with the body of its axiom, in the same state
          bool derived = b.p->sas.variables[l.fact.var].kind == sas::Variable::Kind::Derived;
          for (uint32_t p : l.parents) {
            const auto & pl = lg.landmarks()[p];
            if (!holds(s, pl) && !(derived && holds(t, pl))) {
              return ::testing::AssertionFailure() << b.name(pl) << " < " <<
                     b.name(l) << " is not greedy-necessary";
            }
          }
        } else if (seen.insert(t).second) {
          open.push_back(t);
        }
      }
    }
  }
  return ::testing::AssertionSuccess() << lg.landmarks().size() << " landmarks";
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

TEST(LandmarksPropertiesTest, SmallDomains)
{
  EXPECT_TRUE(sound(*bench(
      kRooms, rooms("(link s0 s1) (link s1 s2) (link s2 s3) (link s1 s0) (link s2 s1)"
      " (link s3 s4) (link s4 s3) (link s0 s4)", "(and (done) (at s0))"))));
  EXPECT_TRUE(sound(*bench(kBlocks,
      "(define (problem p) (:domain blocks) (:objects a b c d)"
      " (:init (clear c) (on c b) (on b a) (ontable a) (clear d) (ontable d) (handempty))"
      " (:goal (and (on a b) (on b c) (on c d))))")));
  // Numeric conditions on the way
  EXPECT_TRUE(sound(*bench(
      "(define (domain n) (:requirements :strips :fluents) (:predicates (a) (b) (g))"
      " (:functions (x))"
      " (:action inc :parameters () :precondition (a) :effect (increase (x) 1))"
      " (:action geta :parameters () :precondition () :effect (a))"
      " (:action fin :parameters () :precondition (and (>= (x) 3) (b)) :effect (g))"
      " (:action getb :parameters () :precondition (a) :effect (b)))",
      "(define (problem p) (:domain n) (:init (= (x) 0)) (:goal (g)))")));
}

TEST(LandmarksPropertiesTest, PlanSys2InstantDomains)
{
  const std::pair<const char *, const char *> pairs[] = {
    {"plansys2_executor__suave_domain", "plansys2_executor__suave_problem"},
    {"plansys2_pddl_parser__dom1", "plansys2_pddl_parser__prob1"},
  };
  for (const auto & [d, p] : pairs) {
    auto b = bench(read_file(data_path(std::string("plansys2/") + d + ".pddl")),
        read_file(data_path(std::string("plansys2/") + p + ".pddl")));
    EXPECT_TRUE(sound(*b)) << d;
  }
}

TEST(LandmarksPropertiesTest, IpcDomains)
{
  const char * dir = std::getenv("KAIROS_PDDL_CORPUS");
  if (dir == nullptr) {
    GTEST_SKIP() << "KAIROS_PDDL_CORPUS not set";
  }
  std::filesystem::path base = std::filesystem::path(dir) / "potassco_pddl-instances";
  const char * domains[] = {
    "ipc-1998/domains/gripper-round-1-strips", "ipc-2000/domains/blocks-strips-typed",
    "ipc-2000/domains/logistics-strips-typed", "ipc-2000/domains/elevator-strips-simple-typed",
    "ipc-2000/domains/elevator-adl-full-typed", "ipc-2002/domains/depots-strips-automatic",
    "ipc-2004/domains/psr-small-strips", "ipc-2002/domains/driverlog-numeric-automatic",
    "ipc-2004/domains/promela-dining-philosophers-derived-predicates-adl",
    "ipc-2008/domains/sokoban-sequential-optimal",
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
    EXPECT_TRUE(sound(*bench(read_file(d), read_file(p)), 50000, &whole)) << dom;
    complete += whole ? 1 : 0;
  }
  EXPECT_GE(complete, 5u);
}

TEST(LandmarksPropertiesTest, IpcNumericDomains)
{
  const char * dir = std::getenv("KAIROS_PDDL_CORPUS");
  if (dir == nullptr) {
    GTEST_SKIP() << "KAIROS_PDDL_CORPUS not set";
  }
  std::filesystem::path base = std::filesystem::path(dir) / "ipc2023-numeric_ipc2023-dataset";
  size_t complete = 0;
  for (const char * dom : {"counters", "fo-counters", "farmland", "drone", "mprime",
      "block-grouping", "tpp", "delivery", "rover", "zenotravel"})
  {
    auto d = base / dom / "domain.pddl";
    auto p = base / dom / "instances" / "pfile1.pddl";
    if (!std::filesystem::exists(p) || !std::filesystem::exists(d)) {
      continue;
    }
    bool whole = false;
    EXPECT_TRUE(sound(*bench(read_file(d), read_file(p)), 100000, &whole)) << dom;
    complete += whole ? 1 : 0;
  }
  EXPECT_GE(complete, 2u);
}

}  // namespace
}  // namespace kairos::heuristics
