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
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <memory>
#include <random>
#include <set>
#include <string>
#include <vector>

#include "ground/grounder.hpp"
#include "sas/invariants.hpp"
#include "support/ground_simulator.hpp"
#include "support/pddl_helpers.hpp"
#include "support/task_helpers.hpp"

namespace kairos::sas
{
namespace
{

using kairos::testing::data_path;
using kairos::testing::dump;
using kairos::testing::normalize_text;
using kairos::testing::read_file;
using ground::Timing;

struct Found
{
  std::unique_ptr<kairos::testing::Normalized> n;
  ground::Task g;
  std::vector<Invariant> invariants;
  std::vector<std::vector<ground::AtomId>> groups;
  std::vector<std::vector<ground::AtomId>> chosen;
  InvariantStats stats;
};

std::unique_ptr<Found> find(
  const std::string & domain, const std::string & problem, const InvariantOptions & o = {},
  bool drop_useless = true)
{
  auto f = std::make_unique<Found>();
  f->n = normalize_text(domain, problem);
  ground::GroundOptions go;
  go.drop_useless_actions = drop_useless;
  auto g = ground::ground(f->n->task, go);
  EXPECT_TRUE(g.ok()) << dump(g.diagnostics());
  if (g.ok()) {
    f->g = std::move(g).value();
  }
  f->invariants = find_invariants(f->n->task, f->g, o, nullptr, &f->stats);
  f->groups = mutex_groups(f->invariants, f->g);
  f->chosen = choose_groups(f->groups, f->g);
  return f;
}

std::set<std::string> texts(const Found & f)
{
  std::set<std::string> out;
  for (const auto & i : f.invariants) {
    out.insert(to_string(f.n->task, i));
  }
  return out;
}

std::set<std::string> group_texts(
  const Found & f, const std::vector<std::vector<ground::AtomId>> & groups)
{
  std::set<std::string> out;
  for (const auto & grp : groups) {
    std::string s;
    for (auto a : grp) {
      s += (s.empty() ? "" : " ") + ground::atom_name(f.g, a);
    }
    out.insert(s);
  }
  return out;
}

const char * kLogistics =
  "(define (domain log) (:requirements :typing) (:types truck package place)"
  " (:predicates (at ?x - (either truck package) ?l - place) (in ?p - package ?t - truck))"
  " (:action load :parameters (?p - package ?t - truck ?l - place)"
  "  :precondition (and (at ?p ?l) (at ?t ?l)) :effect (and (not (at ?p ?l)) (in ?p ?t)))"
  " (:action unload :parameters (?p - package ?t - truck ?l - place)"
  "  :precondition (and (in ?p ?t) (at ?t ?l)) :effect (and (not (in ?p ?t)) (at ?p ?l)))"
  " (:action drive :parameters (?t - truck ?a ?b - place)"
  "  :precondition (at ?t ?a) :effect (and (not (at ?t ?a)) (at ?t ?b))))";

const char * kLogisticsProblem =
  "(define (problem p) (:domain log) (:objects t1 t2 - truck p1 p2 - package a b - place)"
  " (:init (at t1 a) (at t2 b) (at p1 a) (at p2 b)) (:goal (at p1 b)))";

TEST(InvariantsTest, LogisticsPackagesAreAtOneSpotOrInOneTruck)
{
  auto f = find(kLogistics, kLogisticsProblem);
  EXPECT_EQ(texts(*f), (std::set<std::string>{"{at(?v0 *), in(?v0 *)}"}));
  EXPECT_EQ(group_texts(*f, f->chosen), (std::set<std::string>{
        "(at p1 a) (at p1 b) (in p1 t1) (in p1 t2)", "(at p2 a) (at p2 b) (in p2 t1) (in p2 t2)",
        "(at t1 a) (at t1 b)", "(at t2 a) (at t2 b)"}));
}

TEST(InvariantsTest, ActionsAddingTwoAtomsOfAnInstanceAreTooHeavy)
{
  // split puts the robot in two places at once: no invariant about at
  auto f = find(
    "(define (domain d) (:predicates (at ?r ?l) (r ?r) (l ?l))"
    " (:action move :parameters (?r ?a ?b) :precondition (at ?r ?a)"
    "  :effect (and (not (at ?r ?a)) (at ?r ?b)))"
    " (:action split :parameters (?r ?a ?b ?c) :precondition (at ?r ?a)"
    "  :effect (and (not (at ?r ?a)) (at ?r ?b) (at ?r ?c))))",
    "(define (problem p) (:domain d) (:objects x y z w) (:init (at x y)) (:goal (at x z)))");
  for (const auto & t : texts(*f)) {
    EXPECT_EQ(t.find("at(?v0 *)"), std::string::npos) << t;
  }
}

TEST(InvariantsTest, DeletesCancelledByAddsDoNotBalance)
{
  // noop keeps a (delete, then add) and adds c: a and c both true afterwards
  auto f = find(
    "(define (domain d) (:predicates (a) (c) (go))"
    " (:action noop :parameters () :precondition (and (a) (go))"
    "  :effect (and (not (a)) (a) (c)))"
    " (:action back :parameters () :precondition (c) :effect (and (not (c)) (a))))",
    "(define (problem p) (:domain d) (:init (a) (go)) (:goal (c)))");
  for (const auto & t : texts(*f)) {
    bool both = t.find("a()") != std::string::npos && t.find("c()") != std::string::npos;
    EXPECT_FALSE(both) << t;
  }
}

TEST(InvariantsTest, UniversalEffectsAreHeavy)
{
  // reset puts every token on the board: no "one token on the board" invariant
  auto f = find(
    "(define (domain d) (:predicates (on ?t) (off ?t))"
    " (:action take :parameters (?t) :precondition (on ?t) :effect (and (not (on ?t)) (off ?t)))"
    " (:action reset :parameters () :precondition ()"
    "  :effect (forall (?t) (when (off ?t) (and (on ?t) (not (off ?t)))))))",
    "(define (problem p) (:domain d) (:objects a b) (:init (on a) (off b)) (:goal (off a)))");
  EXPECT_FALSE(texts(*f).count("{on(*)}"));
  EXPECT_TRUE(texts(*f).count("{on(?v0), off(?v0)}"));  // each token is on or off
}

TEST(InvariantsTest, InstancesCrowdedAtTheStartGiveNoGroup)
{
  // The invariant holds for the actions, but two atoms are true at the start
  auto f = find(
    "(define (domain d) (:predicates (at ?r ?l))"
    " (:action move :parameters (?r ?a ?b) :precondition (at ?r ?a)"
    "  :effect (and (not (at ?r ?a)) (at ?r ?b))))",
    "(define (problem p) (:domain d) (:objects x y z) (:init (at x y) (at x z)) (:goal (at x x)))");
  EXPECT_TRUE(texts(*f).count("{at(?v0 *)}"));
  EXPECT_TRUE(f->groups.empty());
}

TEST(InvariantsTest, PredicatesAddedByTimedLiteralsAreLeftOut)
{
  auto f = find(
    "(define (domain d) (:requirements :durative-actions :timed-initial-literals)"
    " (:predicates (at ?l) (open ?l))"
    " (:durative-action go :parameters (?a ?b) :duration (= ?duration 1)"
    "  :condition (at start (at ?a)) :effect (and (at start (not (at ?a))) (at end (at ?b))))"
    " (:durative-action shut :parameters (?a) :duration (= ?duration 1)"
    "  :condition (at start (open ?a)) :effect (at end (not (open ?a)))))",
    "(define (problem p) (:domain d) (:objects x y) (:init (at x) (open x) (at 3 (open y)))"
    " (:goal (at y)))");
  auto t = texts(*f);
  EXPECT_TRUE(t.count("{at(*)}"));
  for (const auto & s : t) {
    EXPECT_EQ(s.find("open"), std::string::npos) << s;
  }
}

TEST(InvariantsTest, DurativeMovesReserveTheirStartPlace)
{
  // At start the robot leaves a, at end it arrives at b: between them it is nowhere
  auto f = find(
    "(define (domain d) (:requirements :durative-actions :typing) (:types robot place)"
    " (:predicates (at ?r - robot ?l - place))"
    " (:durative-action move :parameters (?r - robot ?a ?b - place) :duration (= ?duration 2)"
    "  :condition (at start (at ?r ?a))"
    "  :effect (and (at start (not (at ?r ?a))) (at end (at ?r ?b)))))",
    "(define (problem p) (:domain d) (:objects r - robot a b c - place) (:init (at r a))"
    " (:goal (at r c)))");
  EXPECT_TRUE(texts(*f).count("{at(?v0 *)}"));
  EXPECT_EQ(f->chosen.size(), 1u);
}

TEST(InvariantsTest, DurativeEndDeletesNeedAnEndCondition)
{
  // start: leave a, become busy; end: stop being busy, arrive at b. Without (busy) at end,
  // another action could end the busy state first and the end would add a second atom.
  const std::string common =
    "(define (domain d) (:requirements :durative-actions :typing) (:types robot place)"
    " (:predicates (at ?r - robot ?l - place) (busy ?r - robot) (idle ?r - robot))"
    " (:action rest :parameters (?r - robot) :precondition (busy ?r)"
    "  :effect (and (not (busy ?r)) (idle ?r)))"
    " (:durative-action move :parameters (?r - robot ?a ?b - place) :duration (= ?duration 2)"
    "  :condition (and (at start (at ?r ?a)) %s)"
    "  :effect (and (at start (not (at ?r ?a))) (at start (busy ?r))"
    "    (at end (not (busy ?r))) (at end (at ?r ?b)))))";
  const std::string problem =
    "(define (problem p) (:domain d) (:objects r - robot a b - place) (:init (at r a))"
    " (:goal (at r b)))";
  char buf[2048];
  std::snprintf(buf, sizeof(buf), common.c_str(), "");
  auto loose = find(buf, problem);
  for (const auto & t : texts(*loose)) {
    bool both = t.find("at(") != std::string::npos && t.find("busy(") != std::string::npos;
    EXPECT_FALSE(both) << t;
  }
  // With (at end (busy ?r)) the end delete is sure: robot is at one place, busy or idle...
  std::snprintf(buf, sizeof(buf), common.c_str(), "(at end (busy ?r))");
  auto tight = find(buf, problem);
  EXPECT_TRUE(texts(*tight).count("{at(?v0 *), busy(?v0)}"));
}

TEST(InvariantsTest, ChoosingGroupsTakesTheLargestFirstAndSkipsNegatedGoals)
{
  auto f = find(kLogistics,
      "(define (problem p) (:domain log) (:objects t1 - truck p1 - package a b - place)"
      " (:init (at t1 a) (at p1 a)) (:goal (and (at p1 b) (not (at t1 a)))))");
  // (at t1 a) is negated in the goal: it stays out of the groups
  for (const auto & grp : f->chosen) {
    for (auto a : grp) {
      EXPECT_NE(ground::atom_name(f->g, a), "(at t1 a)");
    }
  }
  EXPECT_EQ(group_texts(*f, f->chosen), (std::set<std::string>{
        "(at p1 a) (at p1 b) (in p1 t1)"}));
}

TEST(InvariantsTest, CandidateLimitStopsTheSearch)
{
  InvariantOptions o;
  o.max_candidates = 2;
  auto f = find(kLogistics, kLogisticsProblem, o);
  EXPECT_TRUE(f->stats.limit_reached);
  EXPECT_LE(f->stats.candidates, 2u);
}

// --- Soundness: no reachable state has two atoms of a group true ----------------------------

::testing::AssertionResult groups_hold(
  const std::vector<std::vector<ground::AtomId>> & groups, const std::set<ground::AtomId> & atoms,
  const ground::Task & g, const std::string & trace)
{
  for (const auto & grp : groups) {
    size_t n = 0;
    for (auto a : grp) {
      n += atoms.count(a);
    }
    if (n > 1) {
      std::string s;
      for (auto a : grp) {
        if (atoms.count(a)) {
          s += " " + ground::atom_name(g, a);
        }
      }
      return ::testing::AssertionFailure() << "two atoms of a group true:" << s << " after\n" <<
             trace;
    }
  }
  return ::testing::AssertionSuccess();
}

// Random interleavings: instant actions, starts and ends of durative actions in any order
// (durations ignored: more interleavings than time allows, so a stronger check)
::testing::AssertionResult random_walks(
  const std::string & domain, const std::string & problem, int walks, int steps)
{
  auto f = find(domain, problem, {}, false);
  const ground::Task & g = f->g;
  ground::testing::GroundSimulator sim(g);
  std::mt19937 rng(5);
  for (int w = 0; w < walks; ++w) {
    auto s = sim.initial();
    struct Running
    {
      size_t action;
      ground::testing::GState start;
    };
    std::vector<Running> running;
    std::string trace;
    auto ok = groups_hold(f->groups, s.atoms, g, trace);
    if (!ok) {
      return ok;
    }
    for (int step = 0; step < steps; ++step) {
      std::vector<std::pair<int, size_t>> options;  // 0 instant, 1 start, 2 end
      auto der = sim.derived(s);
      for (size_t i = 0; i < g.actions.size(); ++i) {
        if (sim.holds(g.actions[i].precondition, s, der)) {
          options.emplace_back(0, i);
        }
      }
      for (size_t i = 0; i < g.durative_actions.size(); ++i) {
        if (sim.holds(g.durative_actions[i].at_start, s, der, 1.0)) {
          options.emplace_back(1, i);
        }
      }
      for (size_t i = 0; i < running.size(); ++i) {
        const auto & da = g.durative_actions[running[i].action];
        if (sim.holds(da.over_all, s, der, 1.0) && sim.holds(da.at_end, s, der, 1.0)) {
          options.emplace_back(2, i);
        }
      }
      if (options.empty()) {
        break;
      }
      auto [kind, i] = options[std::uniform_int_distribution<size_t>(0, options.size() - 1)(rng)];
      if (kind == 0) {
        const auto & a = g.actions[i];
        s = sim.apply(a.changes, a.effects, Timing::Instant, s);
        trace += ground::instance_name(g, a) + "\n";
      } else if (kind == 1) {
        const auto & a = g.durative_actions[i];
        running.push_back(Running{i, s});
        s = sim.apply(a.start, a.effects, Timing::Start, s, nullptr, 1.0);
        trace += "start " + ground::instance_name(g, a) + "\n";
      } else {
        const auto & a = g.durative_actions[running[i].action];
        s = sim.apply(a.end, a.effects, Timing::End, s, &running[i].start, 1.0);
        trace += "end " + ground::instance_name(g, a) + "\n";
        running.erase(running.begin() + static_cast<std::ptrdiff_t>(i));
      }
      ok = groups_hold(f->groups, s.atoms, g, trace);
      if (!ok) {
        return ok;
      }
    }
  }
  return ::testing::AssertionSuccess();
}

TEST(InvariantsSoundnessTest, PlanSys2Domains)
{
  const std::pair<const char *, const char *> pairs[] = {
    {"plansys2_domain_expert__domain_charging", "plansys2_domain_expert__problem_charging"},
    {"plansys2_executor__elevator_domain", "plansys2_executor__elevator_problem"},
    {"plansys2_executor__factory2", "plansys2_executor__problem_factory_1"},
    {"plansys2_executor__road_trip_domain", "plansys2_executor__road_trip_problem"},
    {"plansys2_executor__suave_domain", "plansys2_executor__suave_problem"},
    {"plansys2_simple_example__simple_example", "plansys2_simple_example__problem"},
    {"plansys2_terminal__simple_example", "plansys2_terminal__problem_charging"},
    {"plansys2_tests__test_2", "plansys2_tests__problem_2"},
  };
  for (const auto & [d, p] : pairs) {
    std::string dom = read_file(data_path(std::string("plansys2/") + d + ".pddl"));
    std::string prob = read_file(data_path(std::string("plansys2/") + p + ".pddl"));
    EXPECT_TRUE(random_walks(dom, prob, 20, 40)) << d;
  }
}

TEST(InvariantsSoundnessTest, SmallDomains)
{
  EXPECT_TRUE(random_walks(kLogistics, kLogisticsProblem, 20, 40));
}

TEST(InvariantsSoundnessTest, IpcDomains)
{
  const char * dir = std::getenv("KAIROS_PDDL_CORPUS");
  if (dir == nullptr) {
    GTEST_SKIP() << "KAIROS_PDDL_CORPUS not set";
  }
  std::filesystem::path base = std::filesystem::path(dir) / "potassco_pddl-instances";
  const char * domains[] = {
    "ipc-2000/domains/logistics-strips-typed", "ipc-2000/domains/blocks-strips-typed",
    "ipc-2002/domains/depots-strips-automatic", "ipc-2002/domains/rovers-strips-automatic",
    "ipc-2002/domains/zenotravel-time-automatic", "ipc-2002/domains/rovers-time-automatic",
    "ipc-2002/domains/satellite-time-automatic", "ipc-2004/domains/airport-nontemporal-adl",
    "ipc-2004/domains/pipesworld-no-tankage-temporal-strips",
    "ipc-2006/domains/trucks-propositional",
    "ipc-2008/domains/elevator-sequential-satisficing-strips",
    "ipc-2011/domains/parking-temporal-satisficing",
    "ipc-2011/domains/floortile-temporal-satisficing",
  };
  size_t tried = 0;
  for (const char * dom : domains) {
    auto d = base / dom / "domain.pddl";
    auto p = base / dom / "instances" / "instance-1.pddl";
    if (!std::filesystem::exists(d)) {
      d = base / dom / "domains" / "domain-1.pddl";
    }
    if (!std::filesystem::exists(p) || !std::filesystem::exists(d)) {
      continue;
    }
    ++tried;
    EXPECT_TRUE(random_walks(read_file(d), read_file(p), 10, 60)) << dom;
  }
  EXPECT_GT(tried, 0u);
}

}  // namespace
}  // namespace kairos::sas
