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
#include <cmath>
#include <deque>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include "kairos/plan.hpp"
#include "sas/translate.hpp"
#include "state/registry.hpp"
#include "state/successors.hpp"
#include "support/pipeline_helpers.hpp"
#include "temporal/untimed.hpp"
#include "validate/validator.hpp"

namespace kairos::temporal
{
namespace
{

using kairos::testing::pipeline;

struct Bench
{
  std::unique_ptr<kairos::testing::Pipeline> p;
  Untimed u;
  std::unique_ptr<state::StateRegistry> reg;
  std::unique_ptr<state::StateSpace> space;

  // Snap actions: the instant ones, a start and an end per durative action, the literals
  uint32_t start(size_t i) const {return static_cast<uint32_t>(p->sas.actions.size() + 2 * i);}
  uint32_t til(size_t j) const
  {
    return static_cast<uint32_t>(p->sas.actions.size() + 2 * p->sas.durative_actions.size() + j);
  }

  // Whether the goal is reachable, every transition checked as the search does
  bool reachable(size_t * states = nullptr)
  {
    std::deque<state::StateId> open{reg->initial()};
    std::set<state::StateId> seen{reg->initial()};
    std::vector<uint32_t> ops;
    bool found = false;
    while (!open.empty() && !found) {
      auto s = open.front();
      open.pop_front();
      found = space->is_goal(s);
      ops.clear();
      space->applicable(s, ops);
      for (uint32_t a : ops) {
        auto c = space->successor(s, a);
        if (c && u.accept(a, space->values(*c).data(), space->numbers(*c).data()) &&
          seen.insert(*c).second)
        {
          open.push_back(*c);
        }
      }
    }
    if (states) {
      *states = seen.size();
    }
    return found;
  }

  // Takes a transition if applicable and accepted
  bool take(state::StateId & s, uint32_t a)
  {
    std::vector<uint32_t> ops;
    space->applicable(s, ops);
    if (std::find(ops.begin(), ops.end(), a) == ops.end()) {
      return false;
    }
    auto c = space->successor(s, a);
    if (!c || !u.accept(a, space->values(*c).data(), space->numbers(*c).data())) {
      return false;
    }
    s = *c;
    return true;
  }

  // A valid plan, instant by instant (its ends, literals, then starts), with an advance
  // between instants: it must reach the goal (the abstraction is a relaxation)
  void replays(const std::string & plan_text)
  {
    auto parsed = parse_plan(plan_text);
    ASSERT_TRUE(parsed.ok());
    const Plan & plan = *parsed;
    ValidateOptions vo;
    auto report = val::validate(p->n->domain, p->n->dm, p->n->problem, p->n->pm, plan, vo);
    ASSERT_TRUE(report.valid()) << (report.diagnostics.empty() ? "" :
    to_string(report.diagnostics[0]));
    // (time, order within the instant: 0 end, 1 literal, 2 start; action)
    std::vector<std::tuple<double, int, uint32_t>> happenings;
    double last = 0.0;
    for (const auto & st : plan.steps) {
      size_t i = 0;
      while (i < p->sas.durative_actions.size() &&
        sas::instance_name(p->sas, p->sas.durative_actions[i]) != st.action())
      {
        ++i;
      }
      ASSERT_LT(i, p->sas.durative_actions.size()) << st.action();
      happenings.emplace_back(st.time, 2, start(i));
      happenings.emplace_back(st.time + st.duration, 0, start(i) + 1);
      last = std::max(last, st.time + st.duration);
    }
    for (size_t j = 0; j < p->sas.timed_events.size(); ++j) {
      if (p->sas.timed_events[j].time <= last + 1e-9) {
        happenings.emplace_back(p->sas.timed_events[j].time, 1, til(j));
      }
    }
    std::sort(happenings.begin(), happenings.end());
    state::StateId s = reg->initial();
    double now = 0.0;
    for (const auto & [t, order, a] : happenings) {
      if (t > now + 1e-9) {
        ASSERT_TRUE(take(s, u.advance)) << "advance to " << t;
        now = t;
      }
      ASSERT_TRUE(take(s, a)) << u.task.action_names[u.task.actions[a].schema] << " at " << t;
    }
    EXPECT_TRUE(space->is_goal(s));
  }
};

std::unique_ptr<Bench> bench(const std::string & domain, const std::string & problem)
{
  auto b = std::make_unique<Bench>();
  b->p = pipeline(domain, problem);
  EXPECT_TRUE(b->p->ok);
  b->u = untimed(b->p->sas);
  EXPECT_TRUE(b->u.unsupported.empty()) << b->u.unsupported;
  b->reg = std::make_unique<state::StateRegistry>(b->u.task);
  b->space = std::make_unique<state::StateSpace>(b->u.task, *b->reg);
  return b;
}

// A door that stays open only while someone holds it, from the room where they stand
const char * kDoor =
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

std::string door_problem(const std::string & robots, const std::string & at)
{
  return "(define (problem p) (:domain door_holding)"
         " (:objects " + robots + " - robot corridor office - room office_door - door)"
         " (:init " + at + " (door_between office_door corridor office)"
         "  (door_between office_door office corridor))"
         " (:goal (robot_at walker office)))";
}

TEST(UntimedTest, ALoneRobotCannotHoldTheDoorAndPass)
{
  // Holding needs it in the corridor over all, passing takes it out at the start
  auto b = bench(kDoor, door_problem("walker", "(robot_at walker corridor)"));
  size_t states = 0;
  EXPECT_FALSE(b->reachable(&states));
  EXPECT_LT(states, 100u);
}

TEST(UntimedTest, ATeammateHoldsTheDoor)
{
  auto b = bench(kDoor, door_problem("walker holder",
      "(robot_at walker corridor) (robot_at holder corridor)"));
  EXPECT_TRUE(b->reachable());
  // Both start at 0: the door is open after that instant, which is all passing needs
  b->replays(
    "0.000: (hold_door holder office_door corridor office) [10.000]\n"
    "0.000: (pass_door walker office_door corridor office) [4.000]\n");
}

TEST(UntimedTest, AnActionCannotEndInTheInstantItBreaksItsOwnOverAll)
{
  // Its start takes away what it needs over all: no plan, though start and end at once would
  // reach the goal
  const char * domain =
    "(define (domain d) (:requirements :durative-actions)"
    " (:predicates (p) (done))"
    " (:durative-action a :parameters () :duration (= ?duration 3)"
    "  :condition (over all (p)) :effect (and (at start (not (p))) (at end (done)))))";
  auto b = bench(domain, "(define (problem x) (:domain d) (:init (p)) (:goal (done)))");
  EXPECT_FALSE(b->reachable());
  // A duration that may be 0 (it reads a fluent that changes): ending at once is left in (a
  // relaxation)
  const char * zero =
    "(define (domain d) (:requirements :durative-actions :numeric-fluents)"
    " (:predicates (p) (done)) (:functions (len))"
    " (:action shorten :parameters () :precondition () :effect (decrease (len) 1))"
    " (:durative-action a :parameters () :duration (= ?duration (len))"
    "  :condition (over all (p)) :effect (and (at start (not (p))) (at end (done)))))";
  auto z = bench(zero,
      "(define (problem x) (:domain d) (:init (p) (= (len) 3)) (:goal (done)))");
  EXPECT_TRUE(z->reachable());
}

TEST(UntimedTest, RequiredConcurrencyAndOverAllConditionsInTheOpenInterval)
{
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
  auto c = bench(cellar,
      "(define (problem p) (:domain cellar) (:objects m1 - match f1 - fuse)"
      " (:init (unused m1) (handfree)) (:goal (mended f1)))");
  EXPECT_TRUE(c->reachable());
  c->replays("0.000: (light-match m1) [8.000]\n0.001: (mend f1 m1) [5.000]\n");
  // A window compiled as in airport: work ends with the wrapper that takes its over all away
  // at that instant, and starts when idle ends
  const char * window =
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
  auto w = bench(window, "(define (problem x) (:domain w) (:init (go)) (:goal (done)))");
  w->replays("0.000: (wrapper) [10.000]\n0.000: (idle) [4.000]\n4.000: (work) [6.000]\n");
}

TEST(UntimedTest, TimedLiteralsInTheirOrderAndNumericOverAllConditions)
{
  // Power from 1 to 7: charge (2) while it lasts, then work on the energy
  const char * domain =
    "(define (domain charger) (:requirements :typing :durative-actions :numeric-fluents"
    " :timed-initial-literals) (:types robot)"
    " (:predicates (done ?r - robot))"
    " (:functions (energy ?r - robot) (power))"
    " (:durative-action charge :parameters (?r - robot) :duration (= ?duration 2)"
    "  :condition (over all (>= (power) 5))"
    "  :effect (at end (increase (energy ?r) 10)))"
    " (:durative-action work :parameters (?r - robot) :duration (= ?duration 3)"
    "  :condition (at start (>= (energy ?r) 5))"
    "  :effect (and (at start (decrease (energy ?r) 5)) (at end (done ?r)))))";
  auto b = bench(domain,
      "(define (problem p) (:domain charger) (:objects r - robot)"
      " (:init (= (energy r) 0) (= (power) 0) (at 1 (= (power) 8)) (at 7 (= (power) 0)))"
      " (:goal (done r)))");
  EXPECT_TRUE(b->reachable());
  b->replays("1.000: (charge r) [2.000]\n3.001: (work r) [3.000]\n");
  // Charging before the power comes breaks its over all at the first advance
  state::StateId s = b->reg->initial();
  ASSERT_TRUE(b->take(s, b->start(0)));
  EXPECT_FALSE(b->take(s, b->u.advance));
  // The second literal only after the first
  state::StateId t = b->reg->initial();
  EXPECT_FALSE(b->take(t, b->til(1)));
  EXPECT_TRUE(b->take(t, b->til(0)));
  EXPECT_TRUE(b->take(t, b->til(1)));
  // Without power ever: no plan
  auto never = bench(domain,
      "(define (problem p) (:domain charger) (:objects r - robot)"
      " (:init (= (energy r) 0) (= (power) 0)) (:goal (done r)))");
  EXPECT_FALSE(never->reachable());
}

TEST(UntimedTest, WhatItIsNoRelaxationOf)
{
  auto unsupported = [](const std::string & domain, const std::string & problem) {
      auto p = pipeline(domain, problem);
      EXPECT_TRUE(p->ok);
      return untimed(p->sas).unsupported;
    };
  // ?duration of a duration that is not fixed (the charge of PlanSys2's test bank)
  EXPECT_NE(unsupported(
      "(define (domain d) (:requirements :durative-actions :fluents :duration-inequalities)"
      " (:predicates (done)) (:functions (x))"
      " (:durative-action go :parameters () :duration (and (>= ?duration 1) (<= ?duration 3))"
      "  :condition () :effect (and (at end (increase (x) (* 10 ?duration))) (at end (done)))))",
      "(define (problem p) (:domain d) (:init (= (x) 0)) (:goal (done)))"), "");
  // Continuous effects
  EXPECT_NE(unsupported(
      "(define (domain d) (:requirements :durative-actions :fluents :continuous-effects)"
      " (:predicates (done)) (:functions (x))"
      " (:durative-action go :parameters () :duration (= ?duration 2)"
      "  :condition () :effect (and (increase (x) (* #t 1)) (at end (done)))))",
      "(define (problem p) (:domain d) (:init (= (x) 0)) (:goal (done)))"), "");
  // Processes
  EXPECT_NE(unsupported(
      "(define (domain d) (:requirements :time :fluents)"
      " (:predicates (on) (done)) (:functions (x))"
      " (:action start :parameters () :precondition () :effect (on))"
      " (:process grow :parameters () :precondition (on) :effect (increase (x) (* #t 1)))"
      " (:action finish :parameters () :precondition (>= (x) 2) :effect (done)))",
      "(define (problem p) (:domain d) (:init (= (x) 0)) (:goal (done)))"), "");
  // A duration inequality that no effect reads is fine: durations are left out anyway
  EXPECT_EQ(unsupported(
      "(define (domain d) (:requirements :durative-actions :duration-inequalities)"
      " (:predicates (done))"
      " (:durative-action go :parameters () :duration (and (>= ?duration 1) (<= ?duration 3))"
      "  :condition () :effect (at end (done))))",
      "(define (problem p) (:domain d) (:init) (:goal (done)))"), "");
}

}  // namespace
}  // namespace kairos::temporal
