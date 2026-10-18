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
#include <deque>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include "state/registry.hpp"
#include "state/successors.hpp"
#include "support/pipeline_helpers.hpp"
#include "temporal/hybrid.hpp"

namespace kairos::temporal
{
namespace
{

using kairos::testing::pipeline;

struct Bench
{
  std::unique_ptr<kairos::testing::Pipeline> p;
  Hybrid h;
  std::unique_ptr<state::StateRegistry> reg;
  std::unique_ptr<state::StateSpace> space;

  bool applicable(state::StateId s, uint32_t a)
  {
    std::vector<uint32_t> ops;
    space->applicable(s, ops);
    return std::find(ops.begin(), ops.end(), a) != ops.end();
  }
  // Takes a transition if applicable, possible and accepted
  bool take(state::StateId & s, uint32_t a)
  {
    if (!applicable(s, a)) {
      return false;
    }
    auto c = space->successor(s, a);
    if (!c || !h.accept(space->values(*c).data(), space->numbers(*c).data())) {
      return false;
    }
    s = *c;
    return true;
  }
  // Every state reachable (up to `limit`)
  size_t states(size_t limit)
  {
    std::deque<state::StateId> open{reg->initial()};
    std::set<state::StateId> seen{reg->initial()};
    std::vector<uint32_t> ops;
    while (!open.empty() && seen.size() < limit) {
      auto s = open.front();
      open.pop_front();
      ops.clear();
      space->applicable(s, ops);
      for (uint32_t a : ops) {
        auto c = space->successor(s, a);
        if (c && h.accept(space->values(*c).data(), space->numbers(*c).data()) &&
          seen.insert(*c).second)
        {
          open.push_back(*c);
        }
      }
    }
    return seen.size();
  }
  double time(state::StateId s) {return space->numbers(s)[*h.time];}
};

std::unique_ptr<Bench> bench(const std::string & domain, const std::string & problem)
{
  auto b = std::make_unique<Bench>();
  b->p = pipeline(domain, problem);
  EXPECT_TRUE(b->p->ok);
  b->h = hybrid(b->p->sas, 0.1, Waits::Both, 0.001);
  EXPECT_TRUE(b->h.unsupported.empty()) << b->h.unsupported;
  b->reg = std::make_unique<state::StateRegistry>(b->h.task);
  b->space = std::make_unique<state::StateSpace>(b->h.task, *b->reg);
  return b;
}

// PlanSys2's test bank: a door that closes at 3 while passing takes 4
const char * kDoors =
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

std::string doors_problem(const std::string & literals)
{
  return "(define (problem p) (:domain closing_doors)"
         " (:objects r1 - robot corridor office - room office_door - door)"
         " (:init (robot_at r1 corridor) (door_open office_door)"
         "  (door_between office_door corridor office) " + literals + ")"
         " (:goal (robot_at r1 office)))";
}

// And a charge from 1 to 8 s that gives 10 per second, while three moves need 90
const char * kRecharge =
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
  "   (at end (increase (battery ?r) (* 10 ?duration))))))";

const char * kRechargeProblem =
  "(define (problem p) (:domain recharge)"
  " (:objects r1 - robot dock hall office lab - room c1 - charger)"
  " (:init (robot_at r1 dock) (= (battery r1) 0) (charger_at c1 dock) (charger_free c1)"
  "  (connected dock hall) (connected hall office) (connected office lab))"
  " (:goal (robot_at r1 lab)))";

TEST(HybridTest, TheTimeOnlyWithTimedLiterals)
{
  auto none = bench(kRecharge, kRechargeProblem);
  EXPECT_FALSE(none->h.time);  // durative actions alone: their clocks are enough
  auto with = bench(kDoors, doors_problem("(at 3 (not (door_open office_door)))"));
  EXPECT_TRUE(with->h.time);
}

TEST(HybridTest, TheTimeStopsAtTheLastLiteral)
{
  auto b = bench(kDoors, doors_problem(
      "(at 0.5 (not (door_open office_door))) (at 1 (door_open office_door))"));
  ASSERT_TRUE(b->h.time);
  uint32_t first = static_cast<uint32_t>(b->h.task.actions.size() - 2);
  state::StateId s = b->reg->initial();
  // Up to the first literal: waits stop there, and only it lets time go on
  for (int i = 0; i < 5; ++i) {
    ASSERT_TRUE(b->take(s, b->h.wait));
  }
  EXPECT_NEAR(b->time(s), 0.5, 1e-9);
  EXPECT_FALSE(b->take(s, b->h.wait));
  ASSERT_TRUE(b->take(s, first));
  for (int i = 0; i < 5; ++i) {
    ASSERT_TRUE(b->take(s, b->h.wait));
  }
  EXPECT_NEAR(b->time(s), 1.0, 1e-9);
  ASSERT_TRUE(b->take(s, first + 1));
  // After the last one the time stays: waiting leads back to the same state (the first wait
  // only marks that no happening is right before)
  ASSERT_TRUE(b->take(s, b->h.wait));
  state::StateId after = s;
  ASSERT_TRUE(b->take(s, b->h.wait));
  EXPECT_NEAR(b->time(s), 1.0, 1e-9);
  EXPECT_EQ(s, after);
  // The plan still counts the waits while something runs: pass_door from 1 to 5
  ASSERT_TRUE(b->take(s, b->h.first_start));
  std::vector<uint32_t> path(5, b->h.wait);
  path.push_back(first);
  path.insert(path.end(), 5, b->h.wait);
  path.push_back(first + 1);
  path.push_back(b->h.first_start);
  for (int i = 0; i < 40; ++i) {
    ASSERT_TRUE(b->take(s, b->h.wait)) << i;
    path.push_back(b->h.wait);
  }
  EXPECT_FALSE(b->take(s, b->h.wait));  // it must end at 4
  ASSERT_TRUE(b->take(s, b->h.first_start + 1));
  path.push_back(b->h.first_start + 1);
  EXPECT_TRUE(b->space->is_goal(s));
  Plan plan = b->h.plan(path, *b->space);
  ASSERT_EQ(plan.steps.size(), 1u);
  EXPECT_NEAR(plan.steps[0].time, 1.0, 1e-9);
  EXPECT_NEAR(plan.steps[0].duration, 4.0, 1e-9);
}

TEST(HybridTest, EpsilonWaitsOnlyRightAfterAHappening)
{
  auto b = bench(kRecharge, kRechargeProblem);
  ASSERT_NE(b->h.wait_epsilon, UINT32_MAX);
  state::StateId s = b->reg->initial();
  EXPECT_FALSE(b->applicable(s, b->h.wait_epsilon));  // nothing happened yet
  // The start of charge (the only start applicable at first)
  std::vector<uint32_t> ops;
  b->space->applicable(s, ops);
  auto start = std::find_if(ops.begin(), ops.end(), [&](uint32_t a) {
        return a >= b->h.first_start && a < b->h.first_start + 2 * b->h.durative;
      });
  ASSERT_NE(start, ops.end());
  ASSERT_TRUE(b->take(s, *start));
  ASSERT_TRUE(b->take(s, b->h.wait_epsilon));
  EXPECT_FALSE(b->applicable(s, b->h.wait_epsilon));  // not twice in a row
  ASSERT_TRUE(b->take(s, b->h.wait));
  EXPECT_FALSE(b->applicable(s, b->h.wait_epsilon));  // nor after a wait
  ASSERT_TRUE(b->take(s, b->h.wait_long));
  EXPECT_FALSE(b->applicable(s, b->h.wait_epsilon));
}

TEST(HybridTest, SmallTasksHaveSmallSpaces)
{
  // Millions of states before (the time grew for ever, chains of epsilons made a grid)
  auto door = bench(kDoors, doors_problem("(at 3 (not (door_open office_door)))"));
  EXPECT_LT(door->states(100000), 5000u);
  auto recharge = bench(kRecharge, kRechargeProblem);
  EXPECT_LT(recharge->states(100000), 20000u);
}

}  // namespace
}  // namespace kairos::temporal
