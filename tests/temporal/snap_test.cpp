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

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "sas/translate.hpp"
#include "state/registry.hpp"
#include "state/successors.hpp"
#include "support/pddl_helpers.hpp"
#include "support/pipeline_helpers.hpp"
#include "temporal/snap.hpp"

namespace kairos::temporal
{
namespace
{

using kairos::testing::pipeline;

struct Bench
{
  std::unique_ptr<kairos::testing::Pipeline> p;
  SnapTask s;
  std::unique_ptr<state::StateRegistry> reg;
  std::unique_ptr<state::StateSpace> space;
  std::unique_ptr<TemporalFilter> filter;
  state::StateId current = state::kNoState;
  std::vector<uint32_t> taken;

  // "start (a x)", "end (a x)", "til 2" or "(instant x)"
  uint32_t action(const std::string & what) const
  {
    if (what.rfind("til ", 0) == 0) {
      return static_cast<uint32_t>(s.tils + std::stoul(what.substr(4)) - 1);
    }
    bool start = what.rfind("start ", 0) == 0;
    bool end = what.rfind("end ", 0) == 0;
    std::string name = start ? what.substr(6) : (end ? what.substr(4) : what);
    for (uint32_t a = 0; a < s.task.actions.size(); ++a) {
      if (a >= s.tils) {
        break;
      }
      bool kind = start ? s.is_start(a) : (end ? s.is_end(a) : a < s.instant);
      if (kind && sas::instance_name(s.task, s.task.actions[a]) == name) {
        return a;
      }
    }
    ADD_FAILURE() << "no action " << what;
    return 0;
  }
  // Takes the step if the filter accepts it
  bool take(const std::string & what)
  {
    uint32_t a = action(what);
    std::vector<uint32_t> ops;
    space->applicable(current, ops);
    if (std::find(ops.begin(), ops.end(), a) == ops.end()) {
      return false;
    }
    auto next = space->successor(current, a);
    if (!next || !filter->accept(current, a, *next)) {
      return false;
    }
    current = *next;
    taken.push_back(a);
    return true;
  }
  bool goal() {return space->is_goal(current);}
};

std::unique_ptr<Bench> bench(const std::string & domain, const std::string & problem)
{
  auto b = std::make_unique<Bench>();
  b->p = pipeline(domain, problem);
  EXPECT_TRUE(b->p->ok);
  b->s = snap(b->p->sas);
  b->reg = std::make_unique<state::StateRegistry>(b->s.task);
  b->space = std::make_unique<state::StateSpace>(b->s.task, *b->reg);
  b->filter = std::make_unique<TemporalFilter>(b->p->sas, b->s, *b->space, 0.001);
  b->current = b->reg->initial();
  return b;
}

const char * kCellar =
  "(define (domain cellar) (:requirements :typing :durative-actions)"
  " (:types match fuse)"
  " (:predicates (unused ?m - match) (light ?m - match) (mended ?f - fuse) (handfree))"
  " (:durative-action light-match :parameters (?m - match) :duration (= ?duration 8)"
  "  :condition (at start (unused ?m))"
  "  :effect (and (at start (not (unused ?m))) (at start (light ?m)) (at end (not (light ?m)))))"
  " (:durative-action mend :parameters (?f - fuse ?m - match) :duration (= ?duration 5)"
  "  :condition (and (at start (handfree)) (over all (light ?m)))"
  "  :effect (and (at start (not (handfree))) (at end (handfree)) (at end (mended ?f)))))";

const char * kCellarProblem =
  "(define (problem p) (:domain cellar) (:objects m1 - match f1 - fuse)"
  " (:init (unused m1) (handfree)) (:goal (mended f1)))";

TEST(SnapTest, StartsEndsAndRunningVariables)
{
  auto b = bench(kCellar, kCellarProblem);
  EXPECT_EQ(b->s.instant, 0u);
  EXPECT_EQ(b->s.tils, 4u);  // two durative actions, a start and an end each
  ASSERT_EQ(b->s.running.size(), 2u);
  EXPECT_TRUE(b->s.is_start(0));
  EXPECT_TRUE(b->s.is_end(1));
  EXPECT_EQ(b->s.durative(3), 1u);
  EXPECT_TRUE(b->s.unsupported.empty());
  // Nothing runs at the start, and the goal asks nothing running
  for (auto run : b->s.running) {
    EXPECT_EQ(b->s.task.initial[run], 1u);
    EXPECT_NE(
      std::find(b->s.task.goal.facts.begin(), b->s.task.goal.facts.end(), sas::Fact{run, 1}),
      b->s.task.goal.facts.end());
  }
}

TEST(SnapTest, RequiredConcurrencyAndItsPlan)
{
  auto b = bench(kCellar, kCellarProblem);
  // Mending first: no light, the over all condition fails right after the start
  EXPECT_FALSE(b->take("start (mend f1 m1)"));
  ASSERT_TRUE(b->take("start (light-match m1)"));
  ASSERT_TRUE(b->take("start (mend f1 m1)"));
  // The match cannot go out while mending
  EXPECT_FALSE(b->take("end (light-match m1)"));
  ASSERT_TRUE(b->take("end (mend f1 m1)"));
  ASSERT_TRUE(b->take("end (light-match m1)"));
  EXPECT_TRUE(b->goal());
  auto plan = b->filter->plan(b->taken);
  ASSERT_TRUE(plan);
  EXPECT_TRUE(plan->temporal);
  ASSERT_EQ(plan->steps.size(), 2u);
  EXPECT_EQ(plan->steps[0].action(), "(light-match m1)");
  EXPECT_EQ(plan->steps[0].time, 0.0);
  EXPECT_EQ(plan->steps[0].duration, 8.0);
  // The light is needed over all (the open interval): mending may start at the same instant
  EXPECT_EQ(plan->steps[1].time, 0.0);
  EXPECT_EQ(plan->steps[1].duration, 5.0);
}

TEST(SnapTest, ANestedActionThatDoesNotFit)
{
  // Mending takes 10, the match burns 8: never possible inside it
  std::string domain = kCellar;
  domain.replace(domain.find("(= ?duration 5)"), 15, "(= ?duration 10)");
  auto b = bench(domain, kCellarProblem);
  ASSERT_TRUE(b->take("start (light-match m1)"));
  // Its end is in the network from the start on: mending (10) must end before the match goes
  // out (8 after it was lit), since that breaks its over all condition. Rejected at once, not
  // after the search has built on it
  EXPECT_FALSE(b->take("start (mend f1 m1)"));
}

TEST(SnapTest, TwoMendsFitInOneMatchAndAThirdDoesNot)
{
  // A match burns 5, mending takes 2 and needs the hand: two fit, one after the other
  const char * domain =
    "(define (domain cellar) (:requirements :typing :durative-actions)"
    " (:types match fuse)"
    " (:predicates (unused ?m - match) (light ?m - match) (mended ?f - fuse) (handfree))"
    " (:durative-action light-match :parameters (?m - match) :duration (= ?duration 5)"
    "  :condition (at start (unused ?m))"
    "  :effect (and (at start (not (unused ?m))) (at start (light ?m))"
    "   (at end (not (light ?m)))))"
    " (:durative-action mend :parameters (?f - fuse ?m - match) :duration (= ?duration 2)"
    "  :condition (and (at start (handfree)) (over all (light ?m)))"
    "  :effect (and (at start (not (handfree))) (at end (handfree)) (at end (mended ?f)))))";
  auto b = bench(domain,
      "(define (problem p) (:domain cellar) (:objects m1 - match f1 f2 f3 - fuse)"
      " (:init (unused m1) (handfree)) (:goal (and (mended f1) (mended f2) (mended f3))))");
  ASSERT_TRUE(b->take("start (light-match m1)"));
  ASSERT_TRUE(b->take("start (mend f1 m1)"));
  ASSERT_TRUE(b->take("end (mend f1 m1)"));
  ASSERT_TRUE(b->take("start (mend f2 m1)"));
  ASSERT_TRUE(b->take("end (mend f2 m1)"));
  EXPECT_FALSE(b->take("start (mend f3 m1)"));  // 0.001 + 2 + 0.001 + 2 + 0.001 + 2 > 5
}

TEST(SnapTest, VariableDurations)
{
  const char * domain =
    "(define (domain d) (:requirements :durative-actions :duration-inequalities)"
    " (:predicates (p) (q) (done))"
    " (:durative-action wait :parameters () :duration (and (>= ?duration 2) (<= ?duration 6))"
    "  :condition (at start (p)) :effect (and (at start (q)) (at end (not (q))) (at end (done))))"
    " (:durative-action use :parameters () :duration (= ?duration 4)"
    "  :condition (over all (q)) :effect (at end (p))))";
  auto b = bench(domain, "(define (problem x) (:domain d) (:init (p)) (:goal (done)))");
  ASSERT_TRUE(b->take("start (wait)"));
  ASSERT_TRUE(b->take("start (use)"));
  ASSERT_TRUE(b->take("end (use)"));
  ASSERT_TRUE(b->take("end (wait)"));
  auto plan = b->filter->plan(b->taken);
  ASSERT_TRUE(plan);
  std::map<std::string, std::pair<double, double>> at;
  for (const auto & st : plan->steps) {
    at[st.action()] = {st.time, st.duration};
  }
  // use needs q over all, from wait's start to its end: both start at 0, and wait stretches to
  // end with it (4, above its minimum 2)
  EXPECT_EQ(at["(wait)"].first, 0.0);
  EXPECT_NEAR(at["(wait)"].second, 4.0, 1e-9);
  EXPECT_EQ(at["(use)"].first, 0.0);
  EXPECT_EQ(at["(use)"].second, 4.0);
}

const char * kWindow =
  "(define (domain w) (:requirements :durative-actions :timed-initial-literals)"
  " (:predicates (open) (done))"
  " (:durative-action work :parameters () :duration (= ?duration 4)"
  "  :condition (over all (open)) :effect (at end (done))))";

TEST(SnapTest, TimedInitialLiteralsOpenAndCloseWindows)
{
  // Open from 3 to 10: work fits (4 long)
  auto b = bench(kWindow,
      "(define (problem x) (:domain w) (:init (at 3 (open)) (at 10 (not (open))))"
      " (:goal (done)))");
  ASSERT_EQ(b->s.til_time.size(), 2u);
  EXPECT_EQ(b->s.til_time[0], 3.0);
  EXPECT_FALSE(b->take("start (work)"));  // not open yet
  EXPECT_FALSE(b->take("til 2"));         // in order
  ASSERT_TRUE(b->take("til 1"));
  ASSERT_TRUE(b->take("start (work)"));
  ASSERT_TRUE(b->take("end (work)"));
  ASSERT_TRUE(b->goal());
  auto plan = b->filter->plan(b->taken);
  ASSERT_TRUE(plan);
  ASSERT_EQ(plan->steps.size(), 1u);  // the timed literals are not part of the plan
  EXPECT_NEAR(plan->steps[0].time, 3.0, 1e-9);  // open over all: when the window opens

  // Open from 3 to 6: no room for 4
  auto c = bench(kWindow,
      "(define (problem x) (:domain w) (:init (at 3 (open)) (at 6 (not (open))))"
      " (:goal (done)))");
  ASSERT_TRUE(c->take("til 1"));
  // Its end (4 after the start) would come after the window closes at 6, which breaks its over
  // all condition: rejected when it starts
  EXPECT_FALSE(c->take("start (work)"));
}

TEST(SnapTest, WhatM3CannotPlanYet)
{
  const char * domain =
    "(define (domain d) (:requirements :durative-actions :fluents :duration-inequalities)"
    " (:predicates (done)) (:functions (x))"
    " (:durative-action go :parameters () :duration (and (>= ?duration 1) (<= ?duration 3))"
    "  :condition () :effect (and (at end (increase (x) ?duration)) (at end (done)))))";
  auto b = bench(domain, "(define (problem p) (:domain d) (:init (= (x) 0)) (:goal (done)))");
  EXPECT_FALSE(b->s.unsupported.empty());
}

}  // namespace
}  // namespace kairos::temporal
