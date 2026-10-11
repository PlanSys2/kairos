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
#include <string>
#include <vector>

#include "sas/translate.hpp"
#include "state/eval.hpp"
#include "state/registry.hpp"
#include "state/successors.hpp"
#include "support/pddl_helpers.hpp"
#include "support/pipeline_helpers.hpp"
#include "temporal/compress.hpp"

namespace kairos::temporal
{
namespace
{

using kairos::testing::pipeline;

struct Bench
{
  std::unique_ptr<kairos::testing::Pipeline> p;
  Compressed c;
  std::unique_ptr<state::StateRegistry> reg;
  std::unique_ptr<state::StateSpace> space;

  uint32_t action(const std::string & name) const
  {
    for (uint32_t i = 0; i < c.task.actions.size(); ++i) {
      if (sas::instance_name(c.task, c.task.actions[i]) == name) {
        return i;
      }
    }
    ADD_FAILURE() << "no action " << name;
    return 0;
  }
  // The facts that hold after the actions, by name
  std::map<std::string, bool> after(const std::vector<std::string> & names)
  {
    state::StateId s = reg->initial();
    for (const auto & n : names) {
      auto next = space->successor(s, action(n));
      EXPECT_TRUE(next.has_value()) << n;
      if (!next) {
        break;
      }
      s = *next;
    }
    std::map<std::string, bool> out;
    const auto & v = space->values(s);
    for (sas::VarId var = 0; var < v.size(); ++var) {
      out[sas::fact_name(c.task, sas::Fact{var, v[var]})] = true;
    }
    return out;
  }
  bool applicable(const std::string & name, const std::vector<std::string> & before = {})
  {
    state::StateId s = reg->initial();
    for (const auto & n : before) {
      s = *space->successor(s, action(n));
    }
    std::vector<uint32_t> ops;
    space->applicable(s, ops);
    return std::find(ops.begin(), ops.end(), action(name)) != ops.end();
  }
  std::optional<Plan> plan(const std::vector<std::string> & names)
  {
    std::vector<uint32_t> ids;
    for (const auto & n : names) {
      ids.push_back(action(n));
    }
    return schedule(p->sas, c, *space, ids, 0.001);
  }
};

std::unique_ptr<Bench> bench(const std::string & domain, const std::string & problem)
{
  auto b = std::make_unique<Bench>();
  b->p = pipeline(domain, problem);
  EXPECT_TRUE(b->p->ok);
  b->c = compress(b->p->sas);
  b->reg = std::make_unique<state::StateRegistry>(b->c.task);
  b->space = std::make_unique<state::StateSpace>(b->c.task, *b->reg);
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

TEST(CompressTest, StartThenEndAndTheEndWins)
{
  auto b = bench(kCellar,
      "(define (problem p) (:domain cellar) (:objects m1 - match f1 - fuse)"
      " (:init (unused m1) (handfree)) (:goal (mended f1)))");
  EXPECT_EQ(b->c.instant, 0u);
  ASSERT_EQ(b->c.task.actions.size(), 2u);
  // The light is on only while the match burns: after the compressed action it is off
  auto facts = b->after({"(light-match m1)"});
  EXPECT_FALSE(facts.count("(light m1)"));
  EXPECT_FALSE(facts.count("(unused m1)"));
  // So mending (the light over all) is never possible after it: the compression of a task
  // that needs concurrency loses its plans (the profile keeps M2 away from it)
  EXPECT_FALSE(b->applicable("(mend f1 m1)", {"(light-match m1)"}));
}

TEST(CompressTest, OverAllConditionsTheStartGivesAreNotPreconditions)
{
  const char * domain =
    "(define (domain d) (:requirements :durative-actions) (:predicates (p) (busy) (done))"
    " (:durative-action work :parameters () :duration (= ?duration 2)"
    "  :condition (and (at start (p)) (over all (busy)))"
    "  :effect (and (at start (busy)) (at end (not (busy))) (at end (done))))"
    " (:durative-action broken :parameters () :duration (= ?duration 2)"
    "  :condition (and (at start (p)) (over all (p)))"
    "  :effect (and (at start (not (p))) (at end (done)))))";
  auto b = bench(domain, "(define (problem x) (:domain d) (:init (p)) (:goal (done)))");
  EXPECT_TRUE(b->applicable("(work)"));       // busy comes from its own start
  EXPECT_FALSE(b->applicable("(broken)"));    // its own start breaks its over all condition
  auto facts = b->after({"(work)"});
  EXPECT_TRUE(facts.count("(done)"));
  EXPECT_FALSE(facts.count("(busy)"));
}

TEST(CompressTest, DurationInEffectsAndVariableDurations)
{
  const char * domain =
    "(define (domain d) (:requirements :durative-actions :fluents :duration-inequalities)"
    " (:predicates (done)) (:functions (x) (rate))"
    " (:durative-action fill :parameters () :duration (= ?duration (* 2 (rate)))"
    "  :condition () :effect (at end (increase (x) (* ?duration 3))))"
    " (:durative-action wait :parameters () :duration (and (>= ?duration 4) (<= ?duration 9))"
    "  :condition () :effect (at end (done))))";
  auto b = bench(domain,
      "(define (problem x) (:domain d) (:init (= (x) 0) (= (rate) 1.5)) (:goal (done)))");
  // fill lasts 3 and adds 9 (rate is static: a constant after grounding, x the only fluent)
  state::StateId s = *b->space->successor(b->reg->initial(), b->action("(fill)"));
  ASSERT_EQ(b->space->numbers(s).size(), 1u);
  EXPECT_EQ(b->space->numbers(s)[0], 9.0);
  auto p = b->plan({"(fill)", "(wait)"});
  ASSERT_TRUE(p);
  ASSERT_EQ(p->steps.size(), 2u);
  std::map<std::string, double> duration;
  for (const auto & st : p->steps) {
    duration[st.action()] = st.duration;
  }
  EXPECT_EQ(duration["(fill)"], 3.0);
  EXPECT_EQ(duration["(wait)"], 4.0);  // the lower bound
}

TEST(ScheduleTest, IndependentStepsOverlapAndDependentOnesWait)
{
  const char * domain =
    "(define (domain robots) (:requirements :typing :durative-actions)"
    " (:types robot room)"
    " (:predicates (at ?r - robot ?l - room) (cleaned ?l - room) (charged ?r - robot))"
    " (:durative-action move :parameters (?r - robot ?a ?b - room) :duration (= ?duration 5)"
    "  :condition (at start (at ?r ?a))"
    "  :effect (and (at start (not (at ?r ?a))) (at end (at ?r ?b))))"
    " (:durative-action clean :parameters (?r - robot ?l - room) :duration (= ?duration 3)"
    "  :condition (over all (at ?r ?l)) :effect (at end (cleaned ?l))))";
  auto b = bench(domain,
      "(define (problem p) (:domain robots) (:objects r1 r2 - robot a b c d - room)"
      " (:init (at r1 a) (at r2 c)) (:goal (and (cleaned b) (cleaned d))))");
  auto p = b->plan({"(move r1 a b)", "(clean r1 b)", "(move r2 c d)", "(clean r2 d)"});
  ASSERT_TRUE(p);
  std::map<std::string, double> at;
  for (const auto & st : p->steps) {
    at[st.action()] = st.time;
  }
  EXPECT_EQ(at["(move r1 a b)"], 0.0);
  EXPECT_EQ(at["(move r2 c d)"], 0.0);           // the other robot: at once
  EXPECT_NEAR(at["(clean r1 b)"], 5.001, 1e-9);  // after its move ends
  EXPECT_NEAR(at["(clean r2 d)"], 5.001, 1e-9);
  EXPECT_NEAR(p->makespan(), 8.001, 1e-9);
}

TEST(ScheduleTest, OrdersHappeningsNotWholeActions)
{
  // charge needs what askcharge sets at start: it starts epsilon after, not after it ends
  const char * domain =
    "(define (domain d) (:requirements :durative-actions) (:predicates (here) (there) (full))"
    " (:durative-action ask :parameters () :duration (= ?duration 5)"
    "  :condition (at start (here)) :effect (and (at start (not (here))) (at start (there))))"
    " (:durative-action charge :parameters () :duration (= ?duration 5)"
    "  :condition (at start (there)) :effect (at end (full))))";
  auto b = bench(domain, "(define (problem x) (:domain d) (:init (here)) (:goal (full)))");
  auto p = b->plan({"(ask)", "(charge)"});
  ASSERT_TRUE(p);
  ASSERT_EQ(p->steps.size(), 2u);
  EXPECT_EQ(p->steps[0].action(), "(ask)");
  EXPECT_NEAR(p->steps[1].time, 0.001, 1e-9);
  EXPECT_NEAR(p->makespan(), 5.001, 1e-9);
}

TEST(ScheduleTest, AnEmptyPlan)
{
  auto b = bench(kCellar,
      "(define (problem p) (:domain cellar) (:objects m1 - match f1 - fuse)"
      " (:init (unused m1) (handfree)) (:goal (unused m1)))");
  auto p = b->plan({});
  ASSERT_TRUE(p.has_value());
  EXPECT_TRUE(p->temporal);
  EXPECT_TRUE(p->steps.empty());
}

}  // namespace
}  // namespace kairos::temporal
