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
#include <memory>
#include <string>
#include <vector>

#include "sas/translate.hpp"
#include "state/eval.hpp"
#include "state/registry.hpp"
#include "state/successors.hpp"
#include "support/pipeline_helpers.hpp"
#include "temporal/compress.hpp"
#include "temporal/durations.hpp"
#include "temporal/snap.hpp"

namespace kairos::temporal
{
namespace
{

using kairos::testing::pipeline;

// A robot whose battery drains while it moves and charges while it recharges (any time from
// 1 s up to full); `extra` adds actions, `move_reads` replaces the condition of move
std::string domain(
  const std::string & extra = "",
  const std::string & move_reads = "(over all (>= (battery) 0))")
{
  return
    "(define (domain bat) (:requirements :typing :fluents :durative-actions"
    " :duration-inequalities :continuous-effects)"
    " (:types place) (:predicates (at ?p - place) (done))"
    " (:functions (battery) (dist ?a ?b - place))"
    " (:durative-action move :parameters (?a ?b - place)"
    "  :duration (= ?duration (dist ?a ?b))"
    "  :condition (and (at start (at ?a)) " + move_reads + ")"
    "  :effect (and (at start (not (at ?a))) (at end (at ?b))"
    "   (decrease (battery) (* #t 1))))"
    " (:durative-action recharge :parameters (?p - place)"
    "  :duration (and (>= ?duration 1) (<= ?duration (/ (- 100 (battery)) 10)))"
    "  :condition (over all (at ?p))"
    "  :effect (increase (battery) (* #t 10)))" + extra + ")";
}

const char * kProblem =
  "(define (problem p) (:domain bat) (:objects a b - place)"
  " (:init (at a) (= (battery) 20) (= (dist a b) 30) (= (dist b a) 30)) (:goal (at b)))";

FixedDurations fix(const std::string & d, const std::string & problem = kProblem)
{
  auto p = pipeline(d, problem);
  EXPECT_TRUE(p->ok);
  return fix_durations(p->sas);
}

std::vector<const sas::DurativeAction *> named(const FixedDurations & f, const std::string & n)
{
  std::vector<const sas::DurativeAction *> out;
  for (const auto & da : f.task.durative_actions) {
    if (f.task.durative_action_names[da.schema] == n) {
      out.push_back(&da);
    }
  }
  return out;
}

const sas::Effect * numeric(const sas::DurativeAction & da)
{
  for (const auto & e : da.effects) {
    if (e.kind == sas::Effect::Kind::Numeric) {
      return &e;
    }
  }
  return nullptr;
}

TEST(DurationsTest, NothingToFix)
{
  // No continuous effect and no ?duration in the effects: the task as it was
  const char * plain =
    "(define (domain d) (:requirements :durative-actions) (:predicates (p) (q))"
    " (:durative-action a :parameters () :duration (<= ?duration 5)"
    "  :condition (at start (p)) :effect (at end (q))))";
  auto f = fix(plain, "(define (problem x) (:domain d) (:init (p)) (:goal (q)))");
  EXPECT_TRUE(f.unsupported.empty()) << f.unsupported;
  EXPECT_FALSE(f.changed);
  ASSERT_EQ(f.task.durative_actions.size(), 1u);
  EXPECT_EQ(f.task.durative_actions[0].duration.size(), 1u);
  EXPECT_EQ(f.task.durative_actions[0].duration[0].op, sas::CompareOp::Le);
}

TEST(DurationsTest, ConsumptionAtStartProductionAtEnd)
{
  // Read as "at least" (>= 0): drain as soon as it starts, charge only when it ends
  auto f = fix(domain());
  ASSERT_TRUE(f.unsupported.empty()) << f.unsupported;
  EXPECT_TRUE(f.changed);
  auto moves = named(f, "move");
  ASSERT_EQ(moves.size(), 2u);  // a to b, b to a
  const sas::Effect * drain = numeric(*moves[0]);
  ASSERT_NE(drain, nullptr);
  EXPECT_FALSE(drain->continuous);
  EXPECT_EQ(drain->timing, sas::Timing::Start);
  EXPECT_TRUE(moves[0]->uses_duration);
  // Its total: rate times the duration
  EXPECT_DOUBLE_EQ(state::evaluate(drain->value.tree, nullptr, 30.0), 30.0);
  auto recharges = named(f, "recharge");
  ASSERT_FALSE(recharges.empty());
  const sas::Effect * charge = numeric(*recharges[0]);
  ASSERT_NE(charge, nullptr);
  EXPECT_EQ(charge->timing, sas::Timing::End);
  EXPECT_DOUBLE_EQ(state::evaluate(charge->value.tree, nullptr, 2.0), 20.0);
}

TEST(DurationsTest, OneActionPerBound)
{
  // 1 <= d <= (100 - battery) / 10: "the minimum" and "until full", each checking the other
  auto f = fix(domain());
  auto recharges = named(f, "recharge");
  ASSERT_EQ(recharges.size(), 4u);  // two places, two bounds
  std::vector<double> values = {20.0};  // battery, then the two distances
  values.resize(3, 30.0);
  int minimum = 0, full = 0;
  for (const auto * r : recharges) {
    ASSERT_EQ(r->duration.size(), 1u);
    EXPECT_EQ(r->duration[0].op, sas::CompareOp::Eq);
    double d = state::evaluate(r->duration[0].value.tree, values.data(), 0.0);
    minimum += d == 1.0;
    full += d == 8.0;
    EXPECT_EQ(r->at_start.numeric.size(), 2u);  // the chosen one against both bounds
  }
  EXPECT_EQ(minimum, 2);
  EXPECT_EQ(full, 2);
}

TEST(DurationsTest, ReadAsSmallEnough)
{
  // Only "at most" reads: the other way round, a gain at start and a loss at end
  auto f = fix(domain("", "(over all (<= (battery) 100))"));
  ASSERT_TRUE(f.unsupported.empty()) << f.unsupported;
  EXPECT_EQ(numeric(*named(f, "move")[0])->timing, sas::Timing::End);
  EXPECT_EQ(numeric(*named(f, "recharge")[0])->timing, sas::Timing::Start);
}

TEST(DurationsTest, ReadersOfBothKindsAreUnsupported)
{
  auto both = fix(domain(
      " (:action check :parameters () :precondition (<= (battery) 90) :effect (done))"));
  EXPECT_NE(both.unsupported.find("continuous effect"), std::string::npos) << both.unsupported;
  // An equality, or an effect that copies the value, may need any intermediate value
  auto equal = fix(domain(
      " (:action check :parameters () :precondition (= (battery) 50) :effect (done))"));
  EXPECT_FALSE(equal.unsupported.empty());
}

TEST(DurationsTest, NegativeRateGoesTheOtherWay)
{
  // A decrease with a negative rate is a gain: at end for "at least" readers
  std::string d = domain();
  d.replace(d.find("(decrease (battery) (* #t 1))"), 29, "(decrease (battery) (* #t -1))");
  auto f = fix(d);
  ASSERT_TRUE(f.unsupported.empty()) << f.unsupported;
  EXPECT_EQ(numeric(*named(f, "move")[0])->timing, sas::Timing::End);
}

TEST(DurationsTest, RatesWithTheDurationAreUnsupported)
{
  std::string d = domain();
  d.replace(d.find("(* #t 1)"), 8, "(* #t ?duration)");
  EXPECT_FALSE(fix(d).unsupported.empty());
}

TEST(DurationsTest, OnlyAnUpperBound)
{
  // d <= 4 with ?duration in an effect: 0 (the least a duration can be) and 4
  const char * d =
    "(define (domain d) (:requirements :fluents :durative-actions :duration-inequalities)"
    " (:functions (x)) (:predicates (go))"
    " (:durative-action a :parameters () :duration (<= ?duration 4)"
    "  :condition (at start (go)) :effect (at end (increase (x) ?duration))))";
  auto f = fix(d, "(define (problem p) (:domain d) (:init (go) (= (x) 0)) (:goal (>= (x) 4)))");
  ASSERT_TRUE(f.unsupported.empty()) << f.unsupported;
  ASSERT_EQ(f.task.durative_actions.size(), 2u);
  EXPECT_DOUBLE_EQ(
    state::evaluate(f.task.durative_actions[0].duration[0].value.tree, nullptr, 0.0), 0.0);
  EXPECT_DOUBLE_EQ(
    state::evaluate(f.task.durative_actions[1].duration[0].value.tree, nullptr, 0.0), 4.0);
}

TEST(DurationsTest, SnapKeepsADurationThatDependsOnTheState)
{
  // "Until full" reads the battery at start; the charge at end must use that duration even if
  // the battery changes meanwhile: a numeric variable of its own, set at start
  auto f = fix(domain());
  SnapTask s = snap(f.task);
  ASSERT_TRUE(s.unsupported.empty()) << s.unsupported;
  EXPECT_GT(s.task.numeric_fluents.size(), f.task.numeric_fluents.size());
  bool kept = false;
  for (const auto & fn : s.task.function_names) {
    kept = kept || fn.rfind("kairos-duration", 0) == 0;
  }
  EXPECT_TRUE(kept);
}

TEST(DurationsTest, CompressedConditionsSeeTheStart)
{
  // M2: "battery >= 0 over all" after draining 30 at start: not with 20, yes with 40
  auto p = pipeline(domain(), kProblem);
  ASSERT_TRUE(p->ok);
  auto f = fix_durations(p->sas);
  Compressed c = compress(f.task);
  state::StateRegistry reg(c.task);
  state::StateSpace space(c.task, reg);
  auto move = [&]() {
      for (uint32_t i = 0; i < c.task.actions.size(); ++i) {
        if (sas::instance_name(c.task, c.task.actions[i]) == "(move a b)") {
          return i;
        }
      }
      ADD_FAILURE() << "no move";
      return 0u;
    };
  std::vector<uint32_t> ops;
  space.applicable(reg.initial(), ops);
  EXPECT_EQ(std::find(ops.begin(), ops.end(), move()), ops.end());  // 20 < 30

  std::string rich = kProblem;
  rich.replace(rich.find("(= (battery) 20)"), 16, "(= (battery) 40)");
  auto q = pipeline(domain(), rich);
  ASSERT_TRUE(q->ok);
  Compressed c2 = compress(fix_durations(q->sas).task);
  state::StateRegistry reg2(c2.task);
  state::StateSpace space2(c2.task, reg2);
  ops.clear();
  space2.applicable(reg2.initial(), ops);
  bool found = false;
  for (uint32_t op : ops) {
    found = found || sas::instance_name(c2.task, c2.task.actions[op]) == "(move a b)";
  }
  EXPECT_TRUE(found);
}

}  // namespace
}  // namespace kairos::temporal
