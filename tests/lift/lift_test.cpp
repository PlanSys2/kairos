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
#include <optional>
#include <string>

#include "kairos/validate.hpp"
#include "lift/lift.hpp"
#include "support/task_helpers.hpp"

namespace kairos::lift
{
namespace
{

using kairos::testing::normalize_text;

struct Outcome
{
  std::optional<Plan> plan;
  Stats stats;
  bool valid = false;
};

Outcome solve(
  const std::string & domain, const std::string & problem, Budget * budget = nullptr,
  const std::function<bool()> & stop = {})
{
  Outcome out;
  auto n = normalize_text(domain, problem);
  EXPECT_EQ(unsupported(n->task), "");
  out.plan = plan(n->task, budget, Logger{}, out.stats, stop);
  if (out.plan) {
    auto r = validate_plan(domain, problem, *out.plan);
    out.valid = r.valid();
  }
  return out;
}

std::string unsupported_of(const std::string & domain, const std::string & problem)
{
  return unsupported(normalize_text(domain, problem)->task);
}

const char * kBlocks =
  R"(
(define (domain blocks)
  (:requirements :strips :typing)
  (:types block)
  (:predicates (on ?x ?y - block) (ontable ?x - block) (clear ?x - block)
               (handempty) (holding ?x - block))
  (:action pick-up :parameters (?x - block)
    :precondition (and (clear ?x) (ontable ?x) (handempty))
    :effect (and (not (ontable ?x)) (not (clear ?x)) (not (handempty)) (holding ?x)))
  (:action put-down :parameters (?x - block)
    :precondition (holding ?x)
    :effect (and (not (holding ?x)) (clear ?x) (handempty) (ontable ?x)))
  (:action stack :parameters (?x ?y - block)
    :precondition (and (holding ?x) (clear ?y))
    :effect (and (not (holding ?x)) (not (clear ?y)) (clear ?x) (handempty) (on ?x ?y)))
  (:action unstack :parameters (?x ?y - block)
    :precondition (and (on ?x ?y) (clear ?x) (handempty))
    :effect (and (holding ?x) (clear ?y) (not (clear ?x)) (not (handempty))
                 (not (on ?x ?y)))))
)";

TEST(LiftTest, PlansBlocksworldAndTheValidatorAccepts)
{
  auto o = solve(kBlocks,
        R"(
(define (problem p) (:domain blocks)
  (:objects a b c d e - block)
  (:init (clear a) (on a b) (on b c) (ontable c) (clear d) (ontable d) (clear e) (ontable e)
         (handempty))
  (:goal (and (on c b) (on b a) (on a d) (on d e))))
)");
  ASSERT_TRUE(o.plan);
  EXPECT_TRUE(o.valid);
  EXPECT_GT(o.stats.expanded, 0u);
  EXPECT_GE(o.stats.generated, o.stats.expanded);
}

TEST(LiftTest, GoalTrueAtStartGivesAnEmptyPlan)
{
  auto o = solve(kBlocks,
        R"(
(define (problem p) (:domain blocks)
  (:objects a b - block)
  (:init (clear a) (on a b) (ontable b) (handempty))
  (:goal (on a b)))
)");
  ASSERT_TRUE(o.plan);
  EXPECT_TRUE(o.plan->steps.empty());
  EXPECT_TRUE(o.valid);
}

// Types, equality, negative preconditions and a constant of the domain
const char * kRooms =
  R"(
(define (domain rooms)
  (:requirements :strips :typing :equality :negative-preconditions)
  (:types room ball robot)
  (:constants hall - room)
  (:predicates (at ?r - robot ?x - room) (in ?b - ball ?x - room) (carry ?r - robot ?b - ball)
               (busy ?r - robot) (door ?x ?y - room) (locked ?x - room))
  (:action move :parameters (?r - robot ?from ?to - room)
    :precondition (and (at ?r ?from) (door ?from ?to) (not (= ?from ?to)) (not (locked ?to)))
    :effect (and (not (at ?r ?from)) (at ?r ?to)))
  (:action pick :parameters (?r - robot ?b - ball ?x - room)
    :precondition (and (at ?r ?x) (in ?b ?x) (not (busy ?r)))
    :effect (and (carry ?r ?b) (busy ?r) (not (in ?b ?x))))
  (:action drop :parameters (?r - robot ?b - ball ?x - room)
    :precondition (and (at ?r ?x) (carry ?r ?b))
    :effect (and (not (carry ?r ?b)) (not (busy ?r)) (in ?b ?x)))
  (:action unlock :parameters (?r - robot ?x - room)
    :precondition (and (at ?r hall) (locked ?x))
    :effect (not (locked ?x))))
)";

TEST(LiftTest, RespectsTypesEqualityNegationAndConstants)
{
  auto o = solve(kRooms,
        R"(
(define (problem p) (:domain rooms)
  (:objects r1 r2 r3 - room b1 b2 b3 - ball bot - robot)
  (:init (at bot r1) (in b1 r1) (in b2 r2) (in b3 r1) (locked r3)
         (door r1 hall) (door hall r1) (door r2 hall) (door hall r2) (door r3 hall)
         (door hall r3) (door r1 r1))
  (:goal (and (in b1 r3) (in b2 r3) (in b3 r2) (not (busy bot)))))
)");
  ASSERT_TRUE(o.plan);
  EXPECT_TRUE(o.valid);
  // r3 is locked at the start: the plan unlocks it from the hall first
  bool unlocked = false;
  for (const auto & s : o.plan->steps) {
    unlocked = unlocked || s.name == "unlock";
    if (s.name == "move" && s.args.size() == 3 && s.args[2] == "r3") {
      EXPECT_TRUE(unlocked);
    }
  }
}

TEST(LiftTest, ParametersOnlyInEffectsTakeEveryObjectOfTheirType)
{
  const char * domain =
    R"(
(define (domain paint)
  (:requirements :strips :typing)
  (:types item color)
  (:predicates (raw ?i - item) (painted ?i - item ?c - color) (allowed ?i - item))
  (:action make :parameters (?i - item)
    :precondition (allowed ?i)
    :effect (raw ?i))
  (:action paint :parameters (?i - item ?c - color)
    :precondition (raw ?i)
    :effect (and (not (raw ?i)) (painted ?i ?c))))
)";
  // make has only static preconditions: the heuristic must see its effects from the start
  auto o = solve(domain,
        R"(
(define (problem p) (:domain paint)
  (:objects i1 i2 - item red blue - color)
  (:init (allowed i1) (allowed i2))
  (:goal (and (painted i1 blue) (painted i2 red))))
)");
  ASSERT_TRUE(o.plan);
  EXPECT_TRUE(o.valid);
  EXPECT_EQ(o.plan->steps.size(), 4u);
}

TEST(LiftTest, ZeroArityPredicatesAndRepeatedVariables)
{
  const char * domain =
    R"(
(define (domain loops)
  (:requirements :strips)
  (:predicates (loop ?x) (seen ?x) (armed) (done))
  (:action arm :parameters () :precondition (not (armed)) :effect (armed))
  (:action see :parameters (?x) :precondition (and (armed) (loop ?x)) :effect (seen ?x))
  (:action finish :parameters (?x ?y)
    :precondition (and (seen ?x) (seen ?y) (loop ?x))
    :effect (done)))
)";
  auto o = solve(domain,
        R"(
(define (problem p) (:domain loops)
  (:objects a b c)
  (:init (loop b))
  (:goal (done)))
)");
  ASSERT_TRUE(o.plan);
  EXPECT_TRUE(o.valid);
  EXPECT_EQ(o.plan->steps.size(), 3u);  // arm, see b, finish b b
}

TEST(LiftTest, AGoalNoRealStateHoldsIsUnsolvable)
{
  auto o = solve(kBlocks,
        R"(
(define (problem p) (:domain blocks)
  (:objects a b - block)
  (:init (clear a) (ontable a) (clear b) (ontable b) (handempty))
  (:goal (on a a)))
)");
  // (on a a) is reachable relaxed (stack a a needs holding a and clear a, both reachable)...
  // ...but never in a real state: the search exhausts
  EXPECT_FALSE(o.plan);
}

TEST(LiftTest, AFalseStaticGoalIsUnsolvable)
{
  auto o = solve(kRooms,
        R"(
(define (problem p) (:domain rooms)
  (:objects r1 - room bot - robot)
  (:init (at bot r1))
  (:goal (door r1 hall)))
)");
  EXPECT_FALSE(o.plan);
  EXPECT_EQ(o.stats.expanded, 0u);
}

TEST(LiftTest, ExhaustsAReachableButUnsolvableTask)
{
  const char * domain =
    R"(
(define (domain oneway)
  (:requirements :strips :negative-preconditions)
  (:predicates (lit ?x) (next ?x ?y) (burnt))
  (:action light :parameters (?x ?y)
    :precondition (and (lit ?x) (next ?x ?y) (not (lit ?y)))
    :effect (and (lit ?y) (burnt))))
)";
  auto o = solve(domain,
        R"(
(define (problem p) (:domain oneway)
  (:objects a b c d)
  (:init (lit a) (next a b) (next b c) (next c d))
  (:goal (and (lit d) (not (burnt)))))
)");
  EXPECT_FALSE(o.plan);
  EXPECT_GT(o.stats.states, 1u);
}

TEST(LiftTest, StopsWhenTheBudgetIsSpentOrAskedTo)
{
  std::string objects;
  std::string init = "(handempty)";
  std::string goal;
  for (int i = 0; i < 40; ++i) {
    std::string b = "b" + std::to_string(i);
    objects += b + " ";
    init += " (ontable " + b + ") (clear " + b + ")";
    if (i > 0) {
      goal += " (on b" + std::to_string(i) + " b" + std::to_string(i - 1) + ")";
    }
  }
  std::string problem = "(define (problem p) (:domain blocks) (:objects " + objects +
    "- block) (:init " + init + ") (:goal (and" + goal + ")))";
  BudgetConfig c;
  c.deadline = BudgetConfig::Clock::now() - std::chrono::seconds(1);
  Budget spent(c);
  auto o = solve(kBlocks, problem, &spent);
  EXPECT_FALSE(o.plan);
  auto asked = solve(kBlocks, problem, nullptr, [] {return true;});
  EXPECT_FALSE(asked.plan);
  EXPECT_LE(asked.stats.expanded, 1u);
}

TEST(LiftTest, LargerTasksStayFocused)
{
  // A corridor of 60 rooms with balls at the far end: the relaxed plan guides the robot
  std::string rooms;
  std::string init = "(at bot r0)";
  for (int i = 0; i < 60; ++i) {
    rooms += "r" + std::to_string(i) + " ";
    if (i > 0) {
      init += " (door r" + std::to_string(i - 1) + " r" + std::to_string(i) + ")";
      init += " (door r" + std::to_string(i) + " r" + std::to_string(i - 1) + ")";
    }
  }
  init += " (door r0 hall) (door hall r0) (in b1 r59) (in b2 r58)";
  std::string problem = "(define (problem p) (:domain rooms) (:objects " + rooms +
    "- room b1 b2 - ball bot - robot) (:init " + init + ") (:goal (and (in b1 r0) (in b2 r1))))";
  auto o = solve(kRooms, problem);
  ASSERT_TRUE(o.plan);
  EXPECT_TRUE(o.valid);
  EXPECT_LT(o.stats.expanded, 2000u);
}

TEST(LiftTest, TellsWhatItCannotPlan)
{
  const char * problem = "(define (problem p) (:domain d) (:objects a) (:init) (:goal (p a)))";
  auto domain = [](const std::string & reqs, const std::string & body) {
      return "(define (domain d) (:requirements :strips " + reqs + ") (:predicates (p ?x) (q ?x))"
             " (:functions (f) (total-cost)) " + body + ")";
    };
  EXPECT_EQ(
    unsupported_of(
      domain(":action-costs", "(:action a :parameters (?x) :precondition (q ?x) "
      ":effect (and (p ?x) (increase (total-cost) 1)))"), problem), "");
  EXPECT_NE(
    unsupported_of(
      domain(":numeric-fluents", "(:action a :parameters (?x) :precondition (q ?x) "
      ":effect (and (p ?x) (increase (f) 1)))"), problem), "");
  EXPECT_NE(
    unsupported_of(
      domain(":numeric-fluents", "(:action a :parameters (?x) :precondition (> (f) 0) "
      ":effect (p ?x))"), problem), "");
  EXPECT_NE(
    unsupported_of(
      domain(":conditional-effects", "(:action a :parameters (?x) :precondition (q ?x) "
      ":effect (when (q ?x) (p ?x)))"), problem), "");
  EXPECT_NE(
    unsupported_of(
      domain(":universal-preconditions", "(:action a :parameters (?x) "
      ":precondition (forall (?y) (q ?y)) :effect (p ?x))"), problem), "");
  EXPECT_NE(
    unsupported_of(
      domain(":durative-actions", "(:durative-action a :parameters (?x) :duration (= ?duration 1) "
      ":condition (at start (q ?x)) :effect (at end (p ?x)))"), problem), "");
  EXPECT_NE(
    unsupported_of(
      domain(":derived-predicates", "(:derived (p ?x) (q ?x)) (:action a :parameters (?x) "
      ":precondition (q ?x) :effect (q ?x))"), problem), "");
  // A static object fluent read as a term (the engine reads arguments as objects)
  EXPECT_NE(
    unsupported_of(
      "(define (domain d) (:requirements :strips :object-fluents) (:predicates (p ?x) (q ?x))"
      " (:functions (next ?x) - object) (:action a :parameters (?x)"
      " :precondition (q (next ?x)) :effect (p ?x)))",
      "(define (problem p) (:domain d) (:objects a b) (:init (= (next a) b) (q b))"
      " (:goal (p a)))"), "");
}

}  // namespace
}  // namespace kairos::lift
