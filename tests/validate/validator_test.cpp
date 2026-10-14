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
#include <string>
#include <vector>

#include "kairos/kairos.hpp"

namespace kairos
{

namespace
{
// Sanitizers and coverage run several times slower: time limits scale
#if defined(__SANITIZE_ADDRESS__) || defined(__SANITIZE_THREAD__) || defined(KAIROS_COVERAGE_BUILD)
constexpr int kSlowBuild = 20;
#elif defined(__has_feature)
#if __has_feature(address_sanitizer) || __has_feature(thread_sanitizer)
constexpr int kSlowBuild = 20;
#else
constexpr int kSlowBuild = 1;
#endif
#else
constexpr int kSlowBuild = 1;
#endif
}  // namespace

namespace
{

std::string dump(const ValidationReport & r)
{
  std::string out = std::string(to_string(r.status)) + "\n";
  for (const auto & d : r.diagnostics) {
    out += to_string(d) + "\n";
  }
  return out;
}

ValidationReport check(
  const std::string & domain, const std::string & problem, const std::string & plan_text)
{
  auto plan = parse_plan(plan_text, "plan.txt");
  EXPECT_TRUE(plan.ok());
  ValidateOptions options;
  options.plan_filename = "plan.txt";
  return validate_plan(domain, problem, *plan, options);
}

// First error code, or "" if none
std::string error_code(const ValidationReport & r)
{
  for (const auto & d : r.diagnostics) {
    if (d.severity == Severity::Error) {
      return d.code;
    }
  }
  return "";
}

std::string error_message(const ValidationReport & r)
{
  for (const auto & d : r.diagnostics) {
    if (d.severity == Severity::Error) {
      return d.message;
    }
  }
  return "";
}

// --- Sequential (STRIPS / ADL / numeric) ---------------------------------------------------

const char * kRobots =
  "(define (domain robots) (:requirements :strips :typing :negative-preconditions :adl :fluents)\n"
  " (:types robot room box)\n"
  " (:predicates (at ?r - robot ?l - room) (connected ?a ?b - room) (in ?b - box ?l - room)\n"
  "              (carrying ?r - robot ?b - box) (free ?r - robot))\n"
  " (:functions (battery ?r - robot) (total-cost))\n"
  " (:action move :parameters (?r - robot ?a ?b - room)\n"
  "  :precondition (and (at ?r ?a) (connected ?a ?b) (>= (battery ?r) 10))\n"
  "  :effect (and (not (at ?r ?a)) (at ?r ?b) (decrease (battery ?r) 10)\n"
  "               (increase (total-cost) 1)\n"
  "               (forall (?x - box) (when (carrying ?r ?x) (and (not (in ?x ?a)) (in ?x ?b))))))\n"
  " (:action pick :parameters (?r - robot ?x - box ?l - room)\n"
  "  :precondition (and (at ?r ?l) (in ?x ?l) (free ?r))\n"
  "  :effect (and (carrying ?r ?x) (not (free ?r)) (increase (total-cost) 1)))\n"
  " (:action drop :parameters (?r - robot ?x - box ?l - room)\n"
  "  :precondition (and (at ?r ?l) (carrying ?r ?x))\n"
  "  :effect (and (not (carrying ?r ?x)) (free ?r) (increase (total-cost) 1))))\n";

const char * kRobotsProblem =
  "(define (problem p) (:domain robots)\n"
  " (:objects r1 - robot a b c - room box1 - box)\n"
  " (:init (at r1 a) (connected a b) (connected b c) (in box1 a) (free r1)\n"
  "        (= (battery r1) 25) (= (total-cost) 0))\n"
  " (:goal (and (in box1 c) (free r1)))\n"
  " (:metric minimize (total-cost)))\n";

const char * kRobotsPlan =
  "(pick r1 box1 a)\n(move r1 a b)\n(move r1 b c)\n(drop r1 box1 c)\n";

TEST(ValidateSequentialTest, ValidPlanWithConditionalEffectsAndMetric)
{
  auto r = check(kRobots, kRobotsProblem, kRobotsPlan);
  EXPECT_TRUE(r.valid()) << dump(r);
  ASSERT_TRUE(r.metric.has_value());
  EXPECT_DOUBLE_EQ(*r.metric, 4.0);
  EXPECT_DOUBLE_EQ(r.makespan, 4.0);
}

TEST(ValidateSequentialTest, TotalCostStartsAtZeroWhenNotInitialized)
{
  // As FD and the IPC do (IPC 2014 tetris leaves it out); other fluents stay undefined
  const char * domain =
    "(define (domain c) (:requirements :action-costs) (:predicates (done))"
    " (:functions (total-cost) - number (other) - number)"
    " (:action fin :parameters () :precondition () :effect (and (done)"
    "   (increase (total-cost) 3)))"
    " (:action bad :parameters () :precondition () :effect (increase (other) 1)))";
  const char * problem =
    "(define (problem p) (:domain c) (:init) (:goal (done)) (:metric minimize (total-cost)))";
  auto r = check(domain, problem, "(fin)\n");
  EXPECT_TRUE(r.valid()) << dump(r);
  ASSERT_TRUE(r.metric.has_value());
  EXPECT_DOUBLE_EQ(*r.metric, 3.0);
  auto u = check(domain, problem, "(bad)\n(fin)\n");
  EXPECT_EQ(error_code(u), "validate.undefined_fluent");
}

TEST(ValidateSequentialTest, GoalNotReached)
{
  auto r = check(kRobots, kRobotsProblem, "(pick r1 box1 a)\n(move r1 a b)\n");
  EXPECT_EQ(r.status, ValidationStatus::Invalid);
  EXPECT_EQ(error_code(r), "validate.goal_not_reached");
  EXPECT_NE(error_message(r).find("(in box1 c) is false"), std::string::npos) << dump(r);
}

TEST(ValidateSequentialTest, PreconditionFailsWithTheReasonAndLine)
{
  auto r = check(kRobots, kRobotsProblem, "(pick r1 box1 a)\n(move r1 a c)\n");
  EXPECT_EQ(error_code(r), "validate.precondition_failed");
  EXPECT_NE(error_message(r).find("(connected a c) is false"), std::string::npos) << dump(r);
  EXPECT_NE(error_message(r).find("step 2 (move r1 a c)"), std::string::npos) << dump(r);
  EXPECT_EQ(r.diagnostics.at(0).location->line, 2u);
  EXPECT_EQ(r.diagnostics.at(0).location->file, "plan.txt");
}

TEST(ValidateSequentialTest, NumericPreconditionShowsTheValues)
{
  // The battery runs out after two moves (25 -> 15 -> 5)
  const char * problem =
    "(define (problem p) (:domain robots) (:objects r1 - robot a b c - room box1 - box)"
    " (:init (at r1 a) (connected a b) (connected b a) (= (battery r1) 25) (= (total-cost) 0))"
    " (:goal (at r1 a)))";
  auto r = check(kRobots, problem, "(move r1 a b)\n(move r1 b a)\n(move r1 a b)\n");
  EXPECT_EQ(error_code(r), "validate.precondition_failed");
  EXPECT_NE(error_message(r).find("5 >= 10 is false"), std::string::npos) << dump(r);
}

TEST(ValidateSequentialTest, StepErrors)
{
  EXPECT_EQ(error_code(check(kRobots, kRobotsProblem, "(fly r1 a)\n")),
    "validate.unknown_action");
  EXPECT_EQ(error_code(check(kRobots, kRobotsProblem, "(move r1 a)\n")),
    "validate.wrong_arity");
  EXPECT_EQ(error_code(check(kRobots, kRobotsProblem, "(move r1 a z)\n")),
    "validate.unknown_object");
  EXPECT_EQ(error_code(check(kRobots, kRobotsProblem, "(move a r1 b)\n")),
    "validate.type_mismatch");
}

TEST(ValidateSequentialTest, ExecutableWithoutReachingTheGoal)
{
  auto plan = parse_plan("(pick r1 box1 a)\n(move r1 a b)\n");
  ValidateOptions o;
  o.check_goal = false;
  EXPECT_TRUE(validate_plan(kRobots, kRobotsProblem, *plan, o).valid());
  o.check_goal = true;
  EXPECT_FALSE(validate_plan(kRobots, kRobotsProblem, *plan, o).valid());
}

TEST(ValidateSequentialTest, EmptyPlan)
{
  const char * already = "(define (problem p) (:domain robots) (:objects r1 - robot a - room)"
    " (:init (at r1 a)) (:goal (at r1 a)))";
  EXPECT_TRUE(check(kRobots, already, "").valid());
  EXPECT_FALSE(check(kRobots, kRobotsProblem, "").valid());
}

TEST(ValidateSequentialTest, UndefinedFluent)
{
  const char * problem = "(define (problem p) (:domain robots) (:objects r1 - robot a b - room)"
    " (:init (at r1 a) (connected a b)) (:goal (at r1 b)))";
  auto r = check(kRobots, problem, "(move r1 a b)\n");
  EXPECT_EQ(error_code(r), "validate.evaluation");
  EXPECT_NE(error_message(r).find("(battery r1) is undefined"), std::string::npos) << dump(r);
}

TEST(ValidateSequentialTest, DerivedPredicatesWithRecursion)
{
  const char * domain =
    "(define (domain d) (:requirements :derived-predicates)"
    " (:predicates (link ?a ?b) (reach ?a ?b) (at ?x) (done))"
    " (:derived (reach ?a ?b) (or (link ?a ?b) (exists (?c) (and (link ?a ?c) (reach ?c ?b)))))"
    " (:action finish :parameters (?x ?y) :precondition (and (at ?x) (reach ?x ?y))"
    "  :effect (done)))";
  const char * problem = "(define (problem p) (:domain d) (:objects a b c e)"
    " (:init (at a) (link a b) (link b c)) (:goal (done)))";
  EXPECT_TRUE(check(domain, problem, "(finish a c)\n").valid());
  EXPECT_EQ(error_code(check(domain, problem, "(finish a e)\n")),
    "validate.precondition_failed");
}

TEST(ValidateSequentialTest, DerivedWithNegationInALowerStratum)
{
  const char * domain =
    "(define (domain d) (:requirements :derived-predicates :negative-preconditions)"
    " (:predicates (p ?x) (q ?x) (r ?x) (done))"
    " (:derived (q ?x) (p ?x))"
    " (:derived (r ?x) (not (q ?x)))"
    " (:action go :parameters (?x) :precondition (r ?x) :effect (done)))";
  const char * problem = "(define (problem pr) (:domain d) (:objects a b)"
    " (:init (p a)) (:goal (done)))";
  EXPECT_TRUE(check(domain, problem, "(go b)\n").valid());
  EXPECT_FALSE(check(domain, problem, "(go a)\n").valid());
}

TEST(ValidateSequentialTest, DerivedThroughEqualityAndNestedExists)
{
  // The variable fixed by (= ?t yes) and the one of the nested exists join with the atoms
  const char * domain =
    "(define (domain d) (:requirements :derived-predicates :negative-preconditions :equality)"
    " (:constants yes none)"
    " (:predicates (design ?d) (solves ?d ?f) (active ?f ?t) (done ?f))"
    " (:derived (active ?f ?t) (and (= ?t yes)"
    "   (exists (?d) (and (design ?d) (solves ?d ?f) (not (= ?d none))))))"
    " (:action use :parameters (?f ?t) :precondition (active ?f ?t) :effect (done ?f)))";
  const char * problem = "(define (problem p) (:domain d) (:objects f1 f2 f3 d1 d2)"
    " (:init (design d1) (solves d1 f1) (design none) (solves none f2) (solves d2 f3))"
    " (:goal (done f1)))";
  EXPECT_TRUE(check(domain, problem, "(use f1 yes)\n").valid());
  // Not with the other constant, nor through the excluded design, nor a non-design
  EXPECT_EQ(error_code(check(domain, problem, "(use f1 none)\n")),
    "validate.precondition_failed");
  EXPECT_EQ(error_code(check(domain, problem, "(use f2 yes)\n")),
    "validate.precondition_failed");
  EXPECT_EQ(error_code(check(domain, problem, "(use f3 yes)\n")),
    "validate.precondition_failed");
}

TEST(ValidateSequentialTest, ExistentialVariablesTheBodyNeverMentions)
{
  const char * domain =
    "(define (domain d) (:requirements :typing :existential-preconditions :equality"
    " :negative-preconditions) (:types thing ghost)"
    " (:predicates (p ?x - thing) (done))"
    " (:action some :parameters ()"
    "  :precondition (exists (?a ?b - thing ?c ?e - thing) (and (p ?a) (p ?b) (not (= ?a ?b))))"
    "  :effect (done))"
    " (:action haunted :parameters ()"
    "  :precondition (exists (?a - thing ?g - ghost) (p ?a)) :effect (done)))";
  const char * two = "(define (problem p) (:domain d) (:objects t1 t2 t3 - thing)"
    " (:init (p t1) (p t2)) (:goal (done)))";
  const char * one = "(define (problem p) (:domain d) (:objects t1 t2 t3 - thing)"
    " (:init (p t1)) (:goal (done)))";
  EXPECT_TRUE(check(domain, two, "(some)\n").valid());
  EXPECT_FALSE(check(domain, one, "(some)\n").valid());
  // An unused variable of a type without objects: no witness
  EXPECT_EQ(error_code(check(domain, two, "(haunted)\n")), "validate.precondition_failed");
}

TEST(ValidateSequentialTest, DerivedAtomsFollowOnlyTheChangesTheyRead)
{
  const char * domain =
    "(define (domain d) (:requirements :derived-predicates)"
    " (:predicates (p ?x) (q ?x) (m) (used ?x))"
    " (:derived (q ?x) (p ?x))"
    " (:action set :parameters (?x) :effect (p ?x))"
    " (:action unset :parameters (?x) :effect (not (p ?x)))"
    " (:action mark :effect (m))"
    " (:action use :parameters (?x) :precondition (q ?x) :effect (used ?x)))";
  const char * problem = "(define (problem pr) (:domain d) (:objects a b)"
    " (:init) (:goal (used a)))";
  EXPECT_TRUE(check(domain, problem, "(set a)\n(mark)\n(use a)\n").valid());
  EXPECT_TRUE(check(domain, problem, "(mark)\n(set a)\n(mark)\n(use a)\n(unset a)\n").valid());
  EXPECT_EQ(error_code(check(domain, problem, "(set a)\n(use a)\n(unset a)\n(mark)\n(use a)\n")),
    "validate.precondition_failed");
  EXPECT_EQ(error_code(check(domain, problem, "(set b)\n(mark)\n(use a)\n")),
    "validate.precondition_failed");
}

TEST(ValidateSequentialTest, GoalPreferencesCountViolations)
{
  const char * problem =
    "(define (problem p) (:domain robots) (:objects r1 - robot a b - room box1 - box)"
    " (:init (at r1 a) (connected a b) (free r1) (= (battery r1) 25) (= (total-cost) 0))"
    " (:goal (and (at r1 b) (preference keep (in box1 b))))"
    " (:metric minimize (+ (total-cost) (* 10 (is-violated keep)))))";
  auto r = check(kRobots, problem, "(move r1 a b)\n");
  ASSERT_TRUE(r.valid()) << dump(r);
  EXPECT_EQ(r.violated_preferences["keep"], 1u);
  EXPECT_DOUBLE_EQ(*r.metric, 11.0);
}

TEST(ValidateSequentialTest, DeleteThenAddKeepsTheAtom)
{
  const char * domain = "(define (domain d) (:predicates (p))"
    " (:action toggle :effect (and (not (p)) (p))))";
  const char * problem = "(define (problem pr) (:domain d) (:goal (p)))";
  EXPECT_TRUE(check(domain, problem, "(toggle)\n").valid());
}

// --- Temporal --------------------------------------------------------------------------------

const char * kTemporal =
  "(define (domain t) (:requirements :typing :durative-actions :fluents :timed-initial-literals)\n"
  " (:types robot room)\n"
  " (:predicates (at ?r - robot ?l - room) (connected ?a ?b - room) (open ?l - room)\n"
  "              (light) (fixed) (busy ?r - robot))\n"
  " (:functions (battery ?r - robot) (speed ?r - robot) (dist ?a ?b - room))\n"
  " (:durative-action move :parameters (?r - robot ?a ?b - room)\n"
  "  :duration (= ?duration (/ (dist ?a ?b) (speed ?r)))\n"
  "  :condition (and (at start (at ?r ?a)) (over all (connected ?a ?b)) (at end (open ?b)))\n"
  "  :effect (and (at start (not (at ?r ?a))) (at end (at ?r ?b))\n"
  "               (at end (decrease (battery ?r) (* 2 ?duration)))))\n"
  " (:durative-action strike :parameters ()\n"
  "  :duration (= ?duration 5)\n"
  "  :effect (and (at start (light)) (at end (not (light)))))\n"
  " (:durative-action mend :parameters ()\n"
  "  :duration (= ?duration 2)\n"
  "  :condition (over all (light))\n"
  "  :effect (at end (fixed)))\n"
  " (:durative-action peek :parameters ()\n"
  "  :duration (= ?duration 2)\n"
  "  :condition (at start (light))\n"
  "  :effect (at end (fixed))))\n";

const char * kTemporalProblem =
  "(define (problem p) (:domain t) (:objects r1 - robot a b - room)\n"
  " (:init (at r1 a) (connected a b) (open b) (= (battery r1) 100) (= (speed r1) 2)\n"
  "        (= (dist a b) 10))\n"
  " (:goal (at r1 b)) (:metric minimize (total-time)))\n";

TEST(ValidateTemporalTest, ValidPlan)
{
  auto r = check(kTemporal, kTemporalProblem, "0.000: (move r1 a b) [5.000]\n");
  EXPECT_TRUE(r.valid()) << dump(r);
  EXPECT_DOUBLE_EQ(r.makespan, 5.0);
  EXPECT_DOUBLE_EQ(*r.metric, 5.0);
}

TEST(ValidateTemporalTest, DurationMustMatch)
{
  auto r = check(kTemporal, kTemporalProblem, "0.000: (move r1 a b) [4.000]\n");
  EXPECT_EQ(error_code(r), "validate.duration_mismatch") << dump(r);
  // Three decimals of rounding are tolerated
  const char * thirds = "(define (problem p) (:domain t) (:objects r1 - robot a b - room)"
    " (:init (at r1 a) (connected a b) (open b) (= (battery r1) 100) (= (speed r1) 3)"
    " (= (dist a b) 10)) (:goal (at r1 b)))";
  EXPECT_TRUE(check(kTemporal, thirds, "0.000: (move r1 a b) [3.333]\n").valid());
}

TEST(ValidateTemporalTest, AtEndCondition)
{
  const char * closed = "(define (problem p) (:domain t) (:objects r1 - robot a b - room)"
    " (:init (at r1 a) (connected a b) (= (battery r1) 100) (= (speed r1) 2) (= (dist a b) 10))"
    " (:goal (at r1 b)))";
  auto r = check(kTemporal, closed, "0.000: (move r1 a b) [5.000]\n");
  EXPECT_EQ(error_code(r), "validate.precondition_failed");
  EXPECT_NE(error_message(r).find("at end condition false: (open b) is false"),
    std::string::npos) << dump(r);
}

TEST(ValidateTemporalTest, TimedInitialLiteralsOpenAndClose)
{
  const char * windows = "(define (problem p) (:domain t) (:objects r1 - robot a b - room)"
    " (:init (at r1 a) (connected a b) (= (battery r1) 100) (= (speed r1) 2) (= (dist a b) 10)"
    "        (at 10 (open b)) (at 20 (not (open b))))"
    " (:goal (at r1 b)))";
  EXPECT_FALSE(check(kTemporal, windows, "0.000: (move r1 a b) [5.000]\n").valid());
  EXPECT_TRUE(check(kTemporal, windows, "6.000: (move r1 a b) [5.000]\n").valid());
  EXPECT_FALSE(check(kTemporal, windows, "16.000: (move r1 a b) [5.000]\n").valid());
}

const char * kMatchProblem = "(define (problem m) (:domain t) (:goal (fixed)))";

TEST(ValidateTemporalTest, RequiredConcurrency)
{
  // mend needs the light on during all its execution
  EXPECT_TRUE(check(kTemporal, kMatchProblem,
    "0.000: (strike) [5.000]\n1.000: (mend) [2.000]\n").valid());
  auto alone = check(kTemporal, kMatchProblem, "0.000: (mend) [2.000]\n");
  EXPECT_FALSE(alone.valid());
  // The light goes off before mend ends: the over-all condition breaks
  auto late = check(kTemporal, kMatchProblem,
      "0.000: (strike) [5.000]\n4.000: (mend) [2.000]\n");
  EXPECT_EQ(error_code(late), "validate.invariant_failed") << dump(late);
  EXPECT_NE(error_message(late).find("(light) is false"), std::string::npos) << dump(late);
}

// Verdicts checked against VAL (a variant of this domain with parameters: VAL crashes on
// durative actions without parameters)
TEST(ValidateTemporalTest, OverAllStartsAfterTheHappening)
{
  // mend's over-all condition covers the open interval: the light switched on at the same
  // instant already counts (VAL agrees)
  EXPECT_TRUE(check(kTemporal, kMatchProblem,
    "0.000: (strike) [5.000]\n0.000: (mend) [2.000]\n").valid());
}

TEST(ValidateTemporalTest, InterferenceAndEpsilon)
{
  // peek reads (light) at its start: at the same instant as strike's start, it reads the state
  // before the happening, where the light is off
  auto same = check(kTemporal, kMatchProblem,
      "0.000: (strike) [5.000]\n0.000: (peek) [2.000]\n");
  EXPECT_EQ(error_code(same), "validate.precondition_failed") << dump(same);
  // At strike's end, which deletes the light: simultaneous interference
  auto at_end = check(kTemporal, kMatchProblem,
      "0.000: (strike) [5.000]\n5.000: (peek) [2.000]\n");
  EXPECT_FALSE(at_end.valid()) << dump(at_end);
  // Closer than epsilon: a warning, as VAL accepts it...
  auto close = check(kTemporal, kMatchProblem,
      "0.000: (strike) [5.000]\n0.0005: (peek) [2.000]\n");
  EXPECT_TRUE(close.valid()) << dump(close);
  ASSERT_FALSE(close.diagnostics.empty());
  EXPECT_EQ(close.diagnostics[0].code, "validate.too_close");
  EXPECT_EQ(close.diagnostics[0].severity, Severity::Warning);
  // ...and an error with strict_epsilon
  auto plan = parse_plan("0.000: (strike) [5.000]\n0.0005: (peek) [2.000]\n");
  ValidateOptions strict;
  strict.strict_epsilon = true;
  auto r = validate_plan(kTemporal, kMatchProblem, *plan, strict);
  EXPECT_EQ(error_code(r), "validate.too_close") << dump(r);
  // Exactly epsilon apart, as POPF writes them: valid without warnings
  auto exact = check(kTemporal, kMatchProblem,
      "0.000: (strike) [5.000]\n0.001: (peek) [2.000]\n");
  EXPECT_TRUE(exact.valid());
  EXPECT_TRUE(exact.diagnostics.empty()) << dump(exact);
}

TEST(ValidateTemporalTest, NonInterferingSimultaneousActions)
{
  const char * two = "(define (problem p) (:domain t) (:objects r1 r2 - robot a b - room)"
    " (:init (at r1 a) (at r2 a) (connected a b) (open b) (= (battery r1) 100)"
    " (= (battery r2) 100) (= (speed r1) 2) (= (speed r2) 2) (= (dist a b) 10))"
    " (:goal (and (at r1 b) (at r2 b))))";
  EXPECT_TRUE(check(kTemporal, two,
    "0.000: (move r1 a b) [5.000]\n0.000: (move r2 a b) [5.000]\n").valid());
}

TEST(ValidateTemporalTest, DurationInEffects)
{
  const char * problem = "(define (problem p) (:domain t) (:objects r1 - robot a b - room)"
    " (:init (at r1 a) (connected a b) (open b) (= (battery r1) 100) (= (speed r1) 2)"
    " (= (dist a b) 10)) (:goal (and (at r1 b) (<= (battery r1) 90))))";
  // battery: 100 - 2 * 5 = 90
  EXPECT_TRUE(check(kTemporal, problem, "0.000: (move r1 a b) [5.000]\n").valid());
}

TEST(ValidateTemporalTest, ContinuousEffects)
{
  const char * domain =
    "(define (domain c) (:requirements :durative-actions :fluents :continuous-effects)"
    " (:predicates (done))"
    " (:functions (battery))"
    " (:durative-action work :parameters ()"
    "  :duration (<= ?duration 100)"
    "  :condition (over all (>= (battery) 0))"
    "  :effect (and (decrease (battery) (* #t 2)) (at end (done)))))";
  const char * problem = "(define (problem p) (:domain c) (:init (= (battery) 10))"
    " (:goal (and (done) (<= (battery) 1))))";
  // 10 - 2 * 4.5 = 1
  EXPECT_TRUE(check(domain, problem, "0.000: (work) [4.500]\n").valid());
  // 10 - 2 * 6 = -2: the over-all condition breaks during the action
  auto r = check(domain, problem, "0.000: (work) [6.000]\n");
  EXPECT_EQ(error_code(r), "validate.invariant_failed") << dump(r);
}

TEST(ValidateTemporalTest, NonLinearContinuousEffectsAreIntegrated)
{
  // v grows at 1, x at v: x = t^2 / 2, 0.5 after 1 s (numerically, as PDDL+)
  const char * domain =
    "(define (domain c) (:requirements :durative-actions :fluents :continuous-effects)"
    " (:functions (x) (v))"
    " (:durative-action fall :parameters () :duration (= ?duration 1)"
    "  :effect (and (increase (v) (* #t 1)) (increase (x) (* #t (v))))))";
  auto in = [&](const std::string & goal) {
      return check(domain, "(define (problem p) (:domain c) (:init (= (x) 0) (= (v) 0))"
               " (:goal " + goal + "))", "0.000: (fall) [1.000]\n");
    };
  EXPECT_EQ(in("(and (>= (x) 0.4999) (<= (x) 0.5001))").status, ValidationStatus::Valid);
  EXPECT_EQ(in("(>= (x) 0.6)").status, ValidationStatus::Invalid);
}

TEST(ValidateTemporalTest, ProcessesThatNeverRun)
{
  // A process whose condition never holds changes nothing: the empty plan reaches the goal,
  // and waiting does not change it
  const char * domain = "(define (domain p) (:requirements :time :fluents)"
    " (:predicates (on)) (:functions (t))"
    " (:process tick :parameters () :precondition (on) :effect (increase (t) (* #t 1))))";
  const char * problem = "(define (problem p) (:domain p) (:init (= (t) 0)) (:goal (<= (t) 0)))";
  EXPECT_EQ(check(domain, problem, "").status, ValidationStatus::Valid);
  EXPECT_EQ(check(domain, problem, "5: @PlanEND\n").status, ValidationStatus::Valid);
}

TEST(ValidateTemporalTest, ConditionalEffectAcrossStartAndEnd)
{
  const char * domain =
    "(define (domain w) (:requirements :durative-actions :conditional-effects)"
    " (:predicates (flag) (marked) (done))"
    " (:durative-action go :parameters () :duration (= ?duration 1)"
    "  :effect (and (at start (not (flag))) (at end (done))"
    "               (when (at start (flag)) (at end (marked))))))";
  const char * with_flag = "(define (problem p) (:domain w) (:init (flag))"
    " (:goal (and (done) (marked))))";
  const char * without = "(define (problem p) (:domain w) (:goal (and (done) (marked))))";
  // The condition is read at the start, even though the start deletes the flag
  EXPECT_TRUE(check(domain, with_flag, "0.000: (go) [1.000]\n").valid());
  EXPECT_FALSE(check(domain, without, "0.000: (go) [1.000]\n").valid());
}

TEST(ValidateTemporalTest, InstantActionsInATemporalPlan)
{
  const char * domain = "(define (domain m) (:requirements :durative-actions)"
    " (:predicates (a) (b))"
    " (:action flip :effect (a))"
    " (:durative-action wait :parameters () :duration (= ?duration 2)"
    "  :condition (at start (a)) :effect (at end (b))))";
  const char * problem = "(define (problem p) (:domain m) (:goal (b)))";
  EXPECT_TRUE(check(domain, problem, "0.000: (flip)\n0.001: (wait) [2.000]\n").valid());
  EXPECT_FALSE(check(domain, problem, "0.000: (wait) [2.000]\n0.001: (flip)\n").valid());
  // A duration on an instantaneous action is a warning only
  auto r = check(domain, problem, "0.000: (flip) [1.000]\n1.001: (wait) [2.000]\n");
  EXPECT_TRUE(r.valid()) << dump(r);
}

// --- PDDL 3 constraints and ENHSP global constraints ----------------------------------------

std::string with_constraints(const std::string & constraints)
{
  return "(define (problem p) (:domain counter) (:init (= (n) 0))"
         " (:goal (>= (n) 3)) (:constraints " + constraints + "))";
}

const char * kCounter =
  "(define (domain counter) (:requirements :fluents :constraints)"
  " (:functions (n)) (:action inc :effect (increase (n) 1)))";

const char * kThreeIncs = "0: (inc)\n1: (inc)\n2: (inc)\n";

TEST(ValidateConstraintsTest, Modalities)
{
  auto ok = [&](const std::string & c) {
      return check(kCounter, with_constraints(c), kThreeIncs).valid();
    };
  EXPECT_TRUE(ok("(always (<= (n) 3))"));
  EXPECT_FALSE(ok("(always (<= (n) 2))"));
  EXPECT_TRUE(ok("(sometime (= (n) 2))"));
  EXPECT_FALSE(ok("(sometime (= (n) 7))"));
  EXPECT_TRUE(ok("(within 2 (= (n) 2))"));
  EXPECT_FALSE(ok("(within 1 (= (n) 2))"));
  EXPECT_TRUE(ok("(at-most-once (= (n) 1))"));
  EXPECT_TRUE(ok("(at-most-once (>= (n) -1))"));  // true all along: one interval
  EXPECT_TRUE(ok("(sometime-after (= (n) 1) (= (n) 3))"));
  EXPECT_FALSE(ok("(sometime-after (= (n) 3) (= (n) 1))"));
  EXPECT_TRUE(ok("(sometime-before (= (n) 3) (= (n) 1))"));
  EXPECT_FALSE(ok("(sometime-before (= (n) 1) (= (n) 3))"));
  EXPECT_TRUE(ok("(always-within 1 (= (n) 1) (= (n) 2))"));
  EXPECT_FALSE(ok("(always-within 1 (= (n) 1) (= (n) 3))"));
  EXPECT_TRUE(ok("(hold-during 0 1 (<= (n) 1))"));
  EXPECT_FALSE(ok("(hold-during 0 3 (<= (n) 1))"));
  EXPECT_TRUE(ok("(hold-after 2 (>= (n) 2))"));
  EXPECT_FALSE(ok("(hold-after 0 (>= (n) 2))"));
  EXPECT_TRUE(ok("(at end (= (n) 3))"));
}

TEST(ValidateConstraintsTest, AtMostOnceWithTwoIntervals)
{
  const char * domain = "(define (domain sw) (:requirements :constraints)"
    " (:predicates (on)) (:action up :effect (on)) (:action down :effect (not (on))))";
  const char * problem = "(define (problem p) (:domain sw) (:goal (on))"
    " (:constraints (at-most-once (on))))";
  EXPECT_TRUE(check(domain, problem, "(up)\n").valid());
  EXPECT_FALSE(check(domain, problem, "(up)\n(down)\n(up)\n").valid());
}

TEST(ValidateConstraintsTest, PreferencesInConstraintsAreCounted)
{
  auto r = check(kCounter, with_constraints("(preference small (always (<= (n) 1)))"),
      kThreeIncs);
  ASSERT_TRUE(r.valid()) << dump(r);
  EXPECT_EQ(r.violated_preferences["small"], 1u);
}

TEST(ValidateConstraintsTest, EnhspGlobalConstraints)
{
  const char * domain = "(define (domain g) (:requirements :fluents)"
    " (:functions (v)) (:action push :effect (increase (v) 5))"
    " (:constraint limit :parameters () :condition (<= (v) 7)))";
  const char * problem = "(define (problem p) (:domain g) (:init (= (v) 0)) (:goal (>= (v) 5)))";
  EXPECT_TRUE(check(domain, problem, "(push)\n").valid());
  auto r = check(domain, problem, "(push)\n(push)\n");
  EXPECT_EQ(error_code(r), "validate.global_constraint") << dump(r);
}

// --- Bad input -------------------------------------------------------------------------------

// --- PDDL+: processes, events, global constraints ---------------------------------------------

const char * kCar =
  "(define (domain car) (:requirements :fluents :time :negative-preconditions)"
  " (:predicates (running) (stopped))"
  " (:functions (d) (v) (a) (max_speed))"
  " (:constraint speed_limit :parameters () :condition (<= (v) (max_speed)))"
  " (:process displacement :parameters () :precondition (running)"
  "  :effect (increase (d) (* #t (v))))"
  " (:process moving :parameters () :precondition (running) :effect (increase (v) (* #t (a))))"
  " (:action accelerate :parameters () :precondition (running) :effect (increase (a) 1))"
  " (:action decelerate :parameters () :precondition (running) :effect (decrease (a) 1))"
  " (:action start :parameters () :precondition (stopped)"
  "  :effect (and (running) (not (stopped))))"
  " (:action stop :parameters () :precondition (and (running) (< (v) 0.1) (> (v) -0.1))"
  "  :effect (and (stopped) (not (running)) (assign (v) 0))))";

std::string car_problem(double goal_low, double goal_high, double max_speed = 10)
{
  return "(define (problem p) (:domain car) (:init (stopped) (= (d) 0) (= (v) 0) (= (a) 0)"
         " (= (max_speed) " + std::to_string(max_speed) + ")) (:goal (and (stopped) (>= (d) " +
         std::to_string(goal_low) + ") (<= (d) " + std::to_string(goal_high) + "))))";
}

TEST(ValidatePddlPlusTest, ProcessesChangeFluentsBetweenActions)
{
  // Accelerate for 2 s (v = 2, d = 2), cruise for 3 s (d = 8), brake for 2 s (v = 0, d = 10):
  // d is quadratic while accelerating, exact with Runge-Kutta 4
  const char * plan =
    "0: (start)\n0: (accelerate)\n2: (decelerate)\n5: (decelerate)\n7: (accelerate)\n"
    "7: (stop)\n7: @PlanEND\n";
  auto r = check(kCar, car_problem(9.999, 10.001), plan);
  EXPECT_EQ(r.status, ValidationStatus::Valid) << dump(r);
  EXPECT_NEAR(r.makespan, 7.0, 1e-9);
  // Short of the distance: braking a second earlier
  auto early = check(kCar, car_problem(9.999, 10.001),
      "0: (start)\n0: (accelerate)\n2: (decelerate)\n4: (decelerate)\n6: (accelerate)\n"
      "6: (stop)\n6: @PlanEND\n");
  EXPECT_EQ(error_code(early), "validate.goal_not_reached");
  // Steps of one instant apply in plan order (start, then accelerate): reversed, accelerate
  // finds the engine stopped
  auto reversed = check(kCar, car_problem(9.999, 10.001),
      "0: (accelerate)\n0: (start)\n2: (decelerate)\n5: (decelerate)\n7: (accelerate)\n"
      "7: (stop)\n7: @PlanEND\n");
  EXPECT_EQ(error_code(reversed), "validate.precondition_failed");
}

TEST(ValidatePddlPlusTest, GlobalConstraintsHoldBetweenActions)
{
  // At most 3 m/s: accelerating for 4 s breaks it at 3 s, between the steps at 0 and 4
  auto r = check(kCar, car_problem(0, 100, 3),
      "0: (start)\n0: (accelerate)\n4: (decelerate)\n4: @PlanEND\n");
  EXPECT_EQ(error_code(r), "validate.global_constraint");
  ASSERT_FALSE(r.diagnostics.empty());
  EXPECT_NE(r.diagnostics[0].message.find("at 3.0"), std::string::npos) << dump(r);
}

const char * kTank =
  "(define (domain tank) (:requirements :fluents :time :negative-preconditions)"
  " (:predicates (open) (flooded) (alarm))"
  " (:functions (level) (rate))"
  " (:process filling :parameters () :precondition (open)"
  "  :effect (increase (level) (* #t (rate))))"
  " (:event overflow :parameters () :precondition (and (>= (level) 10) (not (flooded)))"
  "  :effect (flooded))"
  " (:event siren :parameters () :precondition (and (flooded) (not (alarm))) :effect (alarm))"
  " (:action open-valve :parameters () :precondition (not (open)) :effect (open))"
  " (:action close-valve :parameters () :precondition (open) :effect (not (open))))";

std::string tank(const std::string & goal)
{
  return "(define (problem p) (:domain tank) (:init (= (level) 0) (= (rate) 2)) (:goal " + goal +
         "))";
}

TEST(ValidatePddlPlusTest, EventsFireWhenTheirConditionBecomesTrue)
{
  // 2 per second: closed at 4 s, level 8, no overflow
  auto ok = check(kTank, tank("(and (>= (level) 7.99) (not (flooded)))"),
      "0: (open-valve)\n4: (close-valve)\n4: @PlanEND\n");
  EXPECT_EQ(ok.status, ValidationStatus::Valid) << dump(ok);
  // Closed at 6 s: at 5 s the level reaches 10, the overflow fires, and the siren after it (a
  // cascade at the same instant)
  auto flooded = check(kTank, tank("(and (flooded) (alarm) (>= (level) 11.99))"),
      "0: (open-valve)\n6: (close-valve)\n6: @PlanEND\n");
  EXPECT_EQ(flooded.status, ValidationStatus::Valid) << dump(flooded);
  EXPECT_EQ(error_code(check(kTank, tank("(not (flooded))"),
      "0: (open-valve)\n6: (close-valve)\n6: @PlanEND\n")), "validate.goal_not_reached");
  // Time passes after the last step: open at 0, the plan ends at 7
  auto waits = check(kTank, tank("(flooded)"), "0: (open-valve)\n7: @PlanEND\n");
  EXPECT_EQ(waits.status, ValidationStatus::Valid) << dump(waits);
  // Without the end marker, the goal is checked at the last step
  EXPECT_EQ(error_code(check(kTank, tank("(flooded)"), "0: (open-valve)\n")),
    "validate.goal_not_reached");
}

TEST(ValidatePddlPlusTest, EventsThatKeepFiringAreAnError)
{
  auto r = check(
    "(define (domain loop) (:requirements :fluents :time) (:predicates (p)) (:functions (x))"
    " (:event bump :parameters () :precondition (p) :effect (increase (x) 1))"
    " (:action go :parameters () :precondition () :effect (p)))",
    "(define (problem q) (:domain loop) (:init (= (x) 0)) (:goal (p)))", "0: (go)\n");
  EXPECT_EQ(error_code(r), "validate.event_loop");
}

TEST(ValidatePddlPlusTest, NonLinearDynamics)
{
  // x' = y, y' = x: from x = 1, y = 0, x = cosh(t); at t = 1, 1.5431 (RK4 within 1e-9 with
  // 0.01 steps)
  const char * domain =
    "(define (domain osc) (:requirements :fluents :time) (:predicates (on))"
    " (:functions (x) (y))"
    " (:process grow :parameters () :precondition (on)"
    "  :effect (and (increase (x) (* #t (y))) (increase (y) (* #t (x)))))"
    " (:action go :parameters () :precondition () :effect (on)))";
  auto in = [&](double lo, double hi) {
      return check(domain, "(define (problem q) (:domain osc) (:init (= (x) 1) (= (y) 0))"
               " (:goal (and (>= (x) " + std::to_string(lo) + ") (<= (x) " + std::to_string(hi) +
               "))))", "0: (go)\n1: @PlanEND\n");
    };
  double cosh1 = std::cosh(1.0);
  EXPECT_EQ(in(cosh1 - 1e-6, cosh1 + 1e-6).status, ValidationStatus::Valid);
  EXPECT_EQ(in(cosh1 + 1e-4, cosh1 + 1).status, ValidationStatus::Invalid);
}

TEST(ValidatePddlPlusTest, ConditionsOnUndefinedValuesAreFalse)
{
  // (clock) has no value until (go): the process that reads it is off, the one that changes it
  // leaves it without a value (as ENHSP), and (not (= ...)) with it holds
  const char * domain =
    "(define (domain c) (:requirements :fluents :time :negative-preconditions)"
    " (:predicates (on) (seen)) (:functions (clock) (x))"
    " (:process tick :parameters () :precondition () :effect (increase (clock) (* #t 1)))"
    " (:process grow :parameters () :precondition (< (clock) 100)"
    "  :effect (increase (x) (* #t 1)))"
    " (:event look :parameters () :precondition (and (not (seen)) (not (= (clock) 5)))"
    "  :effect (seen))"
    " (:action go :parameters () :precondition () :effect (assign (clock) 0)))";
  auto in = [&](const std::string & goal, const std::string & plan) {
      return check(domain, "(define (problem q) (:domain c) (:init (= (x) 0)) (:goal " + goal +
               "))", plan);
    };
  // x grows only once the clock has a value: from 2 to 3
  auto r = in("(and (seen) (>= (x) 0.999) (<= (x) 1.001) (>= (clock) 0.999))",
      "2: (go)\n3: @PlanEND\n");
  EXPECT_EQ(r.status, ValidationStatus::Valid) << dump(r);
}

TEST(ValidatePddlPlusTest, EventConditionsAreExact)
{
  // Two events on the two sides of x = 1 (>= and <): x stops just below 1, where only fall
  // holds; with the tolerance of action preconditions rise would too, and they would keep
  // undoing each other
  const char * domain =
    "(define (domain side) (:requirements :fluents :time :negative-preconditions)"
    " (:predicates (on) (high)) (:functions (x))"
    " (:process up :parameters () :precondition (on) :effect (increase (x) (* #t 1)))"
    " (:event rise :parameters () :precondition (and (not (high)) (>= (x) 1)) :effect (high))"
    " (:event fall :parameters () :precondition (and (high) (< (x) 1)) :effect (not (high)))"
    " (:action go :parameters () :precondition () :effect (on))"
    " (:action halt :parameters () :precondition (on) :effect (not (on))))";
  auto r = check(domain,
      "(define (problem q) (:domain side) (:init (= (x) 0)) (:goal (not (high))))",
      "0: (go)\n0.9999999995: (halt)\n2: @PlanEND\n");
  EXPECT_EQ(r.status, ValidationStatus::Valid) << dump(r);
  // Past 1: rise, once
  auto past = check(domain,
      "(define (problem q) (:domain side) (:init (= (x) 0)) (:goal (high)))",
      "0: (go)\n1.5: (halt)\n2: @PlanEND\n");
  EXPECT_EQ(past.status, ValidationStatus::Valid) << dump(past);
}

TEST(ValidatePddlPlusTest, ChatteringConditionsEnd)
{
  // Friction around 0: each process pushes v to the other side, the conditions flip with ever
  // shorter steps (Zeno); the validator samples them and finishes
  const char * domain =
    "(define (domain friction) (:requirements :fluents :time) (:predicates (on))"
    " (:functions (v) (t))"
    " (:process clock :parameters () :precondition (on) :effect (increase (t) (* #t 1)))"
    " (:process pos :parameters () :precondition (> (v) 0) :effect (decrease (v) (* #t 1)))"
    " (:process neg :parameters () :precondition (< (v) 0) :effect (increase (v) (* #t 1)))"
    " (:action go :parameters () :precondition () :effect (on)))";
  auto start = std::chrono::steady_clock::now();
  auto r = check(domain,
      "(define (problem q) (:domain friction) (:init (= (v) 0.5) (= (t) 0))"
      " (:goal (and (>= (t) 9.99) (<= (v) 0.01) (>= (v) -0.01))))",
      "0: (go)\n10: @PlanEND\n");
  EXPECT_EQ(r.status, ValidationStatus::Valid) << dump(r);
  EXPECT_LT(std::chrono::steady_clock::now() - start, kSlowBuild * std::chrono::seconds(2));
}

TEST(ValidatePddlPlusTest, EventLoopNamesTheEvents)
{
  auto r = check(
    "(define (domain loop) (:requirements :fluents :time) (:predicates (p)) (:functions (x))"
    " (:event bump :parameters () :precondition (p) :effect (increase (x) 1))"
    " (:action go :parameters () :precondition () :effect (p)))",
    "(define (problem q) (:domain loop) (:init (= (x) 0)) (:goal (p)))", "0: (go)\n");
  ASSERT_FALSE(r.diagnostics.empty());
  EXPECT_NE(r.diagnostics[0].message.find("(bump)"), std::string::npos) << dump(r);
}

TEST(ValidatePddlPlusTest, GroundingsOfStaticFactsThatNeverHoldAreLeftOut)
{
  // 200 x 200 pairs, one link: the process runs for that one only (and the plan validates
  // quickly)
  std::string objects;
  for (int i = 0; i < 200; ++i) {
    objects += " n" + std::to_string(i);
  }
  const char * domain =
    "(define (domain links) (:requirements :fluents :time :typing) (:types node)"
    " (:predicates (link ?a ?b - node) (on)) (:functions (flow ?a ?b - node))"
    " (:process pump :parameters (?a ?b - node) :precondition (and (on) (link ?a ?b))"
    "  :effect (increase (flow ?a ?b) (* #t 1)))"
    " (:action go :parameters () :precondition () :effect (on)))";
  auto start = std::chrono::steady_clock::now();
  auto r = check(domain,
      "(define (problem q) (:domain links) (:objects" + objects + " - node)"
      " (:init (link n1 n2) (= (flow n1 n2) 0)) (:goal (>= (flow n1 n2) 4.99)))",
      "0: (go)\n5: @PlanEND\n");
  EXPECT_EQ(r.status, ValidationStatus::Valid) << dump(r);
  EXPECT_LT(std::chrono::steady_clock::now() - start, kSlowBuild * std::chrono::seconds(2));
}

TEST(ValidateInputTest, BrokenDomainOrProblem)
{
  auto r = check("(define (domain d", kRobotsProblem, "");
  EXPECT_EQ(r.status, ValidationStatus::BadInput);
  EXPECT_FALSE(r.diagnostics.empty());
  auto p = check(kRobots, "(define (problem p) (:domain robots) (:init (free nobody)))", "");
  EXPECT_EQ(p.status, ValidationStatus::BadInput);
}

TEST(ValidateInputTest, StatusNames)
{
  EXPECT_EQ(to_string(ValidationStatus::Valid), "valid");
  EXPECT_EQ(to_string(ValidationStatus::Invalid), "invalid");
  EXPECT_EQ(to_string(ValidationStatus::Unsupported), "unsupported");
  EXPECT_EQ(to_string(ValidationStatus::BadInput), "bad input");
}

TEST(ValidatePerformanceTest, LongSequentialPlan)
{
  // 20000 steps: the validator must stay linear in the plan length
  std::string plan;
  for (int i = 0; i < 20000; ++i) {
    plan += "(inc)\n";
  }
  const char * problem = "(define (problem p) (:domain counter) (:init (= (n) 0))"
    " (:goal (>= (n) 20000)))";
  auto r = check(kCounter, problem, plan);
  EXPECT_TRUE(r.valid()) << dump(r);
}

TEST(ValidatePerformanceTest, ManyUntypedObjectsWithDerivedPredicates)
{
  // As in PlanSys2's suave domain: untyped exists with unused variables and a derived
  // predicate over every object, rechecked at each of 1000 steps over 1000 objects
  const char * domain =
    "(define (domain s) (:requirements :derived-predicates :existential-preconditions"
    " :negative-preconditions :equality) (:constants on)"
    " (:predicates (f ?x) (solves ?d ?x) (active ?x ?t) (found ?p))"
    " (:derived (active ?x ?t) (and (= ?t on) (exists (?d) (and (f ?x) (solves ?d ?x)))))"
    " (:action search :parameters (?p)"
    "  :precondition (exists (?a ?b ?u ?v) (and (active ?a on) (active ?b on) (not (= ?a ?b))))"
    "  :effect (found ?p)))";
  std::string objects, goal, plan;
  for (int i = 0; i < 1000; ++i) {
    objects += " p" + std::to_string(i);
    goal += " (found p" + std::to_string(i) + ")";
    plan += "(search p" + std::to_string(i) + ")\n";
  }
  std::string problem = "(define (problem p) (:domain s) (:objects g1 g2 d1" + objects + ")"
    " (:init (f g1) (f g2) (solves d1 g1) (solves d1 g2)) (:goal (and" + goal + ")))";
  auto r = check(domain, problem, plan);
  EXPECT_TRUE(r.valid()) << dump(r);
}

TEST(ValidatePerformanceTest, ManyOverlappingDurativeActions)
{
  std::string domain =
    "(define (domain par) (:requirements :durative-actions :typing) (:types w)"
    " (:predicates (done ?x - w))"
    " (:durative-action job :parameters (?x - w) :duration (= ?duration 10)"
    "  :effect (at end (done ?x))))";
  std::string objects;
  std::string goal;
  std::string plan;
  for (int i = 0; i < 2000; ++i) {
    objects += " w" + std::to_string(i);
    goal += " (done w" + std::to_string(i) + ")";
    plan += std::to_string(i) + ".000: (job w" + std::to_string(i) + ") [10.000]\n";
  }
  std::string problem = "(define (problem p) (:domain par) (:objects" + objects + " - w)"
    " (:goal (and" + goal + ")))";
  auto r = check(domain, problem, plan);
  EXPECT_TRUE(r.valid()) << dump(r);
  EXPECT_DOUBLE_EQ(r.makespan, 2009.0);
}

}  // namespace
}  // namespace kairos
