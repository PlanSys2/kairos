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

#include <string>
#include <vector>

#include "kairos/kairos.hpp"

namespace kairos
{
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

TEST(ValidateTemporalTest, NonLinearContinuousIsUnsupported)
{
  const char * domain =
    "(define (domain c) (:requirements :durative-actions :fluents :continuous-effects)"
    " (:functions (x) (v))"
    " (:durative-action fall :parameters () :duration (= ?duration 1)"
    "  :effect (and (increase (v) (* #t 1)) (increase (x) (* #t (v))))))";
  const char * problem = "(define (problem p) (:domain c) (:init (= (x) 0) (= (v) 0))"
    " (:goal (>= (x) 0)))";
  auto r = check(domain, problem, "0.000: (fall) [1.000]\n");
  EXPECT_EQ(r.status, ValidationStatus::Unsupported) << dump(r);
}

TEST(ValidateTemporalTest, ProcessesAreUnsupported)
{
  const char * domain = "(define (domain p) (:requirements :time :fluents)"
    " (:predicates (on)) (:functions (t))"
    " (:process tick :parameters () :precondition (on) :effect (increase (t) (* #t 1))))";
  const char * problem = "(define (problem p) (:domain p) (:init (= (t) 0)) (:goal (>= (t) 0)))";
  EXPECT_EQ(check(domain, problem, "").status, ValidationStatus::Unsupported);
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

TEST(ValidateInputTest, BrokenDomainOrProblem)
{
  auto r = check("(define (domain d", kRobotsProblem, "");
  EXPECT_EQ(r.status, ValidationStatus::BadInput);
  EXPECT_FALSE(r.diagnostics.empty());
  auto p = check(kRobots, "(define (problem p) (:domain robots) (:init (nope)))", "");
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
