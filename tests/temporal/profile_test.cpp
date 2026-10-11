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
#include <cstdlib>
#include <filesystem>
#include <string>
#include <vector>

#include "support/pddl_helpers.hpp"
#include "support/pipeline_helpers.hpp"
#include "temporal/profile.hpp"

namespace kairos::temporal
{
namespace
{

using kairos::testing::data_path;
using kairos::testing::pipeline;
using kairos::testing::read_file;

TemporalProfile profile(const std::string & domain, const std::string & problem)
{
  auto p = pipeline(domain, problem);
  EXPECT_TRUE(p->ok);
  return analyze(p->sas);
}

// A match lights a fuse box only while it burns: mending needs the light over all
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

TEST(TemporalProfileTest, AMatchThatBurnsWhileSomethingElseHappens)
{
  auto p = profile(kCellar, kCellarProblem);
  EXPECT_TRUE(p.durative);
  EXPECT_FALSE(p.simple);
  ASSERT_FALSE(p.reasons.empty());
  EXPECT_NE(p.reasons[0].find("light-match"), std::string::npos) << p.reasons[0];
  EXPECT_FALSE(p.timed_initial_literals);
}

// Robots moving: in transit they are nowhere, and nobody needs that
const char * kRobots =
  "(define (domain robots) (:requirements :typing :durative-actions)"
  " (:types robot room)"
  " (:predicates (at ?r - robot ?l - room) (connected ?a ?b - room) (cleaned ?l - room))"
  " (:durative-action move :parameters (?r - robot ?a ?b - room) :duration (= ?duration 5)"
  "  :condition (and (at start (at ?r ?a)) (over all (connected ?a ?b)))"
  "  :effect (and (at start (not (at ?r ?a))) (at end (at ?r ?b))))"
  " (:durative-action clean :parameters (?r - robot ?l - room) :duration (= ?duration 3)"
  "  :condition (over all (at ?r ?l)) :effect (at end (cleaned ?l))))";

const char * kRobotsProblem =
  "(define (problem p) (:domain robots) (:objects r1 r2 - robot a b c - room)"
  " (:init (at r1 a) (at r2 b) (connected a b) (connected b c))"
  " (:goal (and (cleaned c) (cleaned a))))";

TEST(TemporalProfileTest, MovingRobotsAreSimple)
{
  auto p = profile(kRobots, kRobotsProblem);
  EXPECT_TRUE(p.durative);
  EXPECT_TRUE(p.simple) << (p.reasons.empty() ? "" : p.reasons[0]);
}

TEST(TemporalProfileTest, NeedingTheIntermediateValueIsNotSimple)
{
  // A photo of the robot in transit: only possible while it moves
  const char * domain =
    "(define (domain robots) (:requirements :typing :durative-actions)"
    " (:types robot room)"
    " (:predicates (at ?r - robot ?l - room) (in-transit ?r - robot) (photo ?r - robot))"
    " (:durative-action move :parameters (?r - robot ?a ?b - room) :duration (= ?duration 5)"
    "  :condition (at start (at ?r ?a))"
    "  :effect (and (at start (not (at ?r ?a))) (at start (in-transit ?r))"
    "   (at end (not (in-transit ?r))) (at end (at ?r ?b))))"
    " (:durative-action shoot :parameters (?r - robot) :duration (= ?duration 1)"
    "  :condition (over all (in-transit ?r)) :effect (at end (photo ?r))))";
  auto p = profile(domain,
      "(define (problem p) (:domain robots) (:objects r1 - robot a b - room) (:init (at r1 a))"
      " (:goal (photo r1)))");
  EXPECT_FALSE(p.simple);
  // Needing the robot not to be in a room: before or after the move does it
  std::string other = kRobots;
  other.insert(other.size() - 1,
    " (:durative-action wave :parameters (?r - robot ?l - room) :duration (= ?duration 1)"
    "  :condition (at start (not (at ?r ?l))) :effect (at end (cleaned ?l)))");
  auto q = profile(other, kRobotsProblem);
  EXPECT_TRUE(q.simple) << (q.reasons.empty() ? "" : q.reasons[0]);
}

TEST(TemporalProfileTest, AValueThatHeldBeforeTheActionIsNothingNew)
{
  // Starts by adding what its start condition already needs, deletes it at end (PlanSys2's
  // domain_1_ok moves a person "from r1 to r2" like that, also with r1 == r2)
  const char * domain =
    "(define (domain d) (:requirements :typing :durative-actions) (:types p r)"
    " (:predicates (at ?p - p ?r - r) (seen ?p - p))"
    " (:durative-action stay :parameters (?p - p ?r - r) :duration (= ?duration 5)"
    "  :condition (at start (at ?p ?r))"
    "  :effect (and (at start (at ?p ?r)) (at end (not (at ?p ?r)))))"
    " (:durative-action look :parameters (?p - p ?r - r) :duration (= ?duration 1)"
    "  :condition (at start (at ?p ?r)) :effect (at end (seen ?p))))";
  auto p = profile(domain,
      "(define (problem x) (:domain d) (:objects j - p a - r) (:init (at j a)) (:goal (seen j)))");
  EXPECT_TRUE(p.simple) << (p.reasons.empty() ? "" : p.reasons[0]);
}

TEST(TemporalProfileTest, OnlyOtherActionsCount)
{
  // Each move alone reads its intermediate values: no other action needs to overlap it
  const char * domain =
    "(define (domain d) (:requirements :typing :durative-actions :numeric-fluents)"
    " (:types robot room)"
    " (:predicates (at ?r - robot ?l - room) (moving ?r - robot ?a ?b - room) (seen ?r - robot))"
    " (:functions (traffic ?a ?b - room))"
    " (:durative-action move :parameters (?r - robot ?a ?b - room) :duration (= ?duration 5)"
    "  :condition (and (at start (at ?r ?a)) (over all (moving ?r ?a ?b))"
    "   (at end (>= (traffic ?a ?b) 1)))"
    "  :effect (and (at start (not (at ?r ?a))) (at start (moving ?r ?a ?b))"
    "   (at start (increase (traffic ?a ?b) 1)) (at end (decrease (traffic ?a ?b) 1))"
    "   (at end (not (moving ?r ?a ?b))) (at end (at ?r ?b)))))";
  const char * one_robot =
    "(define (problem p) (:domain d) (:objects r1 - robot a b - room)"
    " (:init (at r1 a) (= (traffic a b) 0) (= (traffic b a) 0) (= (traffic a a) 0)"
    " (= (traffic b b) 0)) (:goal (at r1 b)))";
  auto p = profile(domain, one_robot);
  EXPECT_TRUE(p.simple) << (p.reasons.empty() ? "" : p.reasons[0]);

  // Two robots: the move of the other one reads the same traffic
  auto q = profile(domain,
      "(define (problem p) (:domain d) (:objects r1 r2 - robot a b - room)"
      " (:init (at r1 a) (at r2 a) (= (traffic a b) 0) (= (traffic b a) 0) (= (traffic a a) 0)"
      " (= (traffic b b) 0)) (:goal (at r1 b)))");
  EXPECT_FALSE(q.simple);
  ASSERT_FALSE(q.reasons.empty());
  EXPECT_NE(q.reasons[0].find("numeric"), std::string::npos) << q.reasons[0];

  // Another action needs the robot moving
  std::string spy = domain;
  spy.insert(spy.size() - 1,
    " (:durative-action spy :parameters (?r - robot ?a ?b - room) :duration (= ?duration 1)"
    "  :condition (over all (moving ?r ?a ?b)) :effect (at end (seen ?r)))");
  auto s = profile(spy, one_robot);
  EXPECT_FALSE(s.simple);
  ASSERT_FALSE(s.reasons.empty());
  EXPECT_NE(s.reasons[0].find("only while it runs"), std::string::npos) << s.reasons[0];
}

TEST(TemporalProfileTest, ConsumableResourcesOfTheExamples)
{
  // Stock carried from zone to zone (also to the same zone) and battery: only "at least" reads,
  // so no plan needs overlapping actions and M2 plans them
  const std::string dir = KAIROS_EXAMPLES_DIR;
  for (const char * problem : {"cooking_problem_simple.pddl", "cooking_problem_long.pddl"}) {
    auto p = profile(
      read_file(dir + "/cooking_domain.pddl"), read_file(dir + "/" + problem));
    EXPECT_TRUE(p.simple) << problem << ": " << (p.reasons.empty() ? "" : p.reasons[0]);
  }
  auto c = profile(read_file(dir + "/cooking_cost_domain.pddl"),
      read_file(dir + "/cooking_cost_problem_long.pddl"));
  EXPECT_TRUE(c.simple) << (c.reasons.empty() ? "" : c.reasons[0]);
}

TEST(TemporalProfileTest, LinearInTheDurativeActions)
{
  // 8 robots in 40 fully connected rooms: 12480 moves, each read by many others (it used to
  // compare every pair of actions)
  std::string problem = "(define (problem big) (:domain robots) (:objects";
  for (int r = 0; r < 8; ++r) {
    problem += " r" + std::to_string(r);
  }
  problem += " - robot";
  for (int l = 0; l < 40; ++l) {
    problem += " l" + std::to_string(l);
  }
  problem += " - room) (:init";
  for (int r = 0; r < 8; ++r) {
    problem += " (at r" + std::to_string(r) + " l" + std::to_string(r) + ")";
  }
  for (int a = 0; a < 40; ++a) {
    for (int b = 0; b < 40; ++b) {
      if (a != b) {
        problem += " (connected l" + std::to_string(a) + " l" + std::to_string(b) + ")";
      }
    }
  }
  problem += ") (:goal (cleaned l39)))";
  auto p = pipeline(kRobots, problem);
  ASSERT_TRUE(p->ok);
  ASSERT_GT(p->sas.durative_actions.size(), 12000u);
  const auto start = std::chrono::steady_clock::now();
  auto profile = analyze(p->sas);
  const auto ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - start).count();
  EXPECT_TRUE(profile.simple);
  EXPECT_LT(ms, 1000.0) << ms;  // generous: debug, sanitizer and coverage builds
}

TEST(TemporalProfileTest, ABorrowedResource)
{
  // A tool taken at start and given back at end: while it is out there are fewer, which helps
  // nobody that needs one ("at least one"), so no plan needs the actions to overlap
  const char * domain =
    "(define (domain tools) (:requirements :fluents :durative-actions)"
    " (:predicates (done-a) (done-b)) (:functions (tools))"
    " (:durative-action a :parameters () :duration (= ?duration 4)"
    "  :condition (at start (>= (tools) 1))"
    "  :effect (and (at start (decrease (tools) 1)) (at end (increase (tools) 1))"
    "   (at end (done-a))))"
    " (:durative-action b :parameters () :duration (= ?duration 2)"
    "  :condition (at start (>= (tools) 1)) :effect (at end (done-b))))";
  const char * problem =
    "(define (problem p) (:domain tools) (:init (= (tools) 1)) (:goal (and (done-a) (done-b))))";
  auto p = profile(domain, problem);
  EXPECT_TRUE(p.simple) << (p.reasons.empty() ? "" : p.reasons[0]);

  // Needing the tool out ("at most 0"): only while a runs
  std::string missing = domain;
  missing.replace(missing.rfind("(>= (tools) 1)"), 14, "(<= (tools) 0)");
  auto q = profile(missing, problem);
  EXPECT_FALSE(q.simple);
  ASSERT_FALSE(q.reasons.empty());
  EXPECT_NE(q.reasons[0].find("numeric"), std::string::npos) << q.reasons[0];

  // Read with an equality, or by an effect: anything may need the intermediate value
  std::string equal = domain;
  equal.replace(equal.rfind("(>= (tools) 1)"), 14, "(= (tools) 1)");
  EXPECT_FALSE(profile(equal, problem).simple);
  std::string effect = domain;
  effect.replace(effect.rfind("(at end (done-b))"), 17,
    "(and (at end (done-b)) (at end (assign (score) (tools))))");
  effect.replace(effect.find("(:functions (tools))"), 20, "(:functions (tools) (score))");
  EXPECT_FALSE(profile(effect, problem).simple);

  // Raising at start, read as "at most": symmetric
  std::string raised = domain;
  raised.replace(raised.find("(at start (decrease (tools) 1))"), 31,
    "(at start (increase (tools) 1))");
  raised.replace(raised.find("(at end (increase (tools) 1))"), 29, "(at end (decrease (tools) 1))");
  raised.replace(raised.rfind("(>= (tools) 1)"), 14, "(<= (tools) 1)");
  auto r = profile(raised, problem);
  EXPECT_TRUE(r.simple) << (r.reasons.empty() ? "" : r.reasons[0]);

  // Nobody else reads it: compressing loses nothing
  const char * alone =
    "(define (domain tools) (:requirements :fluents :durative-actions)"
    " (:predicates (done-a)) (:functions (tools))"
    " (:durative-action a :parameters () :duration (= ?duration 4)"
    "  :condition (at start (>= (tools) 1))"
    "  :effect (and (at start (decrease (tools) 1)) (at end (increase (tools) 1))"
    "   (at end (done-a)))))";
  auto a = profile(alone,
      "(define (problem p) (:domain tools) (:init (= (tools) 1)) (:goal (done-a)))");
  EXPECT_TRUE(a.simple) << (a.reasons.empty() ? "" : a.reasons[0]);
}

TEST(TemporalProfileTest, TimedInitialLiteralsAndDurations)
{
  const char * domain =
    "(define (domain d) (:requirements :fluents :durative-actions :timed-initial-literals"
    "  :duration-inequalities)"
    " (:predicates (open) (done)) (:functions (speed) (dist))"
    " (:durative-action go :parameters () :duration (and (>= ?duration 1) (<= ?duration 4))"
    "  :condition (over all (open))"
    "  :effect (at end (done)))"
    " (:durative-action drive :parameters () :duration (= ?duration (/ (dist) (speed)))"
    "  :condition () :effect (at end (increase (dist) (* ?duration 2)))))";
  auto p = profile(domain,
      "(define (problem p) (:domain d) (:init (= (speed) 2) (= (dist) 4) (at 10 (open)))"
      " (:goal (done)))");
  EXPECT_TRUE(p.timed_initial_literals);
  EXPECT_FALSE(p.simple);
  EXPECT_TRUE(p.variable_duration);
  EXPECT_TRUE(p.duration_in_effects);
}

TEST(TemporalProfileTest, NoDurativeActions)
{
  auto p = profile(
    "(define (domain d) (:predicates (p)) (:action a :parameters () :effect (p)))",
    "(define (problem x) (:domain d) (:init) (:goal (p)))");
  EXPECT_FALSE(p.durative);
  EXPECT_TRUE(p.simple);
}

// PlanSys2's temporal examples
TEST(TemporalProfileTest, PlanSys2Examples)
{
  struct Case
  {
    const char * domain;
    const char * problem;
  };
  const Case cases[] = {
    {"plansys2_simple_example__simple_example", "plansys2_simple_example__problem"},
    {"plansys2_executor__domain_simple", "plansys2_executor__problem_simple_1"},
    {"plansys2_executor__factory2", "plansys2_executor__problem_factory_1"},
    {"plansys2_domain_expert__domain_charging", "plansys2_domain_expert__problem_charging"},
    {"plansys2_executor__elevator_domain", "plansys2_executor__elevator_problem"},
    {"plansys2_executor__road_trip_domain", "plansys2_executor__road_trip_problem"},
  };
  for (const auto & c : cases) {
    auto p = profile(read_file(data_path(std::string("plansys2/") + c.domain + ".pddl")),
        read_file(data_path(std::string("plansys2/") + c.problem + ".pddl")));
    EXPECT_TRUE(p.durative) << c.problem;
    // None needs actions to overlap: M2 can plan them all
    EXPECT_TRUE(p.simple) << c.problem << ": " << (p.reasons.empty() ? "" : p.reasons[0]);
  }
}

TEST(TemporalProfileTest, IpcDomainsThatNeedConcurrency)
{
  const char * dir = std::getenv("KAIROS_PDDL_CORPUS");
  if (dir == nullptr) {
    GTEST_SKIP() << "KAIROS_PDDL_CORPUS not set";
  }
  std::filesystem::path base =
    std::filesystem::path(dir) / "potassco_pddl-instances" / "ipc-2011" / "domains";
  for (const char * d : {"match-cellar-temporal-satisficing", "turn-and-open-temporal-satisficing",
      "temporal-machine-shop-temporal-satisficing"})
  {
    auto folder = base / d;
    auto domain = folder / "domain.pddl";
    if (!std::filesystem::exists(domain)) {
      domain = folder / "domains" / "domain-1.pddl";
    }
    auto p = profile(read_file(domain), read_file(folder / "instances" / "instance-1.pddl"));
    EXPECT_FALSE(p.simple) << d;
  }
  // The rest of the 2011 temporal track is temporally simple (Cushing et al.)
  for (const char * d : {"crew-planning-temporal-satisficing", "elevator-temporal-satisficing",
      "floor-tile-temporal-satisficing", "openstacks-temporal-satisficing",
      "parc-printer-temporal-satisficing", "parking-temporal-satisficing",
      "peg-solitaire-temporal-satisficing", "sokoban-temporal-satisficing",
      "storage-temporal-satisficing"})
  {
    auto folder = base / d;
    auto domain = folder / "domain.pddl";
    if (!std::filesystem::exists(domain)) {
      domain = folder / "domains" / "domain-1.pddl";
    }
    auto p = profile(read_file(domain), read_file(folder / "instances" / "instance-1.pddl"));
    EXPECT_TRUE(p.simple) << d << ": " << (p.reasons.empty() ? "" : p.reasons[0]);
  }
}

}  // namespace
}  // namespace kairos::temporal
