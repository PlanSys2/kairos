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
#include <set>
#include <string>
#include <vector>

#include "kairos/kairos.hpp"

namespace kairos
{
namespace
{

// Links only against libkairos: the public API end to end

const char * kDomain =
  "(define (domain robots)\n"
  " (:requirements :strips :typing :durative-actions)\n"
  " (:types robot room)\n"
  " (:predicates (at ?r - robot ?l - room))\n"
  " (:durative-action move :parameters (?r - robot ?a ?b - room)\n"
  "  :duration (= ?duration 5)\n"
  "  :condition (at start (at ?r ?a))\n"
  "  :effect (and (at start (not (at ?r ?a))) (at end (at ?r ?b)))))\n";

const char * kProblem =
  "(define (problem p) (:domain robots)\n"
  " (:objects r1 - robot kitchen hall - room)\n"
  " (:init (at r1 kitchen))\n"
  " (:goal (at r1 hall)))\n";

bool has(const std::vector<std::string> & v, const std::string & s)
{
  return std::find(v.begin(), v.end(), s) != v.end();
}

TEST(CheckApiTest, ValidDomain)
{
  CheckReport r = check_domain(kDomain);
  EXPECT_TRUE(r.valid);
  EXPECT_TRUE(r.plannable);  // durative actions: engine M2
  EXPECT_TRUE(r.diagnostics.empty());
  EXPECT_TRUE(has(r.features, "durative-actions"));
  EXPECT_TRUE(has(r.features, "typing"));
}

// PDDL+: a process fills, an action stops when full
const char * kProcess =
  "(define (domain flow) (:requirements :fluents :time) (:predicates (on) (done))"
  " (:functions (level))"
  " (:process fill :parameters () :precondition (on) :effect (increase (level) (* #t 2)))"
  " (:action stop :parameters () :precondition (>= (level) 10) :effect (done)))\n";

const char * kProcessProblem =
  "(define (problem x) (:domain flow) (:init (on) (= (level) 0)) (:goal (done)))\n";

TEST(CheckApiTest, ProcessesArePlannable)
{
  CheckReport r = check_domain(kProcess);
  EXPECT_TRUE(r.valid);
  EXPECT_TRUE(r.plannable);
  for (const auto & d : r.diagnostics) {
    EXPECT_NE(d.code.rfind("unsupported.", 0), 0u) << d.message;
  }
  CheckReport p = check_problem(kProcessProblem, kProcess, "problem.pddl");
  EXPECT_TRUE(p.valid);
  EXPECT_TRUE(p.plannable);
}

TEST(CheckApiTest, ClassicalDomainIsPlannable)
{
  const char * domain =
    "(define (domain d) (:requirements :strips) (:predicates (p) (q))\n"
    " (:action a :parameters () :precondition (p) :effect (q)))";
  const char * problem = "(define (problem x) (:domain d) (:init (p)) (:goal (q)))";
  CheckReport rd = check_domain(domain);
  EXPECT_TRUE(rd.valid);
  EXPECT_TRUE(rd.plannable);
  EXPECT_TRUE(rd.diagnostics.empty());
  CheckReport rp = check_problem(problem, domain, "problem.pddl");
  EXPECT_TRUE(rp.plannable);
  // Without its domain the problem cannot be said plannable, and the report says so
  CheckReport alone = check_problem(problem);
  EXPECT_FALSE(alone.plannable);
  EXPECT_FALSE(alone.plannable_known);
  EXPECT_TRUE(rp.plannable_known);
  // Invalid input is never plannable
  EXPECT_FALSE(check_domain("(define (domain d)").plannable);
}

TEST(CheckApiTest, SupportOfEveryFeature)
{
  auto table = pddl_support();
  ASSERT_FALSE(table.empty());
  std::set<std::string> names;
  for (const auto & f : table) {
    EXPECT_TRUE(names.insert(f.feature).second) << "twice: " << f.feature;
    // What is not full says why, or when it comes
    EXPECT_EQ(f.level == SupportLevel::Full || !f.note.empty(), true) << f.feature;
  }
  for (const char * n : {"durative-actions", "derived-predicates", "timed-initial-literals",
      "processes", "preferences", "continuous-effects"})
  {
    EXPECT_TRUE(names.count(n)) << n;
  }
  auto level = [&](const std::string & n) {
      for (const auto & f : table) {
        if (f.feature == n) {
          return f.level;
        }
      }
      return SupportLevel::Full;
    };
  EXPECT_EQ(level("durative-actions"), SupportLevel::Full);
  EXPECT_EQ(level("continuous-effects"), SupportLevel::Partial);
  EXPECT_EQ(level("processes"), SupportLevel::Partial);
  EXPECT_EQ(level("events"), SupportLevel::Partial);
  EXPECT_EQ(level("global-constraints"), SupportLevel::Full);
  EXPECT_EQ(level("preferences"), SupportLevel::Full);
  EXPECT_EQ(level("trajectory-constraints"), SupportLevel::Partial);
  EXPECT_EQ(to_string(SupportLevel::Partial), "partial");
}

TEST(CheckApiTest, TheReportGivesTheSupportOfWhatTheDomainUses)
{
  // A process: partial
  CheckReport r = check_domain(kProcess);
  bool process = false;
  for (const auto & f : r.support) {
    process = process || (f.feature == "processes" && f.level == SupportLevel::Partial);
    // Only features it uses
    EXPECT_NE(std::find(r.features.begin(), r.features.end(), f.feature), r.features.end());
  }
  EXPECT_TRUE(process);
  // Every feature that is none is unsupported: a warning in the report, the error of solve
  CheckReport classical = check_domain(
    "(define (domain d) (:requirements :strips) (:predicates (p)) (:action a :effect (p)))");
  for (const auto & f : classical.support) {
    EXPECT_NE(f.level, SupportLevel::None) << f.feature;
  }
}

TEST(CheckApiTest, DomainWithSyntaxError)
{
  CheckReport r = check_domain("(define (domain d) (:predicates (p))", "broken.pddl");
  EXPECT_FALSE(r.valid);
  ASSERT_FALSE(r.diagnostics.empty());
  EXPECT_EQ(r.diagnostics[0].code, "parse.unexpected_end");
  EXPECT_EQ(r.diagnostics[0].location->file, "broken.pddl");
  EXPECT_TRUE(r.features.empty());
}

TEST(CheckApiTest, DomainWithSemanticError)
{
  CheckReport r = check_domain(
    "(define (domain d) (:predicates (at ?x)) (:action a :parameters (?x) :effect (at ?x ?x)))");
  EXPECT_FALSE(r.valid);
  EXPECT_EQ(r.diagnostics.at(0).code, "semantic.wrong_arity");
}

TEST(CheckApiTest, ProblemAgainstItsDomain)
{
  CheckReport r = check_problem(kProblem, kDomain, "problem.pddl");
  EXPECT_TRUE(r.valid);
  EXPECT_TRUE(r.plannable);
  EXPECT_TRUE(r.diagnostics.empty());
  EXPECT_TRUE(has(r.features, "durative-actions"));
}

TEST(CheckApiTest, ProblemErrorsPointAtTheProblemFile)
{
  CheckReport r = check_problem(
    "(define (problem p) (:domain robots) (:objects r1 - robot k - room) (:init (at k r1)))",
    kDomain, "my_problem.pddl", "my_domain.pddl");
  EXPECT_FALSE(r.valid);
  ASSERT_GE(r.diagnostics.size(), 2u);
  EXPECT_EQ(r.diagnostics[0].location->file, "my_problem.pddl");
  // Formatted with the source line and a caret
  std::string text = to_string(r.diagnostics[0],
      "(define (problem p) (:domain robots) (:objects r1 - robot k - room) (:init (at k r1)))");
  EXPECT_NE(text.find("^"), std::string::npos) << text;
}

TEST(CheckApiTest, BrokenDomainMakesTheProblemCheckFail)
{
  CheckReport r = check_problem(kProblem, "(define (domain robots) (:predicates (at ?x ?y ?z)) "
      "(:action a :effect (at)))", "problem.pddl");
  EXPECT_FALSE(r.valid);
  EXPECT_EQ(r.diagnostics.at(0).code, "semantic.wrong_arity");
}

TEST(CheckApiTest, ProblemAlone)
{
  CheckReport r = check_problem(
    "(define (problem p) (:domain robots) (:objects r1) (:init (at r1 dock)))");
  EXPECT_TRUE(r.valid);  // 'dock' might be a domain constant: only a warning
  ASSERT_EQ(r.diagnostics.size(), 1u);
  EXPECT_EQ(r.diagnostics[0].code, "semantic.unknown_symbol_without_domain");
  EXPECT_EQ(r.diagnostics[0].severity, Severity::Warning);

  CheckReport broken = check_problem("(define (problem p) (:objects a a))");
  EXPECT_FALSE(broken.valid);
}

TEST(CheckApiTest, SyntaxErrorsInBothFiles)
{
  CheckReport r = check_problem("(define (problem p)", "(define (domain d)", "p.pddl", "d.pddl");
  EXPECT_FALSE(r.valid);
  ASSERT_GE(r.diagnostics.size(), 2u);
  bool domain_error = false;
  bool problem_error = false;
  for (const auto & d : r.diagnostics) {
    domain_error = domain_error || (d.location && d.location->file == "d.pddl");
    problem_error = problem_error || (d.location && d.location->file == "p.pddl");
  }
  EXPECT_TRUE(domain_error);
  EXPECT_TRUE(problem_error);
}

}  // namespace
}  // namespace kairos
