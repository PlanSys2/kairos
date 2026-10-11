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
  // Correct PDDL, but durative actions cannot be planned yet: warnings only
  EXPECT_FALSE(r.plannable);
  ASSERT_FALSE(r.diagnostics.empty());
  for (const auto & d : r.diagnostics) {
    EXPECT_EQ(d.severity, Severity::Warning) << d.message;
  }
  EXPECT_TRUE(has(r.features, "durative-actions"));
  EXPECT_TRUE(has(r.features, "typing"));
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
  // Without its domain the problem cannot be said plannable
  EXPECT_FALSE(check_problem(problem).plannable);
  // Invalid input is never plannable
  EXPECT_FALSE(check_domain("(define (domain d)").plannable);
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
  EXPECT_FALSE(r.plannable);
  for (const auto & d : r.diagnostics) {
    EXPECT_EQ(d.severity, Severity::Warning) << d.message;
  }
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
