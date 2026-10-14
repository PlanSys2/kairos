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
#include <string_view>
#include <vector>

#include "pddl/parser.hpp"
#include "semantic/analyzer.hpp"
#include "support/pddl_helpers.hpp"

namespace kairos::sem
{
namespace
{

using kairos::testing::codes;
using kairos::testing::dump;

const char * kDomain =
  "(define (domain robots)\n"
  " (:requirements :strips :typing :fluents :timed-initial-literals :preferences\n"
  "                :constraints :object-fluents :derived-predicates)\n"
  " (:types robot room - object charger - room)\n"
  " (:constants dock - charger)\n"
  " (:predicates (at ?r - robot ?l - room) (connected ?a ?b - room) (busy ?r - robot)\n"
  "              (near ?r - robot ?l - room))\n"
  " (:functions (battery ?r - robot) - number (holder) - robot)\n"
  " (:derived (near ?r ?l) (at ?r ?l))\n"
  " (:action move :parameters (?r - robot ?a ?b - room)\n"
  "  :precondition (at ?r ?a) :effect (and (not (at ?r ?a)) (at ?r ?b))))\n";

struct Analysis
{
  bool ok = false;
  std::vector<Diagnostic> diagnostics;
  std::optional<ProblemModel> model;
};

Analysis analyze(std::string_view problem_text, std::string_view domain_text = kDomain)
{
  Analysis a;
  auto d = pddl::parse_domain(domain_text, "d.pddl");
  EXPECT_TRUE(d.ok()) << dump(d.diagnostics());
  auto dm = analyze_domain(*d);
  EXPECT_TRUE(dm.ok()) << dump(dm.diagnostics());
  auto p = pddl::parse_problem(problem_text, "p.pddl");
  EXPECT_TRUE(p.ok()) << dump(p.diagnostics());
  auto r = analyze_problem(*p, *d, *dm);
  a.ok = r.ok();
  a.diagnostics = r.diagnostics();
  if (r.ok()) {
    a.model = std::move(r).value();
  }
  return a;
}

std::string problem(const std::string & body)
{
  return "(define (problem p) (:domain robots)\n (:objects r1 r2 - robot kitchen hall - room)\n" +
         body + ")";
}

std::vector<std::string> errors(const std::string & body)
{
  Analysis a = analyze(problem(body));
  std::vector<Diagnostic> errs;
  for (const auto & d : a.diagnostics) {
    if (d.severity != Severity::Warning) {
      errs.push_back(d);
    }
  }
  return codes(errs);
}

bool has_code(const std::vector<Diagnostic> & ds, const std::string & code)
{
  return std::any_of(ds.begin(), ds.end(), [&](const auto & d) {return d.code == code;});
}

std::string message(const std::vector<Diagnostic> & ds, const std::string & code)
{
  for (const auto & d : ds) {
    if (d.code == code) {
      return d.message;
    }
  }
  return "<no " + code + ">";
}

// --- Against the domain ----------------------------------------------------------------------

TEST(ProblemAnalysisTest, ValidProblem)
{
  Analysis a = analyze(problem(
      " (:init (at r1 kitchen) (connected kitchen hall) (= (battery r1) 50) (= (holder) r1)\n"
      "        (at 10 (busy r2)))\n"
      " (:goal (and (at r1 hall) (preference p1 (at r2 dock)) (near r1 hall)))\n"
      " (:metric minimize (+ (total-time) (is-violated p1)))"));
  ASSERT_TRUE(a.ok) << dump(a.diagnostics);
  EXPECT_TRUE(a.diagnostics.empty()) << dump(a.diagnostics);
  const ProblemModel & m = *a.model;
  EXPECT_EQ(m.objects.items.size(), 5u);  // dock (constant) + 4 objects
  EXPECT_TRUE(m.objects.items[*m.objects.find("dock")].constant);
  EXPECT_TRUE(m.features.contains(Feature::TimedInitialLiterals));
  EXPECT_TRUE(m.features.contains(Feature::Metric));
  EXPECT_TRUE(m.features.contains(Feature::MetricTotalTime));
  EXPECT_TRUE(m.features.contains(Feature::Preferences));
  EXPECT_TRUE(m.features.contains(Feature::DerivedPredicates));  // from the domain
}

TEST(ProblemAnalysisTest, DomainNameMismatchIsAWarning)
{
  // PlanSys2 validates every domain with "(:domain plansys2)"
  Analysis a = analyze("(define (problem void) (:domain plansys2))");
  ASSERT_TRUE(a.ok);
  EXPECT_EQ(codes(a.diagnostics), (std::vector<std::string>{"semantic.domain_name_mismatch@1:1"}));
  Analysis missing = analyze("(define (problem p))");
  EXPECT_TRUE(has_code(missing.diagnostics, "semantic.missing_domain_name"));
}

TEST(ProblemAnalysisTest, Objects)
{
  EXPECT_EQ(errors("(:objects x - spaceship)").at(0), "semantic.unknown_type@3:15");
  // Redeclaring a constant with its own type: a warning
  Analysis same = analyze(problem("(:objects dock - charger)"));
  ASSERT_TRUE(same.ok);
  EXPECT_EQ(codes(same.diagnostics), (std::vector<std::string>{
        "semantic.redeclared_constant@3:11"}));
  // An object with several types belongs to all of them (IPC elevator, TMS)
  Analysis several = analyze(problem("(:objects r1 - room) (:init (at r1 r1))"));
  ASSERT_TRUE(several.ok) << dump(several.diagnostics);
  EXPECT_TRUE(has_code(several.diagnostics, "semantic.object_with_several_types"));
}

TEST(ProblemAnalysisTest, InitFacts)
{
  EXPECT_EQ(errors("(:init (at kitchen r1))"), (std::vector<std::string>{
        "semantic.type_mismatch@3:12", "semantic.type_mismatch@3:20",
        "semantic.swapped_arguments@3:8"}));
  EXPECT_EQ(errors("(:init (at r1))"), (std::vector<std::string>{"semantic.wrong_arity@3:8"}));
  EXPECT_EQ(errors("(:init (at r1 garage))"),
    (std::vector<std::string>{"semantic.unknown_object@3:15"}));
  EXPECT_EQ(errors("(:init (near r1 kitchen))"),
    (std::vector<std::string>{"semantic.derived_in_init@3:8"}));
  EXPECT_EQ(errors("(:init (at ?x kitchen))"), (std::vector<std::string>{
        "semantic.init_not_ground@3:12", "semantic.unbound_variable@3:12"}));
  Analysis typo = analyze(problem("(:init (conected kitchen hall))"));
  EXPECT_EQ(message(typo.diagnostics, "semantic.unknown_predicate"),
    "in the initial state: unknown predicate 'conected'; the fact is ignored "
    "(did you mean 'connected'?)");
  EXPECT_EQ(typo.diagnostics.at(0).severity, Severity::Warning);
}

TEST(ProblemAnalysisTest, InitValues)
{
  EXPECT_EQ(errors("(:init (= (battery r1) kitchen))"),
    (std::vector<std::string>{"semantic.init_value@3:8"}));
  EXPECT_EQ(errors("(:init (= (holder) 3))"),
    (std::vector<std::string>{"semantic.init_value@3:8"}));
  EXPECT_EQ(errors("(:init (= (holder) kitchen))"),
    (std::vector<std::string>{"semantic.type_mismatch@3:8"}));
  EXPECT_TRUE(errors("(:init (= (holder) undefined))").empty());
  // Two values: warning, the last one is used (IPC 2006 pathways)
  Analysis twice = analyze(problem("(:init (= (battery r1) 1) (= (battery r1) 2))"));
  ASSERT_TRUE(twice.ok);
  EXPECT_TRUE(has_code(twice.diagnostics, "semantic.conflicting_init"));
  Analysis same = analyze(problem("(:init (= (battery r1) 1) (= (battery r1) 1))"));
  EXPECT_TRUE(has_code(same.diagnostics, "semantic.duplicate_init"));
  // A function the domain does not know: its value is ignored (IPC 2023 markettrader)
  Analysis unknown = analyze(problem("(:init (= (fuel r1) 3))"));
  ASSERT_TRUE(unknown.ok);
  EXPECT_TRUE(has_code(unknown.diagnostics, "semantic.unknown_function"));
}

TEST(ProblemAnalysisTest, TimedInitialLiterals)
{
  EXPECT_EQ(errors("(:init (at -1 (busy r1)))"),
    (std::vector<std::string>{"semantic.negative_time@3:8"}));
  Analysis negative = analyze(problem("(:init (not (busy r1)))"));
  ASSERT_TRUE(negative.ok);
  EXPECT_TRUE(has_code(negative.diagnostics, "semantic.negative_init"));
  Analysis timed_negative = analyze(problem("(:init (at 5 (not (busy r1))))"));
  ASSERT_TRUE(timed_negative.ok);
  EXPECT_FALSE(has_code(timed_negative.diagnostics, "semantic.negative_init"));
  Analysis fluent = analyze(problem("(:init (at 5 (= (battery r1) 3)))"));
  EXPECT_TRUE(fluent.model->features.contains(Feature::TimedInitialFluents));
}

TEST(ProblemAnalysisTest, GoalAndMetric)
{
  EXPECT_EQ(errors("(:goal (at r1 ?x))"),
    (std::vector<std::string>{"semantic.unbound_variable@3:15"}));
  EXPECT_TRUE(errors("(:goal (exists (?x - room) (at r1 ?x)))").empty());
  EXPECT_EQ(errors("(:goal (at start (at r1 hall)))"),
    (std::vector<std::string>{"semantic.misplaced_timed@3:8"}));
  EXPECT_EQ(errors("(:goal (at r1 hall)) (:metric minimize (is-violated nope))"),
    (std::vector<std::string>{"semantic.unknown_preference@3:40"}));
  EXPECT_EQ(errors("(:metric minimize (holder))"),
    (std::vector<std::string>{"semantic.type_mismatch@3:2"}));
  Analysis fluents = analyze(problem("(:metric maximize (battery r1))"));
  EXPECT_TRUE(fluents.model->features.contains(Feature::MetricFluents));
  EXPECT_FALSE(fluents.model->features.contains(Feature::MetricTotalTime));
}

TEST(ProblemAnalysisTest, ConstraintsInTheProblem)
{
  EXPECT_TRUE(errors("(:constraints (always (at r1 kitchen)))").empty());
  EXPECT_EQ(errors("(:constraints (always (at r1 basement)))"),
    (std::vector<std::string>{"semantic.unknown_object@3:30"}));
}

TEST(ProblemAnalysisTest, ProblemRequirementsCoverProblemFeatures)
{
  // TILs used, but only the domain could declare them: here neither does
  const char * domain = "(define (domain d) (:predicates (p)))";
  Analysis a = analyze("(define (problem p) (:domain d) (:init (at 3 (p))))", domain);
  ASSERT_TRUE(a.ok);
  EXPECT_TRUE(has_code(a.diagnostics, "semantic.undeclared_requirement"));
  Analysis declared = analyze(
    "(define (problem p) (:domain d) (:requirements :timed-initial-literals) (:init (at 3 (p))))",
    domain);
  EXPECT_FALSE(has_code(declared.diagnostics, "semantic.undeclared_requirement"));
}

TEST(ProblemAnalysisTest, LargeProblemIsFast)
{
  std::string objects = "(:objects";
  std::string init = "(:init";
  for (int i = 0; i < 20000; ++i) {
    objects += " w" + std::to_string(i);
    init += " (connected w" + std::to_string(i) + " w" + std::to_string((i + 1) % 20000) + ")";
  }
  objects += " - room)";
  init += ")";
  Analysis a = analyze("(define (problem big) (:domain robots) " + objects + init + ")");
  ASSERT_TRUE(a.ok) << dump(a.diagnostics);
  EXPECT_EQ(a.model->objects.items.size(), 20001u);
}

// --- Without the domain ----------------------------------------------------------------------

std::vector<Diagnostic> alone(std::string_view text)
{
  auto p = pddl::parse_problem(text, "p.pddl");
  EXPECT_TRUE(p.ok()) << dump(p.diagnostics());
  return check_problem_alone(*p);
}

TEST(ProblemAloneTest, UnknownNamesAreWarningsOnce)
{
  auto ds = alone("(define (problem p) (:domain d) (:objects a b)"
      " (:init (p a c) (q c)) (:goal (p a c)))");
  // 'c' may be a domain constant: one warning, not an error, and only the first time
  EXPECT_EQ(codes(ds), (std::vector<std::string>{
        "semantic.unknown_symbol_without_domain@1:60"}));
  EXPECT_FALSE(has_errors(ds));
}

TEST(ProblemAloneTest, Errors)
{
  EXPECT_EQ(codes(alone("(define (problem p) (:objects a a))")),
    (std::vector<std::string>{"semantic.duplicate_object@1:33"}));
  EXPECT_EQ(codes(alone("(define (problem p) (:objects a) (:init (p ?x)))")),
    (std::vector<std::string>{"semantic.init_not_ground@1:44"}));
  EXPECT_EQ(codes(alone("(define (problem p) (:objects a) (:init (at -2 (p a))))")),
    (std::vector<std::string>{"semantic.negative_time@1:41"}));
  EXPECT_EQ(codes(alone("(define (problem p) (:objects a) (:goal (p ?x)))")),
    (std::vector<std::string>{"semantic.unbound_variable@1:44"}));
  EXPECT_TRUE(alone("(define (problem p) (:objects a) (:goal (forall (?x) (p ?x))))").empty());
}

TEST(ProblemAloneTest, NegativeInitWarning)
{
  auto ds = alone("(define (problem p) (:objects a) (:init (not (p a))))");
  EXPECT_EQ(codes(ds), (std::vector<std::string>{"semantic.negative_init@1:41"}));
}

}  // namespace
}  // namespace kairos::sem
