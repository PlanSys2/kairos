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
#include <string_view>
#include <vector>

#include "pddl/parser.hpp"
#include "pddl/printer.hpp"
#include "support/pddl_helpers.hpp"

namespace kairos::pddl
{
namespace
{

using kairos::testing::codes;
using kairos::testing::dump;

ast::Problem parse_ok(std::string_view text)
{
  auto r = parse_problem(text, "p.pddl");
  EXPECT_TRUE(r.ok()) << dump(r.diagnostics());
  if (!r.ok()) {
    return ast::Problem{};
  }
  return std::move(r).value();
}

std::vector<std::string> errors_of(std::string_view text)
{
  auto r = parse_problem(text, "p.pddl");
  EXPECT_FALSE(r.ok()) << "expected a parse failure";
  return codes(r.diagnostics());
}

std::string name(const ast::Problem & p, Symbol s) {return std::string(p.symbols.name(s));}

std::string problem(std::string_view sections)
{
  return "(define (problem p) (:domain d) " + std::string(sections) + ")";
}

TEST(ParserProblemTest, MinimalProblem)
{
  auto p = parse_ok("(define (problem void) (:domain plansys2))");
  EXPECT_EQ(name(p, p.name), "void");
  EXPECT_EQ(name(p, p.domain_name), "plansys2");
  EXPECT_TRUE(p.objects.empty());
  EXPECT_TRUE(p.init.empty());
  EXPECT_FALSE(p.goal.valid());
  EXPECT_FALSE(p.metric.has_value());
}

TEST(ParserProblemTest, DomainSectionIsOptionalForTheParser)
{
  // The semantic check, not the parser, decides whether the domain must be named
  auto p = parse_ok("(define (problem p))");
  EXPECT_FALSE(p.domain_name.valid());
}

TEST(ParserProblemTest, Objects)
{
  auto p = parse_ok(problem("(:objects r1 r2 - robot kitchen - room x)"));
  ASSERT_EQ(p.objects.size(), 4u);
  EXPECT_EQ(name(p, p.objects[1].types.at(0)), "robot");
  EXPECT_EQ(name(p, p.objects[2].types.at(0)), "room");
  EXPECT_TRUE(p.objects[3].types.empty());
}

TEST(ParserProblemTest, InitFactsAndFluents)
{
  auto p = parse_ok(problem(
      "(:init (at r1 kitchen) (busy) (= (battery r1) 80.5) (= energy 0) (= (holder g) r1)"
      " (not (open door)))"));
  ASSERT_EQ(p.init.size(), 6u);

  EXPECT_EQ(p.init[0].kind, ast::InitItem::Kind::Fact);
  EXPECT_EQ(name(p, p.init[0].name), "at");
  EXPECT_EQ(p.init[0].args.size(), 2u);
  EXPECT_FALSE(p.init[0].time.has_value());

  EXPECT_TRUE(p.init[1].args.empty());

  EXPECT_EQ(p.init[2].kind, ast::InitItem::Kind::Fluent);
  EXPECT_EQ(name(p, p.init[2].name), "battery");
  EXPECT_EQ(p.nodes.exprs[p.init[2].value].number, 80.5);

  EXPECT_EQ(name(p, p.init[3].name), "energy");  // bare zero-arity function
  EXPECT_TRUE(p.init[3].args.empty());

  EXPECT_EQ(p.nodes.exprs[p.init[4].value].kind, ast::ExprKind::Term);  // object fluent

  EXPECT_TRUE(p.init[5].negated);
}

TEST(ParserProblemTest, TimedInitialLiteralsAndFluents)
{
  auto p = parse_ok(problem(
      "(:init (at r1 kitchen) (at 10 (open door)) (at 20.5 (not (open door)))"
      " (at 30 (= (speed r1) 3)))"));
  ASSERT_EQ(p.init.size(), 4u);
  EXPECT_FALSE(p.init[0].time.has_value());  // "at" predicate, not a TIL
  EXPECT_EQ(name(p, p.init[0].name), "at");
  EXPECT_EQ(p.init[1].time, 10.0);
  EXPECT_EQ(name(p, p.init[1].name), "open");
  EXPECT_EQ(p.init[2].time, 20.5);
  EXPECT_TRUE(p.init[2].negated);
  EXPECT_EQ(p.init[3].time, 30.0);
  EXPECT_EQ(p.init[3].kind, ast::InitItem::Kind::Fluent);
  EXPECT_EQ(p.init[1].loc, (ast::Loc{1, 56}));
}

TEST(ParserProblemTest, NestedTimedLiteralsAreRejected)
{
  // (at 10 (at 20 (p))): the inner one is the predicate "at" with a number argument
  EXPECT_EQ(errors_of(problem("(:init (at 10 (at 20 (p))))")),
    (std::vector<std::string>{"parse.unexpected_token@1:51"}));
}

TEST(ParserProblemTest, GoalConstraintsMetric)
{
  auto p = parse_ok(problem(
      "(:goal (and (at r1 hall) (preference pg (at r2 kitchen))))"
      " (:constraints (sometime (busy)))"
      " (:metric maximize (- (reward) (* 0.5 (total-time)))))"));
  EXPECT_EQ(to_pddl(p.nodes, p.symbols, p.goal),
    "(and (at r1 hall) (preference pg (at r2 kitchen)))");
  EXPECT_EQ(to_pddl(p.nodes, p.symbols, p.constraints), "(sometime (busy))");
  ASSERT_TRUE(p.metric.has_value());
  EXPECT_FALSE(p.metric->minimize);
  EXPECT_EQ(to_pddl(p.nodes, p.symbols, p.metric->expr), "(- (reward) (* 0.5 (total-time)))");
}

TEST(ParserProblemTest, MetricForms)
{
  for (std::string_view metric : {"(:metric minimize (total-time))",
      "(:metric minimize total-time)",
      "(:metric MINIMIZE (total-cost))", "(:metric minimize (is-violated p))"})
  {
    auto p = parse_ok(problem(metric));
    ASSERT_TRUE(p.metric.has_value()) << metric;
    EXPECT_TRUE(p.metric->minimize);
  }
  EXPECT_EQ(errors_of(problem("(:metric reduce (total-time))")),
    (std::vector<std::string>{"parse.unexpected_token@1:42"}));
}

TEST(ParserProblemTest, LengthIsIgnoredWithAWarning)
{
  auto r = parse_problem(problem("(:length (:serial 10) (:parallel 3)) (:goal (p))"));
  ASSERT_TRUE(r.ok()) << dump(r.diagnostics());
  EXPECT_EQ(codes(r.diagnostics()), (std::vector<std::string>{"parse.ignored_section@1:34"}));
  EXPECT_TRUE(r->goal.valid());
}

TEST(ParserProblemTest, RepeatedSections)
{
  auto joined = parse_problem(problem("(:objects a) (:init (p a)) (:objects b) (:init (q b))"));
  ASSERT_TRUE(joined.ok()) << dump(joined.diagnostics());
  EXPECT_EQ(joined->objects.size(), 2u);
  EXPECT_EQ(joined->init.size(), 2u);

  // IPC 2004 airport instance 5 repeats an identical :metric
  auto same = parse_problem(problem(
      "(:metric minimize (total-time)) (:metric minimize (total-time))"));
  ASSERT_TRUE(same.ok()) << dump(same.diagnostics());
  EXPECT_EQ(codes(same.diagnostics()), (std::vector<std::string>{"parse.duplicate_section@1:66"}));

  EXPECT_EQ(errors_of(problem("(:metric minimize (total-time)) (:metric maximize (total-time))")),
    (std::vector<std::string>{"parse.duplicate_section@1:66"}));
  EXPECT_EQ(errors_of(problem("(:goal (p)) (:goal (q))")),
    (std::vector<std::string>{"parse.duplicate_section@1:46"}));
  EXPECT_EQ(errors_of("(define (problem p) (:domain a) (:domain b))"),
    (std::vector<std::string>{"parse.duplicate_section@1:34"}));
  EXPECT_TRUE(parse_problem("(define (problem p) (:domain a) (:domain A))").ok());
}

TEST(ParserProblemTest, Errors)
{
  EXPECT_EQ(errors_of("(define (problem p) (:domain ))"),
    (std::vector<std::string>{"parse.unexpected_token@1:30"}));
  EXPECT_EQ(errors_of(problem("(:goa (p))")),
    (std::vector<std::string>{"parse.unknown_section@1:34"}));
  EXPECT_EQ(errors_of(problem("(:init (= (f) ))")),
    (std::vector<std::string>{"parse.unexpected_token@1:47"}));
  EXPECT_EQ(errors_of(problem("(:init 3 (p))")),
    (std::vector<std::string>{"parse.unexpected_token@1:40"}));
}

TEST(ParserProblemTest, ErrorsInInitAreReportedOneByOne)
{
  // One bad fact must not hide the next ones
  auto found = errors_of(problem("(:init (p a) (q b{) (r c) (= (f) ) (s d))"));
  EXPECT_EQ(found, (std::vector<std::string>{"lex.invalid_character@1:49",
      "parse.unexpected_token@1:66"}));
}

TEST(ParserProblemTest, TruncatedObjectsSection)
{
  // PlanSys2's problem_malformed_truncated.pddl
  auto found = errors_of(
    "(define (problem simple_1)\n  (:domain simple)\n  (:objects\n    leia - robot\n"
    "  (:init\n    (robot_at leia kitchen)\n  )\n)\n");
  ASSERT_FALSE(found.empty());
  EXPECT_EQ(found[0], "parse.unexpected_token@5:3");
}

TEST(ParserProblemTest, FullSyntaxProblem)
{
  auto r = parse_problem(
    kairos::testing::read_file(kairos::testing::data_path("syntax/full_problem.pddl")));
  ASSERT_TRUE(r.ok()) << dump(r.diagnostics());
  EXPECT_TRUE(r.diagnostics().empty()) << dump(r.diagnostics());
  EXPECT_EQ(r->objects.size(), 4u);
  EXPECT_EQ(r->init.size(), 11u);
  EXPECT_TRUE(r->goal.valid());
  EXPECT_TRUE(r->constraints.valid());
  ASSERT_TRUE(r->metric.has_value());
}

TEST(ParserProblemTest, LargeProblem)
{
  std::string text = "(define (problem big) (:domain d) (:objects";
  for (int i = 0; i < 20000; ++i) {
    text += " r" + std::to_string(i);
  }
  text += " - robot) (:init";
  for (int i = 0; i < 100000; ++i) {
    text += " (at r" + std::to_string(i % 20000) + " w" + std::to_string(i % 97) + ")";
  }
  text += ") (:goal (and";
  for (int i = 0; i < 5000; ++i) {
    text += " (visited w" + std::to_string(i % 97) + ")";
  }
  text += ")))";
  auto r = parse_problem(text);
  ASSERT_TRUE(r.ok()) << dump(r.diagnostics());
  EXPECT_EQ(r->objects.size(), 20000u);
  EXPECT_EQ(r->init.size(), 100000u);
  EXPECT_EQ(r->nodes.formulas[r->goal].children.size(), 5000u);
  // Names are interned once: 20000 robots, 97 waypoints and a few keywords
  EXPECT_LT(r->symbols.size(), 20200u);
}

}  // namespace
}  // namespace kairos::pddl
