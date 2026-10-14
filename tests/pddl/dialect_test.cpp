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

#include "pddl/dialect.hpp"
#include "pddl/parser.hpp"
#include "support/pddl_helpers.hpp"

namespace kairos::pddl
{
namespace
{

using kairos::testing::codes;
using kairos::testing::dump;

ast::Domain domain(std::string_view text)
{
  auto r = parse_domain(text, "d.pddl");
  EXPECT_TRUE(r.ok()) << dump(r.diagnostics());
  return r.ok() ? std::move(r).value() : ast::Domain{};
}

ast::Problem problem(std::string_view text)
{
  auto r = parse_problem(text, "p.pddl");
  EXPECT_TRUE(r.ok()) << dump(r.diagnostics());
  return r.ok() ? std::move(r).value() : ast::Problem{};
}

std::string name(const ast::Domain & d, Symbol s) {return std::string(d.symbols.name(s));}

int count(const std::vector<Diagnostic> & diags, const std::string & code)
{
  return static_cast<int>(std::count_if(diags.begin(), diags.end(), [&](const Diagnostic & x) {
           return x.code == code;
         }));
}

// The function expressions of a domain, by name
std::vector<std::string> function_exprs(const ast::Domain & d)
{
  std::vector<std::string> out;
  for (uint32_t i = 0; i < d.nodes.exprs.size(); ++i) {
    const auto & e = d.nodes.exprs[ast::ExprId(i)];
    if (e.kind == ast::ExprKind::Function) {
      out.push_back(name(d, e.name));
    }
  }
  return out;
}

const char * kTank =
  "(define (domain tank) (:requirements :fluents :time)"
  " (:constants big - object)"
  " (:predicates (open) (air-flow-on))"
  " (:functions (level) (rate) (air-flow) (cap ?o - object))"
  " (:process fill :parameters () :precondition (open)"
  "  :effect (increase (level) (* #t rate)))"
  " (:action close :parameters () :precondition (and (open) (>= level (cap big)))"
  "  :effect (and (not (open)) (air_flow_on) (assign (air_flow) 0))))";

TEST(DialectTest, BareFunctionsWithoutArgumentsBecomeFunctions)
{
  ast::Domain d = domain(kTank);
  auto diags = repair_domain(d);
  auto f = function_exprs(d);
  // rate and level written bare: now (rate) and (level), each warned once
  EXPECT_EQ(std::count(f.begin(), f.end(), "rate"), 1);
  EXPECT_GE(std::count(f.begin(), f.end(), "level"), 2);
  int bare = 0;
  for (const auto & x : diags) {
    bare += x.code == "dialect.bare_function";
    EXPECT_EQ(x.severity, Severity::Warning);
  }
  EXPECT_EQ(bare, 2);
}

TEST(DialectTest, ConstantsAndFunctionsWithArgumentsStayAsTheyAre)
{
  ast::Domain d = domain(kTank);
  repair_domain(d);
  // big is a constant: a term, not a function; cap takes an argument
  bool big_term = false;
  for (uint32_t i = 0; i < d.nodes.terms.size(); ++i) {
    const auto & t = d.nodes.terms[ast::TermId(i)];
    big_term = big_term || (t.kind == ast::TermKind::Name && name(d, t.name) == "big");
  }
  EXPECT_TRUE(big_term);
  auto f = function_exprs(d);
  EXPECT_EQ(std::count(f.begin(), f.end(), "big"), 0);
}

TEST(DialectTest, UnderscoreForHyphenInNames)
{
  ast::Domain d = domain(kTank);
  auto diags = repair_domain(d);
  auto f = function_exprs(d);
  EXPECT_EQ(std::count(f.begin(), f.end(), "air_flow"), 0);
  EXPECT_EQ(std::count(f.begin(), f.end(), "air-flow"), 1);
  bool predicate = false;
  for (uint32_t i = 0; i < d.nodes.effects.size(); ++i) {
    const auto & e = d.nodes.effects[ast::EffectId(i)];
    if (e.kind == ast::EffectKind::Add) {
      predicate = predicate || name(d, e.name) == "air-flow-on";
    }
  }
  EXPECT_TRUE(predicate);
  EXPECT_EQ(count(diags, "dialect.underscore"), 2);
}

TEST(DialectTest, AmbiguousOrUnknownNamesAreLeftToTheAnalysis)
{
  // a-b and a_b both declared: a_b stays a_b; c_d matches nothing
  ast::Domain d = domain(
    "(define (domain x) (:predicates (a-b) (a_b) (p))"
    " (:action go :parameters () :precondition (and (a_b) (a-b)) :effect (and (c_d) (p))))");
  auto diags = repair_domain(d);
  EXPECT_TRUE(diags.empty()) << dump(diags);
  std::vector<std::string> atoms;
  for (uint32_t i = 0; i < d.nodes.formulas.size(); ++i) {
    const auto & f = d.nodes.formulas[ast::FormulaId(i)];
    if (f.kind == ast::FormulaKind::Atom) {
      atoms.push_back(name(d, f.name));
    }
  }
  EXPECT_EQ(atoms, (std::vector<std::string>{"a_b", "a-b"}));
}

TEST(DialectTest, FunctionDeclaredAgainMergesTheTypes)
{
  ast::Domain d = domain(
    "(define (domain x) (:types wall box)"
    " (:functions (bound ?w - wall) (bound ?b - box) (size ?w - wall) (size ?a ?b - box)))");
  auto diags = repair_domain(d);
  ASSERT_EQ(d.functions.size(), 3u);  // bound once; size twice (other arity: an error later)
  EXPECT_EQ(name(d, d.functions[0].name), "bound");
  ASSERT_EQ(d.functions[0].params.size(), 1u);
  std::vector<std::string> types;
  for (Symbol t : d.functions[0].params[0].types) {
    types.push_back(name(d, t));
  }
  EXPECT_EQ(types, (std::vector<std::string>{"wall", "box"}));
  EXPECT_EQ(codes(diags), (std::vector<std::string>{"dialect.function_redeclared@1:68"}));
}

TEST(DialectTest, UntypedDeclarationMakesTheParameterUntyped)
{
  ast::Domain d = domain("(define (domain x) (:types wall) (:functions (f ?w - wall) (f ?x)))");
  repair_domain(d);
  ASSERT_EQ(d.functions.size(), 1u);
  EXPECT_TRUE(d.functions[0].params[0].types.empty());
}

TEST(DialectTest, ProblemNamesAndInitialFacts)
{
  ast::Domain d = domain(
    "(define (domain x) (:types ship) (:predicates (ahead ?s - ship) (on-deck))"
    " (:functions (top-speed)))");
  repair_domain(d);
  ast::Problem p = problem(
    "(define (problem q) (:domain x) (:objects s - ship)"
    " (:init (ahead) (ahead s) (on_deck) (= (top_speed) 3) (= top-speed 4))"
    " (:goal (ahead s)) (:metric minimize top_speed))");
  auto diags = repair_problem(p, d);
  // (ahead) with no argument: dropped; on_deck and top_speed renamed
  ASSERT_EQ(p.init.size(), 4u);
  EXPECT_EQ(std::string(p.symbols.name(p.init[0].name)), "ahead");
  EXPECT_EQ(p.init[0].args.size(), 1u);
  EXPECT_EQ(std::string(p.symbols.name(p.init[1].name)), "on-deck");
  EXPECT_EQ(std::string(p.symbols.name(p.init[2].name)), "top-speed");
  EXPECT_EQ(count(diags, "dialect.init_arity"), 1);
  EXPECT_GE(count(diags, "dialect.underscore"), 2);
  // The metric's bare top_speed: the function (top-speed)
  const auto & m = p.nodes.exprs[p.metric->expr];
  EXPECT_EQ(m.kind, ast::ExprKind::Function);
  EXPECT_EQ(std::string(p.symbols.name(m.name)), "top-speed");
}

TEST(DialectTest, ProblemObjectsAreNotFunctions)
{
  // An object named like a function without arguments stays an object
  ast::Domain d = domain("(define (domain x) (:predicates (at ?o)) (:functions (home)))");
  repair_domain(d);
  ast::Problem p = problem(
    "(define (problem q) (:domain x) (:objects home) (:init (at home)) (:goal (at home)))");
  EXPECT_TRUE(repair_problem(p, d).empty());
}

TEST(DialectTest, FluentsOnlyTimeChangesStartAtZero)
{
  // (ptime ?t) only grows with #t and nothing assigns it: each tank without a value starts at
  // 0 (as SMTPlan); (level) is assigned somewhere, so it is left alone; t2 has its value
  ast::Domain d = domain(
    "(define (domain g) (:types tank pump - object big - tank)"
    " (:functions (ptime ?t - tank) (level) (flow ?p - pump))"
    " (:process run :parameters (?t - tank)"
    "  :effect (and (increase (ptime ?t) (* #t 1)) (increase (level) (* #t 2))))"
    " (:action reset :parameters () :effect (assign (level) 0)))");
  repair_domain(d);
  ast::Problem p = problem(
    "(define (problem q) (:domain g) (:objects t1 t2 - tank b - big p1 - pump)"
    " (:init (= (ptime t2) 5)))");
  auto diags = repair_problem(p, d);
  std::vector<std::string> added;
  for (const auto & item : p.init) {
    std::string s(p.symbols.name(item.name));
    for (auto a : item.args) {
      s += " " + std::string(p.symbols.name(p.nodes.terms[a].name));
    }
    added.push_back(s + "=" + std::to_string(static_cast<int>(p.nodes.exprs[item.value].number)));
  }
  EXPECT_EQ(added, (std::vector<std::string>{"ptime t2=5", "ptime t1=0", "ptime b=0"}));
  EXPECT_EQ(count(diags, "dialect.uninitialized"), 1);
}

TEST(DialectTest, UndeclaredPredicatesInConditionsAreNeverTrue)
{
  ast::Domain d = domain("(define (domain x) (:predicates (landed)))");
  repair_domain(d);
  ast::Problem p = problem(
    "(define (problem q) (:domain x) (:goal (and (landed) (not (crashed)))))");
  auto diags = repair_problem(p, d);
  EXPECT_EQ(count(diags, "dialect.undeclared_predicate"), 1);
  int empty_or = 0;
  for (uint32_t i = 0; i < p.nodes.formulas.size(); ++i) {
    const auto & f = p.nodes.formulas[ast::FormulaId(i)];
    empty_or += f.kind == ast::FormulaKind::Or && f.children.empty();
  }
  EXPECT_EQ(empty_or, 1);
}

TEST(DialectTest, PredicatesUsedAsFunctionsAreFunctions)
{
  // (splits) declared among the predicates, only read as a number; (on) used as a fact stays
  ast::Domain d = domain(
    "(define (domain x) (:predicates (on) (splits)) (:functions (level))"
    " (:action a :parameters () :precondition (on) :effect (increase (level) (/ 1 (splits)))))");
  auto diags = repair_domain(d);
  ASSERT_EQ(d.predicates.size(), 1u);
  EXPECT_EQ(name(d, d.predicates[0].name), "on");
  ASSERT_EQ(d.functions.size(), 2u);
  EXPECT_EQ(name(d, d.functions[1].name), "splits");
  EXPECT_EQ(count(diags, "dialect.predicate_as_function"), 1);
}

TEST(DialectTest, ObjectsOfTheProblemNamedInTheDomain)
{
  // car5 is the problem's: a constant of the type of the parameter it fills
  ast::Domain d = domain(
    "(define (domain x) (:types car) (:functions (y ?c - car))"
    " (:event far :parameters (?c - car) :precondition (> (- (y car5) (y ?c)) 40) :effect ()))");
  auto diags = repair_domain(d);
  ASSERT_EQ(d.constants.size(), 1u);
  EXPECT_EQ(name(d, d.constants[0].name), "car5");
  ASSERT_EQ(d.constants[0].types.size(), 1u);
  EXPECT_EQ(name(d, d.constants[0].types[0]), "car");
  EXPECT_EQ(count(diags, "dialect.problem_object"), 1);
}

TEST(DialectTest, CorrectPddlIsUntouched)
{
  ast::Domain d = domain(
    "(define (domain x) (:predicates (p ?x)) (:functions (f) (g ?x))"
    " (:action a :parameters (?x) :precondition (and (p ?x) (> (f) (g ?x)))"
    "  :effect (increase (f) 1)))");
  auto before = d.nodes.exprs.size();
  EXPECT_TRUE(repair_domain(d).empty());
  EXPECT_EQ(d.nodes.exprs.size(), before);
}

}  // namespace
}  // namespace kairos::pddl
