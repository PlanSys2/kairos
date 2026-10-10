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

#include <cmath>
#include <limits>
#include <random>
#include <string>
#include <vector>

#include "pddl/parser.hpp"
#include "pddl/printer.hpp"
#include "support/pddl_helpers.hpp"

namespace kairos::pddl
{
namespace
{

using kairos::testing::data_path;
using kairos::testing::dump;
using kairos::testing::read_file;

TEST(PrinterTest, FormatNumber)
{
  EXPECT_EQ(format_number(2.0), "2");
  EXPECT_EQ(format_number(0.5), "0.5");
  EXPECT_EQ(format_number(-1.25), "-1.25");
  EXPECT_EQ(format_number(0.1), "0.1");
  EXPECT_EQ(format_number(1e-7), "1e-07");
  EXPECT_EQ(format_number(1e20), "1e+20");
  EXPECT_EQ(format_number(0.0), "0");
}

TEST(PrinterTest, FormattedNumbersReadBackExactly)
{
  std::mt19937_64 rng(7);
  std::uniform_real_distribution<double> wide(-1e12, 1e12);
  std::uniform_real_distribution<double> narrow(-1.0, 1.0);
  for (int i = 0; i < 5000; ++i) {
    double v = i % 2 ? wide(rng) : narrow(rng);
    std::string text = "(define (problem p) (:init (= (f) " + format_number(v) + ")))";
    auto r = parse_problem(text);
    ASSERT_TRUE(r.ok()) << format_number(v);
    EXPECT_EQ(r->nodes.exprs[r->init[0].value].number, v) << format_number(v);
  }
  EXPECT_EQ(format_number(std::numeric_limits<double>::max()), "1.7976931348623157e+308");
}

TEST(PrinterTest, CanonicalDomain)
{
  auto r = parse_domain(
    "(DEFINE (DOMAIN D) (:REQUIREMENTS :STRIPS :TYPING) (:TYPES A B - T)"
    " (:predicates (p ?x - a) (q))  (:functions (f ?x) - number)"
    "(:action Go :parameters (?x - a ?y - b) :precondition (and (p ?x) (> (f ?x) 1.50))"
    " :effect (and (q) (not (p ?x)))))");
  ASSERT_TRUE(r.ok()) << dump(r.diagnostics());
  EXPECT_EQ(to_pddl(*r),
    "(define (domain d)\n"
    "  (:requirements :strips :typing)\n"
    "  (:types a b - t)\n"
    "  (:predicates\n"
    "    (p ?x - a)\n"
    "    (q))\n"
    "  (:functions\n"
    "    (f ?x) - number)\n"
    "\n"
    "  (:action go\n"
    "    :parameters (?x - a ?y - b)\n"
    "    :precondition (and (p ?x) (> (f ?x) 1.5))\n"
    "    :effect (and (q) (not (p ?x))))\n"
    ")\n");
}

TEST(PrinterTest, CanonicalProblem)
{
  auto r = parse_problem(
    "(define (problem P) (:domain D) (:objects a b - t c)"
    " (:init (p a) (= (f a) 2) (at 5.5 (not (p b))))"
    " (:goal (and (p b))) (:metric minimize (total-time)))");
  ASSERT_TRUE(r.ok()) << dump(r.diagnostics());
  EXPECT_EQ(to_pddl(*r),
    "(define (problem p)\n"
    "  (:domain d)\n"
    "  (:objects a b - t c)\n"
    "  (:init\n"
    "    (p a)\n"
    "    (= (f a) 2)\n"
    "    (at 5.5 (not (p b))))\n"
    "  (:goal (and (p b)))\n"
    "  (:metric minimize (total-time))\n"
    ")\n");
}

TEST(PrinterTest, SingleNodes)
{
  auto r = parse_domain(
    "(define (domain d) (:action a :parameters (?x) :precondition (not (p (f ?x)))"
    " :effect (increase (g) (* 2 (h ?x)))))");
  ASSERT_TRUE(r.ok()) << dump(r.diagnostics());
  const auto & d = *r;
  EXPECT_EQ(to_pddl(d.nodes, d.symbols, d.actions[0].precondition), "(not (p (f ?x)))");
  EXPECT_EQ(to_pddl(d.nodes, d.symbols, d.actions[0].effect), "(increase (g) (* 2 (h ?x)))");
  const auto & effect = d.nodes.effects[d.actions[0].effect];
  EXPECT_EQ(to_pddl(d.nodes, d.symbols, effect.value), "(* 2 (h ?x))");
  const auto & atom = d.nodes.formulas[d.nodes.formulas[d.actions[0].precondition].children[0]];
  EXPECT_EQ(to_pddl(d.nodes, d.symbols, atom.args.at(0)), "(f ?x)");
}

// print(parse(x)) is a fixed point: parsing it again and printing gives the same text
void expect_domain_round_trip(const std::string & text)
{
  auto first = parse_domain(text);
  ASSERT_TRUE(first.ok()) << dump(first.diagnostics());
  std::string printed = to_pddl(*first);
  auto second = parse_domain(printed);
  ASSERT_TRUE(second.ok()) << dump(second.diagnostics()) << printed;
  EXPECT_EQ(to_pddl(*second), printed);
  EXPECT_TRUE(second.diagnostics().empty()) << dump(second.diagnostics());
}

void expect_problem_round_trip(const std::string & text)
{
  auto first = parse_problem(text);
  ASSERT_TRUE(first.ok()) << dump(first.diagnostics());
  std::string printed = to_pddl(*first);
  auto second = parse_problem(printed);
  ASSERT_TRUE(second.ok()) << dump(second.diagnostics()) << printed;
  EXPECT_EQ(to_pddl(*second), printed);
}

TEST(PrinterTest, RoundTripFullSyntax)
{
  expect_domain_round_trip(read_file(data_path("syntax/full_domain.pddl")));
  expect_problem_round_trip(read_file(data_path("syntax/full_problem.pddl")));
}

TEST(PrinterTest, RoundTripNormalizesLenientInput)
{
  // Glued types, :vars and stray parentheses come out as standard PDDL
  auto r = parse_domain(
    "(define (domain d) (:predicates (p ?c -car)) (:action a :parameters (?x) :vars (?y)"
    " :precondition (p ?y)))))");
  ASSERT_TRUE(r.ok());
  std::string printed = to_pddl(*r);
  EXPECT_NE(printed.find("(p ?c - car)"), std::string::npos) << printed;
  EXPECT_NE(printed.find(":parameters (?x ?y)"), std::string::npos) << printed;
  auto again = parse_domain(printed);
  ASSERT_TRUE(again.ok());
  EXPECT_TRUE(again.diagnostics().empty()) << dump(again.diagnostics());
}

}  // namespace
}  // namespace kairos::pddl
