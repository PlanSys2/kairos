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
#include "pddl/printer.hpp"
#include "support/pddl_helpers.hpp"

namespace kairos::pddl
{
namespace
{

using kairos::testing::codes;
using kairos::testing::dump;

ast::Domain parse_ok(std::string_view text)
{
  auto r = parse_domain(text, "d.pddl");
  EXPECT_TRUE(r.ok()) << dump(r.diagnostics());
  if (!r.ok()) {
    return ast::Domain{};
  }
  return std::move(r).value();
}

std::vector<std::string> errors_of(std::string_view text, const ParseOptions & options = {})
{
  auto r = parse_domain(text, "d.pddl", options);
  EXPECT_FALSE(r.ok()) << "expected a parse failure";
  return codes(r.diagnostics());
}

std::string name(const ast::Domain & d, Symbol s) {return std::string(d.symbols.name(s));}

std::string text(const ast::Domain & d, ast::FormulaId id)
{
  return to_pddl(d.nodes, d.symbols, id);
}

std::string text(const ast::Domain & d, ast::EffectId id)
{
  return to_pddl(d.nodes, d.symbols, id);
}

std::string with_action(std::string_view action)
{
  return "(define (domain d) " + std::string(action) + ")";
}

// --- Header and sections ---------------------------------------------------------------------

TEST(ParserDomainTest, MinimalDomain)
{
  auto d = parse_ok("(define (domain minimal))");
  EXPECT_EQ(name(d, d.name), "minimal");
  EXPECT_EQ(d.loc, (ast::Loc{1, 1}));
  EXPECT_EQ(d.file, "d.pddl");
  EXPECT_TRUE(d.actions.empty());
  EXPECT_FALSE(d.constraints.valid());
}

TEST(ParserDomainTest, KeywordsAndNamesAreCaseInsensitive)
{
  auto d = parse_ok(
    "(DEFINE (DOMAIN MiXeD) (:Requirements :STRIPS) (:PREDICATES (At ?X))"
    "(:ACTION Go :PARAMETERS (?X) :PRECONDITION (AND (At ?X)) :EFFECT (NOT (AT ?x))))");
  EXPECT_EQ(name(d, d.name), "mixed");
  EXPECT_EQ(name(d, d.requirements.at(0).name), ":strips");
  EXPECT_EQ(name(d, d.predicates.at(0).name), "at");
  ASSERT_EQ(d.actions.size(), 1u);
  EXPECT_EQ(name(d, d.actions[0].name), "go");
  EXPECT_EQ(text(d, d.actions[0].precondition), "(and (at ?x))");
  EXPECT_EQ(text(d, d.actions[0].effect), "(not (at ?x))");
}

TEST(ParserDomainTest, SectionsInAnyOrder)
{
  auto d = parse_ok(
    "(define (domain d) (:action a :parameters ()) (:predicates (p)) (:requirements :strips)"
    " (:types t))");
  EXPECT_EQ(d.actions.size(), 1u);
  EXPECT_EQ(d.predicates.size(), 1u);
  EXPECT_EQ(d.requirements.size(), 1u);
  EXPECT_EQ(d.types.size(), 1u);
}

TEST(ParserDomainTest, Requirements)
{
  auto r = parse_domain("(define (domain d) (:requirements :strips :typing :made-up))");
  ASSERT_TRUE(r.ok()) << dump(r.diagnostics());
  EXPECT_EQ(r->requirements.size(), 3u);
  EXPECT_EQ(codes(r.diagnostics()),
    (std::vector<std::string>{"parse.unknown_requirement@1:51"}));
}

TEST(ParserDomainTest, KnownRequirements)
{
  EXPECT_TRUE(is_known_requirement(":durative-actions"));
  EXPECT_TRUE(is_known_requirement(":non-deterministic"));
  EXPECT_FALSE(is_known_requirement("durative-actions"));
  EXPECT_FALSE(is_known_requirement(":Strips"));  // callers lower-case first
}

// --- Typed lists -----------------------------------------------------------------------------

TEST(ParserDomainTest, TypeHierarchyWithEither)
{
  auto d = parse_ok("(define (domain d) (:types a b - t c - (either x y) d))");
  ASSERT_EQ(d.types.size(), 4u);
  EXPECT_EQ(name(d, d.types[0].types.at(0)), "t");
  EXPECT_EQ(name(d, d.types[1].types.at(0)), "t");
  ASSERT_EQ(d.types[2].types.size(), 2u);
  EXPECT_EQ(name(d, d.types[2].types[1]), "y");
  EXPECT_TRUE(d.types[3].types.empty());
  EXPECT_EQ(d.types[2].loc, (ast::Loc{1, 36}));
}

TEST(ParserDomainTest, TypeGluedToTheDash)
{
  // Seen in PDDL+ benchmark domains: "?c -car"
  auto d = parse_ok(
    "(define (domain d) (:predicates (p ?c -car ?d - car)) (:functions (f) -number))");
  const auto & params = d.predicates.at(0).params;
  ASSERT_EQ(params.size(), 2u);
  EXPECT_EQ(name(d, params[0].types.at(0)), "car");
  EXPECT_EQ(name(d, params[1].types.at(0)), "car");
  EXPECT_EQ(name(d, d.functions.at(0).type.at(0)), "number");
}

TEST(ParserDomainTest, DoubleDashNameIsNotAGluedType)
{
  auto r = parse_domain("(define (domain d) (:constants --x))");
  ASSERT_TRUE(r.ok()) << dump(r.diagnostics());
  EXPECT_EQ(r->symbols.name(r->constants.at(0).name), "--x");
}

TEST(ParserDomainTest, TypeWithoutNamesIsAWarning)
{
  // IPC 2008 woodworking instances have "- board" with no boards before it
  auto r = parse_domain("(define (domain d) (:constants a - t - u b))");
  ASSERT_TRUE(r.ok()) << dump(r.diagnostics());
  EXPECT_EQ(codes(r.diagnostics()),
    (std::vector<std::string>{"parse.type_without_names@1:38"}));
  EXPECT_EQ(r->constants.size(), 2u);
  EXPECT_TRUE(r->constants[1].types.empty());
}

TEST(ParserDomainTest, TypedListErrors)
{
  EXPECT_EQ(errors_of("(define (domain d) (:types a - ))"),
    (std::vector<std::string>{"parse.unexpected_token@1:32"}));
  EXPECT_EQ(errors_of("(define (domain d) (:types a - (either)))"),
    (std::vector<std::string>{"parse.empty_either@1:32"}));
  EXPECT_EQ(errors_of("(define (domain d) (:types a - (or x y)))"),
    (std::vector<std::string>{"parse.unexpected_token@1:33"}));
  EXPECT_EQ(errors_of("(define (domain d) (:predicates (p ?x - t y)))"),
    (std::vector<std::string>{"parse.unexpected_token@1:43"}));
}

TEST(ParserDomainTest, Constants)
{
  auto d = parse_ok("(define (domain d) (:constants leia lema - robot jack))");
  ASSERT_EQ(d.constants.size(), 3u);
  EXPECT_EQ(name(d, d.constants[1].types.at(0)), "robot");
  EXPECT_TRUE(d.constants[2].types.empty());
}

TEST(ParserDomainTest, Predicates)
{
  auto d = parse_ok("(define (domain d) (:predicates (at ?r - robot ?l) (busy)))");
  ASSERT_EQ(d.predicates.size(), 2u);
  EXPECT_EQ(d.predicates[0].params.size(), 2u);
  EXPECT_TRUE(d.predicates[0].params[1].types.empty());
  EXPECT_TRUE(d.predicates[1].params.empty());
  EXPECT_EQ(d.predicates[1].loc, (ast::Loc{1, 52}));
}

TEST(ParserDomainTest, FunctionsWithAndWithoutTypes)
{
  auto d = parse_ok(
    "(define (domain d) (:functions (f ?x) (g) - number (h ?r - robot) - place (k)))");
  ASSERT_EQ(d.functions.size(), 4u);
  EXPECT_EQ(name(d, d.functions[0].type.at(0)), "number");  // group typed by "- number"
  EXPECT_EQ(name(d, d.functions[1].type.at(0)), "number");
  EXPECT_EQ(name(d, d.functions[2].type.at(0)), "place");   // object fluent
  EXPECT_TRUE(d.functions[3].type.empty());                 // untyped: number by default
}

// --- Actions ---------------------------------------------------------------------------------

TEST(ParserDomainTest, ActionPartsAreOptional)
{
  auto d = parse_ok(with_action("(:action a :parameters ())"));
  EXPECT_FALSE(d.actions.at(0).precondition.valid());
  EXPECT_FALSE(d.actions.at(0).effect.valid());
  auto no_params = parse_ok(with_action("(:action b :effect (p))"));
  EXPECT_TRUE(no_params.actions.at(0).params.empty());
}

TEST(ParserDomainTest, EmptyFormulaAndEffect)
{
  auto d = parse_ok(with_action("(:action a :parameters () :precondition () :effect ())"));
  EXPECT_EQ(text(d, d.actions[0].precondition), "(and)");
  EXPECT_EQ(text(d, d.actions[0].effect), "(and)");
}

TEST(ParserDomainTest, AllFormulaKinds)
{
  auto d = parse_ok(with_action(
      "(:action a :parameters (?x ?y) :precondition (and (p ?x) (or (q) (not (r ?y)))"
      " (imply (p ?x) (q)) (exists (?z - t) (p ?z)) (forall (?w) (not (p ?w)))"
      " (= ?x ?y) (< (f ?x) 3) (<= 1 2) (> (f ?x) (g)) (>= (g) -1.5) (preference pr (q))"
      " (preference (q))))"));
  EXPECT_EQ(
    text(d, d.actions[0].precondition),
    "(and (p ?x) (or (q) (not (r ?y))) (imply (p ?x) (q)) (exists (?z - t) (p ?z))"
    " (forall (?w) (not (p ?w))) (= ?x ?y) (< (f ?x) 3) (<= 1 2) (> (f ?x) (g)) (>= (g) -1.5)"
    " (preference pr (q)) (preference (q)))");
}

TEST(ParserDomainTest, AllEffectKinds)
{
  auto d = parse_ok(with_action(
      "(:action a :parameters (?x) :effect (and (p ?x) (not (q ?x))"
      " (forall (?y - t) (when (and (r ?y)) (not (r ?y)))) (assign (f ?x) 3)"
      " (increase (g) (* 2 (f ?x))) (decrease (g) 1) (scale-up (g) 2) (scale-down (g) 2)"
      " (assign (holder ?x) ?x) (increase total-cost 1)))"));
  EXPECT_EQ(
    text(d, d.actions[0].effect),
    "(and (p ?x) (not (q ?x)) (forall (?y - t) (when (and (r ?y)) (not (r ?y))))"
    " (assign (f ?x) 3) (increase (g) (* 2 (f ?x))) (decrease (g) 1) (scale-up (g) 2)"
    " (scale-down (g) 2) (assign (holder ?x) ?x) (increase (total-cost) 1))");
}

TEST(ParserDomainTest, VarsAreExtraParameters)
{
  auto r = parse_domain(with_action(
      "(:action a :parameters (?x) :vars (?n - food) :precondition (p ?x ?n))"));
  ASSERT_TRUE(r.ok()) << dump(r.diagnostics());
  const auto & params = r->actions.at(0).params;
  ASSERT_EQ(params.size(), 2u);
  EXPECT_EQ(r->symbols.name(params[1].name), "?n");
  EXPECT_EQ(codes(r.diagnostics()), (std::vector<std::string>{"parse.vars_as_parameters@1:48"}));
}

TEST(ParserDomainTest, AtIsAnOrdinaryPredicateOutsideTimedContexts)
{
  auto d = parse_ok(with_action(
      "(:action move :parameters (?r ?a ?b) :precondition (at ?r ?a)"
      " :effect (and (not (at ?r ?a)) (at ?r ?b)))"));
  EXPECT_EQ(text(d, d.actions[0].precondition), "(at ?r ?a)");
  EXPECT_EQ(text(d, d.actions[0].effect), "(and (not (at ?r ?a)) (at ?r ?b))");
}

TEST(ParserDomainTest, ActionErrors)
{
  EXPECT_EQ(errors_of(with_action("(:action a :params ())")),
    (std::vector<std::string>{"parse.unknown_key@1:31"}));
  EXPECT_EQ(errors_of(with_action("(:action a :effect (p) :effect (q))")),
    (std::vector<std::string>{"parse.duplicate_key@1:43"}));
  EXPECT_EQ(errors_of(with_action("(:action :parameters ())")),
    (std::vector<std::string>{"parse.unexpected_token@1:29"}));
  EXPECT_EQ(errors_of(with_action("(:action a :parameters (x))")),
    (std::vector<std::string>{"parse.unexpected_token@1:44"}));
}

// --- Durative actions ------------------------------------------------------------------------

TEST(ParserDomainTest, DurativeAction)
{
  auto d = parse_ok(with_action(
      "(:durative-action go :parameters (?r ?a ?b)"
      " :duration (and (>= ?duration 2) (<= ?duration (* 2 (dist ?a ?b))))"
      " :condition (and (at start (at ?r ?a)) (over all (link ?a ?b)) (at end (free ?b)))"
      " :effect (and (at start (not (at ?r ?a))) (at end (at ?r ?b))"
      " (at end (decrease (battery ?r) (* ?duration 2))) (increase (clock) #t)"
      " (when (at start (busy)) (at end (free ?a)))))"));
  ASSERT_EQ(d.durative_actions.size(), 1u);
  const auto & a = d.durative_actions[0];
  EXPECT_EQ(text(d, a.duration), "(and (>= ?duration 2) (<= ?duration (* 2 (dist ?a ?b))))");
  EXPECT_EQ(text(d, a.condition),
    "(and (at start (at ?r ?a)) (over all (link ?a ?b)) (at end (free ?b)))");
  EXPECT_EQ(text(d, a.effect),
    "(and (at start (not (at ?r ?a))) (at end (at ?r ?b))"
    " (at end (decrease (battery ?r) (* ?duration 2))) (increase (clock) #t)"
    " (when (at start (busy)) (at end (free ?a))))");
}

TEST(ParserDomainTest, AtPredicateInsideTimedConditions)
{
  // "(at start (at ?r ?a))": the inner "at" is the predicate
  auto d = parse_ok(with_action(
      "(:durative-action go :parameters (?r ?a) :duration (= ?duration 1)"
      " :condition (at start (at ?r ?a)) :effect (at end (at ?r ?a)))"));
  const auto & a = d.durative_actions[0];
  EXPECT_EQ(text(d, a.condition), "(at start (at ?r ?a))");
  EXPECT_EQ(text(d, a.effect), "(at end (at ?r ?a))");
  const auto & inner = d.nodes.formulas[d.nodes.formulas[a.condition].children.at(0)];
  EXPECT_EQ(inner.kind, ast::FormulaKind::Atom);
}

TEST(ParserDomainTest, DurativeActionNeedsDuration)
{
  EXPECT_EQ(errors_of(with_action("(:durative-action go :parameters ())")),
    (std::vector<std::string>{"parse.missing_duration@1:21"}));
}

TEST(ParserDomainTest, OverWithoutAllIsAnAtom)
{
  auto d = parse_ok(with_action(
      "(:durative-action go :parameters (?x) :duration (= ?duration 1)"
      " :condition (over ?x))"));
  EXPECT_EQ(d.nodes.formulas[d.durative_actions[0].condition].kind, ast::FormulaKind::Atom);
}

// --- Derived, processes, events, constraints -------------------------------------------------

TEST(ParserDomainTest, Derived)
{
  auto d = parse_ok(
    "(define (domain d) (:derived (reach ?a ?b - place)"
    " (or (link ?a ?b) (exists (?c) (and (link ?a ?c) (reach ?c ?b))))))");
  ASSERT_EQ(d.derived.size(), 1u);
  EXPECT_EQ(name(d, d.derived[0].head.name), "reach");
  EXPECT_EQ(d.derived[0].head.params.size(), 2u);
  EXPECT_EQ(text(d, d.derived[0].body),
    "(or (link ?a ?b) (exists (?c) (and (link ?a ?c) (reach ?c ?b))))");
}

TEST(ParserDomainTest, ProcessesAndEvents)
{
  auto d = parse_ok(
    "(define (domain d)"
    " (:process heat :parameters (?o) :precondition (on ?o)"
    "   :effect (increase (temp ?o) (* #t (^ (power ?o) 2))))"
    " (:event boil :parameters (?o) :precondition (>= (temp ?o) 100) :effect (boiling ?o))"
    " (:process tick :parameters () :precondition () :effect (increase (t) (#t))))");
  ASSERT_EQ(d.processes.size(), 2u);
  ASSERT_EQ(d.events.size(), 1u);
  EXPECT_EQ(text(d, d.processes[0].effect), "(increase (temp ?o) (* #t (^ (power ?o) 2)))");
  EXPECT_EQ(text(d, d.processes[1].effect), "(increase (t) #t)");
  EXPECT_EQ(text(d, d.events[0].effect), "(boiling ?o)");
}

TEST(ParserDomainTest, ProcessesDoNotAcceptVars)
{
  EXPECT_EQ(errors_of("(define (domain d) (:process p :vars (?x)))"),
    (std::vector<std::string>{"parse.unknown_key@1:32"}));
}

TEST(ParserDomainTest, GlobalConstraints)
{
  auto d = parse_ok(
    "(define (domain d) (:constraint speed-limit :parameters (?c) :condition (<= (v ?c) 10)))");
  ASSERT_EQ(d.global_constraints.size(), 1u);
  EXPECT_EQ(name(d, d.global_constraints[0].name), "speed-limit");
  EXPECT_EQ(text(d, d.global_constraints[0].condition), "(<= (v ?c) 10)");
  EXPECT_EQ(errors_of("(define (domain d) (:constraint c :parameters ()))"),
    (std::vector<std::string>{"parse.missing_condition@1:21"}));
}

TEST(ParserDomainTest, TrajectoryConstraints)
{
  auto d = parse_ok(
    "(define (domain d) (:constraints (and (always (p)) (sometime (q)) (within 5 (p))"
    " (at-most-once (p)) (sometime-after (p) (q)) (sometime-before (p) (q))"
    " (always-within 2.5 (p) (q)) (hold-during 1 3 (p)) (hold-after 4 (q))"
    " (at end (p)) (sometime (at r1 dock)) (preference c1 (always (q))))))");
  EXPECT_EQ(text(d, d.constraints),
    "(and (always (p)) (sometime (q)) (within 5 (p)) (at-most-once (p))"
    " (sometime-after (p) (q)) (sometime-before (p) (q)) (always-within 2.5 (p) (q))"
    " (hold-during 1 3 (p)) (hold-after 4 (q)) (at end (p)) (sometime (at r1 dock))"
    " (preference c1 (always (q))))");
}

TEST(ParserDomainTest, ModalNamesAreAtomsOutsideConstraints)
{
  auto d = parse_ok(with_action("(:action a :precondition (always ?x))"));
  EXPECT_EQ(d.nodes.formulas[d.actions[0].precondition].kind, ast::FormulaKind::Atom);
}

TEST(ParserDomainTest, ModalErrors)
{
  EXPECT_EQ(errors_of("(define (domain d) (:constraints (within x (p))))"),
    (std::vector<std::string>{"parse.unexpected_token@1:42"}));
}

// --- Expressions -----------------------------------------------------------------------------

TEST(ParserDomainTest, Expressions)
{
  auto d = parse_ok(with_action(
      "(:action a :precondition (and (> (+ 1 2 3) (* (f) 2 (g))) (= (- (f)) (- (f) 1))"
      " (< (/ (f) 2) (^ (f) 3)) (= total-time (total-time))))"));
  EXPECT_EQ(text(d, d.actions[0].precondition),
    "(and (> (+ 1 2 3) (* (f) 2 (g))) (= (- (f)) (- (f) 1)) (< (/ (f) 2) (^ (f) 3))"
    " (= (total-time) (total-time)))");
  const auto & neg = d.nodes.formulas[d.nodes.formulas[d.actions[0].precondition].children[1]];
  EXPECT_EQ(d.nodes.exprs[neg.lhs].kind, ast::ExprKind::Neg);
  EXPECT_EQ(d.nodes.exprs[neg.rhs].kind, ast::ExprKind::Sub);
}

TEST(ParserDomainTest, OperatorArity)
{
  EXPECT_EQ(errors_of(with_action("(:action a :precondition (> (+ 1) 0))")),
    (std::vector<std::string>{"parse.operator_arity@1:49"}));
  EXPECT_EQ(errors_of(with_action("(:action a :precondition (> (- 1 2 3) 0))")),
    (std::vector<std::string>{"parse.operator_arity@1:49"}));
  EXPECT_EQ(errors_of(with_action("(:action a :precondition (> (/ 1) 0))")),
    (std::vector<std::string>{"parse.operator_arity@1:49"}));
  EXPECT_EQ(errors_of(with_action("(:action a :precondition (> (^ 1 2 3) 0))")),
    (std::vector<std::string>{"parse.operator_arity@1:49"}));
  EXPECT_EQ(errors_of(with_action("(:action a :precondition (> (< 1 2) 0))")),
    (std::vector<std::string>{"parse.unexpected_token@1:49"}));
  EXPECT_EQ(errors_of(with_action("(:action a :precondition (+ 1 2))")),
    (std::vector<std::string>{"parse.unexpected_token@1:46"}));
}

// --- Errors and recovery ---------------------------------------------------------------------

TEST(ParserDomainTest, HeaderErrors)
{
  EXPECT_EQ(errors_of(""), (std::vector<std::string>{"parse.unexpected_end@1:1"}));
  EXPECT_EQ(errors_of("domain d"), (std::vector<std::string>{"parse.unexpected_token@1:1"}));
  EXPECT_EQ(errors_of("(defin (domain d))"),
    (std::vector<std::string>{"parse.unexpected_token@1:2"}));
  EXPECT_EQ(errors_of("(define (problem d))"),
    (std::vector<std::string>{"parse.unexpected_token@1:10"}));
  EXPECT_EQ(errors_of("(define (domain))"),
    (std::vector<std::string>{"parse.unexpected_token@1:16"}));
  EXPECT_EQ(errors_of("(define (domain d)"),
    (std::vector<std::string>{"parse.unexpected_end@1:19", "parse.unclosed@1:1"}));
}

TEST(ParserDomainTest, TruncatedInputs)
{
  // Every prefix of a valid domain fails cleanly with exactly one end-of-input error
  std::string full = kairos::testing::read_file(kairos::testing::data_path(
        "syntax/full_domain.pddl"));
  for (size_t n = 0; n + 1 < full.size(); ++n) {
    // Cut only between words: a cut inside a word changes the word, which is another error
    char next = full[n];
    if (next != ' ' && next != '\n' && next != '(' && next != ')') {
      continue;
    }
    auto r = parse_domain(full.substr(0, n));
    ASSERT_FALSE(r.ok()) << "prefix of length " << n << " parsed";
    int ends = 0;
    for (const auto & dgn : r.diagnostics()) {
      ends += dgn.code == "parse.unexpected_end";
    }
    EXPECT_EQ(ends, 1) << "prefix " << n << ":\n" << dump(r.diagnostics());
  }
}

TEST(ParserDomainTest, UnknownSectionIsSkippedAndTheRestParsed)
{
  auto r = parse_domain(
    "(define (domain d)\n (:predicates (p))\n (:bogus (x) (y))\n (:action a :effect (p)))");
  ASSERT_FALSE(r.ok());
  EXPECT_EQ(codes(r.diagnostics()), (std::vector<std::string>{"parse.unknown_section@3:3"}));
}

TEST(ParserDomainTest, SeveralErrorsInOnePass)
{
  auto codes_found = errors_of(
    "(define (domain d)\n"
    " (:action a :bad 1)\n"
    " (:action b :effect (p))\n"
    " (:action c :parameters (x))\n"
    " (:predicates (p) (q ?x - ) (r)))");
  EXPECT_EQ(codes_found, (std::vector<std::string>{
        "parse.unknown_key@2:13", "parse.unexpected_token@4:26", "parse.unexpected_token@5:27"}));
}

TEST(ParserDomainTest, StrayWordsBetweenSectionsDoNotHang)
{
  EXPECT_EQ(errors_of("(define (domain d) junk 12 ?v (:predicates (p)))"),
    (std::vector<std::string>{
        "parse.unexpected_token@1:20", "parse.unexpected_token@1:25",
        "parse.unexpected_token@1:28"}));
}

TEST(ParserDomainTest, LexerErrorsAreReported)
{
  EXPECT_EQ(errors_of("(define (domain d) (:predicates (p a{b)))"),
    (std::vector<std::string>{"lex.invalid_character@1:36"}));
  EXPECT_EQ(errors_of("(define (domain d) (:constants x{))"),
    (std::vector<std::string>{"lex.invalid_character@1:32"}));
}

TEST(ParserDomainTest, TrailingText)
{
  EXPECT_EQ(errors_of("(define (domain d)) (extra)"),
    (std::vector<std::string>{"parse.trailing_tokens@1:21"}));
  auto r = parse_domain("(define (domain d)))\n)");
  ASSERT_TRUE(r.ok()) << dump(r.diagnostics());
  EXPECT_EQ(codes(r.diagnostics()), (std::vector<std::string>{"parse.extra_parentheses@1:20"}));
}

TEST(ParserDomainTest, RepeatedSections)
{
  auto joined = parse_domain(
    "(define (domain d) (:predicates (p)) (:types a) (:predicates (q)) (:types b))");
  ASSERT_TRUE(joined.ok()) << dump(joined.diagnostics());
  EXPECT_EQ(joined->predicates.size(), 2u);
  EXPECT_EQ(joined->types.size(), 2u);
  EXPECT_EQ(codes(joined.diagnostics()), (std::vector<std::string>{
        "parse.duplicate_section@1:50", "parse.duplicate_section@1:68"}));

  auto same = parse_domain("(define (domain d) (:constraints (always (p))) (:constraints "
      "(always (p))))");
  ASSERT_TRUE(same.ok()) << dump(same.diagnostics());
  EXPECT_EQ(codes(same.diagnostics()), (std::vector<std::string>{"parse.duplicate_section@1:49"}));

  EXPECT_EQ(errors_of("(define (domain d) (:constraints (always (p))) (:constraints "
    "(always (q))))"), (std::vector<std::string>{"parse.duplicate_section@1:49"}));
}

TEST(ParserDomainTest, NestingLimit)
{
  ParseOptions options;
  options.max_depth = 10;
  std::string nested;
  for (int i = 0; i < 12; ++i) {
    nested += "(and ";
  }
  nested += "(p)" + std::string(12, ')');
  std::string deep = with_action("(:action a :precondition " + nested + ")");
  auto found = errors_of(deep, options);
  ASSERT_FALSE(found.empty());
  EXPECT_EQ(found[0].substr(0, found[0].find('@')), "parse.nesting_too_deep");
}

TEST(ParserDomainTest, HugeNestingDoesNotOverflowTheStack)
{
  std::string nested;
  for (int i = 0; i < 200000; ++i) {
    nested += "(not ";
  }
  nested += "(p)" + std::string(200000, ')');
  std::string deep = "(define (domain d) (:action a :precondition " + nested + "))";
  auto found = errors_of(deep);
  ASSERT_FALSE(found.empty());
  EXPECT_EQ(found[0].substr(0, found[0].find('@')), "parse.nesting_too_deep");
}

TEST(ParserDomainTest, TooManyErrors)
{
  std::string text = "(define (domain d)";
  for (int i = 0; i < 100; ++i) {
    text += " (:bogus)";
  }
  text += ")";
  auto r = parse_domain(text);
  ASSERT_FALSE(r.ok());
  ASSERT_EQ(r.diagnostics().size(), 51u);
  EXPECT_EQ(r.diagnostics().back().code, "parse.too_many_errors");

  ParseOptions few;
  few.max_errors = 3;
  auto limited = parse_domain(text, "d.pddl", few);
  EXPECT_EQ(limited.diagnostics().size(), 4u);
}

// --- Helpful diagnostics -------------------------------------------------------------------

std::vector<Diagnostic> diagnostics_of(std::string_view text)
{
  return parse_domain(text, "d.pddl").diagnostics();
}

std::string message_of(const std::vector<Diagnostic> & ds, const std::string & code)
{
  for (const auto & d : ds) {
    if (d.code == code) {
      return d.message;
    }
  }
  return "<no " + code + ">";
}

TEST(ParserDiagnosticsTest, MissingParenthesisInsideAFormula)
{
  // The '(and' is not closed: the error shows at ':effect', the note points at the '(and'
  auto found = codes(diagnostics_of(
      "(define (domain d)\n"
      " (:predicates (p) (q))\n"
      " (:action a\n"
      "  :precondition (and (p)\n"
      "  :effect (q))\n"
      " (:action b :effect (p)))\n"));
  EXPECT_EQ(found, (std::vector<std::string>{
        "parse.unexpected_token@5:3", "parse.maybe_unclosed@4:17", "parse.resuming@6:2"}));
}

TEST(ParserDiagnosticsTest, ParsingResumesAtTheNextSection)
{
  // After resuming, the following sections are parsed normally: an error there is reported
  auto found = codes(diagnostics_of(
      "(define (domain d)\n"
      " (:action a :precondition (and (p) :effect (q))\n"
      " (:action b :effect (p) :bogus 1))\n"));
  EXPECT_EQ(found[0], "parse.unexpected_token@2:36");
  EXPECT_NE(std::find(found.begin(), found.end(), "parse.resuming@3:2"), found.end());
  EXPECT_NE(std::find(found.begin(), found.end(), "parse.unknown_key@3:25"), found.end());
}

TEST(ParserDiagnosticsTest, ResumingInsideTheInitOfAProblem)
{
  auto r = parse_problem(
    "(define (problem p) (:domain d)\n"
    " (:init (at r1 kitchen) (= (f) 3\n"
    " (:goal (at r1 hall)))\n");
  ASSERT_FALSE(r.ok());
  auto found = codes(r.diagnostics());
  EXPECT_NE(std::find(found.begin(), found.end(), "parse.resuming@3:2"), found.end())
    << dump(r.diagnostics());
}

TEST(ParserDiagnosticsTest, UnexpectedEndPointsAtTheUnclosedParenthesis)
{
  auto found = codes(diagnostics_of(
      "(define (domain d)\n (:predicates (p)\n (:action a :effect (p)))"));
  EXPECT_EQ(found.back(), "parse.unclosed@1:1");
}

TEST(ParserDiagnosticsTest, NoUnclosedHintForAMissingName)
{
  // "(:action :parameters" lacks the name; no parenthesis is missing
  auto found = codes(diagnostics_of("(define (domain d) (:action :parameters ()))"));
  EXPECT_EQ(found, (std::vector<std::string>{"parse.unexpected_token@1:29"}));
}

TEST(ParserDiagnosticsTest, Suggestions)
{
  EXPECT_EQ(message_of(diagnostics_of("(define (domain d) (:predicate (p)))"),
    "parse.unknown_section"), "unknown domain section :predicate (did you mean ':predicates'?)");
  EXPECT_EQ(message_of(diagnostics_of("(define (domain d) (:action a :precondtion (p)))"),
    "parse.unknown_key"), "unknown key :precondtion in action (did you mean ':precondition'?)");
  EXPECT_EQ(message_of(diagnostics_of(
      "(define (domain d) (:durative-action a :duraton (= ?duration 1)))"), "parse.unknown_key"),
    "unknown key :duraton in durative action (did you mean ':duration'?)");
  EXPECT_EQ(message_of(diagnostics_of("(define (domain d) (:requirements :durative-action))"),
    "parse.unknown_requirement"),
    "unknown requirement :durative-action (did you mean ':durative-actions'?)");
  auto problem = parse_problem("(define (problem p) (:domain d) (:goals (p)))");
  EXPECT_EQ(message_of(problem.diagnostics(), "parse.unknown_section"),
    "unknown problem section :goals (did you mean ':goal'?)");
  EXPECT_EQ(message_of(diagnostics_of("(define (domain d) (:frobnicate))"),
    "parse.unknown_section"), "unknown domain section :frobnicate");
}

TEST(ParserDiagnosticsTest, MathBuiltins)
{
  auto d = parse_ok(with_action(
      "(:action a :precondition (> (+ (sin (theta)) (ABS (- (x) 3))) 0))"));
  EXPECT_EQ(text(d, d.actions[0].precondition), "(> (+ (sin (theta)) (abs (- (x) 3))) 0)");
  EXPECT_EQ(errors_of(with_action("(:action a :precondition (> (cos 1 2) 0))")),
    (std::vector<std::string>{"parse.operator_arity@1:49"}));
}

TEST(ParserDomainTest, FullSyntaxDomain)
{
  auto r = parse_domain(
    kairos::testing::read_file(kairos::testing::data_path("syntax/full_domain.pddl")),
    "full_domain.pddl");
  ASSERT_TRUE(r.ok()) << dump(r.diagnostics());
  EXPECT_TRUE(r.diagnostics().empty()) << dump(r.diagnostics());
  const auto & d = *r;
  EXPECT_EQ(name(d, d.name), "full-syntax");
  EXPECT_EQ(d.requirements.size(), 13u);
  EXPECT_EQ(d.types.size(), 5u);
  EXPECT_EQ(d.constants.size(), 5u);
  EXPECT_EQ(d.predicates.size(), 6u);
  EXPECT_EQ(d.functions.size(), 5u);
  EXPECT_TRUE(d.constraints.valid());
  EXPECT_EQ(d.actions.size(), 2u);
  EXPECT_EQ(d.durative_actions.size(), 2u);
  EXPECT_EQ(d.derived.size(), 1u);
  EXPECT_EQ(d.processes.size(), 1u);
  EXPECT_EQ(d.events.size(), 1u);
  EXPECT_EQ(d.global_constraints.size(), 1u);
}

TEST(ParserDomainTest, DetectFileKind)
{
  EXPECT_EQ(detect_file_kind("(define (domain d))"), FileKind::Domain);
  EXPECT_EQ(detect_file_kind("; c\n(DEFINE\n ; c\n (Problem p))"), FileKind::Problem);
  EXPECT_EQ(detect_file_kind("\xEF\xBB\xBF(define (domain d))"), FileKind::Domain);
  EXPECT_EQ(detect_file_kind(""), FileKind::Unknown);
  EXPECT_EQ(detect_file_kind("(define (other x))"), FileKind::Unknown);
  EXPECT_EQ(detect_file_kind("(in-package \"PDDL\")"), FileKind::Unknown);
  EXPECT_EQ(detect_file_kind("define (domain d)"), FileKind::Unknown);
}

}  // namespace
}  // namespace kairos::pddl
