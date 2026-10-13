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

struct Analysis
{
  bool parsed = false;
  bool ok = false;
  std::vector<Diagnostic> diagnostics;
  std::optional<DomainModel> model;
};

Analysis analyze(std::string_view text)
{
  Analysis a;
  auto parsed = pddl::parse_domain(text, "d.pddl");
  a.parsed = parsed.ok();
  if (!parsed.ok()) {
    a.diagnostics = parsed.diagnostics();
    return a;
  }
  auto r = analyze_domain(*parsed);
  a.ok = r.ok();
  a.diagnostics = r.diagnostics();
  if (r.ok()) {
    a.model = std::move(r).value();
  }
  return a;
}

// Codes of errors only (warnings about requirements are checked separately)
std::vector<std::string> errors(std::string_view text)
{
  Analysis a = analyze(text);
  EXPECT_TRUE(a.parsed) << dump(a.diagnostics);
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

// A small typed domain to build on
std::string robots(const std::string & extra)
{
  return
    "(define (domain robots)\n"
    " (:requirements :strips :typing :negative-preconditions :fluents :durative-actions\n"
    "                :equality :adl :derived-predicates :duration-inequalities\n"
    "                :continuous-effects :preferences :constraints :time)\n"
    " (:types robot room - object charger - room)\n"
    " (:constants dock - charger)\n"
    " (:predicates (at ?r - robot ?l - room) (connected ?a ?b - room) (busy ?r - robot))\n"
    " (:functions (battery ?r - robot) (distance ?a ?b - room) (speed ?r - robot))\n" +
    extra + ")";
}

// --- Valid domains ---------------------------------------------------------------------------

TEST(DomainAnalysisTest, ValidDomainBuildsTheModel)
{
  Analysis a = analyze(robots(
      "(:action move :parameters (?r - robot ?a ?b - room)"
      " :precondition (and (at ?r ?a) (connected ?a ?b)) :effect (and (not (at ?r ?a)) (at ?r ?b)"
      " (decrease (battery ?r) (distance ?a ?b))))"));
  ASSERT_TRUE(a.ok) << dump(a.diagnostics);
  EXPECT_TRUE(a.diagnostics.empty()) << dump(a.diagnostics);
  const DomainModel & m = *a.model;
  EXPECT_EQ(m.name, "robots");
  EXPECT_EQ(m.predicates.items.size(), 3u);
  EXPECT_EQ(m.functions.items.size(), 3u);
  auto charger = m.types.find("charger");
  ASSERT_TRUE(charger.has_value());
  EXPECT_TRUE(m.types.is_subtype(*charger, *m.types.find("room")));
  EXPECT_TRUE(m.constants.find("dock").has_value());
  EXPECT_TRUE(m.functions.items[*m.functions.find("battery")].changes);
  EXPECT_FALSE(m.functions.items[*m.functions.find("distance")].changes);  // static
  EXPECT_TRUE(m.features.contains(Feature::Typing));
  EXPECT_TRUE(m.features.contains(Feature::NegativePreconditions) == false);
  EXPECT_TRUE(m.features.contains(Feature::NumericFluents));
}

TEST(DomainAnalysisTest, ParentTypesNeedNotBeListed)
{
  Analysis a = analyze("(define (domain d) (:requirements :typing) (:types car - vehicle)"
      " (:predicates (p ?v - vehicle)))");
  ASSERT_TRUE(a.ok) << dump(a.diagnostics);
  EXPECT_TRUE(a.model->types.find("vehicle").has_value());
}

TEST(DomainAnalysisTest, EitherInTypesGivesSeveralParents)
{
  Analysis a = analyze("(define (domain d) (:requirements :typing)"
      " (:types car boat - object amphibian - (either car boat)))");
  ASSERT_TRUE(a.ok) << dump(a.diagnostics);
  const auto & t = a.model->types;
  EXPECT_TRUE(t.is_subtype(*t.find("amphibian"), *t.find("boat")));
  EXPECT_TRUE(t.is_subtype(*t.find("amphibian"), *t.find("car")));
}

TEST(DomainAnalysisTest, ConstantsUsableInActions)
{
  EXPECT_TRUE(errors(robots(
      "(:action dock-it :parameters (?r - robot) :precondition (at ?r dock)"
      " :effect (not (at ?r dock)))")).empty());
}

TEST(DomainAnalysisTest, EmptyDomain)
{
  Analysis a = analyze("(define (domain empty))");
  ASSERT_TRUE(a.ok);
  EXPECT_TRUE(a.diagnostics.empty());
}

// --- Declarations ----------------------------------------------------------------------------

TEST(DomainAnalysisTest, UnknownTypeWithSuggestion)
{
  Analysis a = analyze("(define (domain d) (:requirements :typing) (:types robot)"
      " (:predicates (at ?r - robto)))");
  EXPECT_FALSE(a.ok);
  EXPECT_EQ(codes(a.diagnostics), (std::vector<std::string>{"semantic.unknown_type@1:81"}));
  EXPECT_EQ(message(a.diagnostics, "semantic.unknown_type"),
    "unknown type 'robto' (did you mean 'robot'?)");
}

TEST(DomainAnalysisTest, TypeCycle)
{
  EXPECT_EQ(errors("(define (domain d) (:types a - b b - a))").at(0).substr(0, 19),
    "semantic.type_cycle");
}

TEST(DomainAnalysisTest, NumberAsTypeIsAWarning)
{
  // IPC 2004 promela domains use "number" as an object type
  Analysis a = analyze("(define (domain d) (:requirements :typing) (:types number queue)"
      " (:predicates (size ?q - queue ?n - number)) (:functions (f) - number))");
  ASSERT_TRUE(a.ok) << dump(a.diagnostics);
  EXPECT_TRUE(has_code(a.diagnostics, "semantic.number_as_type"));
  EXPECT_TRUE(a.model->functions.items[*a.model->functions.find("f")].numeric);
}

TEST(DomainAnalysisTest, Duplicates)
{
  EXPECT_EQ(errors("(define (domain d) (:predicates (p) (p ?x)))"),
    (std::vector<std::string>{"semantic.duplicate_predicate@1:37"}));
  EXPECT_EQ(errors("(define (domain d) (:functions (f) (f)))"),
    (std::vector<std::string>{"semantic.duplicate_function@1:36"}));
  EXPECT_EQ(errors("(define (domain d) (:action a) (:action a))"),
    (std::vector<std::string>{"semantic.duplicate_action@1:33"}));
  EXPECT_EQ(errors(
        "(define (domain d) (:action a) (:durative-action a :duration (= ?duration 1)))"),
    (std::vector<std::string>{"semantic.duplicate_action@1:33"}));
  EXPECT_EQ(errors("(define (domain d) (:predicates (p ?x)) (:action a :parameters (?x ?x)))"),
    (std::vector<std::string>{"semantic.duplicate_parameter@1:68"}));
}

TEST(DomainAnalysisTest, LenientDuplicates)
{
  // Repeated parameter names in a declaration only matter for arity (IPC 2000 logistics)
  Analysis decl = analyze("(define (domain d) (:predicates (in ?o ?o)))");
  ASSERT_TRUE(decl.ok);
  EXPECT_TRUE(has_code(decl.diagnostics, "semantic.duplicate_parameter"));
  // A constant declared twice with the same type
  Analysis same = analyze("(define (domain d) (:constants a a))");
  ASSERT_TRUE(same.ok);
  EXPECT_TRUE(has_code(same.diagnostics, "semantic.duplicate_object"));
  // ... and with two types: it belongs to both
  Analysis two = analyze("(define (domain d) (:requirements :typing) (:types t u)"
      " (:constants a - t a - u))");
  ASSERT_TRUE(two.ok) << dump(two.diagnostics);
  EXPECT_TRUE(has_code(two.diagnostics, "semantic.object_with_several_types"));
  EXPECT_EQ(two.model->constants.items[*two.model->constants.find("a")].types.size(), 2u);
}

// --- Formulas --------------------------------------------------------------------------------

TEST(DomainAnalysisTest, UnknownPredicateWithContextAndSuggestion)
{
  Analysis a = analyze(robots(
      "(:action move :parameters (?r - robot ?a - room) :precondition (at-robot ?r ?a))"));
  EXPECT_FALSE(a.ok);
  EXPECT_EQ(message(a.diagnostics, "semantic.unknown_predicate"),
    "in action 'move': unknown predicate 'at-robot'");
  Analysis typo = analyze(robots(
      "(:action move :parameters (?r - robot ?a - room) :precondition (conected ?a ?a))"));
  EXPECT_EQ(message(typo.diagnostics, "semantic.unknown_predicate"),
    "in action 'move': unknown predicate 'conected' (did you mean 'connected'?)");
}

TEST(DomainAnalysisTest, Arity)
{
  Analysis a = analyze(robots(
      "(:action move :parameters (?r - robot ?a - room) :precondition (at ?r))"));
  EXPECT_EQ(message(a.diagnostics, "semantic.wrong_arity"),
    "in action 'move': 'at' takes 2 argument(s), 1 given");
}

TEST(DomainAnalysisTest, UnboundVariableWithSuggestion)
{
  Analysis a = analyze(robots(
      "(:action move :parameters (?robot - robot ?a - room) :precondition (at ?robt ?a))"));
  EXPECT_EQ(message(a.diagnostics, "semantic.unbound_variable"),
    "in action 'move': variable ?robt is not a parameter or quantified variable "
    "(did you mean '?robot'?)");
}

TEST(DomainAnalysisTest, QuantifiedVariablesAreScoped)
{
  EXPECT_TRUE(errors(robots(
      "(:action a :parameters (?r - robot) :precondition (forall (?l - room) (not (at ?r ?l))))"))
    .empty());
  // ?l is not visible outside its quantifier
  EXPECT_EQ(errors(robots(
      "(:action a :parameters (?r - robot) :precondition (and (exists (?l - room) (at ?r ?l))"
      " (connected ?l ?l)))")).size(), 2u);
}

TEST(DomainAnalysisTest, TypeChecksOfArguments)
{
  // A constant of the wrong type is an error
  EXPECT_EQ(errors(robots(
      "(:action a :parameters (?r - robot) :precondition (busy dock))")),
    (std::vector<std::string>{"semantic.type_mismatch@9:57"}));
  // A variable of an incompatible type is an error...
  EXPECT_EQ(errors(robots(
      "(:action a :parameters (?x - room) :precondition (busy ?x))")),
    (std::vector<std::string>{"semantic.type_mismatch@9:56"}));
  // ...but a variable of a more general type only filters at grounding time
  EXPECT_TRUE(errors(robots(
      "(:action a :parameters (?x - object) :precondition (busy ?x))")).empty());
  // A subtype is fine
  EXPECT_TRUE(errors(robots(
      "(:action a :parameters (?r - robot ?c - charger) :precondition (at ?r ?c))")).empty());
}

TEST(DomainAnalysisTest, EqualityAndNumericComparisons)
{
  Analysis a = analyze(robots(
      "(:action a :parameters (?x ?y - room ?r - robot) :precondition (and (not (= ?x ?y))"
      " (= ?x dock) (>= (battery ?r) (distance ?x ?y))))"));
  ASSERT_TRUE(a.ok) << dump(a.diagnostics);
  EXPECT_TRUE(a.model->features.contains(Feature::Equality));
  EXPECT_TRUE(a.model->features.contains(Feature::NegativePreconditions));
  EXPECT_EQ(errors(robots(
      "(:action a :parameters (?x - room ?r - robot) :precondition (= ?x (battery ?r)))")),
    (std::vector<std::string>{"semantic.type_mismatch@9:61"}));
  EXPECT_EQ(errors(robots(
      "(:action a :parameters (?x ?y - room) :precondition (< ?x ?y))")),
    (std::vector<std::string>{"semantic.type_mismatch@9:53"}));
}

TEST(DomainAnalysisTest, FeaturesFollowPolarity)
{
  // and under negation is a disjunction; forall under negation is existential
  Analysis a = analyze(robots(
      "(:action a :parameters (?r - robot) :precondition (not (and (busy ?r)"
      " (forall (?l - room) (at ?r ?l)))))"));
  ASSERT_TRUE(a.ok) << dump(a.diagnostics);
  const auto & f = a.model->features;
  EXPECT_TRUE(f.contains(Feature::DisjunctivePreconditions));
  EXPECT_TRUE(f.contains(Feature::ExistentialPreconditions));
  EXPECT_FALSE(f.contains(Feature::UniversalPreconditions));
  EXPECT_TRUE(f.contains(Feature::NegativePreconditions));

  // A double negation is positive
  Analysis b = analyze(robots(
      "(:action a :parameters (?r - robot) :precondition (not (not (busy ?r))))"));
  EXPECT_FALSE(b.model->features.contains(Feature::NegativePreconditions));

  // imply: the antecedent is negative
  Analysis c = analyze(robots(
      "(:action a :parameters (?r - robot) :precondition (imply (busy ?r) (busy ?r)))"));
  EXPECT_TRUE(c.model->features.contains(Feature::NegativePreconditions));
  EXPECT_TRUE(c.model->features.contains(Feature::DisjunctivePreconditions));
}

TEST(DomainAnalysisTest, MisplacedConstructs)
{
  EXPECT_EQ(errors(robots(
      "(:action a :parameters (?r - robot) :precondition (at start (busy ?r)))")),
    (std::vector<std::string>{"semantic.misplaced_timed@9:51"}));
  EXPECT_EQ(errors(robots(
      "(:action a :parameters (?r - robot) :precondition (> (battery ?r) ?duration))")),
    (std::vector<std::string>{"semantic.misplaced_duration@9:67"}));
  EXPECT_EQ(errors(robots(
      "(:action a :parameters (?r - robot) :effect (increase (battery ?r) #t))")),
    (std::vector<std::string>{"semantic.misplaced_continuous@9:68"}));
  EXPECT_EQ(errors(robots(
      "(:action a :parameters (?r - robot) :precondition (> (total-time) 3))")),
    (std::vector<std::string>{"semantic.misplaced_total_time@9:54"}));
  EXPECT_EQ(errors(robots(
      "(:action a :parameters (?r - robot) :precondition (always (busy ?r)))")),
    (std::vector<std::string>{"semantic.unknown_predicate@9:51"}));  // "always" is a predicate
}

TEST(DomainAnalysisTest, Preferences)
{
  Analysis a = analyze(robots(
      "(:action a :parameters (?r - robot) :precondition (preference p1 (busy ?r)))"));
  ASSERT_TRUE(a.ok) << dump(a.diagnostics);
  EXPECT_TRUE(a.model->preferences.count("p1"));
  EXPECT_TRUE(a.model->features.contains(Feature::Preferences));
  EXPECT_EQ(errors(robots(
      "(:action a :parameters (?r - robot) :precondition (not (preference (busy ?r))))")),
    (std::vector<std::string>{"semantic.negated_preference@9:56"}));
  EXPECT_EQ(errors(robots(
      "(:action a :parameters (?r - robot) :precondition (preference (preference (busy ?r))))")),
    (std::vector<std::string>{"semantic.misplaced_preference@9:63"}));
}

// --- Effects ---------------------------------------------------------------------------------

TEST(DomainAnalysisTest, EffectChecks)
{
  EXPECT_EQ(errors(robots(
      "(:action a :parameters (?r - robot) :effect (assign (battery ?r) dock))")),
    (std::vector<std::string>{"semantic.type_mismatch@9:45"}));
  EXPECT_EQ(errors(robots(
      "(:action a :parameters (?r - robot) :effect (increase (charge ?r) 1))")),
    (std::vector<std::string>{"semantic.unknown_function@9:55"}));
  Analysis cond = analyze(robots(
      "(:action a :parameters (?r - robot) :effect (forall (?l - room)"
      " (when (at ?r ?l) (not (at ?r ?l)))))"));
  ASSERT_TRUE(cond.ok) << dump(cond.diagnostics);
  EXPECT_TRUE(cond.model->features.contains(Feature::ConditionalEffects));
}

TEST(DomainAnalysisTest, ObjectFluents)
{
  Analysis a = analyze(
    "(define (domain d) (:requirements :typing :object-fluents) (:types robot gripper)"
    " (:functions (holder ?g - gripper) - robot)"
    " (:action take :parameters (?g - gripper ?r - robot) :precondition (= (holder ?g) ?r)"
    "  :effect (assign (holder ?g) ?r)))");
  ASSERT_TRUE(a.ok) << dump(a.diagnostics);
  EXPECT_TRUE(a.model->features.contains(Feature::ObjectFluents));
  EXPECT_FALSE(a.model->functions.items[*a.model->functions.find("holder")].numeric);
  EXPECT_FALSE(analyze(
    "(define (domain d) (:types robot gripper) (:functions (holder ?g - gripper) - robot)"
    " (:action take :parameters (?g - gripper) :effect (increase (holder ?g) 1)))").ok);
  // undefined: an object fluent with no value (PDDL 3.1); not for numeric ones
  Analysis release = analyze(
    "(define (domain d) (:requirements :typing :object-fluents :numeric-fluents)"
    " (:types robot gripper) (:functions (holder ?g - gripper) - robot (n))"
    " (:action drop :parameters (?g - gripper) :effect (assign (holder ?g) undefined)))");
  EXPECT_TRUE(release.ok) << dump(release.diagnostics);
  EXPECT_FALSE(analyze(
    "(define (domain d) (:requirements :typing :numeric-fluents) (:types gripper)"
    " (:functions (n))"
    " (:action drop :parameters (?g - gripper) :effect (assign (n) undefined)))").ok);
}

TEST(DomainAnalysisTest, ActionCosts)
{
  Analysis a = analyze("(define (domain d) (:requirements :action-costs)"
      " (:functions (total-cost) - number) (:action a :effect (increase (total-cost) 2)))");
  ASSERT_TRUE(a.ok) << dump(a.diagnostics);
  EXPECT_TRUE(a.model->features.contains(Feature::ActionCosts));
}

// --- Durative actions ------------------------------------------------------------------------

std::string durative(
  const std::string & duration, const std::string & condition,
  const std::string & effect)
{
  return robots(
    "(:durative-action go :parameters (?r - robot ?a ?b - room) :duration " + duration +
    " :condition " + condition + " :effect " + effect + ")");
}

TEST(DomainAnalysisTest, ValidDurativeAction)
{
  Analysis a = analyze(durative(
      "(= ?duration (/ (distance ?a ?b) (speed ?r)))",
      "(and (at start (at ?r ?a)) (over all (connected ?a ?b)) (at end (> (battery ?r) 0)))",
      "(and (at start (not (at ?r ?a))) (at end (at ?r ?b))"
      " (at end (decrease (battery ?r) (* 2 ?duration))) (decrease (battery ?r) (* #t 0.5)))"));
  ASSERT_TRUE(a.ok) << dump(a.diagnostics);
  const auto & f = a.model->features;
  EXPECT_TRUE(f.contains(Feature::DurativeActions));
  EXPECT_TRUE(f.contains(Feature::DurationInEffects));
  EXPECT_TRUE(f.contains(Feature::ContinuousEffects));
  EXPECT_FALSE(f.contains(Feature::VariableDuration));  // distance and speed are static
  EXPECT_FALSE(f.contains(Feature::DurationInequalities));
}

TEST(DomainAnalysisTest, DurationKinds)
{
  Analysis ineq = analyze(durative("(and (>= ?duration 1) (<= ?duration (battery ?r)))",
      "(at start (at ?r ?a))", "(at end (decrease (battery ?r) 1))"));
  ASSERT_TRUE(ineq.ok) << dump(ineq.diagnostics);
  EXPECT_TRUE(ineq.model->features.contains(Feature::DurationInequalities));
  EXPECT_TRUE(ineq.model->features.contains(Feature::VariableDuration));  // battery changes

  EXPECT_EQ(errors(durative("(> ?duration 1)", "(at start (at ?r ?a))", "(at end (busy ?r))")),
    (std::vector<std::string>{"semantic.bad_duration@9:70"}));
  EXPECT_EQ(errors(durative("(= 1 2)", "(at start (at ?r ?a))", "(at end (busy ?r))")),
    (std::vector<std::string>{"semantic.bad_duration@9:70"}));
  EXPECT_EQ(errors(durative("(= ?duration ?duration)", "(at start (at ?r ?a))",
    "(at end (busy ?r))")), (std::vector<std::string>{"semantic.bad_duration@9:70"}));
  EXPECT_EQ(errors(durative("(busy ?r)", "(at start (at ?r ?a))", "(at end (busy ?r))")),
    (std::vector<std::string>{"semantic.bad_duration@9:70"}));
}

TEST(DomainAnalysisTest, DurativeConditionsAndEffectsMustBeTimed)
{
  Analysis cond = analyze(durative("(= ?duration 1)", "(and (at ?r ?a))", "(at end (busy ?r))"));
  EXPECT_EQ(message(cond.diagnostics, "semantic.untimed_condition"),
    "in durative action 'go': conditions of durative actions must be under at start, at end or "
    "over all");
  EXPECT_TRUE(has_code(analyze(durative("(= ?duration 1)", "(at start (at ?r ?a))",
    "(busy ?r)")).diagnostics, "semantic.untimed_effect"));
  EXPECT_TRUE(has_code(analyze(durative("(= ?duration 1)", "(at start (at ?r ?a))",
    "(increase (battery ?r) 1)")).diagnostics, "semantic.untimed_effect"));
  // Nested timed operators
  EXPECT_TRUE(has_code(analyze(durative("(= ?duration 1)",
    "(at start (at end (at ?r ?a)))", "(at end (busy ?r))")).diagnostics,
    "semantic.misplaced_timed"));
  EXPECT_TRUE(has_code(analyze(durative("(= ?duration 1)", "(at start (at ?r ?a))",
    "(at end (at start (busy ?r)))")).diagnostics, "semantic.misplaced_timed"));
  // #t inside at end
  EXPECT_TRUE(has_code(analyze(durative("(= ?duration 1)", "(at start (at ?r ?a))",
    "(at end (increase (battery ?r) #t))")).diagnostics, "semantic.misplaced_continuous"));
}

TEST(DomainAnalysisTest, TimedConditionalEffects)
{
  EXPECT_TRUE(analyze(durative("(= ?duration 1)", "(at start (at ?r ?a))",
    "(when (at start (busy ?r)) (at end (not (busy ?r))))")).ok);
  EXPECT_TRUE(analyze(durative("(= ?duration 1)", "(at start (at ?r ?a))",
    "(forall (?l - room) (at end (not (at ?r ?l))))")).ok);
}

TEST(DomainAnalysisTest, NonLinearDetection)
{
  // battery changes, speed is static: (* speed ?duration) is linear, (* battery battery) is not
  Analysis linear = analyze(durative("(= ?duration 2)", "(at start (at ?r ?a))",
      "(at end (decrease (battery ?r) (* (speed ?r) ?duration)))"));
  EXPECT_FALSE(linear.model->features.contains(Feature::NonLinearNumeric));
  Analysis quadratic = analyze(robots(
      "(:action a :parameters (?r - robot) :effect (assign (battery ?r) (* (battery ?r)"
      " (battery ?r))))"));
  EXPECT_TRUE(quadratic.model->features.contains(Feature::NonLinearNumeric));
  Analysis division = analyze(robots(
      "(:action a :parameters (?r - robot) :effect (assign (speed ?r) (/ 1 (speed ?r))))"));
  EXPECT_TRUE(division.model->features.contains(Feature::NonLinearNumeric));
  Analysis math = analyze(robots(
      "(:action a :parameters (?r - robot) :precondition (> (abs (speed ?r)) 0))"));
  EXPECT_TRUE(math.model->features.contains(Feature::NonLinearNumeric));
}

// --- Processes, events, constraints, derived -------------------------------------------------

TEST(DomainAnalysisTest, ProcessesAndEvents)
{
  Analysis a = analyze(robots(
      "(:process drain :parameters (?r - robot) :precondition (busy ?r)"
      "  :effect (decrease (battery ?r) (* #t 2)))"
      "(:event empty :parameters (?r - robot) :precondition (<= (battery ?r) 0)"
      "  :effect (not (busy ?r)))"));
  ASSERT_TRUE(a.ok) << dump(a.diagnostics);
  EXPECT_TRUE(a.model->features.contains(Feature::Processes));
  EXPECT_TRUE(a.model->features.contains(Feature::Events));
  EXPECT_TRUE(has_code(analyze(robots(
      "(:process p :parameters (?r - robot) :precondition (busy ?r) :effect (not (busy ?r)))"))
    .diagnostics, "semantic.discrete_effect_in_process"));
  EXPECT_TRUE(has_code(analyze(robots(
      "(:event e :parameters (?r - robot) :precondition (busy ?r)"
      " :effect (increase (battery ?r) (* #t 1)))")).diagnostics,
    "semantic.misplaced_continuous"));
}

TEST(DomainAnalysisTest, TrajectoryConstraints)
{
  Analysis a = analyze(robots(
      "(:constraints (and (always (> (battery dock) 0)) (within 10 (busy dock))"
      " (at end (connected dock dock))))"));
  // dock is a charger, not a robot
  EXPECT_FALSE(a.ok);
  EXPECT_TRUE(has_code(a.diagnostics, "semantic.type_mismatch"));
  Analysis ok = analyze(robots("(:constraints (and (always (connected dock dock))"
      " (sometime-after (connected dock dock) (connected dock dock))))"));
  ASSERT_TRUE(ok.ok) << dump(ok.diagnostics);
  EXPECT_TRUE(ok.model->features.contains(Feature::TrajectoryConstraints));
  EXPECT_TRUE(has_code(analyze(robots("(:constraints (within -1 (connected dock dock)))"))
    .diagnostics, "semantic.negative_time"));
}

TEST(DomainAnalysisTest, DerivedPredicates)
{
  Analysis a = analyze(
    "(define (domain d) (:requirements :typing :derived-predicates) (:types place)"
    " (:predicates (link ?a ?b - place) (reach ?a ?b - place))"
    " (:derived (reach ?a ?b) (or (link ?a ?b) (exists (?c - place) (and (link ?a ?c)"
    " (reach ?c ?b))))))");
  ASSERT_TRUE(a.ok) << dump(a.diagnostics);
  EXPECT_TRUE(a.model->predicates.items[*a.model->predicates.find("reach")].derived);
  EXPECT_TRUE(a.model->features.contains(Feature::DerivedPredicates));
}

TEST(DomainAnalysisTest, DerivedPredicateErrors)
{
  // Changing a derived predicate in an effect
  EXPECT_TRUE(has_code(analyze(
      "(define (domain d) (:predicates (p) (q)) (:derived (q) (p)) (:action a :effect (q)))")
    .diagnostics, "semantic.derived_in_effect"));
  // Negative self-dependency is unstratifiable
  EXPECT_TRUE(has_code(analyze(
      "(define (domain d) (:predicates (p) (q)) (:derived (q) (not (q))))").diagnostics,
    "semantic.unstratifiable_derived"));
  EXPECT_TRUE(has_code(analyze(
      "(define (domain d) (:predicates (p) (q) (r)) (:derived (q) (r)) (:derived (r) (not (q))))")
    .diagnostics, "semantic.unstratifiable_derived"));
  // Positive recursion is fine
  EXPECT_TRUE(analyze(
      "(define (domain d) (:predicates (p) (q)) (:derived (q) (or (p) (q))))").ok);
  // Undeclared derived predicates are declared implicitly
  Analysis implicit = analyze("(define (domain d) (:predicates (p)) (:derived (q ?x) (p)))");
  ASSERT_TRUE(implicit.ok) << dump(implicit.diagnostics);
  EXPECT_TRUE(has_code(implicit.diagnostics, "semantic.undeclared_derived"));
  // Wrong arity against the declaration
  EXPECT_TRUE(has_code(analyze(
      "(define (domain d) (:predicates (p) (q ?x)) (:derived (q) (p)))").diagnostics,
    "semantic.wrong_arity"));
}

TEST(DomainAnalysisTest, DerivedHeadTakesDeclaredTypes)
{
  // The head is untyped, as PlanSys2 writes it; the body uses ?r as a robot
  EXPECT_TRUE(analyze(robots(
      "(:derived (busy ?r) (exists (?l - room) (at ?r ?l)))")).ok);
}

// --- Requirements ----------------------------------------------------------------------------

TEST(DomainAnalysisTest, UndeclaredRequirementsAreWarnings)
{
  Analysis a = analyze("(define (domain d) (:requirements :strips) (:types t)"
      " (:predicates (p ?x - t)) (:action a :parameters (?x - t) :precondition (not (p ?x))))");
  ASSERT_TRUE(a.ok);
  std::vector<std::string> found;
  for (const auto & d : a.diagnostics) {
    if (d.code == "semantic.undeclared_requirement") {
      found.push_back(d.message);
    }
  }
  EXPECT_EQ(found, (std::vector<std::string>{
        "uses typing but :requirements does not include :typing",
        "uses negative-preconditions but :requirements does not include :negative-preconditions"}));
  // Located at the requirements section
  EXPECT_EQ(a.diagnostics[0].location->column, 35u);
}

// The undeclared requirements of a domain, as their feature names
std::vector<std::string> undeclared(const std::string & requirements, const std::string & pre)
{
  Analysis a = analyze("(define (domain d) (:requirements :typing " + requirements + ")"
      " (:types t) (:predicates (p ?x - t) (q ?x - t))"
      " (:action a :parameters (?x - t) :precondition " + pre + "))");
  EXPECT_TRUE(a.ok);
  std::vector<std::string> out;
  for (const auto & d : a.diagnostics) {
    if (d.code == "semantic.undeclared_requirement") {
      out.push_back(d.message.substr(5, d.message.find(' ', 5) - 5));
    }
  }
  return out;
}

TEST(DomainAnalysisTest, RequirementsGoByWhatIsWritten)
{
  using V = std::vector<std::string>;
  // (not (exists ...)) means "for all ... not", but only negation and exists are written
  // (PlanSys2's combat domain got warnings of universal and disjunctive preconditions)
  EXPECT_EQ(undeclared(":negative-preconditions :existential-preconditions",
    "(not (exists (?y - t) (and (p ?y) (q ?y))))"), V{});
  EXPECT_EQ(undeclared(":negative-preconditions", "(not (and (p ?x) (q ?x)))"), V{});
  EXPECT_EQ(undeclared(":negative-preconditions :universal-preconditions",
    "(not (forall (?y - t) (p ?y)))"), V{});
  // Written without being declared: still warnings
  EXPECT_EQ(undeclared(":negative-preconditions", "(not (exists (?y - t) (p ?y)))"),
    V{"existential-preconditions"});
  EXPECT_EQ(undeclared("", "(forall (?y - t) (p ?y))"), V{"universal-preconditions"});
  EXPECT_EQ(undeclared("", "(or (p ?x) (q ?x))"), V{"disjunctive-preconditions"});
  EXPECT_EQ(undeclared("", "(imply (p ?x) (q ?x))"), V{"disjunctive-preconditions"});
  EXPECT_EQ(undeclared(":quantified-preconditions", "(exists (?y - t) (p ?y))"), V{});
}

TEST(DomainAnalysisTest, AdlCoversItsParts)
{
  Analysis a = analyze("(define (domain d) (:requirements :adl) (:types t) (:predicates (p ?x - t))"
      " (:action a :parameters (?x - t) :precondition (or (not (p ?x)) (exists (?y - t) (p ?y)))"
      " :effect (forall (?z - t) (when (p ?z) (not (p ?z))))))");
  ASSERT_TRUE(a.ok);
  EXPECT_TRUE(a.diagnostics.empty()) << dump(a.diagnostics);
}

}  // namespace
}  // namespace kairos::sem
