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
#include <cstdlib>
#include <filesystem>
#include <map>
#include <optional>
#include <random>
#include <string>
#include <vector>

#include "kairos/plan.hpp"
#include "kairos/validate.hpp"
#include "normalize/normalizer.hpp"
#include "pddl/parser.hpp"
#include "semantic/analyzer.hpp"
#include "support/lifted_simulator.hpp"
#include "support/pddl_helpers.hpp"
#include "support/task_helpers.hpp"

namespace kairos::lifted
{
namespace
{

using kairos::testing::dump;
using kairos::testing::normalize_text;
using kairos::testing::read_file;
using testing::Simulator;
using testing::SimState;

// Keeps the parsed inputs alive with the task
std::string cond(const Task & t, const Condition & c, const std::vector<Variable> & v)
{
  return to_string(t, c, v);
}

std::string domain_with(const std::string & body)
{
  return "(define (domain d) (:requirements :adl :typing :fluents :derived-predicates)"
         " (:types item) (:predicates (p ?x - item) (q ?x - item) (r ?x ?y - item) (z))"
         " (:functions (f ?x - item) (g)) " + body + ")";
}

const char * kProblem = "(define (problem pr) (:domain d) (:objects a b c - item)"
  " (:init (p a) (= (f a) 1) (= (f b) 2) (= (f c) 3) (= (g) 0)) (:goal (z)))";

// --- Structure -------------------------------------------------------------------------------

TEST(NormalizeTest, SimpleActionKeepsItsShape)
{
  auto n = normalize_text(domain_with(
      "(:action act :parameters (?x - item) :precondition (and (p ?x) (not (q ?x)))"
      " :effect (and (q ?x) (not (p ?x)) (increase (g) 1)))"), kProblem);
  const Task & t = n->task;
  ASSERT_EQ(t.actions.size(), 1u);
  const Action & a = t.actions[0];
  EXPECT_EQ(a.name, "act");
  EXPECT_EQ(a.parameters, 1u);
  EXPECT_EQ(a.hidden, 0u);
  EXPECT_EQ(cond(t, a.precondition, a.variables), "(and (p ?x) (not (q ?x)))");
  ASSERT_EQ(a.effects.size(), 3u);
  EXPECT_EQ(a.effects[0].kind, Effect::Kind::Add);
  EXPECT_EQ(a.effects[1].kind, Effect::Kind::Delete);
  EXPECT_EQ(a.effects[2].kind, Effect::Kind::Numeric);
  EXPECT_EQ(a.effects[2].op, ast::AssignOp::Increase);
  EXPECT_TRUE(t.axioms.empty());
}

TEST(NormalizeTest, NegationIsPushedToAtomsAndComparisons)
{
  auto n = normalize_text(domain_with(
      "(:action act :parameters (?x ?y - item) :precondition (not (or (p ?x) (< (f ?x) 2)"
      " (= ?x ?y) (not (q ?y)))))"), kProblem);
  const Action & a = n->task.actions.at(0);
  EXPECT_EQ(cond(n->task, a.precondition, a.variables),
    "(and (not (p ?x)) (q ?y) (>= (f ?x) 2) (!= ?x ?y))");
}

TEST(NormalizeTest, ImplyBecomesADisjunction)
{
  auto n = normalize_text(domain_with(
      "(:action act :parameters (?x - item) :precondition (imply (p ?x) (q ?x)))"), kProblem);
  ASSERT_EQ(n->task.actions.size(), 2u);
  const Task & t = n->task;
  EXPECT_EQ(cond(t, t.actions[0].precondition, t.actions[0].variables), "(not (p ?x))");
  EXPECT_EQ(cond(t, t.actions[1].precondition, t.actions[1].variables), "(q ?x)");
}

TEST(NormalizeTest, SmallDisjunctionsSplitTheAction)
{
  auto n = normalize_text(domain_with(
      "(:action act :parameters (?x - item) :precondition (and (or (p ?x) (q ?x))"
      " (or (z) (r ?x ?x))) :effect (z))"), kProblem);
  EXPECT_EQ(n->task.actions.size(), 4u);  // 2 x 2
  for (const auto & a : n->task.actions) {
    EXPECT_EQ(a.name, "act");
    EXPECT_EQ(a.effects.size(), 1u);
  }
  EXPECT_TRUE(n->task.axioms.empty());
}

TEST(NormalizeTest, BigDisjunctionsBecomeAuxiliaryPredicates)
{
  NormalizeOptions small;
  small.max_disjuncts = 3;
  auto n = normalize_text(domain_with(
      "(:action act :parameters (?x ?y - item) :precondition (and (or (p ?x) (q ?x))"
      " (or (p ?y) (q ?y) (r ?x ?y))))"), kProblem, small);
  // 2 x 3 = 6 > 3: the widest disjunction becomes an auxiliary predicate with 3 rules
  EXPECT_EQ(n->task.actions.size(), 2u);
  ASSERT_EQ(n->task.axioms.size(), 3u);
  const Axiom & ax = n->task.axioms[0];
  EXPECT_EQ(n->task.predicate_names.at(ax.head.index()), "kairos-aux-0");
  EXPECT_TRUE(n->task.derived.at(ax.head.index()));
  EXPECT_EQ(ax.head_arity, 2u);  // its free variables ?x ?y
}

TEST(NormalizeTest, ExistentialsBecomeHiddenParameters)
{
  auto n = normalize_text(domain_with(
      "(:action act :parameters (?x - item) :precondition (exists (?y - item) (r ?x ?y))"
      " :effect (q ?x))"), kProblem);
  const Action & a = n->task.actions.at(0);
  EXPECT_EQ(a.parameters, 1u);
  EXPECT_EQ(a.hidden, 1u);
  EXPECT_EQ(a.variables.at(1).name, "?y");
  EXPECT_EQ(cond(n->task, a.precondition, a.variables), "(r ?x ?y)");
}

TEST(NormalizeTest, ExistentialsWithSeveralVariablesBecomeAuxiliaryPredicates)
{
  auto n = normalize_text(domain_with(
      "(:action act :parameters (?x - item) :precondition (exists (?y ?w - item)"
      " (and (r ?x ?y) (r ?y ?w))) :effect (q ?x))"), kProblem);
  const Task & t = n->task;
  const Action & a = t.actions.at(0);
  EXPECT_EQ(a.hidden, 0u);
  EXPECT_EQ(cond(t, a.precondition, a.variables), "(kairos-aux-0 ?x)");
  ASSERT_EQ(t.axioms.size(), 1u);
  EXPECT_EQ(t.axioms[0].existential, 2u);

  // The limit is an option
  NormalizeOptions o;
  o.max_hidden_variables = 2;
  auto m = normalize_text(domain_with(
      "(:action act :parameters (?x - item) :precondition (exists (?y ?w - item)"
      " (and (r ?x ?y) (r ?y ?w))) :effect (q ?x))"), kProblem, o);
  EXPECT_EQ(m->task.actions.at(0).hidden, 2u);
  EXPECT_TRUE(m->task.axioms.empty());
}

TEST(NormalizeTest, QuantifiedVariablesTheBodyNeverReadsAreDropped)
{
  // As PlanSys2's suave domain: (exists (?a ?f1 ?f2 ?fd1 ?fd2) ...) without ?fd1 ?fd2 in the
  // body; kept, they multiplied the instances by every pair of objects
  auto n = normalize_text(domain_with(
      "(:action act :parameters (?x - item) :precondition (exists (?y ?u ?v - item) (r ?x ?y))"
      " :effect (q ?x))"
      " (:action all :parameters (?x - item) :precondition (forall (?u - item) (p ?x))"
      " :effect (z))"), kProblem);
  const Task & t = n->task;
  EXPECT_TRUE(t.axioms.empty());
  const Action & a = t.actions.at(0);
  EXPECT_EQ(a.hidden, 1u);
  EXPECT_EQ(cond(t, a.precondition, a.variables), "(r ?x ?y)");
  const Action & b = t.actions.at(1);
  EXPECT_EQ(cond(t, b.precondition, b.variables), "(p ?x)");
}

TEST(NormalizeTest, UnusedVariablesOfATypeWithoutObjects)
{
  // No witness: the existential is false, the universal true
  const char * domain =
    "(define (domain d) (:requirements :adl :typing) (:types item ghost)"
    " (:predicates (p ?x - item) (q ?x - item) (z))"
    " (:action haunted :parameters (?x - item) :precondition (and (p ?x)"
    "  (exists (?g - ghost) (q ?x))) :effect (z))"
    " (:action vacuous :parameters (?x - item) :precondition (and (p ?x)"
    "  (forall (?g - ghost) (q ?x))) :effect (z)))";
  const char * problem = "(define (problem pr) (:domain d) (:objects a b - item)"
    " (:init (p a)) (:goal (z)))";
  auto n = normalize_text(domain, problem);
  const Task & t = n->task;
  std::map<std::string, std::string> pre;
  for (const auto & a : t.actions) {
    pre[a.name] = cond(t, a.precondition, a.variables);
  }
  EXPECT_EQ(pre.count("haunted"), 0u) << pre["haunted"];  // never applicable
  EXPECT_EQ(pre["vacuous"], "(p ?x)");
}

TEST(NormalizeTest, ForallStaysWhenItsBodyIsConjunctive)
{
  auto n = normalize_text(domain_with(
      "(:action act :parameters (?x - item) :precondition (forall (?y - item) (and (p ?y)"
      " (not (r ?x ?y)))))"), kProblem);
  const Action & a = n->task.actions.at(0);
  ASSERT_EQ(a.precondition.foralls.size(), 1u);
  EXPECT_TRUE(n->task.axioms.empty());
}

TEST(NormalizeTest, ForallWithDisjunctiveBodyBecomesANegatedExistential)
{
  // forall y (p y | r x y) = not exists y (not p y & not r x y)
  auto n = normalize_text(domain_with(
      "(:action act :parameters (?x - item) :precondition (forall (?y - item) (or (p ?y)"
      " (r ?x ?y))))"), kProblem);
  const Task & t = n->task;
  const Action & a = t.actions.at(0);
  EXPECT_TRUE(a.precondition.foralls.empty());
  EXPECT_EQ(cond(t, a.precondition, a.variables), "(not (kairos-aux-0 ?x))");
  ASSERT_EQ(t.axioms.size(), 1u);
  EXPECT_EQ(t.axioms[0].head_arity, 1u);
  EXPECT_EQ(t.axioms[0].existential, 1u);
  EXPECT_EQ(cond(t, t.axioms[0].body, t.axioms[0].variables), "(and (not (p ?y)) (not (r ?x ?y)))");
}

TEST(NormalizeTest, ForallWithDisjunctiveBodyInsideAxiomsKeepsAPositiveAuxiliary)
{
  // Negating there could close a cycle through negation (d depends on itself)
  auto n = normalize_text(
    "(define (domain d) (:requirements :adl :derived-predicates) (:predicates (p ?x) (d ?x))"
    " (:derived (d ?x) (forall (?y) (or (p ?y) (d ?y)))))",
    "(define (problem pr) (:domain d) (:objects a b) (:init) (:goal (d a)))");
  const Task & t = n->task;
  ASSERT_EQ(t.axioms.size(), 3u);  // d :- forall y aux(y); aux(y) :- p y | d y
  size_t foralls = 0;
  for (const auto & ax : t.axioms) {
    if (t.predicate_names[ax.head.index()] == "d") {
      foralls += ax.body.foralls.size();
    }
  }
  EXPECT_EQ(foralls, 1u);
  for (const auto & ax : t.axioms) {
    for (const auto & l : ax.body.literals) {
      EXPECT_TRUE(l.positive);
    }
  }
}

TEST(NormalizeTest, NegatedForallIsAnExistential)
{
  auto n = normalize_text(domain_with(
      "(:action act :parameters () :precondition (not (forall (?y - item) (p ?y))))"),
    kProblem);
  const Action & a = n->task.actions.at(0);
  EXPECT_EQ(a.hidden, 1u);
  EXPECT_EQ(cond(n->task, a.precondition, a.variables), "(not (p ?y))");
}

TEST(NormalizeTest, AxiomRulesSeparateExistentialAndForallVariables)
{
  // Found by the random walks on IPC 2000 elevator: the rule
  //   aux(?f) :- forall (?p - never_alone) ...
  // must be true when there are no never_alone objects; ?p is not chosen by the rule
  auto n = normalize_text(domain_with(
      "(:action act :parameters (?x - item) :precondition (or (forall (?y - item) (p ?y))"
      " (exists (?w - item) (r ?x ?w))))"), kProblem, NormalizeOptions{1});
  ASSERT_FALSE(n->task.axioms.empty());
  bool found = false;
  for (const auto & ax : n->task.axioms) {
    EXPECT_LE(ax.head_arity + ax.existential, ax.variables.size());
    if (!ax.body.foralls.empty()) {
      EXPECT_EQ(ax.existential, 0u);  // the forall variable comes after
      found = true;
    } else if (!ax.body.literals.empty()) {
      EXPECT_EQ(ax.existential, 1u);  // ?w
    }
  }
  EXPECT_TRUE(found);
}

TEST(NormalizeTest, DisjunctionsLargerThanTheLimitTerminate)
{
  // An "or" with more cases than the limit becomes an aux predicate with one rule per case
  NormalizeOptions tiny;
  tiny.max_disjuncts = 1;
  auto n = normalize_text(domain_with(
      "(:action act :parameters (?x - item) :precondition (or (p ?x) (q ?x) (z)"
      " (and (or (p ?x) (q ?x)) (or (z) (r ?x ?x)))))"), kProblem, tiny);
  EXPECT_EQ(n->task.actions.size(), 1u);
  EXPECT_GE(n->task.axioms.size(), 4u);
  std::string many = "(or";
  for (int i = 0; i < 40; ++i) {
    many += i % 2 ? " (p ?x)" : " (q ?x)";
  }
  many += ")";
  auto big = normalize_text(domain_with(
      "(:action act :parameters (?x - item) :precondition " + many + ")"), kProblem);
  EXPECT_EQ(big->task.actions.size(), 1u);
  EXPECT_EQ(big->task.axioms.size(), 40u);
}

TEST(NormalizeTest, ActionsWithoutVariables)
{
  // Found normalizing ENHSP's car_nl: numeric comparisons in actions without any variable
  auto n = normalize_text(domain_with(
      "(:action act :parameters () :precondition (and (> (g) 0) (or (z) (< (g) 5)))"
      " :effect (increase (g) 1))"), kProblem);
  EXPECT_EQ(n->task.actions.size(), 2u);
  EXPECT_TRUE(n->task.actions[0].variables.empty());
}

TEST(NormalizeTest, EffectsAreFlattened)
{
  auto n = normalize_text(domain_with(
      "(:action act :parameters (?x - item) :effect (and (z) (forall (?y - item)"
      " (when (and (r ?x ?y) (p ?y)) (and (not (p ?y)) (q ?y))))))"), kProblem);
  const Action & a = n->task.actions.at(0);
  ASSERT_EQ(a.effects.size(), 3u);
  EXPECT_TRUE(a.effects[0].variables.empty());
  ASSERT_EQ(a.effects[1].variables.size(), 1u);
  EXPECT_EQ(cond(n->task, a.effects[1].condition, a.variables), "(and (r ?x ?y) (p ?y))");
  EXPECT_EQ(a.effects[2].condition, a.effects[1].condition);
}

TEST(NormalizeTest, DisjunctiveEffectConditionsAreNeverSplit)
{
  // Splitting (when (or a b) (increase ...)) would increase twice when both hold
  auto n = normalize_text(domain_with(
      "(:action act :parameters (?x - item) :effect (when (or (p ?x) (q ?x))"
      " (increase (g) 1)))"), kProblem);
  const Action & a = n->task.actions.at(0);
  ASSERT_EQ(a.effects.size(), 1u);
  EXPECT_EQ(a.effects[0].condition.literals.size(), 1u);  // the auxiliary predicate
  EXPECT_EQ(n->task.axioms.size(), 2u);
}

TEST(NormalizeTest, DerivedPredicatesBecomeAxioms)
{
  auto n = normalize_text(
    "(define (domain d) (:requirements :derived-predicates :typing) (:types n)"
    " (:predicates (link ?a ?b - n) (reach ?a ?b - n))"
    " (:derived (reach ?a ?b) (or (link ?a ?b) (exists (?c - n) (and (link ?a ?c)"
    " (reach ?c ?b))))))",
    "(define (problem p) (:domain d) (:objects x y - n) (:goal (reach x y)))");
  const Task & t = n->task;
  ASSERT_EQ(t.axioms.size(), 2u);
  EXPECT_EQ(t.axioms[0].head_arity, 2u);
  EXPECT_EQ(t.axioms[1].variables.size(), 3u);  // ?a ?b and the free ?c
  EXPECT_EQ(t.axioms[1].variables[0].types, t.axioms[0].variables[0].types);
}

TEST(NormalizeTest, DurativeActionIsSplitByTime)
{
  auto n = normalize_text(
    "(define (domain t) (:requirements :durative-actions :typing :fluents :conditional-effects)"
    " (:types r l) (:predicates (at ?r - r ?l - l) (link ?a ?b - l) (busy))"
    " (:functions (d ?a ?b - l) (e))"
    " (:durative-action go :parameters (?r - r ?a ?b - l)"
    "  :duration (and (<= 2 ?duration) (<= ?duration (d ?a ?b)))"
    "  :condition (and (at start (at ?r ?a)) (over all (link ?a ?b)) (at end (not (busy))))"
    "  :effect (and (at start (not (at ?r ?a))) (at end (at ?r ?b))"
    "               (decrease (e) (* #t 2)) (when (at start (busy)) (at end (busy))))))",
    "(define (problem p) (:domain t) (:objects r1 - r a b - l) (:goal (busy)))");
  ASSERT_EQ(n->task.durative_actions.size(), 1u);
  const DurativeAction & a = n->task.durative_actions[0];
  const Task & t = n->task;
  EXPECT_EQ(cond(t, a.at_start, a.variables), "(at ?r ?a)");
  EXPECT_EQ(cond(t, a.over_all, a.variables), "(link ?a ?b)");
  EXPECT_EQ(cond(t, a.at_end, a.variables), "(not (busy))");
  ASSERT_EQ(a.duration.size(), 2u);
  EXPECT_EQ(a.duration[0].op, CompareOp::Ge);  // (<= 2 ?duration) is ?duration >= 2
  EXPECT_EQ(a.duration[1].op, CompareOp::Le);
  ASSERT_EQ(a.effects.size(), 4u);
  EXPECT_EQ(a.effects[0].timing, Timing::Start);
  EXPECT_EQ(a.effects[1].timing, Timing::End);
  EXPECT_TRUE(a.effects[2].continuous);
  EXPECT_EQ(a.effects[3].timing, Timing::End);
  EXPECT_EQ(cond(t, a.effects[3].start_condition, a.variables), "(busy)");
  EXPECT_TRUE(a.effects[3].condition.empty());
}

TEST(NormalizeTest, GoalAndMetric)
{
  auto simple = normalize_text(domain_with(""),
      "(define (problem p) (:domain d) (:objects a - item) (:goal (and (p a) (not (z)))))");
  EXPECT_EQ(cond(simple->task, simple->task.goal, {}), "(and (p a) (not (z)))");
  EXPECT_TRUE(simple->task.metric.is_default);
  EXPECT_EQ(simple->task.metric.expr.op, ExprOp::StepCount);

  auto complex = normalize_text(domain_with(""),
      "(define (problem p) (:domain d) (:objects a b - item) (:goal (or (p a) (p b)))"
      " (:metric maximize (g)))");
  EXPECT_EQ(complex->task.goal.literals.size(), 1u);  // an auxiliary goal predicate
  EXPECT_EQ(complex->task.axioms.size(), 2u);
  EXPECT_FALSE(complex->task.metric.minimize);
  EXPECT_EQ(complex->task.metric.expr.op, ExprOp::Fluent);
}

TEST(NormalizeTest, InitialStateAndTimedEvents)
{
  auto n = normalize_text(domain_with(""),
      "(define (problem p) (:domain d) (:objects a b - item)"
      " (:init (p a) (= (f a) 4) (at 5 (not (p a))) (at 2 (q b)) (at 7 (= (g) 1))) (:goal (z)))");
  const Task & t = n->task;
  EXPECT_EQ(t.init_facts.size(), 1u);
  EXPECT_EQ(t.init_fluents.size(), 1u);
  ASSERT_EQ(t.timed_events.size(), 3u);
  EXPECT_EQ(t.timed_events[0].time, 2.0);  // sorted
  EXPECT_TRUE(t.timed_events[1].negated);
  EXPECT_TRUE(t.timed_events[2].fluent.has_value());
}

TEST(NormalizeTest, PreferencesAreSetAside)
{
  auto n = normalize_text(domain_with(
      "(:action act :parameters (?x - item) :precondition (and (p ?x) (preference pp (q ?x))))"),
    "(define (problem p) (:domain d) (:objects a - item) (:goal (and (z) (preference g1 (p a)))))");
  EXPECT_TRUE(n->task.has_soft_goals);
  EXPECT_EQ(cond(n->task, n->task.actions[0].precondition, n->task.actions[0].variables),
    "(p ?x)");
}

// --- Equivalence with the original domain, judged by the validator -----------------------

// Every grounding of every original action: applicable in the original domain (validator, one
// step after `prefix`) if and only if applicable in the normalized task (simulator)
struct Walk
{
  std::string domain;
  std::string problem;
  std::unique_ptr<kairos::testing::Normalized> n;
  std::unique_ptr<Simulator> sim;
  std::string prefix;
  SimState state;
  size_t checked = 0;
  size_t applicable = 0;
};

std::string step_text(const std::string & name, const std::vector<uint32_t> & args, const Task & t)
{
  std::string s = "(" + name;
  for (uint32_t a : args) {
    s += " " + t.object_names[a];
  }
  return s + ")";
}

bool original_applicable(const Walk & w, const std::string & step)
{
  auto plan = parse_plan(w.prefix + step + "\n");
  ValidateOptions o;
  o.check_goal = false;
  return validate_plan(w.domain, w.problem, *plan, o).valid();
}

// One random walk; returns false on the first disagreement (with a message)
::testing::AssertionResult random_walk(
  const std::string & domain, const std::string & problem, uint32_t seed, size_t steps,
  size_t max_candidates)
{
  Walk w;
  w.domain = domain;
  w.problem = problem;
  w.n = normalize_text(domain, problem);
  w.sim = std::make_unique<Simulator>(w.n->task);
  w.state = w.sim->initial();
  const Task & t = w.n->task;
  std::mt19937 rng(seed);

  // Original action signatures (name and parameter types)
  std::map<std::string, std::vector<TypeSpec>> signatures;
  for (const auto & a : t.actions) {
    std::vector<TypeSpec> types;
    for (uint32_t i = 0; i < a.parameters; ++i) {
      types.push_back(a.variables[i].types);
    }
    signatures.emplace(a.name, types);
  }

  for (size_t step = 0; step < steps; ++step) {
    std::vector<std::pair<std::string, std::vector<uint32_t>>> candidates;
    for (const auto & [name, types] : signatures) {
      std::vector<uint32_t> args(types.size());
      std::function<void(size_t)> rec = [&](size_t i) {
          if (candidates.size() > 20000) {
            return;
          }
          if (i == types.size()) {
            candidates.emplace_back(name, args);
            return;
          }
          for (uint32_t o : w.sim->objects_of(types[i])) {
            args[i] = o;
            rec(i + 1);
          }
        };
      rec(0);
    }
    std::shuffle(candidates.begin(), candidates.end(), rng);
    if (candidates.size() > max_candidates) {
      candidates.resize(max_candidates);
    }
    std::vector<std::pair<std::string, std::vector<uint32_t>>> applicable;
    for (const auto & [name, args] : candidates) {
      std::string text = step_text(name, args, t);
      bool orig = original_applicable(w, text);
      bool norm = w.sim->apply(name, args, w.state, nullptr);
      ++w.checked;
      if (orig != norm) {
        return ::testing::AssertionFailure() << "after\n" << w.prefix << text << " is "
                                             << (orig ? "" : "not ") << "applicable in the "
               "original domain but " << (norm ? "" : "not ") << "in the normalized task";
      }
      if (orig) {
        applicable.emplace_back(name, args);
      }
    }
    if (applicable.empty()) {
      break;
    }
    // Take a random applicable action in both
    auto chosen = applicable[std::uniform_int_distribution<size_t>(0, applicable.size() - 1)(rng)];
    SimState next;
    w.sim->apply(chosen.first, chosen.second, w.state, &next);
    w.state = next;
    w.prefix += step_text(chosen.first, chosen.second, t) + "\n";
  }
  // The goal: same verdict at the end of the walk
  auto plan = parse_plan(w.prefix);
  bool orig_goal = validate_plan(w.domain, w.problem, *plan).valid();
  if (orig_goal != w.sim->goal(w.state)) {
    return ::testing::AssertionFailure() << "goal verdicts differ after\n" << w.prefix;
  }
  if (w.checked == 0) {
    return ::testing::AssertionFailure() << "no candidates checked";
  }
  return ::testing::AssertionSuccess();
}

// A domain using everything the normalizer transforms
const char * kAdlDomain =
  "(define (domain adl) (:requirements :adl :typing :derived-predicates :fluents)\n"
  " (:types room box - object robot)\n"
  " (:predicates (at ?r - robot ?l - room) (in ?b - box ?l - room) (holding ?r - robot ?b - box)\n"
  "   (link ?a ?b - room) (dark ?l - room) (open ?l - room) (reach ?a ?b - room) (done))\n"
  " (:functions (energy ?r - robot))\n"
  " (:derived (reach ?a ?b) (or (link ?a ?b) (exists (?c - room) (and (link ?a ?c)\n"
  "   (reach ?c ?b)))))\n"
  " (:action move :parameters (?r - robot ?a ?b - room)\n"
  "  :precondition (and (at ?r ?a) (or (link ?a ?b) (link ?b ?a))\n"
  "    (imply (dark ?b) (exists (?x - box) (holding ?r ?x))) (> (energy ?r) 0))\n"
  "  :effect (and (not (at ?r ?a)) (at ?r ?b) (decrease (energy ?r) 1)\n"
  "    (forall (?x - box) (when (holding ?r ?x) (and (not (in ?x ?a)) (in ?x ?b))))))\n"
  " (:action pick :parameters (?r - robot ?x - box ?l - room)\n"
  "  :precondition (and (at ?r ?l) (in ?x ?l) (not (exists (?y - box) (holding ?r ?y))))\n"
  "  :effect (holding ?r ?x))\n"
  " (:action drop :parameters (?r - robot ?x - box ?l - room)\n"
  "  :precondition (and (at ?r ?l) (holding ?r ?x))\n"
  "  :effect (and (not (holding ?r ?x)) (when (or (dark ?l) (open ?l)) (done))))\n"
  " (:action light :parameters (?r - robot ?l - room)\n"
  "  :precondition (and (at ?r ?l) (forall (?x - box) (or (in ?x ?l) (not (reach ?l ?l)))))\n"
  "  :effect (and (not (dark ?l)) (when (not (open ?l)) (open ?l)))))\n";

const char * kAdlProblem =
  "(define (problem p) (:domain adl) (:objects r1 - robot k h c - room b1 b2 - box)\n"
  " (:init (at r1 k) (in b1 k) (in b2 h) (link k h) (link h c) (link c k) (dark c)\n"
  "        (= (energy r1) 6))\n"
  " (:goal (and (done) (in b1 c))))\n";

TEST(NormalizeEquivalenceTest, RandomWalksOnAnAdlDomain)
{
  for (uint32_t seed = 0; seed < 12; ++seed) {
    EXPECT_TRUE(random_walk(kAdlDomain, kAdlProblem, seed, 8, 60)) << "seed " << seed;
  }
}

TEST(NormalizeEquivalenceTest, RandomPreconditionsOnRandomStates)
{
  // Random formulas over a fixed vocabulary, checked on random initial states
  std::mt19937 rng(3);
  const std::vector<std::string> atoms = {"(p ?x)", "(q ?x)", "(p ?y)", "(r ?x ?y)", "(r ?y ?x)",
    "(z)", "(= ?x ?y)", "(< (f ?x) 2)", "(>= (f ?y) (f ?x))"};
  std::function<std::string(int)> formula = [&](int depth) -> std::string {
      int kind = std::uniform_int_distribution<int>(0, depth > 2 ? 0 : 6)(rng);
      auto atom = [&]() {
          return atoms[std::uniform_int_distribution<size_t>(0, atoms.size() - 1)(rng)];
        };
      switch (kind) {
        case 0: return atom();
        case 1: return "(not " + formula(depth + 1) + ")";
        case 2: return "(and " + formula(depth + 1) + " " + formula(depth + 1) + ")";
        case 3: return "(or " + formula(depth + 1) + " " + formula(depth + 1) + ")";
        case 4: return "(imply " + formula(depth + 1) + " " + formula(depth + 1) + ")";
        case 5: return "(exists (?w - item) (or (r ?x ?w) " + formula(depth + 1) + "))";
        default: return "(forall (?w - item) (or (p ?w) " + formula(depth + 1) + "))";
      }
    };
  const std::vector<std::string> facts = {"(p a)", "(p b)", "(q a)", "(q c)", "(r a b)",
    "(r b a)", "(r c c)", "(z)"};
  for (int i = 0; i < 40; ++i) {
    std::string pre = formula(0);
    std::string domain = domain_with("(:action act :parameters (?x ?y - item) :precondition " +
        pre + " :effect (z))");
    std::string init;
    for (const auto & f : facts) {
      if (std::uniform_int_distribution<int>(0, 1)(rng)) {
        init += " " + f;
      }
    }
    std::string problem = "(define (problem pr) (:domain d) (:objects a b c - item) (:init" +
      init + " (= (f a) 1) (= (f b) 2) (= (f c) 3) (= (g) 0)) (:goal (z)))";
    EXPECT_TRUE(random_walk(domain, problem, static_cast<uint32_t>(i), 1, 9)) << pre;
  }
}

TEST(NormalizeTest, SuaveFromPlanSys2)
{
  // 55 derived predicates with disjunctions and existentials. Their equivalence is checked
  // after grounding (step 8): the test simulator enumerates bindings naively and is too slow
  // for this domain.
  auto n = normalize_text(
    read_file(kairos::testing::data_path("plansys2/plansys2_executor__suave_domain.pddl")),
    read_file(kairos::testing::data_path("plansys2/plansys2_executor__suave_problem.pddl")));
  const Task & t = n->task;
  EXPECT_GE(t.axioms.size(), 55u);
  EXPECT_FALSE(t.actions.empty());
  for (const auto & ax : t.axioms) {
    EXPECT_TRUE(t.derived.at(ax.head.index()));
    EXPECT_LE(ax.head_arity, ax.variables.size());
  }
  EXPECT_NE(to_string(t).find("(:axiom"), std::string::npos);
}

// IPC ADL and derived-predicate domains, when the corpus is available
TEST(NormalizeEquivalenceTest, IpcAdlDomains)
{
  const char * dir = std::getenv("KAIROS_PDDL_CORPUS");
  if (dir == nullptr) {
    GTEST_SKIP() << "KAIROS_PDDL_CORPUS not set";
  }
  std::filesystem::path base = std::filesystem::path(dir) / "potassco_pddl-instances";
  const std::vector<std::string> domains = {
    "ipc-1998/domains/assembly-round-1-adl", "ipc-2000/domains/elevator-adl-full-typed",
    "ipc-2000/domains/schedule-adl-typed", "ipc-2004/domains/psr-small-strips",
    "ipc-2004/domains/promela-dining-philosophers-derived-predicates-adl",
    "ipc-2004/domains/airport-nontemporal-adl", "ipc-2006/domains/openstacks-propositional",
    "ipc-2006/domains/trucks-propositional",
    "ipc-2008/domains/openstacks-sequential-satisficing-adl"};
  size_t tried = 0;
  for (const auto & dom : domains) {
    auto d = base / dom / "domain.pddl";
    auto p = base / dom / "instances" / "instance-1.pddl";
    if (!std::filesystem::exists(d) || !std::filesystem::exists(p)) {
      continue;
    }
    ++tried;
    EXPECT_TRUE(random_walk(read_file(d), read_file(p), 1, 5, 25)) << dom;
  }
  EXPECT_GT(tried, 0u);
}

}  // namespace
}  // namespace kairos::lifted
