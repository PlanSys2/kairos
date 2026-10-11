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
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <functional>
#include <map>
#include <memory>
#include <random>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "ground/grounder.hpp"
#include "kairos/plan.hpp"
#include "kairos/validate.hpp"
#include "support/ground_simulator.hpp"
#include "support/pddl_helpers.hpp"
#include "support/task_helpers.hpp"

namespace kairos::ground
{
namespace
{

using kairos::testing::data_path;
using kairos::testing::dump;
using kairos::testing::normalize_text;
using kairos::testing::read_file;
using testing::GroundSimulator;
using testing::GState;

struct Grounded
{
  std::unique_ptr<kairos::testing::Normalized> n;
  Task task;
  GroundStats stats;
};

std::unique_ptr<Grounded> ground_text(
  const std::string & domain, const std::string & problem, const GroundOptions & o = {})
{
  auto g = std::make_unique<Grounded>();
  g->n = normalize_text(domain, problem);
  auto r = ground(g->n->task, o, nullptr, &g->stats);
  EXPECT_TRUE(r.ok()) << dump(r.diagnostics());
  if (r.ok()) {
    g->task = std::move(r).value();
  }
  return g;
}

std::set<std::string> atom_names(const Task & t)
{
  std::set<std::string> out;
  for (AtomId a = 0; a < t.atoms.size(); ++a) {
    out.insert(atom_name(t, a));
  }
  return out;
}

std::set<std::string> action_names(const Task & t)
{
  std::set<std::string> out;
  for (const auto & a : t.actions) {
    out.insert(instance_name(t, a));
  }
  return out;
}

std::string cond(const Task & t, const Condition & c)
{
  // The text printer shows conditions inside actions; reuse it through a one-action task
  Task copy = t;
  copy.actions = {Action{0, {}, c, {}, {}}};
  copy.action_names = {"x"};
  std::string s = to_string(copy);
  auto from = s.find(":precondition ") + 14;
  return s.substr(from, s.find('\n', from) - from);
}

std::string domain_with(const std::string & body, const std::string & constants = "")
{
  return "(define (domain d) (:requirements :adl :typing :fluents :derived-predicates)"
         " (:types item place - object box - item)" +
         (constants.empty() ? std::string() : " (:constants " + constants + ")") +
         " (:predicates (p ?x - item) (q ?x - item) (r ?x ?y - item) (link ?a ?b - place)"
         "   (at ?x - item ?l - place) (z) (blocked ?l - place))"
         " (:functions (f ?x - item) (g) (dist ?a ?b - place)) " + body + ")";
}

std::string problem_with(const std::string & init, const std::string & goal = "(z)")
{
  return "(define (problem pr) (:domain d) (:objects a b - item c - box l1 l2 l3 - place)"
         " (:init " + init + ") (:goal " + goal + "))";
}

// --- Reachability and statics ----------------------------------------------------------------

TEST(GroundTest, StaticPredicatesAreEvaluatedAndDisappear)
{
  auto g = ground_text(domain_with(
      "(:action move :parameters (?x - item ?a ?b - place)"
      " :precondition (and (at ?x ?a) (link ?a ?b)) :effect (and (not (at ?x ?a)) (at ?x ?b)))"),
    problem_with("(at a l1) (link l1 l2) (link l2 l3)"));
  const Task & t = g->task;
  EXPECT_EQ(action_names(t), (std::set<std::string>{"(move a l1 l2)", "(move a l2 l3)"}));
  EXPECT_EQ(atom_names(t), (std::set<std::string>{"(at a l1)", "(at a l2)", "(at a l3)"}));
  for (const auto & a : t.actions) {
    EXPECT_EQ(a.precondition.positive.size(), 1u);  // (link ...) is gone
  }
  ASSERT_EQ(t.init_atoms.size(), 1u);
  EXPECT_EQ(atom_name(t, t.init_atoms[0]), "(at a l1)");
}

TEST(GroundTest, TypesRestrictParametersIncludingSubtypes)
{
  // ?x - box: only c; ?y - item: a, b and c (box is an item)
  auto g = ground_text(domain_with(
      "(:action pb :parameters (?x - box) :precondition () :effect (p ?x))"
      " (:action pi :parameters (?y - item) :precondition () :effect (q ?y))"),
    problem_with(""));
  EXPECT_EQ(action_names(g->task),
    (std::set<std::string>{"(pb c)", "(pi a)", "(pi b)", "(pi c)"}));
}

TEST(GroundTest, EitherTypesAcceptEitherKind)
{
  auto g = ground_text(
    "(define (domain d) (:requirements :typing) (:types t1 t2 t3)"
    " (:predicates (seen ?x - object))"
    " (:action look :parameters (?x - (either t1 t2)) :precondition () :effect (seen ?x)))",
    "(define (problem p) (:domain d) (:objects x1 - t1 x2 - t2 x3 - t3) (:init) (:goal (and)))");
  EXPECT_EQ(action_names(g->task), (std::set<std::string>{"(look x1)", "(look x2)"}));
}

TEST(GroundTest, NegativeStaticLiteralsAndInequalitiesPrune)
{
  auto g = ground_text(domain_with(
      "(:action go :parameters (?a ?b - place)"
      " :precondition (and (not (= ?a ?b)) (not (blocked ?b))) :effect (z))"),
    problem_with("(blocked l3)"));
  EXPECT_EQ(action_names(g->task), (std::set<std::string>{"(go l1 l2)", "(go l2 l1)",
      "(go l3 l1)", "(go l3 l2)"}));
  for (const auto & a : g->task.actions) {
    EXPECT_TRUE(a.precondition.empty());
  }
}

TEST(GroundTest, EqualityWithConstantsAndSameVariables)
{
  auto g = ground_text(domain_with(
      "(:action same :parameters (?x ?y - item) :precondition (= ?x ?y) :effect (p ?x))"
      " (:action notk :parameters (?x - item) :precondition (not (= ?x k)) :effect (q ?x))",
      "k - item"), problem_with(""));
  EXPECT_EQ(action_names(g->task), (std::set<std::string>{"(same a a)", "(same b b)",
      "(same c c)", "(same k k)", "(notk a)", "(notk b)", "(notk c)"}));
}

TEST(GroundTest, UnreachableAtomsAndActionsAreLeftOut)
{
  // q needs p, p is never added: neither the actions nor q atoms exist; (not (q ?x)) is true
  auto g = ground_text(domain_with(
      "(:action mk :parameters (?x - item) :precondition (p ?x) :effect (q ?x))"
      " (:action ok :parameters (?x - item) :precondition (not (q ?x)) :effect (r ?x ?x))"),
    problem_with(""));
  const Task & t = g->task;
  EXPECT_EQ(action_names(t), (std::set<std::string>{"(ok a)", "(ok b)", "(ok c)"}));
  for (const auto & a : t.actions) {
    EXPECT_TRUE(a.precondition.empty());
  }
  EXPECT_EQ(atom_names(t), (std::set<std::string>{"(r a a)", "(r b b)", "(r c c)"}));
}

TEST(GroundTest, ReachabilityChainsThroughSeveralActions)
{
  auto g = ground_text(domain_with(
      "(:action s1 :parameters (?x - item) :precondition (p ?x) :effect (q ?x))"
      " (:action s2 :parameters (?x ?y - item) :precondition (and (q ?x) (q ?y))"
      "   :effect (r ?x ?y))"
      " (:action s3 :parameters (?x - item) :precondition (r ?x ?x) :effect (z))"),
    problem_with("(p a) (p b)"));
  const Task & t = g->task;
  EXPECT_TRUE(action_names(t).count("(s3 a)"));
  EXPECT_FALSE(action_names(t).count("(s3 c)"));
  EXPECT_TRUE(atom_names(t).count("(z)"));
  EXPECT_FALSE(atom_names(t).count("(q c)"));
  EXPECT_EQ(t.actions.size(), 2u + 4u + 2u);
}

TEST(GroundTest, ForallInPreconditionsIsExpanded)
{
  auto g = ground_text(domain_with(
      "(:action all :parameters () :precondition (forall (?x - item) (p ?x)) :effect (z))"
      " (:action mk :parameters (?x - item) :precondition () :effect (p ?x))"),
    problem_with(""));
  const Task & t = g->task;
  const Action * all = nullptr;
  for (const auto & a : t.actions) {
    if (t.action_names[a.schema] == "all") {
      all = &a;
    }
  }
  ASSERT_NE(all, nullptr);
  EXPECT_EQ(cond(t, all->precondition), "(and (p a) (p b) (p c))");
}

TEST(GroundTest, ForallOverStaticFactsPrunesOrDisappears)
{
  // link is static: the forall holds from l1 only
  auto g = ground_text(domain_with(
      "(:action hub :parameters (?a - place)"
      " :precondition (forall (?b - place) (or (= ?a ?b) (link ?a ?b))) :effect (z))"),
    problem_with("(link l1 l2) (link l1 l3) (link l2 l1)"));
  ASSERT_EQ(action_names(g->task), (std::set<std::string>{"(hub l1)"}));
  EXPECT_TRUE(g->task.actions[0].precondition.empty());
}

TEST(GroundTest, ConditionalAndUniversalEffects)
{
  auto g = ground_text(domain_with(
      "(:action sweep :parameters (?l - place)"
      " :precondition () :effect (forall (?x - item) (when (at ?x ?l) (not (at ?x ?l)))))"),
    problem_with("(at a l1) (at c l2)"));
  const Task & t = g->task;
  // Only atoms that can be true get an effect; (sweep l3) changes nothing and is dropped
  EXPECT_EQ(action_names(t), (std::set<std::string>{"(sweep l1)", "(sweep l2)"}));
  for (const auto & a : t.actions) {
    EXPECT_EQ(a.effects.size(), 1u);
  }
}

TEST(GroundTest, DeletingAndAddingTheSameAtomLeavesItTrue)
{
  // PDDL applies deletes first; the compact form keeps only the add
  auto g = ground_text(domain_with(
      "(:action re :parameters (?x - item) :precondition (p ?x)"
      " :effect (and (not (p ?x)) (p ?x) (q ?x) (q ?x)))"),
    problem_with("(p a)"));
  ASSERT_EQ(g->task.actions.size(), 1u);
  const auto & c = g->task.actions[0].changes;
  EXPECT_TRUE(c.del.empty());
  EXPECT_EQ(c.add.size(), 2u);
}

TEST(GroundTest, ExistentialsBecomeHiddenParametersOfInstances)
{
  auto g = ground_text(domain_with(
      "(:action any :parameters () :precondition (exists (?x - item) (p ?x)) :effect (z))"),
    problem_with("(p a) (p b)"));
  const Task & t = g->task;
  // One instance per witness, both printed as (any)
  ASSERT_EQ(t.actions.size(), 2u);
  for (const auto & a : t.actions) {
    EXPECT_EQ(instance_name(t, a), "(any)");
    EXPECT_TRUE(a.args.empty());
  }
}

const char * kPathDomain =
  "(define (domain d) (:requirements :typing :derived-predicates :negative-preconditions)"
  " (:types n) (:predicates (e ?a ?b - n) (path ?a ?b - n) (on ?a - n))"
  " (:derived (path ?a ?b) (or (e ?a ?b) (exists (?c - n) (and (e ?a ?c) (path ?c ?b)))))"
  " (:action flip :parameters (?a - n) :precondition (not (path ?a ?a)) :effect (on ?a))%s)";

TEST(GroundTest, DerivedPredicatesGiveGroundAxioms)
{
  // e changes (join), so path is a real derived predicate
  char domain[1024];
  std::snprintf(domain, sizeof(domain), kPathDomain,
    " (:action join :parameters (?a ?b - n) :precondition (on ?a) :effect (e ?a ?b))");
  auto g = ground_text(domain,
      "(define (problem p) (:domain d) (:objects x y w - n) (:init (e x y) (e y x))"
      " (:goal (on w)))");
  const Task & t = g->task;
  std::set<std::string> heads;
  for (const auto & ax : t.axioms) {
    heads.insert(atom_name(t, ax.head));
    EXPECT_TRUE(t.is_derived(ax.head));
  }
  EXPECT_EQ(heads.size(), 9u);  // any pair, once join can link anything
  for (const auto & a : t.actions) {
    if (t.action_names[a.schema] == "flip") {
      EXPECT_EQ(a.precondition.negative.size(), 1u);
    }
  }
}

TEST(GroundTest, DerivedPredicatesOverStaticFactsAreStatic)
{
  char domain[1024];
  std::snprintf(domain, sizeof(domain), kPathDomain, "");
  auto g = ground_text(domain,
      "(define (problem p) (:domain d) (:objects x y w - n) (:init (e x y) (e y x))"
      " (:goal (on w)))");
  const Task & t = g->task;
  // path is computed while grounding: x and y are on a cycle, w is not
  EXPECT_TRUE(t.axioms.empty());
  EXPECT_EQ(action_names(t), (std::set<std::string>{"(flip w)"}));
  EXPECT_TRUE(t.actions[0].precondition.empty());
  EXPECT_EQ(atom_names(t), (std::set<std::string>{"(on w)"}));
}

TEST(GroundTest, GoalStates)
{
  auto reachable = ground_text(domain_with(
      "(:action mk :parameters () :precondition () :effect (z))"), problem_with("", "(z)"));
  EXPECT_FALSE(reachable->task.goal_unreachable);
  EXPECT_EQ(cond(reachable->task, reachable->task.goal), "(z)");

  auto unreachable = ground_text(domain_with(
      "(:action mk :parameters (?x - item) :precondition () :effect (p ?x))"),
    problem_with("", "(z)"));
  EXPECT_TRUE(unreachable->task.goal_unreachable);

  // Static goals are decided here
  auto st_true = ground_text(domain_with(""), problem_with("(link l1 l2)", "(link l1 l2)"));
  EXPECT_FALSE(st_true->task.goal_unreachable);
  EXPECT_TRUE(st_true->task.goal.empty());
  auto st_false = ground_text(domain_with(""), problem_with("", "(link l1 l2)"));
  EXPECT_TRUE(st_false->task.goal_unreachable);
}

// --- Numbers ----------------------------------------------------------------------------------

TEST(GroundTest, StaticFluentsBecomeConstants)
{
  auto g = ground_text(domain_with(
      "(:action go :parameters (?a ?b - place) :precondition (< (dist ?a ?b) 10)"
      " :effect (increase (g) (dist ?a ?b)))"),
    problem_with("(= (g) 0) (= (dist l1 l2) 3) (= (dist l2 l3) 20) (= (dist l3 l1) 5)"));
  const Task & t = g->task;
  // dist is static: undefined pairs and (dist l2 l3) >= 10 are pruned
  EXPECT_EQ(action_names(t), (std::set<std::string>{"(go l1 l2)", "(go l3 l1)"}));
  ASSERT_EQ(t.numeric_fluents.size(), 1u);  // only (g)
  for (const auto & a : t.actions) {
    EXPECT_TRUE(a.precondition.numeric.empty());
    ASSERT_EQ(a.effects.size(), 1u);
    EXPECT_EQ(a.effects[0].value.op, ExprOp::Constant);
  }
}

TEST(GroundTest, ChangingFluentsStayAndCountAsDefinedWhenAssigned)
{
  auto g = ground_text(domain_with(
      "(:action use :parameters (?x - item) :precondition (> (f ?x) 0)"
      " :effect (decrease (f ?x) 1))"
      " (:action set :parameters (?x - item) :precondition (p ?x) :effect (assign (f ?x) 5))"),
    problem_with("(= (f a) 2) (p b)"));
  // (f c) never gets a value: (use c) is impossible; (f b) gets one from (set b)
  EXPECT_EQ(action_names(g->task), (std::set<std::string>{"(use a)", "(use b)", "(set b)"}));
}

TEST(GroundTest, IncreasingAnUndefinedFluentIsImpossible)
{
  auto g = ground_text(domain_with(
      "(:action inc :parameters (?x - item) :precondition () :effect (increase (f ?x) 1))"),
    problem_with("(= (f a) 0)"));
  EXPECT_EQ(action_names(g->task), (std::set<std::string>{"(inc a)"}));
}

TEST(GroundTest, ActionsThatChangeNothingAreDropped)
{
  auto g = ground_text(
    "(define (domain d) (:requirements :action-costs) (:predicates (done) (ready))"
    " (:functions (total-cost) - number (x) - number)"
    " (:action idle :parameters () :precondition () :effect ())"
    " (:action pay :parameters () :precondition () :effect (increase (total-cost) 1))"
    " (:action count :parameters () :precondition () :effect (increase (x) 1))"
    " (:action fin :parameters () :precondition (ready) :effect (done))"
    " (:action prep :parameters () :precondition () :effect (and (ready)"
    "   (increase (total-cost) 2))))",
    "(define (problem p) (:domain d) (:init (= (total-cost) 0) (= (x) 0)) (:goal (done)))");
  // (x) may matter to someone; total-cost alone does not
  EXPECT_EQ(action_names(g->task), (std::set<std::string>{"(count)", "(fin)", "(prep)"}));
}

TEST(GroundTest, ConstantComparisonsAreDecided)
{
  auto g = ground_text(domain_with(
      "(:action t :parameters () :precondition (< 1 2) :effect (z))"
      " (:action f :parameters () :precondition (> 1 2) :effect (z))"),
    problem_with(""));
  EXPECT_EQ(action_names(g->task), (std::set<std::string>{"(t)"}));
  EXPECT_TRUE(g->task.actions[0].precondition.empty());
}

TEST(GroundTest, MetricAndTotalCost)
{
  auto g = ground_text(
    "(define (domain d) (:requirements :action-costs) (:predicates (done))"
    " (:functions (total-cost) - number)"
    " (:action fin :parameters () :precondition () :effect (and (done)"
    "   (increase (total-cost) 4))))",
    "(define (problem p) (:domain d) (:init (= (total-cost) 0)) (:goal (done))"
    " (:metric minimize (total-cost)))");
  const Task & t = g->task;
  EXPECT_TRUE(t.metric.minimize);
  EXPECT_FALSE(t.metric.is_default);
  ASSERT_EQ(t.metric.expr.op, ExprOp::Fluent);
  EXPECT_EQ(t.init_numeric.at(t.metric.expr.fluent), 0.0);

  // Without an initial value, (total-cost) starts at 0 (FD and IPC convention)
  auto noinit = ground_text(
    "(define (domain d) (:requirements :action-costs) (:predicates (done))"
    " (:functions (total-cost) - number)"
    " (:action fin :parameters () :precondition () :effect (and (done)"
    "   (increase (total-cost) 4))))",
    "(define (problem p) (:domain d) (:init) (:goal (done)) (:metric minimize (total-cost)))");
  ASSERT_EQ(noinit->task.actions.size(), 1u);
  EXPECT_EQ(noinit->task.init_numeric.at(noinit->task.metric.expr.fluent), 0.0);

  auto def = ground_text(domain_with(""), problem_with(""));
  EXPECT_TRUE(def->task.metric.is_default);
  EXPECT_EQ(def->task.metric.expr.op, ExprOp::StepCount);
}

// --- Object fluents ---------------------------------------------------------------------------

const char * kObjectFluents =
  "(define (domain d) (:requirements :typing :object-fluents :numeric-fluents)"
  " (:types place robot) (:predicates (visited ?l - place) (ready))"
  " (:functions (home ?r - robot) - place (loc ?r - robot) - place)"
  " (:action back :parameters (?r - robot) :precondition (ready)"
  "  :effect (and (visited (home ?r)) (assign (loc ?r) (home ?r))))"
  " (:action check :parameters (?r - robot ?l - place) :precondition (= (loc ?r) ?l)"
  "  :effect (ready)))";

TEST(GroundTest, StaticObjectFluentsAreSubstituted)
{
  auto g = ground_text(kObjectFluents,
      "(define (problem p) (:domain d) (:objects r1 r2 - robot l1 l2 - place)"
      " (:init (= (home r1) l1) (= (loc r1) l2) (= (loc r2) l1)) (:goal (ready)))");
  const Task & t = g->task;
  // home is static and undefined for r2: (back r2) has no visited atom to add
  EXPECT_TRUE(atom_names(t).count("(visited l1)"));
  EXPECT_FALSE(atom_names(t).count("(visited l2)"));
  // loc changes: a condition on an object fluent
  size_t checks = 0;
  for (const auto & a : t.actions) {
    if (t.action_names[a.schema] == "check") {
      ASSERT_EQ(a.precondition.objects.size(), 1u);
      EXPECT_TRUE(a.precondition.objects[0].lhs.is_fluent);
      ++checks;
    }
  }
  EXPECT_EQ(checks, 4u);
  ASSERT_EQ(t.object_fluents.size(), 2u);
}

TEST(GroundTest, ChangingObjectFluentsInsideAtomsAreUnsupported)
{
  auto n = normalize_text(
    "(define (domain d) (:requirements :typing :object-fluents) (:types place robot)"
    " (:predicates (visited ?l - place))"
    " (:functions (loc ?r - robot) - place)"
    " (:action go :parameters (?r - robot ?l - place) :precondition ()"
    "  :effect (and (assign (loc ?r) ?l) (visited (loc ?r)))))",
    "(define (problem p) (:domain d) (:objects r - robot a - place)"
    " (:init (= (loc r) a)) (:goal (visited a)))");
  auto r = ground(n->task);
  ASSERT_FALSE(r.ok());
  EXPECT_EQ(r.diagnostics()[0].code, "ground.unsupported");
  EXPECT_NE(r.diagnostics()[0].message.find("loc"), std::string::npos);
}

// --- Time -------------------------------------------------------------------------------------

TEST(GroundTest, DurativeEndMayNeedItsOwnStartEffects)
{
  auto g = ground_text(
    "(define (domain d) (:requirements :durative-actions :typing) (:types r)"
    " (:predicates (busy ?x - r) (done ?x - r) (never))"
    " (:durative-action work :parameters (?x - r) :duration (= ?duration 5)"
    "  :condition (and (over all (busy ?x)) (at end (busy ?x)))"
    "  :effect (and (at start (busy ?x)) (at end (done ?x)) (at end (not (busy ?x)))))"
    " (:durative-action stuck :parameters (?x - r) :duration (= ?duration 1)"
    "  :condition (at end (never)) :effect (at end (done ?x))))",
    "(define (problem p) (:domain d) (:objects a - r) (:init) (:goal (done a)))");
  const Task & t = g->task;
  ASSERT_EQ(t.durative_actions.size(), 1u);
  const auto & w = t.durative_actions[0];
  EXPECT_EQ(instance_name(t, w), "(work a)");
  ASSERT_EQ(w.duration.size(), 1u);
  EXPECT_EQ(w.duration[0].value.op, ExprOp::Constant);
  EXPECT_EQ(w.duration[0].value.value, 5.0);
  EXPECT_EQ(w.start.add.size(), 1u);
  EXPECT_EQ(w.end.add.size(), 1u);
  EXPECT_EQ(w.end.del.size(), 1u);
  EXPECT_TRUE(w.effects.empty());
}

TEST(GroundTest, DurationsWithUndefinedStaticFluentsArePruned)
{
  auto g = ground_text(
    "(define (domain d) (:requirements :durative-actions :typing :fluents) (:types l)"
    " (:predicates (at ?x - l)) (:functions (time ?a ?b - l))"
    " (:durative-action go :parameters (?a ?b - l) :duration (= ?duration (time ?a ?b))"
    "  :condition (at start (at ?a)) :effect (and (at start (not (at ?a))) (at end (at ?b)))))",
    "(define (problem p) (:domain d) (:objects x y z - l) (:init (at x) (= (time x y) 2)"
    " (= (time y z) 3)) (:goal (at z)))");
  std::set<std::string> names;
  for (const auto & a : g->task.durative_actions) {
    names.insert(instance_name(g->task, a));
  }
  EXPECT_EQ(names, (std::set<std::string>{"(go x y)", "(go y z)"}));
}

TEST(GroundTest, TimedInitialLiteralsMakeAtomsReachable)
{
  auto g = ground_text(
    "(define (domain d) (:requirements :durative-actions :timed-initial-literals)"
    " (:predicates (open) (served) (gone))"
    " (:durative-action serve :parameters () :duration (= ?duration 1)"
    "  :condition (over all (open)) :effect (at end (served))))",
    "(define (problem p) (:domain d) (:init (at 5 (open)) (at 9 (not (open)))"
    " (at 3 (not (gone)))) (:goal (served)))");
  const Task & t = g->task;
  EXPECT_EQ(t.durative_actions.size(), 1u);
  ASSERT_EQ(t.timed_events.size(), 2u);  // deleting (gone), never true, is dropped
  EXPECT_EQ(t.timed_events[0].time, 5.0);
  EXPECT_EQ(t.timed_events[0].kind, TimedEvent::Kind::Add);
  EXPECT_EQ(t.timed_events[1].kind, TimedEvent::Kind::Delete);
}

TEST(GroundTest, ProcessesAndEvents)
{
  auto g = ground_text(
    "(define (domain d) (:requirements :typing :fluents :time) (:types tank)"
    " (:predicates (on ?t - tank) (full ?t - tank)) (:functions (level ?t - tank))"
    " (:process fill :parameters (?t - tank) :precondition (on ?t)"
    "  :effect (increase (level ?t) (* #t 2)))"
    " (:event overflow :parameters (?t - tank) :precondition (>= (level ?t) 10)"
    "  :effect (full ?t))"
    " (:action start :parameters (?t - tank) :precondition () :effect (on ?t)))",
    "(define (problem p) (:domain d) (:objects t1 t2 - tank)"
    " (:init (= (level t1) 0)) (:goal (full t1)))");
  const Task & t = g->task;
  // (level t2) is never defined: no process or event for t2
  EXPECT_EQ(t.processes.size(), 1u);
  EXPECT_EQ(t.events.size(), 1u);
  EXPECT_TRUE(t.processes[0].effects.at(0).continuous);
}

// --- Limits and output -----------------------------------------------------------------------

TEST(GroundTest, BudgetStopsGrounding)
{
  // 30 objects, 4 parameters: 810 000 instances
  std::string objects;
  for (int i = 0; i < 30; ++i) {
    objects += " o" + std::to_string(i);
  }
  auto n = normalize_text(
    "(define (domain d) (:predicates (q ?a ?b ?c ?d))"
    " (:action big :parameters (?a ?b ?c ?d) :precondition () :effect (q ?a ?b ?c ?d)))",
    "(define (problem p) (:domain d) (:objects" + objects + ") (:init) (:goal (q o1 o1 o1 o1)))");
  std::stop_source stop;
  stop.request_stop();
  BudgetConfig cfg;
  cfg.stop = stop.get_token();
  Budget budget(cfg);
  auto r = ground(n->task, {}, &budget);
  ASSERT_FALSE(r.ok());
  EXPECT_EQ(r.diagnostics()[0].code, "ground.budget");
  EXPECT_NE(r.diagnostics()[0].message.find("cancelled"), std::string::npos);
}

TEST(GroundTest, DeadlineStopsGroundingAtAnyStage)
{
  // A clock that advances with every look: the deadline passes after a number of checks,
  // during the Datalog evaluation or the instantiation, whichever comes first
  std::string objects;
  for (int i = 0; i < 12; ++i) {
    objects += " o" + std::to_string(i);
  }
  auto n = normalize_text(
    "(define (domain d) (:predicates (q ?a ?b ?c) (r ?a))"
    " (:action big :parameters (?a ?b ?c) :precondition ()"
    "  :effect (and (q ?a ?b ?c) (forall (?x) (r ?x)))))",
    "(define (problem p) (:domain d) (:objects" + objects + ") (:init) (:goal (q o1 o1 o1)))");
  for (int checks : {1, 3, 6, 10}) {
    auto ticks = std::make_shared<int>(0);
    BudgetConfig cfg;
    cfg.check_interval = 1;
    auto start = BudgetConfig::Clock::now();
    cfg.clock = [ticks, start]() {return start + std::chrono::seconds(++*ticks);};
    cfg.deadline = start + std::chrono::seconds(checks);
    Budget budget(cfg);
    auto r = ground(n->task, {}, &budget);
    ASSERT_FALSE(r.ok()) << checks;
    EXPECT_EQ(r.diagnostics()[0].code, "ground.budget");
    EXPECT_NE(r.diagnostics()[0].message.find("out of time"), std::string::npos);
  }
  // Without a budget the same task grounds: 1 728 instances
  auto r = ground(n->task);
  ASSERT_TRUE(r.ok());
  EXPECT_EQ(r->actions.size(), 1728u);
}

TEST(GroundTest, LogsSizesOnTheGroundChannel)
{
  auto n = normalize_text(domain_with(
      "(:action mk :parameters (?x - item) :precondition () :effect (p ?x))"), problem_with(""));
  std::vector<std::string> lines;
  GroundOptions o;
  LogOptions lo;
  lo.level = LogLevel::Info;
  lo.callback = [&](LogLevel, LogChannel c, std::string_view m) {
      if (c == LogChannel::Ground) {
        lines.emplace_back(m);
      }
    };
  o.logger = Logger(lo);
  ASSERT_TRUE(ground(n->task, o).ok());
  ASSERT_EQ(lines.size(), 1u);
  EXPECT_NE(lines[0].find("3 actions"), std::string::npos) << lines[0];
}

TEST(GroundTest, TextDump)
{
  auto g = ground_text(domain_with(
      "(:action move :parameters (?x - item ?a ?b - place)"
      " :precondition (and (at ?x ?a) (link ?a ?b) (> (g) 0))"
      " :effect (and (not (at ?x ?a)) (at ?x ?b) (decrease (g) 1)))"),
    problem_with("(at a l1) (link l1 l2) (= (g) 2)", "(at a l2)"));
  EXPECT_EQ(to_string(g->task),
    ";; ground task d / pr\n"
    "(:atoms 2)\n"
    "(:init\n  (at a l1)\n  (= (g) 2))\n"
    "(:action (move a l1 l2)\n"
    "  :precondition (and (at a l1) (> (g) 0))\n"
    "  :effect\n    (not (at a l1))\n    (at a l2)\n    (decrease (g) 1))\n"
    "(:goal (at a l2))\n"
    "(:metric minimize (plan-length) ; default)\n");
}

// --- Equivalence with the original domain, judged by the validator ---------------------------

std::string fmt(double v)
{
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%.3f", v);
  return buf;
}

// Typed instantiations of the printed parameters of every schema called `names`
std::vector<std::pair<std::string, std::vector<uint32_t>>> candidates(
  const lifted::Task & t, const std::map<std::string, std::vector<TypeSpec>> & signatures,
  std::mt19937 & rng, size_t max)
{
  std::vector<std::pair<std::string, std::vector<uint32_t>>> out;
  for (const auto & [name, types] : signatures) {
    std::vector<uint32_t> args(types.size());
    std::function<void(size_t)> rec = [&](size_t i) {
        if (out.size() > 50000) {
          return;
        }
        if (i == types.size()) {
          out.emplace_back(name, args);
          return;
        }
        for (uint32_t o = 0; o < t.object_names.size(); ++o) {
          if (t.types.fits(t.object_types[o], types[i])) {
            args[i] = o;
            rec(i + 1);
          }
        }
      };
    rec(0);
  }
  std::shuffle(out.begin(), out.end(), rng);
  if (out.size() > max) {
    out.resize(max);
  }
  return out;
}

std::string text(
  const std::string & name, const std::vector<uint32_t> & args,
  const lifted::Task & t)
{
  std::string s = "(" + name;
  for (uint32_t a : args) {
    s += " " + t.object_names[a];
  }
  return s + ")";
}

bool valid_prefix(
  const std::string & domain, const std::string & problem, const std::string & plan,
  bool check_goal)
{
  auto p = parse_plan(plan);
  if (!p.ok()) {
    ADD_FAILURE() << "bad plan text\n" << plan;
    return false;
  }
  ValidateOptions o;
  o.check_goal = check_goal;
  return validate_plan(domain, problem, *p, o).valid();
}

// Sequential plans: every original instance is applicable for the validator iff some ground
// copy is applicable in the ground simulator
::testing::AssertionResult instant_walk(
  const std::string & domain, const std::string & problem, uint32_t seed, size_t steps,
  size_t max_candidates)
{
  // Keep actions that change nothing: the validator accepts them
  GroundOptions keep;
  keep.drop_useless_actions = false;
  auto g = ground_text(domain, problem, keep);
  const lifted::Task & lt = g->n->task;
  const Task & t = g->task;
  GroundSimulator sim(t);
  GState state = sim.initial();
  std::mt19937 rng(seed);
  std::map<std::string, std::vector<size_t>> by_name;
  for (size_t i = 0; i < t.actions.size(); ++i) {
    by_name[instance_name(t, t.actions[i])].push_back(i);
  }
  std::map<std::string, std::vector<TypeSpec>> signatures;
  for (const auto & a : lt.actions) {
    std::vector<TypeSpec> types;
    for (uint32_t i = 0; i < a.parameters; ++i) {
      types.push_back(a.variables[i].types);
    }
    signatures.emplace(a.name, types);
  }
  std::string prefix;
  size_t checked = 0;
  for (size_t step = 0; step < steps; ++step) {
    std::vector<std::pair<std::string, size_t>> applicable;
    for (const auto & [name, args] : candidates(lt, signatures, rng, max_candidates)) {
      std::string s = text(name, args, lt);
      bool orig = valid_prefix(domain, problem, prefix + s + "\n", false);
      std::optional<size_t> copy;
      for (size_t i : by_name[s]) {
        if (sim.applicable(t.actions[i], state)) {
          copy = i;
          break;
        }
      }
      ++checked;
      if (orig != copy.has_value()) {
        return ::testing::AssertionFailure() << "after\n" << prefix << s << " is "
                                             << (orig ? "" : "not ") << "applicable in the "
               "original domain but " << (copy ? "" : "not ") << "in the ground task";
      }
      if (copy) {
        applicable.emplace_back(s, *copy);
      }
    }
    if (applicable.empty()) {
      break;
    }
    auto chosen = applicable[std::uniform_int_distribution<size_t>(0, applicable.size() - 1)(rng)];
    const Action & act = t.actions[chosen.second];
    state = sim.apply(act.changes, act.effects, Timing::Instant, state);
    prefix += chosen.first + "\n";
  }
  if (valid_prefix(domain, problem, prefix, true) != sim.goal(state)) {
    return ::testing::AssertionFailure() << "goal verdicts differ after\n" << prefix;
  }
  if (checked == 0) {
    return ::testing::AssertionFailure() << "no candidates checked";
  }
  return ::testing::AssertionSuccess();
}

// Duration for an instance: the ground constraints if it exists, else the lifted expression
// on the current values (so missing instances are noticed too)
std::optional<double> duration_of(
  const Grounded & g, const GroundSimulator & sim, const GState & state,
  const std::string & name, const std::vector<uint32_t> & args)
{
  const lifted::Task & lt = g.n->task;
  for (const auto & a : g.task.durative_actions) {
    if (instance_name(g.task, a) == text(name, args, lt)) {
      double lo = 0.001, hi = 1e9;
      for (const auto & c : a.duration) {
        double v = sim.number(c.value, state);
        if (c.op == CompareOp::Eq) {
          lo = hi = v;
        } else if (c.op == CompareOp::Ge) {
          lo = std::max(lo, v);
        } else {
          hi = std::min(hi, v);
        }
      }
      return std::min(std::max(lo, std::min(1.0, hi)), hi);
    }
  }
  // No instance: evaluate an (= ?duration e) with static fluents of the lifted task
  for (const auto & a : lt.durative_actions) {
    if (a.name != name) {
      continue;
    }
    for (const auto & c : a.duration) {
      std::function<std::optional<double>(const lifted::Expr &)> ev =
        [&](const lifted::Expr & e) -> std::optional<double> {
          if (e.op == ExprOp::Constant) {
            return e.value;
          }
          if (e.op == ExprOp::Fluent) {
            for (const auto & f : lt.init_fluents) {
              bool same = f.function == e.function && f.args.size() == e.args.size();
              for (size_t i = 0; same && i < f.args.size(); ++i) {
                const auto & x = e.args[i];
                uint32_t v = x.kind == lifted::Term::Kind::Variable ? args.at(x.index) : x.index;
                same = f.args[i].value() == v;
              }
              if (same) {
                return f.number;
              }
            }
            return std::nullopt;
          }
          if (e.op == ExprOp::Add || e.op == ExprOp::Mul || e.op == ExprOp::Sub ||
            e.op == ExprOp::Div)
          {
            auto x = ev(e.children.at(0));
            auto y = ev(e.children.at(1));
            if (!x || !y) {
              return std::nullopt;
            }
            switch (e.op) {
              case ExprOp::Add: return *x + *y;
              case ExprOp::Mul: return *x * *y;
              case ExprOp::Sub: return *x - *y;
              default: return *x / *y;
            }
          }
          return std::nullopt;
        };
      if (c.op == CompareOp::Eq) {
        return ev(c.value);
      }
    }
    return 1.0;
  }
  return std::nullopt;
}

// Temporal plans with one action at a time
::testing::AssertionResult durative_walk(
  const std::string & domain, const std::string & problem, uint32_t seed, size_t steps,
  size_t max_candidates)
{
  // Keep actions that change nothing: the validator accepts them
  GroundOptions keep;
  keep.drop_useless_actions = false;
  auto g = ground_text(domain, problem, keep);
  const lifted::Task & lt = g->n->task;
  const Task & t = g->task;
  GroundSimulator sim(t);
  GState state = sim.initial();
  std::mt19937 rng(seed);
  std::map<std::string, std::vector<size_t>> by_name;
  for (size_t i = 0; i < t.durative_actions.size(); ++i) {
    by_name[instance_name(t, t.durative_actions[i])].push_back(i);
  }
  std::map<std::string, std::vector<TypeSpec>> signatures;
  for (const auto & a : lt.durative_actions) {
    std::vector<TypeSpec> types;
    for (uint32_t i = 0; i < a.parameters; ++i) {
      types.push_back(a.variables[i].types);
    }
    signatures.emplace(a.name, types);
  }
  std::string prefix;
  double now = 0.0;
  size_t checked = 0;
  for (size_t step = 0; step < steps; ++step) {
    struct Option
    {
      std::string line;
      GState next;
      double end;
    };
    std::vector<Option> applicable;
    for (const auto & [name, args] : candidates(lt, signatures, rng, max_candidates)) {
      std::string s = text(name, args, lt);
      auto d = duration_of(*g, sim, state, name, args);
      if (!d || *d <= 0) {
        continue;  // no usable duration: the validator could not judge it either
      }
      double dur = std::stod(fmt(*d));
      std::string line = fmt(now) + ": " + s + " [" + fmt(dur) + "]\n";
      bool orig = valid_prefix(domain, problem, prefix + line, false);
      std::optional<GState> next;
      for (size_t i : by_name[s]) {
        next = sim.run(t.durative_actions[i], state, dur);
        if (next) {
          break;
        }
      }
      ++checked;
      if (orig != next.has_value()) {
        return ::testing::AssertionFailure() << "after\n" << prefix << line << " is "
                                             << (orig ? "" : "not ") << "applicable in the "
               "original domain but " << (next ? "" : "not ") << "in the ground task";
      }
      if (next) {
        applicable.push_back(Option{line, *next, now + dur});
      }
    }
    if (applicable.empty()) {
      break;
    }
    auto & chosen = applicable[std::uniform_int_distribution<size_t>(0, applicable.size() - 1)(
        rng)];
    state = chosen.next;
    prefix += chosen.line;
    now = chosen.end + 0.1;
  }
  if (checked == 0) {
    return ::testing::AssertionFailure() << "no candidates checked";
  }
  if (valid_prefix(domain, problem, prefix, true) != sim.goal(state)) {
    return ::testing::AssertionFailure() << "goal verdicts differ after\n" << prefix;
  }
  return ::testing::AssertionSuccess();
}

const char * kAdlDomain =
  "(define (domain adl) (:requirements :adl :typing :derived-predicates :fluents)\n"
  " (:types room box - object robot)\n"
  " (:predicates (at ?r - robot ?l - room) (in ?b - box ?l - room) (holding ?r - robot ?b - box)\n"
  "   (link ?a ?b - room) (dark ?l - room) (open ?l - room) (reach ?a ?b - room) (done))\n"
  " (:functions (energy ?r - robot) (cost ?a ?b - room))\n"
  " (:derived (reach ?a ?b) (or (link ?a ?b) (exists (?c - room) (and (link ?a ?c)\n"
  "   (reach ?c ?b)))))\n"
  " (:action move :parameters (?r - robot ?a ?b - room)\n"
  "  :precondition (and (at ?r ?a) (or (link ?a ?b) (link ?b ?a))\n"
  "    (imply (dark ?b) (exists (?x - box) (holding ?r ?x))) (>= (energy ?r) (cost ?a ?b)))\n"
  "  :effect (and (not (at ?r ?a)) (at ?r ?b) (decrease (energy ?r) (cost ?a ?b))\n"
  "    (forall (?x - box) (when (holding ?r ?x) (and (not (in ?x ?a)) (in ?x ?b))))))\n"
  " (:action pick :parameters (?r - robot ?x - box ?l - room)\n"
  "  :precondition (and (at ?r ?l) (in ?x ?l) (not (exists (?y - box) (holding ?r ?y))))\n"
  "  :effect (holding ?r ?x))\n"
  " (:action drop :parameters (?r - robot ?x - box ?l - room)\n"
  "  :precondition (and (at ?r ?l) (holding ?r ?x) (not (= ?l ?l)) )\n"
  "  :effect (and (not (holding ?r ?x)) (when (or (dark ?l) (open ?l)) (done))))\n"
  " (:action put :parameters (?r - robot ?x - box ?l - room)\n"
  "  :precondition (and (at ?r ?l) (holding ?r ?x))\n"
  "  :effect (and (not (holding ?r ?x)) (when (or (dark ?l) (open ?l)) (done))))\n"
  " (:action light :parameters (?r - robot ?l - room)\n"
  "  :precondition (and (at ?r ?l) (forall (?x - box) (or (in ?x ?l) (not (reach ?l ?l)))))\n"
  "  :effect (and (not (dark ?l)) (when (not (open ?l)) (open ?l)))))\n";

const char * kAdlProblem =
  "(define (problem p) (:domain adl) (:objects r1 - robot k h c - room b1 b2 - box)\n"
  " (:init (at r1 k) (in b1 k) (in b2 h) (link k h) (link h c) (link c k) (dark c)\n"
  "        (= (energy r1) 9) (= (cost k h) 1) (= (cost h k) 1) (= (cost h c) 2)\n"
  "        (= (cost c h) 2) (= (cost c k) 3) (= (cost k c) 3))\n"
  " (:goal (and (done) (in b1 c))))\n";

TEST(GroundEquivalenceTest, RandomWalksOnAnAdlDomain)
{
  for (uint32_t seed = 0; seed < 12; ++seed) {
    EXPECT_TRUE(instant_walk(kAdlDomain, kAdlProblem, seed, 10, 80)) << "seed " << seed;
  }
}

// Random STRIPS-like domains with typing, statics, negations, equality and conditional effects
std::pair<std::string, std::string> random_domain(std::mt19937 & rng)
{
  auto pick = [&](int n) {return std::uniform_int_distribution<int>(0, n - 1)(rng);};
  const std::vector<std::string> lits = {"(p ?x)", "(p ?y)", "(q ?x)", "(r ?x ?y)", "(r ?y ?x)",
    "(s ?x)", "(link ?x ?y)", "(= ?x ?y)", "(z)"};
  auto lit = [&]() {
      std::string l = lits[static_cast<size_t>(pick(static_cast<int>(lits.size())))];
      return pick(3) == 0 ? "(not " + l + ")" : l;
    };
  std::string actions;
  for (int a = 0; a < 3; ++a) {
    std::string pre = "(and";
    for (int i = 0, n = 1 + pick(3); i < n; ++i) {
      pre += " " + lit();
    }
    pre += ")";
    const std::vector<std::string> adds = {"(p ?x)", "(q ?y)", "(r ?x ?y)", "(z)", "(p ?y)"};
    std::string eff = "(and";
    for (int i = 0, n = 1 + pick(2); i < n; ++i) {
      std::string e = adds[static_cast<size_t>(pick(static_cast<int>(adds.size())))];
      eff += " " + (pick(3) == 0 ? "(not " + e + ")" : e);
    }
    if (pick(2)) {
      eff += " (when " + lit() + " " + adds[static_cast<size_t>(pick(5))] + ")";
    }
    if (pick(3) == 0) {
      eff += " (forall (?w - t) (when (r ?x ?w) (not (p ?w))))";
    }
    eff += ")";
    actions += " (:action a" + std::to_string(a) + " :parameters (?x " +
      (pick(2) ? "- t" : "- u") + " ?y - t) :precondition " + pre + " :effect " + eff + ")";
  }
  std::string domain =
    "(define (domain rd) (:requirements :adl :typing) (:types u - t)"
    " (:predicates (p ?x - t) (q ?x - t) (r ?x ?y - t) (s ?x - t) (link ?x ?y - t) (z))" +
    actions + ")";
  const std::vector<std::string> facts = {"(p o1)", "(q o2)", "(r o1 o2)", "(s o3)", "(s o1)",
    "(link o1 o2)", "(link o2 o3)", "(link o3 o3)", "(r o3 o1)"};
  std::string init;
  for (const auto & f : facts) {
    if (pick(2)) {
      init += " " + f;
    }
  }
  std::string problem = "(define (problem rp) (:domain rd) (:objects o1 o2 - t o3 - u) (:init" +
    init + ") (:goal (and (z) (q o1))))";
  return {domain, problem};
}

TEST(GroundEquivalenceTest, RandomDomains)
{
  std::mt19937 rng(11);
  for (uint32_t i = 0; i < 40; ++i) {
    auto [domain, problem] = random_domain(rng);
    EXPECT_TRUE(instant_walk(domain, problem, i, 6, 40)) << domain << "\n" << problem;
  }
}

struct PairCase
{
  const char * domain;
  const char * problem;
};

// Every valid PlanSys2 pair (from the semantic tests), walked with the validator as the judge
const PairCase kPlanSys2[] = {
  {"plansys2_core__domain_1_ok", "plansys2_core__problem_simple_1"},
  {"plansys2_domain_expert__domain_charging", "plansys2_domain_expert__problem_charging"},
  {"plansys2_domain_expert__domain_simple", "plansys2_domain_expert__problem_simple_1"},
  {"plansys2_executor__elevator_domain", "plansys2_executor__elevator_problem"},
  {"plansys2_executor__factory2", "plansys2_executor__problem_factory_1"},
  {"plansys2_executor__domain_simple", "plansys2_executor__problem_simple_1"},
  {"plansys2_executor__road_trip_domain", "plansys2_executor__road_trip_problem"},
  {"plansys2_executor__suave_domain", "plansys2_executor__suave_problem"},
  {"plansys2_pddl_parser__dom1", "plansys2_pddl_parser__prob1"},
  {"plansys2_pddl_parser__dom2", "plansys2_pddl_parser__prob2"},
  {"plansys2_simple_example__simple_example", "plansys2_simple_example__problem"},
  {"plansys2_terminal__simple_example", "plansys2_terminal__problem_charging"},
  {"plansys2_problem_expert__domain_simple", "plansys2_problem_expert__problem_simple_1"},
  {"plansys2_tests__test_2", "plansys2_tests__problem_2"},
};

// Sanitizers and coverage (unoptimized, instrumented) slow everything down several times
#if defined(__SANITIZE_ADDRESS__) || defined(__SANITIZE_THREAD__) || defined(KAIROS_COVERAGE_BUILD)
constexpr double kSlowdown = 10.0;
#elif defined(__has_feature)
#if __has_feature(address_sanitizer) || __has_feature(thread_sanitizer)
constexpr double kSlowdown = 10.0;
#else
constexpr double kSlowdown = 1.0;
#endif
#else
constexpr double kSlowdown = 1.0;
#endif

class PlanSys2GroundTest : public ::testing::TestWithParam<PairCase>
{
};

TEST_P(PlanSys2GroundTest, WalksAgreeWithTheValidatorAndGroundingIsFast)
{
  std::string d = read_file(data_path(std::string("plansys2/") + GetParam().domain + ".pddl"));
  std::string p = read_file(data_path(std::string("plansys2/") + GetParam().problem + ".pddl"));
  auto g = ground_text(d, p);
  // PlanSys2 examples must plan in milliseconds: grounding is a small part of it
  EXPECT_LT(g->stats.milliseconds, 50.0 * kSlowdown);
  for (uint32_t seed = 0; seed < 3; ++seed) {
    if (!g->task.actions.empty()) {
      EXPECT_TRUE(instant_walk(d, p, seed, 6, 40)) << "seed " << seed;
    }
    if (!g->task.durative_actions.empty()) {
      EXPECT_TRUE(durative_walk(d, p, seed, 6, 40)) << "seed " << seed;
    }
  }
}

INSTANTIATE_TEST_SUITE_P(
  PlanSys2, PlanSys2GroundTest, ::testing::ValuesIn(kPlanSys2),
  [](const ::testing::TestParamInfo<PairCase> & param_info) {
    std::string n = param_info.param.problem;
    std::replace_if(n.begin(), n.end(), [](char c) {return !std::isalnum(c);}, '_');
    return n + "_" + std::to_string(param_info.index);
  });

// IPC domains, when the corpus is available
TEST(GroundEquivalenceTest, IpcDomains)
{
  const char * dir = std::getenv("KAIROS_PDDL_CORPUS");
  if (dir == nullptr) {
    GTEST_SKIP() << "KAIROS_PDDL_CORPUS not set";
  }
  std::filesystem::path base = std::filesystem::path(dir) / "potassco_pddl-instances";
  const std::vector<std::pair<std::string, bool>> domains = {
    {"ipc-1998/domains/assembly-round-1-adl", false},
    {"ipc-2000/domains/elevator-adl-full-typed", false},
    {"ipc-2000/domains/schedule-adl-typed", false},
    {"ipc-2002/domains/depots-strips-automatic", false},
    {"ipc-2002/domains/driverlog-numeric-automatic", false},
    {"ipc-2002/domains/zenotravel-time-automatic", true},
    {"ipc-2002/domains/rovers-time-automatic", true},
    {"ipc-2004/domains/psr-small-strips", false},
    {"ipc-2004/domains/promela-dining-philosophers-derived-predicates-adl", false},
    {"ipc-2004/domains/airport-nontemporal-adl", false},
    {"ipc-2006/domains/trucks-propositional", false},
    {"ipc-2008/domains/openstacks-sequential-satisficing-adl", false},
    {"ipc-2008/domains/elevator-sequential-satisficing-strips", false},
    {"ipc-2011/domains/parking-temporal-satisficing", true},
  };
  size_t tried = 0;
  for (const auto & [dom, temporal] : domains) {
    auto d = base / dom / "domain.pddl";
    auto p = base / dom / "instances" / "instance-1.pddl";
    if (!std::filesystem::exists(p)) {
      continue;
    }
    if (!std::filesystem::exists(d)) {
      d = base / dom / "domains" / "domain-1.pddl";
    }
    ++tried;
    if (temporal) {
      EXPECT_TRUE(durative_walk(read_file(d), read_file(p), 1, 5, 25)) << dom;
    } else {
      EXPECT_TRUE(instant_walk(read_file(d), read_file(p), 1, 5, 25)) << dom;
    }
  }
  EXPECT_GT(tried, 0u);
}

// --- Same sizes as Fast Downward's translator ------------------------------------------------

std::string lower(std::string s)
{
  std::transform(
    s.begin(), s.end(), s.begin(), [](unsigned char c) {
      return static_cast<char>(std::tolower(c));
        });
  return s;
}

// "ATOM (...)" and "ACTION (...)" lines from tests/ground/fd_explore.py
std::set<std::string> fd_explore(
  const std::string & fd, const std::filesystem::path & d, const std::filesystem::path & p)
{
  std::string cmd = "python3 " + std::string(KAIROS_FD_EXPLORE_SCRIPT) + " '" + fd + "' '" +
    d.string() + "' '" + p.string() + "' 2>/dev/null";
  std::set<std::string> out;
  FILE * pipe = popen(cmd.c_str(), "r");
  if (!pipe) {
    return out;
  }
  char buf[4096];
  while (fgets(buf, sizeof(buf), pipe)) {
    std::string line(buf);
    while (!line.empty() && (line.back() == '\n' || line.back() == '\r')) {
      line.pop_back();
    }
    out.insert(lower(line));
  }
  pclose(pipe);
  return out;
}

// With KAIROS_FD_TRANSLATE=<fast downward>/src/translate and the corpus. Kairos may find fewer:
// it filters with static negations and inequalities while exploring, FD only afterwards.
TEST(GroundFdTest, SameAtomsAndActionsAsFastDownward)
{
  const char * fd = std::getenv("KAIROS_FD_TRANSLATE");
  const char * dir = std::getenv("KAIROS_PDDL_CORPUS");
  if (fd == nullptr || dir == nullptr) {
    GTEST_SKIP() << "KAIROS_FD_TRANSLATE or KAIROS_PDDL_CORPUS not set";
  }
  struct Case
  {
    const char * path;
    const char * problem;
    bool exact;
  };
  const Case cases[] = {
    {"potassco_pddl-instances/ipc-1998/domains/gripper-round-1-strips", "", true},
    {"potassco_pddl-instances/ipc-1998/domains/grid-round-2-strips", "", true},
    {"potassco_pddl-instances/ipc-1998/domains/movie-round-1-strips", "", true},
    {"potassco_pddl-instances/ipc-2000/domains/logistics-strips-typed", "", true},
    {"potassco_pddl-instances/ipc-2000/domains/blocks-strips-typed", "", true},
    {"potassco_pddl-instances/ipc-2000/domains/freecell-strips-typed", "", true},
    {"potassco_pddl-instances/ipc-2002/domains/depots-strips-automatic", "", true},
    {"potassco_pddl-instances/ipc-2002/domains/driverlog-strips-automatic", "", true},
    {"potassco_pddl-instances/ipc-2002/domains/satellite-strips-automatic", "", true},
    {"potassco_pddl-instances/ipc-2004/domains/psr-small-strips", "", true},
    {"potassco_pddl-instances/ipc-2004/domains/promela-optical-telegraph-derived-predicates-strips",
      "", true},
    {"potassco_pddl-instances/ipc-2006/domains/openstacks-propositional-strips", "", true},
    {"potassco_pddl-instances/ipc-2006/domains/tpp-propositional-strips", "", true},
    {"potassco_pddl-instances/ipc-2006/domains/trucks-propositional-strips", "", true},
    {"potassco_pddl-instances/ipc-2008/domains/parc-printer-sequential-satisficing-strips", "",
      true},
    {"potassco_pddl-instances/ipc-2014/domains/tetris-sequential-optimal", "", true},
    {"potassco_pddl-instances/ipc-2014/domains/genome-edit-distances-sequential-optimal", "",
      false},
    {"ipc2023-classical_ipc2023-dataset/opt/rubiks-cube", "p01.pddl", true},
    {"ipc2023-classical_ipc2023-dataset/opt/folding", "p01.pddl", true},
    {"ipc2023-classical_ipc2023-dataset/opt/ricochet-robots", "p01.pddl", false},
  };
  size_t tried = 0;
  for (const auto & c : cases) {
    std::filesystem::path base = std::filesystem::path(dir) / c.path;
    std::filesystem::path p = *c.problem ? base / c.problem : base / "instances/instance-1.pddl";
    std::filesystem::path d = base / "domain.pddl";
    if (!std::filesystem::exists(d)) {
      d = base / "domains/domain-1.pddl";
    }
    if (!std::filesystem::exists(p) || !std::filesystem::exists(d)) {
      continue;
    }
    std::set<std::string> theirs = fd_explore(fd, d, p);
    ASSERT_FALSE(theirs.empty()) << "Fast Downward gave nothing for " << c.path;
    auto g = ground_text(read_file(d), read_file(p));
    std::set<std::string> ours;
    for (AtomId a = 0; a < g->task.atoms.size(); ++a) {
      if (!g->task.is_derived(a)) {
        ours.insert("atom " + lower(atom_name(g->task, a)));
      }
    }
    for (const auto & a : g->task.actions) {
      ours.insert("action " + lower(instance_name(g->task, a)));
    }
    ++tried;
    std::vector<std::string> extra;
    std::set_difference(
      ours.begin(), ours.end(), theirs.begin(), theirs.end(), std::back_inserter(extra));
    EXPECT_TRUE(extra.empty()) << c.path << ": not in Fast Downward: " << extra.front();
    if (c.exact) {
      EXPECT_EQ(ours.size(), theirs.size()) << c.path;
    }
  }
  EXPECT_GT(tried, 0u);
}

}  // namespace
}  // namespace kairos::ground
