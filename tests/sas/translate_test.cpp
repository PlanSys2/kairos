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
#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <map>
#include <memory>
#include <random>
#include <set>
#include <string>
#include <vector>

#include "sas/translate.hpp"
#include "support/ground_simulator.hpp"
#include "support/pddl_helpers.hpp"
#include "support/pipeline_helpers.hpp"
#include "support/sas_simulator.hpp"

namespace kairos::sas
{
namespace
{

using kairos::testing::data_path;
using kairos::testing::dump;
using kairos::testing::pipeline;
using kairos::testing::read_file;
using ground::Timing;

std::string domain_with(const std::string & body)
{
  return "(define (domain d) (:requirements :adl :typing :fluents :derived-predicates)"
         " (:types robot place)"
         " (:predicates (at ?r - robot ?l - place) (link ?a ?b - place) (on ?r - robot)"
         "   (dirty ?l - place) (reach ?l - place) (z))"
         " (:functions (energy ?r - robot) (dist ?a ?b - place)) " + body + ")";
}

std::string problem_with(const std::string & init, const std::string & goal = "(z)")
{
  return "(define (problem pr) (:domain d) (:objects r - robot a b c - place)"
         " (:init " + init + ") (:goal " + goal + "))";
}

const char * kMove =
  "(:action move :parameters (?r - robot ?x ?y - place)"
  " :precondition (and (at ?r ?x) (link ?x ?y)) :effect (and (not (at ?r ?x)) (at ?r ?y)))";

const Variable * group_of(const Task & t, const std::string & atom)
{
  for (VarId v = 0; v < t.variables.size(); ++v) {
    for (uint32_t i = 0; i < t.variables[v].size(); ++i) {
      if (fact_name(t, Fact{v, i}) == atom) {
        return &t.variables[v];
      }
    }
  }
  return nullptr;
}

TEST(TranslateTest, MutexGroupsBecomeVariables)
{
  auto p = pipeline(domain_with(kMove),
      problem_with("(at r a) (link a b) (link b c) (link c a)", "(at r c)"));
  const Task & t = p->sas;
  // One variable for where r is (three places, plus none of those)
  const Variable * at = group_of(t, "(at r a)");
  ASSERT_NE(at, nullptr);
  EXPECT_EQ(at->kind, Variable::Kind::Group);
  EXPECT_EQ(at->size(), 4u);
  EXPECT_EQ(t.variables.size(), 1u);
  EXPECT_EQ(fact_name(t, Fact{0, t.initial[0]}), "(at r a)");
  // move: precondition at = x, effect at := y; the delete is implied by the add
  for (const auto & a : t.actions) {
    ASSERT_EQ(a.pre.facts.size(), 1u);
    ASSERT_EQ(a.changes.size(), 1u);
    EXPECT_TRUE(a.effects.empty());
    EXPECT_NE(a.changes[0].value, t.variables[a.changes[0].var].none());
  }
}

TEST(TranslateTest, WithoutInvariantsEveryAtomIsBinary)
{
  TranslateOptions o;
  o.use_invariants = false;
  auto p = pipeline(domain_with(kMove),
      problem_with("(at r a) (link a b) (link b c) (link c a)", "(at r c)"), o);
  const Task & t = p->sas;
  EXPECT_EQ(t.variables.size(), 3u);
  for (const auto & v : t.variables) {
    EXPECT_EQ(v.kind, Variable::Kind::Binary);
  }
  // A delete sets the binary variable to false (1), unconditionally
  for (const auto & a : t.actions) {
    EXPECT_EQ(a.changes.size(), 2u);
    EXPECT_TRUE(a.effects.empty());
  }
}

TEST(TranslateTest, DeletesOfGroupAtomsNotKnownTrueAreConditional)
{
  // clear deletes (at r ?l) for any ?l without requiring it: conditional on the value
  auto p = pipeline(domain_with(std::string(kMove) +
      " (:action clear :parameters (?r - robot ?l - place) :precondition (on ?r)"
      "  :effect (not (at ?r ?l)))"),
      problem_with("(at r a) (link a b) (on r)", "(at r b)"));
  const Task & t = p->sas;
  size_t clears = 0;
  for (const auto & a : t.actions) {
    if (t.action_names[a.schema] != "clear") {
      continue;
    }
    ++clears;
    const Variable * at = group_of(t, "(at r a)");
    ASSERT_NE(at, nullptr);
    ASSERT_EQ(a.effects.size(), 1u) << instance_name(t, a);
    const Effect & e = a.effects[0];
    EXPECT_EQ(e.fact.value, t.variables[e.fact.var].none());
    ASSERT_EQ(e.condition.facts.size(), 1u);
    EXPECT_EQ(e.condition.facts[0].var, e.fact.var);
  }
  EXPECT_GT(clears, 0u);
}

TEST(TranslateTest, NegatedConditions)
{
  // (not (at r b)) on a group: var != value; (not (on r)) on a binary: value 1
  auto p = pipeline(domain_with(std::string(kMove) +
      " (:action wave :parameters (?r - robot ?l - place)"
      "  :precondition (and (not (at ?r ?l)) (not (on ?r))) :effect (z))"),
      problem_with("(at r a) (link a b) (link b a) (on r)", "(z)"));
  const Task & t = p->sas;
  for (const auto & a : t.actions) {
    if (instance_name(t, a) == "(wave r b)") {
      ASSERT_EQ(a.pre.not_facts.size(), 1u);
      EXPECT_EQ(fact_name(t, a.pre.not_facts[0]), "(at r b)");
      ASSERT_EQ(a.pre.facts.size(), 1u);
      EXPECT_EQ(fact_name(t, a.pre.facts[0]), "(not (on r))");
    }
  }
}

TEST(TranslateTest, DerivedPredicatesGetLayers)
{
  // reach (layer 0) from links; dirty-free (z) needs no unreachable place: layer 1
  auto p = pipeline(
    "(define (domain d) (:requirements :typing :derived-predicates :negative-preconditions)"
    " (:types place) (:predicates (link ?a ?b - place) (reach ?a ?b - place) (lonely ?a - place)"
    "   (open ?a - place))"
    " (:derived (reach ?a ?b) (or (and (open ?a) (link ?a ?b))"
    "   (exists (?c - place) (and (open ?a) (link ?a ?c) (reach ?c ?b)))))"
    " (:derived (lonely ?a) (not (exists (?b - place) (reach ?b ?a))))"
    " (:action close :parameters (?a - place) :precondition (open ?a) :effect (not (open ?a))))",
    "(define (problem p) (:domain d) (:objects x y w - place)"
    " (:init (link x y) (link y x) (open x) (open y)) (:goal (lonely w)))");
  const Task & t = p->sas;
  int reach = -1, lonely = -1;
  for (const auto & v : t.variables) {
    if (v.kind == Variable::Kind::Derived) {
      EXPECT_EQ(v.default_value, 1u);
      const auto & name = t.predicate_names[t.atoms[v.atoms[0]].predicate.index()];
      if (name == "reach") {
        reach = std::max(reach, v.layer);
      } else if (name == "lonely") {
        lonely = std::max(lonely, v.layer);
      }
    }
  }
  EXPECT_LT(reach, lonely);
  EXPECT_GE(t.axiom_layers, 2);
  for (size_t i = 1; i < t.axioms.size(); ++i) {
    EXPECT_LE(t.variables[t.axioms[i - 1].head.var].layer, t.variables[t.axioms[i].head.var].layer);
  }
}

TEST(TranslateTest, CyclesThroughNegationAreErrors)
{
  // The semantic analysis rejects such domains; the translation checks again, on a ground task
  // built by hand: p :- not q, q :- not p
  ground::Task g;
  g.predicate_names = {"p", "q"};
  g.derived = {true, true};
  g.atoms = {ground::AtomInfo{sem::PredicateId(0), {}}, ground::AtomInfo{sem::PredicateId(1), {}}};
  ground::Axiom p;
  p.head = 0;
  p.body.negative = {1};
  ground::Axiom q;
  q.head = 1;
  q.body.negative = {0};
  g.axioms = {p, q};
  lifted::Task lt;
  TranslateOptions o;
  o.use_invariants = false;
  auto r = translate(lt, g, o);
  ASSERT_FALSE(r.ok());
  EXPECT_EQ(r.diagnostics()[0].code, "sas.unstratified");
  EXPECT_NE(r.diagnostics()[0].message.find("through a negation"), std::string::npos);
}

TEST(TranslateTest, LinearExpressions)
{
  using ground::Expr;
  auto fluent = [](uint32_t f) {Expr e; e.op = ExprOp::Fluent; e.fluent = f; return e;};
  auto constant = [](double v) {Expr e; e.value = v; return e;};
  auto node = [](ExprOp op, std::vector<Expr> c) {Expr e; e.op = op; e.children = c; return e;};
  // 2*f0 - (f1 / 4) + 3
  auto l = linearize(
    node(ExprOp::Add, {node(ExprOp::Sub, {node(ExprOp::Mul, {constant(2), fluent(0)}),
        node(ExprOp::Div, {fluent(1), constant(4)})}), constant(3)}));
  ASSERT_TRUE(l.has_value());
  EXPECT_EQ(l->constant, 3.0);
  EXPECT_EQ(l->terms, (std::vector<std::pair<NumVarId, double>>{{0, 2.0}, {1, -0.25}}));
  // f0 - f0 cancels
  auto z = linearize(node(ExprOp::Sub, {fluent(0), fluent(0)}));
  ASSERT_TRUE(z.has_value());
  EXPECT_TRUE(z->terms.empty());
  // Not linear: products of fluents, division by a fluent or zero, powers, ?duration
  EXPECT_FALSE(linearize(node(ExprOp::Mul, {fluent(0), fluent(1)})).has_value());
  EXPECT_FALSE(linearize(node(ExprOp::Div, {constant(1), fluent(1)})).has_value());
  EXPECT_FALSE(linearize(node(ExprOp::Div, {fluent(1), constant(0)})).has_value());
  EXPECT_FALSE(linearize(node(ExprOp::Pow, {fluent(0), constant(2)})).has_value());
  Expr d;
  d.op = ExprOp::Duration;
  EXPECT_FALSE(linearize(d).has_value());
  // Unary minus
  auto neg = linearize(node(ExprOp::Sub, {fluent(2)}));
  ASSERT_TRUE(neg.has_value());
  EXPECT_EQ(neg->terms, (std::vector<std::pair<NumVarId, double>>{{2, -1.0}}));
}

TEST(TranslateTest, NumericConditionsAndEffects)
{
  auto p = pipeline(domain_with(
      "(:action go :parameters (?r - robot ?x ?y - place)"
      " :precondition (and (at ?r ?x) (link ?x ?y) (>= (energy ?r) (* 2 (dist ?x ?y))))"
      " :effect (and (not (at ?r ?x)) (at ?r ?y) (decrease (energy ?r) (dist ?x ?y))))"),
      problem_with("(at r a) (link a b) (= (energy r) 10) (= (dist a b) 3)", "(at r b)"));
  const Task & t = p->sas;
  ASSERT_EQ(t.actions.size(), 1u);
  const auto & a = t.actions[0];
  ASSERT_EQ(a.pre.numeric.size(), 1u);
  ASSERT_TRUE(a.pre.numeric[0].difference.has_value());
  // energy - 6 >= 0 (dist is static: a constant)
  EXPECT_EQ(a.pre.numeric[0].difference->constant, -6.0);
  ASSERT_EQ(a.effects.size(), 1u);
  EXPECT_EQ(a.effects[0].kind, Effect::Kind::Numeric);
  EXPECT_EQ(a.effects[0].op, ast::AssignOp::Decrease);
  ASSERT_TRUE(a.effects[0].value.linear.has_value());
  EXPECT_EQ(a.effects[0].value.linear->constant, 3.0);
  EXPECT_EQ(t.initial_numeric.at(a.effects[0].num), 10.0);
}

TEST(TranslateTest, ObjectFluentsBecomeVariables)
{
  auto p = pipeline(
    "(define (domain d) (:requirements :typing :object-fluents :equality) (:types place robot)"
    " (:predicates (ready))"
    " (:functions (loc ?r - robot) - place (dest ?r - robot) - place)"
    " (:action go :parameters (?r - robot ?l - place) :precondition (ready)"
    "  :effect (assign (loc ?r) ?l))"
    " (:action follow :parameters (?r ?s - robot) :precondition (= (loc ?r) (loc ?s))"
    "  :effect (assign (dest ?r) (loc ?s)))"
    " (:action check :parameters (?r - robot ?l - place) :precondition (= (loc ?r) ?l)"
    "  :effect (ready)))",
    "(define (problem p) (:domain d) (:objects r1 r2 - robot a b - place)"
    " (:init (= (loc r1) a) (ready)) (:goal (ready)))");
  const Task & t = p->sas;
  size_t objs = 0;
  for (const auto & v : t.variables) {
    if (v.kind == Variable::Kind::ObjectFluent) {
      ++objs;
      EXPECT_EQ(v.objects.size(), 2u);  // a and b (dest takes what loc holds)
    }
  }
  EXPECT_EQ(objs, 4u);
  bool copy = false, compare = false;
  for (const auto & a : t.actions) {
    for (const auto & e : a.effects) {
      copy = copy || e.kind == Effect::Kind::Copy;
    }
    compare = compare || !a.pre.vars.empty();
  }
  EXPECT_TRUE(copy);
  EXPECT_TRUE(compare);
}

TEST(TranslateTest, DurativeActionsAndTimedEvents)
{
  auto p = pipeline(
    "(define (domain d) (:requirements :durative-actions :typing :timed-initial-literals)"
    " (:types robot place) (:predicates (at ?r - robot ?l - place) (open))"
    " (:durative-action move :parameters (?r - robot ?a ?b - place) :duration (= ?duration 2)"
    "  :condition (and (at start (at ?r ?a)) (over all (open)))"
    "  :effect (and (at start (not (at ?r ?a))) (at end (at ?r ?b)))))",
    "(define (problem p) (:domain d) (:objects r - robot a b - place)"
    " (:init (at r a) (open) (at 5 (not (open)))) (:goal (at r b)))");
  const Task & t = p->sas;
  ASSERT_FALSE(t.durative_actions.empty());
  for (const auto & a : t.durative_actions) {
    EXPECT_EQ(a.start.size(), 1u);  // at := none (it was known true at start)
    EXPECT_EQ(t.variables[a.start[0].var].none(), a.start[0].value);
    EXPECT_EQ(a.end.size(), 1u);
    ASSERT_EQ(a.duration.size(), 1u);
    EXPECT_EQ(a.duration[0].value.linear->constant, 2.0);
    EXPECT_FALSE(a.uses_duration);
  }
  ASSERT_EQ(t.timed_events.size(), 1u);
  EXPECT_EQ(t.timed_events[0].time, 5.0);
}

TEST(TranslateTest, TextDump)
{
  auto p = pipeline(domain_with(kMove),
      problem_with("(at r a) (link a b)", "(at r b)"));
  EXPECT_EQ(to_string(p->sas),
    ";; SAS+ task d / pr\n"
    "(:variable v0 group (at r a) | (at r b) | (none-of v0))\n"
    "(:init (at r a))\n"
    "(:action (move r a b)\n"
    "  :precondition (at r a)\n"
    "  :effect\n    v0 := (at r b))\n"
    "(:goal (at r b))\n");
}

TEST(TranslateTest, TranslatingAMovedGroundTaskGivesTheSameAndReleasesIt)
{
  auto p = pipeline(
    read_file(data_path("plansys2/plansys2_executor__factory2.pddl")),
    read_file(data_path("plansys2/plansys2_executor__problem_factory_1.pddl")));
  ASSERT_TRUE(p->ok);
  ground::Task copy = p->ground;
  auto moved = translate(p->n->task, std::move(copy));
  ASSERT_TRUE(moved.ok()) << dump(moved.diagnostics());
  EXPECT_EQ(to_string(*moved), to_string(p->sas));
  EXPECT_TRUE(copy.durative_actions.empty());  // released after the move
  EXPECT_TRUE(copy.atoms.empty());
}

TEST(TranslateTest, LogsOnTheSasChannel)
{
  std::vector<std::string> lines;
  TranslateOptions o;
  LogOptions lo;
  lo.level = LogLevel::Info;
  lo.callback = [&](LogLevel, LogChannel c, std::string_view m) {
      if (c == LogChannel::Sas) {
        lines.emplace_back(m);
      }
    };
  o.logger = Logger(lo);
  auto p = pipeline(domain_with(kMove), problem_with("(at r a) (link a b)", "(at r b)"), o);
  ASSERT_EQ(lines.size(), 1u);
  EXPECT_NE(lines[0].find("1 variables"), std::string::npos) << lines[0];
}

// --- Equivalence with the ground task ----------------------------------------------------------

// Same applicable instances and same next states, step by step
::testing::AssertionResult walk(const kairos::testing::Pipeline & p, uint32_t seed, int steps)
{
  const ground::Task & g = p.ground;
  const Task & t = p.sas;
  ground::testing::GroundSimulator gs(g);
  testing::SasSimulator ss(t);
  auto gstate = gs.initial();
  auto sstate = ss.initial();
  std::mt19937 rng(seed);
  std::map<std::string, std::vector<size_t>> gi, si, gd, sd;
  for (size_t i = 0; i < g.actions.size(); ++i) {
    gi[ground::instance_name(g, g.actions[i])].push_back(i);
  }
  for (size_t i = 0; i < t.actions.size(); ++i) {
    si[instance_name(t, t.actions[i])].push_back(i);
  }
  for (size_t i = 0; i < g.durative_actions.size(); ++i) {
    gd[ground::instance_name(g, g.durative_actions[i])].push_back(i);
  }
  for (size_t i = 0; i < t.durative_actions.size(); ++i) {
    sd[instance_name(t, t.durative_actions[i])].push_back(i);
  }
  std::string trace;
  for (int step = 0; step < steps; ++step) {
    if (!(ss.to_ground(sstate) == gstate)) {
      return ::testing::AssertionFailure() << "states differ after\n" << trace;
    }
    if (ss.goal(sstate) != gs.goal(gstate)) {
      return ::testing::AssertionFailure() << "goal verdicts differ after\n" << trace;
    }
    struct Option
    {
      std::string name;
      ground::testing::GState g;
      testing::SState s;
    };
    std::vector<Option> options;
    for (const auto & [name, list] : gi) {
      std::optional<ground::testing::GState> gn;
      for (size_t i : list) {
        if (gs.applicable(g.actions[i], gstate)) {
          gn = gs.apply(g.actions[i].changes, g.actions[i].effects, Timing::Instant, gstate);
          break;
        }
      }
      std::optional<testing::SState> sn;
      for (size_t i : si[name]) {
        if (ss.holds(t.actions[i].pre, sstate)) {
          sn = ss.apply(t.actions[i].changes, t.actions[i].effects, Timing::Instant, sstate);
          break;
        }
      }
      if (gn.has_value() != sn.has_value()) {
        return ::testing::AssertionFailure() << name << " is " << (gn ? "" : "not ") <<
               "applicable in the ground task but " << (sn ? "" : "not ") << "in SAS+ after\n" <<
               trace;
      }
      if (gn) {
        options.push_back(Option{name, *gn, *sn});
      }
    }
    for (const auto & [name, list] : gd) {
      const auto & ga = g.durative_actions[list[0]];
      double d = 1.0;
      for (const auto & c : ga.duration) {
        if (c.op == CompareOp::Eq) {
          d = gs.number(c.value, gstate);
        }
      }
      std::optional<ground::testing::GState> gn;
      for (size_t i : list) {
        gn = gs.run(g.durative_actions[i], gstate, d);
        if (gn) {
          break;
        }
      }
      std::optional<testing::SState> sn;
      for (size_t i : sd[name]) {
        sn = ss.run(t.durative_actions[i], sstate, d);
        if (sn) {
          break;
        }
      }
      if (gn.has_value() != sn.has_value()) {
        return ::testing::AssertionFailure() << name << " (durative) is " << (gn ? "" : "not ") <<
               "applicable in the ground task but " << (sn ? "" : "not ") << "in SAS+ after\n" <<
               trace;
      }
      if (gn) {
        options.push_back(Option{name, *gn, *sn});
      }
    }
    if (options.empty()) {
      break;
    }
    auto & o = options[std::uniform_int_distribution<size_t>(0, options.size() - 1)(rng)];
    gstate = o.g;
    sstate = o.s;
    trace += o.name + "\n";
  }
  if (ss.linear_mismatch()) {
    return ::testing::AssertionFailure() << "a linear form disagrees with its expression";
  }
  return ::testing::AssertionSuccess();
}

::testing::AssertionResult walks(const std::string & domain, const std::string & problem, int n)
{
  ground::GroundOptions keep;
  keep.drop_useless_actions = false;
  auto p = pipeline(domain, problem, {}, keep);
  if (!p->ok) {
    return ::testing::AssertionFailure() << "pipeline failed";
  }
  for (int i = 0; i < n; ++i) {
    auto r = walk(*p, static_cast<uint32_t>(i), 30);
    if (!r) {
      return r;
    }
  }
  return ::testing::AssertionSuccess();
}

TEST(TranslateEquivalenceTest, SmallDomains)
{
  EXPECT_TRUE(walks(domain_with(std::string(kMove) +
      " (:action clear :parameters (?r - robot ?l - place) :precondition (on ?r)"
      "  :effect (and (not (at ?r ?l)) (not (on ?r))))"
      " (:action put :parameters (?r - robot ?l - place) :precondition (not (on ?r))"
      "  :effect (and (on ?r) (when (dirty ?l) (and (at ?r ?l) (not (dirty ?l))))))"
      " (:action go :parameters (?r - robot ?x ?y - place)"
      "  :precondition (and (at ?r ?x) (link ?x ?y) (>= (energy ?r) (dist ?x ?y)))"
      "  :effect (and (not (at ?r ?x)) (at ?r ?y) (decrease (energy ?r) (dist ?x ?y))))"),
    problem_with("(at r a) (link a b) (link b c) (link c a) (on r) (dirty b) (= (energy r) 9)"
    " (= (dist a b) 2) (= (dist b c) 3) (= (dist c a) 4)", "(at r c)"), 20));
}

TEST(TranslateEquivalenceTest, PlanSys2Domains)
{
  const std::pair<const char *, const char *> pairs[] = {
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
  for (const auto & [d, p] : pairs) {
    std::string dom = read_file(data_path(std::string("plansys2/") + d + ".pddl"));
    std::string prob = read_file(data_path(std::string("plansys2/") + p + ".pddl"));
    EXPECT_TRUE(walks(dom, prob, 5)) << d;
  }
}

TEST(TranslateEquivalenceTest, IpcDomains)
{
  const char * dir = std::getenv("KAIROS_PDDL_CORPUS");
  if (dir == nullptr) {
    GTEST_SKIP() << "KAIROS_PDDL_CORPUS not set";
  }
  std::filesystem::path base = std::filesystem::path(dir) / "potassco_pddl-instances";
  const char * domains[] = {
    "ipc-1998/domains/assembly-round-1-adl", "ipc-2000/domains/elevator-adl-full-typed",
    "ipc-2000/domains/schedule-adl-typed", "ipc-2002/domains/depots-strips-automatic",
    "ipc-2002/domains/driverlog-numeric-automatic", "ipc-2002/domains/zenotravel-time-automatic",
    "ipc-2002/domains/rovers-time-automatic", "ipc-2004/domains/psr-small-strips",
    "ipc-2004/domains/promela-dining-philosophers-derived-predicates-adl",
    "ipc-2004/domains/airport-nontemporal-adl", "ipc-2006/domains/trucks-propositional",
    "ipc-2008/domains/openstacks-sequential-satisficing-adl",
    "ipc-2011/domains/parking-temporal-satisficing",
  };
  size_t tried = 0;
  for (const char * dom : domains) {
    auto d = base / dom / "domain.pddl";
    auto p = base / dom / "instances" / "instance-1.pddl";
    if (!std::filesystem::exists(d)) {
      d = base / dom / "domains" / "domain-1.pddl";
    }
    if (!std::filesystem::exists(p) || !std::filesystem::exists(d)) {
      continue;
    }
    ++tried;
    EXPECT_TRUE(walks(read_file(d), read_file(p), 3)) << dom;
  }
  EXPECT_GT(tried, 0u);
}

// --- Fast Downward ---------------------------------------------------------------------------

std::string lower(std::string s)
{
  std::transform(
    s.begin(), s.end(), s.begin(), [](unsigned char c) {
      return static_cast<char>(std::tolower(c));
        });
  return s;
}

std::vector<std::string> run(const std::string & cmd, const std::string & input = "")
{
  std::string full = cmd;
  std::string tmp;
  if (!input.empty()) {
    tmp = (std::filesystem::temp_directory_path() / ("kairos-inv-" +
      std::to_string(std::random_device{}()) + ".txt")).string();
    FILE * f = std::fopen(tmp.c_str(), "w");
    std::fputs(input.c_str(), f);
    std::fclose(f);
    full += " < '" + tmp + "'";
  }
  std::vector<std::string> out;
  FILE * pipe = popen((full + " 2>/dev/null").c_str(), "r");
  if (pipe) {
    char buf[1 << 16];
    while (fgets(buf, sizeof(buf), pipe)) {
      std::string line(buf);
      while (!line.empty() && (line.back() == '\n' || line.back() == '\r')) {
        line.pop_back();
      }
      out.push_back(line);
    }
    pclose(pipe);
  }
  if (!tmp.empty()) {
    std::filesystem::remove(tmp);
  }
  return out;
}

// With KAIROS_FD_TRANSLATE and the corpus: every invariant Kairos proves is accepted by FD's
// balance checker, and the mutex groups are FD's (except where grounding is tighter, or FD's
// random order finds other invariants)
TEST(TranslateFdTest, SameMutexGroupsAsFastDownward)
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
    {"potassco_pddl-instances/ipc-1998/domains/logistics-round-1-strips", "", true},
    {"potassco_pddl-instances/ipc-2000/domains/blocks-strips-typed", "", true},
    {"potassco_pddl-instances/ipc-2000/domains/freecell-strips-typed", "", true},
    {"potassco_pddl-instances/ipc-2002/domains/depots-strips-automatic", "", true},
    {"potassco_pddl-instances/ipc-2002/domains/driverlog-strips-automatic", "", true},
    {"potassco_pddl-instances/ipc-2002/domains/rovers-strips-automatic", "", true},
    {"potassco_pddl-instances/ipc-2002/domains/satellite-strips-automatic", "", true},
    {"potassco_pddl-instances/ipc-2004/domains/pipesworld-no-tankage-nontemporal-strips", "",
      true},
    {"potassco_pddl-instances/ipc-2006/domains/tpp-propositional-strips", "", true},
    {"potassco_pddl-instances/ipc-2006/domains/trucks-propositional-strips", "", true},
    {"potassco_pddl-instances/ipc-2008/domains/elevator-sequential-satisficing-strips", "", true},
    {"potassco_pddl-instances/ipc-2008/domains/woodworking-sequential-satisficing-strips", "",
      true},
    {"potassco_pddl-instances/ipc-2011/domains/barman-sequential-satisficing", "", true},
    {"potassco_pddl-instances/ipc-2011/domains/visitall-sequential-satisficing", "", true},
    {"potassco_pddl-instances/ipc-2014/domains/tetris-sequential-optimal", "", true},
    {"potassco_pddl-instances/ipc-2014/domains/city-car-sequential-optimal", "", false},
    {"ipc2023-classical_ipc2023-dataset/opt/folding", "p01.pddl", true},
    {"ipc2023-classical_ipc2023-dataset/opt/rubiks-cube", "p01.pddl", true},
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
    ++tried;
    auto pl = pipeline(read_file(d), read_file(p));
    ASSERT_TRUE(pl->ok) << c.path;
    auto invariants = find_invariants(pl->n->task, pl->ground);
    // Invariants in the checker's syntax
    std::string input;
    for (const auto & inv : invariants) {
      std::string line;
      for (const auto & part : inv.parts) {
        line += (line.empty() ? "" : ";") +
          lower(pl->n->task.predicate_names[part.predicate.index()]);
        for (int a : part.args) {
          line += a == kCounted ? std::string(" *") : " " + std::to_string(a);
        }
      }
      input += line + "\n";
    }
    std::string args = std::string(" '") + fd + "' '" + d.string() + "' '" + p.string() + "'";
    if (!input.empty()) {
      auto checked = run("python3 " + std::string(KAIROS_FD_CHECK_SCRIPT) + args, input);
      EXPECT_EQ(checked.size(), invariants.size()) << c.path;
      for (const auto & line : checked) {
        EXPECT_EQ(line.rfind("OK ", 0), 0u) << c.path << ": " << line;
      }
    }
    // Groups
    auto theirs = run("python3 " + std::string(KAIROS_FD_GROUPS_SCRIPT) + args);
    std::set<std::string> fd_groups(theirs.begin(), theirs.end());
    std::set<std::string> ours;
    auto groups = mutex_groups(invariants, pl->ground);
    auto chosen = choose_groups(groups, pl->ground);
    const std::pair<const char *, const std::vector<std::vector<ground::AtomId>> *> kinds[] = {
      {"GROUP", &groups}, {"CHOSEN", &chosen}};
    for (const auto & [kind, list] : kinds) {
      for (const auto & grp : *list) {
        if (grp.size() < 2) {
          continue;
        }
        std::vector<std::string> names;
        for (auto a : grp) {
          names.push_back(lower(ground::atom_name(pl->ground, a)));
        }
        std::sort(names.begin(), names.end());
        std::string s = kind;
        for (size_t i = 0; i < names.size(); ++i) {
          s += (i ? "|" : " ") + names[i];
        }
        ours.insert(s);
      }
    }
    if (c.exact) {
      EXPECT_EQ(ours, fd_groups) << c.path;
    }
  }
  EXPECT_GT(tried, 0u);
}

}  // namespace
}  // namespace kairos::sas
