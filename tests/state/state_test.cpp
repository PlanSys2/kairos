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

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <deque>
#include <filesystem>
#include <limits>
#include <map>
#include <memory>
#include <random>
#include <set>
#include <string>
#include <vector>

#include "state/eval.hpp"
#include "state/packing.hpp"
#include "state/registry.hpp"
#include "state/successors.hpp"
#include "support/pddl_helpers.hpp"
#include "support/pipeline_helpers.hpp"
#include "support/sas_simulator.hpp"

namespace kairos::state
{
namespace
{

using kairos::testing::data_path;
using kairos::testing::pipeline;
using kairos::testing::read_file;

// --- Packing ---------------------------------------------------------------------------------

TEST(PackingTest, BitsForDomains)
{
  EXPECT_EQ(bits_for(0), 1u);
  EXPECT_EQ(bits_for(1), 1u);
  EXPECT_EQ(bits_for(2), 1u);
  EXPECT_EQ(bits_for(3), 2u);
  EXPECT_EQ(bits_for(4), 2u);
  EXPECT_EQ(bits_for(5), 3u);
  EXPECT_EQ(bits_for(256), 8u);
  EXPECT_EQ(bits_for(257), 9u);
  EXPECT_EQ(bits_for(uint64_t{1} << 63), 63u);
  EXPECT_EQ(bits_for((uint64_t{1} << 63) + 1), 64u);
  EXPECT_EQ(bits_for(std::numeric_limits<uint64_t>::max()), 64u);
}

TEST(PackingTest, OneTwoSixtyThreeAndSixtyFourBits)
{
  PackedLayout layout({1, 2, 63, 64, 1, 2});
  // 64 alone; 63 + 1; 2 + 2 + 1 ... first fit by decreasing width
  EXPECT_EQ(layout.words(), 3u);
  std::vector<uint64_t> buf(layout.words(), 0);
  std::mt19937_64 rng(1);
  std::vector<uint64_t> mask = {1, 3, (uint64_t{1} << 63) - 1, ~uint64_t{0}, 1, 3};
  for (int round = 0; round < 1000; ++round) {
    std::vector<uint64_t> want(6);
    for (size_t v = 0; v < 6; ++v) {
      want[v] = rng() & mask[v];
      layout.set(buf.data(), v, want[v]);
    }
    for (size_t v = 0; v < 6; ++v) {
      ASSERT_EQ(layout.get(buf.data(), v), want[v]) << "variable " << v;
    }
    // Rewriting one leaves the others alone
    size_t v = rng() % 6;
    want[v] = rng() & mask[v];
    layout.set(buf.data(), v, want[v]);
    for (size_t w = 0; w < 6; ++w) {
      ASSERT_EQ(layout.get(buf.data(), w), want[w]);
    }
  }
}

TEST(PackingTest, ManySmallVariablesShareWords)
{
  PackedLayout layout(std::vector<uint32_t>(100, 1));
  EXPECT_EQ(layout.words(), 2u);
  PackedLayout three(std::vector<uint32_t>(64, 3));  // 21 per word
  EXPECT_EQ(three.words(), 4u);
  EXPECT_THROW(PackedLayout({0}), std::invalid_argument);
  EXPECT_THROW(PackedLayout({65}), std::invalid_argument);
}

// --- Registry --------------------------------------------------------------------------------

std::string domain_with(const std::string & body)
{
  return "(define (domain d) (:requirements :adl :typing :fluents :derived-predicates)"
         " (:types robot place) (:constants r - robot a b c - place)"
         " (:predicates (at ?r - robot ?l - place) (link ?a ?b - place) (on ?r - robot)"
         "   (p) (q) (s) (near ?l - place))"
         " (:functions (energy ?r - robot) (x) (y)) " + body + ")";
}

std::string problem_with(const std::string & init, const std::string & goal = "(p)")
{
  return "(define (problem pr) (:domain d) (:init " + init + ") (:goal " + goal + "))";
}

const char * kMove =
  "(:action move :parameters (?r - robot ?x ?y - place)"
  " :precondition (and (at ?r ?x) (link ?x ?y)) :effect (and (not (at ?r ?x)) (at ?r ?y)))";

TEST(RegistryTest, StatesAreStoredOnce)
{
  auto p = pipeline(domain_with(std::string(kMove) +
      " (:action inc :parameters () :precondition () :effect (increase (x) 1))"),
      problem_with("(at r a) (link a b) (link b a) (= (x) 1)", "(at r b)"));
  ASSERT_EQ(p->sas.numeric_fluents.size(), 1u);
  StateRegistry reg(p->sas);
  StateId s0 = reg.initial();
  EXPECT_EQ(reg.initial(), s0);
  std::vector<uint32_t> v;
  reg.unpack(s0, v);
  std::vector<double> n(reg.numbers(s0).begin(), reg.numbers(s0).end());
  auto [again, fresh] = reg.insert(v, n);
  EXPECT_EQ(again, s0);
  EXPECT_FALSE(fresh);
  // Another number: another state; -0.0 and 0.0 are the same; NaNs are the same
  n[0] = 2.0;
  auto [s1, f1] = reg.insert(v, n);
  EXPECT_NE(s1, s0);
  EXPECT_TRUE(f1);
  n[0] = 0.0;
  auto zero = reg.insert(v, n).first;
  n[0] = -0.0;
  EXPECT_EQ(reg.insert(v, n).first, zero);
  n[0] = std::nan("1");
  auto undef = reg.insert(v, n).first;
  n[0] = -std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(reg.insert(v, n).first, undef);
  EXPECT_EQ(reg.size(), 4u);
  EXPECT_EQ(reg.numbers(s1)[0], 2.0);
}

TEST(RegistryTest, DerivedVariablesAreRecomputedOnInsert)
{
  auto p = pipeline(domain_with(std::string(kMove) +
      " (:derived (near ?l - place) (exists (?r - robot) (at ?r ?l)))"
      " (:derived (p) (and (near b) (not (near a))))"),
      problem_with("(at r a) (link a b)", "(p)"));
  const auto & t = p->sas;
  StateRegistry reg(t);
  StateId s0 = reg.initial();
  std::vector<uint32_t> v;
  reg.unpack(s0, v);
  // Corrupt the derived values: inserting gives the same state back
  std::vector<uint32_t> bad = v;
  for (sas::VarId x = 0; x < t.variables.size(); ++x) {
    if (t.variables[x].kind == sas::Variable::Kind::Derived) {
      bad[x] = 1 - bad[x];
    }
  }
  std::vector<double> n;
  EXPECT_EQ(reg.insert(bad, n).first, s0);
  // (near a) is true at the start, (p) false; after moving, (p) holds
  StateSpace space(t, reg);
  std::vector<uint32_t> ops;
  space.applicable(s0, ops);
  ASSERT_EQ(ops.size(), 1u);
  EXPECT_FALSE(space.is_goal(s0));
  auto s1 = space.successor(s0, ops[0]);
  ASSERT_TRUE(s1.has_value());
  EXPECT_TRUE(space.is_goal(*s1));
}

TEST(RegistryTest, BytesPerStateWithinTheTarget)
{
  // 40 binary variables (one word) and two numbers: per state 8 + 16 bytes, plus at most
  // about 21 bytes of table (load from 3/8 to 3/4)
  std::string preds, objects;
  for (int i = 0; i < 40; ++i) {
    objects += " o" + std::to_string(i);
  }
  auto p = pipeline(
    "(define (domain d) (:requirements :fluents) (:predicates (b ?x))"
    " (:functions (u) (w))"
    " (:action set :parameters (?x) :precondition () :effect (b ?x))"
    " (:action unset :parameters (?x) :precondition (b ?x) :effect (not (b ?x)))"
    " (:action inc :parameters () :precondition () :effect (and (increase (u) 1)"
    "   (increase (w) 1))))",
    "(define (problem p) (:domain d) (:objects" + objects + ") (:init (= (u) 0) (= (w) 0))"
    " (:goal (b o1)))");
  const auto & t = p->sas;
  ASSERT_EQ(t.variables.size(), 40u);
  BudgetConfig cfg;
  Budget budget(cfg);
  size_t states = 6 * (1 << 14);  // whole blocks
  {
    StateRegistry reg(t, &budget);
    EXPECT_EQ(reg.layout().words(), 1u);
    std::mt19937_64 rng(3);
    std::vector<uint32_t> v(40);
    std::vector<double> n(2);
    while (reg.size() < states) {
      uint64_t bits = rng();
      for (size_t i = 0; i < 40; ++i) {
        v[i] = static_cast<uint32_t>((bits >> i) & 1);
      }
      n[0] = static_cast<double>(rng() % 7);
      reg.insert(v, n);
    }
    EXPECT_LE(reg.bytes_per_state(), 8.0 + 16.0 + 22.0) << reg.bytes_per_state();
    EXPECT_EQ(budget.accounted_bytes(), reg.bytes());
  }
  EXPECT_EQ(budget.accounted_bytes(), 0u);  // given back
}

// --- Effects ---------------------------------------------------------------------------------

struct Space
{
  std::unique_ptr<kairos::testing::Pipeline> p;
  std::unique_ptr<StateRegistry> reg;
  std::unique_ptr<StateSpace> space;
};

Space space_for(const std::string & body, const std::string & init)
{
  Space s;
  ground::GroundOptions keep;
  keep.drop_useless_actions = false;
  s.p = pipeline(domain_with(body), problem_with(init), {}, keep);
  s.reg = std::make_unique<StateRegistry>(s.p->sas);
  s.space = std::make_unique<StateSpace>(s.p->sas, *s.reg);
  return s;
}

uint32_t action(const Space & s, const std::string & name)
{
  for (uint32_t i = 0; i < s.p->sas.actions.size(); ++i) {
    if (sas::instance_name(s.p->sas, s.p->sas.actions[i]) == name) {
      return i;
    }
  }
  ADD_FAILURE() << "no action " << name;
  return 0;
}

bool atom(const Space & s, StateId id, const std::string & name)
{
  std::vector<uint32_t> v;
  s.reg->unpack(id, v);
  for (sas::VarId x = 0; x < v.size(); ++x) {
    if (sas::fact_name(s.p->sas, sas::Fact{x, v[x]}) == name) {
      return true;
    }
  }
  return false;
}

TEST(EffectsTest, NumericEffectsReadTheStateBefore)
{
  // swap: x := y and y := x at once
  auto s = space_for(
    "(:action swap :parameters () :precondition ()"
    " :effect (and (assign (x) (y)) (assign (y) (x))))",
    "(= (x) 1) (= (y) 2)");
  auto next = s.space->successor(s.reg->initial(), action(s, "(swap)"));
  ASSERT_TRUE(next.has_value());
  auto n = s.reg->numbers(*next);
  auto t = s.p->sas;
  std::map<std::string, double> by_name;
  for (size_t i = 0; i < t.numeric_fluents.size(); ++i) {
    by_name[t.function_names[t.numeric_fluents[i].function.index()]] = n[i];
  }
  EXPECT_EQ(by_name["x"], 2.0);
  EXPECT_EQ(by_name["y"], 1.0);
}

TEST(EffectsTest, ConditionalEffectsReadTheStateBefore)
{
  // make adds (p), and when (p) adds (q): (p) was false before, so (q) is not added
  auto s = space_for(
    "(:action make :parameters () :precondition () :effect (and (p) (when (p) (q))))", "(s)");
  auto next = s.space->successor(s.reg->initial(), action(s, "(make)"));
  ASSERT_TRUE(next.has_value());
  EXPECT_TRUE(atom(s, *next, "(p)"));
  EXPECT_FALSE(atom(s, *next, "(q)"));
  auto twice = s.space->successor(*next, action(s, "(make)"));
  EXPECT_TRUE(atom(s, *twice, "(q)"));
}

TEST(EffectsTest, DeleteThenAddKeepsTheAtom)
{
  auto s = space_for(std::string(kMove) +
      " (:action stay :parameters (?r - robot ?l - place) :precondition (at ?r ?l)"
      "  :effect (and (not (at ?r ?l)) (at ?r ?l)))",
      "(at r a) (link a b)");
  auto next = s.space->successor(s.reg->initial(), action(s, "(stay r a)"));
  ASSERT_TRUE(next.has_value());
  EXPECT_EQ(*next, s.reg->initial());
}

TEST(EffectsTest, ConflictingEffectsMakeTheStepInvalid)
{
  // Two conditional assignments of one object fluent: both fire only after (ps)
  Space s;
  ground::GroundOptions keep;
  keep.drop_useless_actions = false;
  s.p = pipeline(
    "(define (domain d) (:requirements :typing :object-fluents :conditional-effects)"
    " (:types place robot) (:predicates (p) (s))"
    " (:functions (loc ?r - robot) - place)"
    " (:action split :parameters (?r - robot ?a ?b - place) :precondition ()"
    "  :effect (and (when (s) (assign (loc ?r) ?a)) (when (p) (assign (loc ?r) ?b))))"
    " (:action ps :parameters () :precondition () :effect (and (p) (s)))"
    " (:action only :parameters () :precondition () :effect (s)))",
    "(define (problem pr) (:domain d) (:objects r - robot a b - place) (:init (= (loc r) a))"
    " (:goal (p)))", {}, keep);
  s.reg = std::make_unique<StateRegistry>(s.p->sas);
  s.space = std::make_unique<StateSpace>(s.p->sas, *s.reg);
  StateId s0 = s.reg->initial();
  auto both = s.space->successor(s0, action(s, "(ps)"));
  ASSERT_TRUE(both.has_value());
  EXPECT_FALSE(s.space->successor(*both, action(s, "(split r a b)")).has_value());
  EXPECT_TRUE(s.space->successor(*both, action(s, "(split r a a)")).has_value());  // same value
  auto one = s.space->successor(s0, action(s, "(only)"));
  ASSERT_TRUE(one.has_value());
  EXPECT_TRUE(s.space->successor(*one, action(s, "(split r a b)")).has_value());
}

TEST(EffectsTest, ChangingAnUndefinedFluentOrDividingByZeroIsInvalid)
{
  auto s = space_for(
    "(:action inc :parameters () :precondition (p) :effect (increase (x) 1))"
    " (:action div :parameters () :precondition () :effect (assign (y) (/ 1 (y))))"
    " (:action def :parameters () :precondition () :effect (and (p) (assign (x) 5)))",
    "(= (y) 0)");
  StateId s0 = s.reg->initial();
  EXPECT_FALSE(s.space->successor(s0, action(s, "(div)")).has_value());
  auto defined = s.space->successor(s0, action(s, "(def)"));
  ASSERT_TRUE(defined.has_value());
  EXPECT_TRUE(s.space->successor(*defined, action(s, "(inc)")).has_value());
}

TEST(EffectsTest, SeveralChangesOfOneFluentApplyInOrder)
{
  // As the validator: assign then increase on the same fluent gives value + increment
  auto s = space_for(
    "(:action both :parameters () :precondition () :effect (and (assign (x) 10) (increase (x) 2)"
    "  (increase (y) 1) (increase (y) 3)))",
    "(= (x) 1) (= (y) 0)");
  auto next = s.space->successor(s.reg->initial(), action(s, "(both)"));
  ASSERT_TRUE(next.has_value());
  const auto & t = s.p->sas;
  for (size_t i = 0; i < t.numeric_fluents.size(); ++i) {
    const auto & name = t.function_names[t.numeric_fluents[i].function.index()];
    if (name == "x") {
      EXPECT_EQ(s.reg->numbers(*next)[i], 12.0);
    } else if (name == "y") {
      EXPECT_EQ(s.reg->numbers(*next)[i], 4.0);
    }
  }
}

// --- Successor generation --------------------------------------------------------------------

std::set<uint32_t> brute_force(
  const sas::Task & t, const std::vector<uint32_t> & v, const std::vector<double> & n)
{
  std::set<uint32_t> out;
  for (uint32_t i = 0; i < t.actions.size(); ++i) {
    if (holds(t.actions[i].pre, t, v.data(), n.data())) {
      out.insert(i);
    }
  }
  return out;
}

std::vector<const sas::Condition *> pres(const sas::Task & t)
{
  std::vector<const sas::Condition *> out;
  for (const auto & a : t.actions) {
    out.push_back(&a.pre);
  }
  return out;
}

::testing::AssertionResult generator_agrees(const sas::Task & t, uint32_t seed, int states)
{
  SuccessorGenerator gen(t, pres(t));
  std::mt19937 rng(seed);
  std::vector<uint32_t> v(t.variables.size());
  std::vector<double> n(t.numeric_fluents.size());
  for (int k = 0; k < states; ++k) {
    // Arbitrary states, reachable or not
    for (size_t x = 0; x < v.size(); ++x) {
      v[x] = std::uniform_int_distribution<uint32_t>(0, t.variables[x].size() - 1)(rng);
    }
    for (auto & x : n) {
      x = static_cast<double>(std::uniform_int_distribution<int>(-2, 12)(rng));
    }
    std::vector<uint32_t> got;
    gen.applicable(v.data(), n.data(), got);
    std::set<uint32_t> got_set(got.begin(), got.end());
    if (got_set.size() != got.size()) {
      return ::testing::AssertionFailure() << "an operator twice";
    }
    if (got_set != brute_force(t, v, n)) {
      return ::testing::AssertionFailure() << "different applicable sets in state " << k;
    }
  }
  return ::testing::AssertionSuccess();
}

const std::pair<const char *, const char *> kPlanSys2[] = {
  {"plansys2_domain_expert__domain_charging", "plansys2_domain_expert__problem_charging"},
  {"plansys2_executor__elevator_domain", "plansys2_executor__elevator_problem"},
  {"plansys2_executor__factory2", "plansys2_executor__problem_factory_1"},
  {"plansys2_executor__road_trip_domain", "plansys2_executor__road_trip_problem"},
  {"plansys2_executor__suave_domain", "plansys2_executor__suave_problem"},
  {"plansys2_pddl_parser__dom1", "plansys2_pddl_parser__prob1"},
  {"plansys2_simple_example__simple_example", "plansys2_simple_example__problem"},
  {"plansys2_tests__test_2", "plansys2_tests__problem_2"},
};

TEST(SuccessorGeneratorTest, AgreesWithBruteForce)
{
  auto p = pipeline(domain_with(std::string(kMove) +
      " (:action go :parameters (?r - robot ?x ?y - place)"
      "  :precondition (and (at ?r ?x) (link ?x ?y) (not (on ?r)) (>= (energy ?r) 3))"
      "  :effect (and (not (at ?r ?x)) (at ?r ?y) (decrease (energy ?r) 3)))"
      " (:action hop :parameters () :precondition (not (at r b)) :effect (p))"
      " (:action any :parameters () :precondition () :effect (q))"),
      problem_with("(at r a) (link a b) (link b c) (link c a) (link a c) (= (energy r) 5)"));
  EXPECT_TRUE(generator_agrees(p->sas, 1, 500));
  for (const auto & [d, pr] : kPlanSys2) {
    auto q = pipeline(read_file(data_path(std::string("plansys2/") + d + ".pddl")),
        read_file(data_path(std::string("plansys2/") + pr + ".pddl")));
    EXPECT_TRUE(generator_agrees(q->sas, 2, 300)) << d;
  }
}

TEST(SuccessorGeneratorTest, DeepTreesDoNotOverflowTheStack)
{
  // 30 000 objects, two facts per action: a don't-care chain 30 000 nodes long
  std::string objects;
  const int n = 30000;
  for (int i = 0; i < n; ++i) {
    objects += " o" + std::to_string(i);
  }
  auto p = pipeline(
    "(define (domain d) (:predicates (a ?x) (b ?x) (c ?x))"
    " (:action go :parameters (?x) :precondition (and (a ?x) (b ?x)) :effect (c ?x))"
    " (:action mk :parameters (?x) :precondition () :effect (and (a ?x) (b ?x))))",
    "(define (problem p) (:domain d) (:objects" + objects + ") (:init (a o0) (b o0))"
    " (:goal (c o1)))");
  const auto & t = p->sas;
  SuccessorGenerator gen(t, pres(t));
  std::vector<uint32_t> v = t.initial;
  std::vector<double> nums;
  std::vector<uint32_t> got;
  gen.applicable(v.data(), nums.data(), got);
  EXPECT_EQ(got.size(), static_cast<size_t>(n) + 1);  // every mk, and go o0
}

// --- Walks: the state space against the test simulator ----------------------------------------

::testing::AssertionResult walk(const kairos::testing::Pipeline & p, uint32_t seed, int steps)
{
  const auto & t = p.sas;
  StateRegistry reg(t);
  StateSpace space(t, reg);
  sas::testing::SasSimulator sim(t);
  auto expected = sim.initial();
  StateId s = reg.initial();
  std::mt19937 rng(seed);
  for (int step = 0; step < steps; ++step) {
    std::vector<uint32_t> v;
    reg.unpack(s, v);
    std::vector<double> n(reg.numbers(s).begin(), reg.numbers(s).end());
    if (v != expected.values) {
      return ::testing::AssertionFailure() << "different values at step " << step;
    }
    for (size_t i = 0; i < n.size(); ++i) {
      auto e = expected.numbers[i];
      if (e.has_value() != !std::isnan(n[i]) || (e && *e != n[i])) {
        return ::testing::AssertionFailure() << "different numbers at step " << step;
      }
    }
    if (space.is_goal(s) != sim.goal(expected)) {
      return ::testing::AssertionFailure() << "different goal verdicts at step " << step;
    }
    std::vector<uint32_t> ops;
    space.applicable(s, ops);
    std::set<uint32_t> mine(ops.begin(), ops.end());
    std::set<uint32_t> theirs;
    for (uint32_t i = 0; i < t.actions.size(); ++i) {
      if (sim.holds(t.actions[i].pre, expected)) {
        theirs.insert(i);
      }
    }
    if (mine != theirs) {
      return ::testing::AssertionFailure() << "different applicable actions at step " << step;
    }
    std::vector<uint32_t> valid;
    for (uint32_t op : ops) {
      if (space.successor(s, op)) {
        valid.push_back(op);
      }
    }
    if (valid.empty()) {
      break;
    }
    uint32_t op = valid[std::uniform_int_distribution<size_t>(0, valid.size() - 1)(rng)];
    s = *space.successor(s, op);
    expected = sim.apply(t.actions[op].changes, t.actions[op].effects, sas::Timing::Instant,
        expected);
  }
  return ::testing::AssertionSuccess();
}

TEST(StateSpaceTest, WalksAgreeWithTheSimulator)
{
  ground::GroundOptions keep;
  keep.drop_useless_actions = false;
  for (const auto & [d, pr] : kPlanSys2) {
    auto p = pipeline(read_file(data_path(std::string("plansys2/") + d + ".pddl")),
        read_file(data_path(std::string("plansys2/") + pr + ".pddl")), {}, keep);
    for (uint32_t seed = 0; seed < 5; ++seed) {
      EXPECT_TRUE(walk(*p, seed, 40)) << d;
    }
  }
}

TEST(StateSpaceTest, IpcDomainsAgreeWithTheSimulator)
{
  const char * dir = std::getenv("KAIROS_PDDL_CORPUS");
  if (dir == nullptr) {
    GTEST_SKIP() << "KAIROS_PDDL_CORPUS not set";
  }
  std::filesystem::path base = std::filesystem::path(dir) / "potassco_pddl-instances";
  const char * domains[] = {
    "ipc-1998/domains/assembly-round-1-adl", "ipc-2000/domains/elevator-adl-full-typed",
    "ipc-2002/domains/depots-strips-automatic", "ipc-2002/domains/driverlog-numeric-automatic",
    "ipc-2004/domains/psr-small-strips", "ipc-2004/domains/airport-nontemporal-adl",
    "ipc-2004/domains/promela-dining-philosophers-derived-predicates-adl",
    "ipc-2006/domains/trucks-propositional",
    "ipc-2008/domains/openstacks-sequential-satisficing-adl",
  };
  size_t tried = 0;
  for (const char * dom : domains) {
    auto d = base / dom / "domain.pddl";
    auto pr = base / dom / "instances" / "instance-3.pddl";
    if (!std::filesystem::exists(d)) {
      d = base / dom / "domains" / "domain-3.pddl";
    }
    if (!std::filesystem::exists(pr) || !std::filesystem::exists(d)) {
      continue;
    }
    ++tried;
    auto p = pipeline(read_file(d), read_file(pr));
    ASSERT_TRUE(p->ok) << dom;
    EXPECT_TRUE(generator_agrees(p->sas, 4, 100)) << dom;
    EXPECT_TRUE(walk(*p, 1, 30)) << dom;
  }
  EXPECT_GT(tried, 0u);
}

TEST(StateSpaceTest, BreadthFirstExpansionThroughput)
{
  // A small grid: breadth-first over every reachable state, as search will do
  std::string objects, links;
  const int side = 7;
  for (int i = 0; i < side * side; ++i) {
    objects += " c" + std::to_string(i);
    if (i % side + 1 < side) {
      links += " (link c" + std::to_string(i) + " c" + std::to_string(i + 1) + ")";
      links += " (link c" + std::to_string(i + 1) + " c" + std::to_string(i) + ")";
    }
    if (i + side < side * side) {
      links += " (link c" + std::to_string(i) + " c" + std::to_string(i + side) + ")";
      links += " (link c" + std::to_string(i + side) + " c" + std::to_string(i) + ")";
    }
  }
  auto p = pipeline(
    "(define (domain g) (:requirements :typing) (:types robot cell)"
    " (:predicates (at ?r - robot ?c - cell) (link ?a ?b - cell) (free ?c - cell))"
    " (:action move :parameters (?r - robot ?a ?b - cell)"
    "  :precondition (and (at ?r ?a) (link ?a ?b) (free ?b))"
    "  :effect (and (not (at ?r ?a)) (at ?r ?b) (not (free ?b)) (free ?a))))",
    "(define (problem p) (:domain g) (:objects r1 r2 r3 - robot" + objects + " - cell)"
    " (:init (at r1 c0) (at r2 c1) (at r3 c2)" + links +[&]() {
        std::string f;
        for (int i = 3; i < side * side; ++i) {
          f += " (free c" + std::to_string(i) + ")";
        }
        return f;
    } () + ") (:goal (at r1 c48)))");
  ASSERT_TRUE(p->ok);
  StateRegistry reg(p->sas);
  StateSpace space(p->sas, reg);
  std::deque<StateId> open{reg.initial()};
  std::vector<uint32_t> ops;
  size_t expanded = 0;
  auto t0 = std::chrono::steady_clock::now();
  while (!open.empty()) {
    StateId s = open.front();
    open.pop_front();
    ++expanded;
    ops.clear();
    space.applicable(s, ops);
    for (uint32_t op : ops) {
      size_t before = reg.size();
      auto next = space.successor(s, op);
      if (next && reg.size() > before) {
        open.push_back(*next);
      }
    }
  }
  double secs = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  EXPECT_EQ(expanded, 49u * 48u * 47u);  // three robots on distinct cells
  EXPECT_EQ(reg.size(), expanded);
#ifdef KAIROS_COVERAGE_BUILD
  EXPECT_LT(secs, 60.0);  // atomic counters on a CI runner: 22 s seen
#else
  EXPECT_LT(secs, 10.0);  // generous: debug and sanitizer builds
#endif
  RecordProperty(
    "states_per_second", std::to_string(static_cast<int>(static_cast<double>(expanded) / secs)));
}

}  // namespace
}  // namespace kairos::state
