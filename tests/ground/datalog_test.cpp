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
#include <random>
#include <set>
#include <stdexcept>
#include <vector>

#include "core/memory_info.hpp"
#include "ground/datalog.hpp"

namespace kairos::datalog
{
namespace
{

using Tuple = std::vector<uint32_t>;

Term V(uint32_t v) {return Term::var(v);}
Term C(uint32_t c) {return Term::constant(c);}

std::set<Tuple> model(const Program & p, uint32_t pred)
{
  std::set<Tuple> out;
  for (uint32_t f : p.facts(pred)) {
    auto a = p.args(f);
    out.emplace(a.begin(), a.end());
  }
  return out;
}

void fact(Program & p, uint32_t pred, Tuple args) {p.add_fact(pred, args);}

TEST(DatalogTest, FactsWithoutRules)
{
  Program p;
  uint32_t e = p.add_predicate(2, "e");
  fact(p, e, {1, 2});
  fact(p, e, {1, 2});  // duplicate
  ASSERT_TRUE(p.solve());
  EXPECT_EQ(model(p, e), (std::set<Tuple>{{1, 2}}));
  EXPECT_NE(p.find(e, Tuple{1, 2}), Program::kUnbound);
  EXPECT_EQ(p.find(e, Tuple{2, 1}), Program::kUnbound);
}

TEST(DatalogTest, TransitiveClosure)
{
  Program p;
  uint32_t e = p.add_predicate(2, "e");
  uint32_t path = p.add_predicate(2, "path");
  for (uint32_t i = 0; i < 20; ++i) {
    fact(p, e, {i, i + 1});
  }
  p.add_rule(Rule{Atom{path, {V(0), V(1)}}, {Atom{e, {V(0), V(1)}}}, 2, {}});
  p.add_rule(
    Rule{Atom{path, {V(0), V(2)}}, {Atom{path, {V(0), V(1)}}, Atom{e, {V(1), V(2)}}}, 3, {}});
  ASSERT_TRUE(p.solve());
  EXPECT_EQ(model(p, path).size(), 21u * 20u / 2u);
  EXPECT_NE(p.find(path, Tuple{0, 20}), Program::kUnbound);
  EXPECT_EQ(p.find(path, Tuple{20, 0}), Program::kUnbound);
}

TEST(DatalogTest, ConstantsInBodyAndHead)
{
  Program p;
  uint32_t e = p.add_predicate(2, "e");
  uint32_t out = p.add_predicate(2, "out");
  fact(p, e, {1, 2});
  fact(p, e, {3, 4});
  p.add_rule(Rule{Atom{out, {C(9), V(0)}}, {Atom{e, {C(3), V(0)}}}, 1, {}});
  ASSERT_TRUE(p.solve());
  EXPECT_EQ(model(p, out), (std::set<Tuple>{{9, 4}}));
}

TEST(DatalogTest, RepeatedVariableInAnAtom)
{
  Program p;
  uint32_t e = p.add_predicate(2, "e");
  uint32_t loop = p.add_predicate(1, "loop");
  fact(p, e, {1, 1});
  fact(p, e, {1, 2});
  fact(p, e, {3, 3});
  p.add_rule(Rule{Atom{loop, {V(0)}}, {Atom{e, {V(0), V(0)}}}, 1, {}});
  ASSERT_TRUE(p.solve());
  EXPECT_EQ(model(p, loop), (std::set<Tuple>{{1}, {3}}));
}

TEST(DatalogTest, SelfJoinOnTheSamePredicate)
{
  // Both conditions read e: a fact may pair with itself and with later ones
  Program p;
  uint32_t e = p.add_predicate(2, "e");
  uint32_t two = p.add_predicate(2, "two");
  fact(p, e, {1, 1});
  fact(p, e, {1, 2});
  fact(p, e, {2, 3});
  p.add_rule(
    Rule{Atom{two, {V(0), V(2)}}, {Atom{e, {V(0), V(1)}}, Atom{e, {V(1), V(2)}}}, 3, {}});
  ASSERT_TRUE(p.solve());
  EXPECT_EQ(model(p, two), (std::set<Tuple>{{1, 1}, {1, 2}, {1, 3}}));
}

TEST(DatalogTest, ProductWithoutSharedVariables)
{
  Program p;
  uint32_t a = p.add_predicate(1, "a");
  uint32_t b = p.add_predicate(1, "b");
  uint32_t ab = p.add_predicate(2, "ab");
  fact(p, a, {1});
  fact(p, a, {2});
  fact(p, b, {7});
  fact(p, b, {8});
  fact(p, b, {9});
  p.add_rule(Rule{Atom{ab, {V(0), V(1)}}, {Atom{a, {V(0)}}, Atom{b, {V(1)}}}, 2, {}});
  ASSERT_TRUE(p.solve());
  EXPECT_EQ(model(p, ab).size(), 6u);
}

TEST(DatalogTest, ChecksFilterBindings)
{
  Program p;
  uint32_t n = p.add_predicate(1, "n");
  uint32_t ne = p.add_predicate(2, "ne");
  for (uint32_t i = 0; i < 4; ++i) {
    fact(p, n, {i});
  }
  Check different{{0, 1}, [](const uint32_t * b) {return b[0] != b[1];}};
  p.add_rule(Rule{Atom{ne, {V(0), V(1)}}, {Atom{n, {V(0)}}, Atom{n, {V(1)}}}, 2, {different}});
  ASSERT_TRUE(p.solve());
  EXPECT_EQ(model(p, ne).size(), 12u);
  EXPECT_EQ(p.find(ne, Tuple{2, 2}), Program::kUnbound);
}

TEST(DatalogTest, ChecksOnLongRulesAreAppliedOnce)
{
  // Four conditions: the check sees all its variables at some join and filters there
  Program p;
  uint32_t n = p.add_predicate(1, "n");
  uint32_t out = p.add_predicate(1, "out");
  for (uint32_t i = 0; i < 3; ++i) {
    fact(p, n, {i});
  }
  int calls = 0;
  Check sum{{0, 1, 2, 3}, [&calls](const uint32_t * b) {
      ++calls;
      return b[0] + b[1] + b[2] + b[3] == 8;
    }};
  p.add_rule(
    Rule{Atom{out, {V(0)}},
      {Atom{n, {V(0)}}, Atom{n, {V(1)}}, Atom{n, {V(2)}}, Atom{n, {V(3)}}}, 4, {sum}});
  ASSERT_TRUE(p.solve());
  // 8 needs all four at 2
  EXPECT_EQ(model(p, out), (std::set<Tuple>{{2}}));
  EXPECT_EQ(calls, 81);  // once per complete binding
}

TEST(DatalogTest, GroundRulesFireOnce)
{
  Program p;
  uint32_t on = p.add_predicate(0, "on");
  uint32_t a = p.add_predicate(1, "a");
  p.add_rule(Rule{Atom{on, {}}, {}, 0, {}});
  p.add_rule(Rule{Atom{a, {C(5)}}, {Atom{on, {}}}, 0, {}});
  ASSERT_TRUE(p.solve());
  EXPECT_EQ(model(p, on).size(), 1u);
  EXPECT_EQ(model(p, a), (std::set<Tuple>{{5}}));
}

TEST(DatalogTest, ZeroArityConditionsInJoins)
{
  Program p;
  uint32_t gate = p.add_predicate(0, "gate");
  uint32_t e = p.add_predicate(1, "e");
  uint32_t out = p.add_predicate(1, "out");
  fact(p, e, {1});
  p.add_rule(Rule{Atom{out, {V(0)}}, {Atom{gate, {}}, Atom{e, {V(0)}}}, 1, {}});
  ASSERT_TRUE(p.solve());
  EXPECT_TRUE(model(p, out).empty());  // the gate never opens
}

TEST(DatalogTest, UnboundHeadVariableIsAProgrammingError)
{
  Program p;
  uint32_t e = p.add_predicate(1, "e");
  uint32_t out = p.add_predicate(2, "out");
  EXPECT_THROW(
    p.add_rule(Rule{Atom{out, {V(0), V(1)}}, {Atom{e, {V(0)}}}, 2, {}}), std::logic_error);
  Check c{{1}, [](const uint32_t *) {return true;}};
  EXPECT_THROW(p.add_rule(Rule{Atom{e, {V(0)}}, {Atom{e, {V(0)}}}, 2, {c}}), std::logic_error);
  EXPECT_THROW(fact(p, e, {1, 2}), std::logic_error);
}

TEST(DatalogTest, BudgetStopsTheEvaluation)
{
  Program p;
  uint32_t n = p.add_predicate(1, "n");
  uint32_t pair = p.add_predicate(2, "pair");
  for (uint32_t i = 0; i < 2000; ++i) {
    fact(p, n, {i});
  }
  p.add_rule(Rule{Atom{pair, {V(0), V(1)}}, {Atom{n, {V(0)}}, Atom{n, {V(1)}}}, 2, {}});
  std::stop_source stop;
  stop.request_stop();
  BudgetConfig cfg;
  cfg.stop = stop.get_token();
  cfg.check_interval = 1;
  Budget budget(cfg);
  EXPECT_FALSE(p.solve(&budget));
  EXPECT_LT(p.stats().facts, 2000u + 2000u * 2000u);
}

TEST(DatalogTest, StopsWithinTheMemoryLimit)
{
  // 3000 x 3000 pairs need ~400 MB: the doubling vectors would pass a limit of 96 MB by far
  // between two full checks; reserving before each large allocation stops in time
  Program p;
  uint32_t n = p.add_predicate(1, "n");
  uint32_t pair = p.add_predicate(2, "pair");
  for (uint32_t i = 0; i < 3000; ++i) {
    fact(p, n, {i});
  }
  p.add_rule(Rule{Atom{pair, {V(0), V(1)}}, {Atom{n, {V(0)}}, Atom{n, {V(1)}}}, 2, {}});
  BudgetConfig cfg;
  cfg.memory_limit_bytes = size_t{96} << 20;
  cfg.check_interval = 1000000;  // only the reservations and the period
  auto before = current_rss_bytes();
  Budget budget(cfg);
  EXPECT_FALSE(p.solve(&budget));
  EXPECT_EQ(budget.state(), BudgetState::OutOfMemory);
  EXPECT_LT(p.stats().facts, 3000u + 3000u * 3000u);
  auto after = current_rss_bytes();
  ASSERT_TRUE(before && after);
  EXPECT_LT(*after - std::min(*after, *before), size_t{160} << 20);
}

// Naive fixpoint: apply every rule to every combination until nothing changes
std::set<std::pair<uint32_t, Tuple>> naive(
  const std::vector<std::pair<uint32_t, Tuple>> & facts, const std::vector<Rule> & rules)
{
  std::set<std::pair<uint32_t, Tuple>> m(facts.begin(), facts.end());
  for (bool changed = true; changed; ) {
    changed = false;
    for (const auto & r : rules) {
      std::vector<uint32_t> b(r.variables, Program::kUnbound);
      std::function<void(size_t)> go = [&](size_t i) {
          if (i == r.body.size()) {
            for (const auto & c : r.checks) {
              if (!c.test(b.data())) {
                return;
              }
            }
            Tuple h;
            for (const auto & t : r.head.args) {
              h.push_back(t.is_variable ? b[t.value] : t.value);
            }
            changed = m.emplace(r.head.predicate, h).second || changed;
            return;
          }
          const Atom & a = r.body[i];
          for (const auto & [pred, args] : std::vector<std::pair<uint32_t, Tuple>>(m.begin(),
            m.end()))
          {
            if (pred != a.predicate) {
              continue;
            }
            auto saved = b;
            bool ok = true;
            for (size_t k = 0; k < args.size() && ok; ++k) {
              const Term & t = a.args[k];
              if (!t.is_variable) {
                ok = t.value == args[k];
              } else if (b[t.value] == Program::kUnbound) {
                b[t.value] = args[k];
              } else {
                ok = b[t.value] == args[k];
              }
            }
            if (ok) {
              go(i + 1);
            }
            b = saved;
          }
        };
      go(0);
    }
  }
  return m;
}

// Random programs against the naive fixpoint
TEST(DatalogTest, RandomProgramsMatchTheNaiveFixpoint)
{
  std::mt19937 rng(7);
  for (int round = 0; round < 150; ++round) {
    std::vector<uint32_t> arity;
    size_t npred = 2 + rng() % 4;
    for (size_t i = 0; i < npred; ++i) {
      arity.push_back(static_cast<uint32_t>(rng() % 3));
    }
    const uint32_t objects = 4;
    std::vector<std::pair<uint32_t, Tuple>> facts;
    for (int i = 0; i < 6; ++i) {
      uint32_t p = static_cast<uint32_t>(rng() % npred);
      Tuple t;
      for (uint32_t k = 0; k < arity[p]; ++k) {
        t.push_back(static_cast<uint32_t>(rng() % objects));
      }
      facts.emplace_back(p, t);
    }
    std::vector<Rule> rules;
    for (int i = 0; i < 4; ++i) {
      Rule r;
      r.variables = 3;
      size_t nbody = 1 + rng() % 4;
      std::set<uint32_t> used;
      for (size_t j = 0; j < nbody; ++j) {
        Atom a{static_cast<uint32_t>(rng() % npred), {}};
        for (uint32_t k = 0; k < arity[a.predicate]; ++k) {
          if (rng() % 4 == 0) {
            a.args.push_back(C(static_cast<uint32_t>(rng() % objects)));
          } else {
            uint32_t v = static_cast<uint32_t>(rng() % 3);
            used.insert(v);
            a.args.push_back(V(v));
          }
        }
        r.body.push_back(a);
      }
      r.head.predicate = static_cast<uint32_t>(rng() % npred);
      for (uint32_t k = 0; k < arity[r.head.predicate]; ++k) {
        if (used.empty() || rng() % 4 == 0) {
          r.head.args.push_back(C(static_cast<uint32_t>(rng() % objects)));
        } else {
          auto it = used.begin();
          std::advance(it, rng() % used.size());
          r.head.args.push_back(V(*it));
        }
      }
      if (used.size() >= 2 && rng() % 2) {
        uint32_t x = *used.begin(), y = *used.rbegin();
        r.checks.push_back(Check{{x, y}, [x, y](const uint32_t * b) {return b[x] != b[y];}});
      }
      rules.push_back(r);
    }
    Program p;
    for (size_t i = 0; i < npred; ++i) {
      p.add_predicate(arity[i], "p" + std::to_string(i));
    }
    for (const auto & [pred, t] : facts) {
      p.add_fact(pred, t);
    }
    for (const auto & r : rules) {
      p.add_rule(r);
    }
    ASSERT_TRUE(p.solve());
    std::set<std::pair<uint32_t, Tuple>> got;
    for (uint32_t pred = 0; pred < npred; ++pred) {
      for (const auto & t : model(p, pred)) {
        got.emplace(pred, t);
      }
    }
    ASSERT_EQ(got, naive(facts, rules)) << "round " << round;
  }
}

TEST(DatalogTest, StaticSizesGuideTheJoinOrder)
{
  // big(x, y) x big(z, y) shares y but is huge; sel(x) and sel(z) have one fact each. Joining
  // the small static relations first keeps intermediate results tiny.
  auto run = [](bool hint) {
      Program p;
      uint32_t big = p.add_predicate(2, "big");
      uint32_t sel = p.add_predicate(1, "sel");
      uint32_t out = p.add_predicate(2, "out");
      for (uint32_t i = 0; i < 400; ++i) {
        fact(p, big, {i, i % 4});
      }
      fact(p, sel, {7});
      if (hint) {
        p.mark_static(big);
        p.mark_static(sel);
      }
      p.add_rule(
        Rule{Atom{out, {V(0), V(2)}},
          {Atom{big, {V(0), V(1)}}, Atom{big, {V(2), V(1)}}, Atom{sel, {V(0)}},
            Atom{sel, {V(2)}}}, 3, {}});
      EXPECT_TRUE(p.solve());
      EXPECT_EQ(model(p, out), (std::set<Tuple>{{7, 7}}));
      return p.stats().facts;
    };
  EXPECT_LT(run(true), 410u);  // the facts plus a handful of intermediate ones
  EXPECT_LT(run(false), 410u);  // ties go to joins on shared variables
}

TEST(DatalogTest, LongChainIsFast)
{
  // 300 000 facts through a two-condition join: well under a second
  Program p;
  uint32_t e = p.add_predicate(2, "e");
  uint32_t n = p.add_predicate(1, "n");
  uint32_t out = p.add_predicate(2, "out");
  for (uint32_t i = 0; i < 150000; ++i) {
    fact(p, e, {i, i + 1});
    fact(p, n, {i});
  }
  p.add_rule(Rule{Atom{out, {V(0), V(1)}}, {Atom{n, {V(0)}}, Atom{e, {V(0), V(1)}}}, 2, {}});
  auto t0 = std::chrono::steady_clock::now();
  ASSERT_TRUE(p.solve());
  double s = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  EXPECT_EQ(model(p, out).size(), 150000u);
  EXPECT_LT(s, 5.0);  // generous: sanitizer and debug builds
}

}  // namespace
}  // namespace kairos::datalog
