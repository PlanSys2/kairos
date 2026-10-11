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
#include <random>
#include <tuple>
#include <vector>

#include "temporal/stn.hpp"

namespace kairos::temporal
{
namespace
{

constexpr double kInf = Stn::kInfinity;

TEST(StnTest, OriginAndNewNodes)
{
  Stn stn;
  EXPECT_EQ(stn.nodes(), 1u);
  EXPECT_EQ(stn.earliest(Stn::kOrigin), 0.0);
  EXPECT_EQ(stn.latest(Stn::kOrigin), 0.0);
  auto a = stn.add_node();
  EXPECT_EQ(a, 1u);
  EXPECT_EQ(stn.earliest(a), 0.0);  // never before the origin
  EXPECT_EQ(stn.latest(a), kInf);
}

TEST(StnTest, ADurativeActionAndAnEpsilonAfterIt)
{
  // start, end = start + 5; next starts at least 0.001 after the end
  Stn stn;
  auto s = stn.add_node(), e = stn.add_node(), n = stn.add_node();
  ASSERT_TRUE(stn.exactly(s, e, 5.0));
  ASSERT_TRUE(stn.at_least(e, n, 0.001));
  EXPECT_EQ(stn.earliest(s), 0.0);
  EXPECT_EQ(stn.earliest(e), 5.0);
  EXPECT_NEAR(stn.earliest(n), 5.001, 1e-12);
  EXPECT_EQ(stn.latest(e), kInf);
  // A deadline on n: everything before it gets a latest time
  ASSERT_TRUE(stn.add(Stn::kOrigin, n, 10.0));
  EXPECT_NEAR(stn.latest(e), 9.999, 1e-12);
  EXPECT_NEAR(stn.latest(s), 4.999, 1e-12);
}

TEST(StnTest, InconsistentConstraintsAreRejectedAndChangeNothing)
{
  Stn stn;
  auto a = stn.add_node(), b = stn.add_node();
  ASSERT_TRUE(stn.at_least(a, b, 3.0));      // b >= a + 3
  ASSERT_TRUE(stn.add(Stn::kOrigin, b, 4.0));  // b <= 4
  auto before = std::make_tuple(stn.earliest(a), stn.latest(a), stn.earliest(b), stn.latest(b));
  size_t constraints = stn.constraints();
  EXPECT_FALSE(stn.at_least(b, a, 1.0));     // a >= b + 1: cycle a -> b -> a
  EXPECT_FALSE(stn.at_least(Stn::kOrigin, a, 2.0));  // a >= 2 makes b >= 5 > 4
  EXPECT_FALSE(stn.add(Stn::kOrigin, a, -1.0));  // a <= -1: before the origin
  EXPECT_EQ(
    before, std::make_tuple(stn.earliest(a), stn.latest(a), stn.earliest(b), stn.latest(b)));
  EXPECT_EQ(stn.constraints(), constraints);
  EXPECT_FALSE(stn.exactly(a, b, 1.0));  // the first half fits, the second does not: none stays
  EXPECT_EQ(stn.constraints(), constraints);
  // Still usable
  EXPECT_TRUE(stn.at_least(Stn::kOrigin, a, 1.0));
  EXPECT_EQ(stn.earliest(b), 4.0);
  EXPECT_EQ(stn.latest(b), 4.0);
}

TEST(StnTest, SelfLoopsAndBadInput)
{
  Stn stn;
  auto a = stn.add_node();
  EXPECT_TRUE(stn.add(a, a, 0.0));
  EXPECT_TRUE(stn.add(a, a, 2.0));
  EXPECT_FALSE(stn.add(a, a, -1.0));
  EXPECT_FALSE(stn.add(a, 7, 1.0));     // no such node
  EXPECT_FALSE(stn.add(a, Stn::kOrigin, std::nan("")));
  EXPECT_TRUE(stn.add(a, Stn::kOrigin, kInf));  // no constraint at all
  EXPECT_TRUE(stn.between(Stn::kOrigin, a, 2.0, 3.0));
  EXPECT_EQ(stn.earliest(a), 2.0);
  EXPECT_EQ(stn.latest(a), 3.0);
  EXPECT_FALSE(stn.between(Stn::kOrigin, a, 4.0, 5.0));
  EXPECT_FALSE(stn.between(Stn::kOrigin, a, 3.5, 2.5));  // empty interval
}

TEST(StnTest, UndoGoesBackToTheMark)
{
  Stn stn;
  auto a = stn.add_node();
  ASSERT_TRUE(stn.at_least(Stn::kOrigin, a, 1.0));
  auto m = stn.mark();
  auto b = stn.add_node();
  ASSERT_TRUE(stn.at_least(a, b, 2.0));
  ASSERT_TRUE(stn.at_least(Stn::kOrigin, a, 5.0));
  EXPECT_EQ(stn.earliest(b), 7.0);
  stn.undo(m);
  EXPECT_EQ(stn.nodes(), 2u);
  EXPECT_EQ(stn.earliest(a), 1.0);
  EXPECT_EQ(stn.constraints(), 1u);
  // Nested marks
  auto m1 = stn.mark();
  ASSERT_TRUE(stn.at_least(Stn::kOrigin, a, 3.0));
  auto m2 = stn.mark();
  ASSERT_TRUE(stn.add(Stn::kOrigin, a, 3.0));
  EXPECT_EQ(stn.latest(a), 3.0);
  stn.undo(m2);
  EXPECT_EQ(stn.latest(a), kInf);
  EXPECT_EQ(stn.earliest(a), 3.0);
  stn.undo(m1);
  EXPECT_EQ(stn.earliest(a), 1.0);
  stn.undo(stn.mark());  // nothing to undo
  EXPECT_EQ(stn.earliest(a), 1.0);
}

// --- Against Floyd-Warshall -------------------------------------------------------------------

struct Reference
{
  // Edges i -> j with weight w: t_j - t_i <= w; every node n also has n -> 0 with weight 0
  std::vector<std::tuple<size_t, size_t, double>> edges;
  size_t nodes = 1;

  // Shortest distances, or false on a negative cycle
  bool solve(std::vector<std::vector<double>> & d) const
  {
    d.assign(nodes, std::vector<double>(nodes, kInf));
    for (size_t n = 0; n < nodes; ++n) {
      d[n][n] = 0.0;
      d[n][0] = std::min(d[n][0], 0.0);
    }
    for (const auto & [i, j, w] : edges) {
      d[i][j] = std::min(d[i][j], w);
    }
    for (size_t k = 0; k < nodes; ++k) {
      for (size_t i = 0; i < nodes; ++i) {
        for (size_t j = 0; j < nodes; ++j) {
          if (d[i][k] + d[k][j] < d[i][j]) {
            d[i][j] = d[i][k] + d[k][j];
          }
        }
      }
    }
    for (size_t n = 0; n < nodes; ++n) {
      if (d[n][n] < -1e-7) {
        return false;
      }
    }
    return true;
  }
};

::testing::AssertionResult agrees(const Stn & stn, const Reference & ref)
{
  std::vector<std::vector<double>> d;
  if (!ref.solve(d)) {
    return ::testing::AssertionFailure() << "the reference is inconsistent";
  }
  if (stn.nodes() != ref.nodes) {
    return ::testing::AssertionFailure() << "nodes " << stn.nodes() << " vs " << ref.nodes;
  }
  for (size_t n = 0; n < ref.nodes; ++n) {
    double earliest = -d[n][0];
    double latest = d[0][n];
    auto node = static_cast<Stn::Node>(n);
    if (std::abs(stn.earliest(node) - earliest) > 1e-6 ||
      (std::isinf(latest) != std::isinf(stn.latest(node))) ||
      (!std::isinf(latest) && std::abs(stn.latest(node) - latest) > 1e-6))
    {
      return ::testing::AssertionFailure() << "node " << n << ": [" << stn.earliest(node) <<
             ", " << stn.latest(node) << "] vs [" << earliest << ", " << latest << "]";
    }
  }
  return ::testing::AssertionSuccess();
}

TEST(StnPropertiesTest, AgreesWithFloydWarshallUnderRandomSequencesAndUndo)
{
  std::mt19937 rng(20261011);
  size_t rejected = 0, accepted = 0, undone = 0;
  for (int round = 0; round < 3000; ++round) {
    Stn stn;
    Reference ref;
    struct Saved
    {
      Stn::Mark mark;
      Reference ref;
    };
    std::vector<Saved> marks;
    int steps = std::uniform_int_distribution<int>(1, 40)(rng);
    for (int s = 0; s < steps; ++s) {
      int what = std::uniform_int_distribution<int>(0, 9)(rng);
      if (what == 0 || ref.nodes < 3) {
        stn.add_node();
        ++ref.nodes;
      } else if (what == 1) {
        marks.push_back(Saved{stn.mark(), ref});
      } else if (what == 2 && !marks.empty()) {
        size_t k = std::uniform_int_distribution<size_t>(0, marks.size() - 1)(rng);
        stn.undo(marks[k].mark);
        ref = marks[k].ref;
        marks.resize(k);
        ++undone;
      } else {
        size_t n = ref.nodes;
        auto i = std::uniform_int_distribution<size_t>(0, n - 1)(rng);
        auto j = std::uniform_int_distribution<size_t>(0, n - 1)(rng);
        // Integers, and some fractions as in plans with epsilon separations
        double w = std::uniform_int_distribution<int>(-6, 12)(rng);
        if (std::uniform_int_distribution<int>(0, 3)(rng) == 0) {
          w += 0.001 * std::uniform_int_distribution<int>(-3, 3)(rng);
        }
        Reference tried = ref;
        tried.edges.emplace_back(i, j, w);
        std::vector<std::vector<double>> d;
        bool should = tried.solve(d);
        bool did = stn.add(static_cast<Stn::Node>(i), static_cast<Stn::Node>(j), w);
        ASSERT_EQ(did, should) << "round " << round << " step " << s << ": " << i << " -> " <<
          j << " w " << w;
        if (did) {
          ref = tried;
          ++accepted;
        } else {
          ++rejected;
        }
      }
      ASSERT_TRUE(agrees(stn, ref)) << "round " << round << " step " << s;
    }
  }
  // The sequences tried every case many times
  EXPECT_GT(accepted, 10000u);
  EXPECT_GT(rejected, 1000u);
  EXPECT_GT(undone, 1000u);
}

TEST(StnTest, ALongChainIsIncremental)
{
  // 20 000 actions in sequence: each constraint touches the new node only, so adding them all
  // is linear (a full recomputation each time would be quadratic)
  Stn stn;
  auto prev = Stn::kOrigin;
  auto t0 = std::chrono::steady_clock::now();
  for (int k = 0; k < 20000; ++k) {
    auto s = stn.add_node();
    auto e = stn.add_node();
    ASSERT_TRUE(stn.exactly(s, e, 1.0));
    ASSERT_TRUE(stn.at_least(prev, s, 0.001));
    prev = e;
  }
  double ms = std::chrono::duration<double,
      std::milli>(std::chrono::steady_clock::now() - t0).count();
  EXPECT_NEAR(stn.earliest(prev), 20000 * 1.001 - 0.001 + 0.001, 1e-6);
  EXPECT_LT(ms, 2000.0);  // generous: debug, sanitizer and coverage builds
}

}  // namespace
}  // namespace kairos::temporal
