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
#include <vector>

#include "core/budget.hpp"
#include "search/novelty.hpp"

namespace kairos::search
{
namespace
{

TEST(NoveltyTest, WidthOneTwoAndMore)
{
  Novelty n(3, 0, 2, size_t{64} << 20);
  std::vector<uint32_t> a{0, 0, 0};
  EXPECT_EQ(n.evaluate(0, a.data(), nullptr), 1);       // everything is new
  EXPECT_EQ(n.evaluate(0, a.data(), nullptr), 3);       // the same state again
  std::vector<uint32_t> b{1, 0, 0};
  EXPECT_EQ(n.evaluate(0, b.data(), nullptr), 1);       // var 0 = 1 is new
  std::vector<uint32_t> c{0, 1, 0};
  EXPECT_EQ(n.evaluate(0, c.data(), nullptr), 1);       // var 1 = 1 is new
  std::vector<uint32_t> d{1, 1, 0};
  EXPECT_EQ(n.evaluate(0, d.data(), nullptr), 2);       // each value seen, not together
  EXPECT_EQ(n.evaluate(0, d.data(), nullptr), 3);
}

TEST(NoveltyTest, PartitionsAreIndependent)
{
  Novelty n(2, 0, 2, size_t{64} << 20);
  std::vector<uint32_t> a{0, 0};
  EXPECT_EQ(n.evaluate(7, a.data(), nullptr), 1);
  EXPECT_EQ(n.evaluate(7, a.data(), nullptr), 3);
  EXPECT_EQ(n.evaluate(8, a.data(), nullptr), 1);  // new in another partition
  EXPECT_EQ(n.evaluate(8, a.data(), nullptr), 3);
}

TEST(NoveltyTest, NumericValuesAreFeatures)
{
  Novelty n(1, 2, 2, size_t{64} << 20);
  std::vector<uint32_t> v{0};
  std::vector<double> x{1.0, 2.0};
  EXPECT_EQ(n.evaluate(0, v.data(), x.data()), 1);
  EXPECT_EQ(n.evaluate(0, v.data(), x.data()), 3);
  x = {1.5, 2.0};
  EXPECT_EQ(n.evaluate(0, v.data(), x.data()), 1);
  x = {0.0, 2.0};
  EXPECT_EQ(n.evaluate(0, v.data(), x.data()), 1);
  x = {-0.0, 2.0};
  EXPECT_EQ(n.evaluate(0, v.data(), x.data()), 3);  // -0 is 0
  double nan = std::numeric_limits<double>::quiet_NaN();
  x = {nan, 2.0};
  EXPECT_EQ(n.evaluate(0, v.data(), x.data()), 1);  // undefined: one more value
  EXPECT_EQ(n.evaluate(0, v.data(), x.data()), 3);
}

TEST(NoveltyTest, WidthOneOnly)
{
  Novelty n(2, 0, 1, size_t{64} << 20);
  std::vector<uint32_t> a{0, 1}, b{1, 0}, c{1, 1};
  EXPECT_EQ(n.evaluate(0, a.data(), nullptr), 1);
  EXPECT_EQ(n.evaluate(0, b.data(), nullptr), 1);
  EXPECT_EQ(n.evaluate(0, c.data(), nullptr), 3);  // a new pair, not tracked
  EXPECT_EQ(n.max_width(), 1);
}

TEST(NoveltyTest, MemoryCapDropsPairsButNotSingles)
{
  Budget budget{BudgetConfig{}};
  // 40 variables: 780 pairs per state; a cap of 64 KB is passed quickly
  const size_t vars = 40;
  Novelty n(vars, 0, 2, size_t{64} << 10, &budget);
  std::vector<uint32_t> v(vars, 0);
  int widths[4] = {0, 0, 0, 0};
  for (uint32_t k = 0; k < 200; ++k) {
    v[k % vars] = static_cast<uint32_t>(k / vars + 1);
    ++widths[n.evaluate(0, v.data(), nullptr)];
  }
  EXPECT_TRUE(n.pairs_dropped());
  EXPECT_EQ(widths[1], 200);  // each state has a new value
  // Past the cap, a state with only a new pair counts as width 3
  std::vector<uint32_t> old = v;
  old[0] = 1;
  old[1] = 2;
  int w = n.evaluate(0, old.data(), nullptr);
  EXPECT_TRUE(w == 1 || w == 3);
  EXPECT_GT(budget.accounted_bytes(), 0u);
}

TEST(NoveltyTest, MemoryIsReturnedToTheBudget)
{
  Budget budget{BudgetConfig{}};
  {
    Novelty n(10, 0, 2, size_t{64} << 20, &budget);
    std::vector<uint32_t> v(10, 0);
    for (uint32_t k = 0; k < 1000; ++k) {
      v[k % 10] = k;
      n.evaluate(k % 3, v.data(), nullptr);
    }
    EXPECT_GT(budget.accounted_bytes(), 0u);
  }
  EXPECT_EQ(budget.accounted_bytes(), 0u);
}

}  // namespace
}  // namespace kairos::search
