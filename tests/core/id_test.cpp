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

#include <cstdint>
#include <set>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <unordered_set>
#include <vector>

#include "core/id.hpp"

namespace kairos
{
namespace
{

using AtomId = Id<struct AtomTag>;
using ActionId = Id<struct ActionTag>;
using SmallId = Id<struct SmallTag, uint8_t>;

// Different tags must not convert into each other, nor from raw integers implicitly
static_assert(!std::is_convertible_v<AtomId, ActionId>);
static_assert(!std::is_convertible_v<ActionId, AtomId>);
static_assert(!std::is_convertible_v<uint32_t, AtomId>);
static_assert(std::is_trivially_copyable_v<AtomId>);
static_assert(sizeof(AtomId) == sizeof(uint32_t));

TEST(IdTest, DefaultIsInvalid)
{
  AtomId id;
  EXPECT_FALSE(id.valid());
  EXPECT_EQ(id, AtomId::invalid());
  EXPECT_EQ(id.value(), AtomId::invalid_value);
}

TEST(IdTest, ExplicitValuesAreValid)
{
  EXPECT_TRUE(AtomId(0).valid());
  EXPECT_TRUE(AtomId(AtomId::invalid_value - 1).valid());
  EXPECT_EQ(AtomId(42).value(), 42u);
  EXPECT_EQ(AtomId(42).index(), size_t{42});
}

TEST(IdTest, MaxValueIsTheInvalidSentinel)
{
  EXPECT_FALSE(AtomId(AtomId::invalid_value).valid());
  EXPECT_FALSE(SmallId(255).valid());
  EXPECT_TRUE(SmallId(254).valid());
}

TEST(IdTest, Ordering)
{
  EXPECT_LT(AtomId(1), AtomId(2));
  EXPECT_GT(AtomId(3), AtomId(2));
  EXPECT_EQ(AtomId(7), AtomId(7));
  EXPECT_NE(AtomId(7), AtomId(8));
  // The invalid id sorts last
  EXPECT_LT(AtomId(0), AtomId::invalid());
  std::set<AtomId> sorted{AtomId(5), AtomId(1), AtomId(3)};
  EXPECT_EQ(sorted.begin()->value(), 1u);
}

TEST(IdTest, Hashing)
{
  std::unordered_set<AtomId> ids;
  for (uint32_t i = 0; i < 1000; ++i) {
    ids.insert(AtomId(i));
  }
  ids.insert(AtomId(10));  // duplicate
  EXPECT_EQ(ids.size(), 1000u);
  EXPECT_TRUE(ids.contains(AtomId(999)));
  EXPECT_FALSE(ids.contains(AtomId(1000)));
}

TEST(IdVectorTest, PushBackReturnsConsecutiveIds)
{
  IdVector<ActionId, std::string> v;
  EXPECT_TRUE(v.empty());
  ActionId a = v.push_back("a");
  ActionId b = v.push_back("b");
  ActionId c = v.emplace_back(size_t{3}, 'c');
  EXPECT_EQ(a.value(), 0u);
  EXPECT_EQ(b.value(), 1u);
  EXPECT_EQ(c.value(), 2u);
  EXPECT_EQ(v.size(), 3u);
  EXPECT_EQ(v[a], "a");
  EXPECT_EQ(v[c], "ccc");
}

TEST(IdVectorTest, ElementsAreMutable)
{
  IdVector<ActionId, int> v;
  ActionId id = v.push_back(1);
  v[id] = 5;
  v.at(id) += 1;
  EXPECT_EQ(v[id], 6);
}

TEST(IdVectorTest, CheckedAccessRejectsBadIds)
{
  IdVector<ActionId, int> v(2, 7);
  EXPECT_EQ(v.at(ActionId(1)), 7);
  EXPECT_THROW(v.at(ActionId(2)), std::out_of_range);
  EXPECT_THROW(v.at(ActionId::invalid()), std::out_of_range);
  const auto & cv = v;
  EXPECT_THROW(cv.at(ActionId(99)), std::out_of_range);
}

TEST(IdVectorTest, Contains)
{
  IdVector<ActionId, int> v;
  EXPECT_FALSE(v.contains(ActionId(0)));
  v.push_back(1);
  EXPECT_TRUE(v.contains(ActionId(0)));
  EXPECT_FALSE(v.contains(ActionId(1)));
  EXPECT_FALSE(v.contains(ActionId::invalid()));
}

TEST(IdVectorTest, IterationAndIds)
{
  IdVector<ActionId, int> v;
  for (int i = 0; i < 5; ++i) {
    v.push_back(i * 10);
  }
  int sum = 0;
  for (int x : v) {
    sum += x;
  }
  EXPECT_EQ(sum, 100);
  auto ids = v.ids();
  ASSERT_EQ(ids.size(), 5u);
  for (size_t i = 0; i < ids.size(); ++i) {
    EXPECT_EQ(ids[i].index(), i);
    EXPECT_EQ(v[ids[i]], static_cast<int>(i) * 10);
  }
  v.clear();
  EXPECT_TRUE(v.empty());
  EXPECT_TRUE(v.ids().empty());
}

TEST(IdVectorTest, IdSpaceExhaustionThrows)
{
  // uint8_t ids: 255 is the invalid sentinel, so 255 elements fit
  IdVector<SmallId, char> v;
  for (int i = 0; i < 255; ++i) {
    EXPECT_TRUE(v.push_back('x').valid());
  }
  EXPECT_THROW(v.push_back('x'), std::length_error);
  EXPECT_EQ(v.size(), 255u);
}

}  // namespace
}  // namespace kairos
