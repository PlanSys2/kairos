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

#include <memory>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "core/symbol_table.hpp"

namespace kairos
{
namespace
{

TEST(SymbolTableTest, InternIsIdempotent)
{
  SymbolTable t;
  Symbol a = t.intern("robot");
  Symbol b = t.intern("robot");
  EXPECT_EQ(a, b);
  EXPECT_EQ(t.size(), 1u);
}

TEST(SymbolTableTest, CaseInsensitive)
{
  SymbolTable t;
  Symbol a = t.intern("Move-Robot");
  EXPECT_EQ(t.intern("MOVE-ROBOT"), a);
  EXPECT_EQ(t.intern("move-robot"), a);
  EXPECT_EQ(t.name(a), "move-robot");
  EXPECT_EQ(t.find("mOvE-rObOt"), a);
}

TEST(SymbolTableTest, OnlyAsciiLettersAreLowered)
{
  EXPECT_EQ(SymbolTable::to_lower("AZaz09-_?"), "azaz09-_?");
  // Bytes outside ASCII are kept as they are
  EXPECT_EQ(SymbolTable::to_lower("\xC3\x91"), "\xC3\x91");
  EXPECT_EQ(SymbolTable::to_lower(""), "");
}

TEST(SymbolTableTest, DistinctNamesGetDistinctSymbolsInOrder)
{
  SymbolTable t;
  std::vector<std::string> names = {"a", "b", "?x", "at-robby", "1"};
  for (size_t i = 0; i < names.size(); ++i) {
    EXPECT_EQ(t.intern(names[i]).index(), i);
  }
  for (size_t i = 0; i < names.size(); ++i) {
    EXPECT_EQ(t.name(Symbol(static_cast<uint32_t>(i))), names[i]);
  }
}

TEST(SymbolTableTest, FindDoesNotIntern)
{
  SymbolTable t;
  EXPECT_FALSE(t.find("missing").has_value());
  EXPECT_EQ(t.size(), 0u);
  t.intern("present");
  EXPECT_TRUE(t.find("present").has_value());
  EXPECT_FALSE(t.find("pres").has_value());
}

TEST(SymbolTableTest, EmptyNameIsASymbolLikeAnyOther)
{
  SymbolTable t;
  Symbol e = t.intern("");
  EXPECT_TRUE(e.valid());
  EXPECT_EQ(t.name(e), "");
  EXPECT_EQ(t.find(""), e);
}

TEST(SymbolTableTest, UnknownSymbolThrows)
{
  SymbolTable t;
  t.intern("a");
  EXPECT_THROW(t.name(Symbol(1)), std::out_of_range);
  EXPECT_THROW(t.name(Symbol::invalid()), std::out_of_range);
}

TEST(SymbolTableTest, ViewsStayValidWhileGrowing)
{
  SymbolTable t;
  Symbol first = t.intern("first-symbol-with-a-long-name-to-avoid-sso");
  std::string_view view = t.name(first);
  for (int i = 0; i < 100000; ++i) {
    t.intern("s" + std::to_string(i));
  }
  EXPECT_EQ(view, "first-symbol-with-a-long-name-to-avoid-sso");
  EXPECT_EQ(t.size(), 100001u);
  EXPECT_EQ(t.find("s99999")->index(), 100000u);
}

TEST(SymbolTableTest, CopyIsIndependentAndKeepsIds)
{
  auto original = std::make_unique<SymbolTable>();
  Symbol a = original->intern("alpha-long-name-beyond-small-string");
  Symbol b = original->intern("beta");
  SymbolTable copy(*original);
  original.reset();  // the copy must not point into freed strings
  EXPECT_EQ(copy.name(a), "alpha-long-name-beyond-small-string");
  EXPECT_EQ(copy.find("BETA"), b);
  copy.intern("gamma");
  EXPECT_EQ(copy.size(), 3u);
}

TEST(SymbolTableTest, CopyAssignment)
{
  SymbolTable source;
  source.intern("x");
  SymbolTable target;
  target.intern("old");
  target = source;
  EXPECT_EQ(target.size(), 1u);
  EXPECT_FALSE(target.find("old").has_value());
  EXPECT_EQ(target.find("x")->index(), 0u);
  const SymbolTable & alias = target;
  target = alias;  // self-assignment
  EXPECT_EQ(target.size(), 1u);
}

TEST(SymbolTableTest, MoveKeepsLookupsWorking)
{
  SymbolTable source;
  Symbol a = source.intern("a-name-long-enough-to-live-on-the-heap");
  SymbolTable moved(std::move(source));
  EXPECT_EQ(moved.name(a), "a-name-long-enough-to-live-on-the-heap");
  EXPECT_EQ(moved.find("A-NAME-LONG-ENOUGH-TO-LIVE-ON-THE-HEAP"), a);
  SymbolTable assigned;
  assigned = std::move(moved);
  EXPECT_EQ(assigned.find("a-name-long-enough-to-live-on-the-heap"), a);
}

}  // namespace
}  // namespace kairos
