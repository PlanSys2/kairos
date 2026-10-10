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

#include <string>
#include <vector>

#include "core/suggest.hpp"

namespace kairos
{
namespace
{

TEST(SuggestTest, EditDistance)
{
  EXPECT_EQ(edit_distance("", ""), 0u);
  EXPECT_EQ(edit_distance("abc", ""), 3u);
  EXPECT_EQ(edit_distance("", "abc"), 3u);
  EXPECT_EQ(edit_distance("robot", "robot"), 0u);
  EXPECT_EQ(edit_distance("robot", "robto"), 1u);    // transposition counts as one
  EXPECT_EQ(edit_distance("robot-at", "robot_at"), 1u);
  EXPECT_EQ(edit_distance("precondtion", "precondition"), 1u);
  EXPECT_EQ(edit_distance("kitten", "sitting"), 3u);
  EXPECT_EQ(edit_distance("a", "b"), 1u);
}

TEST(SuggestTest, ClosestMatch)
{
  std::vector<std::string> names = {"robot_at", "person_at", "connected", "robot"};
  EXPECT_EQ(closest_match("robot-at", names), "robot_at");
  EXPECT_EQ(closest_match("conected", names), "connected");
  EXPECT_EQ(closest_match("robto", names), "robot");
}

TEST(SuggestTest, NoSuggestionWhenNothingIsClose)
{
  std::vector<std::string> names = {"robot_at", "connected"};
  EXPECT_FALSE(closest_match("hall", names).has_value());
  EXPECT_FALSE(closest_match("xyz", names).has_value());
  EXPECT_FALSE(closest_match("anything", {}).has_value());
}

TEST(SuggestTest, ShortNamesNeedCloserMatches)
{
  // One edit on a two-letter name is half the name: not a convincing typo
  EXPECT_FALSE(closest_match("ab", {"ac"}).has_value());
  EXPECT_FALSE(closest_match("p", {"q"}).has_value());
  EXPECT_EQ(closest_match("abc", {"abd"}), "abd");
}

TEST(SuggestTest, ExactMatchIsNotSuggested)
{
  EXPECT_FALSE(closest_match("robot", {"robot"}).has_value());
  EXPECT_EQ(closest_match("robot", {"robot", "robots"}), "robots");
}

TEST(SuggestTest, TiesKeepTheFirstCandidate)
{
  EXPECT_EQ(closest_match("cat", {"bat", "hat"}), "bat");
}

TEST(SuggestTest, DidYouMeanText)
{
  EXPECT_EQ(did_you_mean("robto", {"robot"}), " (did you mean 'robot'?)");
  EXPECT_EQ(did_you_mean("zzz", {"robot"}), "");
}

TEST(SuggestTest, ManyCandidates)
{
  std::vector<std::string> names;
  for (int i = 0; i < 20000; ++i) {
    names.push_back("object" + std::to_string(i));
  }
  EXPECT_EQ(closest_match("object1234x", names), "object1234");
  EXPECT_FALSE(closest_match("completely-different", names).has_value());
}

}  // namespace
}  // namespace kairos
