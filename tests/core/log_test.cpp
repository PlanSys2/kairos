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
#include <string>

#include "kairos/log.hpp"

namespace kairos
{
namespace
{

TEST(LogTest, LevelNames)
{
  EXPECT_EQ(to_string(LogLevel::Debug), "debug");
  EXPECT_EQ(to_string(LogLevel::Info), "info");
  EXPECT_EQ(to_string(LogLevel::Warning), "warning");
  EXPECT_EQ(to_string(LogLevel::Error), "error");
}

TEST(LogTest, EveryChannelRoundTripsThroughItsName)
{
  std::set<std::string> names;
  for (uint8_t i = 0; i < static_cast<uint8_t>(LogChannel::Count); ++i) {
    auto channel = static_cast<LogChannel>(i);
    std::string name(to_string(channel));
    EXPECT_FALSE(name.empty());
    EXPECT_NE(name, "count");
    EXPECT_TRUE(names.insert(name).second) << "duplicated name " << name;
    EXPECT_EQ(parse_log_channel(name), channel);
  }
  EXPECT_EQ(to_string(LogChannel::Count), "count");
}

TEST(LogTest, UnknownChannelNames)
{
  EXPECT_FALSE(parse_log_channel("").has_value());
  EXPECT_FALSE(parse_log_channel("SEARCH").has_value());  // names are lower case
  EXPECT_FALSE(parse_log_channel("count").has_value());
  EXPECT_FALSE(parse_log_channel("searchx").has_value());
}

TEST(LogChannelsTest, AddAndContains)
{
  LogChannels c;
  EXPECT_TRUE(c.empty());
  c.add(LogChannel::Search).add(LogChannel::Ground);
  EXPECT_TRUE(c.contains(LogChannel::Search));
  EXPECT_TRUE(c.contains(LogChannel::Ground));
  EXPECT_FALSE(c.contains(LogChannel::Parse));
  EXPECT_FALSE(c.empty());
}

TEST(LogChannelsTest, AllAndNone)
{
  for (uint8_t i = 0; i < static_cast<uint8_t>(LogChannel::Count); ++i) {
    EXPECT_TRUE(LogChannels::all().contains(static_cast<LogChannel>(i)));
    EXPECT_FALSE(LogChannels::none().contains(static_cast<LogChannel>(i)));
  }
  EXPECT_FALSE(LogChannels::all().contains(LogChannel::Count));
  EXPECT_TRUE(LogChannels::none().empty());
}

TEST(LogChannelsTest, ParseAll)
{
  EXPECT_EQ(LogChannels::parse("all"), LogChannels::all());
  EXPECT_EQ(LogChannels::parse("  all "), LogChannels::all());
}

TEST(LogChannelsTest, ParseList)
{
  auto c = LogChannels::parse("search,ground");
  ASSERT_TRUE(c.has_value());
  EXPECT_TRUE(c->contains(LogChannel::Search));
  EXPECT_TRUE(c->contains(LogChannel::Ground));
  EXPECT_FALSE(c->contains(LogChannel::Sas));

  auto spaced = LogChannels::parse(" search , memory ");
  ASSERT_TRUE(spaced.has_value());
  EXPECT_TRUE(spaced->contains(LogChannel::Memory));

  auto repeated = LogChannels::parse("sas,sas");
  ASSERT_TRUE(repeated.has_value());
  EXPECT_EQ(*repeated, LogChannels().add(LogChannel::Sas));
}

TEST(LogChannelsTest, ParseErrors)
{
  EXPECT_FALSE(LogChannels::parse("").has_value());
  EXPECT_FALSE(LogChannels::parse("   ").has_value());
  EXPECT_FALSE(LogChannels::parse("search,").has_value());
  EXPECT_FALSE(LogChannels::parse(",search").has_value());
  EXPECT_FALSE(LogChannels::parse("search,bogus").has_value());
  EXPECT_FALSE(LogChannels::parse("all,search").has_value());
}

}  // namespace
}  // namespace kairos
