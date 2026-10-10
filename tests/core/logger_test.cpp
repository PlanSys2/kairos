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

#include <format>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include "core/logger.hpp"

namespace
{

// Counts how many times it is formatted, to check that filtered messages cost nothing
struct Counted
{
  int * count;
};

}  // namespace

template<>
struct std::formatter<Counted>: std::formatter<std::string_view>
{
  auto format(const Counted & c, std::format_context & ctx) const
  {
    ++*c.count;
    return std::formatter<std::string_view>::format("counted", ctx);
  }
};

namespace kairos
{
namespace
{

struct Record
{
  LogLevel level;
  LogChannel channel;
  std::string message;
};

LogOptions capture(std::vector<Record> & out, LogLevel level, LogChannels debug)
{
  LogOptions options;
  options.level = level;
  options.debug_channels = debug;
  options.callback = [&out](LogLevel l, LogChannel c, std::string_view m) {
      out.push_back({l, c, std::string(m)});
    };
  return options;
}

TEST(LoggerTest, DefaultLoggerDiscardsEverything)
{
  Logger logger;
  EXPECT_FALSE(logger.enabled(LogLevel::Error, LogChannel::General));
  int formatted = 0;
  logger.error(LogChannel::General, "{}", Counted{&formatted});
  EXPECT_EQ(formatted, 0);
}

TEST(LoggerTest, DeliversFormattedMessagesWithLevelAndChannel)
{
  std::vector<Record> out;
  Logger logger(capture(out, LogLevel::Info, LogChannels::all()));
  logger.info(LogChannel::Ground, "{} actions", 42);
  logger.warning(LogChannel::Memory, "pressure at {}%", 80);
  logger.error(LogChannel::Parse, "bad");
  ASSERT_EQ(out.size(), 3u);
  EXPECT_EQ(out[0].level, LogLevel::Info);
  EXPECT_EQ(out[0].channel, LogChannel::Ground);
  EXPECT_EQ(out[0].message, "42 actions");
  EXPECT_EQ(out[1].message, "pressure at 80%");
  EXPECT_EQ(out[2].level, LogLevel::Error);
}

TEST(LoggerTest, LevelFilter)
{
  std::vector<Record> out;
  Logger logger(capture(out, LogLevel::Warning, LogChannels::all()));
  logger.debug(LogChannel::Search, "d");
  logger.info(LogChannel::Search, "i");
  logger.warning(LogChannel::Search, "w");
  logger.error(LogChannel::Search, "e");
  ASSERT_EQ(out.size(), 2u);
  EXPECT_EQ(out[0].message, "w");
  EXPECT_EQ(out[1].message, "e");
}

TEST(LoggerTest, DebugOnlyForSelectedChannels)
{
  std::vector<Record> out;
  Logger logger(capture(out, LogLevel::Debug, LogChannels().add(LogChannel::Search)));
  logger.debug(LogChannel::Search, "search debug");
  logger.debug(LogChannel::Ground, "ground debug");
  // Channel selection only applies to Debug: other levels always pass
  logger.info(LogChannel::Ground, "ground info");
  ASSERT_EQ(out.size(), 2u);
  EXPECT_EQ(out[0].message, "search debug");
  EXPECT_EQ(out[1].message, "ground info");
  EXPECT_TRUE(logger.enabled(LogLevel::Debug, LogChannel::Search));
  EXPECT_FALSE(logger.enabled(LogLevel::Debug, LogChannel::Ground));
}

TEST(LoggerTest, DebugWithNoChannelsShowsNoDebug)
{
  std::vector<Record> out;
  Logger logger(capture(out, LogLevel::Debug, LogChannels::none()));
  logger.debug(LogChannel::Search, "x");
  logger.info(LogChannel::Search, "y");
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].message, "y");
}

TEST(LoggerTest, FilteredMessagesAreNotFormatted)
{
  std::vector<Record> out;
  Logger logger(capture(out, LogLevel::Info, LogChannels::all()));
  int formatted = 0;
  logger.debug(LogChannel::Search, "{}", Counted{&formatted});
  EXPECT_EQ(formatted, 0);
  logger.info(LogChannel::Search, "{}", Counted{&formatted});
  EXPECT_EQ(formatted, 1);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].message, "counted");
}

TEST(LoggerTest, ThrowingCallbackIsSwallowed)
{
  LogOptions options;
  int calls = 0;
  options.callback = [&calls](LogLevel, LogChannel, std::string_view) {
      ++calls;
      throw std::runtime_error("user callback failed");
    };
  Logger logger(options);
  EXPECT_NO_THROW(logger.info(LogChannel::General, "one"));
  EXPECT_NO_THROW(logger.error(LogChannel::General, "two"));
  EXPECT_EQ(calls, 2);
}

TEST(LoggerTest, CopiesShareTheCallback)
{
  std::vector<Record> out;
  Logger logger(capture(out, LogLevel::Info, LogChannels::all()));
  Logger copy = logger;
  copy.info(LogChannel::General, "from copy");
  logger.info(LogChannel::General, "from original");
  EXPECT_EQ(out.size(), 2u);
  EXPECT_EQ(copy.options().level, LogLevel::Info);
}

}  // namespace
}  // namespace kairos
