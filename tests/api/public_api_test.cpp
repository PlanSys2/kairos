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

#include "kairos/kairos.hpp"

namespace kairos
{
namespace
{

// These tests link only against libkairos: they fail to link if a public function is missing
// from the library (e.g. dropped by the linker or not exported).

TEST(PublicApiTest, VersionMatchesHeaders)
{
  EXPECT_EQ(version(), KAIROS_VERSION_STRING);
  std::string expected = std::to_string(KAIROS_VERSION_MAJOR) + "." +
    std::to_string(KAIROS_VERSION_MINOR) + "." + std::to_string(KAIROS_VERSION_PATCH);
  EXPECT_EQ(version(), expected);
}

TEST(PublicApiTest, DiagnosticsFunctionsAreExported)
{
  auto d = Diagnostic::error("e", "msg", SourceLocation{"f", 1, 2});
  EXPECT_EQ(to_string(d), "f:1:2: error: msg [e]");
  EXPECT_TRUE(has_errors({d}));
  EXPECT_EQ(to_string(Severity::Warning), "warning");
}

TEST(PublicApiTest, LogFunctionsAreExported)
{
  EXPECT_EQ(to_string(LogLevel::Info), "info");
  EXPECT_EQ(to_string(LogChannel::Search), "search");
  EXPECT_EQ(parse_log_channel("ground"), LogChannel::Ground);
  EXPECT_EQ(LogChannels::parse("all"), LogChannels::all());
}

TEST(PublicApiTest, ResultIsUsable)
{
  auto r = Result<int>::success(3);
  EXPECT_EQ(*r, 3);
}

}  // namespace
}  // namespace kairos
