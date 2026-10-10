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

#include <vector>

#include "kairos/diagnostics.hpp"

namespace kairos
{
namespace
{

TEST(DiagnosticsTest, Factories)
{
  auto e = Diagnostic::error("c.e", "an error", SourceLocation{"f", 1, 2});
  auto w = Diagnostic::warning("c.w", "a warning");
  auto n = Diagnostic::note("c.n", "a note");
  EXPECT_EQ(e.severity, Severity::Error);
  EXPECT_EQ(w.severity, Severity::Warning);
  EXPECT_EQ(n.severity, Severity::Note);
  EXPECT_EQ(e.code, "c.e");
  EXPECT_EQ(e.message, "an error");
  ASSERT_TRUE(e.location.has_value());
  EXPECT_EQ(*e.location, (SourceLocation{"f", 1, 2}));
  EXPECT_FALSE(w.location.has_value());
}

TEST(DiagnosticsTest, SeverityNames)
{
  EXPECT_EQ(to_string(Severity::Error), "error");
  EXPECT_EQ(to_string(Severity::Warning), "warning");
  EXPECT_EQ(to_string(Severity::Note), "note");
}

TEST(DiagnosticsTest, FormatWithFullLocation)
{
  auto d = Diagnostic::error(
    "parse.unexpected_token", "unexpected ')'", SourceLocation{"domain.pddl", 12, 5});
  EXPECT_EQ(to_string(d), "domain.pddl:12:5: error: unexpected ')' [parse.unexpected_token]");
}

TEST(DiagnosticsTest, FormatWithPartialLocations)
{
  auto line_only = Diagnostic::warning("w", "msg", SourceLocation{"p.pddl", 3, 0});
  EXPECT_EQ(to_string(line_only), "p.pddl:3: warning: msg [w]");

  auto file_only = Diagnostic::note("n", "msg", SourceLocation{"p.pddl", 0, 0});
  EXPECT_EQ(to_string(file_only), "p.pddl: note: msg [n]");

  auto unnamed = Diagnostic::error("e", "msg", SourceLocation{"", 1, 1});
  EXPECT_EQ(to_string(unnamed), "<input>:1:1: error: msg [e]");
}

TEST(DiagnosticsTest, FormatWithoutLocationOrCode)
{
  EXPECT_EQ(to_string(Diagnostic::error("e", "msg")), "error: msg [e]");
  EXPECT_EQ(to_string(Diagnostic::error("", "msg")), "error: msg");
}

TEST(DiagnosticsTest, FormatWithSourceExcerpt)
{
  const std::string source = "(define (domain d)\n  (:predicates (p))\n  (:action a :bad 1))\n";
  auto d = Diagnostic::error("parse.unknown_key", "unknown key :bad", SourceLocation{"d", 3, 15});
  EXPECT_EQ(to_string(d, source),
    "d:3:15: error: unknown key :bad [parse.unknown_key]\n"
    " 3 |   (:action a :bad 1))\n"
    "   |               ^");
}

TEST(DiagnosticsTest, ExcerptKeepsTabsAligned)
{
  auto d = Diagnostic::error("e", "m", SourceLocation{"f", 1, 3});
  EXPECT_EQ(to_string(d, "\t\tx"), "f:1:3: error: m [e]\n 1 | \t\tx\n   | \t\t^");
}

TEST(DiagnosticsTest, ExcerptWithCrLfCrAndBom)
{
  auto d = Diagnostic::error("e", "m", SourceLocation{"f", 3, 1});
  EXPECT_EQ(to_string(d, "\xEF\xBB\xBF" "a\r\nb\rc\r\nd"), "f:3:1: error: m [e]\n 3 | c\n   | ^");
}

TEST(DiagnosticsTest, ExcerptWithWideLineNumbers)
{
  std::string source;
  for (int i = 1; i <= 120; ++i) {
    source += "line" + std::to_string(i) + "\n";
  }
  auto d = Diagnostic::error("e", "m", SourceLocation{"f", 120, 2});
  EXPECT_EQ(to_string(d, source), "f:120:2: error: m [e]\n 120 | line120\n     |  ^");
}

TEST(DiagnosticsTest, ExcerptFallsBackWhenTheLocationIsUnusable)
{
  auto beyond = Diagnostic::error("e", "m", SourceLocation{"f", 9, 1});
  EXPECT_EQ(to_string(beyond, "one line"), "f:9:1: error: m [e]");
  auto no_location = Diagnostic::error("e", "m");
  EXPECT_EQ(to_string(no_location, "text"), "error: m [e]");
  auto line_only = Diagnostic::error("e", "m", SourceLocation{"f", 1, 0});
  EXPECT_EQ(to_string(line_only, "abc"), "f:1: error: m [e]\n 1 | abc");
  auto column_past_end = Diagnostic::error("e", "m", SourceLocation{"f", 1, 40});
  EXPECT_EQ(to_string(column_past_end, "abc"), "f:1:40: error: m [e]\n 1 | abc");
  auto end_of_line = Diagnostic::error("e", "m", SourceLocation{"f", 1, 4});
  EXPECT_EQ(to_string(end_of_line, "abc"), "f:1:4: error: m [e]\n 1 | abc\n   |    ^");
}

TEST(DiagnosticsTest, HasErrors)
{
  EXPECT_FALSE(has_errors({}));
  EXPECT_FALSE(has_errors({Diagnostic::warning("w", ""), Diagnostic::note("n", "")}));
  EXPECT_TRUE(has_errors({Diagnostic::warning("w", ""), Diagnostic::error("e", "")}));
  EXPECT_TRUE(has_errors({Diagnostic::error("e", "")}));
}

TEST(DiagnosticsTest, Equality)
{
  EXPECT_EQ(Diagnostic::error("a", "b"), Diagnostic::error("a", "b"));
  EXPECT_NE(Diagnostic::error("a", "b"), Diagnostic::warning("a", "b"));
  EXPECT_NE(Diagnostic::error("a", "b", SourceLocation{"f", 1, 1}), Diagnostic::error("a", "b"));
}

}  // namespace
}  // namespace kairos
