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
#include <string>
#include <utility>
#include <vector>

#include "kairos/result.hpp"

namespace kairos
{
namespace
{

TEST(ResultTest, SuccessHoldsValue)
{
  auto r = Result<int>::success(42);
  EXPECT_TRUE(r.ok());
  EXPECT_TRUE(static_cast<bool>(r));
  EXPECT_EQ(r.value(), 42);
  EXPECT_EQ(*r, 42);
  EXPECT_TRUE(r.diagnostics().empty());
}

TEST(ResultTest, SuccessCanCarryWarnings)
{
  auto r = Result<int>::success(
    1, {Diagnostic::warning("semantic.undeclared_requirement", "uses :fluents")});
  EXPECT_TRUE(r.ok());
  ASSERT_EQ(r.diagnostics().size(), 1u);
  EXPECT_EQ(r.diagnostics()[0].severity, Severity::Warning);
  EXPECT_FALSE(has_errors(r.diagnostics()));
}

TEST(ResultTest, FailureHoldsErrors)
{
  auto r = Result<int>::failure(
    Diagnostic::error("parse.unexpected_token", "bad", SourceLocation{"d.pddl", 3, 7}));
  EXPECT_FALSE(r.ok());
  EXPECT_FALSE(static_cast<bool>(r));
  ASSERT_EQ(r.diagnostics().size(), 1u);
  EXPECT_EQ(r.diagnostics()[0].code, "parse.unexpected_token");
  EXPECT_EQ(r.diagnostics()[0].location->line, 3u);
}

TEST(ResultTest, FailureWithoutErrorsGetsMissingError)
{
  auto empty = Result<int>::failure(std::vector<Diagnostic>{});
  EXPECT_FALSE(empty.ok());
  ASSERT_EQ(empty.diagnostics().size(), 1u);
  EXPECT_EQ(empty.diagnostics()[0].code, "core.missing_error");

  auto only_warning = Result<int>::failure(Diagnostic::warning("w", "just a warning"));
  ASSERT_EQ(only_warning.diagnostics().size(), 2u);
  EXPECT_EQ(only_warning.diagnostics()[0].code, "w");
  EXPECT_EQ(only_warning.diagnostics()[1].code, "core.missing_error");
  EXPECT_TRUE(has_errors(only_warning.diagnostics()));
}

TEST(ResultTest, FailureKeepsAllDiagnosticsInOrder)
{
  std::vector<Diagnostic> diags = {
    Diagnostic::error("e1", "first"),
    Diagnostic::note("n1", "context"),
    Diagnostic::error("e2", "second"),
  };
  auto r = Result<int>::failure(diags);
  ASSERT_EQ(r.diagnostics().size(), 3u);
  EXPECT_EQ(r.diagnostics()[0].code, "e1");
  EXPECT_EQ(r.diagnostics()[1].code, "n1");
  EXPECT_EQ(r.diagnostics()[2].code, "e2");
}

TEST(ResultTest, ValueOnFailureThrows)
{
  auto r = Result<std::string>::failure(Diagnostic::error("e", "x"));
  EXPECT_THROW(r.value(), BadResultAccess);
  EXPECT_THROW(*r, BadResultAccess);
  EXPECT_THROW(static_cast<void>(r->size()), BadResultAccess);
  const auto & cr = r;
  EXPECT_THROW(cr.value(), BadResultAccess);
  EXPECT_THROW(std::move(r).value(), BadResultAccess);
}

TEST(ResultTest, MoveOnlyValues)
{
  auto r = Result<std::unique_ptr<int>>::success(std::make_unique<int>(5));
  ASSERT_TRUE(r.ok());
  EXPECT_EQ(**r, 5);
  std::unique_ptr<int> taken = std::move(r).value();
  ASSERT_NE(taken, nullptr);
  EXPECT_EQ(*taken, 5);
}

TEST(ResultTest, ArrowAndMutation)
{
  auto r = Result<std::string>::success("abc");
  EXPECT_EQ(r->size(), 3u);
  r.value() += "d";
  EXPECT_EQ(*r, "abcd");
}

TEST(ResultTest, TakeDiagnosticsMovesThemOut)
{
  auto r = Result<int>::failure(Diagnostic::error("e", "x"));
  auto diags = r.take_diagnostics();
  ASSERT_EQ(diags.size(), 1u);
  EXPECT_EQ(diags[0].code, "e");
  // Forwarding into another result type
  auto forwarded = Result<std::string>::failure(std::move(diags));
  EXPECT_FALSE(forwarded.ok());
  EXPECT_EQ(forwarded.diagnostics()[0].code, "e");
}

TEST(ResultTest, CopyAndMoveResults)
{
  auto r = Result<std::vector<int>>::success({1, 2, 3}, {Diagnostic::note("n", "note")});
  auto copy = r;
  auto moved = std::move(r);
  EXPECT_EQ(copy->size(), 3u);
  EXPECT_EQ(moved->size(), 3u);
  EXPECT_EQ(copy.diagnostics().size(), 1u);
}

}  // namespace
}  // namespace kairos
