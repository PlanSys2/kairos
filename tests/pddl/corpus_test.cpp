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

#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <map>
#include <string>
#include <vector>

#include "pddl/parser.hpp"
#include "pddl/printer.hpp"
#include "support/pddl_helpers.hpp"

namespace kairos::pddl
{
namespace
{

namespace fs = std::filesystem;
using kairos::testing::codes;
using kairos::testing::dump;
using kairos::testing::read_file;

struct Outcome
{
  bool ok = false;
  std::vector<std::string> diagnostics;
  bool round_trip = true;
  std::string round_trip_error;
};

template<class Parse>
Outcome check(const std::string & text, const std::string & file, Parse parse)
{
  Outcome out;
  auto r = parse(text, file);
  out.ok = r.ok();
  out.diagnostics = codes(r.diagnostics());
  if (r.ok()) {
    std::string printed = to_pddl(*r);
    auto again = parse(printed, file);
    if (!again.ok()) {
      out.round_trip = false;
      out.round_trip_error = dump(again.diagnostics());
    } else if (to_pddl(*again) != printed) {
      out.round_trip = false;
      out.round_trip_error = "printed text changes on a second round trip";
    }
  }
  return out;
}

Outcome check_file(const fs::path & path)
{
  std::string text = read_file(path);
  std::string file = path.filename().string();
  switch (detect_file_kind(text)) {
    case FileKind::Domain:
      return check(text, file, [](auto & t, auto & f) {return parse_domain(t, f);});
    case FileKind::Problem:
      return check(text, file, [](auto & t, auto & f) {return parse_problem(t, f);});
    case FileKind::Unknown:
      break;
  }
  Outcome unknown;
  unknown.diagnostics = {"unknown_file_kind"};
  return unknown;
}

std::vector<fs::path> pddl_files(const fs::path & dir)
{
  std::vector<fs::path> files;
  for (const auto & entry : fs::recursive_directory_iterator(dir)) {
    if (entry.is_regular_file() && entry.path().extension() == ".pddl") {
      files.push_back(entry.path());
    }
  }
  std::sort(files.begin(), files.end());
  return files;
}

// --- PlanSys2 corpus (tests/data/plansys2) ---------------------------------------------------

// Files broken on purpose in PlanSys2's own tests, with the first expected error
const std::map<std::string, std::string> kPlanSys2Broken = {
  {"plansys2_core__domain_2_error.pddl", "parse.unexpected_token@49:1"},
  {"plansys2_domain_expert__domain_2_error.pddl", "parse.unexpected_token@37:1"},
  {"plansys2_tests__test_5_error.pddl", "parse.unexpected_token@20:1"},
  {"plansys2_core__problem_simple_3.pddl", "parse.unknown_section@14:4"},
  {"plansys2_problem_expert__problem_empty_domain.pddl", "parse.unexpected_token@2:12"},
  {"plansys2_problem_expert__problem_malformed_truncated.pddl", "parse.unexpected_token@6:3"},
};

class PlanSys2CorpusTest : public ::testing::TestWithParam<fs::path>
{
};

TEST_P(PlanSys2CorpusTest, ParsesAsExpected)
{
  const fs::path & path = GetParam();
  Outcome out = check_file(path);
  auto broken = kPlanSys2Broken.find(path.filename().string());
  if (broken != kPlanSys2Broken.end()) {
    EXPECT_FALSE(out.ok);
    ASSERT_FALSE(out.diagnostics.empty());
    EXPECT_EQ(out.diagnostics[0], broken->second);
    return;
  }
  EXPECT_TRUE(out.ok) << ::testing::PrintToString(out.diagnostics);
  EXPECT_TRUE(out.diagnostics.empty()) << ::testing::PrintToString(out.diagnostics);
  EXPECT_TRUE(out.round_trip) << out.round_trip_error;
}

INSTANTIATE_TEST_SUITE_P(
  PlanSys2, PlanSys2CorpusTest,
  ::testing::ValuesIn(pddl_files(kairos::testing::data_path("plansys2"))),
  [](const ::testing::TestParamInfo<fs::path> & param_info) {
    std::string n = param_info.param.stem().string();
    std::replace_if(n.begin(), n.end(), [](char c) {return !std::isalnum(c);}, '_');
    return n;
  });

TEST(PlanSys2CorpusSanity, AllBrokenFilesExist)
{
  for (const auto & [file, code] : kPlanSys2Broken) {
    EXPECT_TRUE(fs::exists(kairos::testing::data_path("plansys2/" + file))) << file;
  }
}

// --- External corpus (optional) --------------------------------------------------------------

// Files known to be broken in public benchmark collections (relative path suffixes)
const std::vector<std::string> kExternalBroken = {
  "1D-powered-descent/instances/prob_earth20.pddl",  // :metric after the closing ')'
  "hasp/domain_cost.pddl",                           // starts with "qq(define"
  "mystery-round-1-adl/domain.pddl",                 // Lisp (in-package "PDDL") header
};

// Set KAIROS_PDDL_CORPUS to a directory (e.g. clones of potassco/pddl-instances and the IPC
// 2023/2026 numeric datasets): every .pddl in it must parse and round-trip.
TEST(ExternalCorpusTest, EveryFileParses)
{
  const char * dir = std::getenv("KAIROS_PDDL_CORPUS");
  if (dir == nullptr || !fs::is_directory(dir)) {
    GTEST_SKIP() << "KAIROS_PDDL_CORPUS not set";
  }
  size_t checked = 0;
  for (const auto & path : pddl_files(dir)) {
    std::string p = path.generic_string();
    bool known_broken = std::any_of(kExternalBroken.begin(), kExternalBroken.end(),
        [&](const std::string & suffix) {return p.ends_with(suffix);});
    Outcome out = check_file(path);
    if (known_broken) {
      EXPECT_FALSE(out.ok) << p << " is no longer broken: update kExternalBroken";
      continue;
    }
    ++checked;
    EXPECT_TRUE(out.ok) << p << "\n" << ::testing::PrintToString(out.diagnostics);
    EXPECT_TRUE(out.round_trip) << p << "\n" << out.round_trip_error;
  }
  RecordProperty("files_checked", static_cast<int>(checked));
  EXPECT_GT(checked, 0u);
}

}  // namespace
}  // namespace kairos::pddl
