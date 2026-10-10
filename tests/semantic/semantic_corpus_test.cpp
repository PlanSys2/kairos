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
#include <regex>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "pddl/parser.hpp"
#include "semantic/analyzer.hpp"
#include "support/pddl_helpers.hpp"

namespace kairos::sem
{
namespace
{

namespace fs = std::filesystem;
using kairos::testing::data_path;
using kairos::testing::dump;
using kairos::testing::read_file;

std::vector<Diagnostic> errors_only(const std::vector<Diagnostic> & ds)
{
  std::vector<Diagnostic> out;
  std::copy_if(ds.begin(), ds.end(), std::back_inserter(out),
    [](const Diagnostic & d) {return d.severity == Severity::Error;});
  return out;
}

struct PairOutcome
{
  bool parsed = false;
  bool domain_ok = false;
  bool problem_ok = false;
  std::vector<Diagnostic> diagnostics;
};

PairOutcome check_pair(const fs::path & domain_file, const fs::path & problem_file)
{
  PairOutcome out;
  auto d = pddl::parse_domain(read_file(domain_file), domain_file.filename().string());
  auto p = pddl::parse_problem(read_file(problem_file), problem_file.filename().string());
  if (!d.ok() || !p.ok()) {
    out.diagnostics = d.diagnostics();
    out.diagnostics.insert(out.diagnostics.end(), p.diagnostics().begin(), p.diagnostics().end());
    return out;
  }
  out.parsed = true;
  auto dm = analyze_domain(*d);
  out.domain_ok = dm.ok();
  out.diagnostics = dm.diagnostics();
  if (!dm.ok()) {
    return out;
  }
  auto pm = analyze_problem(*p, *d, *dm);
  out.problem_ok = pm.ok();
  out.diagnostics.insert(out.diagnostics.end(), pm.diagnostics().begin(), pm.diagnostics().end());
  return out;
}

// --- PlanSys2 domains ------------------------------------------------------------------------

// Domains with errors, and the first expected error
const std::map<std::string, std::string> kPlanSys2BadDomains = {
  // Parse errors (broken on purpose in PlanSys2's tests)
  {"plansys2_core__domain_2_error.pddl", "parse.unexpected_token"},
  {"plansys2_domain_expert__domain_2_error.pddl", "parse.unexpected_token"},
  {"plansys2_tests__test_5_error.pddl", "parse.unexpected_token"},
  // A real type error in PlanSys2's test domains: (inferred-robot_at ?r1 ?r2) with ?r1 a room
  {"plansys2_domain_expert__domain_simple_derived.pddl", "semantic.type_mismatch"},
  {"plansys2_domain_expert__domain_simple_derived_processed.pddl", "semantic.type_mismatch"},
};

std::vector<fs::path> plansys2_domains()
{
  std::vector<fs::path> out;
  for (const auto & e : fs::directory_iterator(data_path("plansys2"))) {
    if (e.path().extension() == ".pddl" &&
      pddl::detect_file_kind(read_file(e.path())) == pddl::FileKind::Domain)
    {
      out.push_back(e.path());
    }
  }
  std::sort(out.begin(), out.end());
  return out;
}

class PlanSys2DomainTest : public ::testing::TestWithParam<fs::path>
{
};

TEST_P(PlanSys2DomainTest, AnalyzesAsExpected)
{
  const fs::path & path = GetParam();
  auto parsed = pddl::parse_domain(read_file(path), path.filename().string());
  std::vector<Diagnostic> ds = parsed.diagnostics();
  bool ok = parsed.ok();
  if (ok) {
    auto m = analyze_domain(*parsed);
    ok = m.ok();
    ds = m.diagnostics();
  }
  auto bad = kPlanSys2BadDomains.find(path.filename().string());
  if (bad != kPlanSys2BadDomains.end()) {
    EXPECT_FALSE(ok);
    auto errs = errors_only(ds);
    ASSERT_FALSE(errs.empty());
    EXPECT_EQ(errs[0].code, bad->second) << dump(ds);
    return;
  }
  EXPECT_TRUE(ok) << dump(ds);
  EXPECT_TRUE(errors_only(ds).empty()) << dump(ds);
}

INSTANTIATE_TEST_SUITE_P(
  PlanSys2, PlanSys2DomainTest, ::testing::ValuesIn(plansys2_domains()),
  [](const ::testing::TestParamInfo<fs::path> & param_info) {
    std::string n = param_info.param.stem().string();
    std::replace_if(n.begin(), n.end(), [](char c) {return !std::isalnum(c);}, '_');
    return n;
  });

// --- PlanSys2 problems, each with its domain -------------------------------------------------

struct Pair
{
  std::string domain;
  std::string problem;
  bool valid;
};

const std::vector<Pair> kPlanSys2Pairs = {
  {"plansys2_core__domain_1_ok", "plansys2_core__problem_simple_1", true},
  {"plansys2_core__domain_1_ok", "plansys2_core__problem_simple_2", true},
  {"plansys2_domain_expert__domain_charging", "plansys2_domain_expert__problem_charging", true},
  {"plansys2_domain_expert__domain_simple", "plansys2_domain_expert__problem_simple_1", true},
  {"plansys2_domain_expert__domain_simple", "plansys2_domain_expert__problem_0", true},
  {"plansys2_executor__elevator_domain", "plansys2_executor__elevator_problem", true},
  {"plansys2_executor__factory2", "plansys2_executor__problem_factory_1", true},
  {"plansys2_executor__domain_simple", "plansys2_executor__problem_simple_1", true},
  {"plansys2_executor__road_trip_domain", "plansys2_executor__road_trip_problem", true},
  {"plansys2_executor__suave_domain", "plansys2_executor__suave_problem", true},
  {"plansys2_pddl_parser__dom1", "plansys2_pddl_parser__prob1", true},
  {"plansys2_pddl_parser__dom2", "plansys2_pddl_parser__prob2", true},
  {"plansys2_pddl_parser__suave_domain", "plansys2_pddl_parser__suave_problem", true},
  {"plansys2_pddl_parser__suave_domain_extended_created",
    "plansys2_pddl_parser__suave_problem_extended_created", true},
  {"plansys2_planner__domain_simple_constants", "plansys2_planner__problem_simple_constants_1",
    true},
  {"plansys2_planner__domain_simple_constants", "plansys2_planner__problem_simple_constants_2",
    true},
  {"plansys2_planner__domain_simple", "plansys2_popf_plan_solver__problem_hard_unsolvable", true},
  {"plansys2_simple_example__simple_example", "plansys2_simple_example__problem", true},
  {"plansys2_terminal__simple_example", "plansys2_terminal__problem_charging", true},
  {"plansys2_problem_expert__domain_simple", "plansys2_problem_expert__problem_simple_1", true},
  {"plansys2_tests__test_2", "plansys2_tests__problem_2", true},
  // Broken on purpose in PlanSys2's tests: wrong arity and a robot where a room goes
  {"plansys2_problem_expert__domain_simple", "plansys2_problem_expert__problem_unexpected_syntax",
    false},
};

class PlanSys2PairTest : public ::testing::TestWithParam<Pair>
{
};

TEST_P(PlanSys2PairTest, AnalyzesAsExpected)
{
  const Pair & pair = GetParam();
  PairOutcome out = check_pair(
    data_path("plansys2/" + pair.domain + ".pddl"),
        data_path("plansys2/" + pair.problem + ".pddl"));
  ASSERT_TRUE(out.parsed) << dump(out.diagnostics);
  ASSERT_TRUE(out.domain_ok) << dump(out.diagnostics);
  EXPECT_EQ(out.problem_ok, pair.valid) << dump(out.diagnostics);
}

INSTANTIATE_TEST_SUITE_P(
  PlanSys2, PlanSys2PairTest, ::testing::ValuesIn(kPlanSys2Pairs),
  [](const ::testing::TestParamInfo<Pair> & param_info) {
    std::string n = param_info.param.problem;
    std::replace_if(n.begin(), n.end(), [](char c) {return !std::isalnum(c);}, '_');
    return n;
  });

TEST(PlanSys2UnexpectedSyntax, ReportsTheTwoPlantedErrors)
{
  PairOutcome out = check_pair(
    data_path("plansys2/plansys2_problem_expert__domain_simple.pddl"),
    data_path("plansys2/plansys2_problem_expert__problem_unexpected_syntax.pddl"));
  auto errs = kairos::testing::codes(errors_only(out.diagnostics));
  EXPECT_EQ(errs, (std::vector<std::string>{"semantic.wrong_arity@16:5",
      "semantic.type_mismatch@18:23"}));
}

TEST(FullSyntaxFiles, AreSemanticallyValid)
{
  PairOutcome out = check_pair(data_path("syntax/full_domain.pddl"),
      data_path("syntax/full_problem.pddl"));
  EXPECT_TRUE(out.problem_ok) << dump(out.diagnostics);
  EXPECT_TRUE(out.diagnostics.empty()) << dump(out.diagnostics);
}

// --- External corpus (optional) --------------------------------------------------------------

// Pairs with real errors in public collections (path fragments)
const std::vector<std::string> kExternalBadPairs = {
  "hstairs_pddl_plus_domains/hvac/",                         // uses air_flow, declares air-flow
  "hstairs_pddl_plus_domains/zermelo_navigation_problem/",   // (ahead) without the ship
  "assembly-round-1-adl/additional-notes/",                  // instance-*-bug.pddl
  "mystery-round-1-adl/",                                    // Lisp (in-package ...) header
  "satellite-time-time-windows-strips/instances/instance-31.pddl",           // senty_image
  "satellite-time-time-windows-compiled-strips/instances/instance-31.pddl",  // senty_image
};

// Domain for a problem in the usual layouts: X/domains/domain-N.pddl, X/domain.pddl
std::optional<fs::path> domain_for(const fs::path & problem)
{
  fs::path dir = problem.parent_path();
  fs::path base = dir.filename() == "instances" ? dir.parent_path() : dir;
  std::smatch m;
  std::string name = problem.filename().string();
  std::vector<fs::path> candidates;
  if (std::regex_search(name, m, std::regex("(\\d+)"))) {
    candidates.push_back(base / "domains" / ("domain-" + m[1].str() + ".pddl"));
  }
  candidates.push_back(base / "domain.pddl");
  candidates.push_back(base.parent_path() / "domain.pddl");
  for (const auto & c : candidates) {
    if (fs::exists(c)) {
      return c;
    }
  }
  return std::nullopt;
}

// With KAIROS_PDDL_CORPUS set (see tools/fetch_corpus.sh), every domain/problem pair that can
// be matched must pass the semantic analysis, except the known bad ones
TEST(ExternalSemanticCorpusTest, EveryPairIsValid)
{
  const char * dir = std::getenv("KAIROS_PDDL_CORPUS");
  if (dir == nullptr || !fs::is_directory(dir)) {
    GTEST_SKIP() << "KAIROS_PDDL_CORPUS not set";
  }
  size_t checked = 0;
  std::set<std::string> bad_seen;
  for (const auto & e : fs::recursive_directory_iterator(dir)) {
    if (!e.is_regular_file() || e.path().extension() != ".pddl") {
      continue;
    }
    std::string text = read_file(e.path());
    if (pddl::detect_file_kind(text) != pddl::FileKind::Problem) {
      continue;
    }
    auto domain = domain_for(e.path());
    if (!domain) {
      continue;
    }
    std::string p = e.path().generic_string();
    auto known = std::find_if(kExternalBadPairs.begin(), kExternalBadPairs.end(),
        [&](const std::string & frag) {return p.find(frag) != std::string::npos;});
    PairOutcome out = check_pair(*domain, e.path());
    bool valid = out.parsed && out.domain_ok && out.problem_ok;
    if (known != kExternalBadPairs.end()) {
      if (!valid) {
        bad_seen.insert(*known);
      }
      continue;
    }
    ++checked;
    EXPECT_TRUE(valid) << p << "\n" << dump(errors_only(out.diagnostics));
  }
  EXPECT_GT(checked, 0u);
  for (const auto & frag : kExternalBadPairs) {
    EXPECT_TRUE(bad_seen.count(frag)) << frag << " no longer fails: update kExternalBadPairs";
  }
}

}  // namespace
}  // namespace kairos::sem
