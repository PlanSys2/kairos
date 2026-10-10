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
#include <random>
#include <string>

#include "pddl/parser.hpp"
#include "pddl/printer.hpp"
#include "support/pddl_helpers.hpp"

namespace kairos::pddl
{
namespace
{

using kairos::testing::data_path;
using kairos::testing::dump;
using kairos::testing::read_file;

// Random edits that keep the text PDDL-like: delete, duplicate or replace a few characters
std::string mutate(std::string text, std::mt19937 & rng)
{
  static const std::string alphabet = "()()?:-= \n;#tabxyz019.e+*/<>^";
  std::uniform_int_distribution<int> edits(1, 4);
  std::uniform_int_distribution<int> kind(0, 2);
  std::uniform_int_distribution<size_t> letter(0, alphabet.size() - 1);
  for (int e = edits(rng); e > 0 && !text.empty(); --e) {
    std::uniform_int_distribution<size_t> at(0, text.size() - 1);
    size_t pos = at(rng);
    switch (kind(rng)) {
      case 0: text.erase(pos, 1); break;
      case 1: text.insert(pos, 1, text[pos]); break;
      default: text[pos] = alphabet[letter(rng)]; break;
    }
  }
  return text;
}

template<class Parse>
void expect_invariants(const std::string & source, Parse parse, uint32_t seed)
{
  auto first = parse(source);
  if (!first.ok()) {
    EXPECT_TRUE(has_errors(first.diagnostics())) << "seed " << seed;
    for (const auto & d : first.diagnostics()) {
      EXPECT_FALSE(d.code.empty()) << "seed " << seed;
    }
    return;
  }
  std::string printed = to_pddl(*first);
  auto second = parse(printed);
  ASSERT_TRUE(second.ok()) << "seed " << seed << "\n" << dump(second.diagnostics()) << printed;
  EXPECT_EQ(to_pddl(*second), printed) << "seed " << seed;
}

TEST(ParserPropertyTest, MutatedDomainsNeverBreakTheInvariants)
{
  const std::string domain = read_file(data_path("syntax/full_domain.pddl"));
  int parsed = 0;
  for (uint32_t seed = 0; seed < 1500; ++seed) {
    std::mt19937 rng(seed);
    std::string source = mutate(domain, rng);
    expect_invariants(source, [](const std::string & s) {return parse_domain(s);}, seed);
    parsed += parse_domain(source).ok();
  }
  // Some mutations hit comments or whitespace and still parse: both paths are exercised
  EXPECT_GT(parsed, 0);
  EXPECT_LT(parsed, 1500);
}

TEST(ParserPropertyTest, MutatedProblemsNeverBreakTheInvariants)
{
  const std::string problem = read_file(data_path("syntax/full_problem.pddl"));
  for (uint32_t seed = 0; seed < 1500; ++seed) {
    std::mt19937 rng(seed);
    expect_invariants(
      mutate(problem, rng), [](const std::string & s) {return parse_problem(s);}, seed);
  }
}

}  // namespace
}  // namespace kairos::pddl
