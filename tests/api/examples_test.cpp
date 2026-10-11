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

// The examples of docs/examples: robotics domains with consumable resources, costs and
// concurrency. Every one solves, the first plan comes fast, and the costs reach the optimum.

#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <regex>
#include <sstream>
#include <string>
#include <vector>

#include "kairos/kairos.hpp"

namespace kairos
{
namespace
{

namespace fs = std::filesystem;
using namespace std::chrono_literals;

// Time bounds grow in builds that run slower
double slowdown()
{
#if defined(__SANITIZE_ADDRESS__) || defined(__SANITIZE_THREAD__) || defined(KAIROS_COVERAGE_BUILD)
  return 20.0;
#elif defined(__has_feature)
#if __has_feature(address_sanitizer) || __has_feature(thread_sanitizer)
  return 20.0;
#endif
#endif
#ifndef NDEBUG
  return 10.0;
#else
  return 1.0;
#endif
}

fs::path examples() {return fs::path(KAIROS_EXAMPLES_DIR);}

struct Example
{
  const char * domain;
  const char * problem;
};

const std::vector<Example> kSolvable = {
  {"cooking_domain.pddl", "cooking_problem_simple.pddl"},
  {"cooking_domain.pddl", "cooking_problem_long.pddl"},
  {"cooking_domain.pddl", "cooking_problem_huge.pddl"},
  {"cooking_cost_domain.pddl", "cooking_cost_problem_simple.pddl"},
  {"cooking_cost_domain.pddl", "cooking_cost_problem_medium.pddl"},
  {"cooking_cost_domain.pddl", "cooking_cost_problem_long.pddl"},
  {"factory.pddl", "factory_problem_1.pddl"},
  {"elevator_domain.pddl", "elevator_problem.pddl"},
  {"road_trip_domain.pddl", "road_trip_problem.pddl"},
  {"suave_domain.pddl", "suave_problem.pddl"},
  {"domain_simple.pddl", "domain_simple_problem_1.pddl"},
};

Problem load(const Example & e)
{
  auto d = load_domain(examples() / e.domain);
  EXPECT_TRUE(d.ok()) << e.domain;
  auto p = load_problem(examples() / e.problem, *d);
  EXPECT_TRUE(p.ok()) << e.problem;
  return *p;
}

// "; Minimum cost: N" in the problem file
double minimum_cost(const char * problem)
{
  std::ifstream in(examples() / problem);
  std::stringstream text;
  text << in.rdbuf();
  std::smatch m;
  std::string s = text.str();
  EXPECT_TRUE(std::regex_search(s, m, std::regex("; Minimum cost: ([0-9.]+)"))) << problem;
  return std::stod(m[1]);
}

TEST(ExamplesTest, EverySolvableExampleHasAValidPlanFast)
{
  for (const auto & e : kSolvable) {
    SolveOptions o;
    o.anytime = false;
    o.timeout = 60s;
    auto p = load(e);
    SolveResult r = Planner().solve(p, o);
    ASSERT_EQ(r.status, Status::Solved) << e.problem;
    EXPECT_TRUE(validate(p, *r.plan).valid()) << e.problem;
    // Milliseconds in release (the largest, cooking_problem_huge, about 150 ms): the temporal
    // profile sends them to M2, and lazy LAMA finds their numeric first plans
    ASSERT_TRUE(r.stats.first_plan_ms) << e.problem;
    EXPECT_LT(*r.stats.first_plan_ms, 1000.0 * slowdown()) << e.problem;
  }
}

TEST(ExamplesTest, IngredientCostsReachTheOptimum)
{
  // Actions that cost nothing (moving, carrying) used to stall the search on the metric
  for (const char * problem : {"cooking_cost_problem_simple.pddl",
      "cooking_cost_problem_medium.pddl", "cooking_cost_problem_long.pddl"})
  {
    SearchOptions o;
    o.timeout = 60s;
    o.target_cost = minimum_cost(problem);
    auto p = load({"cooking_cost_domain.pddl", problem});
    SolveResult r = Planner().start(p, o).wait();
    ASSERT_EQ(r.status, Status::Solved) << problem;
    EXPECT_EQ(r.stop_reason, StopReason::TargetReached) << problem;
    EXPECT_DOUBLE_EQ(*r.plan->cost, *o.target_cost) << problem;
  }
}

TEST(ExamplesTest, RobotsWorkInParallel)
{
  // The robots carry pieces in parallel: makespan 70 in milliseconds (one robot alone: 105).
  // M3 finds 54, POPF's, after about 14 s of a 30 s search: not checked here
  SearchOptions o;
  o.timeout = 60s;
  o.target_cost = 70.1;
  auto p = load({"factory.pddl", "factory_problem_1.pddl"});
  SolveResult r = Planner().start(p, o).wait();
  ASSERT_EQ(r.status, Status::Solved);
  EXPECT_EQ(r.stop_reason, StopReason::TargetReached);
  EXPECT_LE(r.plan->makespan(), 70.1);
}

TEST(ExamplesTest, AssembledPiecesAreGone)
{
  // A piece used in a car cannot be carried away afterwards
  auto p = load({"factory.pddl", "factory_problem_1.pddl"});
  SolveOptions o;
  o.anytime = false;
  SolveResult r = Planner().solve(p, o);
  ASSERT_EQ(r.status, Status::Solved);
  std::vector<std::string> used;
  for (const auto & s : r.plan->steps) {
    if (s.name == "assemble") {
      used.insert(used.end(), s.args.begin() + 2, s.args.begin() + 5);
    }
    if (s.name == "transport") {
      for (const auto & u : used) {
        EXPECT_NE(s.args[1], u) << s.action();
      }
    }
  }
  EXPECT_EQ(used.size(), 9u);
}

TEST(ExamplesTest, WrongProblemsOfTheSimpleDomain)
{
  auto d = load_domain(examples() / "domain_simple.pddl");
  ASSERT_TRUE(d.ok());
  // Jack is in no room
  auto unsolvable = load_problem(examples() / "domain_simple_problem_2.pddl", *d);
  ASSERT_TRUE(unsolvable.ok());
  EXPECT_EQ(Planner().solve(*unsolvable).status, Status::Unsolvable);
  // ":goa" and a missing parenthesis
  auto broken = load_problem(examples() / "domain_simple_problem_3.pddl", *d);
  EXPECT_FALSE(broken.ok());
}

TEST(ExamplesTest, DomainsWithoutProblemsAreValid)
{
  for (const char * domain : {"matchcellar.pddl", "stn_test.pddl", "domain_charging.pddl",
      "domain_simple_2.pddl", "simple_example.pddl", "simple_move_example.pddl",
      "factory2.pddl", "factory3.pddl", "factory4.pddl"})
  {
    std::ifstream in(examples() / domain);
    std::stringstream text;
    text << in.rdbuf();
    EXPECT_TRUE(check_domain(text.str(), domain).valid) << domain;
  }
}

}  // namespace
}  // namespace kairos
