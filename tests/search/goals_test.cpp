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

#include <chrono>
#include <memory>
#include <stop_token>  // NOLINT(build/include_order): cpplint does not know this header
#include <string>
#include <vector>

#include "heuristics/heuristic.hpp"
#include "kairos/plan.hpp"
#include "kairos/validate.hpp"
#include "search/goals.hpp"
#include "search/search.hpp"
#include "support/pipeline_helpers.hpp"

namespace kairos::search
{
namespace
{

using kairos::testing::pipeline;

struct Run
{
  std::string domain, problem;
  std::unique_ptr<kairos::testing::Pipeline> p;
  std::unique_ptr<Budget> budget;
  std::unique_ptr<state::StateRegistry> reg;
  std::unique_ptr<state::StateSpace> space;
  std::vector<double> costs;
  std::unique_ptr<PlanSink> sink;
  std::unique_ptr<SearchContext> ctx;

  ::testing::AssertionResult best_is_valid() const
  {
    auto best = sink->best();
    if (!best) {
      return ::testing::AssertionFailure() << "no plan";
    }
    Plan plan = to_plan(p->sas, best->actions);
    auto r = validate_plan(domain, problem, plan);
    if (!r.valid()) {
      std::string msg;
      for (const auto & d : r.diagnostics) {
        msg += to_string(d) + "\n";
      }
      return ::testing::AssertionFailure() << msg << to_string(plan, PlanFormat::Ipc);
    }
    return ::testing::AssertionSuccess();
  }
};

std::unique_ptr<Run> setup(
  const std::string & domain, const std::string & problem, std::stop_token stop = {})
{
  auto r = std::make_unique<Run>();
  r->domain = domain;
  r->problem = problem;
  r->p = pipeline(domain, problem);
  BudgetConfig cfg;
  cfg.stop = stop;
  cfg.deadline = BudgetConfig::Clock::now() + std::chrono::seconds(30);
  r->budget = std::make_unique<Budget>(cfg);
  r->reg = std::make_unique<state::StateRegistry>(r->p->sas, r->budget.get());
  r->space = std::make_unique<state::StateSpace>(r->p->sas, *r->reg);
  r->costs = heuristics::action_costs(r->p->sas, heuristics::CostModel::Unit);
  r->sink = std::make_unique<PlanSink>();
  r->ctx = std::make_unique<SearchContext>(
    r->p->sas, *r->reg, *r->space, r->costs, *r->sink, r->budget.get());
  return r;
}

const char * kPatrol =
  "(define (domain patrol) (:requirements :strips :typing) (:types robot place)"
  " (:predicates (at ?r - robot ?p - place) (link ?a ?b - place) (seen ?p - place))"
  " (:action move :parameters (?r - robot ?a ?b - place)"
  "  :precondition (and (at ?r ?a) (link ?a ?b)) :effect (and (not (at ?r ?a)) (at ?r ?b)))"
  " (:action look :parameters (?r - robot ?p - place)"
  "  :precondition (at ?r ?p) :effect (seen ?p)))";

// A side x side grid, robots in opposite corners, goals: every place seen (and more)
std::string grid(int side, const std::string & init_extra, const std::string & goal_extra)
{
  std::string objects, links, goals;
  auto name = [](int x, int y) {return "p" + std::to_string(x) + "_" + std::to_string(y);};
  for (int x = 0; x < side; ++x) {
    for (int y = 0; y < side; ++y) {
      objects += " " + name(x, y);
      goals += " (seen " + name(x, y) + ")";
      if (x + 1 < side) {
        links += " (link " + name(x, y) + " " + name(x + 1, y) + ")";
        links += " (link " + name(x + 1, y) + " " + name(x, y) + ")";
      }
      if (y + 1 < side) {
        links += " (link " + name(x, y) + " " + name(x, y + 1) + ")";
        links += " (link " + name(x, y + 1) + " " + name(x, y) + ")";
      }
    }
  }
  return "(define (problem g) (:domain patrol) (:objects r1 r2 - robot" + objects +
         " - place) (:init (at r1 " + name(0, 0) + ") (at r2 " + name(side - 1, side - 1) +
         ")" + links + init_extra + ") (:goal (and" + goals + goal_extra + ")))";
}

TEST(GoalsTest, ManyGoalsOneAtATime)
{
  auto r = setup(kPatrol, grid(8, "", ""));
  ASSERT_TRUE(goals_apply(r->p->sas));
  EXPECT_EQ(incremental_goals(*r->ctx), Outcome::Solved);
  EXPECT_TRUE(r->best_is_valid());
  // 64 looks and the moves between them: greedy, but not wandering
  EXPECT_LE(r->sink->best()->actions.size(), 64u + 3 * 64u);
}

TEST(GoalsTest, KeepsTheGoalsAlreadyTrue)
{
  // r1 starts where it must end: every look elsewhere has to bring it back
  auto r = setup(kPatrol, grid(4, "", " (at r1 p0_0)"));
  EXPECT_EQ(incremental_goals(*r->ctx), Outcome::Solved);
  EXPECT_TRUE(r->best_is_valid());
}

TEST(GoalsTest, GoalsTrueFromTheStartMayBreakForAWhile)
{
  // A warehouse: every shelf starts home (a goal) and must visit the station for its item;
  // each delivery breaks a goal true from the start, which then comes back as a goal
  std::string objects, init, goals;
  const int n = 25;
  for (int i = 0; i < n; ++i) {
    std::string s = "s" + std::to_string(i), z = "z" + std::to_string(i);
    objects += " " + s;
    init += " (shelf-at " + s + " " + z + ") (home " + s + " " + z + ") (link st " + z + ")" +
      " (link " + z + " st)";
    goals += " (shown " + s + ") (shelf-at " + s + " " + z + ")";
  }
  std::string zones;
  for (int i = 0; i < n; ++i) {
    zones += " z" + std::to_string(i);
  }
  const char * domain =
    "(define (domain wh) (:requirements :strips :typing) (:types shelf zone)"
    " (:predicates (at ?z - zone) (shelf-at ?s - shelf ?z - zone) (home ?s - shelf ?z - zone)"
    "  (carrying ?s - shelf) (free) (link ?a ?b - zone) (shown ?s - shelf)"
    "  (station ?z - zone))"
    " (:action move :parameters (?a ?b - zone) :precondition (and (at ?a) (link ?a ?b))"
    "  :effect (and (not (at ?a)) (at ?b)))"
    " (:action lift :parameters (?s - shelf ?z - zone)"
    "  :precondition (and (at ?z) (shelf-at ?s ?z) (free))"
    "  :effect (and (not (shelf-at ?s ?z)) (not (free)) (carrying ?s)))"
    " (:action show :parameters (?s - shelf ?z - zone)"
    "  :precondition (and (at ?z) (station ?z) (carrying ?s))"
    "  :effect (shown ?s))"
    " (:action lower :parameters (?s - shelf ?z - zone)"
    "  :precondition (and (at ?z) (carrying ?s) (home ?s ?z))"
    "  :effect (and (shelf-at ?s ?z) (free) (not (carrying ?s)))))";
  std::string problem = "(define (problem p) (:domain wh) (:objects" + objects +
    " - shelf st" + zones + " - zone) (:init (at st) (station st) (free)" + init +
    ") (:goal (and" + goals + ")))";
  auto r = setup(domain, problem);
  ASSERT_TRUE(goals_apply(r->p->sas));
  EXPECT_EQ(incremental_goals(*r->ctx), Outcome::Solved);
  EXPECT_TRUE(r->best_is_valid());
}

TEST(GoalsTest, GoalsTrueFromTheStartNeedNothing)
{
  std::string seen;
  for (int x = 0; x < 3; ++x) {
    for (int y = 0; y < 3; ++y) {
      seen += " (seen p" + std::to_string(x) + "_" + std::to_string(y) + ")";
    }
  }
  auto r = setup(kPatrol, grid(3, seen, ""));
  EXPECT_EQ(incremental_goals(*r->ctx), Outcome::Solved);
  ASSERT_TRUE(r->sink->best());
  EXPECT_TRUE(r->sink->best()->actions.empty());
}

TEST(GoalsTest, FailsWhenTheGoalsCannotAllHold)
{
  // One token for two goals: each is reachable alone (and relaxed), not both
  const char * domain =
    "(define (domain t) (:requirements :strips)"
    " (:predicates (token) (g1) (g2))"
    " (:action a1 :precondition (token) :effect (and (g1) (not (token))))"
    " (:action a2 :precondition (token) :effect (and (g2) (not (token)))))";
  const char * problem = "(define (problem p) (:domain t) (:init (token))"
    " (:goal (and (g1) (g2))))";
  auto r = setup(domain, problem);
  EXPECT_EQ(incremental_goals(*r->ctx), Outcome::Failed);
  EXPECT_FALSE(r->sink->best());
}

TEST(GoalsTest, OnlyGoalsOfFacts)
{
  auto many = setup(kPatrol, grid(5, "", ""));
  EXPECT_TRUE(goals_apply(many->p->sas));
  EXPECT_FALSE(goals_apply(many->p->sas, 26));
  auto few = setup(kPatrol, grid(3, "", ""));
  EXPECT_FALSE(goals_apply(few->p->sas));  // 9 goals
  EXPECT_TRUE(goals_apply(few->p->sas, 9));
  const char * numeric =
    "(define (domain n) (:requirements :fluents) (:predicates (p)) (:functions (x))"
    " (:action inc :effect (increase (x) 1)))";
  auto n = setup(numeric, "(define (problem p) (:domain n) (:init (= (x) 0))"
    " (:goal (>= (x) 3)))");
  EXPECT_FALSE(goals_apply(n->p->sas, 1));
  EXPECT_EQ(incremental_goals(*n->ctx), Outcome::Failed);
}

TEST(GoalsTest, StopsWhenAsked)
{
  std::stop_source stop;
  stop.request_stop();
  auto r = setup(kPatrol, grid(6, "", ""), stop.get_token());
  EXPECT_EQ(incremental_goals(*r->ctx), Outcome::Stopped);
  EXPECT_FALSE(r->sink->best());
}

TEST(GoalsTest, AnytimeImprovesAPlanItDidNotFind)
{
  auto r = setup(kPatrol, grid(3, "", ""));
  ASSERT_EQ(incremental_goals(*r->ctx), Outcome::Solved);
  double first = r->sink->bound();
  size_t expanded = r->ctx->stats.expanded;
  AnytimeOptions ao;
  ao.skip_first = true;
  ao.improve = false;
  EXPECT_EQ(anytime(*r->ctx, ao), AnytimeResult::NoMoreImprovement);
  EXPECT_EQ(r->ctx->stats.expanded, expanded);  // no first search
  // Improving: only cheaper plans, down to a proof
  r->ctx->prune_with_bound = true;
  ao.improve = true;
  EXPECT_EQ(anytime(*r->ctx, ao), AnytimeResult::ProvenOptimal);
  EXPECT_LE(r->sink->bound(), first);
  EXPECT_TRUE(r->best_is_valid());
  // Without a plan in the sink, skip_first searches as usual
  auto e = setup(kPatrol, grid(3, "", ""));
  ao.improve = false;
  EXPECT_EQ(anytime(*e->ctx, ao), AnytimeResult::NoMoreImprovement);
  EXPECT_TRUE(e->best_is_valid());
}

}  // namespace
}  // namespace kairos::search
