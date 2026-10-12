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

/// \file
/// \brief Incremental goals: the goal facts one at a time, each with a short search.

#ifndef KAIROS__SEARCH__GOALS_HPP_
#define KAIROS__SEARCH__GOALS_HPP_

#include <cstddef>

#include "sas/task.hpp"
#include "search/search.hpp"

namespace kairos::search
{

struct GoalsOptions
{
  // Short searches give up soon: goals that undo each other (rubiks-cube, parking) would
  // otherwise spend seconds before the other searches start. Each may expand `per_goal`
  // times the mean of the goals reached so far, within [min_expansions, max_expansions]
  size_t min_expansions = 500;
  size_t max_expansions = 5000;
  size_t per_goal = 50;
  size_t candidates = 2;  // goals tried in a round before giving up
};

/// @brief Whether incremental goals applies: a goal of facts only, and enough of them.
bool goals_apply(const sas::Task & task, size_t min_goals = 20);

/**
 * @brief A first plan for tasks with many goal facts (patrolling, visiting, delivering), where
 * one search for all of them meets plateaus that grow with the agents and goals.
 *
 * From the current state, the nearest goal fact not yet true (the first a relaxed exploration
 * reaches) is reached by a short lazy greedy search (FF, preferred operators) that keeps the
 * goal facts already true; its steps are appended and it goes on from there. A goal
 * whose search fails is skipped in that round; when every candidate of a round fails, the
 * method fails (other phases go on). The plan goes to ctx.sink. The heuristic sees one goal at
 * a time, so its relaxed exploration stops close to the state.
 *
 * Only instant actions with the costs of ctx (M1 tasks, and M2's compressed ones).
 */
Outcome incremental_goals(SearchContext & ctx, const GoalsOptions & options = {});

}  // namespace kairos::search

#endif  // KAIROS__SEARCH__GOALS_HPP_
