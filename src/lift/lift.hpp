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
/// \brief Planning without grounding: for tasks too large to ground.

#ifndef KAIROS__LIFT__LIFT_HPP_
#define KAIROS__LIFT__LIFT_HPP_

#include <cstdint>
#include <functional>
#include <optional>
#include <string>

#include "core/budget.hpp"
#include "core/logger.hpp"
#include "kairos/plan.hpp"
#include "normalize/task.hpp"

namespace kairos::lift
{

/// @brief Why the lifted engine cannot plan this task (empty if it can): classical tasks with
/// types, equality, negative preconditions and existential parameters; no numbers, durative
/// actions, processes, axioms, quantified conditions or conditional effects.
std::string unsupported(const lifted::Task & task);

struct Stats
{
  uint64_t expanded = 0;
  uint64_t generated = 0;
  uint64_t states = 0;
};

/**
 * @brief Searches the task as it is, without grounding it (as Powerlifted, Correa et al.):
 * states are the true atoms of the predicates actions change, successors come from joining
 * each action's preconditions over the state and the static facts, and the search alternates
 * a queue by novelty and goal count with one by a lifted FF heuristic (evaluated on expansion;
 * the goal count when the relaxed exploration is too large). Returns the first plan found, or
 * none when the space is exhausted (then the task is unsolvable), the budget is spent or
 * `stop` says so.
 */
std::optional<Plan> plan(
  const lifted::Task & task, Budget * budget, Logger logger, Stats & stats,
  const std::function<bool()> & stop = {});

}  // namespace kairos::lift

#endif  // KAIROS__LIFT__LIFT_HPP_
