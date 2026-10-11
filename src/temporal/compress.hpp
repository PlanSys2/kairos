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
/// \brief Engine M2: durative actions compressed into instantaneous ones, and the sequential
/// plans of the compressed task turned into temporal plans.

#ifndef KAIROS__TEMPORAL__COMPRESS_HPP_
#define KAIROS__TEMPORAL__COMPRESS_HPP_

#include <cstdint>
#include <optional>
#include <vector>

#include "kairos/plan.hpp"
#include "sas/task.hpp"
#include "state/successors.hpp"

namespace kairos::temporal
{

/**
 * @brief The compressed task: every durative action becomes an instantaneous action appended
 * after the original instantaneous ones.
 *
 * - precondition: the at start conditions, plus the over all and at end ones its own start
 *   effects do not give;
 * - effects: those at start and then those at end (the end wins on a variable both set);
 *   effect conditions are read before the action;
 * - ?duration: replaced by the duration (an equality, or else the lower bound, or the upper).
 *
 * Sound for temporally simple tasks (temporal::analyze); the validator checks every plan.
 */
struct Compressed
{
  sas::Task task;
  size_t instant = 0;                     // task.actions[0, instant): the original instant ones
  std::vector<sas::NumericExpr> duration;  // per original durative action
};

Compressed compress(const sas::Task & task);

/**
 * @brief A temporal plan from a plan of the compressed task: replays it for each duration, and
 * orders happenings (the start and end of each step), as POPF: each goes `epsilon` after the
 * last one that changed what it reads or changes, and after those that read what it changes
 * (over all conditions are read at start and at end). The rest overlap; the earliest schedule
 * of that partial order gives the least makespan for it. `actions` must be a plan of the
 * compressed task (as the search gives them: preconditions are not checked again); nullopt if
 * a step cannot be replayed or scheduled.
 */
std::optional<Plan> schedule(
  const sas::Task & original, const Compressed & compressed, state::StateSpace & space,
  const std::vector<uint32_t> & actions, double epsilon);

}  // namespace kairos::temporal

#endif  // KAIROS__TEMPORAL__COMPRESS_HPP_
