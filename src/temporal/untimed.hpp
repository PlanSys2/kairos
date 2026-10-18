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
/// \brief A temporal task without durations nor times: a relaxation whose space running out
/// proves that no plan exists.

#ifndef KAIROS__TEMPORAL__UNTIMED_HPP_
#define KAIROS__TEMPORAL__UNTIMED_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include "sas/task.hpp"

namespace kairos::temporal
{

/**
 * @brief The snap task (temporal/snap.hpp) with time reduced to its order: happenings in any
 * order, `kairos-advance` between instants. The validator checks the over all conditions of
 * the running actions after each instant, so only `kairos-advance` checks them here (accept);
 * an action with an over all condition and a duration surely above 0 cannot end in the instant
 * it starts. Durations, epsilon, interference and the times of the timed initial literals are
 * left out (any order of the literals that keeps theirs); the goal holds after all of them.
 *
 * Every plan is then a path to the goal here, so a space without one proves there is no plan,
 * as long as no durative action overlaps itself (as in POPF; the snap task runs one instance
 * of each at a time). Not for continuous effects, ?duration of a duration that is not fixed,
 * processes or events (unsupported says why).
 */
struct Untimed
{
  sas::Task task;
  uint32_t advance = 0;                  // index of kairos-advance
  std::vector<sas::VarId> running;       // per durative action: 0 while it runs
  std::vector<sas::Condition> over_all;  // per durative action
  std::string unsupported;               // why this is no relaxation of the task (empty: it is)

  /// @brief Whether a transition keeps the over all conditions and the global constraints
  /// (checked when time advances, on the state of the instant).
  bool accept(uint32_t action, const uint32_t * values, const double * numbers) const;
};

Untimed untimed(const sas::Task & task);

}  // namespace kairos::temporal

#endif  // KAIROS__TEMPORAL__UNTIMED_HPP_
