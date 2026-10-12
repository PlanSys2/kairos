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
/// \brief Durations the engines can plan with: duration inequalities with ?duration in the
/// effects become one action per bound, and linear continuous effects become discrete ones.

#ifndef KAIROS__TEMPORAL__DURATIONS_HPP_
#define KAIROS__TEMPORAL__DURATIONS_HPP_

#include <string>

#include "sas/task.hpp"

namespace kairos::temporal
{

struct FixedDurations
{
  sas::Task task;
  /// @brief Why the task cannot be planned this way (empty if it can).
  std::string unsupported;
  /// @brief Whether anything changed (actions split, continuous effects made discrete).
  bool changed = false;
};

/**
 * @brief Rewrites the durative actions that need a concrete duration: those with continuous
 * effects or ?duration in their effects and conditions.
 *
 * - A duration given by inequalities becomes one action per bound, each with that bound as
 *   its duration and the condition that it satisfies the other bounds ("recharge the minimum",
 *   "recharge until full"). Durations in between are not tried: sound, not complete.
 * - A linear continuous effect (rate * #t) becomes its total change, rate * ?duration, at the
 *   happening where it is safest for the conditions that read the variable: when they all
 *   need it large enough (>=), a decrease at start and an increase at end; when they all need
 *   it small enough, the other way round. Since the change is linear, a condition that holds
 *   there holds along the real trajectory. Mixed readers: unsupported.
 *
 * The validator checks every plan with the real semantics (continuous change included).
 */
FixedDurations fix_durations(const sas::Task & task);

}  // namespace kairos::temporal

#endif  // KAIROS__TEMPORAL__DURATIONS_HPP_
