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
/// \brief What kind of temporal task it is: which engine can plan it.

#ifndef KAIROS__TEMPORAL__PROFILE_HPP_
#define KAIROS__TEMPORAL__PROFILE_HPP_

#include <string>
#include <vector>

#include "sas/task.hpp"

namespace kairos::temporal
{

struct TemporalProfile
{
  bool durative = false;           // some durative action
  /// @brief No plan needs actions to overlap: compressing each durative action into one
  /// instantaneous step loses no plan (engine M2). Conservative: false when unsure.
  bool simple = true;
  bool timed_initial_literals = false;
  bool variable_duration = false;  // a duration constraint that is not an equality
  bool duration_in_effects = false;
  /// @brief Why the task may require concurrency, one line per reason (for the log).
  std::vector<std::string> reasons;
};

/**
 * @brief Analyses the task, conservatively (after Cushing et al., IJCAI 2007): it may need
 * concurrency if a durative action leaves a value visible only while it runs (it changes a
 * variable at start and again at end) and another action reads that variable, or if timed
 * initial literals change what actions read. A false "may need" only costs time (M3 tries
 * too); a false "simple" would give plans the validator rejects.
 */
TemporalProfile analyze(const sas::Task & task);

}  // namespace kairos::temporal

#endif  // KAIROS__TEMPORAL__PROFILE_HPP_
