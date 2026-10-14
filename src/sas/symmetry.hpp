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
/// \brief Objects that the task does not tell apart: swapping them maps it to itself.

#ifndef KAIROS__SAS__SYMMETRY_HPP_
#define KAIROS__SAS__SYMMETRY_HPP_

#include <cstddef>
#include <vector>

#include "sas/task.hpp"

namespace kairos::sas
{

/**
 * @brief Finds transpositions of two objects that map the task to itself: every action to an
 * action, and the axioms, goal, global constraints, rates, deadlines and metric to themselves
 * (not the initial state: a state where the two objects look the same is enough to prune).
 * Candidates are objects with the same signature (the argument positions they appear in),
 * each class tested pair by pair in order. None with object fluents, durative actions,
 * processes or events (compiled first), or more than `max_actions` actions.
 */
std::vector<Symmetry> find_symmetries(const Task & task, size_t max_actions = 100000);

/// @brief Whether `values` and `numbers` stay the same when the symmetry swaps its objects.
bool invariant(const Symmetry & s, const uint32_t * values, const double * numbers);

}  // namespace kairos::sas

#endif  // KAIROS__SAS__SYMMETRY_HPP_
