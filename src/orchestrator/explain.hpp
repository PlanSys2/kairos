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
/// \brief Why the goal cannot be reached: the goal facts no action sequence can make true, and
/// the chain of preconditions down to what nothing adds and the initial state does not have.

#ifndef KAIROS__ORCHESTRATOR__EXPLAIN_HPP_
#define KAIROS__ORCHESTRATOR__EXPLAIN_HPP_

#include <string>
#include <vector>

#include "ground/task.hpp"
#include "normalize/task.hpp"

namespace kairos::orch
{

/**
 * @brief One line per step of the explanation, indented by depth, for a goal that grounding
 * found unreachable even ignoring deletes. Empty if it cannot say more (a goal with
 * disjunctions or quantifiers, or numeric conditions).
 *
 * For each goal fact that is not reachable: the actions that add it and, for each, the
 * preconditions that never hold with those objects, down to facts that no action adds and the
 * initial state does not have. Bounded (depth 4, 3 actions per fact) to stay readable.
 */
std::vector<std::string> explain_unreachable_goal(
  const lifted::Task & task, const ground::Task & grounded);

}  // namespace kairos::orch

#endif  // KAIROS__ORCHESTRATOR__EXPLAIN_HPP_
