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
/// \brief Grounding: lifted task to ground task, by relaxed reachability.

#ifndef KAIROS__GROUND__GROUNDER_HPP_
#define KAIROS__GROUND__GROUNDER_HPP_

#include <cstddef>
#include <string>
#include <vector>

#include "core/budget.hpp"
#include "core/logger.hpp"
#include "ground/task.hpp"
#include "kairos/result.hpp"
#include "normalize/task.hpp"

namespace kairos::ground
{

struct GroundOptions
{
  Logger logger;  // channel "ground": sizes and times
  // Instant actions with no effect, or that only add to an unread (total-cost), are dropped
  bool drop_useless_actions = true;
};

struct GroundStats
{
  size_t datalog_facts = 0;
  size_t datalog_rules = 0;
  size_t pruned_instances = 0;  // reachable in the relaxation but impossible once instantiated
  double milliseconds = 0.0;
};

/**
 * @brief Instantiates the reachable part of a lifted task.
 *
 * - Relaxed reachability in Datalog (positive atoms; static negative atoms, (in)equalities and
 *   comparisons of static fluents as filters), with one rule per action, durative start and
 *   end, effect, process, event and axiom.
 * - Static predicates and fluents (never changed) are evaluated and disappear; static fluents
 *   read in expressions become constants. A comparison that reads an undefined fluent is false.
 * - foralls are expanded over the objects of their types.
 *
 * Errors: "ground.unsupported" (a changing object fluent inside an atom), "ground.budget"
 * (time, memory or cancellation; too large to ground, the lifted path of F5 is the way out).
 */
Result<Task> ground(
  const lifted::Task & task, const GroundOptions & options = {}, Budget * budget = nullptr,
  GroundStats * stats = nullptr);

/// @brief "(move r1 a b)": the name and printed arguments of an instance.
std::string instance_name(
  const Task & task, const std::string & schema, const std::vector<ObjectId> & args);
std::string instance_name(const Task & task, const Action & a);  // from task.actions
std::string instance_name(const Task & task, const DurativeAction & a);
std::string atom_name(const Task & task, AtomId atom);

/// @brief Readable text of the task (for --dump=ground and tests).
std::string to_string(const Task & task);

}  // namespace kairos::ground

#endif  // KAIROS__GROUND__GROUNDER_HPP_
