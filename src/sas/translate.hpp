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
/// \brief Ground task to SAS+ task.

#ifndef KAIROS__SAS__TRANSLATE_HPP_
#define KAIROS__SAS__TRANSLATE_HPP_

#include <cstddef>
#include <optional>
#include <string>

#include "core/budget.hpp"
#include "core/logger.hpp"
#include "ground/task.hpp"
#include "kairos/result.hpp"
#include "normalize/task.hpp"
#include "sas/invariants.hpp"
#include "sas/task.hpp"

namespace kairos::sas
{

struct TranslateOptions
{
  bool use_invariants = true;  // false: every atom is a binary variable
  InvariantOptions invariants;
  Logger logger;               // channel "sas"
};

struct TranslateStats
{
  InvariantStats invariants;
  size_t mutex_groups = 0;     // instantiated
  size_t group_variables = 0;  // chosen groups
  size_t binary_variables = 0;
  double milliseconds = 0.0;
};

/**
 * @brief Builds the SAS+ task: mutex groups become multi-valued variables, other atoms binary
 * variables, derived atoms binary variables with axiom layers, object fluents variables over
 * the objects they can take.
 *
 * Effects: setting a variable to its last value (none of those, false, undefined) comes from a
 * delete and applies before the other assignments of the same step (PDDL: delete, then add).
 * A delete of a group atom not known to be true becomes conditional on it.
 *
 * Errors: "sas.unstratified" (derived predicates depend on themselves through a negation),
 * "sas.budget".
 */
Result<Task> translate(
  const lifted::Task & lifted, const ground::Task & ground, const TranslateOptions & options = {},
  Budget * budget = nullptr, TranslateStats * stats = nullptr);

/// @brief The same, releasing each ground action as soon as it is translated: the peak memory
/// is little more than one of the two tasks (millions of actions do not live twice).
Result<Task> translate(
  const lifted::Task & lifted, ground::Task && ground, const TranslateOptions & options = {},
  Budget * budget = nullptr, TranslateStats * stats = nullptr);

/// @brief Linear form of an expression over numeric variables, if it has one.
std::optional<LinearExpr> linearize(const ground::Expr & e);

std::string fact_name(const Task & task, const Fact & fact);
std::string instance_name(const Task & task, const Action & a);  // from task.actions
std::string instance_name(const Task & task, const DurativeAction & a);

/// @brief Readable text of the task (for --dump=sas and tests).
std::string to_string(const Task & task);

}  // namespace kairos::sas

#endif  // KAIROS__SAS__TRANSLATE_HPP_
