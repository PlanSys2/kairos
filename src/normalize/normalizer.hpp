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
/// \brief Normalization: checked syntax tree to lifted task.

#ifndef KAIROS__NORMALIZE__NORMALIZER_HPP_
#define KAIROS__NORMALIZE__NORMALIZER_HPP_

#include <cstddef>
#include <string>
#include <vector>

#include "normalize/task.hpp"
#include "pddl/ast.hpp"
#include "semantic/model.hpp"

namespace kairos::lifted
{

struct NormalizeOptions
{
  // Disjunctions with at most this many cases split the action into copies; bigger ones
  // become auxiliary derived predicates (as Fast Downward's translator does)
  size_t max_disjuncts = 16;
};

/**
 * @brief Builds the lifted task of an analyzed domain and problem.
 *
 * - Conditions to negation normal form, as conjunctions of literals, comparisons and
 *   foralls; imply becomes or; negated comparisons are inverted.
 * - Disjunctions: few cases split the action into copies; many become auxiliary derived
 *   predicates. Existentials in preconditions become hidden parameters; elsewhere, free
 *   variables of axiom rules.
 * - Effects flattened, each with its forall variables, condition and timing; conditions of
 *   numeric effects are never split (it would apply them twice).
 * - Durative actions split into start, over-all and end conditions, timed and continuous
 *   effects, and duration constraints.
 * - Preferences are set aside (soft goals, from phase F4).
 */
Task normalize(
  const ast::Domain & domain, const sem::DomainModel & domain_model,
  const ast::Problem & problem, const sem::ProblemModel & problem_model,
  const NormalizeOptions & options = {});

/// @brief Readable text of the task (for --dump=lifted and tests).
std::string to_string(const Task & task);
std::string to_string(const Task & task, const Condition & c, const std::vector<Variable> & vars);

}  // namespace kairos::lifted

#endif  // KAIROS__NORMALIZE__NORMALIZER_HPP_
