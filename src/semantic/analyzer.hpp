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
/// \brief Semantic analysis: checks a parsed domain and problem and builds their models.

#ifndef KAIROS__SEMANTIC__ANALYZER_HPP_
#define KAIROS__SEMANTIC__ANALYZER_HPP_

#include <vector>

#include "kairos/result.hpp"
#include "pddl/ast.hpp"
#include "semantic/model.hpp"

namespace kairos::sem
{

/**
 * @brief Checks a domain: declarations, types, arities, variables, placement of temporal
 * constructs, derived predicates, and requirements declared versus used.
 *
 * Fails if any error is found; warnings travel with the model.
 */
Result<DomainModel> analyze_domain(const ast::Domain & domain);

/// @brief Checks a problem against its (already analyzed) domain.
Result<ProblemModel> analyze_problem(
  const ast::Problem & problem, const ast::Domain & domain, const DomainModel & model);

/**
 * @brief Checks a problem on its own: what can be checked without the domain.
 *
 * Symbols not declared in :objects are warnings (they may be domain constants), not errors.
 */
std::vector<Diagnostic> check_problem_alone(const ast::Problem & problem);

}  // namespace kairos::sem

#endif  // KAIROS__SEMANTIC__ANALYZER_HPP_
