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
/// \brief Plan validation on the syntax tree and semantic model.

#ifndef KAIROS__VALIDATE__VALIDATOR_HPP_
#define KAIROS__VALIDATE__VALIDATOR_HPP_

#include "kairos/plan.hpp"
#include "kairos/validate.hpp"
#include "pddl/ast.hpp"
#include "semantic/model.hpp"

namespace kairos::val
{

/// @brief Validates `plan` against an already analyzed domain and problem.
ValidationReport validate(
  const ast::Domain & domain, const sem::DomainModel & domain_model,
  const ast::Problem & problem, const sem::ProblemModel & problem_model, const Plan & plan,
  const ValidateOptions & options);

}  // namespace kairos::val

#endif  // KAIROS__VALIDATE__VALIDATOR_HPP_
