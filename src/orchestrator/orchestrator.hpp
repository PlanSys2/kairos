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
/// \brief The orchestrator: from a checked problem to validated plans.

#ifndef KAIROS__ORCHESTRATOR__ORCHESTRATOR_HPP_
#define KAIROS__ORCHESTRATOR__ORCHESTRATOR_HPP_

#include <chrono>
#include <cstdint>
#include <functional>
#include <stop_token>  // NOLINT(build/include_order): cpplint does not know this header
#include <vector>

#include "kairos/planner.hpp"
#include "pddl/ast.hpp"
#include "semantic/features.hpp"
#include "semantic/model.hpp"

namespace kairos::orch
{

struct Input
{
  const ast::Domain & domain;
  const sem::DomainModel & domain_model;
  const ast::Problem & problem;
  const sem::ProblemModel & problem_model;
};

/// @brief Called for each better valid plan, in the planning thread.
using PlanCallback = std::function<void (const Plan & plan, uint32_t index,
    std::chrono::milliseconds elapsed)>;

/// @brief Why Kairos cannot plan with these features yet (empty if it can). Shared with
/// check_*() for CheckReport::plannable.
std::vector<Diagnostic> unsupported(const sem::FeatureSet & features, Engine engine = Engine::Auto);

/**
 * @brief Runs the whole pipeline: normalization, grounding, SAS+, search (M1), validating every
 * plan with the independent validator before handing it over. Never throws.
 */
SolveResult run(
  const Input & input, const SearchOptions & options, std::stop_token stop,
  const PlanCallback & on_plan);

}  // namespace kairos::orch

#endif  // KAIROS__ORCHESTRATOR__ORCHESTRATOR_HPP_
