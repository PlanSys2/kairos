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
/// \brief Repairs of the ENHSP dialect on the syntax tree, before the semantic analysis.

#ifndef KAIROS__PDDL__DIALECT_HPP_
#define KAIROS__PDDL__DIALECT_HPP_

#include <vector>

#include "kairos/diagnostics.hpp"
#include "pddl/ast.hpp"

namespace kairos::pddl
{

/**
 * @brief What ENHSP accepts and PDDL does not, rewritten as PDDL with a warning each:
 * a function without arguments written without parentheses (`(* #t q)`), '_' for '-' or the
 * reverse in the name of a predicate or function (`air_flow` for `air-flow`), a function
 * declared again with other parameter types (merged into `either` types), and a condition on a
 * predicate the domain does not declare (never true).
 */
std::vector<Diagnostic> repair_domain(ast::Domain & domain);

/// @brief The same for a problem, against its domain (already repaired); initial facts with
/// another number of arguments than their predicate are dropped, and a function that only
/// continuous effects change, left without a value, starts at 0 (SMTPlan's benchmarks).
std::vector<Diagnostic> repair_problem(ast::Problem & problem, const ast::Domain & domain);

}  // namespace kairos::pddl

#endif  // KAIROS__PDDL__DIALECT_HPP_
