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
/// \brief Validation of PDDL domains and problems without planning.

#ifndef KAIROS__CHECK_HPP_
#define KAIROS__CHECK_HPP_

#include <string>
#include <string_view>
#include <vector>

#include "kairos/diagnostics.hpp"
#include "kairos/export.hpp"

namespace kairos
{

/// @brief Outcome of a check.
struct CheckReport
{
  bool valid = false;                   // correct PDDL: no error diagnostics
  bool plannable = false;               // valid, and Kairos can plan with everything it uses
  std::vector<Diagnostic> diagnostics;  // errors, warnings and notes, with positions
  std::vector<std::string> features;    // PDDL features actually used, e.g. "durative-actions"
};

/// @brief Checks a domain given as text: syntax, declarations, types, arities, variables.
KAIROS_API CheckReport check_domain(
  std::string_view domain_text, std::string_view filename = "domain.pddl");

/**
 * @brief Checks a problem on its own. Without the domain, types and arities cannot be checked,
 * and names not in :objects are warnings (they may be domain constants).
 */
KAIROS_API CheckReport check_problem(
  std::string_view problem_text, std::string_view filename = "problem.pddl");

/// @brief Checks a problem against its domain (the domain is checked too).
KAIROS_API CheckReport check_problem(
  std::string_view problem_text, std::string_view domain_text,
  std::string_view problem_filename, std::string_view domain_filename = "domain.pddl");

}  // namespace kairos

#endif  // KAIROS__CHECK_HPP_
