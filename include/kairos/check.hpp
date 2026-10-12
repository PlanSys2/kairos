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

#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

#include "kairos/diagnostics.hpp"
#include "kairos/export.hpp"

namespace kairos
{

/// @brief How far Kairos plans with a PDDL feature.
enum class SupportLevel : uint8_t
{
  Full,     // plans with it
  Partial,  // plans with it with limits (the note says which)
  None      // cannot plan with it yet
};

/// @brief "full", "partial" or "none".
KAIROS_API std::string_view to_string(SupportLevel level);

/// @brief One PDDL feature and how far Kairos plans with it.
struct FeatureSupport
{
  std::string feature;                    // e.g. "durative-actions"
  std::vector<std::string> requirements;  // the :requirements that declare it
  SupportLevel level = SupportLevel::Full;
  std::string note;                       // the limits, or when it will come
};

/// @brief Every PDDL feature Kairos knows, with its support (what `kairos --features` prints).
KAIROS_API std::vector<FeatureSupport> pddl_support();

/// @brief Outcome of a check.
struct CheckReport
{
  bool valid = false;                   // correct PDDL: no error diagnostics
  bool plannable = false;               // valid, and Kairos can plan with everything it uses
  /// @brief Whether `plannable` is known: not for a problem checked without its domain
  bool plannable_known = true;
  std::vector<Diagnostic> diagnostics;  // errors, warnings and notes, with positions
  std::vector<std::string> features;    // PDDL features actually used, e.g. "durative-actions"
  std::vector<FeatureSupport> support;  // the support of each of those features
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
