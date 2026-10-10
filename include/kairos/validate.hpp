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
/// \brief Plan validation, independent of how the plan was produced.

#ifndef KAIROS__VALIDATE_HPP_
#define KAIROS__VALIDATE_HPP_

#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "kairos/diagnostics.hpp"
#include "kairos/export.hpp"
#include "kairos/plan.hpp"

namespace kairos
{

enum class ValidationStatus : uint8_t
{
  Valid,        // the plan is executable and reaches the goal
  Invalid,      // it is not; the diagnostics say where and why
  Unsupported,  // uses something the validator cannot check yet (e.g. processes)
  BadInput      // the domain or problem has errors (in the diagnostics)
};

struct ValidateOptions
{
  double epsilon = 0.001;            // minimum separation between interfering happenings
  double tolerance = 1e-6;           // for numeric comparisons and simultaneity
  double duration_tolerance = 1e-3;  // for (= ?duration ...): plans print 3 decimals
  // Interfering happenings closer than epsilon: an error if true, a warning if false. PDDL 2.1
  // asks for the separation, but VAL does not enforce it; false keeps VAL's verdicts.
  bool strict_epsilon = false;
  bool check_goal = true;            // false: only check that the plan is executable
  std::string plan_filename = "plan";
};

struct ValidationReport
{
  ValidationStatus status = ValidationStatus::BadInput;
  std::vector<Diagnostic> diagnostics;  // the first failure, plus warnings
  double makespan = 0.0;
  std::optional<double> metric;         // value of the problem :metric, if any and valid
  std::map<std::string, uint32_t> violated_preferences;  // name -> times violated

  bool valid() const {return status == ValidationStatus::Valid;}
};

/**
 * @brief Checks that `plan` solves the problem.
 *
 * Follows VAL's semantics: the starts and ends of durative actions, instantaneous actions and
 * timed initial literals form happenings; the conditions of a happening hold before it and its
 * effects apply together; simultaneous or too close (less than epsilon) happenings must not
 * interfere; over-all conditions hold over the open interval of each action; linear continuous
 * effects are integrated between happenings; the goal holds at the end. Also computes the metric
 * (total-time, fluents, violated preferences, PDDL3 trajectory constraints).
 *
 * The validator works on the syntax tree, not on the planner's internal task, so a bug in
 * grounding cannot make a wrong plan look valid.
 */
KAIROS_API ValidationReport validate_plan(
  std::string_view domain_text, std::string_view problem_text, const Plan & plan,
  const ValidateOptions & options = {}, std::string_view domain_filename = "domain.pddl",
  std::string_view problem_filename = "problem.pddl");

/// @brief "valid", "invalid", "unsupported" or "bad input".
KAIROS_API std::string_view to_string(ValidationStatus status);

}  // namespace kairos

#endif  // KAIROS__VALIDATE_HPP_
