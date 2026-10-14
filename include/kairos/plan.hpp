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
/// \brief Plans: their representation, reading plans written by planners, and output formats.

#ifndef KAIROS__PLAN_HPP_
#define KAIROS__PLAN_HPP_

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "kairos/export.hpp"
#include "kairos/result.hpp"

namespace kairos
{

/// @brief One action in a plan.
struct PlanStep
{
  double time = 0.0;          // start time; the position for sequential plans (0, 1, 2...)
  double duration = 0.0;      // 0 for instantaneous actions
  std::string name;           // lower case, e.g. "move"
  std::vector<std::string> args;  // lower case, e.g. {"r1", "kitchen", "hall"}
  uint32_t line = 0;          // line in the plan file, when read with parse_plan (0 otherwise)

  /// @brief "(move r1 kitchen hall)".
  KAIROS_API std::string action() const;

  /// @brief Same action, time and duration (the source line is not compared).
  bool operator==(const PlanStep & other) const
  {
    return time == other.time && duration == other.duration && name == other.name &&
           args == other.args;
  }
};

struct Plan
{
  std::vector<PlanStep> steps;  // sorted by time (stable for equal times)
  bool temporal = false;        // steps have times and durations
  std::optional<double> cost;   // value of the problem metric, when known
  /// @brief PDDL+: the plan ends here ("t: @PlanEND"), after letting time pass since its last
  /// step (processes and events act meanwhile); the goal is checked then.
  std::optional<double> end;

  /// @brief End of the last action, or `end` if later (0 for an empty plan; the number of
  /// steps if sequential).
  KAIROS_API double makespan() const;

  bool operator==(const Plan &) const = default;
};

enum class PlanFormat : uint8_t
{
  Ipc,   // VAL / IPC: "0.000: (a x) [5.000]", or "(a x)" per line if sequential
  Popf,  // what PlanSys2's POPF plugin parses: ";;;; Solution Found" and "t: (a x)  [d]"
  Json   // one line: {"temporal":...,"cost":...,"makespan":...,"steps":[...]}
};

/**
 * @brief Text of a plan.
 *
 * `precision` is the number of decimals for times and durations (3 matches the default
 * separation of 0.001 between happenings). Comments (';') give the cost and makespan in the
 * Ipc and Popf formats.
 */
KAIROS_API std::string to_string(const Plan & plan, PlanFormat format, int precision = 3);

/**
 * @brief Reads a plan written by a planner.
 *
 * Accepts:
 * - timed lines "t: (a x y) [d]" (IPC, VAL, TFD, OPTIC); duration optional;
 * - sequential lines "(a x y)" or "n: (a x y)" (Fast Downward's sas_plan and similar);
 * - the whole stdout of POPF or OPTIC: everything before the last ";;;; Solution Found" is
 *   ignored, so the last (best) plan is read;
 * - comments from ';' to the end of the line, and blank lines.
 *
 * Names are lower-cased. Steps are sorted by time. Errors carry line and column.
 */
KAIROS_API Result<Plan> parse_plan(std::string_view text, std::string_view filename = "plan");

}  // namespace kairos

#endif  // KAIROS__PLAN_HPP_
