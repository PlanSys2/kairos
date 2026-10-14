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
/// \brief PDDL+ (processes and events) as a discrete task the search plans with.

#ifndef KAIROS__TEMPORAL__HYBRID_HPP_
#define KAIROS__TEMPORAL__HYBRID_HPP_

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "kairos/plan.hpp"
#include "sas/task.hpp"
#include "state/successors.hpp"

namespace kairos::temporal
{

/**
 * @brief A PDDL+ task as a discrete one: the domain's actions (instantaneous), the start and
 * end of each durative action, then `kairos-wait`, which lets up to `delta` seconds pass (the
 * active processes integrated with Runge-Kutta; its effects, Euler steps, only guide the
 * heuristics), and `kairos-wait-long`, which lets steps pass until some numeric condition
 * changes, then the events and the timed initial literals as actions. Events are forced:
 * while one holds, only events can be taken. Global constraints, and the over all conditions
 * of the running actions, must hold in every state reached.
 *
 * A durative action runs as start, process and end (Fox and Long): its start sets a clock and
 * the duration bounds, a process advances the clock and applies its continuous effects, its
 * end needs the clock at the duration. Waits stop at those ends and at the timed literals, so
 * both happen at their exact time.
 *
 * The plans are those of the discrete semantics; every one is validated afterwards with the
 * continuous one (the validator integrates exactly and fires events when they cross).
 */
struct Hybrid
{
  sas::Task task;
  uint32_t wait = 0;         // index of kairos-wait in task.actions
  uint32_t wait_long = 0;    // and of kairos-wait-long (until a numeric condition changes)
  uint32_t wait_epsilon = UINT32_MAX;  // and of kairos-wait-epsilon (timed tasks only)
  uint32_t first_event = 0;  // the events and timed literals: [first_event, task.actions.size())
  double delta = 0.1;
  uint32_t first_start = 0;  // the start and end of durative action i: first_start + 2i (+1)
  uint32_t durative = 0;
  std::vector<sas::VarId> running;      // per durative action: 0 while it runs
  std::vector<sas::Condition> over_all;  // per durative action
  // Global and over all conditions as accept checks them (the validator's tolerance)
  sas::Condition always;
  std::vector<sas::Condition> loose_over_all;
  std::optional<sas::NumVarId> time;    // the time, with durative actions or timed literals
  std::string unsupported;              // why this task cannot be planned (empty if it can)
  // For the heuristics (same actions): each end also needs its over all conditions projected
  // to its last moment with its own rates
  sas::Task guide;
  bool has_guide = false;

  bool is_event(uint32_t action) const {return action >= first_event;}

  /// @brief Whether a state reached keeps the global constraints and the over all conditions
  /// of the actions that run (that events go first, the state space sees to:
  /// task.first_event).
  bool accept(const uint32_t * child_values, const double * child_numbers) const;

  /// @brief The plan of a path from the initial state: the domain's actions at their times
  /// (waits add their length, events are implicit), each durative action with its duration,
  /// ending where the path ends.
  Plan plan(const std::vector<uint32_t> & actions, state::StateSpace & space) const;
};

/// @brief How time may pass: short steps only, short and long, or long with short steps only
/// to cross a change (no waiting to arbitrary instants: fewer states, some plans lost).
enum class Waits : uint8_t {Short, Both, Crossing};

/// @brief The discrete task of a PDDL+ task (processes and events moved into actions); with
/// durative actions or timed literals, kairos-wait-epsilon lets `epsilon` pass (to separate
/// happenings that interfere); with `symmetries`, the objects it does not tell apart.
Hybrid hybrid(
  const sas::Task & task, double delta, Waits waits = Waits::Both, double epsilon = 0.001,
  bool symmetries = true);

}  // namespace kairos::temporal

#endif  // KAIROS__TEMPORAL__HYBRID_HPP_
