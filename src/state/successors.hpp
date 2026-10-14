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
/// \brief Applicable operators (decision tree) and their effects.

#ifndef KAIROS__STATE__SUCCESSORS_HPP_
#define KAIROS__STATE__SUCCESSORS_HPP_

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

#include "sas/task.hpp"
#include "state/eval.hpp"
#include "state/registry.hpp"

namespace kairos::state
{

/**
 * @brief Finds the operators whose condition holds, as FD's successor generator: a decision
 * tree over the facts of the conditions (each node looks at one variable and follows its
 * value, plus a branch for operators that do not care). The rest of a condition (negated
 * facts, numbers, object comparisons) is checked only on the candidates.
 */
class SuccessorGenerator
{
public:
  /// @brief One condition per operator, in operator order. They must outlive the generator.
  SuccessorGenerator(
    const sas::Task & task,
    const std::vector<const sas::Condition *> & conditions);

  /// @brief Appends the operators whose condition holds.
  void applicable(
    const uint32_t * values, const double * numbers, std::vector<uint32_t> & out,
    double duration = kUndefined) const;

  size_t nodes() const {return nodes_.size();}

private:
  static constexpr uint32_t kNone = UINT32_MAX;

  struct Node
  {
    uint32_t var = kNone;          // kNone: only leaves
    uint32_t children = kNone;     // first of var's domain-size child slots
    uint32_t dont_care = kNone;
    uint32_t leaves_begin = 0;
    uint32_t leaves_end = 0;
  };

  const sas::Task & task_;
  std::vector<const sas::Condition *> conditions_;
  std::vector<bool> extra_;          // per operator: more than facts to check
  std::vector<Node> nodes_;
  std::vector<uint32_t> child_;      // node ids
  std::vector<uint32_t> leaves_;     // operator ids
  mutable std::vector<uint32_t> stack_;
};

/**
 * @brief Applies unconditional changes and the effects with this timing, as the validator does:
 * - conditions are read in the state before (and in `start` for start conditions);
 * - assignments of a variable's last value (deletes) go first, then the others; two different
 *   other values for one variable are a conflict;
 * - numeric effects in order, their values from the state before; changing an undefined fluent,
 *   or a value that is undefined (undefined fluent, division by zero) makes the step invalid.
 * Returns false if the step is invalid.
 */
bool apply_effects(
  const sas::Task & task, const std::vector<sas::Fact> & changes,
  const std::vector<sas::Effect> & effects, sas::Timing timing, const uint32_t * values,
  const double * numbers, std::vector<uint32_t> & out_values, std::vector<double> & out_numbers,
  const uint32_t * start_values = nullptr, const double * start_numbers = nullptr,
  double duration = kUndefined);

/// @brief Instant actions over registered states: applicable actions, successors, goal test.
class StateSpace
{
public:
  StateSpace(const sas::Task & task, StateRegistry & registry);

  StateRegistry & registry() {return registry_;}
  const sas::Task & task() const {return task_;}

  /// @brief Actions applicable in the state (indices into task.actions).
  void applicable(StateId s, std::vector<uint32_t> & out);

  /// @brief The state after the action, registered; nullopt if the step is invalid (effect
  /// conflict, undefined value).
  std::optional<StateId> successor(StateId s, uint32_t action);

  bool is_goal(StateId s);

  /// @brief PDDL+: how long the last time passing successor let pass.
  double last_wait() const {return last_wait_;}

  /// @brief All variable values and numbers of a state (valid until another state is loaded).
  const std::vector<uint32_t> & values(StateId s)
  {
    load(s);
    return values_;
  }
  const std::vector<double> & numbers(StateId s)
  {
    load(s);
    return numbers_;
  }

private:
  void load(StateId s);
  bool integrate(double max_dt, bool long_wait);
  bool holds_numeric(const sas::NumericCondition & c, const double * numbers) const;

  static constexpr double kClockStep = 0.1;  // Runge-Kutta step of PDDL+ time passing
  static constexpr double kDeadlineSlack = 1e-9;  // a deadline this close: reached
  static constexpr int64_t kLongSteps = 20000;  // a long wait: at most this many steps
  double last_wait_ = 0.0;

  const sas::Task & task_;
  StateRegistry & registry_;
  SuccessorGenerator generator_;
  StateId loaded_ = kNoState;
  std::vector<uint32_t> values_, next_values_;
  std::vector<double> numbers_, next_numbers_;
};

}  // namespace kairos::state

#endif  // KAIROS__STATE__SUCCESSORS_HPP_
