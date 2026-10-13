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
/// \brief Heuristic interface, action costs and the heuristics by name.

#ifndef KAIROS__HEURISTICS__HEURISTIC_HPP_
#define KAIROS__HEURISTICS__HEURISTIC_HPP_

#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "core/budget.hpp"
#include "sas/task.hpp"
#include "state/registry.hpp"

namespace kairos::heuristics
{

inline constexpr double kInfinity = std::numeric_limits<double>::infinity();

/// @brief A state to evaluate: its values (all variables, derived ones computed) and numbers.
struct EvalContext
{
  state::StateId state = state::kNoState;
  const uint32_t * values = nullptr;
  const double * numbers = nullptr;
  double g = 0.0;
  const std::vector<uint32_t> * applicable = nullptr;  // actions applicable here, if known
};

struct HeuristicResult
{
  double value = 0.0;
  bool dead_end = false;              // no plan from here, even relaxed
  std::vector<uint32_t> preferred;    // actions (task.actions indices), if computed
};

class Heuristic
{
public:
  virtual ~Heuristic() = default;
  virtual HeuristicResult evaluate(const EvalContext & ctx) = 0;
  /// @brief Never above the cost of an optimal plan (with the costs it was built with).
  virtual bool admissible() const {return false;}
  virtual std::string name() const = 0;

  // Path-dependent heuristics (landmarks) learn from the transitions the search takes
  virtual bool path_dependent() const {return false;}
  /// @brief The initial state of a search, before evaluating it.
  virtual void initial(const EvalContext & /*state*/) {}
  /// @brief The search reached `child` from `parent` with `action` (once per transition,
  /// also for states seen before), before evaluating it.
  virtual void transition(
    state::StateId /*parent*/, uint32_t /*action*/, const EvalContext & /*child*/) {}
};

/**
 * @brief Cost of each action of task.actions.
 *
 * - Unit: 1 each (plan length).
 * - Metric: when the metric minimizes a linear expression and every action changes it by a
 *   constant amount that is not negative (action costs, as (increase (total-cost) 3)), that
 *   amount; otherwise 1 each.
 */
enum class CostModel : uint8_t {Unit, Metric};
std::vector<double> action_costs(const sas::Task & task, CostModel model);

/// @brief True if action_costs(task, Metric) uses the metric (not the fallback to 1).
bool metric_costs_apply(const sas::Task & task);

/**
 * @brief Heuristics by name: "blind", "goalcount", "hmax", "hadd", "ff", "lmcount".
 *
 * The relaxed ones work on the delete relaxation of the SAS+ task with numeric subgoaling:
 * an unsatisfied linear condition is reached by an action that moves it the right way, repeated
 * as often as needed. Non-linear conditions and object comparisons count as satisfied
 * (optimistic). Returns nullptr for an unknown name.
 */
std::unique_ptr<Heuristic> make_heuristic(
  const std::string & name, const sas::Task & task, const std::vector<double> & costs,
  Budget * budget = nullptr);

}  // namespace kairos::heuristics

#endif  // KAIROS__HEURISTICS__HEURISTIC_HPP_
