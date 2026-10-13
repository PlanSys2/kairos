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
/// \brief Engine M2: durative actions compressed into instantaneous ones, and the sequential
/// plans of the compressed task turned into temporal plans.

#ifndef KAIROS__TEMPORAL__COMPRESS_HPP_
#define KAIROS__TEMPORAL__COMPRESS_HPP_

#include <cstdint>
#include <functional>
#include <optional>
#include <vector>

#include "kairos/plan.hpp"
#include "sas/task.hpp"
#include "state/successors.hpp"

namespace kairos::temporal
{

/**
 * @brief The compressed task: every durative action becomes an instantaneous action appended
 * after the original instantaneous ones.
 *
 * - precondition: the at start conditions, plus the over all and at end ones its own start
 *   effects do not give;
 * - effects: those at start and then those at end (the end wins on a variable both set);
 *   effect conditions are read before the action;
 * - ?duration: replaced by the duration (an equality, or else the lower bound, or the upper).
 *
 * Sound for temporally simple tasks (temporal::analyze); the validator checks every plan.
 */
struct Compressed
{
  sas::Task task;
  size_t instant = 0;                     // task.actions[0, instant): the original instant ones
  std::vector<sas::NumericExpr> duration;  // per original durative action
};

Compressed compress(const sas::Task & task);

/**
 * @brief A temporal plan from a plan of the compressed task: replays it for each duration, and
 * orders happenings (the start and end of each step), as POPF: each goes `epsilon` after the
 * last one that changed what it reads or changes, and after those that read what it changes
 * (over all conditions are read at start and at end). The rest overlap; the earliest schedule
 * of that partial order gives the least makespan for it. `actions` must be a plan of the
 * compressed task (as the search gives them: preconditions are not checked again); nullopt if
 * a step cannot be replayed or scheduled.
 *
 * With `prefix_valid` (whether the steps placed so far form a valid plan, goal aside): resources
 * (numeric variables only increased or decreased by constants, as capacity borrowed at start
 * and given back at end) do not order happenings by themselves. Each step that touches one is
 * placed without those orderings when the plan so far stays valid, and with them otherwise:
 * steps overlap as far as the capacities allow (umts: a makespan of 922 became ~530).
 */
std::optional<Plan> schedule(
  const sas::Task & original, const Compressed & compressed, state::StateSpace & space,
  const std::vector<uint32_t> & actions, double epsilon,
  const std::function<bool(const Plan &)> & prefix_valid = {});

/**
 * @brief A schedule of the partial plans of a search on the compressed task, kept per state:
 * when each variable was last written and last read. The end of a step after a state, with
 * steps starting when what they read or write was last written and what they write was last
 * read, picks among the variants of a preferred step the one that ends first (in the factory of
 * PlanSys2's bt_example, a robot that is free, not the one that did everything). Ordering the
 * open lists by it too lost storage instances in B1 (FIFO ties work better there).
 * Off for tasks with many variables, and once it holds too many states.
 */
class PartialSchedule
{
public:
  PartialSchedule(const Compressed & compressed, state::StateSpace & space);
  bool active() const {return active_;}
  /// @brief The end of `action` after `parent` (0 if the parent is not kept).
  double end(state::StateId parent, uint32_t action) const;
  /// @brief Keeps the schedule of `child`, reached from `parent` (kNoState: the initial state).
  void reach(state::StateId parent, uint32_t action, state::StateId child);

private:
  double duration(state::StateId parent, uint32_t action) const;
  double start(const float * times, uint32_t action) const;

  const Compressed & c_;
  state::StateSpace & space_;
  bool active_ = false;
  size_t width_ = 0;  // per state: write and read times of each fact and numeric variable
  std::vector<std::vector<uint32_t>> reads_, writes_;  // per action: slots
  std::vector<float> times_;
  std::vector<int64_t> slot_;  // per state: offset in times_, or -1
};

}  // namespace kairos::temporal

#endif  // KAIROS__TEMPORAL__COMPRESS_HPP_
