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
/// \brief Engine M3: durative actions as start and end snap actions, searched forward with a
/// Simple Temporal Network over the partial-order plan (as POPF).

#ifndef KAIROS__TEMPORAL__SNAP_HPP_
#define KAIROS__TEMPORAL__SNAP_HPP_

#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "kairos/plan.hpp"
#include "sas/task.hpp"
#include "state/successors.hpp"
#include "temporal/access.hpp"
#include "temporal/stn.hpp"

namespace kairos::temporal
{

/**
 * @brief The snap task: the original instantaneous actions, then a start and an end action per
 * durative action, then one action per timed initial literal (in order).
 *
 * - start: the at start conditions and "not running"; its start effects and "running";
 * - end: the at end conditions and "running"; its end effects and "not running";
 * - a timed initial literal: the ones before it happened; its effects;
 * - goal: the original goal and nothing running.
 *
 * Over all conditions are not in the task: the TemporalFilter checks them while the action
 * runs, and the STN checks durations, epsilon separations and the times of the timed initial
 * literals.
 */
struct SnapTask
{
  sas::Task task;
  size_t instant = 0;                // task.actions[0, instant): the original instant ones
  size_t tils = 0;                   // task.actions[tils, ...): timed initial literals
  std::vector<sas::VarId> running;   // per durative action: 0 running, 1 not
  std::vector<double> til_time;      // per timed initial literal
  /// @brief Why M3 cannot plan this task yet (empty if it can).
  std::string unsupported;

  bool is_start(uint32_t a) const {return a >= instant && a < tils && (a - instant) % 2 == 0;}
  bool is_end(uint32_t a) const {return a >= instant && a < tils && (a - instant) % 2 == 1;}
  size_t durative(uint32_t a) const {return (a - instant) / 2;}
};

SnapTask snap(const sas::Task & task);

/// @brief Duration bounds of a durative action in a state (nullopt: no bound on that side).
struct DurationBounds
{
  double lo = 0.0;
  double hi = Stn::kInfinity;
};
DurationBounds bounds(const sas::DurativeAction & a, const double * numbers);

/**
 * @brief Decides whether the search may take a transition of the snap task: the over all
 * conditions of the running actions hold in the new state, and the plan so far plus the new
 * happening has a consistent STN (durations, epsilon between happenings that interfere, the
 * times of the timed initial literals). Keeps, per state, the first plan that reached it.
 */
class TemporalFilter
{
public:
  TemporalFilter(
    const sas::Task & original, const SnapTask & snap, state::StateSpace & space, double epsilon);
  ~TemporalFilter();
  TemporalFilter(const TemporalFilter &) = delete;
  TemporalFilter & operator=(const TemporalFilter &) = delete;

  bool accept(state::StateId parent, uint32_t action, state::StateId child);

  /// @brief The makespan of the plan of `parent` plus a transition accepted while expanding it
  /// (pending ends included); `fallback` if unknown.
  double makespan(
    state::StateId parent, uint32_t action, state::StateId child, double fallback) const;

  /// @brief The temporal plan of a sequence of snap actions (nullopt if inconsistent).
  std::optional<Plan> plan(const std::vector<uint32_t> & actions);

private:
  struct Node
  {
    int32_t parent;
    uint32_t action;
    DurationBounds duration;  // starts: from the state where they start
  };
  struct Builder;

  bool invariants(state::StateId s);
  void rebuild(state::StateId parent);
  bool add(
    Builder & b, uint32_t action, const DurationBounds & duration, Stn::Node & node,
    bool record) const;
  int32_t node_of(state::StateId s) const;
  std::optional<double> next_til(size_t from, const Access & acc) const;
  bool breaks(size_t r, size_t a) const;

  const sas::Task & original_;
  const SnapTask & snap_;
  state::StateSpace & space_;
  double epsilon_;
  std::vector<Access> access_;    // per snap action
  // What the end of each durative action writes: facts (a value, or nullopt when unknown or
  // conditional) and numeric variables
  struct EndWrites
  {
    std::map<sas::VarId, std::optional<uint32_t>> facts;
    std::set<sas::NumVarId> nums;
  };
  std::vector<EndWrites> end_writes_;
  // Timed initial literals (indices, in order) that change or read each variable
  std::map<sas::VarId, std::vector<uint32_t>> til_writes_, til_reads_;
  std::map<sas::NumVarId, std::vector<uint32_t>> til_num_writes_;
  std::vector<int32_t> node_of_;  // per state: the last node of its plan (-1 none, -2 unknown)
  std::vector<Node> nodes_;
  std::unique_ptr<Builder> builder_;  // the STN of the plan of the parent being expanded
  state::StateId cached_ = state::kNoState;
  std::map<std::pair<uint32_t, state::StateId>, double> makespans_;  // of the cached parent
};

}  // namespace kairos::temporal

#endif  // KAIROS__TEMPORAL__SNAP_HPP_
