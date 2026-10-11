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
/// \brief Search engines (GBFS lazy and eager, WA*, EHC) and the anytime driver.

#ifndef KAIROS__SEARCH__SEARCH_HPP_
#define KAIROS__SEARCH__SEARCH_HPP_

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "core/budget.hpp"
#include "core/logger.hpp"
#include "kairos/plan.hpp"
#include "sas/task.hpp"
#include "state/registry.hpp"
#include "state/successors.hpp"

namespace kairos::search
{

enum class Outcome : uint8_t
{
  Solved,          // found a plan (better than the bound, if any)
  Unsolvable,      // exhausted the reachable space: no plan (better than the bound)
  Failed,          // an incomplete method gave up (EHC in a dead end)
  Stopped,         // budget: time, memory or cancellation
};

struct SearchStats
{
  size_t expanded = 0;
  size_t generated = 0;
  size_t evaluated = 0;
  size_t dead_ends = 0;
  size_t reopened = 0;
};

/**
 * @brief Receives the plans the engines find (action indices of task.actions and their cost)
 * and holds the bound: the cost of the best plan so far. Thread-safe.
 */
class PlanSink
{
public:
  struct Entry
  {
    std::vector<uint32_t> actions;
    double cost;
    std::chrono::steady_clock::duration elapsed;
  };

  explicit PlanSink(std::function<void(const Entry &)> on_plan = {});

  /// @brief Keeps the plan if strictly cheaper than the best one; true if kept.
  bool offer(std::vector<uint32_t> actions, double cost);
  /// @brief Cost of the best plan (infinity if none).
  double bound() const {return bound_.load(std::memory_order_acquire);}
  std::optional<Entry> best() const;
  std::vector<Entry> all() const;

private:
  std::function<void(const Entry &)> on_plan_;
  std::chrono::steady_clock::time_point start_;
  mutable std::mutex mutex_;
  std::vector<Entry> plans_;
  std::atomic<double> bound_{std::numeric_limits<double>::infinity()};
};

/// @brief What engines share: the task, its states, the budget, the costs and the sink.
struct SearchContext
{
  SearchContext(
    const sas::Task & t, state::StateRegistry & r, state::StateSpace & s,
    const std::vector<double> & c, PlanSink & k, Budget * b = nullptr)
  : task(t), registry(r), space(s), costs(c), sink(k), budget(b) {}

  const sas::Task & task;
  state::StateRegistry & registry;
  state::StateSpace & space;
  const std::vector<double> & costs;  // per action of task.actions
  PlanSink & sink;
  Budget * budget = nullptr;
  Logger logger;
  /// @brief Prune nodes whose g already reaches the bound: only when no action can lower the
  /// plan cost (costs are never negative)
  bool prune_with_bound = false;
  /// @brief Optional: whether a transition may be taken (the temporal engine checks over all
  /// conditions and the temporal network); a rejected one counts as an invalid step
  std::function<bool(state::StateId parent, uint32_t action, state::StateId child)> accept;
  SearchStats stats;
};

struct EngineOptions
{
  enum class Kind : uint8_t {LazyGreedy, EagerGreedy, WeightedAStar, EnforcedHillClimbing};

  Kind kind = Kind::LazyGreedy;
  /// @brief One heuristic, or several separated by commas ("ff,lmcount"): each gets its own
  /// queues in the alternation, and preferred operators are the union.
  std::string heuristic = "ff";
  double weight = 1.0;          // WA*: f = g + w h
  bool preferred = true;        // preferred-operator queue (lazy, eager) or helpful actions (EHC)
  bool reopen = true;           // eager: reopen closed states reached more cheaply
  int boost = 1000;             // preferred queue boost on progress, as FD
  /// @brief Eager engines: a further queue ordered by novelty (1 or 2: the width tracked)
  /// within the partition of the heuristic values, then by the first heuristic; 0: none.
  int novelty = 0;
  size_t novelty_memory = size_t{64} << 20;  // pairs stop being tracked past this
};

/// @brief Runs one engine until a plan, exhaustion, failure or the budget.
Outcome run_engine(SearchContext & ctx, const EngineOptions & options);

struct AnytimeOptions
{
  EngineOptions first;                    // the first plan
  bool fallback_after_ehc = true;         // if first is EHC and fails: lazy GBFS
  bool improve = true;                    // keep improving after the first plan
  // Restarts with WA* of these weights (the last repeats while pruning by the bound)
  std::vector<double> weights = {5.0, 3.0, 2.0, 1.0};
  std::string improve_heuristic = "ff";
};

enum class AnytimeResult : uint8_t
{
  ProvenOptimal,   // the last search exhausted every cheaper plan
  Exhausted,       // no plan at all: unsolvable
  NoMoreImprovement,  // improving searches stopped without proof (pruning not allowed)
  Stopped,         // budget
  Failed,          // no plan and no proof (incomplete first search only)
};

/**
 * @brief The anytime driver, LAMA style: a first plan with the first engine, then restarts
 * with WA* of decreasing weight, each needing a strictly cheaper plan. With bound pruning, a
 * restart that exhausts the space proves the best plan optimal.
 */
AnytimeResult anytime(SearchContext & ctx, const AnytimeOptions & options);

/// @brief A sequential plan from action indices of the task.
Plan to_plan(const sas::Task & task, const std::vector<uint32_t> & actions);

}  // namespace kairos::search

#endif  // KAIROS__SEARCH__SEARCH_HPP_
