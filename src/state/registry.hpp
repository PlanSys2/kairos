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
/// \brief Registry of packed states: each state stored once, named by a 4-byte id.

#ifndef KAIROS__STATE__REGISTRY_HPP_
#define KAIROS__STATE__REGISTRY_HPP_

#include <span>  // cpplint takes it for a C header

#include <cstddef>
#include <cstdint>
#include <memory>
#include <utility>
#include <vector>

#include "core/budget.hpp"
#include "sas/task.hpp"
#include "state/eval.hpp"
#include "state/packing.hpp"

namespace kairos::state
{

using StateId = uint32_t;
inline constexpr StateId kNoState = UINT32_MAX;

/**
 * @brief Stores states packed (variables in the fewest bits, numbers as doubles) in blocks
 * that never move: growing never copies the states, so it never needs twice the memory. An
 * open-addressing table maps states to ids. Derived variables are computed when a state is
 * registered. The memory is accounted in the budget.
 *
 * Numbers are compared bit by bit after making -0.0 equal to 0.0 and every NaN equal (all of
 * them mean "undefined").
 */
class StateRegistry
{
public:
  /// @brief With merge_unread, a numeric variable nothing reads (a cost the metric adds up)
  /// keeps its initial value in every state, so it does not tell states apart; false keeps
  /// every value (to look at what effects compute)
  explicit StateRegistry(
    const sas::Task & task, Budget * budget = nullptr, bool merge_unread = true);
  ~StateRegistry();
  StateRegistry(const StateRegistry &) = delete;
  StateRegistry & operator=(const StateRegistry &) = delete;

  const sas::Task & task() const {return task_;}
  const PackedLayout & layout() const {return layout_;}

  /// @brief The initial state (registered on the first call).
  StateId initial();

  /**
   * @brief Registers a state given all variable values (derived ones are recomputed) and its
   * numbers. Returns its id and whether it is new.
   */
  std::pair<StateId, bool> insert(std::vector<uint32_t> & values, std::span<const double> numbers);

  /// @brief All variable values of a state.
  void unpack(StateId id, std::vector<uint32_t> & values) const;
  uint32_t value(StateId id, sas::VarId var) const
  {
    return static_cast<uint32_t>(layout_.get(packed(id), var));
  }
  std::span<const double> numbers(StateId id) const;

  size_t size() const {return count_;}
  /// @brief Bytes held: blocks, table and numbers.
  size_t bytes() const;
  /// @brief Bytes per registered state, the table included.
  double bytes_per_state() const
  {
    return count_ ? static_cast<double>(bytes()) / static_cast<double>(count_) : 0.0;
  }

private:
  static constexpr size_t kBlock = 1 << 14;  // states per block

  const uint64_t * packed(StateId id) const
  {
    return blocks_[id / kBlock].get() + (id % kBlock) * words_;
  }
  const double * nums(StateId id) const
  {
    return num_blocks_[id / kBlock].get() + (id % kBlock) * num_vars_;
  }
  uint64_t hash(const uint64_t * packed, const double * numbers) const;
  bool equal(StateId id, const uint64_t * packed, const double * numbers) const;
  void grow_table();
  void account(size_t bytes);

  const sas::Task & task_;
  Budget * budget_;
  PackedLayout layout_;
  AxiomEvaluator axioms_;
  size_t words_ = 0;
  size_t num_vars_ = 0;
  std::vector<std::unique_ptr<uint64_t[]>> blocks_;
  std::vector<std::unique_ptr<double[]>> num_blocks_;
  std::vector<uint64_t> table_;  // hash bits << 32 | id: fewer state comparisons, one access
  size_t count_ = 0;
  size_t accounted_ = 0;
  StateId initial_ = kNoState;
  std::vector<uint64_t> scratch_;
  std::vector<double> scratch_numbers_;
  std::vector<uint8_t> unread_;  // numeric variables nothing reads: not part of a state
  bool any_unread_ = false;
};

}  // namespace kairos::state

#endif  // KAIROS__STATE__REGISTRY_HPP_
