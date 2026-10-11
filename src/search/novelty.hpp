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
/// \brief Novelty of states (width-based search), partitioned and with numeric features.

#ifndef KAIROS__SEARCH__NOVELTY_HPP_
#define KAIROS__SEARCH__NOVELTY_HPP_

#include <cstddef>
#include <cstdint>
#include <vector>

#include "core/budget.hpp"

namespace kairos::search
{

/**
 * @brief Novelty w(s) of each evaluated state within its partition (Lipovetzky and Geffner):
 * 1 if one of its features was never seen in the partition, 2 if a pair of them was never
 * seen, 3 otherwise. Features: a value per variable, and the value of each numeric variable
 * (as Panino). The partition is a key the caller chooses, such as the heuristic value.
 *
 * Seen tuples live in a flat hash set of 64-bit fingerprints. Pairs grow quadratically with the
 * features, so they are tracked only while the set stays under its memory cap; past it, width
 * 2 is no longer distinguished from 3 (degrades to w = 1, never fails).
 */
class Novelty
{
public:
  Novelty(
    size_t variables, size_t numeric, int max_width, size_t memory_cap_bytes,
    Budget * budget = nullptr);
  ~Novelty();
  Novelty(const Novelty &) = delete;
  Novelty & operator=(const Novelty &) = delete;

  /// @brief Novelty of a state in a partition; records its tuples. With the parent's values and
  /// more than kIncremental features, pairs are those with a feature that changed (as BFWS).
  int evaluate(
    uint64_t partition, const uint32_t * values, const double * numbers,
    const uint32_t * parent_values = nullptr, const double * parent_numbers = nullptr);

  static constexpr size_t kIncremental = 128;

  int max_width() const {return max_width_;}
  bool pairs_dropped() const {return pairs_dropped_;}
  size_t tuples() const {return size_;}

private:
  bool insert(uint64_t key);  // true if new
  void grow();

  size_t variables_;
  size_t numeric_;
  int max_width_;
  size_t cap_;
  Budget * budget_;
  bool pairs_dropped_ = false;
  std::vector<uint64_t> table_;  // 0 = empty
  size_t size_ = 0;
  size_t accounted_ = 0;
  std::vector<uint64_t> features_;
  std::vector<size_t> changed_;
};

}  // namespace kairos::search

#endif  // KAIROS__SEARCH__NOVELTY_HPP_
