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
/// \brief Budget: cancellation, deadline and memory limit, cheap to poll in hot loops.

#ifndef KAIROS__CORE__BUDGET_HPP_
#define KAIROS__CORE__BUDGET_HPP_

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <optional>
#include <stop_token>  // NOLINT(build/include_order): cpplint does not know this header

namespace kairos
{

enum class BudgetState : uint8_t
{
  Ok,
  Cancelled,
  Timeout,
  OutOfMemory
};

struct BudgetConfig
{
  using Clock = std::chrono::steady_clock;

  std::stop_token stop;
  std::optional<Clock::time_point> deadline;
  size_t memory_limit_bytes = 0;      // 0 = no memory limit
  double memory_warning_fraction = 0.8;
  uint32_t check_interval = 1024;     // exhausted() calls between full checks; 0 is taken as 1
  // Also a full check when this much real time passed since the last one, however rarely
  // exhausted() is called (zero: only by calls)
  std::chrono::microseconds check_period{5000};
  std::function<Clock::time_point()> clock;           // default: Clock::now
  std::function<std::optional<size_t>()> memory_probe;  // default: current_rss_bytes
};

/**
 * @brief Tells long-running stages when to stop.
 *
 * - exhausted() is the hot-loop call: it reads the stop token and a coarse clock every time,
 *   and the clock and memory only every `check_interval` calls or `check_period`, whichever
 *   comes first (a search whose expansions take 30 ms still sees its deadline in time).
 * - Memory used = max(bytes accounted with account(), RSS growth since construction). RSS
 *   growth, not total RSS, because Kairos may live inside a larger process.
 * - The first reason to stop wins and is sticky.
 * - Thread-safe: several threads may poll and account on the same Budget.
 */
class Budget
{
public:
  using Clock = BudgetConfig::Clock;

  explicit Budget(BudgetConfig config = {});

  Budget(const Budget &) = delete;
  Budget & operator=(const Budget &) = delete;

  /// @brief True once any limit is hit. Cheap; call it often.
  bool exhausted() noexcept;

  /// @brief Full check of all limits right now.
  BudgetState check() noexcept;

  BudgetState state() const noexcept {return state_.load(std::memory_order_acquire);}

  /// @brief True if memory use reached the warning fraction at the last full check.
  bool memory_pressure() const noexcept {return pressure_.load(std::memory_order_relaxed);}

  /// @brief Adds (or with a negative delta, removes) bytes owned by Kairos structures.
  void account(std::ptrdiff_t delta_bytes) noexcept;

  size_t accounted_bytes() const noexcept;

  /// @brief Memory used as of the last full check.
  size_t memory_used() const noexcept {return used_.load(std::memory_order_relaxed);}

  size_t memory_limit() const noexcept {return config_.memory_limit_bytes;}

  /// @brief Time left until the deadline (zero if passed), or nullopt without deadline.
  std::optional<Clock::duration> remaining_time() const;

private:
  void stop_with(BudgetState reason) noexcept;
  Clock::time_point now() const;
  std::optional<size_t> probe() const;

  BudgetConfig config_;
  std::optional<size_t> rss_baseline_;
  std::atomic<BudgetState> state_{BudgetState::Ok};
  std::atomic<int64_t> accounted_{0};
  std::atomic<size_t> used_{0};
  std::atomic<bool> pressure_{false};
  std::atomic<uint32_t> calls_{0};
  std::atomic<int64_t> last_check_ns_{0};  // coarse clock, at the last full check
};

}  // namespace kairos

#endif  // KAIROS__CORE__BUDGET_HPP_
