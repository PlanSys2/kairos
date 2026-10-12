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

#include "core/budget.hpp"

#include <time.h>

#include <algorithm>
#include <utility>

#include "core/memory_info.hpp"

namespace kairos
{

namespace
{

// Nanoseconds of a monotonic clock that is cheap to read in hot loops (a few ms coarse on Linux)
int64_t coarse_ns() noexcept
{
#ifdef CLOCK_MONOTONIC_COARSE
  timespec ts;
  clock_gettime(CLOCK_MONOTONIC_COARSE, &ts);
  return int64_t{ts.tv_sec} *1'000'000'000 + ts.tv_nsec;
#else
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
#endif
}

}  // namespace

Budget::Budget(BudgetConfig config)
: config_(std::move(config))
{
  if (config_.check_interval == 0) {
    config_.check_interval = 1;
  }
  config_.memory_warning_fraction = std::clamp(config_.memory_warning_fraction, 0.0, 1.0);
  last_check_ns_.store(coarse_ns(), std::memory_order_relaxed);
  if (config_.memory_limit_bytes > 0) {
    rss_baseline_ = probe();
  }
}

bool Budget::exhausted() noexcept
{
  if (state() != BudgetState::Ok) {
    return true;
  }
  if (config_.stop.stop_requested()) {
    stop_with(BudgetState::Cancelled);
    return true;
  }
  uint32_t calls = calls_.fetch_add(1, std::memory_order_relaxed) + 1;
  if (calls % config_.check_interval != 0) {
    int64_t period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      config_.check_period).count();
    if (period <= 0 || coarse_ns() - last_check_ns_.load(std::memory_order_relaxed) < period) {
      return false;
    }
  }
  return check() != BudgetState::Ok;
}

BudgetState Budget::check() noexcept
{
  last_check_ns_.store(coarse_ns(), std::memory_order_relaxed);
  if (state() != BudgetState::Ok) {
    return state();
  }
  if (config_.stop.stop_requested()) {
    stop_with(BudgetState::Cancelled);
    return state();
  }
  try {
    if (config_.deadline && now() >= *config_.deadline) {
      stop_with(BudgetState::Timeout);
      return state();
    }
    if (config_.memory_limit_bytes > 0) {
      size_t used = accounted_bytes();
      if (rss_baseline_) {
        if (auto rss = probe(); rss && *rss > *rss_baseline_) {
          used = std::max(used, *rss - *rss_baseline_);
        }
      }
      used_.store(used, std::memory_order_relaxed);
      const double warning =
        config_.memory_warning_fraction * static_cast<double>(config_.memory_limit_bytes);
      pressure_.store(static_cast<double>(used) >= warning, std::memory_order_relaxed);
      if (used >= config_.memory_limit_bytes) {
        stop_with(BudgetState::OutOfMemory);
      }
    }
  } catch (...) {
    // A user-provided clock or probe threw: keep the current state
  }
  return state();
}

bool Budget::reserve(size_t bytes) noexcept
{
  if (check() != BudgetState::Ok) {
    return false;
  }
  if (config_.memory_limit_bytes > 0 && used_.load(std::memory_order_relaxed) + bytes >
    config_.memory_limit_bytes)
  {
    stop_with(BudgetState::OutOfMemory);
    return false;
  }
  return true;
}

void Budget::account(std::ptrdiff_t delta_bytes) noexcept
{
  accounted_.fetch_add(static_cast<int64_t>(delta_bytes), std::memory_order_relaxed);
}

size_t Budget::accounted_bytes() const noexcept
{
  int64_t value = accounted_.load(std::memory_order_relaxed);
  return value > 0 ? static_cast<size_t>(value) : 0;
}

std::optional<Budget::Clock::duration> Budget::remaining_time() const
{
  if (!config_.deadline) {
    return std::nullopt;
  }
  auto left = *config_.deadline - now();
  return left > Clock::duration::zero() ? left : Clock::duration::zero();
}

void Budget::stop_with(BudgetState reason) noexcept
{
  BudgetState expected = BudgetState::Ok;
  state_.compare_exchange_strong(expected, reason, std::memory_order_acq_rel);
}

Budget::Clock::time_point Budget::now() const
{
  return config_.clock ? config_.clock() : Clock::now();
}

std::optional<size_t> Budget::probe() const
{
  return config_.memory_probe ? config_.memory_probe() : current_rss_bytes();
}

}  // namespace kairos
