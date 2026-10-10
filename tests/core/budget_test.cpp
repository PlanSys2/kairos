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

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cstddef>
#include <optional>
#include <stdexcept>
#include <stop_token>  // NOLINT(build/include_order): cpplint does not know this header
#include <thread>
#include <vector>

#include "core/budget.hpp"

namespace kairos
{
namespace
{

using Clock = Budget::Clock;
using std::chrono::seconds;

// Manually advanced clock that counts how often it is read
struct FakeClock
{
  Clock::time_point now{};
  int reads = 0;
};

BudgetConfig with_clock(FakeClock & fake)
{
  BudgetConfig config;
  config.clock = [&fake]() {
      ++fake.reads;
      return fake.now;
    };
  return config;
}

TEST(BudgetTest, NoLimitsNeverExhausts)
{
  Budget budget;
  for (int i = 0; i < 100000; ++i) {
    ASSERT_FALSE(budget.exhausted());
  }
  EXPECT_EQ(budget.check(), BudgetState::Ok);
  EXPECT_FALSE(budget.remaining_time().has_value());
  EXPECT_EQ(budget.memory_limit(), 0u);
}

TEST(BudgetTest, StopRequestIsSeenImmediately)
{
  std::stop_source source;
  BudgetConfig config;
  config.stop = source.get_token();
  config.check_interval = 1000000;  // cancellation must not wait for a full check
  Budget budget(config);
  EXPECT_FALSE(budget.exhausted());
  source.request_stop();
  EXPECT_TRUE(budget.exhausted());
  EXPECT_EQ(budget.state(), BudgetState::Cancelled);
}

TEST(BudgetTest, DeadlineWithFakeClock)
{
  FakeClock fake;
  auto config = with_clock(fake);
  config.deadline = Clock::time_point{} + seconds(10);
  config.check_interval = 1;
  Budget budget(config);

  fake.now = Clock::time_point{} + seconds(9);
  EXPECT_FALSE(budget.exhausted());
  EXPECT_EQ(budget.remaining_time(), seconds(1));

  fake.now = Clock::time_point{} + seconds(10);  // reaching the deadline is a timeout
  EXPECT_TRUE(budget.exhausted());
  EXPECT_EQ(budget.state(), BudgetState::Timeout);
  EXPECT_EQ(budget.remaining_time(), Clock::duration::zero());
}

TEST(BudgetTest, ClockIsOnlyReadEveryCheckInterval)
{
  FakeClock fake;
  auto config = with_clock(fake);
  config.deadline = Clock::time_point{} + seconds(10);
  config.check_interval = 100;
  Budget budget(config);
  fake.reads = 0;

  for (int i = 0; i < 99; ++i) {
    EXPECT_FALSE(budget.exhausted());
  }
  EXPECT_EQ(fake.reads, 0);
  EXPECT_FALSE(budget.exhausted());  // call 100: full check
  EXPECT_EQ(fake.reads, 1);

  // The deadline passes, but it is only noticed at the next full check
  fake.now = Clock::time_point{} + seconds(20);
  for (int i = 0; i < 99; ++i) {
    EXPECT_FALSE(budget.exhausted());
  }
  EXPECT_TRUE(budget.exhausted());
  EXPECT_EQ(budget.state(), BudgetState::Timeout);
}

TEST(BudgetTest, ZeroCheckIntervalMeansEveryCall)
{
  FakeClock fake;
  auto config = with_clock(fake);
  config.deadline = Clock::time_point{} + seconds(1);
  config.check_interval = 0;
  Budget budget(config);
  fake.now = Clock::time_point{} + seconds(2);
  EXPECT_TRUE(budget.exhausted());
}

TEST(BudgetTest, CheckForcesAFullCheck)
{
  FakeClock fake;
  auto config = with_clock(fake);
  config.deadline = Clock::time_point{} + seconds(1);
  config.check_interval = 1000000;
  Budget budget(config);
  fake.now = Clock::time_point{} + seconds(2);
  EXPECT_EQ(budget.check(), BudgetState::Timeout);
  EXPECT_TRUE(budget.exhausted());
}

TEST(BudgetTest, FirstReasonWinsAndIsSticky)
{
  std::stop_source source;
  FakeClock fake;
  auto config = with_clock(fake);
  config.stop = source.get_token();
  config.deadline = Clock::time_point{} + seconds(1);
  config.check_interval = 1;
  Budget budget(config);

  fake.now = Clock::time_point{} + seconds(5);
  EXPECT_TRUE(budget.exhausted());
  EXPECT_EQ(budget.state(), BudgetState::Timeout);
  source.request_stop();
  EXPECT_TRUE(budget.exhausted());
  EXPECT_EQ(budget.state(), BudgetState::Timeout);
  fake.now = Clock::time_point{};  // even if time went "back", it stays exhausted
  EXPECT_EQ(budget.check(), BudgetState::Timeout);
}

TEST(BudgetTest, AccountedMemoryOverTheLimit)
{
  BudgetConfig config;
  config.memory_limit_bytes = 1000;
  config.memory_probe = []() {return std::optional<size_t>{};};  // no RSS info
  config.check_interval = 1;
  Budget budget(config);

  budget.account(500);
  EXPECT_FALSE(budget.exhausted());
  EXPECT_EQ(budget.memory_used(), 500u);
  EXPECT_FALSE(budget.memory_pressure());

  budget.account(300);  // 800 = 80 %: warning, not yet a stop
  EXPECT_FALSE(budget.exhausted());
  EXPECT_TRUE(budget.memory_pressure());

  budget.account(-100);  // freeing memory clears the warning
  EXPECT_FALSE(budget.exhausted());
  EXPECT_FALSE(budget.memory_pressure());

  budget.account(300);  // 1000 = limit
  EXPECT_TRUE(budget.exhausted());
  EXPECT_EQ(budget.state(), BudgetState::OutOfMemory);
  EXPECT_EQ(budget.accounted_bytes(), 1000u);
}

TEST(BudgetTest, NegativeAccountingClampsToZero)
{
  BudgetConfig config;
  config.memory_limit_bytes = 1000;
  config.memory_probe = []() {return std::optional<size_t>{};};
  Budget budget(config);
  budget.account(-50);
  EXPECT_EQ(budget.accounted_bytes(), 0u);
  EXPECT_EQ(budget.check(), BudgetState::Ok);
}

TEST(BudgetTest, RssGrowthCountsFromTheBaseline)
{
  size_t rss = 10000;  // the host process already uses 10000 bytes
  BudgetConfig config;
  config.memory_limit_bytes = 1000;
  config.memory_probe = [&rss]() {return std::optional<size_t>{rss};};
  config.check_interval = 1;
  Budget budget(config);

  EXPECT_FALSE(budget.exhausted());
  EXPECT_EQ(budget.memory_used(), 0u);

  rss = 10900;  // grew 900: pressure
  EXPECT_FALSE(budget.exhausted());
  EXPECT_EQ(budget.memory_used(), 900u);
  EXPECT_TRUE(budget.memory_pressure());

  rss = 9000;  // below the baseline counts as no growth
  EXPECT_FALSE(budget.exhausted());
  EXPECT_EQ(budget.memory_used(), 0u);

  rss = 11000;
  EXPECT_TRUE(budget.exhausted());
  EXPECT_EQ(budget.state(), BudgetState::OutOfMemory);
}

TEST(BudgetTest, MemoryUsedIsTheLargerOfAccountedAndRssGrowth)
{
  size_t rss = 5000;
  BudgetConfig config;
  config.memory_limit_bytes = 10000;
  config.memory_probe = [&rss]() {return std::optional<size_t>{rss};};
  Budget budget(config);

  budget.account(3000);
  rss = 6000;  // growth 1000 < accounted 3000
  budget.check();
  EXPECT_EQ(budget.memory_used(), 3000u);

  rss = 9000;  // growth 4000 > accounted 3000
  budget.check();
  EXPECT_EQ(budget.memory_used(), 4000u);
}

TEST(BudgetTest, NoMemoryLimitIgnoresMemory)
{
  int probes = 0;
  BudgetConfig config;
  config.memory_probe = [&probes]() {
      ++probes;
      return std::optional<size_t>{1};
    };
  config.check_interval = 1;
  Budget budget(config);
  budget.account(1 << 30);
  EXPECT_FALSE(budget.exhausted());
  EXPECT_EQ(probes, 0);
  EXPECT_FALSE(budget.memory_pressure());
}

TEST(BudgetTest, CustomWarningFractionIsClamped)
{
  BudgetConfig config;
  config.memory_limit_bytes = 100;
  config.memory_warning_fraction = 5.0;  // clamped to 1.0
  config.memory_probe = []() {return std::optional<size_t>{};};
  Budget budget(config);
  budget.account(99);
  budget.check();
  EXPECT_FALSE(budget.memory_pressure());

  BudgetConfig low;
  low.memory_limit_bytes = 100;
  low.memory_warning_fraction = 0.5;
  low.memory_probe = []() {return std::optional<size_t>{};};
  Budget early(low);
  early.account(50);
  early.check();
  EXPECT_TRUE(early.memory_pressure());
}

TEST(BudgetTest, ThrowingClockDoesNotEscape)
{
  BudgetConfig config;
  config.deadline = Clock::now() + seconds(100);
  config.clock = []() -> Clock::time_point {throw std::runtime_error("broken clock");};
  config.check_interval = 1;
  Budget budget(config);
  EXPECT_NO_THROW(budget.exhausted());
  EXPECT_EQ(budget.state(), BudgetState::Ok);
}

TEST(BudgetTest, RealClockDeadline)
{
  BudgetConfig config;
  config.deadline = Clock::now() + std::chrono::milliseconds(20);
  config.check_interval = 1;
  Budget budget(config);
  auto start = Clock::now();
  while (!budget.exhausted()) {
    ASSERT_LT(Clock::now() - start, seconds(5)) << "deadline never reached";
  }
  EXPECT_EQ(budget.state(), BudgetState::Timeout);
}

TEST(BudgetTest, ConcurrentPollingAndAccounting)
{
  std::stop_source source;
  BudgetConfig config;
  config.stop = source.get_token();
  config.memory_limit_bytes = size_t{1} << 40;
  config.memory_probe = []() {return std::optional<size_t>{};};
  config.check_interval = 7;
  Budget budget(config);

  std::atomic<int> running{0};
  std::vector<std::thread> threads;
  for (int t = 0; t < 4; ++t) {
    threads.emplace_back([&]() {
        ++running;
        while (!budget.exhausted()) {
          budget.account(16);
          budget.account(-16);
        }
      });
  }
  while (running.load() < 4) {
    std::this_thread::yield();
  }
  source.request_stop();
  for (auto & t : threads) {
    t.join();
  }
  EXPECT_EQ(budget.state(), BudgetState::Cancelled);
  EXPECT_EQ(budget.accounted_bytes(), 0u);
}

}  // namespace
}  // namespace kairos
