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
/// \brief Planning: domains and problems, solve() and anytime searches.

#ifndef KAIROS__PLANNER_HPP_
#define KAIROS__PLANNER_HPP_

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <memory>
#include <optional>
#include <set>
#include <stop_token>  // NOLINT(build/include_order): cpplint does not know this header
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include "kairos/check.hpp"
#include "kairos/diagnostics.hpp"
#include "kairos/export.hpp"
#include "kairos/log.hpp"
#include "kairos/plan.hpp"
#include "kairos/result.hpp"
#include "kairos/validate.hpp"

namespace kairos
{

enum class Status : uint8_t
{
  Solved,             // at least one valid plan
  Unsolvable,         // proved: no plan exists
  Timeout,            // out of time without a plan
  Cancelled,          // stopped by the caller without a plan
  OutOfMemory,        // out of memory without a plan
  Unsupported,        // the task uses something Kairos cannot plan with yet (see diagnostics)
  InvalidInput,       // parse or semantic errors
  GroundingTooLarge,  // grounding does not fit in memory: the lifted path (F5) would be needed
  InternalError,      // a bug: every plan found failed validation, or an exception
};

enum class StopReason : uint8_t
{
  Exhausted,       // the search space was exhausted (no plan, or no better one without proof)
  ProvenOptimal,   // the best plan is optimal
  Timeout,
  NoImprovement,   // stop_after_no_improvement elapsed, or no strategy left to improve
  TargetReached,   // a plan as good as target_cost
  Stopped,         // stop() or the stop token
  MemoryLimit,
};

enum class Engine : uint8_t
{
  Auto,  // chosen from the task
  M1,    // sequential and numeric (no durative actions)
  M2,    // compressed temporal with rescheduling (step 15)
  M3,    // temporal with snap actions (step 15)
};

/// @brief Names of the M1 search presets accepted by SolveOptions::search, besides "auto".
KAIROS_API std::vector<std::string> search_presets();

KAIROS_API std::string_view to_string(Status status);
KAIROS_API std::string_view to_string(StopReason reason);

struct Statistics
{
  double parse_ms = 0.0;      // when the problem was parsed
  double normalize_ms = 0.0;
  double ground_ms = 0.0;
  double sas_ms = 0.0;
  double search_ms = 0.0;
  double total_ms = 0.0;      // the call, without parsing
  std::optional<double> first_plan_ms;
  size_t atoms = 0;
  size_t actions = 0;
  size_t variables = 0;
  size_t mutex_groups = 0;
  size_t expanded = 0;
  size_t generated = 0;
  size_t evaluated = 0;
  size_t states = 0;
  size_t plans = 0;           // plans found (each better than the one before)
  size_t memory_limit_bytes = 0;
  size_t peak_memory_bytes = 0;
  double bytes_per_state = 0.0;
};

class Domain;
class Problem;

/// @brief A parsed and checked domain: immutable and cheap to copy.
class KAIROS_API Domain
{
public:
  const std::string & name() const;
  /// @brief PDDL features the domain uses.
  std::vector<std::string> features() const;

  struct Impl;
  explicit Domain(std::shared_ptr<const Impl> impl);
  const Impl & impl() const {return *impl_;}

private:
  std::shared_ptr<const Impl> impl_;
};

/// @brief A parsed and checked problem, with its domain: immutable, cheap to copy, and usable
/// from several threads at once.
class KAIROS_API Problem
{
public:
  const std::string & name() const;
  const Domain & domain() const;
  std::vector<std::string> features() const;

  struct Impl;
  explicit Problem(std::shared_ptr<const Impl> impl);
  const Impl & impl() const {return *impl_;}

private:
  std::shared_ptr<const Impl> impl_;
};

KAIROS_API Result<Domain> parse_domain(
  std::string_view text, std::string_view filename = "domain.pddl");
KAIROS_API Result<Problem> parse_problem(
  std::string_view text, const Domain & domain, std::string_view filename = "problem.pddl");
KAIROS_API Result<Domain> load_domain(const std::filesystem::path & file);
KAIROS_API Result<Problem> load_problem(const std::filesystem::path & file, const Domain & domain);

/// @brief Checks a problem against an already parsed domain.
KAIROS_API CheckReport check_problem(
  std::string_view problem_text, const Domain & domain,
  std::string_view filename = "problem.pddl");

/// @brief Validates a plan for a parsed problem (no parsing again).
KAIROS_API ValidationReport validate(
  const Problem & problem, const Plan & plan, const ValidateOptions & options = {});

struct PlanUpdate
{
  const Plan & plan;
  uint32_t index;                     // 0 for the first plan, then 1, 2...
  std::chrono::milliseconds elapsed;  // since the search started
  bool proven_optimal;
};

struct SolveOptions
{
  std::chrono::milliseconds timeout{std::chrono::seconds(30)};
  size_t memory_limit_mb = 0;  // 0: automatic, half of what is available (RAM, cgroup, rlimit)
  bool anytime = true;         // keep improving until the timeout or a proof of optimality
  Engine engine = Engine::Auto;
  /// @brief Search configuration of M1: "auto", or a preset (see search_presets()).
  std::string search = "auto";
  unsigned threads = 1;
  double epsilon = 0.001;
  uint64_t seed = 0;
  LogOptions log;
  /// @brief Write intermediate structures there: "ast", "lifted", "ground", "sas",
  /// "plan-trace" (all of them if `dump` is empty).
  std::optional<std::filesystem::path> dump_dir;
  std::set<std::string> dump;
};

struct SearchOptions : SolveOptions
{
  std::optional<std::chrono::milliseconds> stop_after_no_improvement;
  std::optional<double> target_cost;
};

struct SolveResult
{
  Status status = Status::InternalError;
  std::optional<Plan> plan;           // the best valid plan
  StopReason stop_reason = StopReason::Exhausted;
  std::vector<Diagnostic> diagnostics;
  Statistics stats;
};

/**
 * @brief An anytime search running in its own thread. Move-only; the destructor stops it and
 * waits for the thread. Every member function is thread-safe.
 */
class KAIROS_API Search
{
public:
  Search();
  Search(Search &&) noexcept;
  Search & operator=(Search &&) noexcept;
  Search(const Search &) = delete;
  Search & operator=(const Search &) = delete;
  ~Search();

  /// @brief The first plan, waiting at most `timeout`; nullopt if there is none by then (or
  /// the search ended without plan).
  std::optional<Plan> wait_first(std::chrono::milliseconds timeout);
  /// @brief The next plan not returned yet by next_plan(), waiting at most `timeout`.
  std::optional<Plan> next_plan(std::chrono::milliseconds timeout);
  std::optional<Plan> best() const;
  bool running() const;
  void stop();
  /// @brief Waits until the search ends.
  SolveResult wait();
  StopReason stop_reason() const;

  struct State;

private:
  friend class Planner;
  std::shared_ptr<State> state_;
  std::unique_ptr<std::jthread> thread_;
};

class KAIROS_API Planner
{
public:
  /// @brief Plans in the calling thread until the timeout, the stop token, or (with anytime)
  /// a proof of optimality; returns the best plan.
  SolveResult solve(
    const Problem & problem, const SolveOptions & options = {}, std::stop_token stop = {});

  /// @brief Starts an anytime search in its own thread; `on_plan` runs in that thread for each
  /// better plan, after validating it and outside any lock (it may call the Search back).
  Search start(
    const Problem & problem, const SearchOptions & options = {},
    std::function<void(const PlanUpdate &)> on_plan = {});
};

}  // namespace kairos

#endif  // KAIROS__PLANNER_HPP_
