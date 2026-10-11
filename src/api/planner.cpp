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

#include "kairos/planner.hpp"

#include <chrono>
#include <condition_variable>
#include <exception>
#include <fstream>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "core/logger.hpp"
#include "orchestrator/orchestrator.hpp"
#include "pddl/parser.hpp"
#include "semantic/analyzer.hpp"
#include "validate/validator.hpp"

namespace kairos
{

// --- Domains and problems ---------------------------------------------------------------------

struct Domain::Impl
{
  std::string text;
  std::string filename;
  ast::Domain ast;
  sem::DomainModel model;
  double parse_ms = 0.0;
};

struct Problem::Impl
{
  Domain domain;
  std::string text;
  std::string filename;
  ast::Problem ast;
  sem::ProblemModel model;
  double parse_ms = 0.0;
};

Domain::Domain(std::shared_ptr<const Impl> impl)
: impl_(std::move(impl)) {}
const std::string & Domain::name() const {return impl_->model.name;}
std::vector<std::string> Domain::features() const {return impl_->model.features.names();}

Problem::Problem(std::shared_ptr<const Impl> impl)
: impl_(std::move(impl)) {}
const std::string & Problem::name() const {return impl_->model.name;}
const Domain & Problem::domain() const {return impl_->domain;}
std::vector<std::string> Problem::features() const {return impl_->model.features.names();}

namespace
{

using Clock = std::chrono::steady_clock;

void append(std::vector<Diagnostic> & to, std::vector<Diagnostic> from)
{
  to.insert(to.end(), std::make_move_iterator(from.begin()), std::make_move_iterator(from.end()));
}

std::optional<std::string> read(const std::filesystem::path & file)
{
  std::ifstream in(file, std::ios::binary);
  if (!in) {
    return std::nullopt;
  }
  std::stringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

Diagnostic cannot_read(const std::filesystem::path & file)
{
  return Diagnostic::error("io.cannot_read", "cannot read " + file.string());
}

}  // namespace

Result<Domain> parse_domain(std::string_view text, std::string_view filename)
{
  auto t0 = Clock::now();
  auto impl = std::make_shared<Domain::Impl>();
  impl->text = std::string(text);
  impl->filename = std::string(filename);
  auto parsed = pddl::parse_domain(impl->text, filename);
  std::vector<Diagnostic> diagnostics = parsed.take_diagnostics();
  if (!parsed.ok()) {
    return Result<Domain>::failure(std::move(diagnostics));
  }
  impl->ast = std::move(parsed).value();
  auto model = sem::analyze_domain(impl->ast);
  append(diagnostics, model.take_diagnostics());
  if (!model.ok()) {
    return Result<Domain>::failure(std::move(diagnostics));
  }
  impl->model = std::move(model).value();
  impl->parse_ms = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
  return Result<Domain>::success(Domain(std::move(impl)), std::move(diagnostics));
}

Result<Problem> parse_problem(
  std::string_view text, const Domain & domain, std::string_view filename)
{
  auto t0 = Clock::now();
  auto impl = std::make_shared<Problem::Impl>(Problem::Impl{domain, {}, {}, {}, {}, 0.0});
  impl->text = std::string(text);
  impl->filename = std::string(filename);
  auto parsed = pddl::parse_problem(impl->text, filename);
  std::vector<Diagnostic> diagnostics = parsed.take_diagnostics();
  if (!parsed.ok()) {
    return Result<Problem>::failure(std::move(diagnostics));
  }
  impl->ast = std::move(parsed).value();
  auto model = sem::analyze_problem(impl->ast, domain.impl().ast, domain.impl().model);
  append(diagnostics, model.take_diagnostics());
  if (!model.ok()) {
    return Result<Problem>::failure(std::move(diagnostics));
  }
  impl->model = std::move(model).value();
  impl->parse_ms = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
  return Result<Problem>::success(Problem(std::move(impl)), std::move(diagnostics));
}

Result<Domain> load_domain(const std::filesystem::path & file)
{
  auto text = read(file);
  if (!text) {
    return Result<Domain>::failure(cannot_read(file));
  }
  return parse_domain(*text, file.string());
}

Result<Problem> load_problem(const std::filesystem::path & file, const Domain & domain)
{
  auto text = read(file);
  if (!text) {
    return Result<Problem>::failure(cannot_read(file));
  }
  return parse_problem(*text, domain, file.string());
}

CheckReport check_problem(
  std::string_view problem_text, const Domain & domain, std::string_view filename)
{
  CheckReport report;
  auto parsed = pddl::parse_problem(problem_text, filename);
  report.diagnostics = parsed.take_diagnostics();
  if (parsed.ok()) {
    auto model = sem::analyze_problem(*parsed, domain.impl().ast, domain.impl().model);
    if (model.ok()) {
      report.features = model->features.names();
    }
    bool supported = false;
    if (model.ok()) {
      auto missing = orch::unsupported(model->features);
      supported = missing.empty();
      for (auto & d : missing) {
        d.severity = Severity::Warning;  // correct PDDL, not plannable yet
        report.diagnostics.push_back(std::move(d));
      }
    }
    append(report.diagnostics, model.take_diagnostics());
    report.valid = !has_errors(report.diagnostics);
    report.plannable = report.valid && supported;
  }
  report.valid = !has_errors(report.diagnostics);
  return report;
}

ValidationReport validate(
  const Problem & problem, const Plan & plan, const ValidateOptions & options)
{
  const auto & p = problem.impl();
  const auto & d = p.domain.impl();
  return val::validate(d.ast, d.model, p.ast, p.model, plan, options);
}

std::vector<std::string> search_presets()
{
  return orch::presets();
}

std::string_view to_string(Status status)
{
  switch (status) {
    case Status::Solved: return "solved";
    case Status::Unsolvable: return "unsolvable";
    case Status::Timeout: return "timeout";
    case Status::Cancelled: return "cancelled";
    case Status::OutOfMemory: return "out of memory";
    case Status::Unsupported: return "unsupported";
    case Status::InvalidInput: return "invalid input";
    case Status::GroundingTooLarge: return "grounding too large";
    case Status::InternalError: return "internal error";
  }
  return "unknown";
}

std::string_view to_string(StopReason reason)
{
  switch (reason) {
    case StopReason::Exhausted: return "exhausted";
    case StopReason::ProvenOptimal: return "proven optimal";
    case StopReason::Timeout: return "timeout";
    case StopReason::NoImprovement: return "no improvement";
    case StopReason::TargetReached: return "target reached";
    case StopReason::Stopped: return "stopped";
    case StopReason::MemoryLimit: return "memory limit";
  }
  return "unknown";
}

// --- Planning ---------------------------------------------------------------------------------

namespace
{

orch::Input input(const Problem & problem)
{
  const auto & p = problem.impl();
  const auto & d = p.domain.impl();
  return orch::Input{d.ast, d.model, p.ast, p.model};
}

}  // namespace

SolveResult Planner::solve(
  const Problem & problem, const SolveOptions & options, std::stop_token stop)
{
  SearchOptions o;
  static_cast<SolveOptions &>(o) = options;
  SolveResult r = orch::run(input(problem), o, stop, {});
  r.stats.parse_ms = problem.domain().impl().parse_ms + problem.impl().parse_ms;
  return r;
}

struct Search::State
{
  mutable std::mutex mutex;
  std::condition_variable changed;
  std::vector<Plan> plans;
  size_t next = 0;
  bool done = false;
  SolveResult result;
};

Search::Search() = default;
Search::Search(Search &&) noexcept = default;
Search & Search::operator=(Search &&) noexcept = default;

Search::~Search()
{
  if (thread_ && thread_->joinable()) {
    thread_->request_stop();
    thread_->join();
  }
}

Search Planner::start(
  const Problem & problem, const SearchOptions & options,
  std::function<void(const PlanUpdate &)> on_plan)
{
  Search s;
  s.state_ = std::make_shared<Search::State>();
  auto state = s.state_;
  Logger log(options.log);
  s.thread_ = std::make_unique<std::jthread>(
    [state, problem, options, on_plan = std::move(on_plan), log](std::stop_token stop) {
      auto callback = [&](const Plan & plan, uint32_t index, std::chrono::milliseconds elapsed) {
        {
          std::lock_guard<std::mutex> lock(state->mutex);
          state->plans.push_back(plan);
        }
        state->changed.notify_all();
        if (on_plan) {
          try {
            on_plan(PlanUpdate{plan, index, elapsed, false});
          } catch (const std::exception & e) {
            log.error(LogChannel::Orchestrator, "the plan callback threw: {}", e.what());
          } catch (...) {
            log.error(LogChannel::Orchestrator, "the plan callback threw");
          }
        }
      };
      SolveResult r = orch::run(input(problem), options, stop, callback);
      r.stats.parse_ms = problem.domain().impl().parse_ms + problem.impl().parse_ms;
      {
        std::lock_guard<std::mutex> lock(state->mutex);
        state->result = std::move(r);
        state->done = true;
      }
      state->changed.notify_all();
    });
  return s;
}

std::optional<Plan> Search::wait_first(std::chrono::milliseconds timeout)
{
  if (!state_) {
    return std::nullopt;
  }
  std::unique_lock<std::mutex> lock(state_->mutex);
  state_->changed.wait_for(
    lock, timeout, [&]() {return !state_->plans.empty() || state_->done;});
  if (state_->plans.empty()) {
    return std::nullopt;
  }
  return state_->plans.front();
}

std::optional<Plan> Search::next_plan(std::chrono::milliseconds timeout)
{
  if (!state_) {
    return std::nullopt;
  }
  std::unique_lock<std::mutex> lock(state_->mutex);
  state_->changed.wait_for(
    lock, timeout, [&]() {return state_->plans.size() > state_->next || state_->done;});
  if (state_->plans.size() <= state_->next) {
    return std::nullopt;
  }
  return state_->plans[state_->next++];
}

std::optional<Plan> Search::best() const
{
  if (!state_) {
    return std::nullopt;
  }
  std::lock_guard<std::mutex> lock(state_->mutex);
  if (state_->plans.empty()) {
    return std::nullopt;
  }
  return state_->plans.back();
}

bool Search::running() const
{
  if (!state_) {
    return false;
  }
  std::lock_guard<std::mutex> lock(state_->mutex);
  return !state_->done;
}

void Search::stop()
{
  if (thread_) {
    thread_->request_stop();
  }
}

SolveResult Search::wait()
{
  if (!state_) {
    SolveResult r;
    r.diagnostics.push_back(Diagnostic::error("search.empty", "no search was started"));
    return r;
  }
  if (thread_ && thread_->joinable()) {
    thread_->join();
  }
  std::lock_guard<std::mutex> lock(state_->mutex);
  return state_->result;
}

StopReason Search::stop_reason() const
{
  if (!state_) {
    return StopReason::Exhausted;
  }
  std::lock_guard<std::mutex> lock(state_->mutex);
  return state_->result.stop_reason;
}

}  // namespace kairos
