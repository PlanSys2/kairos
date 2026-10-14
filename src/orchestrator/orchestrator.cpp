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

#include "orchestrator/orchestrator.hpp"

#if defined(__GLIBC__)
#include <malloc.h>
#endif
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <exception>
#include <fstream>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "core/budget.hpp"
#include "core/logger.hpp"
#include "core/memory_info.hpp"
#include "ground/grounder.hpp"
#include "heuristics/heuristic.hpp"
#include "lift/lift.hpp"
#include "normalize/normalizer.hpp"
#include "orchestrator/explain.hpp"
#include "pddl/printer.hpp"
#include "sas/translate.hpp"
#include "search/goals.hpp"
#include "search/search.hpp"
#include "temporal/compress.hpp"
#include "temporal/hybrid.hpp"
#include "temporal/durations.hpp"
#include "temporal/profile.hpp"
#include "temporal/snap.hpp"
#include "state/eval.hpp"
#include "state/registry.hpp"
#include "state/successors.hpp"
#include "validate/validator.hpp"

namespace kairos::orch
{

namespace
{

using Clock = std::chrono::steady_clock;

double ms_since(Clock::time_point t)
{
  return std::chrono::duration<double, std::milli>(Clock::now() - t).count();
}

}  // namespace

const std::vector<Support> & support_table()
{
  using F = sem::Feature;
  static const std::vector<Support> table = {
    {F::Typing, Support::Full, "", ""},
    {F::NegativePreconditions, Support::Full, "", ""},
    {F::DisjunctivePreconditions, Support::Full, "", ""},
    {F::Equality, Support::Full, "", ""},
    {F::ExistentialPreconditions, Support::Full, "", ""},
    {F::UniversalPreconditions, Support::Full, "", ""},
    {F::ConditionalEffects, Support::Full, "", ""},
    {F::NumericFluents, Support::Full, "", ""},
    {F::ObjectFluents, Support::Full, "", ""},
    {F::NonLinearNumeric, Support::Full,
      "planned and validated exactly; the heuristics linearize non-linear conditions at each "
      "state", ""},
    {F::ActionCosts, Support::Full, "", ""},
    {F::DurativeActions, Support::Full, "required concurrency included, in engine M3", ""},
    {F::DurationInequalities, Support::Full,
      "the bounds in engines M2 and M3, durations in between in steps of time", ""},
    {F::VariableDuration, Support::Full, "", ""},
    {F::DurationInEffects, Support::Full, "", ""},
    {F::ContinuousEffects, Support::Full,
      "linear ones as discrete changes in engines M2 and M3, any in steps of time (integrated "
      "with Runge-Kutta)", ""},
    {F::DerivedPredicates, Support::Full, "recursive ones included", ""},
    {F::TimedInitialLiterals, Support::Full, "", ""},
    {F::TimedInitialFluents, Support::Full, "", ""},
    {F::Preferences, Support::Full,
      "compiled in goals (also under forall), at the top of preconditions (four per action) "
      "and over constraints; counted elsewhere (more, or nested), where only improving plans "
      "sees their cost",
      ""},
    {F::TrajectoryConstraints, Support::Full,
      "all, hard or as preferences; always-within in temporal plans with a clock", ""},
    {F::Processes, Support::Full,
      "planned in steps of time (to where a condition changes), integrated with Runge-Kutta; "
      "every plan validated in continuous time", ""},
    {F::Events, Support::Full, "fired between the steps; every plan validated in continuous "
      "time (events fire when their condition crosses)", ""},
    {F::GlobalConstraints, Support::Full, "", ""},
    {F::Metric, Support::Full, "", ""},
    {F::MetricTotalTime, Support::Full, "alone or added to numeric fluents", ""},
    {F::MetricFluents, Support::Full, "", ""},
  };
  return table;
}

namespace
{

void dump(
  const SearchOptions & o, const char * what, const std::string & file,
  const std::function<std::string()> & text, const Logger & log)
{
  if (!o.dump_dir || (!o.dump.empty() && !o.dump.count(what))) {
    return;
  }
  std::error_code ec;
  std::filesystem::create_directories(*o.dump_dir, ec);
  std::ofstream out(*o.dump_dir / file);
  out << text();
  if (!out) {
    log.warning(LogChannel::Orchestrator, "could not write {}", (*o.dump_dir / file).string());
  }
}

// The plan step by step: each action and the facts that hold after it
std::string plan_trace(const sas::Task & task, const std::vector<uint32_t> & actions)
{
  std::vector<uint32_t> values = task.initial;
  std::vector<double> numbers;
  for (const auto & n : task.initial_numeric) {
    numbers.push_back(n ? *n : state::kUndefined);
  }
  state::AxiomEvaluator axioms(task);
  auto facts = [&]() {
      axioms.evaluate(values.data(), numbers.data());
      std::string out;
      for (sas::VarId v = 0; v < values.size(); ++v) {
        if (values[v] != task.variables[v].none()) {
          out += "  " + sas::fact_name(task, sas::Fact{v, values[v]}) + "\n";
        }
      }
      for (size_t i = 0; i < numbers.size(); ++i) {
        const auto & f = task.numeric_fluents[i];
        std::string name = "(" + task.function_names[f.function.index()];
        for (auto o : f.args) {
          name += " " + task.object_names[o.index()];
        }
        out += "  (= " + name + ") " + std::to_string(numbers[i]) + ")\n";
      }
      return out;
    };
  std::string out = "; initial state\n" + facts();
  std::vector<uint32_t> next_values;
  std::vector<double> next_numbers;
  for (size_t i = 0; i < actions.size(); ++i) {
    const auto & a = task.actions[actions[i]];
    state::apply_effects(
      task, a.changes, a.effects, sas::Timing::Instant, values.data(), numbers.data(),
      next_values, next_numbers);
    values = next_values;
    numbers = next_numbers;
    out += "; step " + std::to_string(i + 1) + ": " + sas::instance_name(task, a) + "\n" + facts();
  }
  return out;
}

}  // namespace

std::vector<Diagnostic> unsupported(const sem::FeatureSet & features, Engine /*engine*/)
{
  std::vector<Diagnostic> out;
  for (const auto & m : support_table()) {
    if (m.level == Support::None && features.contains(m.feature)) {
      out.push_back(Diagnostic::error(
          m.code, "Kairos cannot plan yet with " + std::string(sem::to_string(m.feature)) +
          " (" + m.note + ")"));
    }
  }
  return out;
}

namespace
{

// The M1 search presets: first plan, then the improving restarts
std::optional<search::AnytimeOptions> preset(const std::string & name)
{
  search::AnytimeOptions ao;
  auto & f = ao.first;
  if (name == "ff") {
    f.heuristic = "ff";
    ao.improve_heuristic = "ff";
  } else if (name == "lama") {
    f.heuristic = "ff,lmcount";
    ao.improve_heuristic = "ff,lmcount";
  } else if (name == "eager-lama") {
    f.kind = search::EngineOptions::Kind::EagerGreedy;
    f.heuristic = "ff,lmcount";
    ao.improve_heuristic = "ff,lmcount";
  } else if (name == "novelty") {
    f.kind = search::EngineOptions::Kind::EagerGreedy;
    f.heuristic = "ff";
    f.novelty = 2;
    ao.improve_heuristic = "ff";
  } else if (name == "lama-novelty") {
    f.kind = search::EngineOptions::Kind::EagerGreedy;
    f.heuristic = "ff,lmcount";
    f.novelty = 2;
    ao.improve_heuristic = "ff,lmcount";
  } else if (name == "ehc") {
    f.kind = search::EngineOptions::Kind::EnforcedHillClimbing;
    f.heuristic = "ff";
    ao.improve_heuristic = "ff";
  } else if (name == "goals") {
    // Incremental goals for the first plan (search/goals.hpp), then improved as ff
    f.heuristic = "ff";
    ao.improve_heuristic = "ff";
  } else {
    return std::nullopt;
  }
  return ao;
}

// A metric to minimize of the form sum of c * fluent + k * total-time (k > 0, some fluent):
// euros plus time, as PlanSys2's combat example weighs them
struct MixedMetric
{
  std::map<sas::NumVarId, double> weight;
  double time = 0.0;
};

std::optional<MixedMetric> mixed_metric(const sas::Task & task)
{
  if (task.metric.is_default || !task.metric.minimize) {
    return std::nullopt;
  }
  MixedMetric m;
  bool ok = true;
  std::function<void(const ground::Expr &, double)> walk = [&](const ground::Expr & e, double c) {
      switch (e.op) {
        case ground::ExprOp::Constant: return;
        case ground::ExprOp::Fluent: m.weight[e.fluent] += c; return;
        case ground::ExprOp::TotalTime: m.time += c; return;
        case ground::ExprOp::Add:
          for (const auto & ch : e.children) {
            walk(ch, c);
          }
          return;
        case ground::ExprOp::Sub:
          if (e.children.size() == 2) {
            walk(e.children[0], c);
            walk(e.children[1], -c);
            return;
          }
          break;
        case ground::ExprOp::Neg:
          walk(e.children.at(0), -c);
          return;
        case ground::ExprOp::Mul:
          if (e.children.size() == 2 && e.children[0].op == ground::ExprOp::Constant) {
            walk(e.children[1], c * e.children[0].value);
            return;
          }
          if (e.children.size() == 2 && e.children[1].op == ground::ExprOp::Constant) {
            walk(e.children[0], c * e.children[1].value);
            return;
          }
          break;
        default:
          break;
      }
      ok = false;
    };
  walk(task.metric.expr.tree, 1.0);
  if (!ok || m.time <= 0.0 || m.weight.empty()) {
    return std::nullopt;
  }
  return m;
}

// What an action adds to the weighted fluents (nullopt: not a constant increase or decrease)
std::optional<double> weighted_delta(
  const std::vector<sas::Effect> & effects, const std::map<sas::NumVarId, double> & weight)
{
  double delta = 0.0;
  for (const auto & e : effects) {
    auto w = weight.find(e.num);
    if (e.kind != sas::Effect::Kind::Numeric || w == weight.end()) {
      continue;
    }
    bool constant = e.value.linear && e.value.linear->terms.empty() && e.condition.empty() &&
      !e.continuous;
    if (!constant || (e.op != ast::AssignOp::Increase && e.op != ast::AssignOp::Decrease)) {
      return std::nullopt;
    }
    double d = e.value.linear->constant;
    delta += w->second * (e.op == ast::AssignOp::Increase ? d : -d);
  }
  return delta;
}

// Numeric conditions anywhere: what novelty helps with (measured on IPC numeric and classical
// suites, diseño.md 8.3)
bool has_numeric_conditions(const sas::Task & task)
{
  auto any = [](const sas::Condition & c) {return !c.numeric.empty();};
  if (any(task.goal)) {
    return true;
  }
  for (const auto & a : task.actions) {
    if (any(a.pre)) {
      return true;
    }
    for (const auto & e : a.effects) {
      if (any(e.condition)) {
        return true;
      }
    }
  }
  for (const auto & ax : task.axioms) {
    if (any(ax.body)) {
      return true;
    }
  }
  return false;
}

// "auto": LAMA (hFF and landmarks, lazy) on classical tasks, with novelty on numeric ones; on
// snap tasks (M3) always eager with novelty: lazy search pops children of other parents, and the
// temporal filter rebuilds the plan of each (match-cellar 6: 72 000 expansions against 64)
std::string resolve_auto(const sas::Task & task, bool snap)
{
  return snap || has_numeric_conditions(task) ? "lama-novelty" : "lama";
}

}  // namespace

std::vector<std::string> presets()
{
  return {"ff", "lama", "eager-lama", "novelty", "lama-novelty", "ehc", "goals"};
}

namespace
{

// The phases of the search, each on a task (plain, compressed for M2 or snap for M3) with a
// search configuration and a share of the remaining time:
// - classical and numeric tasks: one phase;
// - M3 with "auto": a portfolio, since no configuration wins everywhere (lazy LAMA finds
//   PlanSys2's bt_example at once; eager search with novelty, match-cellar);
// - simple temporal tasks with "auto" and anytime: M2 for the first plan (its improvements
//   lower the sum of durations, which says little of the makespan), then the M3 portfolio
//   with g the makespan of the partial plan
// Steps: the task in steps of time (temporal/hybrid.hpp) beside M2 or M3, for durations in
// between their bounds and continuous effects as they are
enum class Kind : uint8_t {Plain, Compressed, Snap, Steps};

struct Phase
{
  Kind kind;
  std::string search;
  double share;      // of the time left when it starts (1: all)
  bool improve;
  bool makespan_g;   // g is the makespan of the partial plan (M3)
  bool only_without_plan = false;  // skipped once an earlier phase found a plan
  temporal::Waits waits = temporal::Waits::Both;  // PDDL+: how time passes
};

// Forwards the caller's stop request to the run's own stop source
struct ForwardStop
{
  std::stop_source * to;
  void operator()() const {to->request_stop();}
};

// One call of run(): the pipeline down to the SAS+ task, the engine, the search phases and the
// plans they find (validated before anyone sees them)
class Run
{
public:
  Run(
    const Input & in, const SearchOptions & options, std::stop_token stop,
    const PlanCallback & on_plan)
  : in_(in), o_(options), on_plan_(on_plan), t0_(Clock::now()), log_(log_options(options)),
    forward_(std::move(stop), ForwardStop{&internal_})
  {
  }

  SolveResult execute()
  {
    try {
      if (auto done = setup()) {
        return *done;
      }
      if (auto done = build_task()) {
        return *done;
      }
      if (lifted_task_ && (budget_->check(), budget_->memory_used() > bc_.memory_limit_bytes / 4)) {
        // A search takes about three times its task: no room for it, the lifted engine plans
        sas_ = nullptr;
        fixed_ = {};
        translated_.reset();
        const char * why = "the grounded task takes over a quarter of the memory";
        grounded_ = true;
        return lifted_.joinable() ? wait_lifted(why) : plan_lifted(*lifted_task_, why);
      }
      if (auto done = choose_engine()) {
        return *done;
      }
      // One thread and a large task: the grounded search has half of the time left, the
      // lifted engine the rest if it found no plan
      auto deadline = *bc_.deadline;
      bool split = large_ && !has_plan();
      if (split) {
        bc_.deadline = Clock::now() + (deadline - Clock::now()) / 2;
        budget_.emplace(bc_);
      }
      plan_phases();
      run_phases();
      if (split) {
        bc_.deadline = deadline;
        budget_.emplace(bc_);
        if (!has_plan() && !proof_ && !internal_.stop_requested()) {
          grounded_ = true;
          return plan_lifted(*lifted_task_, "no plan from the grounded search in its half");
        }
      }
      if (lifted_.joinable() && !has_plan() && !proof_ && !m3_exhausted_) {
        lifted_.join();  // the grounded search gave up: the lifted engine may still find one
      }
      stop_lifted();
      return conclude();
    } catch (const std::exception & e) {
      res_.diagnostics.push_back(Diagnostic::error("orchestrator.exception", e.what()));
      return finish(Status::InternalError, StopReason::Exhausted);
    }
  }

private:
  // With several threads, the caller's log callback is called by one at a time
  static LogOptions log_options(const SearchOptions & o)
  {
    LogOptions lo = o.log;
    if (o.threads > 1 && lo.callback) {
      auto mutex = std::make_shared<std::mutex>();
      lo.callback = [mutex, cb = lo.callback](LogLevel l, LogChannel c, std::string_view m) {
          std::lock_guard<std::mutex> lock(*mutex);
          cb(l, c, m);
        };
    }
    return lo;
  }

  bool has_plan()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return res_.plan.has_value();
  }

  SolveResult finish(Status status, StopReason reason)
  {
    res_.status = status;
    res_.stop_reason = reason;
    res_.stats.total_ms = ms_since(t0_);
    // The highest resident size (the current one is what is left after freeing)
    if (auto rss = peak_rss_bytes() ? peak_rss_bytes() : current_rss_bytes()) {
      res_.stats.peak_memory_bytes = *rss;
    }
    log_.info(
      LogChannel::Orchestrator, "{} ({}) in {:.1f} ms", to_string(status), to_string(reason),
      res_.stats.total_ms);
    return res_;
  }

  std::pair<Status, StopReason> budget_status() const
  {
    // The lifted engine's, when it ran out alone
    auto state = budget_->state();
    if (state == BudgetState::Ok && lifted_budget_) {
      state = lifted_budget_->state();
    }
    switch (state) {
      case BudgetState::Timeout: return std::make_pair(Status::Timeout, StopReason::Timeout);
      case BudgetState::OutOfMemory:
        return std::make_pair(Status::OutOfMemory, StopReason::MemoryLimit);
      default: return std::make_pair(Status::Cancelled, StopReason::Stopped);
    }
  }

  // What Kairos cannot plan with, the search configuration and the budget
  std::optional<SolveResult> setup()
  {
    res_.diagnostics = unsupported(in_.problem_model.features, o_.engine);
    if (!res_.diagnostics.empty()) {
      return finish(Status::Unsupported, StopReason::Exhausted);
    }
    // "auto" is resolved once the SAS+ task is known
    if (!preset(o_.search == "auto" ? "lama" : o_.search)) {
      res_.diagnostics.push_back(Diagnostic::error(
          "orchestrator.unknown_search", "unknown search configuration '" + o_.search + "'"));
      return finish(Status::InvalidInput, StopReason::Exhausted);
    }
    // Budget: the caller's stop token, the timeout and the memory limit
    size_t memory = o_.memory_limit_mb ? o_.memory_limit_mb * (size_t{1} << 20) :
      default_memory_limit(read_memory_snapshot());
    res_.stats.memory_limit_bytes = memory;
    log_.info(LogChannel::Memory, "memory limit {} MB", memory >> 20);
    bc_.stop = internal_.get_token();
    bc_.deadline = t0_ + o_.timeout;
    bc_.memory_limit_bytes = memory;
    budget_.emplace(bc_);
    return std::nullopt;
  }

  // Normalize, ground, translate to SAS+ and fix the durations the engines cannot use as given
  std::optional<SolveResult> build_task()
  {
    // Never the usual names of the inputs: --dump in their directory would overwrite them
    dump(o_, "ast", "ast-domain.pddl", [&]() {return pddl::to_pddl(in_.domain);}, log_);
    dump(o_, "ast", "ast-problem.pddl", [&]() {return pddl::to_pddl(in_.problem);}, log_);

    auto t = Clock::now();
    lifted::Task task = lifted::normalize(in_.domain, in_.domain_model, in_.problem,
        in_.problem_model);
    res_.stats.normalize_ms = ms_since(t);
    dump(o_, "lifted", "lifted.txt", [&]() {return lifted::to_string(task);}, log_);
    if (task.has_timed_constraints) {
      res_.diagnostics.push_back(Diagnostic::error(
          "unsupported.timed_constraints", "Kairos cannot plan yet with always-within in "
          "temporal plans (trajectory constraints)"));
      return finish(Status::Unsupported, StopReason::Exhausted);
    }

    t = Clock::now();
    ground::GroundOptions go;
    go.logger = log_;
    // A task the lifted engine can plan grounds with part of the time and memory: if that is
    // not enough, it is planned without grounding
    bool liftable = lift::unsupported(task).empty() && o_.engine == Engine::Auto;
    std::optional<Budget> ground_budget;
    if (liftable) {
      BudgetConfig gc = bc_;
      auto left = (bc_.deadline ? *bc_.deadline : t0_ + o_.timeout) - Clock::now();
      gc.deadline = Clock::now() + std::chrono::duration_cast<Clock::duration>(left * 0.4);
      gc.memory_limit_bytes = bc_.memory_limit_bytes / 10 * 6;
      ground_budget.emplace(gc);
      lifted_task_ = task;
    }
    using Grounded = decltype(ground::ground(task, go, nullptr));
    std::optional<Grounded> attempt;
    if (liftable) {
      // A short grounding first: a task that grounds in it is planned as always. A larger one
      // has the lifted engine beside the whole grounding (several threads) or, with one, for a
      // slice of time before it
      BudgetConfig pc = bc_;
      pc.deadline = std::min(Clock::now() + kGroundProbe, *bc_.deadline);
      pc.memory_limit_bytes = bc_.memory_limit_bytes / 10 * 6;
      Budget probe(pc);
      Grounded g = ground::ground(task, go, & probe);
      if (g.ok() || g.diagnostics().front().code != "ground.budget" || budget_->exhausted()) {
        attempt.emplace(std::move(g));
      } else if (o_.threads > 1) {
        start_lifted();
      } else if (auto done = lifted_slice(task)) {
        return done;
      } else {
        large_ = true;
      }
    }
    if (!attempt) {
      attempt.emplace(ground::ground(task, go, ground_budget ? & *ground_budget : & *budget_));
    }
    auto & grounded = *attempt;
    res_.stats.ground_ms = ms_since(t);
    if (!grounded.ok() && liftable && grounded.diagnostics().front().code == "ground.budget") {
      if (lifted_.joinable()) {
        return wait_lifted(grounded.diagnostics().front().message);
      }
      if (has_plan()) {
        return finish(Status::Solved, StopReason::Exhausted);  // the lifted slice's
      }
      if (!budget_->exhausted()) {
        return plan_lifted(task, grounded.diagnostics().front().message);
      }
    }
    if (!grounded.ok()) {
      res_.diagnostics = grounded.take_diagnostics();
      if (res_.diagnostics.front().code == "ground.unsupported") {
        return finish(Status::Unsupported, StopReason::Exhausted);
      }
      auto [s, r] = budget_status();
      return finish(s == Status::OutOfMemory ? Status::GroundingTooLarge : s, r);
    }
    res_.stats.atoms = grounded->atoms.size();
    res_.stats.actions = grounded->actions.size();
    res_.stats.durative_actions = grounded->durative_actions.size();
    dump(o_, "ground", "ground.txt", [&]() {return ground::to_string(*grounded);}, log_);
    if (grounded->goal_unreachable) {
      // Which goal facts, and why: the preconditions that never hold, down to what no action
      // adds and the initial state does not have
      std::string why = "the goal needs atoms that no action can reach";
      for (const auto & line : explain_unreachable_goal(task, *grounded)) {
        why += "\n" + line;
      }
      res_.diagnostics.push_back(Diagnostic::note("orchestrator.unreachable_goal", why));
      return finish(Status::Unsolvable, StopReason::Exhausted);
    }

    t = Clock::now();
    sas::TranslateOptions so;
    so.logger = log_;
    sas::TranslateStats sst;
    auto translated = sas::translate(task, std::move(*grounded), so, &*budget_, &sst);
    res_.stats.sas_ms = ms_since(t);
    if (!translated.ok()) {
      res_.diagnostics = translated.take_diagnostics();
      if (res_.diagnostics.front().code == "sas.unstratified") {
        return finish(Status::InvalidInput, StopReason::Exhausted);
      }
      auto [s, r] = budget_status();
      return finish(s, r);
    }
    translated_.emplace(std::move(translated).value());
    // PDDL+ plans with durations and continuous effects as they are (clocks and processes)
    if (!translated_->processes.empty() || !translated_->events.empty()) {
      sas_ = &*translated_;
      res_.stats.variables = sas_->variables.size();
      res_.stats.mutex_groups = sst.group_variables;
      dump(o_, "sas", "sas.txt", [&]() {return sas::to_string(*sas_);}, log_);
      return std::nullopt;
    }
    // Durations the engines can plan with: one action per bound of a duration inequality used
    // by its effects, continuous effects as discrete ones (sound, not complete)
    fixed_ = temporal::fix_durations(*translated_);
    if (!fixed_.unsupported.empty()) {
      // What M2 and M3 cannot plan (a continuous change read other than as a bound): time in
      // steps, as PDDL+
      log_.info(LogChannel::Orchestrator, "durations: {}; planned as PDDL+", fixed_.unsupported);
      continuous_steps_ = true;
      sas_ = &*translated_;
      res_.stats.variables = sas_->variables.size();
      res_.stats.mutex_groups = sst.group_variables;
      dump(o_, "sas", "sas.txt", [&]() {return sas::to_string(*sas_);}, log_);
      return std::nullopt;
    }
    if (fixed_.changed) {
      log_.info(
        LogChannel::Orchestrator, "durations: {} durative actions after fixing durations and "
        "continuous effects", fixed_.task.durative_actions.size());
    }
    sas_ = fixed_.changed ? &fixed_.task : &*translated_;
    res_.stats.variables = sas_->variables.size();
    res_.stats.mutex_groups = sst.group_variables;
    dump(o_, "sas", "sas.txt", [&]() {return sas::to_string(*sas_);}, log_);
    return std::nullopt;
  }

  // A task too large to ground: searched as it is (lift/lift.hpp)
  SolveResult plan_lifted(const lifted::Task & task, const std::string & why)
  {
    log_.info(LogChannel::Orchestrator, "{}; planned without grounding", why);
#if defined(__GLIBC__)
    malloc_trim(0);  // what grounding freed, back to the system (the limit reads the RSS)
#endif
    lift::Stats ls;
    auto t = Clock::now();
    auto plan = lift::plan(task, &*budget_, log_, ls);
    res_.stats.search_ms = ms_since(t);
    res_.stats.expanded = ls.expanded;
    res_.stats.evaluated = ls.generated;
    res_.stats.states = ls.states;
    if ((plan && offer(std::move(plan), {}, empty_task_)) || has_plan()) {
      return finish(Status::Solved, StopReason::Exhausted);
    }
    if (budget_->check() == BudgetState::Ok) {
      return finish(Status::Unsolvable, StopReason::Exhausted);  // its whole space
    }
    auto [s, r] = budget_status();
    return finish(s == Status::OutOfMemory && !grounded_ ? Status::GroundingTooLarge : s, r);
  }
  sas::Task empty_task_;  // the task of plans the lifted engine finds (there is none)

  // One thread, a large task: the lifted engine for a slice of the time. A result if that
  // settles it (a plan without anytime, or its whole space without one)
  std::optional<SolveResult> lifted_slice(const lifted::Task & task)
  {
    auto left = *bc_.deadline - Clock::now();
    BudgetConfig sc = bc_;
    sc.deadline = Clock::now() + std::min<Clock::duration>(kLiftedSlice,
        std::chrono::duration_cast<Clock::duration>(left * 0.05));
    Budget slice(sc);
    lift::Stats ls;
    auto plan = lift::plan(task, &slice, log_, ls);
    if (plan) {
      offer(std::move(plan), {}, empty_task_);
    }
    if (has_plan() && !o_.anytime) {
      return finish(Status::Solved, StopReason::NoImprovement);
    }
    if (!plan && slice.check() == BudgetState::Ok) {
      return finish(Status::Unsolvable, StopReason::Exhausted);  // its whole space
    }
    log_.info(LogChannel::Orchestrator, "lifted search first: {} expanded, {}", ls.expanded,
      has_plan() ? "a plan" : "no plan");
    return std::nullopt;
  }
  bool large_ = false;  // one thread: the short grounding did not fit
  static constexpr auto kGroundProbe = std::chrono::milliseconds(200);
  static constexpr auto kLiftedSlice = std::chrono::milliseconds(1000);

  // The lifted engine in a thread of its own while the task grounds and the grounded search
  // runs: its first plan is offered as any other
  void start_lifted()
  {
    BudgetConfig lc = bc_;
    lifted_budget_.emplace(lc);
    lifted_ = std::jthread([this](std::stop_token st) {
          auto plan = lift::plan(*lifted_task_, &*lifted_budget_, log_, lifted_stats_,
          [&st]() {return st.stop_requested();});
          if (plan) {
            lifted_solved_ = offer(std::move(plan), {}, empty_task_);
          } else if (lifted_budget_->check() == BudgetState::Ok && !st.stop_requested()) {
            lifted_exhausted_ = true;  // its whole space: no plan
          }
        });
  }

  void stop_lifted()
  {
    if (lifted_.joinable()) {
      lifted_.request_stop();
      lifted_.join();
    }
  }

  // Grounding did not fit: what the lifted engine (running already) finds
  SolveResult wait_lifted(const std::string & why)
  {
    log_.info(LogChannel::Orchestrator, "{}; planned without grounding", why);
#if defined(__GLIBC__)
    malloc_trim(0);
#endif
    auto t = Clock::now();
    lifted_.join();
    res_.stats.search_ms = ms_since(t);
    res_.stats.expanded = lifted_stats_.expanded;
    res_.stats.evaluated = lifted_stats_.generated;
    res_.stats.states = lifted_stats_.states;
    if (lifted_solved_ || has_plan()) {
      return finish(
        Status::Solved, first_only_ ? StopReason::NoImprovement : StopReason::Exhausted);
    }
    if (lifted_exhausted_) {
      return finish(Status::Unsolvable, StopReason::Exhausted);
    }
    auto [s, r] = budget_status();
    return finish(s == Status::OutOfMemory && !grounded_ ? Status::GroundingTooLarge : s, r);
  }
  bool grounded_ = false;  // the task did ground (memory then ran out searching)
  std::optional<lifted::Task> lifted_task_;
  std::optional<Budget> lifted_budget_;
  lift::Stats lifted_stats_;
  std::atomic<bool> lifted_solved_{false};
  std::atomic<bool> lifted_exhausted_{false};

  // Durations fixed to their bounds for M2 and M3, with steps of time beside: their spaces
  // running out proves nothing
  bool bounded_durations() const {return hybrid_ && !hybrid_primary_;}

  // Without processes or events in the domain, plans are PDDL 2.1 ones: happenings that
  // depend on each other epsilon apart
  void separate_happenings()
  {
    if (in_.domain.processes.empty() && in_.domain.events.empty()) {
      for (auto * h : {&hybrid_, &hybrid_short_, &hybrid_crossing_}) {
        (*h)->separation = o_.epsilon;
      }
    }
  }

  // Temporal tasks: M2 compresses durative actions when no plan needs them to overlap; M3
  // searches start and end snap actions over a temporal network otherwise (as POPF)
  std::optional<SolveResult> choose_engine()
  {
    const sas::Task & task = *sas_;
    if (!task.processes.empty() || !task.events.empty() || continuous_steps_) {
      // PDDL+: discrete time steps, the plans validated with continuous time
      hybrid_primary_ = true;
      hybrid_ = temporal::hybrid(task, kHybridDelta, temporal::Waits::Both, o_.epsilon);
      hybrid_short_ = temporal::hybrid(task, kHybridDelta, temporal::Waits::Short, o_.epsilon,
          false);
      hybrid_crossing_ = temporal::hybrid(task, kHybridDelta, temporal::Waits::Crossing,
          o_.epsilon, false);
      // The same symmetries (the variants differ in the waits, which name no object)
      hybrid_short_->task.symmetries = hybrid_->task.symmetries;
      hybrid_crossing_->task.symmetries = hybrid_->task.symmetries;
      separate_happenings();
      if (!hybrid_->unsupported.empty()) {
        res_.diagnostics.push_back(Diagnostic::error(
            "unsupported.pddl_plus", "Kairos cannot plan yet with " + hybrid_->unsupported));
        return finish(Status::Unsupported, StopReason::Exhausted);
      }
      log_.info(
        LogChannel::Orchestrator, "PDDL+: {} processes and {} events, steps of {} s, {} pairs "
        "of symmetric objects", task.processes.size(), task.events.size(), kHybridDelta,
        hybrid_->task.symmetries.size());
      return std::nullopt;
    }
    temporal_task_ = !task.durative_actions.empty() || !task.timed_events.empty();
    if (temporal_task_) {
      if (o_.engine == Engine::M1) {
        res_.diagnostics.push_back(Diagnostic::error(
            "unsupported.engine", "engine M1 plans without durative actions or timed initial "
            "literals: use auto, m2 or m3"));
        return finish(Status::Unsupported, StopReason::Exhausted);
      }
      auto profile = temporal::analyze(task);
      bool m2 = o_.engine == Engine::M2 ||
        (o_.engine == Engine::Auto && profile.simple && task.timed_events.empty());
      if (m2) {
        if (!profile.simple) {
          res_.diagnostics.push_back(Diagnostic::warning(
              "orchestrator.forced_m2",
              "engine M2 forced on a task that may need actions to overlap: plans may be missed"));
        }
        // One duration of each interval: some plans may need another
        m2_complete_ = profile.simple && !profile.variable_duration;
        compressed_ = temporal::compress(task);
        log_.info(
          LogChannel::Orchestrator, "engine M2: {} durative actions compressed",
          task.durative_actions.size());
      } else {
        snapped_ = temporal::snap(task);
        if (!snapped_->unsupported.empty()) {
          res_.diagnostics.push_back(Diagnostic::error(
              "unsupported.m3", "engine M3 cannot plan this yet: " + snapped_->unsupported));
          return finish(Status::Unsupported, StopReason::Exhausted);
        }
        for (const auto & why : profile.reasons) {
          log_.info(LogChannel::Orchestrator, "engine M3: {}", why);
        }
        log_.info(
          LogChannel::Orchestrator, "engine M3: {} durative actions as snap actions, {} timed "
          "initial literals", task.durative_actions.size(), task.timed_events.size());
      }
    }
    if (temporal_task_ && fixed_.changed && o_.search == "auto") {
      // Durations between their bounds and continuous effects as they are: in steps of time
      // too (the battery patrol recharges once 4 s, not four times 1 s)
      auto both = temporal::hybrid(*translated_, kHybridDelta, temporal::Waits::Both, o_.epsilon);
      if (both.unsupported.empty()) {
        hybrid_ = std::move(both);
        hybrid_short_ = temporal::hybrid(
          *translated_, kHybridDelta, temporal::Waits::Short, o_.epsilon, false);
        hybrid_crossing_ = temporal::hybrid(
          *translated_, kHybridDelta, temporal::Waits::Crossing, o_.epsilon, false);
        hybrid_short_->task.symmetries = hybrid_->task.symmetries;
        hybrid_crossing_->task.symmetries = hybrid_->task.symmetries;
        separate_happenings();
        log_.info(LogChannel::Orchestrator, "durations: also in steps of {} s", kHybridDelta);
      }
    }
    length_metric_ = in_.problem.metric == std::nullopt;
    // Temporal tasks minimizing the makespan (the default): each durative action costs its
    // duration (at the initial state), so the search prefers short and spread work, which the
    // schedule then overlaps
    time_metric_ = temporal_task_ && task.metric.minimize &&
      (length_metric_ || task.metric.expr.tree.op == ground::ExprOp::TotalTime);
    for (const auto & n : task.initial_numeric) {
      initial_numbers_.push_back(n ? *n : state::kUndefined);
    }
    return std::nullopt;
  }

  const sas::Task & search_task() const
  {
    if (hybrid_ && hybrid_primary_) {
      return hybrid_->task;
    }
    return compressed_ ? compressed_->task : (snapped_ ? snapped_->task : *sas_);
  }

  // The cost of each action of a search task, and whether bounds may prune
  std::vector<double> costs_of(
    const sas::Task & of, const temporal::Compressed * comp, const temporal::SnapTask * snap,
    bool & prune) const
  {
    bool action_costs = heuristics::metric_costs_apply(of);
    auto costs = heuristics::action_costs(
      of, action_costs ? heuristics::CostModel::Metric : heuristics::CostModel::Unit);
    if (!time_metric_ && temporal_task_) {
      if (auto mixed = mixed_metric(*sas_)) {
        // Euros plus time: what each action adds to the fluents, plus the weight of time by
        // its duration (their sum bounds the makespan from above; the plans keep the metric)
        std::vector<double> mc(costs.size(), 0.0);
        bool usable = true;
        for (uint32_t a = 0; a < costs.size() && usable; ++a) {
          auto d = weighted_delta(of.actions[a].effects, mixed->weight);
          double dur = 0.0;
          if (comp && a >= comp->instant) {
            dur = state::evaluate(comp->duration[a - comp->instant], initial_numbers_.data());
          } else if (snap && snap->is_start(a)) {
            dur = temporal::bounds(
              sas_->durative_actions[snap->durative(a)], initial_numbers_.data()).lo;
          }
          usable = d && *d >= 0.0 && !std::isnan(dur);
          mc[a] = usable ? *d + mixed->time * std::max(0.0, dur) : 0.0;
        }
        if (usable) {
          prune = true;
          return mc;
        }
      }
    }
    if (time_metric_) {
      for (uint32_t a = 0; a < costs.size(); ++a) {
        double c = 0.001;  // instantaneous actions (and ends) take no time
        if (comp && a >= comp->instant) {
          c = state::evaluate(comp->duration[a - comp->instant], initial_numbers_.data());
        } else if (snap && snap->is_start(a)) {
          c = temporal::bounds(
            sas_->durative_actions[snap->durative(a)], initial_numbers_.data()).lo;
        } else if (snap && a >= snap->tils) {
          c = 0.0;
        }
        costs[a] = std::isnan(c) || c < 0.0 ? 0.001 : c;
      }
      action_costs = true;
    }
    prune = action_costs || length_metric_;
    return costs;
  }

  // The steps of compiled preferences and constraints (normalize: kairos-pref-*) are not the
  // domain's
  void strip_preference_steps(Plan & plan) const
  {
    const auto & f = in_.problem_model.features;
    if (!f.contains(sem::Feature::Preferences) &&
      !f.contains(sem::Feature::TrajectoryConstraints))
    {
      return;
    }
    std::erase_if(
      plan.steps, [](const PlanStep & st) {return st.name.starts_with("kairos-pref-");});
    if (!plan.temporal) {
      for (size_t i = 0; i < plan.steps.size(); ++i) {
        plan.steps[i].time = static_cast<double>(i);
      }
    }
  }

  ValidationReport validate(const Plan & plan)
  {
    ValidateOptions vo;
    vo.epsilon = o_.epsilon;
    auto tv = Clock::now();
    auto report = val::validate(in_.domain, in_.domain_model, in_.problem, in_.problem_model,
      plan, vo);
    log_.debug(LogChannel::Orchestrator, "plan of {} steps, makespan {}: {} in {:.1f} ms",
      plan.steps.size(), plan.makespan(), report.valid() ? "valid" : "invalid",
      std::chrono::duration<double, std::milli>(Clock::now() - tv).count());
    if (!report.valid()) {
      log_.debug(LogChannel::Orchestrator, "{}\n{}",
        report.diagnostics.empty() ? std::string("?") : to_string(report.diagnostics[0]),
        to_string(plan, PlanFormat::Ipc, 6));
    }
    return report;
  }

  // The last plan check_plan validated, in this thread (the search's: offer reuses it)
  static thread_local inline std::optional<std::pair<std::vector<uint32_t>, ValidationReport>>
  checked_;

  // A plan of a phase: validated against the original task; only better ones (by the metric,
  // or the makespan or length) are kept and handed over
  // Tentative: an invalid plan is not an error (false: the caller has another one). True
  // when the plan was valid, kept or not
  bool offer(
    std::optional<Plan> plan, const std::vector<uint32_t> & actions, const sas::Task & from,
    bool tentative = false)
  {
    if (!plan) {
      if (!tentative) {
        ++invalid_;
        log_.error(LogChannel::Orchestrator, "internal error: a plan could not be scheduled");
      }
      return false;
    }
    strip_preference_steps(*plan);
    // PDDL+: check_plan validated it already
    auto report = checked_ && checked_->first == actions ? checked_->second : validate(*plan);
    if (!report.valid()) {
      if (tentative) {
        return false;
      }
      ++invalid_;
      log_.error(
        LogChannel::Orchestrator, "internal error: a plan failed validation: {}",
        report.diagnostics.empty() ? std::string("?") : to_string(report.diagnostics[0]));
      return false;
    }
    // Without a metric: the makespan of temporal plans, the length of sequential ones
    plan->cost = report.metric ? *report.metric :
      (plan->temporal ? plan->makespan() : static_cast<double>(plan->steps.size()));
    bool minimize = !in_.problem.metric || in_.problem.metric->minimize;  // as written
    // Kept and handed over under the lock: with several threads, in order and numbered
    std::lock_guard<std::mutex> lock(mutex_);
    if (res_.plan && res_.plan->cost &&
      (minimize ? *plan->cost >= *res_.plan->cost : *plan->cost <= *res_.plan->cost))
    {
      return true;  // cheaper for the search, not better by the metric
    }
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - t0_);
    auto index = static_cast<uint32_t>(res_.stats.plans++);
    res_.plan = *plan;
    best_actions_ = actions;
    best_task_ = &from;
    if (!res_.stats.first_plan_ms) {
      res_.stats.first_plan_ms = ms_since(t0_);
    }
    last_plan_ = elapsed.count();
    log_.info(
      LogChannel::Search, "plan {} with {} actions, cost {}", index, plan->steps.size(),
      *plan->cost);
    if (on_plan_) {
      on_plan_(*plan, index, elapsed);
    }
    if (o_.target_cost && *plan->cost <= *o_.target_cost) {
      target_reached_ = true;
      internal_.request_stop();
    }
    if (!o_.anytime) {
      // The first plan is all that was asked: the other phases stop (in parallel they would
      // go on until the timeout, and in order search again)
      first_only_ = true;
      internal_.request_stop();
    }
    return true;
  }

  // The phases, from the task, the engine and the options (see Phase)
  void plan_phases()
  {
    search_start_ = Clock::now();
    bool portfolio = o_.search == "auto";
    auto m3_portfolio = [&](bool makespan_g) {
        if (snapped_ && has_numeric_conditions(snapped_->task)) {
          // Resources (battery, ammunition): novelty first, as in M2 (PlanSys2's combat s3:
          // lazy LAMA needs 4000 expansions and 3 s, eager search with novelty 40 and 50 ms)
          phases_.push_back({Kind::Snap, "lama-novelty", 0.3, true, makespan_g});
          phases_.push_back({Kind::Snap, "lama", 0.4, true, makespan_g});
          phases_.push_back({Kind::Snap, "eager-lama", 1.0, true, makespan_g});
          return;
        }
        phases_.push_back({Kind::Snap, "lama", 0.2, true, makespan_g});
        phases_.push_back({Kind::Snap, "eager-lama", 0.375, true, makespan_g});
        phases_.push_back({Kind::Snap, "lama-novelty", 1.0, true, makespan_g});
      };
    if (compressed_ && portfolio && o_.anytime && o_.engine == Engine::Auto && time_metric_) {
      improver_ = temporal::snap(*sas_);
      if (!improver_->unsupported.empty()) {
        improver_.reset();
      }
    }
    if (snapped_) {
      if (portfolio) {
        m3_portfolio(time_metric_ && o_.anytime);
      } else {
        phases_.push_back({Kind::Snap, o_.search, 1.0, o_.anytime, false});
      }
    } else {
      Kind kind = compressed_ ? Kind::Compressed : Kind::Plain;
      double share = improver_ ? 0.0 : 1.0;  // M2 before M3: until its first plan
      bool improve = o_.anytime && !improver_;
      const sas::Task & first_task = compressed_ ? compressed_->task :
        (hybrid_primary_ ? hybrid_->task : *sas_);
      if (portfolio && search::goals_apply(first_task)) {
        // Many goal facts (patrolling, visiting, delivering): one at a time first, where one
        // search for all meets plateaus that grow with the agents (B6 patrol L: no plan in
        // 100 s, 0.1 s this way); the searches below if it fails
        log_.info(LogChannel::Orchestrator, "search: incremental goals first (auto)");
        phases_.push_back({kind, "goals", 0.5, improve, false});
      }
      if (portfolio && hybrid_primary_) {
        // PDDL+: novelty crosses the plateaus of waiting (car: 40 ms, lazy LAMA 0.5 s), lazy
        // LAMA finds others at once (agile robot, intercept); waits until a condition changes
        // cross long stretches (a generator that runs for 1000 s) but mislead elsewhere
        // (agile robot): short turns of each
        using W = temporal::Waits;
        if (hybrid_->durative > 0) {
          // Durative actions (a generator that runs for 1000 s): long waits first, and short
          // ones only to cross a change
          log_.info(LogChannel::Orchestrator, "search: lama with long waits, lama-novelty, lama, "
            "lama-novelty with long waits (auto)");
          phases_.push_back({kind, "lama", 0.1, improve, false, false, W::Crossing});
          phases_.push_back({kind, "lama-novelty", 0.2, improve, false, true, W::Crossing});
          phases_.push_back({kind, "lama-novelty", 0.02, improve, false, true, W::Short});
        } else {
          log_.info(LogChannel::Orchestrator, "search: lama-novelty, lama, then both with long "
            "waits (auto)");
          phases_.push_back({kind, "lama-novelty", 0.02, improve, false, false, W::Short});
        }
        phases_.push_back({kind, "lama", 0.05, improve, false, true, W::Short});
        if (hybrid_->durative == 0) {
          phases_.push_back({kind, "lama", 0.1, improve, false, true, W::Both});
        }
        phases_.push_back({kind, "lama-novelty", share, improve, false, true, W::Both});
      } else if (portfolio && compressed_ && resolve_auto(first_task, false) == "lama") {
        // Temporal without numeric conditions: when lmcount misleads (airport: lazy LAMA finds
        // nothing in 30 s, FF alone a plan in 1.4 s), FF after a quarter of the time
        log_.info(LogChannel::Orchestrator, "search: lama, then ff (auto)");
        phases_.push_back({kind, "lama", 0.25, improve, false});
        phases_.push_back({kind, "ff", share, improve, false, true});
      } else if (portfolio && resolve_auto(first_task, false) == "lama-novelty") {
        // Numeric: lazy LAMA finds most first plans at once (cooking, factory); eager search
        // with novelty, the ones it misses
        log_.info(LogChannel::Orchestrator, "search: lama, then lama-novelty (auto)");
        phases_.push_back({kind, "lama", 0.25, improve, false});
        if (improver_) {
          // Before M3: improving in M2 (each plan rescheduled) also shortens the makespan of
          // large tasks, where M3 finds nothing better in time
          phases_.push_back({kind, "lama-novelty", 0.4, true, false});
        } else {
          phases_.push_back({kind, "lama-novelty", share, improve, false, !improve});
        }
      } else if (portfolio && kind == Kind::Plain && resolve_auto(first_task, false) == "lama") {
        // Classical: LAMA, and if it has no plan, eager search with novelty for the last part
        // (blocksworld, depots, sokoban, recharging-robots: 0.2-2.5 s where lazy LAMA finds
        // nothing in 30 s). Novelty first slowed what LAMA solves at once (woodworking: 55
        // expansions, 1.5 s with novelty first) and, as a share of a long limit, waited long
        log_.info(LogChannel::Orchestrator, "search: lama, then novelty (auto)");
        phases_.push_back({kind, "lama", 0.85, improve, false});
        phases_.push_back({kind, "novelty", share, false, false, true});
      } else {
        phases_.push_back({kind, o_.search, share, improve, false});
      }
      if (improver_) {
        m3_portfolio(true);
      }
    }
    if (hybrid_ && !hybrid_primary_ && !phases_.empty()) {
      // Steps of time before the last phase of M2 or M3, if they have no plan yet: plans that
      // need durations in between their bounds
      using W = temporal::Waits;
      auto at = phases_.end() - 1;
      at = phases_.insert(
        at, {Kind::Steps, "lama-novelty", 0.1, o_.anytime, false, true, W::Crossing});
      phases_.insert(at + 1, {Kind::Steps, "lama", 0.15, o_.anytime, false, true, W::Both});
    }
    snap_task_ = snapped_ ? &*snapped_ : (improver_ ? &*improver_ : nullptr);
    if (!time_metric_ && !heuristics::metric_costs_apply(search_task()) && !length_metric_ &&
      !(temporal_task_ && mixed_metric(*sas_)))
    {
      res_.diagnostics.push_back(Diagnostic::note(
          "orchestrator.metric", "the search minimizes the number of actions; the metric of "
          "the problem is reported for each plan but does not guide the search yet"));
    }
  }

  // Stops the search when no plan improves for options.stop_after_no_improvement
  void start_watchdog()
  {
    if (!o_.stop_after_no_improvement) {
      return;
    }
    auto window = o_.stop_after_no_improvement->count();
    watchdog_ = std::jthread([this, window](std::stop_token st) {
          // Wakes at once when the search ends, so small tasks do not wait for it
          std::mutex m;
          std::condition_variable_any wake;
          std::unique_lock<std::mutex> lock(m);
          while (!st.stop_requested()) {
            wake.wait_for(lock, st, std::chrono::milliseconds(5), [] {return false;});
            int64_t last = last_plan_.load();
            auto now = std::chrono::duration_cast<std::chrono::milliseconds>(
              Clock::now() - t0_).count();
            if (last >= 0 && now - last >= window) {
              no_improvement_ = true;
              internal_.request_stop();
              return;
            }
          }
        });
  }

  // One phase: its task, budget, costs and search configuration, and the anytime search
  search::AnytimeResult run_phase(const Phase & ph, bool last)
  {
    bool steps = hybrid_ && (ph.kind == Kind::Steps || (ph.kind == Kind::Plain && hybrid_primary_));
    const temporal::Hybrid * hy = !steps ? nullptr :
      (ph.waits == temporal::Waits::Both ? &*hybrid_ :
      (ph.waits == temporal::Waits::Short ? &*hybrid_short_ : &*hybrid_crossing_));
    const sas::Task & task_k = ph.kind == Kind::Compressed ? compressed_->task :
      (ph.kind == Kind::Snap ? snap_task_->task : (hy ? hy->task : *sas_));
    // Its own deadline, unless it is the last phase (then the global budget)
    std::optional<Budget> own;
    if ((ph.share < 1.0 && !last) || lifted_.joinable()) {
      BudgetConfig pc = bc_;
      if (ph.share < 1.0 && !last) {
        auto left = (bc_.deadline ? *bc_.deadline : t0_ + o_.timeout) - Clock::now();
        pc.deadline = Clock::now() + std::chrono::duration_cast<Clock::duration>(left * ph.share);
        if (ph.share == 0.0) {
          pc.deadline = bc_.deadline;  // until its first plan (it does not improve)
        }
      }
      if (lifted_.joinable()) {
        // Beside the lifted engine: out of memory first, which then goes on alone
        pc.memory_limit_bytes = bc_.memory_limit_bytes / 4 * 3;
      }
      own.emplace(pc);
    }
    Budget & bk = own ? *own : *budget_;
    bool prune = false;
    // Steps of time: unit costs (the durations are waits there)
    auto costs = ph.kind == Kind::Steps ?
      std::vector<double>(task_k.actions.size(), 1.0) :
      costs_of(task_k, ph.kind == Kind::Compressed ? &*compressed_ : nullptr,
        ph.kind == Kind::Snap ? snap_task_ : nullptr, prune);
    std::string chosen = ph.search == "auto" ?
      resolve_auto(task_k, ph.kind == Kind::Snap) : ph.search;
    if (ph.search == "auto") {
      log_.info(LogChannel::Orchestrator, "search: {} (auto)", chosen);
    }
    log_.info(
      LogChannel::Orchestrator, "{}: {} variables, {} actions, search {}, costs {}",
      ph.kind == Kind::Compressed ? "engine M2" :
      (ph.kind == Kind::Snap ? "engine M3" : "search"), task_k.variables.size(),
      task_k.actions.size(), chosen, ph.makespan_g ? "makespan" :
      (time_metric_ ? "durations" : (prune && !length_metric_ ? "from the metric" : "unit")));
    state::StateRegistry registry(task_k, &bk);
    state::StateSpace space(task_k, registry);
    std::unique_ptr<temporal::TemporalFilter> filter;
    if (ph.kind == Kind::Snap) {
      filter = std::make_unique<temporal::TemporalFilter>(*sas_, *snap_task_, space, o_.epsilon);
    }
    search::PlanSink sink([&](const search::PlanSink::Entry & e) {
        if (ph.kind == Kind::Compressed) {
          // Overlap on resources as far as the plan so far stays valid (goal aside); the plain
          // schedule if the plan is not valid in the end (offer validates it: once)
          auto prefix_valid = [this](const Plan & p) {
            ValidateOptions vo;
            vo.epsilon = o_.epsilon;
            vo.check_goal = false;
            return val::validate(in_.domain, in_.domain_model, in_.problem,
                in_.problem_model, p, vo).valid();
          };
          if (!offer(temporal::schedule(*sas_, *compressed_, space, e.actions, o_.epsilon,
            prefix_valid), e.actions, task_k, true))
          {
            offer(temporal::schedule(*sas_, *compressed_, space, e.actions, o_.epsilon),
              e.actions, task_k);
          }
        } else if (filter) {
          offer(filter->plan(e.actions), e.actions, task_k);
        } else if (hy) {
          offer(hy->plan(e.actions, space), e.actions, task_k);
        } else {
          offer(search::to_plan(*sas_, e.actions), e.actions, task_k);
        }
      });
    search::SearchContext ctx(task_k, registry, space, costs, sink, &bk);
    if (ph.kind == Kind::Snap) {
      ctx.heuristic_task = &snap_task_->guide;
    }
    // M2: preferred steps go to the agent that is free first in a schedule of the partial
    // plan (bt_example's first plan had one robot doing everything)
    std::optional<temporal::PartialSchedule> partial;
    if (ph.kind == Kind::Compressed) {
      partial.emplace(*compressed_, space);
      log_.debug(LogChannel::Orchestrator, "partial schedule: {}",
            partial->active() ? "on" : "off");
      if (partial->active()) {
        ctx.tie_key = [&partial](state::StateId p, uint32_t a) {return partial->end(p, a);};
        ctx.on_reach = [&partial](state::StateId p, uint32_t a, state::StateId c) {
            partial->reach(p, a, c);
          };
      }
    }
    if (hy && hy->has_guide) {
      ctx.heuristic_task = &hy->guide;
    }
    if (hy) {
      // Events first; the global constraints in every state
      ctx.accept = [hy, &space](state::StateId, uint32_t, state::StateId c) {
          return hy->accept(space.values(c).data(), space.numbers(c).data());
        };
      // The discrete semantics may reach a goal the continuous one does not: search on
      ctx.check_plan = [this, hy, &space](const std::vector<uint32_t> & actions) {
          Plan p = hy->plan(actions, space);
          strip_preference_steps(p);
          auto report = validate(p);
          bool ok = report.valid();
          checked_ = std::make_pair(actions, std::move(report));
          return ok;
        };
    } else if (!task_k.always.empty()) {
      ctx.accept = [&task_k, &space](state::StateId, uint32_t, state::StateId c) {
          return state::holds(task_k.always, task_k, space.values(c).data(),
                 space.numbers(c).data());
        };
    }
    if (filter) {
      ctx.accept = [&filter](state::StateId p, uint32_t a, state::StateId c) {
          return filter->accept(p, a, c);
        };
      if (ph.makespan_g) {
        ctx.path_cost = [&filter, &costs](
          state::StateId p, uint32_t a, state::StateId c, double g) {
            return filter->makespan(p, a, c, g + costs[a]);
          };
      }
    }
    ctx.logger = log_;
    ctx.prune_with_bound = prune;
    search::AnytimeOptions ao = *preset(chosen);
    ao.improve = ph.improve;
    // Durations are what the search minimizes there: nothing to adapt (timed initial
    // literals cost 0 but are forced)
    ao.adapt_zero_costs = !time_metric_;
    // Unit costs for the first plan only in M2 (durations as costs): not on snap actions,
    // where ends cost 0.001 and starts their duration, an unevenness that guides
    // (turn-and-open lost an instance), nor on action costs of classical tasks (parking 7 -> 3)
    ao.unit_first = ph.kind == Kind::Compressed;
    // With numeric conditions cost + 1 instead: unit costs slowed the large cooking example
    // (150 ms -> 0.7 s), real costs lost temporal numeric instances (transport, rovers)
    ao.plus_one_first = has_numeric_conditions(task_k);
    search::AnytimeResult result;
    if (chosen == "goals") {
      // Then improved as the other searches; if it fails, a lazy search when it is the only
      // phase (asked for), else the next phases go on
      auto r = search::incremental_goals(ctx);
      ao.skip_first = r == search::Outcome::Solved;
      if (r == search::Outcome::Stopped) {
        result = search::AnytimeResult::Stopped;
      } else if (r == search::Outcome::Solved || phases_.size() == 1) {
        result = search::anytime(ctx, ao);
      } else {
        result = search::AnytimeResult::Failed;
      }
    } else {
      result = search::anytime(ctx, ao);
    }
    std::lock_guard<std::mutex> lock(mutex_);
    stats_.expanded += ctx.stats.expanded;
    stats_.generated += ctx.stats.generated;
    stats_.evaluated += ctx.stats.evaluated;
    states_ += registry.size();
    bytes_per_state_ = registry.bytes_per_state() + 32.0;  // and the search node
    if (res_.plan && best_task_ == &task_k) {
      dump(o_, "plan-trace", "plan-trace.txt", [&]() {
          return plan_trace(task_k, best_actions_);
        }, log_);
    }
    return result;
  }

  // With several threads: the phases at once, each with the whole budget (a pool of
  // options.threads threads takes them in order). A proof or an optimum stops the others.
  // Not deterministic: which phase finds a plan first depends on the machine.
  void run_phases_parallel(size_t first = 0)
  {
    std::vector<search::AnytimeResult> results(phases_.size(), search::AnytimeResult::Failed);
    std::atomic<size_t> next{first};
    std::atomic<bool> optimal{false};
    auto worker = [&]() {
        for (size_t k; (k = next++) < phases_.size(); ) {
          const Phase & ph = phases_[k];
          if (budget_->check() != BudgetState::Ok || target_reached_ || no_improvement_) {
            return;
          }
          if (ph.only_without_plan && has_plan()) {
            continue;
          }
          auto result = run_phase(ph, true);
          results[k] = result;
          if (result == search::AnytimeResult::Exhausted && !has_plan() && !bounded_durations()) {
            bool proof = ph.kind == Kind::Plain ||
              (ph.kind == Kind::Compressed && m2_complete_);
            (proof ? proof_ : m3_exhausted_) = true;
            internal_.request_stop();
          }
          if (result == search::AnytimeResult::ProvenOptimal && ph.kind == Kind::Plain) {
            optimal = true;
            internal_.request_stop();
          }
        }
      };
    {
      std::vector<std::jthread> pool;
      for (unsigned i = 0; i < std::min<size_t>(o_.threads, phases_.size() - first); ++i) {
        pool.emplace_back(worker);
      }
    }
    // As the sequential loop would see it: an optimum, else a phase that exhausted its space,
    // else a stop, else how the last phase ended
    results.erase(results.begin(), results.begin() + static_cast<std::ptrdiff_t>(first));
    outcome_ = results.empty() ? search::AnytimeResult::Failed : results.back();
    auto any = [&](search::AnytimeResult r) {
        return std::find(results.begin(), results.end(), r) != results.end();
      };
    if (optimal) {
      outcome_ = search::AnytimeResult::ProvenOptimal;
    } else if (any(search::AnytimeResult::Exhausted)) {
      outcome_ = search::AnytimeResult::Exhausted;
    } else if (any(search::AnytimeResult::Stopped)) {
      outcome_ = search::AnytimeResult::Stopped;
    }
  }

  // The phases in order, until the budget, a target, no improvement or a proof
  void run_phases()
  {
    best_task_ = &search_task();
    start_watchdog();
    if (o_.threads > 1 && phases_.size() > 1) {
      // Incremental goals alone first: it needs little, the others at once would take the
      // memory it needs (B6 patrol XL: out of 2 GB with 4 threads, solved in 3.6 s alone)
      size_t first = 0;
      if (phases_[0].search == "goals") {
        outcome_ = run_phase(phases_[0], false);
        first = 1;
      }
      if (first == 0 || (budget_->check() == BudgetState::Ok && !target_reached_ &&
        !no_improvement_ && !first_only_))
      {
        run_phases_parallel(first);
      }
      if (watchdog_.joinable()) {
        watchdog_.request_stop();
        watchdog_.join();
      }
      return;
    }
    for (size_t k = 0; k < phases_.size(); ++k) {
      const Phase & ph = phases_[k];
      if (budget_->check() != BudgetState::Ok || target_reached_ || no_improvement_) {
        break;
      }
      if (ph.only_without_plan && res_.plan) {
        continue;
      }
      auto result = run_phase(ph, k + 1 == phases_.size());
      outcome_ = result;
      if (result == search::AnytimeResult::Exhausted && !res_.plan && !bounded_durations()) {
        // No plan in its whole space: a proof for the plain task and a complete M2; strong
        // evidence for M3 (one partial plan per state)
        proof_ = ph.kind == Kind::Plain || (ph.kind == Kind::Compressed && m2_complete_);
        m3_exhausted_ = ph.kind == Kind::Snap;
        if (proof_ || m3_exhausted_) {
          break;
        }
      }
      if (result == search::AnytimeResult::ProvenOptimal && ph.kind == Kind::Plain) {
        break;  // nothing better exists
      }
    }
    if (watchdog_.joinable()) {
      watchdog_.request_stop();
      watchdog_.join();
    }
  }

  // Statistics, and the status and stop reason from how the phases ended
  SolveResult conclude()
  {
    res_.stats.search_ms = ms_since(search_start_);
    res_.stats.expanded = stats_.expanded;
    res_.stats.generated = stats_.generated;
    res_.stats.evaluated = stats_.evaluated;
    res_.stats.states = states_;
    res_.stats.bytes_per_state = bytes_per_state_;
    if (res_.plan) {
      bool stopped = outcome_ == search::AnytimeResult::Stopped ||
        budget_->check() != BudgetState::Ok;
      StopReason reason = StopReason::NoImprovement;
      if (target_reached_) {
        reason = StopReason::TargetReached;
      } else if (no_improvement_ || first_only_) {
        reason = StopReason::NoImprovement;
      } else if (outcome_ == search::AnytimeResult::ProvenOptimal) {
        // In M2 and M3 the search proves the sum of durations optimal, not the makespan
        reason = temporal_task_ ? StopReason::Exhausted : StopReason::ProvenOptimal;
      } else if (stopped) {
        reason = budget_status().second;
      } else if (outcome_ == search::AnytimeResult::Exhausted && temporal_task_) {
        reason = StopReason::Exhausted;
      }
      return finish(Status::Solved, reason);
    }
    if (invalid_ > 0) {
      res_.diagnostics.push_back(Diagnostic::error(
          "orchestrator.invalid_plans", "every plan found failed validation (a bug in Kairos)"));
      return finish(Status::InternalError, StopReason::Exhausted);
    }
    if (proof_ || lifted_exhausted_) {
      return finish(Status::Unsolvable, StopReason::Exhausted);
    }
    if (m3_exhausted_) {
      res_.diagnostics.push_back(Diagnostic::note(
          "orchestrator.m3_exhausted", "engine M3 found no plan; as POPF, it keeps one "
          "partial plan per state, so this is strong evidence but not a proof"));
      return finish(Status::Unsolvable, StopReason::Exhausted);
    }
    if (outcome_ == search::AnytimeResult::Exhausted) {
      res_.diagnostics.push_back(Diagnostic::error(
          "unsupported.m2_incomplete",
          "engine M2 found no plan, which does not prove there is none for this task "
          "(variable durations or possible required concurrency)"));
      return finish(Status::Unsupported, StopReason::Exhausted);
    }
    if (outcome_ == search::AnytimeResult::Stopped || budget_->check() != BudgetState::Ok) {
      auto [st, r] = budget_status();
      return finish(st, r);
    }
    res_.diagnostics.push_back(Diagnostic::error(
        "orchestrator.no_plan", "the search ended without a plan and without a proof"));
    return finish(Status::InternalError, StopReason::Exhausted);
  }

  // Inputs
  const Input & in_;
  const SearchOptions & o_;
  const PlanCallback & on_plan_;
  Clock::time_point t0_;
  SolveResult res_;
  Logger log_;
  std::stop_source internal_;               // stop(), the caller's token, targets, watchdog
  std::stop_callback<ForwardStop> forward_;  // the caller's token into internal_
  BudgetConfig bc_;
  std::optional<Budget> budget_;
  // The task
  std::optional<sas::Task> translated_;
  temporal::FixedDurations fixed_;
  const sas::Task * sas_ = nullptr;          // translated_, or fixed_.task if it changed
  // The engine
  std::optional<temporal::Compressed> compressed_;
  std::optional<temporal::SnapTask> snapped_;
  std::optional<temporal::SnapTask> improver_;  // M3 after M2
  const temporal::SnapTask * snap_task_ = nullptr;
  bool m2_complete_ = true;  // exhausting the compressed task proves there is no plan
  bool temporal_task_ = false;
  // PDDL+: processes and events as discrete steps
  bool hybrid_primary_ = false;  // PDDL+: the only engine (else beside M2 or M3, Kind::Steps)
  std::optional<temporal::Hybrid> hybrid_;
  std::optional<temporal::Hybrid> hybrid_short_;  // the same without long waits
  std::optional<temporal::Hybrid> hybrid_crossing_;  // short waits only to cross a change
  bool continuous_steps_ = false;  // continuous effects M2 and M3 cannot plan: as PDDL+
  static constexpr double kHybridDelta = 0.1;
  bool length_metric_ = false;
  bool time_metric_ = false;
  std::vector<double> initial_numbers_;
  std::vector<Phase> phases_;
  // The plans
  std::mutex mutex_;
  std::vector<uint32_t> best_actions_;
  const sas::Task * best_task_ = nullptr;
  std::atomic<bool> target_reached_{false};
  std::atomic<int64_t> last_plan_{-1};  // ms since t0
  size_t invalid_ = 0;
  std::atomic<bool> no_improvement_{false};
  std::atomic<bool> first_only_{false};  // without anytime, the first plan stopped the search
  std::jthread watchdog_;
  // How the phases ended
  Clock::time_point search_start_;
  search::AnytimeResult outcome_ = search::AnytimeResult::Failed;
  search::SearchStats stats_;
  size_t states_ = 0;
  double bytes_per_state_ = 0.0;
  std::atomic<bool> proof_{false};  // a complete phase exhausted its space without a plan
  std::atomic<bool> m3_exhausted_{false};
  std::jthread lifted_;  // last: stopped and joined before the members it uses go
};

}  // namespace

SolveResult run(
  const Input & in, const SearchOptions & options, std::stop_token stop,
  const PlanCallback & on_plan)
{
  return Run(in, options, std::move(stop), on_plan).execute();
}

}  // namespace kairos::orch
