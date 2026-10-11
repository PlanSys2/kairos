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

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <exception>
#include <fstream>
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
#include "normalize/normalizer.hpp"
#include "pddl/printer.hpp"
#include "sas/translate.hpp"
#include "search/search.hpp"
#include "temporal/compress.hpp"
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

// What each later phase will bring
struct Missing
{
  sem::Feature feature;
  const char * code;
  const char * what;
};

const Missing kMissing[] = {
  {sem::Feature::ContinuousEffects, "unsupported.continuous_effects",
    "continuous effects (phase F3)"},
  {sem::Feature::Processes, "unsupported.processes", "processes (PDDL+, phase F5)"},
  {sem::Feature::Events, "unsupported.events", "events (PDDL+, phase F5)"},
  {sem::Feature::Preferences, "unsupported.preferences", "preferences (PDDL3, phase F4)"},
  {sem::Feature::TrajectoryConstraints, "unsupported.trajectory_constraints",
    "trajectory constraints (PDDL3, phase F4)"},
  {sem::Feature::GlobalConstraints, "unsupported.global_constraints",
    "global constraints (phase F5)"},
};

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
  for (const auto & m : kMissing) {
    if (features.contains(m.feature)) {
      out.push_back(
        Diagnostic::error(m.code, std::string("Kairos cannot plan yet with ") + m.what));
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
  } else {
    return std::nullopt;
  }
  return ao;
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

// "auto": LAMA (hFF and landmarks, lazy) on classical tasks, with novelty on numeric ones
std::string resolve_auto(const sas::Task & task)
{
  return has_numeric_conditions(task) ? "lama-novelty" : "lama";
}

}  // namespace

std::vector<std::string> presets()
{
  return {"ff", "lama", "eager-lama", "novelty", "lama-novelty", "ehc"};
}

SolveResult run(
  const Input & in, const SearchOptions & options, std::stop_token stop,
  const PlanCallback & on_plan)
{
  auto t0 = Clock::now();
  SolveResult res;
  Logger log(options.log);
  auto finish = [&](Status status, StopReason reason) {
      res.status = status;
      res.stop_reason = reason;
      res.stats.total_ms = ms_since(t0);
      if (auto rss = current_rss_bytes()) {
        res.stats.peak_memory_bytes = *rss;
      }
      log.info(
        LogChannel::Orchestrator, "{} ({}) in {:.1f} ms", to_string(status), to_string(reason),
        res.stats.total_ms);
      return res;
    };
  try {
    res.diagnostics = unsupported(in.problem_model.features, options.engine);
    if (!res.diagnostics.empty()) {
      return finish(Status::Unsupported, StopReason::Exhausted);
    }
    // "auto" is resolved once the SAS+ task is known
    auto configured = preset(options.search == "auto" ? "lama" : options.search);
    if (!configured) {
      res.diagnostics.push_back(Diagnostic::error(
          "orchestrator.unknown_search", "unknown search configuration '" + options.search +
          "'"));
      return finish(Status::InvalidInput, StopReason::Exhausted);
    }
    if (options.threads > 1) {
      res.diagnostics.push_back(Diagnostic::note(
          "orchestrator.threads", "parallel search is not available yet: using one thread"));
    }
    // Budget: the caller's stop token, the timeout and the memory limit
    size_t memory = options.memory_limit_mb ? options.memory_limit_mb * (size_t{1} << 20) :
      default_memory_limit(read_memory_snapshot());
    res.stats.memory_limit_bytes = memory;
    log.info(LogChannel::Memory, "memory limit {} MB", memory >> 20);
    std::stop_source internal;
    std::stop_callback forward(stop, [&internal]() {internal.request_stop();});
    BudgetConfig bc;
    bc.stop = internal.get_token();
    bc.deadline = t0 + options.timeout;
    bc.memory_limit_bytes = memory;
    Budget budget(bc);
    auto budget_status = [&]() {
        switch (budget.state()) {
          case BudgetState::Timeout: return std::make_pair(Status::Timeout, StopReason::Timeout);
          case BudgetState::OutOfMemory:
            return std::make_pair(Status::OutOfMemory, StopReason::MemoryLimit);
          default: return std::make_pair(Status::Cancelled, StopReason::Stopped);
        }
      };

    dump(options, "ast", "domain.pddl", [&]() {return pddl::to_pddl(in.domain);}, log);
    dump(options, "ast", "problem.pddl", [&]() {return pddl::to_pddl(in.problem);}, log);

    // Normalize
    auto t = Clock::now();
    lifted::Task task = lifted::normalize(in.domain, in.domain_model, in.problem,
        in.problem_model);
    res.stats.normalize_ms = ms_since(t);
    dump(options, "lifted", "lifted.txt", [&]() {return lifted::to_string(task);}, log);

    // Ground
    t = Clock::now();
    ground::GroundOptions go;
    go.logger = log;
    auto grounded = ground::ground(task, go, & budget);
    res.stats.ground_ms = ms_since(t);
    if (!grounded.ok()) {
      res.diagnostics = grounded.take_diagnostics();
      if (res.diagnostics.front().code == "ground.unsupported") {
        return finish(Status::Unsupported, StopReason::Exhausted);
      }
      auto [s, r] = budget_status();
      return finish(s == Status::OutOfMemory ? Status::GroundingTooLarge : s, r);
    }
    res.stats.atoms = grounded->atoms.size();
    res.stats.actions = grounded->actions.size();
    dump(options, "ground", "ground.txt", [&]() {return ground::to_string(*grounded);}, log);
    if (grounded->goal_unreachable) {
      res.diagnostics.push_back(Diagnostic::note(
          "orchestrator.unreachable_goal", "the goal needs atoms that no action can reach"));
      return finish(Status::Unsolvable, StopReason::Exhausted);
    }

    // SAS+
    t = Clock::now();
    sas::TranslateOptions so;
    so.logger = log;
    sas::TranslateStats sst;
    auto translated = sas::translate(task, std::move(*grounded), so, &budget, &sst);
    res.stats.sas_ms = ms_since(t);
    if (!translated.ok()) {
      res.diagnostics = translated.take_diagnostics();
      if (res.diagnostics.front().code == "sas.unstratified") {
        return finish(Status::InvalidInput, StopReason::Exhausted);
      }
      auto [s, r] = budget_status();
      return finish(s, r);
    }
    const sas::Task & sas_task = *translated;
    res.stats.variables = sas_task.variables.size();
    res.stats.mutex_groups = sst.group_variables;
    dump(options, "sas", "sas.txt", [&]() {return sas::to_string(sas_task);}, log);

    // Temporal tasks: M2 compresses durative actions when no plan needs them to overlap; M3
    // searches start and end snap actions over a temporal network otherwise (as POPF)
    std::optional<temporal::Compressed> compressed;
    std::optional<temporal::SnapTask> snapped;
    bool m2_complete = true;  // exhausting the compressed task proves there is no plan
    bool temporal_task = !sas_task.durative_actions.empty() || !sas_task.timed_events.empty();
    if (temporal_task) {
      if (options.engine == Engine::M1) {
        res.diagnostics.push_back(Diagnostic::error(
            "unsupported.engine", "engine M1 plans without durative actions or timed initial "
            "literals: use auto, m2 or m3"));
        return finish(Status::Unsupported, StopReason::Exhausted);
      }
      auto profile = temporal::analyze(sas_task);
      bool m2 = options.engine == Engine::M2 ||
        (options.engine == Engine::Auto && profile.simple && sas_task.timed_events.empty());
      if (m2) {
        if (!profile.simple) {
          res.diagnostics.push_back(Diagnostic::warning(
              "orchestrator.forced_m2",
              "engine M2 forced on a task that may need actions to overlap: plans may be missed"));
        }
        // One duration of each interval: some plans may need another
        m2_complete = profile.simple && !profile.variable_duration;
        compressed = temporal::compress(sas_task);
        log.info(
          LogChannel::Orchestrator, "engine M2: {} durative actions compressed",
          sas_task.durative_actions.size());
      } else {
        snapped = temporal::snap(sas_task);
        if (!snapped->unsupported.empty()) {
          res.diagnostics.push_back(Diagnostic::error(
              "unsupported.m3", "engine M3 cannot plan this yet: " + snapped->unsupported));
          return finish(Status::Unsupported, StopReason::Exhausted);
        }
        for (const auto & why : profile.reasons) {
          log.info(LogChannel::Orchestrator, "engine M3: {}", why);
        }
        log.info(
          LogChannel::Orchestrator, "engine M3: {} durative actions as snap actions, {} timed "
          "initial literals", sas_task.durative_actions.size(), sas_task.timed_events.size());
      }
    }
    const sas::Task & search_task =
      compressed ? compressed->task : (snapped ? snapped->task : sas_task);

    // Search (M1, or M1 on the compressed task)
    bool action_costs = heuristics::metric_costs_apply(search_task);
    bool length_metric = in.problem.metric == std::nullopt;
    auto costs = heuristics::action_costs(
      search_task, action_costs ? heuristics::CostModel::Metric : heuristics::CostModel::Unit);
    // Temporal tasks minimizing the makespan (the default): each durative action costs its
    // duration (at the initial state), so the search prefers short and spread work, which the
    // schedule then overlaps
    bool time_metric = temporal_task && sas_task.metric.minimize &&
      (length_metric || sas_task.metric.expr.tree.op == ground::ExprOp::TotalTime);
    if (time_metric) {
      std::vector<double> initial;
      for (const auto & n : sas_task.initial_numeric) {
        initial.push_back(n ? *n : state::kUndefined);
      }
      for (uint32_t a = 0; a < costs.size(); ++a) {
        double c = 0.001;  // instantaneous actions (and ends) take no time
        if (compressed && a >= compressed->instant) {
          c = state::evaluate(compressed->duration[a - compressed->instant], initial.data());
        } else if (snapped && snapped->is_start(a)) {
          c = temporal::bounds(
            sas_task.durative_actions[snapped->durative(a)], initial.data()).lo;
        } else if (snapped && a >= snapped->tils) {
          c = 0.0;
        }
        costs[a] = std::isnan(c) || c < 0.0 ? 0.001 : c;
      }
      action_costs = true;
    }
    if (!action_costs && !length_metric) {
      res.diagnostics.push_back(Diagnostic::note(
          "orchestrator.metric", "the search minimizes the number of actions; the metric of "
          "the problem is reported for each plan but does not guide the search yet"));
    }
    log.info(
      LogChannel::Orchestrator, "search: {} variables, {} actions, costs {}",
      search_task.variables.size(), search_task.actions.size(),
      time_metric ? "durations" : (action_costs ? "from the metric" : "unit"));
    state::StateRegistry registry(search_task, &budget);
    state::StateSpace space(search_task, registry);
    std::unique_ptr<temporal::TemporalFilter> filter;
    if (snapped) {
      filter = std::make_unique<temporal::TemporalFilter>(
        sas_task, *snapped, space, options.epsilon);
    }

    // Plans: validated against the original task before anyone sees them
    std::mutex mutex;
    std::vector<uint32_t> best_actions;
    std::atomic<bool> target_reached{false};
    std::atomic<int64_t> last_plan{-1};  // ms since t0
    size_t invalid = 0;
    auto search_start = Clock::now();
    search::PlanSink sink([&](const search::PlanSink::Entry & e) {
        Plan plan;
        if (compressed) {
          auto temporal_plan = temporal::schedule(
            sas_task, *compressed, space, e.actions, options.epsilon);
          if (!temporal_plan) {
            ++invalid;
            log.error(LogChannel::Orchestrator, "internal error: a plan could not be scheduled");
            return;
          }
          plan = std::move(*temporal_plan);
        } else if (filter) {
          auto temporal_plan = filter->plan(e.actions);
          if (!temporal_plan) {
            ++invalid;
            log.error(LogChannel::Orchestrator, "internal error: a plan could not be scheduled");
            return;
          }
          plan = std::move(*temporal_plan);
        } else {
          plan = search::to_plan(sas_task, e.actions);
        }
        ValidateOptions vo;
        vo.epsilon = options.epsilon;
        auto report = val::validate(in.domain, in.domain_model, in.problem, in.problem_model,
          plan, vo);
        if (!report.valid()) {
          ++invalid;
          log.error(
            LogChannel::Orchestrator, "internal error: a plan failed validation: {}",
            report.diagnostics.empty() ? std::string("?") : to_string(report.diagnostics[0]));
          return;
        }
        // Without a metric: the makespan of temporal plans, the length of sequential ones
        plan.cost = report.metric ? *report.metric :
        (plan.temporal ? plan.makespan() : static_cast<double>(plan.steps.size()));
        uint32_t index;
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
          Clock::now() - t0);
        {
          std::lock_guard<std::mutex> lock(mutex);
          index = static_cast<uint32_t>(res.stats.plans++);
          res.plan = plan;
          best_actions = e.actions;
          if (!res.stats.first_plan_ms) {
            res.stats.first_plan_ms = ms_since(t0);
          }
        }
        last_plan = elapsed.count();
        log.info(
          LogChannel::Search, "plan {} with {} actions, cost {}", index, plan.steps.size(),
          *plan.cost);
        if (on_plan) {
          on_plan(plan, index, elapsed);
        }
        if (options.target_cost && *plan.cost <= *options.target_cost) {
          target_reached = true;
          internal.request_stop();
        }
      });
    // Stop when no plan improves for a while
    std::atomic<bool> no_improvement{false};
    std::jthread watchdog;
    if (options.stop_after_no_improvement) {
      auto window = options.stop_after_no_improvement->count();
      watchdog = std::jthread([&, window](std::stop_token st) {
            while (!st.stop_requested()) {
              std::this_thread::sleep_for(std::chrono::milliseconds(5));
              int64_t last = last_plan.load();
              auto now = std::chrono::duration_cast<std::chrono::milliseconds>(
                Clock::now() - t0).count();
              if (last >= 0 && now - last >= window) {
                no_improvement = true;
                internal.request_stop();
                return;
              }
            }
          });
    }
    search::SearchContext ctx(search_task, registry, space, costs, sink, &budget);
    if (filter) {
      ctx.accept = [&filter](state::StateId p, uint32_t a, state::StateId c) {
          return filter->accept(p, a, c);
        };
    }
    ctx.logger = log;
    ctx.prune_with_bound = action_costs || length_metric;
    if (options.search == "auto") {
      std::string chosen = resolve_auto(search_task);
      configured = preset(chosen);
      log.info(LogChannel::Orchestrator, "search: {} (auto)", chosen);
    }
    search::AnytimeOptions ao = *configured;
    ao.improve = options.anytime;
    auto outcome = search::anytime(ctx, ao);
    if (watchdog.joinable()) {
      watchdog.request_stop();
      watchdog.join();
    }
    res.stats.search_ms = ms_since(search_start);
    res.stats.expanded = ctx.stats.expanded;
    res.stats.generated = ctx.stats.generated;
    res.stats.evaluated = ctx.stats.evaluated;
    res.stats.states = registry.size();
    res.stats.bytes_per_state = registry.bytes_per_state() + 32.0;  // and the search node
    if (res.plan) {
      dump(options, "plan-trace", "plan-trace.txt", [&]() {
          return plan_trace(search_task, best_actions);
        }, log);
      StopReason reason = StopReason::NoImprovement;
      if (outcome == search::AnytimeResult::ProvenOptimal) {
        // In M2 and M3 the search proves the sum of durations optimal, not the makespan
        reason = temporal_task ? StopReason::Exhausted : StopReason::ProvenOptimal;
      } else if (target_reached) {
        reason = StopReason::TargetReached;
      } else if (no_improvement) {
        reason = StopReason::NoImprovement;
      } else if (outcome == search::AnytimeResult::Stopped) {
        reason = budget_status().second;
      }
      return finish(Status::Solved, reason);
    }
    if (invalid > 0) {
      res.diagnostics.push_back(Diagnostic::error(
          "orchestrator.invalid_plans", "every plan found failed validation (a bug in Kairos)"));
      return finish(Status::InternalError, StopReason::Exhausted);
    }
    switch (outcome) {
      case search::AnytimeResult::Exhausted:
        if (!m2_complete) {
          res.diagnostics.push_back(Diagnostic::error(
              "unsupported.m2_incomplete",
              "engine M2 found no plan, which does not prove there is none for this task "
              "(variable durations or possible required concurrency: engine M3, step 15d)"));
          return finish(Status::Unsupported, StopReason::Exhausted);
        }
        if (snapped) {
          res.diagnostics.push_back(Diagnostic::note(
              "orchestrator.m3_exhausted", "engine M3 found no plan; as POPF, it keeps one "
              "partial plan per state, so this is strong evidence but not a proof"));
        }
        return finish(Status::Unsolvable, StopReason::Exhausted);
      case search::AnytimeResult::Stopped: {
          auto [s, r] = budget_status();
          return finish(s, r);
        }
      default:
        res.diagnostics.push_back(Diagnostic::error(
            "orchestrator.no_plan", "the search ended without a plan and without a proof"));
        return finish(Status::InternalError, StopReason::Exhausted);
    }
  } catch (const std::exception & e) {
    res.diagnostics.push_back(Diagnostic::error("orchestrator.exception", e.what()));
    return finish(Status::InternalError, StopReason::Exhausted);
  }
}

}  // namespace kairos::orch
