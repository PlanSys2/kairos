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
#include <condition_variable>
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
    bool length_metric = in.problem.metric == std::nullopt;
    // Temporal tasks minimizing the makespan (the default): each durative action costs its
    // duration (at the initial state), so the search prefers short and spread work, which the
    // schedule then overlaps
    bool time_metric = temporal_task && sas_task.metric.minimize &&
      (length_metric || sas_task.metric.expr.tree.op == ground::ExprOp::TotalTime);
    std::vector<double> initial_numbers;
    for (const auto & n : sas_task.initial_numeric) {
      initial_numbers.push_back(n ? *n : state::kUndefined);
    }
    // The cost of each action of a search task, and whether bounds may prune
    auto costs_of = [&](
      const sas::Task & of, const temporal::Compressed * comp, const temporal::SnapTask * snap,
      bool & prune) {
        bool action_costs = heuristics::metric_costs_apply(of);
        auto costs = heuristics::action_costs(
          of, action_costs ? heuristics::CostModel::Metric : heuristics::CostModel::Unit);
        if (time_metric) {
          for (uint32_t a = 0; a < costs.size(); ++a) {
            double c = 0.001;  // instantaneous actions (and ends) take no time
            if (comp && a >= comp->instant) {
              c = state::evaluate(comp->duration[a - comp->instant], initial_numbers.data());
            } else if (snap && snap->is_start(a)) {
              c = temporal::bounds(
                sas_task.durative_actions[snap->durative(a)], initial_numbers.data()).lo;
            } else if (snap && a >= snap->tils) {
              c = 0.0;
            }
            costs[a] = std::isnan(c) || c < 0.0 ? 0.001 : c;
          }
          action_costs = true;
        }
        prune = action_costs || length_metric;
        return costs;
      };

    // Plans: validated against the original task before anyone sees them; only better ones
    // (by the metric, or the makespan or length) are kept and handed over
    std::mutex mutex;
    std::vector<uint32_t> best_actions;
    const sas::Task * best_task = &search_task;
    std::atomic<bool> target_reached{false};
    std::atomic<int64_t> last_plan{-1};  // ms since t0
    size_t invalid = 0;
    auto search_start = Clock::now();
    auto offer = [&](std::optional<Plan> plan, const std::vector<uint32_t> & actions,
      const sas::Task & from) {
        if (!plan) {
          ++invalid;
          log.error(LogChannel::Orchestrator, "internal error: a plan could not be scheduled");
          return;
        }
        ValidateOptions vo;
        vo.epsilon = options.epsilon;
        auto report = val::validate(in.domain, in.domain_model, in.problem, in.problem_model,
          *plan, vo);
        if (!report.valid()) {
          ++invalid;
          log.error(
            LogChannel::Orchestrator, "internal error: a plan failed validation: {}",
            report.diagnostics.empty() ? std::string("?") : to_string(report.diagnostics[0]));
          return;
        }
        // Without a metric: the makespan of temporal plans, the length of sequential ones
        plan->cost = report.metric ? *report.metric :
          (plan->temporal ? plan->makespan() : static_cast<double>(plan->steps.size()));
        bool minimize = !in.problem.metric || sas_task.metric.minimize;
        uint32_t index;
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - t0);
        {
          std::lock_guard<std::mutex> lock(mutex);
          if (res.plan && res.plan->cost &&
            (minimize ? *plan->cost >= *res.plan->cost : *plan->cost <= *res.plan->cost))
          {
            return;  // cheaper for the search, not better by the metric
          }
          index = static_cast<uint32_t>(res.stats.plans++);
          res.plan = *plan;
          best_actions = actions;
          best_task = &from;
          if (!res.stats.first_plan_ms) {
            res.stats.first_plan_ms = ms_since(t0);
          }
        }
        last_plan = elapsed.count();
        log.info(
          LogChannel::Search, "plan {} with {} actions, cost {}", index, plan->steps.size(),
          *plan->cost);
        if (on_plan) {
          on_plan(*plan, index, elapsed);
        }
        if (options.target_cost && *plan->cost <= *options.target_cost) {
          target_reached = true;
          internal.request_stop();
        }
      };
    // Stop when no plan improves for a while
    std::atomic<bool> no_improvement{false};
    std::jthread watchdog;
    if (options.stop_after_no_improvement) {
      auto window = options.stop_after_no_improvement->count();
      watchdog = std::jthread([&, window](std::stop_token st) {
            // Wakes at once when the search ends, so small tasks do not wait for it
            std::mutex m;
            std::condition_variable_any wake;
            std::unique_lock<std::mutex> lock(m);
            while (!st.stop_requested()) {
              wake.wait_for(lock, st, std::chrono::milliseconds(5), [] {return false;});
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
    // The phases of the search, each on a task (plain, compressed for M2 or snap for M3) with a
    // search configuration and a share of the remaining time:
    // - classical and numeric tasks: one phase;
    // - M3 with "auto": a portfolio, since no configuration wins everywhere (lazy LAMA finds
    //   PlanSys2's bt_example at once; eager search with novelty, match-cellar);
    // - simple temporal tasks with "auto" and anytime: M2 for the first plan (its improvements
    //   lower the sum of durations, which says little of the makespan), then the M3 portfolio
    //   with g the makespan of the partial plan
    enum class Kind : uint8_t {Plain, Compressed, Snap};
    struct Phase
    {
      Kind kind;
      std::string search;
      double share;      // of the time left when it starts (1: all)
      bool improve;
      bool makespan_g;   // g is the makespan of the partial plan (M3)
    };
    std::vector<Phase> phases;
    bool portfolio = options.search == "auto";
    auto m3_portfolio = [&](bool makespan_g) {
        phases.push_back({Kind::Snap, "lama", 0.2, true, makespan_g});
        phases.push_back({Kind::Snap, "eager-lama", 0.375, true, makespan_g});
        phases.push_back({Kind::Snap, "lama-novelty", 1.0, true, makespan_g});
      };
    std::optional<temporal::SnapTask> improver;
    if (compressed && portfolio && options.anytime && options.engine == Engine::Auto &&
      time_metric)
    {
      improver = temporal::snap(sas_task);
      if (!improver->unsupported.empty()) {
        improver.reset();
      }
    }
    if (snapped) {
      if (portfolio) {
        m3_portfolio(time_metric && options.anytime);
      } else {
        phases.push_back({Kind::Snap, options.search, 1.0, options.anytime, false});
      }
    } else if (compressed) {
      phases.push_back({Kind::Compressed, options.search, improver ? 0.0 : 1.0,
          options.anytime && !improver, false});
      if (improver) {
        m3_portfolio(true);
      }
    } else {
      phases.push_back({Kind::Plain, options.search, 1.0, options.anytime, false});
    }
    const temporal::SnapTask * snap_task = snapped ? &*snapped : (improver ? &*improver : nullptr);

    if (!time_metric && !heuristics::metric_costs_apply(search_task) && !length_metric) {
      res.diagnostics.push_back(Diagnostic::note(
          "orchestrator.metric", "the search minimizes the number of actions; the metric of "
          "the problem is reported for each plan but does not guide the search yet"));
    }
    search::AnytimeResult outcome = search::AnytimeResult::Failed;
    search::SearchStats stats;
    size_t states = 0;
    double bytes_per_state = 0.0;
    bool proof = false;           // a complete phase exhausted its space without a plan
    bool m3_exhausted = false;
    for (size_t k = 0; k < phases.size(); ++k) {
      const Phase & ph = phases[k];
      if (budget.check() != BudgetState::Ok || target_reached || no_improvement) {
        break;
      }
      const sas::Task & task_k = ph.kind == Kind::Compressed ? compressed->task :
        (ph.kind == Kind::Snap ? snap_task->task : sas_task);
      // Its own deadline, unless it is the last phase (then the global budget)
      std::optional<Budget> own;
      if (ph.share < 1.0 && k + 1 < phases.size()) {
        BudgetConfig pc = bc;
        auto left = (bc.deadline ? *bc.deadline : t0 + options.timeout) - Clock::now();
        pc.deadline = Clock::now() + std::chrono::duration_cast<Clock::duration>(
          left * ph.share);
        if (ph.share == 0.0) {
          pc.deadline = bc.deadline;  // until its first plan (it does not improve)
        }
        own.emplace(pc);
      }
      Budget & bk = own ? *own : budget;
      bool prune = false;
      auto costs = costs_of(task_k, ph.kind == Kind::Compressed ? &*compressed : nullptr,
          ph.kind == Kind::Snap ? snap_task : nullptr, prune);
      std::string chosen = ph.search == "auto" ?
        resolve_auto(task_k, ph.kind == Kind::Snap) : ph.search;
      if (ph.search == "auto") {
        log.info(LogChannel::Orchestrator, "search: {} (auto)", chosen);
      }
      log.info(
        LogChannel::Orchestrator, "{}: {} variables, {} actions, search {}, costs {}",
        ph.kind == Kind::Compressed ? "engine M2" :
        (ph.kind == Kind::Snap ? "engine M3" : "search"), task_k.variables.size(),
        task_k.actions.size(), chosen, ph.makespan_g ? "makespan" :
        (time_metric ? "durations" : (prune && !length_metric ? "from the metric" : "unit")));
      state::StateRegistry registry(task_k, &bk);
      state::StateSpace space(task_k, registry);
      std::unique_ptr<temporal::TemporalFilter> filter;
      if (ph.kind == Kind::Snap) {
        filter = std::make_unique<temporal::TemporalFilter>(
          sas_task, *snap_task, space, options.epsilon);
      }
      search::PlanSink sink([&](const search::PlanSink::Entry & e) {
          if (ph.kind == Kind::Compressed) {
            offer(temporal::schedule(sas_task, *compressed, space, e.actions, options.epsilon),
              e.actions, task_k);
          } else if (filter) {
            offer(filter->plan(e.actions), e.actions, task_k);
          } else {
            offer(search::to_plan(sas_task, e.actions), e.actions, task_k);
          }
        });
      search::SearchContext ctx(task_k, registry, space, costs, sink, &bk);
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
      ctx.logger = log;
      ctx.prune_with_bound = prune;
      search::AnytimeOptions ao = *preset(chosen);
      ao.improve = ph.improve;
      auto result = search::anytime(ctx, ao);
      stats.expanded += ctx.stats.expanded;
      stats.generated += ctx.stats.generated;
      stats.evaluated += ctx.stats.evaluated;
      states += registry.size();
      bytes_per_state = registry.bytes_per_state() + 32.0;  // and the search node
      if (res.plan && best_task == &task_k) {
        dump(options, "plan-trace", "plan-trace.txt", [&]() {
            return plan_trace(task_k, best_actions);
          }, log);
      }
      outcome = result;
      if (result == search::AnytimeResult::Exhausted && !res.plan) {
        // No plan in its whole space: a proof for the plain task and a complete M2; strong
        // evidence for M3 (one partial plan per state)
        proof = ph.kind == Kind::Plain || (ph.kind == Kind::Compressed && m2_complete);
        m3_exhausted = ph.kind == Kind::Snap;
        if (proof || m3_exhausted) {
          break;
        }
      }
      if (result == search::AnytimeResult::ProvenOptimal && ph.kind == Kind::Plain) {
        break;  // nothing better exists
      }
    }
    if (watchdog.joinable()) {
      watchdog.request_stop();
      watchdog.join();
    }
    res.stats.search_ms = ms_since(search_start);
    res.stats.expanded = stats.expanded;
    res.stats.generated = stats.generated;
    res.stats.evaluated = stats.evaluated;
    res.stats.states = states;
    res.stats.bytes_per_state = bytes_per_state;
    if (res.plan) {
      StopReason reason = StopReason::NoImprovement;
      if (target_reached) {
        reason = StopReason::TargetReached;
      } else if (no_improvement) {
        reason = StopReason::NoImprovement;
      } else if (outcome == search::AnytimeResult::ProvenOptimal) {
        // In M2 and M3 the search proves the sum of durations optimal, not the makespan
        reason = temporal_task ? StopReason::Exhausted : StopReason::ProvenOptimal;
      } else if (outcome == search::AnytimeResult::Stopped || budget.check() != BudgetState::Ok) {
        reason = budget_status().second;
      } else if (outcome == search::AnytimeResult::Exhausted && temporal_task) {
        reason = StopReason::Exhausted;
      }
      return finish(Status::Solved, reason);
    }
    if (invalid > 0) {
      res.diagnostics.push_back(Diagnostic::error(
          "orchestrator.invalid_plans", "every plan found failed validation (a bug in Kairos)"));
      return finish(Status::InternalError, StopReason::Exhausted);
    }
    if (proof) {
      return finish(Status::Unsolvable, StopReason::Exhausted);
    }
    if (m3_exhausted) {
      res.diagnostics.push_back(Diagnostic::note(
          "orchestrator.m3_exhausted", "engine M3 found no plan; as POPF, it keeps one "
          "partial plan per state, so this is strong evidence but not a proof"));
      return finish(Status::Unsolvable, StopReason::Exhausted);
    }
    if (outcome == search::AnytimeResult::Exhausted) {
      res.diagnostics.push_back(Diagnostic::error(
          "unsupported.m2_incomplete",
          "engine M2 found no plan, which does not prove there is none for this task "
          "(variable durations or possible required concurrency)"));
      return finish(Status::Unsupported, StopReason::Exhausted);
    }
    if (outcome == search::AnytimeResult::Stopped || budget.check() != BudgetState::Ok) {
      auto [st, r] = budget_status();
      return finish(st, r);
    }
    res.diagnostics.push_back(Diagnostic::error(
        "orchestrator.no_plan", "the search ended without a plan and without a proof"));
    return finish(Status::InternalError, StopReason::Exhausted);
  } catch (const std::exception & e) {
    res.diagnostics.push_back(Diagnostic::error("orchestrator.exception", e.what()));
    return finish(Status::InternalError, StopReason::Exhausted);
  }
}

}  // namespace kairos::orch
