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

#include "validate/validator.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <functional>
#include <initializer_list>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "pddl/printer.hpp"
#include "validate/world.hpp"

namespace kairos::val
{

namespace
{

// --- Validation ------------------------------------------------------------------------------

struct StepInfo
{
  const PlanStep * step = nullptr;
  size_t index = 0;            // position in the plan (1-based in messages)
  bool durative = false;
  size_t action = 0;           // index into actions or durative_actions
  Bindings bindings;
  double start = 0.0;
  double end = 0.0;
  std::unique_ptr<State> start_state;  // for when-conditions mixing at start and at end
  bool started = false;
  bool ended = false;
};

struct EventInfo
{
  enum class Kind {Instant, Start, End, Til} kind;
  double time;
  size_t index;  // step or init item
  Tracking reads;
  Changes changes;
  int order;     // tie-break for sorting
};

class Validator
{
public:
  Validator(World & world, const Plan & plan, ValidationReport & report)
  : w_(world), plan_(plan), report_(report) {}

  void run();

private:
  [[noreturn]] void fail(const std::string & code, const std::string & message, uint32_t line)
  {
    report_.failed_step = current_;
    report_.diagnostics.push_back(
      Diagnostic::error(code, message,
      SourceLocation{w_.options.plan_filename, line, 0}));
    throw Stop{};
  }

  struct Stop
  {
  };

  std::string step_name(const StepInfo & s) const
  {
    char t[64];
    std::snprintf(t, sizeof(t), "%.3f", s.step->time);
    return "step " + std::to_string(s.index) + " " + s.step->action() + " at " + t;
  }

  void resolve_steps();
  void initial_state();
  void check_global_constraints(const State & s, const std::string & when);
  bool interfere(const EventInfo & a, const EventInfo & b) const;
  bool conflict(const EventInfo & a, const EventInfo & b) const;  // writes that disagree
  bool integrate(double from, double to);  // true if some fluent changed
  bool nonlinear_ = false;  // continuous effects whose rates change: evolve, not integrate
  // PDDL+: processes and events
  struct Grounded
  {
    size_t index;
    Bindings bindings;
  };
  void ground_processes_and_events();
  void derivative(const State & s, std::unordered_map<Key, double, KeyHash> & out);
  bool active(const ast::Action & a, const State & s, Bindings b);
  using Numbers = std::unordered_map<Key, double, KeyHash>;
  void step_numbers(double h, const Numbers & k1, Numbers & to, const Numbers * over = nullptr);
  // state_ with some numbers changed: copied only after a discrete change, then patched
  struct Scratch
  {
    State state;
    uint64_t version = UINT64_MAX;
    std::vector<Key> written;
  };
  const State & patched(Scratch & s, const Numbers & values);
  void commit(const Numbers & values);
  Scratch stage_, probe_;
  std::vector<uint8_t> watch(const State & s);
  void evolve(double from, double to);
  void fire_events(double t);
  void apply_changes(const std::vector<Changes> & all, uint32_t line);
  void check_invariants(const State & s, const std::string & when);
  void record_trajectory();
  void finish();
  bool constraint(FormulaId id, Evaluator & ev, Bindings & b);

  World & w_;
  const Plan & plan_;
  ValidationReport & report_;
  std::optional<size_t> current_;  // the step whose happening is being checked
  std::vector<StepInfo> steps_;
  std::vector<size_t> running_;  // durative steps started and not ended
  State state_;
  std::vector<std::pair<double, State>> trajectory_;
  bool record_ = false;
  double makespan_ = 0.0;
  bool pddl_plus_ = false;
  std::vector<Grounded> processes_, events_;
  // The atoms of each process and event condition (they only change at happenings and
  // events): the groundings whose atoms hold are the only ones to look at while time passes
  std::vector<std::vector<ast::FormulaId>> process_atoms_, event_atoms_;
  std::vector<uint8_t> live_processes_, live_events_;
  uint64_t atoms_version_ = 0;
  uint64_t live_version_ = UINT64_MAX;
  void refresh_live();
};

void Validator::resolve_steps()
{
  const auto & d = w_.domain;
  std::map<std::string, std::pair<bool, size_t>> actions;
  for (size_t i = 0; i < d.actions.size(); ++i) {
    actions[std::string(d.symbols.name(d.actions[i].name))] = {false, i};
  }
  for (size_t i = 0; i < d.durative_actions.size(); ++i) {
    actions[std::string(d.symbols.name(d.durative_actions[i].name))] = {true, i};
  }
  steps_.resize(plan_.steps.size());
  for (size_t i = 0; i < plan_.steps.size(); ++i) {
    const PlanStep & ps = plan_.steps[i];
    StepInfo & s = steps_[i];
    current_ = i;
    s.step = &ps;
    s.index = i + 1;
    auto a = actions.find(ps.name);
    if (a == actions.end()) {
      fail("validate.unknown_action", step_name(s) + ": the domain has no action '" + ps.name +
        "'", ps.line);
    }
    s.durative = a->second.first;
    s.action = a->second.second;
    const auto & params = s.durative ? d.durative_actions[s.action].params :
      d.actions[s.action].params;
    if (params.size() != ps.args.size()) {
      fail("validate.wrong_arity", step_name(s) + ": '" + ps.name + "' takes " +
        std::to_string(params.size()) + " argument(s), " + std::to_string(ps.args.size()) +
        " given", ps.line);
    }
    for (size_t k = 0; k < params.size(); ++k) {
      auto o = w_.pm.objects.find(ps.args[k]);
      if (!o) {
        fail("validate.unknown_object", step_name(s) + ": unknown object '" + ps.args[k] + "'",
          ps.line);
      }
      sem::TypeSpec required = w_.resolve(params[k].types, d.symbols);
      if (!w_.dm.types.fits(w_.pm.objects.items[*o].types, required)) {
        fail("validate.type_mismatch", step_name(s) + ": argument " + std::to_string(k + 1) +
          " must be " + w_.dm.types.to_string(required) + ", '" + ps.args[k] + "' is " +
          w_.dm.types.to_string(w_.pm.objects.items[*o].types), ps.line);
      }
      s.bindings.emplace_back(params[k].name, o->value());
    }
    s.start = ps.time;
    s.end = ps.time + (s.durative ? ps.duration : 0.0);
    if (!s.durative && ps.duration > w_.options.tolerance) {
      report_.diagnostics.push_back(
        Diagnostic::warning(
          "validate.duration_on_instant", step_name(s) + ": '" + ps.name +
          "' is not durative; its duration is ignored",
          SourceLocation{w_.options.plan_filename, ps.line, 0}));
    }
  }
}

void Validator::initial_state()
{
  w_.initial_state(state_);
}

bool Validator::interfere(const EventInfo & a, const EventInfo & b) const
{
  auto writes_read = [](const EventInfo & x, const EventInfo & y) {
      for (const auto & k : x.changes.adds) {
        if (y.reads.atoms.count(k)) {
          return true;
        }
      }
      for (const auto & k : x.changes.dels) {
        if (y.reads.atoms.count(k)) {
          return true;
        }
      }
      for (const auto & n : x.changes.nums) {
        if (y.reads.fluents.count(n.key)) {
          return true;
        }
      }
      for (const auto & o : x.changes.objs) {
        if (y.reads.fluents.count(o.key)) {
          return true;
        }
      }
      return false;
    };
  return writes_read(a, b) || writes_read(b, a) || conflict(a, b);
}

bool Validator::conflict(const EventInfo & a, const EventInfo & b) const
{
  auto contains = [](const std::vector<Key> & v, const Key & k) {
      return std::find(v.begin(), v.end(), k) != v.end();
    };
  for (const auto & k : a.changes.adds) {
    if (contains(b.changes.dels, k)) {
      return true;
    }
  }
  for (const auto & k : b.changes.adds) {
    if (contains(a.changes.dels, k)) {
      return true;
    }
  }
  auto additive = [](ast::AssignOp op) {
      return op == ast::AssignOp::Increase || op == ast::AssignOp::Decrease;
    };
  for (const auto & x : a.changes.nums) {
    for (const auto & y : b.changes.nums) {
      if (x.key == y.key && !(additive(x.op) && additive(y.op))) {
        return true;
      }
    }
  }
  for (const auto & x : a.changes.objs) {
    for (const auto & y : b.changes.objs) {
      if (x.key == y.key) {
        return true;
      }
    }
  }
  return false;
}

bool Validator::integrate(double from, double to)
{
  if (to <= from || running_.empty()) {
    return false;
  }
  // Rates of the running actions, evaluated at the start of the interval
  std::unordered_map<Key, double, KeyHash> rates;
  std::vector<std::pair<Key, double>> contributions;
  for (size_t r : running_) {
    StepInfo & s = steps_[r];
    const auto & a = w_.domain.durative_actions[s.action];
    if (!a.effect.valid()) {
      continue;
    }
    Bindings b = s.bindings;
    Tracking reads;
    Evaluator::Ctx c{state_, nullptr, Phase::Invariant, s.step->duration};
    c.reads = &reads;
    w_.domain_eval.rates(a.effect, c, b, contributions);
    for (const auto & [k, rate] : contributions) {
      rates[k] += rate;
    }
    // A rate that depends on a continuously changing fluent is not linear
    for (const auto & k : reads.fluents) {
      if (rates.count(k)) {
        throw Unsupported{"continuous effects whose rate depends on another continuous change"
                " (non-linear dynamics)"};
      }
    }
    contributions.clear();
  }
  if (rates.empty()) {
    return false;
  }
  double dt = to - from;
  for (const auto & [k, rate] : rates) {
    auto it = state_.numbers.find(k);
    if (it == state_.numbers.end()) {
      fail("validate.undefined_fluent", w_.key_text(k, false) +
        " changes continuously but has no value", 0);
    }
    it->second += rate * dt;
  }
  state_.changed(w_.axioms_read_fluents);
  return true;
}

void Validator::check_invariants(const State & s, const std::string & when)
{
  for (size_t r : running_) {
    StepInfo & st = steps_[r];
    const auto & a = w_.domain.durative_actions[st.action];
    if (!a.condition.valid()) {
      continue;
    }
    Bindings b = st.bindings;
    Evaluator::Ctx c{s, nullptr, Phase::Invariant, st.step->duration};
    try {
      if (!w_.domain_eval.holds(a.condition, c, b)) {
        fail("validate.invariant_failed", step_name(st) + ": over-all condition broken " +
          when + ": " + w_.domain_eval.why(a.condition, c, b), st.step->line);
      }
    } catch (const EvalError & e) {
      fail("validate.evaluation", step_name(st) + ": " + e.message, st.step->line);
    }
  }
}

void Validator::check_global_constraints(const State & s, const std::string & when)
{
  for (const auto & gc : w_.domain.global_constraints) {
    Bindings b;
    w_.domain_eval.for_each_binding(gc.params, b, [&]() {
        Evaluator::Ctx c{s};
        if (!w_.domain_eval.holds(gc.condition, c, b)) {
          fail("validate.global_constraint", "constraint '" +
            std::string(w_.domain.symbols.name(gc.name)) + "' broken " + when + ": " +
            w_.domain_eval.why(gc.condition, c, b), 0);
        }
        return true;
      });
  }
}

void Validator::record_trajectory()
{
  if (record_) {
    State copy;
    copy.atoms = state_.atoms;
    copy.numbers = state_.numbers;
    copy.objects = state_.objects;
    trajectory_.emplace_back(state_.time, std::move(copy));
  }
}

void Validator::run()
{
  pddl_plus_ = !w_.domain.processes.empty() || !w_.domain.events.empty();
  record_ = w_.domain.constraints.valid() || w_.problem.constraints.valid();
  resolve_steps();
  current_.reset();
  initial_state();
  if (pddl_plus_) {
    ground_processes_and_events();
    fire_events(0.0);
  }
  record_trajectory();
  check_global_constraints(state_, "in the initial state");

  // Events
  std::vector<EventInfo> events;
  int order = 0;
  for (size_t i = 0; i < steps_.size(); ++i) {
    if (steps_[i].durative) {
      events.push_back({EventInfo::Kind::Start, steps_[i].start, i, {}, {}, order++});
      events.push_back({EventInfo::Kind::End, steps_[i].end, i, {}, {}, order++});
    } else {
      events.push_back({EventInfo::Kind::Instant, steps_[i].start, i, {}, {}, order++});
    }
  }
  for (size_t i = 0; i < w_.problem.init.size(); ++i) {
    if (w_.problem.init[i].time) {
      events.push_back({EventInfo::Kind::Til, *w_.problem.init[i].time, i, {}, {}, order++});
    }
  }
  std::stable_sort(events.begin(), events.end(),
    [](const EventInfo & a, const EventInfo & b) {return a.time < b.time;});

  const double tol = w_.options.tolerance;
  std::vector<EventInfo> previous;
  double previous_time = 0.0;
  bool have_previous = false;
  double clock = 0.0;
  size_t i = 0;
  while (i < events.size()) {
    // A happening: events at the same time
    size_t j = i;
    while (j < events.size() && events[j].time - events[i].time <= tol) {
      ++j;
    }
    if (pddl_plus_) {
      // PDDL+ plans (ENHSP's) apply the steps of an instant one after another, in plan order
      j = i + 1;
    }
    double t = events[i].time;
    bool changed = false;
    if (pddl_plus_ || nonlinear_) {
      evolve(clock, t);
    } else {
      try {
        changed = integrate(clock, t);
      } catch (const Unsupported &) {
        // A rate that changes with time: numerically, as PDDL+ (integrate changed nothing)
        nonlinear_ = true;
        evolve(clock, t);
      }
    }
    clock = t;
    state_.time = t;
    if (changed) {
      // Continuous change: the over-all conditions must still hold at the end of the interval
      check_invariants(state_, "before the happening at " + std::to_string(t));
    }

    // Conditions and effects of every event, on the state before the happening
    std::vector<EventInfo> group(events.begin() + static_cast<std::ptrdiff_t>(i),
      events.begin() + static_cast<std::ptrdiff_t>(j));
    for (auto & ev : group) {
      if (ev.kind == EventInfo::Kind::Til) {
        const auto & item = w_.problem.init[ev.index];
        Bindings none;
        Evaluator::Ctx c{state_};
        auto & pe = w_.problem_eval;
        if (item.kind == ast::InitItem::Kind::Fact) {
          auto pred = w_.dm.predicates.find(w_.problem.symbols.name(item.name));
          if (!pred) {
            continue;
          }
          Key key{pred->value()};
          for (TermId a : item.args) {
            key.push_back(pe.object_of(a, c, none));
          }
          (item.negated ? ev.changes.dels : ev.changes.adds).push_back(std::move(key));
        } else {
          auto fn = w_.dm.functions.find(w_.problem.symbols.name(item.name));
          if (!fn) {
            continue;
          }
          Key key{fn->value()};
          for (TermId a : item.args) {
            key.push_back(pe.object_of(a, c, none));
          }
          const ast::Expr & v = w_.problem.nodes.exprs[item.value];
          if (w_.dm.functions.items[*fn].numeric) {
            ev.changes.nums.push_back({std::move(key), ast::AssignOp::Assign, v.number});
          } else {
            ev.changes.objs.push_back({std::move(key), pe.object_of(v.term, c, none)});
          }
        }
        continue;
      }
      StepInfo & s = steps_[ev.index];
      current_ = ev.index;
      Bindings b = s.bindings;
      try {
        if (!s.durative) {
          const auto & a = w_.domain.actions[s.action];
          Evaluator::Ctx c{state_};
          c.reads = &ev.reads;
          c.prefs = &report_.violated_preferences;
          if (a.precondition.valid() && !w_.domain_eval.holds(a.precondition, c, b)) {
            Evaluator::Ctx plain{state_};
            fail("validate.precondition_failed", step_name(s) + ": precondition false: " +
              w_.domain_eval.why(a.precondition, plain, b), s.step->line);
          }
          if (a.effect.valid()) {
            w_.domain_eval.effects(a.effect, c, b, true, ev.changes);
          }
          continue;
        }
        const auto & a = w_.domain.durative_actions[s.action];
        Phase phase = ev.kind == EventInfo::Kind::Start ? Phase::Start : Phase::End;
        Evaluator::Ctx c{state_, s.start_state.get(), phase, s.step->duration};
        c.reads = &ev.reads;
        c.prefs = &report_.violated_preferences;
        if (phase == Phase::Start) {
          Evaluator::Ctx dc{state_, nullptr, Phase::Instant, s.step->duration};
          dc.duration_check = true;
          if (!w_.domain_eval.holds(a.duration, dc, b)) {
            fail("validate.duration_mismatch", step_name(s) + ": duration " +
              std::to_string(s.step->duration) + " does not satisfy " +
              pddl::to_pddl(w_.domain.nodes, w_.domain.symbols, a.duration) + ": " +
              w_.domain_eval.why(a.duration, dc, b), s.step->line);
          }
        }
        if (a.condition.valid() && !w_.domain_eval.holds(a.condition, c, b)) {
          Evaluator::Ctx plain{state_, s.start_state.get(), phase, s.step->duration};
          fail("validate.precondition_failed", step_name(s) + ": " +
            (phase == Phase::Start ? "at start" : "at end") + " condition false: " +
            w_.domain_eval.why(a.condition, plain, b), s.step->line);
        }
        if (a.effect.valid()) {
          w_.domain_eval.effects(a.effect, c, b, false, ev.changes);
        }
      } catch (const EvalError & e) {
        fail("validate.evaluation", step_name(s) + ": " + e.message, s.step->line);
      }
    }

    // Simultaneous events must not interfere; nor events closer than epsilon
    auto describe = [&](const EventInfo & ev) {
        if (ev.kind == EventInfo::Kind::Til) {
          return std::string("a timed initial literal");
        }
        std::string what = ev.kind == EventInfo::Kind::Start ? "the start of " :
          ev.kind == EventInfo::Kind::End ? "the end of " : "";
        return what + step_name(steps_[ev.index]);
      };
    auto line_of = [&](const EventInfo & ev) {
        // (the step a failure is reported at, when it is one)
        current_ = ev.kind == EventInfo::Kind::Til ? std::nullopt : std::optional<size_t>(ev.index);
        return ev.kind == EventInfo::Kind::Til ? 0u : steps_[ev.index].step->line;
      };
    // Epsilon 0: no separation, so one happening may read what another at the same instant
    // writes (UPF without an epsilon): conditions read the state before, effects apply
    // together. Writes that disagree are still an error
    bool separated = w_.options.epsilon > 0.0;
    for (size_t x = 0; x < group.size(); ++x) {
      for (size_t y = x + 1; y < group.size(); ++y) {
        if (group[x].kind == EventInfo::Kind::Start && group[y].kind == EventInfo::Kind::End &&
          group[x].index == group[y].index)
        {
          continue;
        }
        if (separated ? interfere(group[x], group[y]) : conflict(group[x], group[y])) {
          fail("validate.interference", describe(group[x]) + " and " + describe(group[y]) +
            " happen at the same time and interfere", line_of(group[y]));
        }
      }
    }
    if (!pddl_plus_ && have_previous && t - previous_time < w_.options.epsilon - tol) {
      for (const auto & pv : previous) {
        for (const auto & ev : group) {
          if (!interfere(pv, ev)) {
            continue;
          }
          std::string message = describe(pv) + " and " + describe(ev) + " interfere and are "
            "less than " + std::to_string(w_.options.epsilon) + " apart";
          if (w_.options.strict_epsilon) {
            fail("validate.too_close", message, line_of(ev));
          }
          report_.diagnostics.push_back(
            Diagnostic::warning("validate.too_close", message,
            SourceLocation{w_.options.plan_filename, line_of(ev), 0}));
        }
      }
    }

    // Starts whose when-conditions read "at start" need the state before the happening
    for (const auto & ev : group) {
      if (ev.kind != EventInfo::Kind::Start) {
        continue;
      }
      StepInfo & s = steps_[ev.index];
      const auto & a = w_.domain.durative_actions[s.action];
      if (a.effect.valid() && w_.domain_eval.has_when(a.effect)) {
        s.start_state = std::make_unique<State>();
        s.start_state->atoms = state_.atoms;
        s.start_state->numbers = state_.numbers;
        s.start_state->objects = state_.objects;
      }
    }

    // Apply all effects together: deletes, then adds (an action that deletes and adds the same
    // atom keeps it), then fluents
    State before;
    before.numbers = state_.numbers;
    bool axioms = false;  // whether the changes touch what the axioms read
    ++atoms_version_;
    for (const auto & ev : group) {
      for (const auto & k : ev.changes.dels) {
        axioms = axioms || w_.axioms_read(k[0]);
        state_.atoms.erase(k);
      }
    }
    for (const auto & ev : group) {
      for (const auto & k : ev.changes.adds) {
        axioms = axioms || w_.axioms_read(k[0]);
        state_.atoms.insert(k);
      }
      axioms = axioms || (w_.axioms_read_fluents && (!ev.changes.nums.empty() ||
        !ev.changes.objs.empty()));
    }
    for (const auto & ev : group) {
      for (const auto & n : ev.changes.nums) {
        auto old = before.numbers.find(n.key);
        auto & slot = state_.numbers[n.key];
        if (n.op != ast::AssignOp::Assign && old == before.numbers.end()) {
          fail("validate.undefined_fluent", w_.key_text(n.key, false) +
            " is changed but has no value", line_of(ev));
        }
        switch (n.op) {
          case ast::AssignOp::Assign: slot = n.value; break;
          case ast::AssignOp::Increase: slot += n.value; break;
          case ast::AssignOp::Decrease: slot -= n.value; break;
          case ast::AssignOp::ScaleUp: slot *= n.value; break;
          case ast::AssignOp::ScaleDown:
            if (n.value == 0.0) {
              fail("validate.evaluation", "scale-down by zero", line_of(ev));
            }
            slot /= n.value;
            break;
        }
      }
      for (const auto & o : ev.changes.objs) {
        if (o.value) {
          state_.objects[o.key] = *o.value;
        } else {
          state_.objects.erase(o.key);
        }
      }
    }
    state_.changed(axioms);

    for (const auto & ev : group) {
      if (ev.kind == EventInfo::Kind::Start) {
        StepInfo & s = steps_[ev.index];
        s.started = true;
        running_.push_back(ev.index);
      } else if (ev.kind == EventInfo::Kind::End) {
        steps_[ev.index].ended = true;
        running_.erase(std::find(running_.begin(), running_.end(), ev.index));
      }
      if (ev.kind != EventInfo::Kind::Til) {
        makespan_ = std::max(makespan_, t);
      }
    }
    if (pddl_plus_) {
      fire_events(t);
    }
    check_invariants(state_, "after the happening at " + std::to_string(t));
    check_global_constraints(state_, "at " + std::to_string(t));
    // In sequential plans the state after step k (at position k) is state k+1
    state_.time = plan_.temporal ? t : t + 1.0;
    record_trajectory();

    previous = std::move(group);
    previous_time = t;
    have_previous = true;
    i = j;
  }
  // PDDL+: time passes until the end of the plan, then the goal is checked
  if (pddl_plus_ && plan_.end && *plan_.end > clock) {
    current_.reset();
    evolve(clock, *plan_.end);
    state_.time = *plan_.end;
    record_trajectory();
  }
  if (plan_.end) {
    makespan_ = std::max(makespan_, *plan_.end);
  }
  // Sequential plans: one time unit per step
  if (!plan_.temporal) {
    makespan_ = static_cast<double>(plan_.steps.size());
  }
  finish();
}

// --- PDDL+ -------------------------------------------------------------------------------------

void Validator::ground_processes_and_events()
{
  // Static predicates (no effect or timed literal changes them): false in the initial state,
  // false forever; groundings that need one never run
  const auto & nodes = w_.domain.nodes;
  std::set<std::string> changing;
  for (uint32_t i = 0; i < nodes.effects.size(); ++i) {
    const auto & e = nodes.effects[ast::EffectId(i)];
    if (e.kind == ast::EffectKind::Add || e.kind == ast::EffectKind::Delete) {
      changing.insert(std::string(w_.domain.symbols.name(e.name)));
    }
  }
  for (const auto & item : w_.problem.init) {
    if (item.time) {
      changing.insert(std::string(w_.problem.symbols.name(item.name)));
    }
  }
  for (const auto & d : w_.domain.derived) {
    changing.insert(std::string(w_.domain.symbols.name(d.head.name)));
  }
  std::function<void(ast::FormulaId, std::vector<ast::FormulaId> &)> statics =
    [&](ast::FormulaId id, std::vector<ast::FormulaId> & out) {
      const auto & f = nodes.formulas[id];
      if (f.kind == ast::FormulaKind::And) {
        for (auto ch : f.children) {
          statics(ch, out);
        }
        return;
      }
      if (f.kind == ast::FormulaKind::Atom &&
        !changing.count(std::string(w_.domain.symbols.name(f.name))))
      {
        out.push_back(id);
      }
    };
  auto ground = [&](const std::vector<ast::Action> & list, std::vector<Grounded> & out) {
      for (size_t i = 0; i < list.size(); ++i) {
        std::vector<ast::FormulaId> fixed;
        if (list[i].precondition.valid()) {
          statics(list[i].precondition, fixed);
        }
        Bindings b;
        w_.domain_eval.for_each_binding(list[i].params, b, [&]() {
            Evaluator::Ctx c{state_};
            for (auto f : fixed) {
              Bindings fb = b;
              if (!w_.domain_eval.holds(f, c, fb)) {
                return true;
              }
            }
            out.push_back({i, b});
            return true;
          });
      }
    };
  ground(w_.domain.processes, processes_);
  ground(w_.domain.events, events_);
  std::set<std::string> derived;
  for (const auto & d : w_.domain.derived) {
    derived.insert(std::string(w_.domain.symbols.name(d.head.name)));
  }
  std::function<void(ast::FormulaId, std::vector<ast::FormulaId> &)> atoms =
    [&](ast::FormulaId id, std::vector<ast::FormulaId> & out) {
      const auto & f = nodes.formulas[id];
      const ast::Formula * a = &f;
      if (f.kind == ast::FormulaKind::And) {
        for (auto ch : f.children) {
          atoms(ch, out);
        }
        return;
      }
      if (f.kind == ast::FormulaKind::Not) {
        a = &nodes.formulas[f.children.at(0)];
      }
      if (a->kind == ast::FormulaKind::Atom &&
        !derived.count(std::string(w_.domain.symbols.name(a->name))))
      {
        out.push_back(id);
      }
    };
  auto collect = [&](const std::vector<ast::Action> & list, auto & out) {
      out.resize(list.size());
      for (size_t i = 0; i < list.size(); ++i) {
        if (list[i].precondition.valid()) {
          atoms(list[i].precondition, out[i]);
        }
      }
    };
  collect(w_.domain.processes, process_atoms_);
  collect(w_.domain.events, event_atoms_);
}

void Validator::refresh_live()
{
  if (live_version_ == atoms_version_) {
    return;
  }
  live_version_ = atoms_version_;
  auto mark = [&](const std::vector<Grounded> & gs, const auto & atoms, auto & live) {
      live.assign(gs.size(), 1);
      Evaluator::Ctx c{state_};
      for (size_t i = 0; i < gs.size(); ++i) {
        for (auto f : atoms[gs[i].index]) {
          Bindings b = gs[i].bindings;
          if (!w_.domain_eval.holds(f, c, b)) {
            live[i] = 0;
            break;
          }
        }
      }
    };
  mark(processes_, process_atoms_, live_processes_);
  mark(events_, event_atoms_, live_events_);
}

// Whether a process or event runs in `s`: a condition on an undefined value is false
bool Validator::active(const ast::Action & a, const State & s, Bindings b)
{
  if (!a.precondition.valid()) {
    return true;
  }
  Evaluator::Ctx c{s};
  c.exact = true;
  try {
    return w_.domain_eval.holds(a.precondition, c, b);
  } catch (const EvalError &) {
    return false;
  }
}

// The rate of change of each fluent in `s`: active processes and running durative actions
void Validator::derivative(const State & s, std::unordered_map<Key, double, KeyHash> & out)
{
  out.clear();
  std::vector<std::pair<Key, double>> contributions;
  for (size_t i = 0; i < processes_.size(); ++i) {
    const auto & g = processes_[i];
    const auto & a = w_.domain.processes[g.index];
    if (!live_processes_[i] || !active(a, s, g.bindings)) {
      continue;
    }
    Evaluator::Ctx c{s};
    Bindings b = g.bindings;
    if (a.effect.valid()) {
      w_.domain_eval.rates(a.effect, c, b, contributions);
    }
  }
  for (size_t r : running_) {
    StepInfo & st = steps_[r];
    const auto & a = w_.domain.durative_actions[st.action];
    if (!a.effect.valid()) {
      continue;
    }
    Bindings b = st.bindings;
    Evaluator::Ctx c{s, nullptr, Phase::Invariant, st.step->duration};
    w_.domain_eval.rates(a.effect, c, b, contributions);
  }
  for (const auto & [k, rate] : contributions) {
    out[k] += rate;
  }
}

// One Runge-Kutta 4 step of length h from `from` (k1: its derivative)
const State & Validator::patched(Scratch & s, const Numbers & values)
{
  if (s.version != atoms_version_) {
    s.state.atoms = state_.atoms;
    s.state.objects = state_.objects;
    s.state.numbers = state_.numbers;
    s.version = atoms_version_;
  } else {
    for (const auto & k : s.written) {
      s.state.numbers[k] = state_.numbers.at(k);
    }
  }
  s.written.clear();
  for (const auto & [k, v] : values) {
    s.state.numbers[k] = v;
    s.written.push_back(k);
  }
  s.state.changed(true);
  return s.state;
}

// The numbers changed as time passes, in state_ and in the copies
void Validator::commit(const Numbers & values)
{
  for (Scratch * s : {&stage_, &probe_}) {
    if (s->version == atoms_version_) {
      for (const auto & [k, v] : values) {
        s->state.numbers[k] = v;
      }
    }
  }
  for (const auto & [k, v] : values) {
    state_.numbers[k] = v;
  }
  state_.changed(w_.axioms_read_fluents);
}

// One Runge-Kutta 4 step of length h from state_ (k1: its derivative): the numbers it changes
void Validator::step_numbers(double h, const Numbers & k1, Numbers & to, const Numbers * over)
{
  // From state_, or state_ with `over` changed
  auto value = [&](const Key & key) {
      if (over) {
        if (auto it = over->find(key); it != over->end()) {
          return it->second;
        }
      }
      return state_.numbers.at(key);
    };
  // A fluent without a value stays without one (as ENHSP): only the defined ones change
  auto defined = [&](const Key & key) {return state_.numbers.count(key) > 0;};
  Numbers mixed;
  auto shifted = [&](const Numbers & k, double f) -> const State & {
      mixed = over ? *over : Numbers{};
      for (const auto & [key, rate] : k) {
        if (defined(key)) {
          mixed[key] = value(key) + f * rate;
        }
      }
      return patched(stage_, mixed);
    };
  Numbers k2, k3, k4;
  derivative(shifted(k1, h / 2), k2);
  derivative(shifted(k2, h / 2), k3);
  derivative(shifted(k3, h), k4);
  to.clear();
  auto rate = [](const Numbers & k, const Key & key) {
      auto it = k.find(key);
      return it == k.end() ? 0.0 : it->second;
    };
  std::unordered_set<Key, KeyHash> keys;
  for (const Numbers * k : std::array<const Numbers *, 4>{&k1, &k2, &k3, &k4}) {
    for (const auto & [key, r] : *k) {
      keys.insert(key);
    }
  }
  if (over) {
    to = *over;
  }
  for (const auto & key : keys) {
    if (defined(key)) {
      to[key] = value(key) + h / 6.0 * (rate(k1, key) + 2.0 * rate(k2, key) +
        2.0 * rate(k3, key) + rate(k4, key));
    }
  }
}

// What decides when something happens: process and event conditions, global constraints and
// the over-all conditions of the running actions, true or false in `s`
std::vector<uint8_t> Validator::watch(const State & s)
{
  std::vector<uint8_t> out;
  auto holds = [&](const ast::Action & a, Bindings b) {return active(a, s, std::move(b));};
  for (size_t i = 0; i < processes_.size(); ++i) {
    const auto & g = processes_[i];
    out.push_back(live_processes_[i] && holds(w_.domain.processes[g.index], g.bindings));
  }
  for (size_t i = 0; i < events_.size(); ++i) {
    const auto & g = events_[i];
    out.push_back(live_events_[i] && holds(w_.domain.events[g.index], g.bindings));
  }
  for (const auto & gc : w_.domain.global_constraints) {
    bool all = true;
    Bindings b;
    w_.domain_eval.for_each_binding(gc.params, b, [&]() {
        Evaluator::Ctx c{s};
        all = all && w_.domain_eval.holds(gc.condition, c, b);
        return all;
      });
    out.push_back(all);
  }
  for (size_t r : running_) {
    StepInfo & st = steps_[r];
    const auto & a = w_.domain.durative_actions[st.action];
    Bindings b = st.bindings;
    Evaluator::Ctx c{s, nullptr, Phase::Invariant, st.step->duration};
    out.push_back(!a.condition.valid() || w_.domain_eval.holds(a.condition, c, b));
  }
  return out;
}

// Time passes from `from` to `to`: continuous change under the active processes and running
// actions, stopping where some watched condition changes (an event fires, a process starts or
// stops, a constraint breaks)
void Validator::evolve(double from, double to)
{
  const double tol = w_.options.tolerance;
  const double max_step = std::max(1e-6, w_.options.continuous_step);
  double grow = max_step;
  double t = from;
  Numbers k1, next, probe;
  size_t guard = 0;
  // Conditions that keep flipping with almost no time between (friction around 0): from then on
  // sampled at each step, not located
  std::vector<uint8_t> flips, chattering;
  int tiny = 0;
  constexpr int kChatter = 8;
  auto differ = [&](const std::vector<uint8_t> & a, const std::vector<uint8_t> & b) {
      for (size_t i = 0; i < a.size(); ++i) {
        if (a[i] != b[i] && (i >= chattering.size() || !chattering[i])) {
          return true;
        }
      }
      return false;
    };
  try {
    while (t < to - tol) {
      refresh_live();
      derivative(state_, k1);
      if (k1.empty()) {
        // Nothing changes until the next happening: nor do the conditions
        break;
      }
      auto before = watch(state_);
      // Steps that grow while two halves agree with one (polynomial changes of low degree are
      // exact with any step) and shrink otherwise
      double h = std::min(grow, to - t);
      step_numbers(h, k1, next);
      if (h > max_step) {
        Numbers mid, twice, k_mid;
        step_numbers(h / 2, k1, mid);
        derivative(patched(probe_, mid), k_mid);
        step_numbers(h / 2, k_mid, twice, &mid);
        double err = 0.0;
        for (const auto & [key, v] : twice) {
          auto it = next.find(key);
          double w = it == next.end() ? state_.numbers.at(key) : it->second;
          err = std::max(err, std::fabs(v - w) / (1.0 + std::fabs(v)));
        }
        if (err > 1e-10) {
          grow = std::max(max_step, grow / 4);
          continue;
        }
        next.swap(twice);
      }
      if (differ(before, watch(patched(probe_, next)))) {
        // Bisection on the length of the step: the first instant the conditions change
        double lo = 0.0, hi = h;
        for (int it = 0; it < 60 && hi - lo > tol * 1e-3; ++it) {
          double mid = (lo + hi) / 2;
          step_numbers(mid, k1, probe);
          if (differ(before, watch(patched(probe_, probe)))) {
            hi = mid;
          } else {
            lo = mid;
          }
        }
        step_numbers(hi, k1, next);
        if (hi < max_step * 0.01) {
          auto now = watch(patched(probe_, next));
          flips.resize(now.size(), 0);
          for (size_t i = 0; i < now.size(); ++i) {
            flips[i] |= now[i] != before[i];
          }
          if (++tiny >= kChatter) {
            chattering = flips;
          }
        } else {
          tiny = 0;
          std::fill(flips.begin(), flips.end(), 0);
        }
        h = hi;
      }
      commit(next);
      t += h;
      state_.time = t;
      grow = std::min(grow * 2, 100.0);
      std::string when = "at " + std::to_string(t);
      check_global_constraints(state_, when);
      check_invariants(state_, when);
      fire_events(t);
      if (++guard > 50'000'000) {
        fail("validate.evaluation", "continuous change takes too many steps", 0);
      }
    }
  } catch (const EvalError & e) {
    fail("validate.evaluation", "continuous change: " + e.message, 0);
  }
  state_.time = to;
}

// Events whose condition holds fire, at once, again while some does (bounded)
void Validator::fire_events(double t)
{
  std::string fired;
  for (int round = 0; round < 1000; ++round) {
    std::vector<Changes> all;
    fired.clear();
    refresh_live();
    for (size_t i = 0; i < events_.size(); ++i) {
      if (!live_events_[i]) {
        continue;
      }
      const auto & g = events_[i];
      const auto & e = w_.domain.events[g.index];
      Bindings b = g.bindings;
      Evaluator::Ctx c{state_};
      if (!active(e, state_, b)) {
        continue;
      }
      try {
        Changes ch;
        if (e.effect.valid()) {
          w_.domain_eval.effects(e.effect, c, b, true, ch);
        }
        all.push_back(std::move(ch));
        if (fired.size() < 200) {
          fired += (fired.empty() ? "" : ", ") + w_.domain_eval.name(e.name);
          for (const auto & [var, obj] : g.bindings) {
            fired += " " + w_.object_name(obj);
          }
        }
      } catch (const EvalError & err) {
        fail("validate.evaluation", "event " + w_.domain_eval.name(e.name) + ": " + err.message,
          0);
      }
    }
    if (all.empty()) {
      return;
    }
    apply_changes(all, 0);
  }
  fail("validate.event_loop", "events keep firing at " + std::to_string(t) + " (" + fired +
    "): an event must make its own condition false", 0);
}

void Validator::apply_changes(const std::vector<Changes> & all, uint32_t line)
{
  State before;
  before.numbers = state_.numbers;
  ++atoms_version_;
  for (const auto & ch : all) {
    for (const auto & k : ch.dels) {
      state_.atoms.erase(k);
    }
  }
  for (const auto & ch : all) {
    for (const auto & k : ch.adds) {
      state_.atoms.insert(k);
    }
  }
  for (const auto & ch : all) {
    for (const auto & n : ch.nums) {
      auto old = before.numbers.find(n.key);
      auto & slot = state_.numbers[n.key];
      if (n.op != ast::AssignOp::Assign && old == before.numbers.end()) {
        fail("validate.undefined_fluent", w_.key_text(n.key, false) +
          " is changed but has no value", line);
      }
      switch (n.op) {
        case ast::AssignOp::Assign: slot = n.value; break;
        case ast::AssignOp::Increase: slot += n.value; break;
        case ast::AssignOp::Decrease: slot -= n.value; break;
        case ast::AssignOp::ScaleUp: slot *= n.value; break;
        case ast::AssignOp::ScaleDown:
          if (n.value == 0.0) {
            fail("validate.evaluation", "scale-down by zero", line);
          }
          slot /= n.value;
          break;
      }
    }
    for (const auto & o : ch.objs) {
      if (o.value) {
        state_.objects[o.key] = *o.value;
      } else {
        state_.objects.erase(o.key);
      }
    }
  }
  state_.changed(true);
}

bool Validator::constraint(FormulaId id, Evaluator & ev, Bindings & b)
{
  const ast::Formula & f = ev.nodes().formulas[id];
  auto at = [&](size_t k, FormulaId g) {
      Evaluator::Ctx c{trajectory_[k].second};
      return ev.holds(g, c, b);
    };
  const size_t n = trajectory_.size();
  switch (f.kind) {
    case ast::FormulaKind::And:
      for (FormulaId ch : f.children) {
        if (!constraint(ch, ev, b)) {
          return false;
        }
      }
      return true;
    case ast::FormulaKind::Forall: {
        bool ok = true;
        ev.for_each_binding(f.params, b, [&]() {
            ok = constraint(f.children.at(0), ev, b);
            return ok;
          });
        return ok;
      }
    case ast::FormulaKind::Preference: {
        bool v = constraint(f.children.at(0), ev, b);
        if (!v) {
          report_.violated_preferences[f.name.valid() ? ev.name(f.name) : "anonymous"] += 1;
        }
        return true;
      }
    case ast::FormulaKind::AtEnd:
      return at(n - 1, f.children.at(0));
    case ast::FormulaKind::Modal:
      break;
    default:
      return at(n - 1, id);
  }
  FormulaId p = f.children.at(0);
  auto time = [&](size_t k) {return trajectory_[k].first;};
  switch (f.modal) {
    case ast::ModalOp::Always:
      for (size_t k = 0; k < n; ++k) {
        if (!at(k, p)) {
          return false;
        }
      }
      return true;
    case ast::ModalOp::Sometime:
      for (size_t k = 0; k < n; ++k) {
        if (at(k, p)) {
          return true;
        }
      }
      return false;
    case ast::ModalOp::Within:
      for (size_t k = 0; k < n && time(k) <= f.times[0]; ++k) {
        if (at(k, p)) {
          return true;
        }
      }
      return false;
    case ast::ModalOp::AtMostOnce: {
        int rises = 0;
        bool was = false;
        for (size_t k = 0; k < n; ++k) {
          bool v = at(k, p);
          rises += v && !was;
          was = v;
        }
        return rises <= 1;
      }
    case ast::ModalOp::SometimeAfter: {
        FormulaId q = f.children.at(1);
        for (size_t k = 0; k < n; ++k) {
          if (at(k, p)) {
            bool found = false;
            for (size_t m = k; m < n && !found; ++m) {
              found = at(m, q);
            }
            if (!found) {
              return false;
            }
          }
        }
        return true;
      }
    case ast::ModalOp::SometimeBefore: {
        FormulaId q = f.children.at(1);
        bool seen_q = false;
        for (size_t k = 0; k < n; ++k) {
          if (at(k, p) && !seen_q) {
            return false;
          }
          seen_q = seen_q || at(k, q);
        }
        return true;
      }
    case ast::ModalOp::AlwaysWithin: {
        FormulaId q = f.children.at(1);
        for (size_t k = 0; k < n; ++k) {
          if (at(k, p)) {
            bool found = false;
            for (size_t m = k; m < n && time(m) <= time(k) + f.times[0] && !found; ++m) {
              found = at(m, q);
            }
            if (!found) {
              return false;
            }
          }
        }
        return true;
      }
    case ast::ModalOp::HoldDuring:
      for (size_t k = 0; k < n; ++k) {
        if (time(k) >= f.times[0] && time(k) < f.times[1] && !at(k, p)) {
          return false;
        }
      }
      return true;
    case ast::ModalOp::HoldAfter:
      for (size_t k = 0; k < n; ++k) {
        if (time(k) > f.times[0] && !at(k, p)) {
          return false;
        }
      }
      return true;
  }
  return true;
}

void Validator::finish()
{
  current_.reset();  // the goal and the metric are no step's
  report_.makespan = makespan_;
  const auto & p = w_.problem;
  if (p.goal.valid() && w_.options.check_goal) {
    Bindings b;
    Evaluator::Ctx c{state_};
    c.prefs = &report_.violated_preferences;
    try {
      if (!w_.problem_eval.holds(p.goal, c, b)) {
        Evaluator::Ctx plain{state_};
        fail("validate.goal_not_reached",
          "the goal does not hold at the end of the plan: " + w_.problem_eval.why(p.goal, plain,
          b), 0);
      }
    } catch (const EvalError & e) {
      fail("validate.evaluation", "evaluating the goal: " + e.message, 0);
    }
  }
  try {
    Bindings b;
    if (w_.domain.constraints.valid() && !constraint(w_.domain.constraints, w_.domain_eval, b)) {
      fail("validate.constraint_violated", "the plan violates the domain :constraints", 0);
    }
    if (p.constraints.valid() && !constraint(p.constraints, w_.problem_eval, b)) {
      fail("validate.constraint_violated", "the plan violates the problem :constraints", 0);
    }
  } catch (const EvalError & e) {
    fail("validate.evaluation", "evaluating constraints: " + e.message, 0);
  }
  if (p.metric) {
    Bindings b;
    Evaluator::Ctx c{state_};
    c.total_time = makespan_;
    c.prefs = &report_.violated_preferences;
    try {
      report_.metric = w_.problem_eval.number(p.metric->expr, c, b);
    } catch (const EvalError & e) {
      report_.diagnostics.push_back(
        Diagnostic::warning("validate.metric", "the metric cannot be computed: " + e.message));
    }
  }
  report_.status = ValidationStatus::Valid;
}

}  // namespace

ValidationReport validate(
  const ast::Domain & domain, const sem::DomainModel & domain_model,
  const ast::Problem & problem, const sem::ProblemModel & problem_model, const Plan & plan,
  const ValidateOptions & options)
{
  ValidationReport report;
  report.status = ValidationStatus::Invalid;
  World world(domain, domain_model, problem, problem_model, options);
  Validator v(world, plan, report);
  try {
    v.run();
  } catch (const Unsupported & u) {
    report.status = ValidationStatus::Unsupported;
    report.diagnostics.push_back(Diagnostic::error("validate.unsupported", u.message));
  } catch (const EvalError & e) {
    report.status = ValidationStatus::Invalid;
    report.diagnostics.push_back(Diagnostic::error("validate.evaluation", e.message));
  } catch (...) {
    // Stop: the failure is already in the diagnostics
    report.status = ValidationStatus::Invalid;
  }
  return report;
}

}  // namespace kairos::val
