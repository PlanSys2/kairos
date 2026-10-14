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

#include "temporal/hybrid.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include "sas/symmetry.hpp"
#include "sas/translate.hpp"
#include "state/eval.hpp"
#include "temporal/snap.hpp"

namespace kairos::temporal
{

namespace
{

constexpr double kSlack = 1e-9;  // a clock this close to its bound has reached it
constexpr double kTolerance = 1e-6;  // the validator's, for global and over all conditions

ground::Expr constant(double v)
{
  ground::Expr c;
  c.op = ground::ExprOp::Constant;
  c.value = v;
  return c;
}

ground::Expr fluent(sas::NumVarId v)
{
  ground::Expr f;
  f.op = ground::ExprOp::Fluent;
  f.fluent = v;
  return f;
}

ground::Expr binary(ground::ExprOp op, ground::Expr a, ground::Expr b)
{
  ground::Expr e;
  e.op = op;
  e.children = {std::move(a), std::move(b)};
  return e;
}

sas::NumericExpr numeric(ground::Expr e)
{
  auto linear = sas::linearize(e);
  return sas::NumericExpr{std::move(e), std::move(linear)};
}

// lhs op rhs
sas::NumericCondition compare(sas::CompareOp op, ground::Expr lhs, ground::Expr rhs)
{
  sas::NumericCondition c;
  c.op = op;
  c.difference = sas::linearize(binary(ground::ExprOp::Sub, lhs, rhs));
  c.lhs = numeric(std::move(lhs));
  c.rhs = numeric(std::move(rhs));
  return c;
}

sas::Effect numeric_effect(sas::NumVarId var, ast::AssignOp op, ground::Expr value)
{
  sas::Effect e;
  e.kind = sas::Effect::Kind::Numeric;
  e.timing = sas::Timing::Instant;
  e.num = var;
  e.op = op;
  e.value = numeric(std::move(value));
  return e;
}

// #t replaced by the length of the step
ground::Expr at_delta(const ground::Expr & e, double delta)
{
  if (e.op == ground::ExprOp::ContinuousT) {
    return constant(delta);
  }
  ground::Expr out = e;
  for (auto & ch : out.children) {
    ch = at_delta(ch, delta);
  }
  return out;
}

bool uses_duration(const ground::Expr & e)
{
  return e.op == ground::ExprOp::Duration ||
         std::any_of(e.children.begin(), e.children.end(), uses_duration);
}

// d/dt of a continuous change: X for (* #t X), else its change in one second less at once
ground::Expr rate_of(const ground::Expr & e)
{
  if (e.op == ground::ExprOp::Mul) {
    auto t = std::find_if(e.children.begin(), e.children.end(), [](const ground::Expr & c) {
          return c.op == ground::ExprOp::ContinuousT;
        });
    if (t != e.children.end()) {
      ground::Expr rest;
      rest.op = ground::ExprOp::Mul;
      for (auto it = e.children.begin(); it != e.children.end(); ++it) {
        if (it != t) {
          rest.children.push_back(*it);
        }
      }
      if (rest.children.empty()) {
        return constant(1.0);
      }
      return rest.children.size() == 1 ? rest.children[0] : rest;
    }
  }
  return binary(ground::ExprOp::Sub, at_delta(e, 1.0), at_delta(e, 0.0));
}

// The non-strict numeric bounds of a condition widened by tol, as the validator does (a fuel
// that ends at -1e-12 after rounding is not a violation of >= 0; strict ones stay strict)
sas::Condition loosen(sas::Condition c, double tol)
{
  for (auto & n : c.numeric) {
    double by = n.op == sas::CompareOp::Ge ? -tol : (n.op == sas::CompareOp::Le ? tol : 0.0);
    if (by != 0.0) {
      n = compare(n.op, n.lhs.tree, binary(ground::ExprOp::Add, n.rhs.tree, constant(by)));
    }
  }
  return c;
}

void append(sas::Condition & to, const sas::Condition & from)
{
  to.facts.insert(to.facts.end(), from.facts.begin(), from.facts.end());
  to.not_facts.insert(to.not_facts.end(), from.not_facts.begin(), from.not_facts.end());
  to.numeric.insert(to.numeric.end(), from.numeric.begin(), from.numeric.end());
  to.vars.insert(to.vars.end(), from.vars.begin(), from.vars.end());
}

class Builder
{
public:
  Builder(Hybrid & h, double delta)
  : h_(h), delta_(delta) {}

  // A numeric variable: function `name` (one per name) over `args`
  sas::NumVarId add_fluent(
    const std::string & name, std::optional<double> initial,
    const std::vector<sas::ObjectId> & args = {})
  {
    auto [it, fresh] = functions_.emplace(name, h_.task.function_names.size());
    if (fresh) {
      h_.task.function_names.push_back(name);
    }
    ground::FluentInfo info;
    info.function = ground::FunctionId(static_cast<uint32_t>(it->second));
    info.args = args;
    h_.task.numeric_fluents.push_back(info);
    h_.task.initial_numeric.push_back(initial);
    return static_cast<sas::NumVarId>(h_.task.initial_numeric.size() - 1);
  }

  // A rate of `r.num` (integrated by the clock while r.condition holds) and its Euler step
  // (for the heuristics)
  void rate(sas::Effect r, ground::Expr value)
  {
    r.kind = sas::Effect::Kind::Numeric;
    r.timing = sas::Timing::Instant;
    r.continuous = false;
    sas::Effect step = r;
    step.value = numeric(binary(ground::ExprOp::Mul, constant(delta_), value));
    r.value = numeric(std::move(value));
    h_.task.clock_rates.push_back(std::move(r));
    wait_.effects.push_back(std::move(step));
  }

  sas::Action take_wait() {return std::move(wait_);}

private:
  Hybrid & h_;
  double delta_;
  sas::Action wait_;
  std::map<std::string, size_t> functions_;
};

sas::Effect increase(sas::NumVarId var)
{
  sas::Effect e;
  e.num = var;
  e.op = ast::AssignOp::Increase;
  return e;
}

}  // namespace

Hybrid hybrid(
  const sas::Task & task, double delta, Waits waits, double epsilon, bool symmetries)
{
  bool long_waits = waits != Waits::Short;
  Hybrid h;
  h.delta = delta;
  bool timed = !task.durative_actions.empty() || !task.timed_events.empty();
  std::optional<SnapTask> snapped;
  if (timed) {
    snapped = snap(task);
    h.unsupported = snapped->unsupported;
    h.task = snapped->task;
  } else {
    h.task = task;
  }
  h.task.processes.clear();
  h.task.events.clear();
  Builder b(h, delta);
  // Timed literals: with the events, after the wait
  std::vector<sas::Action> tils;
  if (snapped) {
    tils.assign(
      h.task.actions.begin() + static_cast<std::ptrdiff_t>(snapped->tils), h.task.actions.end());
    h.task.actions.resize(snapped->tils);
    h.first_start = static_cast<uint32_t>(snapped->instant);
    h.durative = static_cast<uint32_t>(task.durative_actions.size());
    h.running = snapped->running;
    h.time = b.add_fluent("kairos-time", 0.0);
    b.rate(increase(*h.time), constant(1.0));
  }
  // Per durative action, for the heuristics: its clock, the last moment it may end, and the
  // constant rates of its own continuous effects
  struct Projection
  {
    sas::NumVarId clock;
    std::optional<sas::NumVarId> until;
    std::map<sas::NumVarId, double> rates;
  };
  std::vector<Projection> projections;
  // Durative actions: a clock and the bounds of the duration, set at start
  for (uint32_t i = 0; i < h.durative; ++i) {
    const auto & da = task.durative_actions[i];
    std::string name = sas::instance_name(task, da);
    sas::Fact runs{h.running[i], 0};
    // Per schema, over the action's arguments (symmetric objects map clocks to clocks)
    const std::string & schema = task.durative_action_names[da.schema];
    sas::NumVarId clock = b.add_fluent("kairos-clock " + schema, 0.0, da.args);
    auto & start = h.task.actions[h.first_start + 2 * i];
    start.effects.push_back(numeric_effect(clock, ast::AssignOp::Assign, constant(0.0)));
    sas::Effect tick = increase(clock);
    tick.condition.facts.push_back(runs);
    b.rate(tick, constant(1.0));
    Projection proj{clock, std::nullopt, {}};
    for (size_t j = 0; j < da.duration.size(); ++j) {
      const auto & d = da.duration[j];
      if (d.timing != sas::Timing::Start && h.unsupported.empty()) {
        h.unsupported = name + " bounds its duration with values at end";
      }
      sas::NumVarId bound = b.add_fluent(
        "kairos-bound-" + std::to_string(j) + " " + schema, std::nullopt, da.args);
      h.task.actions[h.first_start + 2 * i].effects.push_back(
        numeric_effect(bound, ast::AssignOp::Assign, d.value.tree));
      auto & end = h.task.actions[h.first_start + 2 * i + 1];
      bool upper = d.op == sas::CompareOp::Le || d.op == sas::CompareOp::Lt;
      if (!upper) {
        end.pre.numeric.push_back(
          compare(
            sas::CompareOp::Ge, fluent(clock),
            binary(ground::ExprOp::Sub, fluent(bound), constant(kSlack))));
      }
      if (upper || d.op == sas::CompareOp::Eq) {
        end.pre.numeric.push_back(
          compare(
            sas::CompareOp::Le, fluent(clock),
            binary(ground::ExprOp::Add, fluent(bound), constant(kSlack))));
      }
      // = and <=: time may not pass the bound; >=: it stops there once
      h.task.clock_deadlines.push_back(
        {runs, clock, bound, 0.0, d.op == sas::CompareOp::Eq || upper});
      if (upper || d.op == sas::CompareOp::Eq) {
        proj.until = bound;
      }
    }
    for (const auto & e : da.effects) {
      if (!e.continuous) {
        continue;
      }
      if (uses_duration(e.value.tree) && h.unsupported.empty()) {
        h.unsupported = name + " has a continuous effect that reads ?duration";
      }
      sas::Effect r = e;
      r.condition.facts.push_back(runs);
      r.start_condition = {};
      ground::Expr rate = rate_of(e.value.tree);
      auto lin = sas::linearize(rate);
      if (e.condition.empty() && lin && lin->terms.empty()) {
        proj.rates[e.num] += e.op == ast::AssignOp::Decrease ? -lin->constant : lin->constant;
      }
      b.rate(r, std::move(rate));
    }
    h.over_all.push_back(da.over_all);
    projections.push_back(std::move(proj));
  }
  // Processes: their rates while their conditions hold
  for (const auto & p : task.processes) {
    for (const auto & e : p.effects) {
      if (e.kind != sas::Effect::Kind::Numeric) {
        continue;
      }
      sas::Effect r = e;
      r.condition = p.pre;
      append(r.condition, e.condition);
      b.rate(r, rate_of(e.value.tree));
    }
  }
  sas::Action wait = b.take_wait();
  // Waiting until something changes: one step for the heuristics too (they count steps; a
  // long wait then lowers h by as many as it takes). Without long waits: never applicable
  sas::Action wait_long = wait;
  if (!long_waits) {
    wait_long.pre.numeric.push_back(compare(sas::CompareOp::Lt, constant(1.0), constant(0.0)));
  }
  wait.schema = static_cast<uint32_t>(h.task.action_names.size());
  h.task.action_names.push_back("kairos-wait");
  wait_long.schema = static_cast<uint32_t>(h.task.action_names.size());
  h.task.action_names.push_back("kairos-wait-long");
  h.wait = static_cast<uint32_t>(h.task.actions.size());
  h.task.actions.push_back(std::move(wait));
  h.wait_long = static_cast<uint32_t>(h.task.actions.size());
  h.task.actions.push_back(std::move(wait_long));
  h.task.clock = h.wait;
  h.task.long_clock = h.wait_long;
  h.task.clock_delta = delta;
  h.task.short_only_at_change = waits == Waits::Crossing;
  // With durative actions or timed literals: happenings that interfere need epsilon between.
  // Only with both waits: chains of epsilon waits widen the search a lot
  if (timed && epsilon > 0.0 && waits == Waits::Both) {
    sas::Action wait_epsilon = h.task.actions[h.wait];
    for (auto & e : wait_epsilon.effects) {
      e.value = numeric(binary(ground::ExprOp::Mul, constant(epsilon / delta), e.value.tree));
    }
    wait_epsilon.schema = static_cast<uint32_t>(h.task.action_names.size());
    h.task.action_names.push_back("kairos-wait-epsilon");
    h.wait_epsilon = static_cast<uint32_t>(h.task.actions.size());
    h.task.actions.push_back(std::move(wait_epsilon));
    h.task.epsilon_clock = h.wait_epsilon;
    h.task.clock_epsilon = epsilon;
  }
  // The events, as actions (named as events), then the timed literals at their time
  auto offset = static_cast<uint32_t>(h.task.action_names.size());
  h.task.action_names.insert(
    h.task.action_names.end(), task.event_names.begin(), task.event_names.end());
  h.first_event = static_cast<uint32_t>(h.task.actions.size());
  h.task.first_event = h.first_event;
  for (const auto & ev : task.events) {
    sas::Action a = ev;
    a.schema = offset + ev.schema;
    h.task.actions.push_back(std::move(a));
  }
  for (size_t j = 0; j < tils.size(); ++j) {
    auto & til = tils[j];
    double at = snapped->til_time[j];
    til.pre.numeric.push_back(
      compare(sas::CompareOp::Ge, fluent(*h.time), constant(at - kSlack)));
    // Before it happens (its precondition: the ones before it did), time stops at it
    h.task.clock_deadlines.push_back({til.pre.facts.at(0), *h.time, std::nullopt, at, true});
    h.task.actions.push_back(std::move(til));
  }
  h.always = loosen(h.task.always, kTolerance);
  for (const auto & c : h.over_all) {
    h.loose_over_all.push_back(loosen(c, kTolerance));
  }
  // What a long wait watches: every numeric condition (facts do not change while time passes)
  auto watch = [&](const sas::Condition & c) {
      for (const auto & n : c.numeric) {
        if (std::find(h.task.clock_watch.begin(), h.task.clock_watch.end(), n) ==
          h.task.clock_watch.end())
        {
          h.task.clock_watch.push_back(n);
        }
      }
    };
  for (uint32_t a = 0; a < h.task.actions.size(); ++a) {
    if (a == h.wait || a == h.wait_long || a == h.wait_epsilon) {
      continue;
    }
    watch(h.task.actions[a].pre);
    for (const auto & e : h.task.actions[a].effects) {
      watch(e.condition);
    }
  }
  for (const auto & r : h.task.clock_rates) {
    watch(r.condition);
  }
  watch(h.task.goal);
  watch(h.task.always);
  for (const auto & c : h.over_all) {
    watch(c);
  }
  // Objects it does not tell apart (tanks of a generator): one order of their actions
  if (symmetries) {
    h.task.symmetries = sas::find_symmetries(h.task);
  }
  // For the heuristics, each end also needs its over all conditions as they will be at the
  // last moment, with its own rates: (> fuel 0) of a generator that burns 1 per second is
  // (> (+ fuel (* -1 (- until clock))) 0). Waiting leaves it as it is (the relaxation ignores
  // what waiting burns); refuelling helps, so the heuristics count it
  h.guide = h.task;
  bool projected = false;
  for (uint32_t i = 0; i < h.durative; ++i) {
    const auto & proj = projections[i];
    if (!proj.until || proj.rates.empty()) {
      continue;
    }
    std::function<ground::Expr(const ground::Expr &)> at_end = [&](const ground::Expr & e) {
        if (e.op == ground::ExprOp::Fluent) {
          auto r = proj.rates.find(e.fluent);
          if (r != proj.rates.end()) {
            return binary(
              ground::ExprOp::Add, e,
              binary(
                ground::ExprOp::Mul, constant(r->second),
                binary(ground::ExprOp::Sub, fluent(*proj.until), fluent(proj.clock))));
          }
          return e;
        }
        ground::Expr out = e;
        for (auto & c : out.children) {
          c = at_end(c);
        }
        return out;
      };
    auto & pre = h.guide.actions[h.first_start + 2 * i + 1].pre;
    for (const auto & n : h.over_all[i].numeric) {
      auto c = compare(n.op, at_end(n.lhs.tree), at_end(n.rhs.tree));
      if (!(c.lhs == n.lhs && c.rhs == n.rhs)) {
        pre.numeric.push_back(std::move(c));
        projected = true;
      }
    }
  }
  if (!projected) {
    h.guide = {};
  }
  h.has_guide = projected;
  return h;
}

bool Hybrid::accept(const uint32_t * child_values, const double * child_numbers) const
{
  if (!always.empty() && !state::holds(always, task, child_values, child_numbers)) {
    return false;
  }
  for (uint32_t i = 0; i < durative; ++i) {
    if (child_values[running[i]] == 0 &&
      !state::holds(loose_over_all[i], task, child_values, child_numbers))
    {
      return false;
    }
  }
  return true;
}

Plan Hybrid::plan(const std::vector<uint32_t> & actions, state::StateSpace & space) const
{
  Plan p;
  p.temporal = true;
  double t = 0.0;
  state::StateId s = space.registry().initial();
  std::vector<size_t> open(durative, 0);
  for (uint32_t a : actions) {
    if (auto next = space.successor(s, a)) {
      s = *next;
    }
    if (a == wait || a == wait_long || a == wait_epsilon) {
      // The time itself if kept; else the sum, rounded as the clocks (1e-9)
      t = time ? space.numbers(s)[*time] : std::round((t + space.last_wait()) * 1e9) / 1e9;
      continue;
    }
    if (is_event(a)) {
      continue;
    }
    bool timed = a >= first_start && a < first_start + 2 * durative;
    uint32_t i = timed ? (a - first_start) / 2 : 0;
    if (timed && (a - first_start) % 2 == 1) {
      auto & st = p.steps[open[i]];
      st.duration = t - st.time;
      continue;
    }
    const auto & act = task.actions[a];
    PlanStep step;
    step.time = t;
    step.name = task.action_names[act.schema];
    for (auto o : act.args) {
      step.args.push_back(task.object_names[o.index()]);
    }
    if (timed) {
      open[i] = p.steps.size();
    }
    p.steps.push_back(std::move(step));
  }
  p.end = t;
  return p;
}

}  // namespace kairos::temporal
