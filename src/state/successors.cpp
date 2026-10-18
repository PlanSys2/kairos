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

#include "state/successors.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "sas/symmetry.hpp"

namespace kairos::state
{

SuccessorGenerator::SuccessorGenerator(
  const sas::Task & task, const std::vector<const sas::Condition *> & conditions)
: task_(task), conditions_(conditions), extra_(conditions.size())
{
  for (size_t i = 0; i < conditions.size(); ++i) {
    const auto & c = *conditions[i];
    extra_[i] = !c.not_facts.empty() || !c.numeric.empty() || !c.vars.empty();
  }
  // Built with an explicit stack: deep trees (tens of thousands of variables) would overflow
  // the call stack. An item: the node to fill and its operators, each with how many of its
  // (sorted) facts are already decided above. The operators of an item are sorted by their next
  // variable once, and each group becomes one node of the chain of "don't care" children: an
  // operator moves down only when its variable is decided (copying the rest at every node of a
  // long chain was quadratic: snap tasks have a variable per durative action)
  struct Item
  {
    uint32_t node;
    std::vector<std::pair<uint32_t, uint32_t>> ops;
  };
  std::vector<Item> work;
  Item root{0, {}};
  for (uint32_t i = 0; i < conditions.size(); ++i) {
    root.ops.emplace_back(i, 0);
  }
  nodes_.emplace_back();
  work.push_back(std::move(root));
  struct Pending
  {
    uint32_t var;
    uint32_t op;
    uint32_t pos;
  };
  std::vector<Pending> pending;
  while (!work.empty()) {
    Item item = std::move(work.back());
    work.pop_back();
    Node top;
    top.leaves_begin = static_cast<uint32_t>(leaves_.size());
    pending.clear();
    for (const auto & [op, pos] : item.ops) {
      const auto & facts = conditions_[op]->facts;
      if (pos == facts.size()) {
        leaves_.push_back(op);
      } else {
        pending.push_back(Pending{facts[pos].var, op, pos});
      }
    }
    top.leaves_end = static_cast<uint32_t>(leaves_.size());
    std::stable_sort(
      pending.begin(), pending.end(),
      [](const Pending & x, const Pending & y) {return x.var < y.var;});
    uint32_t current = item.node;
    Node node = top;
    for (size_t i = 0; i < pending.size(); ) {
      size_t j = i;
      while (j < pending.size() && pending[j].var == pending[i].var) {
        ++j;
      }
      uint32_t var = pending[i].var;
      node.var = var;
      uint32_t domain = task.variables[var].size();
      std::vector<std::vector<std::pair<uint32_t, uint32_t>>> by_value(domain);
      for (size_t k = i; k < j; ++k) {
        const auto & f = conditions_[pending[k].op]->facts[pending[k].pos];
        by_value[f.value].emplace_back(pending[k].op, pending[k].pos + 1);
      }
      node.children = static_cast<uint32_t>(child_.size());
      child_.resize(child_.size() + domain, kNone);
      for (uint32_t v = 0; v < domain; ++v) {
        if (!by_value[v].empty()) {
          auto id = static_cast<uint32_t>(nodes_.size());
          nodes_.emplace_back();
          child_[node.children + v] = id;
          work.push_back(Item{id, std::move(by_value[v])});
        }
      }
      uint32_t next = kNone;
      if (j < pending.size()) {
        next = static_cast<uint32_t>(nodes_.size());
        nodes_.emplace_back();
        node.dont_care = next;
      }
      nodes_[current] = node;
      current = next;
      node = Node{};
      node.leaves_begin = node.leaves_end = static_cast<uint32_t>(leaves_.size());
      i = j;
    }
    if (pending.empty()) {
      nodes_[item.node] = top;
    }
  }
}

void SuccessorGenerator::applicable(
  const uint32_t * values, const double * numbers, std::vector<uint32_t> & out,
  double duration) const
{
  stack_.clear();
  stack_.push_back(0);
  while (!stack_.empty()) {
    const Node & n = nodes_[stack_.back()];
    stack_.pop_back();
    for (uint32_t i = n.leaves_begin; i < n.leaves_end; ++i) {
      uint32_t op = leaves_[i];
      if (!extra_[op] || holds(*conditions_[op], task_, values, numbers, duration)) {
        out.push_back(op);
      }
    }
    if (n.var == kNone) {
      continue;
    }
    if (n.dont_care != kNone) {
      stack_.push_back(n.dont_care);
    }
    uint32_t c = child_[n.children + values[n.var]];
    if (c != kNone) {
      stack_.push_back(c);
    }
  }
}

bool apply_effects(
  const sas::Task & task, const std::vector<sas::Fact> & changes,
  const std::vector<sas::Effect> & effects, sas::Timing timing, const uint32_t * values,
  const double * numbers, std::vector<uint32_t> & out_values, std::vector<double> & out_numbers,
  const uint32_t * start_values, const double * start_numbers, double duration)
{
  size_t nv = task.variables.size();
  size_t nn = task.numeric_fluents.size();
  out_values.assign(values, values + nv);
  out_numbers.assign(numbers, numbers + nn);
  // Values other than the last set in this step, to find conflicts; only the entries touched
  // are reset (states may have tens of thousands of variables)
  thread_local std::vector<uint32_t> set_to;
  thread_local std::vector<sas::VarId> touched;
  if (set_to.size() < nv) {
    set_to.assign(nv, UINT32_MAX);
  }
  struct Reset
  {
    ~Reset()
    {
      for (sas::VarId v : touched) {
        set_to[v] = UINT32_MAX;
      }
      touched.clear();
    }
  } reset;
  auto assign_last = [&](const sas::Fact & f) {
      if (set_to[f.var] == UINT32_MAX) {
        out_values[f.var] = f.value;
      }
    };
  auto assign_other = [&](const sas::Fact & f) {
      if (set_to[f.var] != UINT32_MAX && set_to[f.var] != f.value) {
        return false;  // two values at once
      }
      set_to[f.var] = f.value;
      touched.push_back(f.var);
      out_values[f.var] = f.value;
      return true;
    };
  // Deletes first
  for (const auto & f : changes) {
    if (f.value == task.variables[f.var].none()) {
      assign_last(f);
    }
  }
  auto fires = [&](const sas::Effect & e) {
      if (e.timing != timing || e.continuous) {
        return false;
      }
      if (!holds(e.condition, task, values, numbers, duration)) {
        return false;
      }
      if (!e.start_condition.empty()) {
        return start_values && holds(e.start_condition, task, start_values, start_numbers,
                 duration);
      }
      return true;
    };
  thread_local std::vector<uint8_t> firing;
  firing.assign(effects.size(), 0);
  for (size_t i = 0; i < effects.size(); ++i) {
    firing[i] = fires(effects[i]) ? 1 : 0;
  }
  for (size_t i = 0; i < effects.size(); ++i) {
    const auto & e = effects[i];
    if (firing[i] && e.kind == sas::Effect::Kind::Assign &&
      e.fact.value == task.variables[e.fact.var].none())
    {
      assign_last(e.fact);
    }
  }
  // Then the other values
  for (const auto & f : changes) {
    if (f.value != task.variables[f.var].none() && !assign_other(f)) {
      return false;
    }
  }
  for (size_t i = 0; i < effects.size(); ++i) {
    if (!firing[i]) {
      continue;
    }
    const auto & e = effects[i];
    switch (e.kind) {
      case sas::Effect::Kind::Assign:
        if (e.fact.value != task.variables[e.fact.var].none() && !assign_other(e.fact)) {
          return false;
        }
        break;
      case sas::Effect::Kind::Copy: {
          const auto & tv = task.variables[e.target];
          const auto & sv = task.variables[e.source];
          uint32_t x = values[e.source];
          uint32_t v = tv.none();
          if (x < sv.objects.size()) {
            auto it = std::lower_bound(tv.objects.begin(), tv.objects.end(), sv.objects[x]);
            v = static_cast<uint32_t>(it - tv.objects.begin());
          }
          if (v != tv.none() && !assign_other(sas::Fact{e.target, v})) {
            return false;
          }
          if (v == tv.none()) {
            out_values[e.target] = v;
          }
          break;
        }
      case sas::Effect::Kind::Numeric: {
          double x = evaluate(e.value, numbers, duration);
          if (std::isnan(x)) {
            return false;  // undefined value
          }
          double & slot = out_numbers[e.num];
          if (e.op != ast::AssignOp::Assign && std::isnan(numbers[e.num])) {
            return false;  // changes a fluent without a value
          }
          switch (e.op) {
            case ast::AssignOp::Assign: slot = x; break;
            case ast::AssignOp::Increase: slot += x; break;
            case ast::AssignOp::Decrease: slot -= x; break;
            case ast::AssignOp::ScaleUp: slot *= x; break;
            case ast::AssignOp::ScaleDown:
              if (x == 0.0) {
                return false;
              }
              slot /= x;
              break;
          }
          break;
        }
    }
  }
  return true;
}

namespace
{

std::vector<const sas::Condition *> preconditions(const sas::Task & task)
{
  std::vector<const sas::Condition *> out;
  out.reserve(task.actions.size());
  for (const auto & a : task.actions) {
    out.push_back(&a.pre);
  }
  return out;
}

}  // namespace

StateSpace::StateSpace(const sas::Task & task, StateRegistry & registry)
: task_(task), registry_(registry), generator_(task, preconditions(task))
{
}

void StateSpace::load(StateId s)
{
  if (loaded_ == s) {
    return;
  }
  registry_.unpack(s, values_);
  auto n = registry_.numbers(s);
  numbers_.assign(n.begin(), n.end());
  loaded_ = s;
}

void StateSpace::applicable(StateId s, std::vector<uint32_t> & out)
{
  load(s);
  generator_.applicable(values_.data(), numbers_.data(), out);
  // Symmetric objects that look the same here: of an action and its image, only the first
  if (!task_.symmetries.empty() && out.size() > 1) {
    thread_local std::vector<uint32_t> stamp;
    thread_local uint32_t epoch = 0;
    if (stamp.size() < task_.actions.size()) {
      stamp.assign(task_.actions.size(), 0);
    }
    if (++epoch == 0) {
      std::fill(stamp.begin(), stamp.end(), 0);
      epoch = 1;
    }
    for (uint32_t a : out) {
      stamp[a] = epoch;
    }
    for (const auto & sym : task_.symmetries) {
      if (!sas::invariant(sym, values_.data(), numbers_.data())) {
        continue;
      }
      std::erase_if(out, [&](uint32_t a) {
          uint32_t b = sym.image[a];
          return b < a && stamp[b] == epoch;
        });
    }
  }
  // PDDL+: an event that can happen does, before anything else
  if (task_.first_event != UINT32_MAX &&
    std::any_of(out.begin(), out.end(), [&](uint32_t a) {return a >= task_.first_event;}))
  {
    std::erase_if(out, [&](uint32_t a) {return a < task_.first_event;});
  }
}

// PDDL+ time passing: steps of clock_delta (the processes active at the start of each, with
// Runge-Kutta 4), one, or (long) the most before a numeric condition changes (a short wait
// crosses it then: what becomes true is reached, what becomes false can be acted on before);
// never past a hard deadline
bool StateSpace::integrate(double max_dt, bool long_wait)
{
  next_values_ = values_;
  next_numbers_ = numbers_;
  last_wait_ = 0.0;
  thread_local std::vector<const sas::Effect *> active;
  thread_local std::vector<uint8_t> watched, now;
  thread_local std::vector<double> before;
  auto watch = [&](const std::vector<double> & x, std::vector<uint8_t> & out) {
      out.clear();
      for (const auto & c : task_.clock_watch) {
        out.push_back(holds_numeric(c, x.data()) ? 1 : 0);
      }
    };
  auto derivative = [&](const std::vector<double> & x, std::vector<double> & k) {
      k.assign(x.size(), 0.0);
      for (const sas::Effect * pr : active) {
        const auto & r = *pr;
        if (std::isnan(x[r.num])) {
          continue;  // a fluent without a value stays without one (as ENHSP)
        }
        double v = evaluate(r.value, x.data());
        if (std::isnan(v)) {
          return false;  // an undefined rate
        }
        k[r.num] += r.op == ast::AssignOp::Decrease ? -v : v;
      }
      return true;
    };
  // One Runge-Kutta 4 step of length h from x (the active rates of x's start)
  thread_local std::vector<double> rk_k1, rk_k2, rk_k3, rk_k4, rk_at;
  auto rk4 = [&](const std::vector<double> & x, double h, std::vector<double> & out) {
      auto shift = [&](const std::vector<double> & k, double f) -> const std::vector<double> & {
          rk_at = x;
          for (size_t i = 0; i < rk_at.size(); ++i) {
            rk_at[i] += f * k[i];
          }
          return rk_at;
        };
      if (!derivative(x, rk_k1) || !derivative(shift(rk_k1, h / 2), rk_k2) ||
        !derivative(shift(rk_k2, h / 2), rk_k3) || !derivative(shift(rk_k3, h), rk_k4))
      {
        return false;
      }
      out = x;
      for (size_t j = 0; j < out.size(); ++j) {
        double s = rk_k1[j] + 2.0 * rk_k2[j] + 2.0 * rk_k3[j] + rk_k4[j];
        if (s != 0.0) {
          out[j] += h / 6.0 * s;
        }
      }
      return true;
    };
  // Rounded to 1e-9: 50 steps of 0.2 reach 10, not 9.999999999999998
  auto round_changed = [&](std::vector<double> & x) {
      for (size_t j = 0; j < x.size(); ++j) {
        if (x[j] != numbers_[j] && std::fabs(x[j]) < 1e6) {
          x[j] = std::round(x[j] * 1e9) / 1e9;
        }
      }
    };
  // Time left to the nearest deadline from x (infinity if none); nullopt: a hard one is reached
  auto deadline_left = [&](const std::vector<double> & x) -> std::optional<double> {
      double best = std::numeric_limits<double>::infinity();
      for (const auto & d : task_.clock_deadlines) {
        if (values_[d.when.var] != d.when.value) {
          continue;
        }
        double left = (d.bound ? x[*d.bound] : d.at) - x[d.clock];
        if (std::isnan(left)) {
          continue;
        }
        if (left <= kDeadlineSlack) {
          if (d.hard) {
            return std::nullopt;
          }
          continue;
        }
        best = std::min(best, left);
      }
      return best;
    };
  auto set_active = [&](const std::vector<double> & x) {
      active.clear();
      for (const auto & r : task_.clock_rates) {
        if (holds(r.condition, task_, values_.data(), x.data())) {
          active.push_back(&r);
        }
      }
    };
  if (!long_wait) {
    // One step, up to a deadline
    auto left = deadline_left(next_numbers_);
    if (!left) {
      return false;  // something must happen first
    }
    double dt = std::min(max_dt, *left);
    set_active(next_numbers_);
    if (active.empty()) {
      return true;  // nothing changes
    }
    bool crossing_only = task_.short_only_at_change && max_dt == task_.clock_delta;
    if (crossing_only) {
      watch(numbers_, watched);
    }
    int steps = std::max(1, static_cast<int>(std::ceil(dt / kClockStep - 1e-9)));
    double h = dt / steps;
    for (int i = 0; i < steps; ++i) {
      if (!rk4(next_numbers_, h, before)) {
        return false;
      }
      next_numbers_.swap(before);
    }
    round_changed(next_numbers_);
    last_wait_ = dt;
    if (crossing_only && dt >= max_dt) {
      watch(next_numbers_, now);
      if (now == watched) {
        return false;  // nothing crossed: the long wait goes further
      }
    }
    return true;
  }
  // Long: blocks of steps that double while the error is small and nothing watched changes,
  // halve otherwise; it stops at the last step before a change, or at a deadline
  watch(numbers_, watched);
  const double step = max_dt;
  bool stopped = false;
  int64_t blocks = 1;
  int64_t taken = 0;
  thread_local std::vector<double> full, half, result;
  while (taken < kLongSteps) {
    auto left = deadline_left(next_numbers_);
    if (!left) {
      if (taken == 0) {
        return false;
      }
      stopped = true;
      break;
    }
    set_active(next_numbers_);
    if (active.empty()) {
      break;  // nothing changes
    }
    blocks = std::min(blocks, kLongSteps - taken);
    double dt = static_cast<double>(blocks) * step;
    bool to_deadline = *left < dt;
    if (to_deadline) {
      dt = *left;
    }
    if (!rk4(next_numbers_, dt, full)) {
      return false;
    }
    if (dt > step + 1e-12) {
      // The error of the block: one step against two halves
      if (!rk4(next_numbers_, dt / 2, half) || !rk4(half, dt / 2, result)) {
        return false;
      }
      double err = 0.0;
      for (size_t j = 0; j < full.size(); ++j) {
        if (!std::isnan(full[j])) {
          err = std::max(err, std::fabs(full[j] - result[j]) / (1.0 + std::fabs(result[j])));
        }
      }
      if (err > 1e-10) {
        blocks = std::max<int64_t>(1, blocks / 2);
        if (blocks > 1 || !to_deadline) {
          continue;
        }
      }
      full.swap(result);
    }
    round_changed(full);
    watch(full, now);
    if (now != watched) {
      if (dt > step + 1e-12) {
        blocks = std::max<int64_t>(1, blocks / 2);  // the change is inside: smaller blocks
        if (to_deadline) {
          blocks = std::max<int64_t>(1, static_cast<int64_t>(std::floor(*left / step)));
          blocks = std::max<int64_t>(1, blocks / 2);
        }
        continue;
      }
      stopped = true;  // just before the change
      break;
    }
    next_numbers_.swap(full);
    last_wait_ += dt;
    taken += to_deadline ? kLongSteps : blocks;
    if (to_deadline) {
      stopped = true;  // a deadline
      break;
    }
    blocks = std::min<int64_t>(blocks * 2, 4096);
  }
  // A long wait that nothing stops changes nothing that matters: not applicable. One that ran
  // all its steps towards a deadline still ahead is (a drive of 2316 s: the next wait goes on)
  bool towards = false;
  if (taken >= kLongSteps) {
    auto left = deadline_left(next_numbers_);
    towards = left && std::isfinite(*left);
  }
  return !long_wait || ((stopped || towards) && last_wait_ > 0.0);
}

bool StateSpace::holds_numeric(const sas::NumericCondition & c, const double * numbers) const
{
  return compare(c.op, evaluate(c.lhs, numbers), evaluate(c.rhs, numbers));
}

std::optional<StateId> StateSpace::successor(StateId s, uint32_t action)
{
  load(s);
  if (action == task_.clock || action == task_.long_clock || action == task_.epsilon_clock) {
    double dt = action == task_.epsilon_clock ? task_.clock_epsilon : task_.clock_delta;
    if (!integrate(dt, action == task_.long_clock)) {
      return std::nullopt;
    }
    for (const auto & f : task_.actions[action].changes) {
      next_values_[f.var] = f.value;  // what a wait marks (temporal/hybrid.hpp)
    }
    return registry_.insert(next_values_, next_numbers_).first;
  }
  const auto & a = task_.actions[action];
  if (!apply_effects(
      task_, a.changes, a.effects, sas::Timing::Instant, values_.data(), numbers_.data(),
      next_values_, next_numbers_))
  {
    return std::nullopt;
  }
  StateId id = registry_.insert(next_values_, next_numbers_).first;
  return id;
}

bool StateSpace::is_goal(StateId s)
{
  if (task_.goal_unreachable) {
    return false;
  }
  load(s);
  return holds(task_.goal, task_, values_.data(), numbers_.data());
}

}  // namespace kairos::state
