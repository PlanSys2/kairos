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
#include <utility>
#include <vector>

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
}

std::optional<StateId> StateSpace::successor(StateId s, uint32_t action)
{
  load(s);
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
