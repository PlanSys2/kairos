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

#include "heuristics/relaxed.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <set>
#include <utility>
#include <vector>

#include "state/eval.hpp"

namespace kairos::heuristics
{

RelaxedGraph::RelaxedGraph(const sas::Task & task, const std::vector<double> & costs)
: task_(task), costs_(costs)
{
  fact_offset_.resize(task.variables.size());
  for (sas::VarId v = 0; v < task.variables.size(); ++v) {
    fact_offset_[v] = nodes_;
    nodes_ += task.variables[v].size();
  }
  node_num_.assign(nodes_, -1);
  node_action_.assign(nodes_, -1);
  num_effects_.resize(task.actions.size());
  action_num_.resize(task.actions.size());
  for (uint32_t a = 0; a < task.actions.size(); ++a) {
    for (const auto & e : task.actions[a].effects) {
      if (e.kind == sas::Effect::Kind::Numeric && !e.continuous) {
        num_effects_[a].push_back(NumEffect{e.num, e.op, e.value});
        bool constant = (e.op == ast::AssignOp::Increase || e.op == ast::AssignOp::Decrease) &&
          e.value.linear && e.value.linear->terms.empty() && e.condition.empty();
        simple_numeric_ = simple_numeric_ && constant;
      }
    }
  }
  // Action nodes and their precondition ops
  act_node_.resize(task.actions.size());
  act_op_.resize(task.actions.size());
  for (uint32_t a = 0; a < task.actions.size(); ++a) {
    act_node_[a] = add_node();
    node_action_[act_node_[a]] = static_cast<int32_t>(a);
  }
  for (uint32_t a = 0; a < task.actions.size(); ++a) {
    std::vector<uint32_t> pre;
    condition(task.actions[a].pre, pre);
    act_op_[a] = static_cast<uint32_t>(ops_.size());
    add_op(std::move(pre), act_node_[a], 0.0, kNone);
    // Effects
    for (const auto & f : task.actions[a].changes) {
      add_op({act_node_[a]}, fact_offset_[f.var] + f.value, costs_[a], a);
    }
    for (const auto & e : task.actions[a].effects) {
      if (e.kind != sas::Effect::Kind::Assign) {
        continue;
      }
      std::vector<uint32_t> p{act_node_[a]};
      condition(e.condition, p);
      add_op(std::move(p), fact_offset_[e.fact.var] + e.fact.value, costs_[a], a);
    }
  }
  for (const auto & ax : task.axioms) {
    std::vector<uint32_t> pre;
    condition(ax.body, pre);
    add_op(std::move(pre), fact_offset_[ax.head.var] + ax.head.value, 0.0, kNone);
  }
  goal_ = add_node();
  {
    std::vector<uint32_t> pre;
    condition(task.goal, pre);
    add_op(std::move(pre), goal_, 0.0, kNone);
  }
  // A negated fact is reached by any other value of its variable
  for (size_t i = 0; i < neg_fact_.size(); ++i) {
    auto [var, value] = neg_fact_[i];
    for (uint32_t x = 0; x < task.variables[var].size(); ++x) {
      if (x != value) {
        add_op({fact_offset_[var] + x}, neg_id_[i], 0.0, kNone);
      }
    }
  }
  // Which numeric nodes each action can move
  for (uint32_t n = 0; n < num_.size(); ++n) {
    std::set<sas::NumVarId> vars;
    for (const auto & [v, coef] : num_[n].e.terms) {
      vars.insert(v);
    }
    for (uint32_t a = 0; a < task.actions.size(); ++a) {
      bool touches = std::any_of(
        num_effects_[a].begin(), num_effects_[a].end(),
        [&](const NumEffect & e) {return vars.count(e.var) > 0;});
      if (touches) {
        num_[n].contributors.push_back(a);
        action_num_[a].push_back(n);
      }
    }
  }
  triggers_.assign(nodes_, {});
  for (uint32_t o = 0; o < ops_.size(); ++o) {
    for (uint32_t p : ops_[o].pre) {
      triggers_[p].push_back(o);
    }
  }
}

uint32_t RelaxedGraph::add_node()
{
  node_num_.push_back(-1);
  node_action_.push_back(-1);
  return nodes_++;
}

uint32_t RelaxedGraph::neg_node(sas::VarId var, uint32_t value)
{
  auto it = neg_.find({var, value});
  if (it != neg_.end()) {
    return it->second;
  }
  uint32_t id = add_node();
  neg_.emplace(std::make_pair(var, value), id);
  neg_fact_.emplace_back(var, value);
  neg_id_.push_back(id);
  return id;
}

uint32_t RelaxedGraph::num_node(const sas::LinearExpr & e, bool strict)
{
  uint32_t id = add_node();
  node_num_[id] = static_cast<int32_t>(num_.size());
  num_node_id_.push_back(id);
  num_.push_back(NumNode{e, strict, {}});
  return id;
}

void RelaxedGraph::condition(const sas::Condition & c, std::vector<uint32_t> & out)
{
  for (const auto & f : c.facts) {
    const auto & var = task_.variables[f.var];
    if (var.kind == sas::Variable::Kind::Derived && f.value == var.default_value) {
      continue;  // a derived atom being false: optimistic
    }
    out.push_back(fact_offset_[f.var] + f.value);
  }
  for (const auto & f : c.not_facts) {
    const auto & var = task_.variables[f.var];
    if (var.kind == sas::Variable::Kind::Derived) {
      if (f.value != var.default_value) {
        continue;  // derived atom not true, i.e. false: optimistic
      }
      out.push_back(fact_offset_[f.var] + (1 - var.default_value));  // not false: true
      continue;
    }
    out.push_back(neg_node(f.var, f.value));
  }
  for (const auto & n : c.numeric) {
    if (!n.difference) {
      continue;  // non-linear: optimistic
    }
    sas::LinearExpr neg = *n.difference;
    neg.constant = -neg.constant;
    for (auto & [v, coef] : neg.terms) {
      coef = -coef;
    }
    switch (n.op) {
      case sas::CompareOp::Ge: out.push_back(num_node(*n.difference, false)); break;
      case sas::CompareOp::Gt: out.push_back(num_node(*n.difference, true)); break;
      case sas::CompareOp::Le: out.push_back(num_node(neg, false)); break;
      case sas::CompareOp::Lt: out.push_back(num_node(neg, true)); break;
      case sas::CompareOp::Eq:
        out.push_back(num_node(*n.difference, false));
        out.push_back(num_node(neg, false));
        break;
      case sas::CompareOp::Ne: break;  // optimistic
    }
  }
  std::sort(out.begin(), out.end());
  out.erase(std::unique(out.begin(), out.end()), out.end());
}

void RelaxedGraph::add_op(std::vector<uint32_t> pre, uint32_t effect, double base, uint32_t action)
{
  ops_.push_back(Op{std::move(pre), effect, base, action});
}

// How much the action moves e up in this state
double RelaxedGraph::change(uint32_t action, const NumNode & n, const EvalContext & ctx) const
{
  double total = 0.0;
  for (const auto & eff : num_effects_[action]) {
    double coef = 0.0;
    for (const auto & [v, c] : n.e.terms) {
      if (v == eff.var) {
        coef = c;
      }
    }
    if (coef == 0.0) {
      continue;
    }
    double value = state::evaluate(eff.value, ctx.numbers);
    double old = ctx.numbers[eff.var];
    double d = 0.0;
    switch (eff.op) {
      case ast::AssignOp::Increase: d = value; break;
      case ast::AssignOp::Decrease: d = -value; break;
      case ast::AssignOp::Assign: d = std::isnan(old) ? value : value - old; break;
      case ast::AssignOp::ScaleUp: d = old * (value - 1.0); break;
      case ast::AssignOp::ScaleDown: d = value == 0.0 ? 0.0 : old * (1.0 / value - 1.0); break;
    }
    if (std::isnan(d)) {
      continue;
    }
    total += coef * d;
  }
  return total;
}

// How far e is from holding in this state (0 if it holds; infinity if undefined)
double RelaxedGraph::missing(const NumNode & n, const EvalContext & ctx) const
{
  double v = n.e.constant;
  for (const auto & [var, c] : n.e.terms) {
    v += c * ctx.numbers[var];
  }
  if (std::isnan(v)) {
    return kInfinity;
  }
  bool holds = n.strict ? v > 0.0 : v >= 0.0;
  return holds ? 0.0 : -v;
}

uint32_t RelaxedGraph::repetitions(const NumNode & n, double gap, double step) const
{
  if (std::isinf(gap)) {
    return 1;
  }
  double k = n.strict ? std::floor(gap / step) + 1.0 : std::ceil(gap / step - 1e-9);
  return static_cast<uint32_t>(std::max(1.0, std::min(k, 1e9)));
}

void RelaxedGraph::relax(uint32_t node, double cost, uint32_t supporter, uint32_t repeat)
{
  if (cost < cost_[node]) {
    cost_[node] = cost;
    supporter_[node] = supporter;
    repeat_[node] = repeat;
    queue_.emplace(cost, node);
  }
}

double RelaxedGraph::run(const EvalContext & ctx, Mode mode)
{
  mode_ = mode;
  ctx_ = &ctx;
  cost_.assign(nodes_, kInfinity);
  op_cost_.assign(ops_.size(), 0.0);
  unsatisfied_.resize(ops_.size());
  supporter_.assign(nodes_, kNone);
  repeat_.assign(nodes_, 0);
  initial_.assign(nodes_, 0);
  queue_ = {};
  for (uint32_t o = 0; o < ops_.size(); ++o) {
    unsatisfied_[o] = static_cast<uint32_t>(ops_[o].pre.size());
  }
  auto start = [&](uint32_t n) {
      initial_[n] = 1;
      relax(n, 0.0, kNone, 0);
    };
  for (sas::VarId v = 0; v < task_.variables.size(); ++v) {
    start(fact_offset_[v] + ctx.values[v]);
  }
  for (size_t i = 0; i < neg_fact_.size(); ++i) {
    auto [var, value] = neg_fact_[i];
    if (ctx.values[var] != value) {
      start(neg_id_[i]);
    }
  }
  missing_.assign(num_.size(), 0.0);
  best_ratio_.assign(num_.size(), kInfinity);
  for (uint32_t n = 0; n < num_.size(); ++n) {
    missing_[n] = missing(num_[n], ctx);
    if (missing_[n] == 0.0) {
      start(num_node_id_[n]);
      continue;
    }
    if (mode == Mode::Max && !std::isinf(missing_[n])) {
      for (uint32_t a : num_[n].contributors) {
        double d = change(a, num_[n], ctx);
        if (d > 0.0) {
          best_ratio_[n] = std::min(best_ratio_[n], costs_[a] / d);
        }
      }
    }
  }
  for (uint32_t o = 0; o < ops_.size(); ++o) {
    if (ops_[o].pre.empty()) {
      relax(ops_[o].effect, ops_[o].base, o, 0);
    }
  }
  while (!queue_.empty()) {
    auto [c, n] = queue_.top();
    queue_.pop();
    if (c > cost_[n]) {
      continue;
    }
    if (n == goal_) {
      break;
    }
    for (uint32_t o : triggers_[n]) {
      if (mode == Mode::Add) {
        op_cost_[o] += c;
      } else {
        op_cost_[o] = std::max(op_cost_[o], c);
      }
      if (--unsatisfied_[o] == 0) {
        relax(ops_[o].effect, op_cost_[o] + ops_[o].base, o, 0);
      }
    }
    int32_t a = node_action_[n];
    if (a < 0) {
      continue;
    }
    // An action reached: the numeric conditions it moves the right way
    for (uint32_t k : action_num_[static_cast<size_t>(a)]) {
      if (missing_[k] == 0.0) {
        continue;
      }
      double d = change(static_cast<uint32_t>(a), num_[k], ctx);
      if (!(d > 0.0)) {
        continue;
      }
      uint32_t times = repetitions(num_[k], missing_[k], d);
      double cand;
      if (mode == Mode::Add) {
        cand = c + costs_[static_cast<size_t>(a)] * times;
      } else {
        double bound = std::isinf(missing_[k]) ? costs_[static_cast<size_t>(a)] :
          missing_[k] * best_ratio_[k];
        cand = std::max(c, bound);
      }
      relax(num_node_id_[k], cand, static_cast<uint32_t>(a), times);
    }
  }
  return cost_[goal_];
}

double RelaxedGraph::relaxed_plan(std::vector<uint32_t> * preferred)
{
  // Best supporters back from the goal; an action counts once, or as many times as a numeric
  // condition needs it
  std::map<uint32_t, uint32_t> times;
  std::vector<uint8_t> seen(nodes_, 0);
  std::vector<uint32_t> stack{goal_};
  while (!stack.empty()) {
    uint32_t n = stack.back();
    stack.pop_back();
    if (seen[n] || initial_[n]) {
      continue;
    }
    seen[n] = 1;
    uint32_t s = supporter_[n];
    if (s == kNone) {
      continue;
    }
    if (node_num_[n] >= 0) {
      auto & t = times[s];
      t = std::max(t, repeat_[n]);
      stack.push_back(act_node_[s]);
      continue;
    }
    const Op & op = ops_[s];
    if (op.action != kNone) {
      auto & t = times[op.action];
      t = std::max<uint32_t>(t, 1);
    }
    for (uint32_t p : op.pre) {
      stack.push_back(p);
    }
  }
  double h = 0.0;
  for (const auto & [a, t] : times) {
    h += costs_[a] * t;
    // Preferred: applicable now (the real precondition: the relaxed one leaves some out)
    if (preferred && state::holds(task_.actions[a].pre, task_, ctx_->values, ctx_->numbers)) {
      preferred->push_back(a);
    }
  }
  return h;
}

}  // namespace kairos::heuristics
