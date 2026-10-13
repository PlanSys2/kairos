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
#include <unordered_map>
#include <utility>
#include <vector>

#include "state/eval.hpp"

namespace kairos::heuristics
{

RelaxedGraph::RelaxedGraph(
  const sas::Task & task, const std::vector<double> & costs, bool simplify_ops)
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
    goal_op_ = static_cast<uint32_t>(ops_.size());
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
  // Which numeric nodes each action can move (through the actions that change each variable)
  std::map<sas::NumVarId, std::vector<uint32_t>> changers;
  for (uint32_t a = 0; a < task.actions.size(); ++a) {
    for (const auto & e : num_effects_[a]) {
      auto & list = changers[e.var];
      if (list.empty() || list.back() != a) {
        list.push_back(a);
      }
    }
  }
  for (uint32_t n = 0; n < num_.size(); ++n) {
    std::vector<uint32_t> actions;
    for (const auto & [v, coef] : num_[n].e.terms) {
      if (auto it = changers.find(v); it != changers.end()) {
        actions.insert(actions.end(), it->second.begin(), it->second.end());
      }
    }
    std::sort(actions.begin(), actions.end());
    actions.erase(std::unique(actions.begin(), actions.end()), actions.end());
    for (uint32_t a : actions) {
      num_[n].contributors.push_back(a);
      action_num_[a].push_back(n);
    }
  }
  if (simplify_ops && num_.empty()) {
    simplify();
  }
  triggers_.assign(nodes_, {});
  for (uint32_t o = 0; o < ops_.size(); ++o) {
    for (uint32_t p : ops_[o].pre) {
      triggers_[p].push_back(o);
    }
    if (ops_[o].pre.empty()) {
      free_ops_.push_back(o);
    }
  }
  op_.assign(ops_.size(), OpState{});
  for (uint32_t o = 0; o < ops_.size(); ++o) {
    op_[o].preconditions = static_cast<uint32_t>(ops_[o].pre.size());
  }
}

void RelaxedGraph::set_goal(const std::vector<sas::Fact> & facts, bool any)
{
  Op & op = ops_[goal_op_];
  for (uint32_t p : op.pre) {
    auto & t = triggers_[p];
    t.erase(std::remove(t.begin(), t.end(), goal_op_), t.end());
  }
  op.pre.clear();
  for (const auto & f : facts) {
    op.pre.push_back(fact_offset_[f.var] + f.value);
  }
  std::sort(op.pre.begin(), op.pre.end());
  op.pre.erase(std::unique(op.pre.begin(), op.pre.end()), op.pre.end());
  for (uint32_t p : op.pre) {
    triggers_[p].push_back(goal_op_);
  }
  op_[goal_op_].preconditions = any && !op.pre.empty() ? 1 :
    static_cast<uint32_t>(op.pre.size());
  free_ops_.erase(std::remove(free_ops_.begin(), free_ops_.end(), goal_op_), free_ops_.end());
  if (op.pre.empty()) {
    free_ops_.push_back(goal_op_);
  }
  // Nothing to reset: the next run has a new epoch, and its operators count their
  // preconditions again when touched
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
  auto terms = e.terms;
  std::sort(terms.begin(), terms.end());
  auto key = std::make_tuple(strict, e.constant, std::move(terms));
  if (auto it = num_ids_.find(key); it != num_ids_.end()) {
    return it->second;
  }
  uint32_t id = add_node();
  num_ids_.emplace(std::move(key), id);
  node_num_[id] = static_cast<int32_t>(num_.size());
  num_node_id_.push_back(id);
  num_.push_back(NumNode{e, strict, {}});
  return id;
}

void RelaxedGraph::condition(const sas::Condition & c, std::vector<uint32_t> & out)
{
  auto derived_false = [&](sas::VarId v) {
      uint32_t n = false_node(v);
      if (n != kNone) {
        out.push_back(n);
      }
    };
  for (const auto & f : c.facts) {
    const auto & var = task_.variables[f.var];
    if (var.kind == sas::Variable::Kind::Derived && f.value == var.default_value) {
      derived_false(f.var);
      continue;
    }
    out.push_back(fact_offset_[f.var] + f.value);
  }
  for (const auto & f : c.not_facts) {
    const auto & var = task_.variables[f.var];
    if (var.kind == sas::Variable::Kind::Derived) {
      if (f.value != var.default_value) {
        derived_false(f.var);  // derived atom not true, i.e. false
        continue;
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

// A derived atom being false, as FD's negated axioms: in every rule that would make it true some
// literal of the body is false, which the relaxation reaches as another value of a variable.
// kNone (optimistic, as before) when a rule reads derived atoms or numbers
uint32_t RelaxedGraph::false_node(sas::VarId v)
{
  if (auto it = false_of_.find(v); it != false_of_.end()) {
    return it->second;
  }
  std::vector<const sas::Axiom *> rules;
  for (const auto & ax : task_.axioms) {
    if (ax.head.var != v) {
      continue;
    }
    bool basic = ax.body.numeric.empty() && ax.body.vars.empty();
    for (const auto & f : ax.body.facts) {
      basic = basic && task_.variables[f.var].kind != sas::Variable::Kind::Derived;
    }
    for (const auto & f : ax.body.not_facts) {
      basic = basic && task_.variables[f.var].kind != sas::Variable::Kind::Derived;
    }
    if (!basic) {
      false_of_[v] = kNone;
      return kNone;
    }
    rules.push_back(&ax);
  }
  uint32_t node = add_node();
  false_of_[v] = node;
  std::vector<uint32_t> each;  // per rule: reached when one of its literals is false
  for (const sas::Axiom * ax : rules) {
    uint32_t any = add_node();
    for (const auto & f : ax->body.facts) {
      add_op({neg_node(f.var, f.value)}, any, 0.0, kNone);
    }
    for (const auto & f : ax->body.not_facts) {
      add_op({fact_offset_[f.var] + f.value}, any, 0.0, kNone);
    }
    each.push_back(any);  // an empty body: never false
  }
  std::sort(each.begin(), each.end());
  add_op(std::move(each), node, 0.0, kNone);
  return node;
}

void RelaxedGraph::add_op(std::vector<uint32_t> pre, uint32_t effect, double base, uint32_t action)
{
  ops_.push_back(Op{std::move(pre), effect, base, action});
}

// One unary operator per effect (precondition and condition together); then, per effect, the
// duplicates and those dominated by one with a subset of the preconditions and no higher cost
// go (FD tests subsets of up to 5 preconditions): hiking 15, 65 430 -> 4 194 in FD
void RelaxedGraph::simplify()
{
  // Each effect gets its action's precondition: not when that is too big (patrol XL of B6,
  // 537 680 actions, ran out of memory)
  constexpr size_t kMaxEntries = size_t{2} << 20;
  size_t entries = 0;
  for (const Op & op : ops_) {
    if (op.action != kNone) {
      entries += op.pre.size() + ops_[act_op_[op.action]].pre.size();
    }
  }
  if (entries > kMaxEntries) {
    return;
  }
  std::vector<Op> unary, other;
  for (uint32_t o = 0; o < ops_.size(); ++o) {
    const Op & op = ops_[o];
    if (op.action == kNone) {
      if (node_action_[op.effect] < 0) {
        other.push_back(op);  // axioms, the goal, negated facts
      }
      continue;  // the precondition op of an action: folded into its effects
    }
    Op u = op;
    u.pre.clear();
    const auto & pre = ops_[act_op_[op.action]].pre;
    u.pre.insert(u.pre.end(), pre.begin(), pre.end());
    for (uint32_t p : op.pre) {
      if (p != act_node_[op.action]) {
        u.pre.push_back(p);
      }
    }
    std::sort(u.pre.begin(), u.pre.end());
    u.pre.erase(std::unique(u.pre.begin(), u.pre.end()), u.pre.end());
    unary.push_back(std::move(u));
  }
  // The cheapest operator per (effect, preconditions)
  using Key = std::pair<uint32_t, std::vector<uint32_t>>;
  struct KeyHash
  {
    size_t operator()(const Key & k) const
    {
      uint64_t h = k.first * 0x9E3779B97F4A7C15ull;
      for (uint32_t p : k.second) {
        h = (h ^ p) * 0x100000001B3ull;
      }
      return static_cast<size_t>(h ^ (h >> 29));
    }
  };
  std::unordered_map<Key, uint32_t, KeyHash> best;
  best.reserve(unary.size());
  for (uint32_t i = 0; i < unary.size(); ++i) {
    auto [it, fresh] = best.emplace(Key{unary[i].effect, unary[i].pre}, i);
    if (!fresh && unary[i].base < unary[it->second].base) {
      it->second = i;
    }
  }
  constexpr size_t kMaxTested = 5;
  std::vector<uint32_t> order;
  order.reserve(best.size());
  for (const auto & [key, i] : best) {
    order.push_back(i);
  }
  std::sort(order.begin(), order.end());  // deterministic, whatever the hash order
  std::vector<Op> kept;
  for (uint32_t i : order) {
    const Op & u = unary[i];
    bool dominated = false;
    const size_t k = u.pre.size();
    if (k > 0 && k <= kMaxTested) {
      Key sub{u.effect, {}};
      for (uint32_t mask = 0; mask + 1 < (1u << k) && !dominated; ++mask) {
        sub.second.clear();
        for (size_t b = 0; b < k; ++b) {
          if (mask & (1u << b)) {
            sub.second.push_back(u.pre[b]);
          }
        }
        auto it = best.find(sub);
        dominated = it != best.end() && unary[it->second].base <= u.base;
      }
    }
    if (!dominated) {
      kept.push_back(u);
    }
  }
  // Only when it removes most of them (hiking: 94 %): a smaller cut changes the relaxed plans
  // the search follows for little gain (storage, 12 %: B1 lost temporal instances)
  if (kept.size() * 2 > unary.size()) {
    return;
  }
  ops_.clear();
  for (auto & op : other) {
    if (op.effect == goal_) {
      goal_op_ = static_cast<uint32_t>(ops_.size());
    }
    ops_.push_back(std::move(op));
  }
  for (auto & op : kept) {
    ops_.push_back(std::move(op));
  }
  std::fill(act_op_.begin(), act_op_.end(), kNone);
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

// Some change of a variable of e depends on the state: its effect in this state says nothing
// about later states
bool RelaxedGraph::state_dependent(uint32_t action, const NumNode & n) const
{
  for (const auto & eff : num_effects_[action]) {
    bool used = std::any_of(
      n.e.terms.begin(), n.e.terms.end(), [&](const auto & t) {return t.first == eff.var;});
    if (!used) {
      continue;
    }
    bool constant = eff.value.linear && eff.value.linear->terms.empty() &&
      (eff.op == ast::AssignOp::Increase || eff.op == ast::AssignOp::Decrease ||
      eff.op == ast::AssignOp::Assign);
    if (!constant) {
      return true;
    }
  }
  return false;
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

void RelaxedGraph::push(double cost, uint32_t node)
{
  if (cost >= 0.0 && cost < static_cast<double>(kMaxBucket) && cost == std::floor(cost)) {
    auto k = static_cast<size_t>(cost);
    if (buckets_.size() <= k) {
      buckets_.resize(k + 1);
    }
    buckets_[k].push_back(node);
    ++in_buckets_;
    bucket_ = std::min(bucket_, k);
    return;
  }
  heap_.push(Entry{cost, seq_++, node});
}

bool RelaxedGraph::pop(double & cost, uint32_t & node)
{
  while (in_buckets_ && buckets_[bucket_].empty()) {
    ++bucket_;
  }
  double b = in_buckets_ ? static_cast<double>(bucket_) : kInfinity;
  double h = heap_.empty() ? kInfinity : heap_.top().cost;
  if (b == kInfinity && h == kInfinity) {
    return false;
  }
  if (b <= h) {
    cost = b;
    node = buckets_[bucket_].back();
    buckets_[bucket_].pop_back();
    --in_buckets_;
  } else {
    cost = h;
    node = heap_.top().node;
    heap_.pop();
  }
  return true;
}

void RelaxedGraph::relax(uint32_t node, double cost, uint32_t supporter, uint32_t repeat)
{
  NodeState & x = touch(node);
  if (cost < x.cost && node != blocked_) {
    x.cost = cost;
    x.supporter = supporter;
    x.repeat = repeat;
    if (node_action_[node] >= 0) {
      expand(node, cost);  // an action fires its effects right away, as FD's unary operators
    } else {
      push(cost, node);
    }
  }
}

void RelaxedGraph::expand(uint32_t n, double c)
{
  for (uint32_t o : triggers_[n]) {
    OpState & x = touch_op(o);
    if (mode_ == Mode::Add) {
      x.cost += c;
    } else {
      x.cost = std::max(x.cost, c);
    }
    if (--x.unsatisfied == 0) {
      if (o == goal_op_) {
        goal_trigger_ = n;
      }
      relax(ops_[o].effect, x.cost + ops_[o].base, o, 0);
    }
  }
  int32_t a = node_action_[n];
  if (a < 0) {
    return;
  }
  // An action reached: the numeric conditions it moves the right way
  for (uint32_t k : action_num_[static_cast<size_t>(a)]) {
    if (missing_[k] == 0.0) {
      continue;
    }
    double d = change(static_cast<uint32_t>(a), num_[k], *ctx_);
    uint32_t times = 1;
    if (d > 0.0) {
      times = repetitions(num_[k], missing_[k], d);
    } else if (!state_dependent(static_cast<uint32_t>(a), num_[k]) && !std::isinf(missing_[k])) {
      continue;  // a constant change that does not help
    }
    // Otherwise the change depends on values that may grow (or the expression is undefined
    // now): optimistic, once; never a false dead end
    double cand;
    if (mode_ == Mode::Add) {
      cand = c + costs_[static_cast<size_t>(a)] * times;
    } else {
      // The contributor needs its precondition and itself at least once; all contributors
      // together cover what is missing at the best cost per unit
      double bound = std::isinf(missing_[k]) ? 0.0 : missing_[k] * best_ratio_[k];
      cand = std::max(c + costs_[static_cast<size_t>(a)], bound);
    }
    relax(num_node_id_[k], cand, static_cast<uint32_t>(a), times);
  }
}

double RelaxedGraph::run(const EvalContext & ctx, Mode mode)
{
  mode_ = mode;
  explore(ctx, true);
  return cost(goal_);
}

void RelaxedGraph::run_all(const EvalContext & ctx)
{
  mode_ = Mode::Max;
  explore(ctx, false);
}

void RelaxedGraph::explore(const EvalContext & ctx, bool stop_at_goal)
{
  ctx_ = &ctx;
  goal_trigger_ = kNone;
  if (node_.size() != nodes_) {
    node_.assign(nodes_, NodeState{});
  }
  if (++epoch_ == 0) {  // wrapped: no stamp may look current
    for (auto & x : node_) {
      x.stamp = 0;
    }
    for (auto & x : op_) {
      x.stamp = 0;
    }
    epoch_ = 1;
  }
  for (auto & bucket : buckets_) {
    bucket.clear();
  }
  bucket_ = 0;
  in_buckets_ = 0;
  heap_ = {};
  seq_ = 0;
  missing_.assign(num_.size(), 0.0);
  best_ratio_.assign(num_.size(), kInfinity);
  for (uint32_t n = 0; n < num_.size(); ++n) {
    missing_[n] = missing(num_[n], ctx);
    if (mode_ == Mode::Max && missing_[n] != 0.0 && !std::isinf(missing_[n])) {
      for (uint32_t a : num_[n].contributors) {
        double d = change(a, num_[n], ctx);
        if (state_dependent(a, num_[n])) {
          best_ratio_[n] = 0.0;  // no bound from what it does now
        } else if (d > 0.0) {
          best_ratio_[n] = std::min(best_ratio_[n], costs_[a] / d);
        }
      }
    }
  }
  // As FD: operators without preconditions first, then the facts of the state
  for (uint32_t o : free_ops_) {
    relax(ops_[o].effect, ops_[o].base, o, 0);
  }
  auto start = [&](uint32_t n) {
      touch(n).initial = 1;
      relax(n, 0.0, kNone, 0);
    };
  for (sas::VarId v = 0; v < task_.variables.size(); ++v) {
    start(fact_offset_[v] + ctx.values[v]);
  }
  for (const auto & [v, n] : false_of_) {
    if (n != kNone && ctx.values[v] == task_.variables[v].default_value) {
      start(n);
    }
  }
  for (size_t i = 0; i < neg_fact_.size(); ++i) {
    auto [var, value] = neg_fact_[i];
    if (ctx.values[var] != value) {
      start(neg_id_[i]);
    }
  }
  for (uint32_t n = 0; n < num_.size(); ++n) {
    if (missing_[n] == 0.0) {
      start(num_node_id_[n]);
    }
  }
  double c = 0.0;
  uint32_t n = 0;
  while (pop(c, n)) {
    if (c > cost(n)) {
      continue;
    }
    if (n == goal_ && stop_at_goal) {
      break;
    }
    expand(n, c);
  }
}

std::optional<std::pair<sas::VarId, uint32_t>> RelaxedGraph::fact_of(uint32_t node) const
{
  if (fact_offset_.empty() || node >= fact_offset_.back() + task_.variables.back().size()) {
    return std::nullopt;
  }
  auto it = std::upper_bound(fact_offset_.begin(), fact_offset_.end(), node);
  auto var = static_cast<sas::VarId>(it - fact_offset_.begin() - 1);
  return std::make_pair(var, node - fact_offset_[var]);
}

std::vector<uint8_t> RelaxedGraph::reachable(const EvalContext & ctx, uint32_t blocked)
{
  blocked_ = blocked;
  run_all(ctx);
  blocked_ = kNone;
  std::vector<uint8_t> out(nodes_);
  for (uint32_t n = 0; n < nodes_; ++n) {
    out[n] = cost(n) < kInfinity ? 1 : 0;
  }
  return out;
}

double RelaxedGraph::relaxed_plan(std::vector<uint32_t> * preferred)
{
  // Best supporters back from the goal; an action counts once, or as many times as a numeric
  // condition needs it
  std::map<uint32_t, uint32_t> times;
  // Actions with an effect of the plan that would happen now (preferred only then, as FD)
  std::set<uint32_t> firing;
  if (seen_stamp_.size() != nodes_) {
    seen_stamp_.assign(nodes_, 0);
  }
  if (++seen_epoch_ == 0) {
    std::fill(seen_stamp_.begin(), seen_stamp_.end(), 0);
    seen_epoch_ = 1;
  }
  std::vector<uint32_t> stack{goal_};
  while (!stack.empty()) {
    uint32_t n = stack.back();
    stack.pop_back();
    if (seen_stamp_[n] == seen_epoch_ || !fresh(n) || node_[n].initial) {
      continue;  // a node of no run's plan has no supporter
    }
    seen_stamp_[n] = seen_epoch_;
    uint32_t s = node_[n].supporter;
    if (s == kNone) {
      continue;
    }
    if (node_num_[n] >= 0) {
      auto & t = times[s];
      t = std::max(t, node_[n].repeat);
      firing.insert(s);
      stack.push_back(act_node_[s]);
      continue;
    }
    const Op & op = ops_[s];
    if (op.action != kNone) {
      auto & t = times[op.action];
      t = std::max<uint32_t>(t, 1);
      bool now = std::all_of(op.pre.begin(), op.pre.end(), [&](uint32_t p) {
            return p == act_node_[op.action] || (fresh(p) && node_[p].initial);
          });
      if (now) {
        firing.insert(op.action);
      }
    }
    for (uint32_t p : op.pre) {
      stack.push_back(p);
    }
  }
  double h = 0.0;
  for (const auto & [a, t] : times) {
    h += costs_[a] * t;
    // Preferred: applicable now (the real precondition: the relaxed one leaves some out)
    if (preferred && firing.count(a) &&
      state::holds(task_.actions[a].pre, task_, ctx_->values, ctx_->numbers))
    {
      preferred->push_back(a);
    }
  }
  return h;
}

}  // namespace kairos::heuristics
