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

#include "temporal/compress.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "state/eval.hpp"
#include "temporal/access.hpp"
#include "temporal/stn.hpp"

namespace kairos::temporal
{

namespace
{

using ground::Expr;
using ground::ExprOp;

Expr constant(double v)
{
  Expr e;
  e.op = ExprOp::Constant;
  e.value = v;
  return e;
}

// ?duration replaced by the expression of the duration
bool substitute(Expr & e, const Expr & duration)
{
  if (e.op == ExprOp::Duration) {
    e = duration;
    return true;
  }
  bool any = false;
  for (auto & c : e.children) {
    any = substitute(c, duration) || any;
  }
  return any;
}

// The linear form of an expression, when it is one
std::optional<sas::LinearExpr> linear(const Expr & e)
{
  sas::LinearExpr out;
  switch (e.op) {
    case ExprOp::Constant:
      out.constant = e.value;
      return out;
    case ExprOp::Fluent:
      out.terms.emplace_back(static_cast<sas::NumVarId>(e.fluent), 1.0);
      return out;
    case ExprOp::Neg: {
        auto a = linear(e.children.at(0));
        if (!a) {
          return std::nullopt;
        }
        a->constant = -a->constant;
        for (auto & [v, c] : a->terms) {
          c = -c;
        }
        return a;
      }
    case ExprOp::Add:
    case ExprOp::Sub: {
        std::map<sas::NumVarId, double> sum;
        for (size_t i = 0; i < e.children.size(); ++i) {
          auto a = linear(e.children[i]);
          if (!a) {
            return std::nullopt;
          }
          double sign = (e.op == ExprOp::Sub && i > 0) ? -1.0 : 1.0;
          out.constant += sign * a->constant;
          for (const auto & [v, c] : a->terms) {
            sum[v] += sign * c;
          }
        }
        for (const auto & [v, c] : sum) {
          if (c != 0.0) {
            out.terms.emplace_back(v, c);
          }
        }
        return out;
      }
    case ExprOp::Mul:
    case ExprOp::Div: {
        if (e.children.size() != 2) {
          return std::nullopt;
        }
        auto a = linear(e.children[0]);
        auto b = linear(e.children[1]);
        if (!a || !b) {
          return std::nullopt;
        }
        if (e.op == ExprOp::Div) {
          if (!b->terms.empty() || b->constant == 0.0) {
            return std::nullopt;
          }
          b->constant = 1.0 / b->constant;
        }
        if (!a->terms.empty() && !b->terms.empty()) {
          return std::nullopt;
        }
        const auto & k = a->terms.empty() ? *a : *b;
        auto & x = a->terms.empty() ? *b : *a;
        x.constant *= k.constant;
        for (auto & [v, c] : x.terms) {
          c *= k.constant;
        }
        return x;
      }
    default:
      return std::nullopt;
  }
}

void substitute(sas::NumericExpr & e, const Expr & duration)
{
  if (substitute(e.tree, duration)) {
    e.linear = linear(e.tree);
  }
}

void substitute(sas::Condition & c, const Expr & duration)
{
  for (auto & n : c.numeric) {
    bool l = substitute(n.lhs.tree, duration);
    bool r = substitute(n.rhs.tree, duration);
    if (l || r) {
      n.lhs.linear = linear(n.lhs.tree);
      n.rhs.linear = linear(n.rhs.tree);
      Expr diff;
      diff.op = ExprOp::Sub;
      diff.children = {n.lhs.tree, n.rhs.tree};
      n.difference = linear(diff);
    }
  }
}

// The duration M2 uses: the equality, or else the lower bound, or else the upper one
sas::NumericExpr chosen_duration(const sas::DurativeAction & a, double epsilon)
{
  const sas::NumericExpr * eq = nullptr;
  const sas::NumericExpr * lower = nullptr;
  const sas::NumericExpr * upper = nullptr;
  bool strict = false;
  for (const auto & d : a.duration) {
    switch (d.op) {
      case sas::CompareOp::Eq: eq = &d.value; break;
      case sas::CompareOp::Ge: lower = &d.value; break;
      case sas::CompareOp::Gt: lower = &d.value; strict = true; break;
      case sas::CompareOp::Le: case sas::CompareOp::Lt: upper = &d.value; break;
      default: break;
    }
  }
  if (eq) {
    return *eq;
  }
  if (lower) {
    if (!strict) {
      return *lower;
    }
    sas::NumericExpr e;
    e.tree.op = ExprOp::Add;
    e.tree.children = {lower->tree, constant(epsilon)};
    e.linear = linear(e.tree);
    return e;
  }
  if (upper) {
    return *upper;
  }
  sas::NumericExpr e;
  e.tree = constant(0.0);
  e.linear = sas::LinearExpr{};
  return e;
}

// Adds the facts and conditions of b to a; false if they ask two values of a variable
bool merge(sas::Condition & a, const sas::Condition & b)
{
  for (const auto & f : b.facts) {
    auto it = std::find_if(
      a.facts.begin(), a.facts.end(), [&](const sas::Fact & g) {return g.var == f.var;});
    if (it != a.facts.end()) {
      if (it->value != f.value) {
        return false;
      }
      continue;
    }
    a.facts.push_back(f);
  }
  for (const auto & f : b.not_facts) {
    if (std::find(a.not_facts.begin(), a.not_facts.end(), f) == a.not_facts.end()) {
      a.not_facts.push_back(f);
    }
  }
  a.numeric.insert(a.numeric.end(), b.numeric.begin(), b.numeric.end());
  a.vars.insert(a.vars.end(), b.vars.begin(), b.vars.end());
  return true;
}

// The over all or at end conditions the start effects do not give; nullopt if the start
// effects break them (the action could never run)
std::optional<sas::Condition> after_start(
  const sas::Condition & c, const std::map<sas::VarId, uint32_t> & start)
{
  sas::Condition out;
  for (const auto & f : c.facts) {
    auto it = start.find(f.var);
    if (it == start.end()) {
      out.facts.push_back(f);
    } else if (it->second != f.value) {
      return std::nullopt;
    }
  }
  for (const auto & f : c.not_facts) {
    auto it = start.find(f.var);
    if (it == start.end()) {
      out.not_facts.push_back(f);
    } else if (it->second == f.value) {
      return std::nullopt;
    }
  }
  out.numeric = c.numeric;
  out.vars = c.vars;
  return out;
}

// A fluent read after the start effects: v replaced by what they leave (v + x, v - x or x),
// so that a condition checked before the action sees the state after its start
void after_numeric(ground::Expr & e, const std::map<sas::NumVarId, const sas::Effect *> & starts)
{
  if (e.op == ground::ExprOp::Fluent) {
    auto it = starts.find(static_cast<sas::NumVarId>(e.fluent));
    if (it == starts.end()) {
      return;
    }
    const sas::Effect & s = *it->second;
    ground::Expr v = e;
    switch (s.op) {
      case ast::AssignOp::Increase: e = Expr{}; e.op = ground::ExprOp::Add; break;
      case ast::AssignOp::Decrease: e = Expr{}; e.op = ground::ExprOp::Sub; break;
      case ast::AssignOp::ScaleUp: e = Expr{}; e.op = ground::ExprOp::Mul; break;
      case ast::AssignOp::ScaleDown: e = Expr{}; e.op = ground::ExprOp::Div; break;
      case ast::AssignOp::Assign: e = s.value.tree; return;
    }
    e.children = {v, s.value.tree};
    return;
  }
  for (auto & c : e.children) {
    after_numeric(c, starts);
  }
}

void after_numeric(sas::Condition & c, const std::map<sas::NumVarId, const sas::Effect *> & starts)
{
  if (starts.empty()) {
    return;
  }
  for (auto & n : c.numeric) {
    after_numeric(n.lhs.tree, starts);
    after_numeric(n.rhs.tree, starts);
    n.lhs.linear.reset();
    n.rhs.linear.reset();
    n.difference.reset();
  }
}

void sort_condition(sas::Condition & c)
{
  std::sort(c.facts.begin(), c.facts.end());
  std::sort(c.not_facts.begin(), c.not_facts.end());
}

}  // namespace

Compressed compress(const sas::Task & task)
{
  Compressed c;
  c.task = task;
  c.task.durative_actions.clear();
  c.instant = task.actions.size();
  auto offset = static_cast<uint32_t>(c.task.action_names.size());
  c.task.action_names.insert(
    c.task.action_names.end(), task.durative_action_names.begin(),
    task.durative_action_names.end());
  for (const auto & da : task.durative_actions) {
    sas::NumericExpr d = chosen_duration(da, 0.001);
    c.duration.push_back(d);
    sas::Action a;
    a.schema = offset + da.schema;
    a.args = da.args;
    // Start values (the start facts and unconditional start assignments)
    auto unconditional = [](const sas::Effect & e) {
        return e.kind == sas::Effect::Kind::Assign && e.condition.empty() &&
               e.start_condition.empty();
      };
    std::map<sas::VarId, uint32_t> start;
    for (const auto & f : da.start) {
      start[f.var] = f.value;
    }
    for (const auto & e : da.effects) {
      if (e.timing == sas::Timing::Start && unconditional(e)) {
        start[e.fact.var] = e.fact.value;
      }
    }
    // End effects see the state after the start ones: what the start sets decides their
    // conditions (a delete "when it holds" of what the start added becomes unconditional)
    std::vector<sas::Effect> effects;
    for (const auto & e : da.effects) {
      if (e.timing != sas::Timing::End) {
        effects.push_back(e);
        continue;
      }
      if (auto cond = after_start(e.condition, start)) {
        sas::Effect simplified = e;
        simplified.condition = *cond;
        effects.push_back(std::move(simplified));
      }
    }
    // Then the end values: the end wins on a variable both set
    std::map<sas::VarId, uint32_t> final = start;
    for (const auto & f : da.end) {
      final[f.var] = f.value;
    }
    for (const auto & e : effects) {
      if (e.timing == sas::Timing::End && unconditional(e)) {
        final[e.fact.var] = e.fact.value;
      }
    }
    for (const auto & [v, x] : final) {
      a.changes.push_back(sas::Fact{v, x});
    }
    a.pre = da.at_start;
    auto over = after_start(da.over_all, start);
    auto end = after_start(da.at_end, start);
    // Numeric over all and at end conditions see the unconditional numeric start effects
    // (a battery drained at start must still be >= 0)
    std::map<sas::NumVarId, const sas::Effect *> numeric_starts;
    for (const auto & e : da.effects) {
      if (e.timing == sas::Timing::Start && e.kind == sas::Effect::Kind::Numeric &&
        e.condition.empty() && !e.continuous)
      {
        numeric_starts[e.num] = &e;
      }
    }
    if (over) {
      after_numeric(*over, numeric_starts);
    }
    if (end) {
      after_numeric(*end, numeric_starts);
    }
    bool possible = over && end && merge(a.pre, *over) && merge(a.pre, *end);
    for (auto e : effects) {
      if ((e.timing == sas::Timing::Start || e.timing == sas::Timing::End) && unconditional(e)) {
        continue;  // in the changes already
      }
      if (e.timing == sas::Timing::End && !e.start_condition.empty()) {
        possible = merge(e.condition, e.start_condition) && possible;
        e.start_condition = {};
      }
      e.timing = sas::Timing::Instant;
      substitute(e.condition, d.tree);
      if (e.kind == sas::Effect::Kind::Numeric) {
        substitute(e.value, d.tree);
      }
      a.effects.push_back(std::move(e));
    }
    substitute(a.pre, d.tree);
    sort_condition(a.pre);
    if (!possible) {
      // Never applicable (its own start breaks its conditions): 0 >= 1
      sas::NumericCondition never;
      never.op = sas::CompareOp::Ge;
      never.lhs.tree = constant(0.0);
      never.lhs.linear = sas::LinearExpr{};
      never.rhs.tree = constant(1.0);
      never.rhs.linear = sas::LinearExpr{{}, 1.0};
      never.difference = sas::LinearExpr{{}, -1.0};
      a.pre = sas::Condition{};
      a.pre.numeric.push_back(never);
    }
    c.task.actions.push_back(std::move(a));
  }
  return c;
}

std::optional<Plan> schedule(
  const sas::Task & original, const Compressed & compressed, state::StateSpace & space,
  const std::vector<uint32_t> & actions, double epsilon)
{
  auto bases = derived_bases(original);
  Stn stn;
  struct Step
  {
    Stn::Node start, end;
    double duration;
  };
  std::vector<Step> steps;
  // Happenings (start and end of each step) in the order the compressed plan executes them;
  // each goes after the last happening that wrote what it reads or writes, and after those
  // that read what it writes since. Over all conditions are read at start and at end (with no
  // separation: they hold in the open interval), so no later write can break them
  Orderer<Stn::Node> order;
  std::vector<uint32_t> step_of;  // per STN node
  step_of.push_back(UINT32_MAX);
  auto happen = [&](Stn::Node node, uint32_t k, const Access & acc) {
      auto before = order.before(acc, epsilon);
      order.record(node, acc);
      for (auto [b, sep] : before) {
        // Within a step the duration already orders start and end
        if (step_of[b] != k && !stn.at_least(b, node, sep)) {
          return false;
        }
      }
      return true;
    };
  state::StateId s = space.registry().initial();
  for (uint32_t k = 0; k < actions.size(); ++k) {
    uint32_t a = actions[k];
    double d = 0.0;
    Access at_start, at_end;
    bool durative = a >= compressed.instant;
    if (durative) {
      size_t i = a - compressed.instant;
      const auto & da = original.durative_actions.at(i);
      d = state::evaluate(compressed.duration[i], space.numbers(s).data());
      if (std::isnan(d) || d < 0.0) {
        return std::nullopt;
      }
      at_start.read(da.at_start);
      at_start.invariant(da.over_all);
      for (const auto & dc : da.duration) {
        at_start.read(dc.value.tree);
      }
      at_end.invariant(da.over_all);
      at_end.read(da.at_end);
      for (const auto & f : da.start) {
        at_start.writes.push_back(f.var);
      }
      for (const auto & f : da.end) {
        at_end.writes.push_back(f.var);
      }
      for (const auto & e : da.effects) {
        (e.timing == sas::Timing::End ? at_end : at_start).effect(e);
        if (e.timing == sas::Timing::End) {
          at_start.read(e.start_condition);
        }
      }
    } else {
      const auto & ia = original.actions.at(a);
      at_start.read(ia.pre);
      for (const auto & f : ia.changes) {
        at_start.writes.push_back(f.var);
      }
      for (const auto & e : ia.effects) {
        at_start.effect(e);
      }
    }
    at_start.finish(bases);
    at_end.finish(bases);
    auto next = space.successor(s, a);
    if (!next) {
      return std::nullopt;
    }
    s = *next;
    Step st{stn.add_node(), 0, d};
    step_of.push_back(k);
    if (durative) {
      st.end = stn.add_node();
      step_of.push_back(k);
      if (!stn.exactly(st.start, st.end, d)) {
        return std::nullopt;
      }
    } else {
      st.end = st.start;  // an instantaneous action: one happening
    }
    if (!happen(st.start, k, at_start) || (durative && !happen(st.end, k, at_end))) {
      return std::nullopt;
    }
    steps.push_back(st);
  }
  Plan plan;
  plan.temporal = true;
  for (uint32_t k = 0; k < actions.size(); ++k) {
    uint32_t a = actions[k];
    PlanStep step;
    step.time = stn.earliest(steps[k].start);
    step.duration = steps[k].duration;
    if (a >= compressed.instant) {
      const auto & da = original.durative_actions[a - compressed.instant];
      step.name = original.durative_action_names.at(da.schema);
      for (auto o : da.args) {
        step.args.push_back(original.object_names.at(o.index()));
      }
    } else {
      const auto & ia = original.actions[a];
      step.name = original.action_names.at(ia.schema);
      for (auto o : ia.args) {
        step.args.push_back(original.object_names.at(o.index()));
      }
    }
    plan.steps.push_back(std::move(step));
  }
  std::stable_sort(
    plan.steps.begin(), plan.steps.end(),
    [](const PlanStep & x, const PlanStep & y) {return x.time < y.time;});
  return plan;
}

}  // namespace kairos::temporal
