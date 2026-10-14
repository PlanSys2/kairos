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

#include "state/eval.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include <utility>
#include <vector>

namespace kairos::state
{

using sas::ExprOp;

double evaluate(const ground::Expr & e, const double * numbers, double duration)
{
  auto c = [&](size_t i) {return evaluate(e.children[i], numbers, duration);};
  switch (e.op) {
    case ExprOp::Constant: return e.value;
    case ExprOp::Fluent: return numbers[e.fluent];
    case ExprOp::Duration: return duration;
    case ExprOp::Add: {
        double r = 0;
        for (size_t i = 0; i < e.children.size(); ++i) {
          r += c(i);
        }
        return r;
      }
    case ExprOp::Mul: {
        double r = 1;
        for (size_t i = 0; i < e.children.size(); ++i) {
          r *= c(i);
        }
        return r;
      }
    case ExprOp::Sub: {
        if (e.children.size() == 1) {
          return -c(0);
        }
        double r = c(0);
        for (size_t i = 1; i < e.children.size(); ++i) {
          r -= c(i);
        }
        return r;
      }
    case ExprOp::Neg: return -c(0);
    case ExprOp::Div: {
        double d = c(1);
        return d == 0.0 ? kUndefined : c(0) / d;
      }
    case ExprOp::Pow: return std::pow(c(0), c(1));
    case ExprOp::Math: {
        double x = c(0);
        if (e.name == "sin") {
          return std::sin(x);
        }
        if (e.name == "cos") {
          return std::cos(x);
        }
        if (e.name == "tan") {
          return std::tan(x);
        }
        if (e.name == "asin") {
          return std::asin(x);
        }
        if (e.name == "acos") {
          return std::acos(x);
        }
        if (e.name == "atan") {
          return std::atan(x);
        }
        return std::fabs(x);
      }
    default:
      return kUndefined;  // total-time, #t, is-violated: not part of states
  }
}

double evaluate(const sas::NumericExpr & e, const double * numbers, double duration)
{
  if (!e.linear) {
    return evaluate(e.tree, numbers, duration);
  }
  double r = e.linear->constant;
  for (const auto & [v, coef] : e.linear->terms) {
    r += coef * numbers[v];
  }
  return r;
}

bool compare(sas::CompareOp op, double a, double b)
{
  switch (op) {
    case sas::CompareOp::Eq: return a == b;
    case sas::CompareOp::Ne: return a != b;  // an undefined value equals nothing (as ENHSP)
    case sas::CompareOp::Lt: return a < b;
    case sas::CompareOp::Le: return a <= b;
    case sas::CompareOp::Gt: return a > b;
    case sas::CompareOp::Ge: return a >= b;
  }
  return false;
}

bool holds(
  const sas::Condition & c, const sas::Task & task, const uint32_t * values, const double * numbers,
  double duration)
{
  for (const auto & f : c.facts) {
    if (values[f.var] != f.value) {
      return false;
    }
  }
  for (const auto & f : c.not_facts) {
    if (values[f.var] == f.value) {
      return false;
    }
  }
  for (const auto & n : c.numeric) {
    if (n.difference) {
      double d = n.difference->constant;
      for (const auto & [v, coef] : n.difference->terms) {
        d += coef * numbers[v];
      }
      if (!compare(n.op, d, 0.0)) {
        return false;
      }
      continue;
    }
    double l = evaluate(n.lhs, numbers, duration);
    double r = evaluate(n.rhs, numbers, duration);
    if (!compare(n.op, l, r)) {
      return false;
    }
  }
  for (const auto & vc : c.vars) {
    const auto & a = task.variables[vc.a];
    const auto & b = task.variables[vc.b];
    uint32_t x = values[vc.a], y = values[vc.b];
    if (x >= a.objects.size() || y >= b.objects.size()) {
      return false;  // undefined compares false
    }
    if ((a.objects[x] == b.objects[y]) != vc.equal) {
      return false;
    }
  }
  return true;
}

AxiomEvaluator::AxiomEvaluator(const sas::Task & task)
: task_(task), watchers_(task.variables.size())
{
  layers_.resize(static_cast<size_t>(std::max(task.axiom_layers, 0)));
  for (sas::VarId v = 0; v < task.variables.size(); ++v) {
    const auto & var = task.variables[v];
    if (var.kind == sas::Variable::Kind::Derived && var.layer >= 0) {
      layers_[static_cast<size_t>(var.layer)].vars.push_back(v);
    }
  }
  for (const auto & ax : task.axioms) {
    int layer = task.variables[ax.head.var].layer;
    Rule r;
    r.head = ax.head.var;
    r.fixed = ax.body;
    r.fixed.facts.clear();
    for (const auto & f : ax.body.facts) {
      const auto & v = task.variables[f.var];
      if (v.kind == sas::Variable::Kind::Derived && v.layer == layer && f.value == 0) {
        r.waiting.push_back(f.var);
      } else {
        r.fixed.facts.push_back(f);
      }
    }
    auto id = static_cast<uint32_t>(rules_.size());
    for (sas::VarId w : r.waiting) {
      watchers_[w].push_back(id);
    }
    layers_[static_cast<size_t>(layer)].rules.push_back(id);
    rules_.push_back(std::move(r));
  }
  pending_.resize(rules_.size());
}

void AxiomEvaluator::evaluate(uint32_t * values, const double * numbers) const
{
  for (const auto & layer : layers_) {
    for (sas::VarId v : layer.vars) {
      values[v] = task_.variables[v].default_value;
    }
    queue_.clear();
    for (uint32_t r : layer.rules) {
      const Rule & rule = rules_[r];
      if (!holds(rule.fixed, task_, values, numbers)) {
        pending_[r] = UINT32_MAX;  // never fires in this state
        continue;
      }
      pending_[r] = static_cast<uint32_t>(rule.waiting.size());
      if (pending_[r] == 0 && values[rule.head] != 0) {
        values[rule.head] = 0;
        queue_.push_back(rule.head);
      }
    }
    while (!queue_.empty()) {
      sas::VarId v = queue_.back();
      queue_.pop_back();
      for (uint32_t r : watchers_[v]) {
        if (pending_[r] != UINT32_MAX && --pending_[r] == 0) {
          sas::VarId h = rules_[r].head;
          if (values[h] != 0) {
            values[h] = 0;
            queue_.push_back(h);
          }
        }
      }
    }
  }
}

}  // namespace kairos::state
