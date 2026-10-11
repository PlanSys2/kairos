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

/// \file
/// \brief Straightforward executor of a SAS+ task, for tests (not the planner's).

#ifndef KAIROS__TESTS__SUPPORT__SAS_SIMULATOR_HPP_
#define KAIROS__TESTS__SUPPORT__SAS_SIMULATOR_HPP_

#include <algorithm>
#include <cmath>
#include <optional>
#include <set>
#include <tuple>
#include <utility>
#include <vector>

#include "sas/task.hpp"
#include "support/ground_simulator.hpp"

namespace kairos::sas::testing
{

struct SState
{
  std::vector<uint32_t> values;
  std::vector<std::optional<double>> numbers;

  bool operator==(const SState &) const = default;
};

class SasSimulator
{
public:
  explicit SasSimulator(const Task & t)
  : t_(t) {}

  SState initial() const {return derive(SState{t_.initial, t_.initial_numeric});}

  // Derived variables from scratch, layer by layer
  SState derive(SState s) const
  {
    for (VarId v = 0; v < t_.variables.size(); ++v) {
      if (t_.variables[v].kind == Variable::Kind::Derived) {
        s.values[v] = t_.variables[v].default_value;
      }
    }
    size_t i = 0;
    for (int layer = 0; layer < t_.axiom_layers; ++layer) {
      size_t begin = i;
      while (i < t_.axioms.size() && t_.variables[t_.axioms[i].head.var].layer == layer) {
        ++i;
      }
      for (bool changed = true; changed; ) {
        changed = false;
        for (size_t k = begin; k < i; ++k) {
          const auto & ax = t_.axioms[k];
          if (s.values[ax.head.var] != ax.head.value && holds(ax.body, s)) {
            s.values[ax.head.var] = ax.head.value;
            changed = true;
          }
        }
      }
    }
    return s;
  }

  double number(const ground::Expr & e, const SState & s, double duration = 0.0) const
  {
    auto c = [&](size_t i) {return number(e.children.at(i), s, duration);};
    switch (e.op) {
      case ExprOp::Constant: return e.value;
      case ExprOp::Fluent: return s.numbers.at(e.fluent).value_or(std::nan(""));
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
      case ExprOp::Sub: return e.children.size() == 1 ? -c(0) : c(0) - c(1);
      case ExprOp::Neg: return -c(0);
      case ExprOp::Div: return c(0) / c(1);
      case ExprOp::Pow: return std::pow(c(0), c(1));
      default: return std::nan("");
    }
  }

  // The linear form must agree with the tree
  double value(const NumericExpr & e, const SState & s, double duration = 0.0) const
  {
    double tree = number(e.tree, s, duration);
    if (e.linear && !std::isnan(tree)) {
      double lin = e.linear->constant;
      for (const auto & [v, c] : e.linear->terms) {
        lin += c * s.numbers.at(v).value_or(std::nan(""));
      }
      if (std::abs(lin - tree) > 1e-6 * std::max(1.0, std::abs(tree))) {
        mismatch_ = true;
      }
    }
    return tree;
  }

  bool holds(const Condition & c, const SState & s, double duration = 0.0) const
  {
    for (const auto & f : c.facts) {
      if (s.values[f.var] != f.value) {
        return false;
      }
    }
    for (const auto & f : c.not_facts) {
      if (s.values[f.var] == f.value) {
        return false;
      }
    }
    for (const auto & n : c.numeric) {
      double l = value(n.lhs, s, duration), r = value(n.rhs, s, duration);
      bool v = false;
      switch (n.op) {
        case CompareOp::Eq: v = l == r; break;
        case CompareOp::Ne: v = l != r; break;
        case CompareOp::Lt: v = l < r; break;
        case CompareOp::Le: v = l <= r; break;
        case CompareOp::Gt: v = l > r; break;
        case CompareOp::Ge: v = l >= r; break;
      }
      if (!v) {
        return false;
      }
    }
    for (const auto & vc : c.vars) {
      auto obj = [&](VarId v) -> std::optional<ObjectId> {
          const auto & var = t_.variables[v];
          uint32_t x = s.values[v];
          return x < var.objects.size() ? std::optional<ObjectId>(var.objects[x]) : std::nullopt;
        };
      auto a = obj(vc.a), b = obj(vc.b);
      if (!a || !b || ((*a == *b) != vc.equal)) {
        return false;
      }
    }
    return true;
  }

  /// @brief Changes plus effects with this timing; deletes (last values) apply first.
  SState apply(
    const std::vector<Fact> & changes, const std::vector<Effect> & effects, Timing timing,
    const SState & s, const SState * start = nullptr, double duration = 0.0) const
  {
    std::vector<Fact> assigns = changes;
    std::vector<std::tuple<NumVarId, ast::AssignOp, double>> nums;
    std::vector<std::pair<VarId, VarId>> copies;
    for (const auto & e : effects) {
      if (e.timing != timing || e.continuous || !holds(e.condition, s, duration)) {
        continue;
      }
      if (!e.start_condition.empty() && !(start && holds(e.start_condition, *start, duration))) {
        continue;
      }
      switch (e.kind) {
        case Effect::Kind::Assign: assigns.push_back(e.fact); break;
        case Effect::Kind::Numeric: nums.emplace_back(e.num, e.op, value(e.value, s, duration));
          break;
        case Effect::Kind::Copy: copies.emplace_back(e.target, e.source); break;
      }
    }
    SState n = s;
    for (const auto & f : assigns) {
      if (f.value == t_.variables[f.var].none()) {
        n.values[f.var] = f.value;
      }
    }
    for (const auto & f : assigns) {
      if (f.value != t_.variables[f.var].none()) {
        n.values[f.var] = f.value;
      }
    }
    for (const auto & [target, source] : copies) {
      const auto & tv = t_.variables[target];
      const auto & sv = t_.variables[source];
      uint32_t x = s.values[source];
      if (x >= sv.objects.size()) {
        n.values[target] = tv.none();
      } else {
        auto it = std::find(tv.objects.begin(), tv.objects.end(), sv.objects[x]);
        n.values[target] = static_cast<uint32_t>(it - tv.objects.begin());
      }
    }
    for (const auto & [v, op, x] : nums) {
      double old = s.numbers[v].value_or(0.0);
      double r = 0;
      switch (op) {
        case ast::AssignOp::Assign: r = x; break;
        case ast::AssignOp::Increase: r = old + x; break;
        case ast::AssignOp::Decrease: r = old - x; break;
        case ast::AssignOp::ScaleUp: r = old * x; break;
        case ast::AssignOp::ScaleDown: r = old / x; break;
      }
      n.numbers[v] = r;
    }
    return derive(n);
  }

  std::optional<SState> run(const DurativeAction & a, const SState & s, double d) const
  {
    for (const auto & c : a.duration) {
      double v = value(c.value, s, d);
      bool ok = c.op == CompareOp::Eq ? std::abs(d - v) < 1e-3 :
        (c.op == CompareOp::Le ? d <= v + 1e-3 : d >= v - 1e-3);
      if (!ok) {
        return std::nullopt;
      }
    }
    if (!holds(a.at_start, s, d)) {
      return std::nullopt;
    }
    SState mid = apply(a.start, a.effects, Timing::Start, s, nullptr, d);
    if (!holds(a.over_all, mid, d) || !holds(a.at_end, mid, d)) {
      return std::nullopt;
    }
    return apply(a.end, a.effects, Timing::End, mid, &s, d);
  }

  bool goal(const SState & s) const {return !t_.goal_unreachable && holds(t_.goal, s);}

  /// @brief The same state as the ground simulator sees it (basic atoms, numbers, objects).
  ground::testing::GState to_ground(const SState & s) const
  {
    ground::testing::GState g;
    for (VarId v = 0; v < t_.variables.size(); ++v) {
      const auto & var = t_.variables[v];
      uint32_t x = s.values[v];
      switch (var.kind) {
        case Variable::Kind::Group:
          if (x < var.atoms.size()) {
            g.atoms.insert(var.atoms[x]);
          }
          break;
        case Variable::Kind::Binary:
          if (x == 0) {
            g.atoms.insert(var.atoms[0]);
          }
          break;
        case Variable::Kind::Derived:
          break;
        case Variable::Kind::ObjectFluent:
          if (g.objects.size() <= var.fluent) {
            g.objects.resize(var.fluent + 1);
          }
          g.objects[var.fluent] = x < var.objects.size() ?
            std::optional<ObjectId>(var.objects[x]) : std::nullopt;
          break;
      }
    }
    g.objects.resize(t_.object_fluents.size());
    g.numbers = s.numbers;
    return g;
  }

  bool linear_mismatch() const {return mismatch_;}

private:
  const Task & t_;
  mutable bool mismatch_ = false;
};

}  // namespace kairos::sas::testing

#endif  // KAIROS__TESTS__SUPPORT__SAS_SIMULATOR_HPP_
