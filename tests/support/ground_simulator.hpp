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
/// \brief Straightforward executor of a ground task, for tests (not the planner's).

#ifndef KAIROS__TESTS__SUPPORT__GROUND_SIMULATOR_HPP_
#define KAIROS__TESTS__SUPPORT__GROUND_SIMULATOR_HPP_

#include <algorithm>
#include <cmath>
#include <optional>
#include <set>
#include <tuple>
#include <utility>
#include <vector>

#include "ground/task.hpp"

namespace kairos::ground::testing
{

struct GState
{
  std::set<AtomId> atoms;  // basic atoms
  std::vector<std::optional<double>> numbers;
  std::vector<std::optional<ObjectId>> objects;

  bool operator==(const GState &) const = default;
};

class GroundSimulator
{
public:
  explicit GroundSimulator(const Task & t)
  : t_(t)
  {
    // Strata by predicate: a derived atom read negatively must be complete before
    std::vector<int> level(t.predicate_names.size(), 0);
    for (bool changed = true; changed; ) {
      changed = false;
      for (const auto & ax : t.axioms) {
        int & head = level[t.atoms[ax.head].predicate.index()];
        auto need = [&](AtomId a, int extra) {
            if (!t.is_derived(a)) {
              return;
            }
            int n = level[t.atoms[a].predicate.index()] + extra;
            if (n > head && n <= static_cast<int>(level.size())) {
              head = n;
              changed = true;
            }
          };
        for (AtomId a : ax.body.positive) {
          need(a, 0);
        }
        for (AtomId a : ax.body.negative) {
          need(a, 1);
        }
      }
    }
    for (size_t i = 0; i < t.axioms.size(); ++i) {
      auto l = static_cast<size_t>(level[t.atoms[t.axioms[i].head].predicate.index()]);
      if (strata_.size() <= l) {
        strata_.resize(l + 1);
      }
      strata_[l].push_back(i);
    }
  }

  GState initial() const
  {
    GState s;
    s.atoms.insert(t_.init_atoms.begin(), t_.init_atoms.end());
    s.numbers = t_.init_numeric;
    s.objects = t_.init_objects;
    return s;
  }

  std::set<AtomId> derived(const GState & s) const
  {
    std::set<AtomId> der;
    for (const auto & stratum : strata_) {
      for (bool changed = true; changed; ) {
        changed = false;
        for (size_t i : stratum) {
          const auto & ax = t_.axioms[i];
          if (!der.count(ax.head) && holds(ax.body, s, der)) {
            der.insert(ax.head);
            changed = true;
          }
        }
      }
    }
    return der;
  }

  double number(const Expr & e, const GState & s, double duration = 0.0) const
  {
    auto c = [&](size_t i) {return number(e.children.at(i), s, duration);};
    switch (e.op) {
      case ExprOp::Constant: return e.value;
      case ExprOp::Fluent: {
          const auto & v = s.numbers.at(e.fluent);
          return v ? *v : std::nan("");
        }
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

  std::optional<ObjectId> object(const ObjectTerm & o, const GState & s) const
  {
    if (!o.is_fluent) {
      return ObjectId(o.index);
    }
    return s.objects.at(o.index);
  }

  bool holds(
    const Condition & c, const GState & s, const std::set<AtomId> & der,
    double duration = 0.0) const
  {
    auto truth = [&](AtomId a) {return t_.is_derived(a) ? der.count(a) > 0 : s.atoms.count(a) > 0;};
    for (AtomId a : c.positive) {
      if (!truth(a)) {
        return false;
      }
    }
    for (AtomId a : c.negative) {
      if (truth(a)) {
        return false;
      }
    }
    for (const auto & n : c.numeric) {
      double l = number(n.lhs, s, duration);
      double r = number(n.rhs, s, duration);
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
    for (const auto & o : c.objects) {
      auto x = object(o.lhs, s);
      auto y = object(o.rhs, s);
      if (!x || !y || ((*x == *y) != o.equal)) {
        return false;
      }
    }
    return true;
  }

  /// @brief Unconditional changes plus the effects with this timing; conditions read `s` (and
  /// `start` for start conditions).
  GState apply(
    const AtomChanges & changes, const std::vector<Effect> & effects, Timing timing,
    const GState & s, const GState * start = nullptr, double duration = 0.0) const
  {
    std::set<AtomId> der = derived(s);
    std::set<AtomId> start_der = start ? derived(*start) : std::set<AtomId>{};
    std::vector<AtomId> adds = changes.add, dels = changes.del;
    std::vector<std::tuple<FluentId, ast::AssignOp, double>> nums;
    std::vector<std::pair<FluentId, std::optional<ObjectId>>> objs;
    for (const auto & e : effects) {
      if (e.timing != timing || e.continuous) {
        continue;
      }
      if (!holds(e.condition, s, der, duration)) {
        continue;
      }
      if (!e.start_condition.empty() &&
        !(start && holds(e.start_condition, *start, start_der, duration)))
      {
        continue;
      }
      switch (e.kind) {
        case Effect::Kind::Add: adds.push_back(e.atom); break;
        case Effect::Kind::Delete: dels.push_back(e.atom); break;
        case Effect::Kind::Numeric:
          nums.emplace_back(e.fluent, e.op, number(e.value, s, duration));
          break;
        case Effect::Kind::ObjectAssign:
          objs.emplace_back(e.fluent, e.object ? object(*e.object, s) : std::nullopt);
          break;
      }
    }
    GState n = s;
    for (AtomId a : dels) {
      n.atoms.erase(a);
    }
    for (AtomId a : adds) {
      n.atoms.insert(a);
    }
    for (const auto & [f, op, v] : nums) {
      double old = s.numbers[f].value_or(0.0);
      double r = 0;
      switch (op) {
        case ast::AssignOp::Assign: r = v; break;
        case ast::AssignOp::Increase: r = old + v; break;
        case ast::AssignOp::Decrease: r = old - v; break;
        case ast::AssignOp::ScaleUp: r = old * v; break;
        case ast::AssignOp::ScaleDown: r = old / v; break;
      }
      n.numbers[f] = r;
    }
    for (const auto & [f, v] : objs) {
      n.objects[f] = v;
    }
    return n;
  }

  bool applicable(const Action & a, const GState & s) const
  {
    return holds(a.precondition, s, derived(s));
  }

  bool goal(const GState & s) const
  {
    return !t_.goal_unreachable && holds(t_.goal, s, derived(s));
  }

  /// @brief Runs a durative action alone: start, over all, end. nullopt if it cannot.
  std::optional<GState> run(const DurativeAction & a, const GState & s, double d) const
  {
    std::set<AtomId> der = derived(s);
    for (const auto & c : a.duration) {
      double v = number(c.value, s, d);
      bool ok = c.op == CompareOp::Eq ? std::abs(d - v) < 1e-3 :
        (c.op == CompareOp::Le ? d <= v + 1e-3 : d >= v - 1e-3);
      if (!ok) {
        return std::nullopt;
      }
    }
    if (!holds(a.at_start, s, der, d)) {
      return std::nullopt;
    }
    GState mid = apply(a.start, a.effects, Timing::Start, s, nullptr, d);
    std::set<AtomId> mid_der = derived(mid);
    if (!holds(a.over_all, mid, mid_der, d) || !holds(a.at_end, mid, mid_der, d)) {
      return std::nullopt;
    }
    return apply(a.end, a.effects, Timing::End, mid, &s, d);
  }

private:
  const Task & t_;
  std::vector<std::vector<size_t>> strata_;
};

}  // namespace kairos::ground::testing

#endif  // KAIROS__TESTS__SUPPORT__GROUND_SIMULATOR_HPP_
