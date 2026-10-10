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
/// \brief Test-only executor of a lifted task on concrete states (non-temporal).
///
/// Deliberately simple and slow: it enumerates bindings. It exists to check the normalizer
/// against the validator, which reads the original domain.

#ifndef KAIROS__TESTS__SUPPORT__LIFTED_SIMULATOR_HPP_
#define KAIROS__TESTS__SUPPORT__LIFTED_SIMULATOR_HPP_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "normalize/task.hpp"

namespace kairos::lifted::testing
{

using Key = std::vector<uint32_t>;

struct SimState
{
  std::set<Key> atoms;                     // basic atoms
  std::map<Key, double> numbers;
  std::map<Key, uint32_t> objects;

  bool operator==(const SimState &) const = default;
};

class Simulator
{
public:
  explicit Simulator(const Task & t)
  : t_(t)
  {
    // Strata of derived predicates: a predicate read negatively or under forall must be
    // complete before
    std::vector<int> level(t.predicate_names.size(), 0);
    for (bool changed = true; changed; ) {
      changed = false;
      for (const auto & ax : t.axioms) {
        int & head = level[ax.head.index()];
        visit_literals(ax.body, false, [&](const Literal & l, bool under_forall) {
            if (!t.derived[l.atom.predicate.index()]) {
              return;
            }
            int need = level[l.atom.predicate.index()] + (!l.positive || under_forall ? 1 : 0);
            if (need > head && need <= static_cast<int>(level.size())) {
              head = need;
              changed = true;
            }
          });
      }
    }
    for (size_t i = 0; i < t.axioms.size(); ++i) {
      size_t l = static_cast<size_t>(level[t.axioms[i].head.index()]);
      if (strata_.size() <= l) {
        strata_.resize(l + 1);
      }
      strata_[l].push_back(i);
    }
  }

  SimState initial() const
  {
    SimState s;
    for (const auto & [p, args] : t_.init_facts) {
      Key k{p.value()};
      for (auto o : args) {
        k.push_back(o.value());
      }
      s.atoms.insert(k);
    }
    for (const auto & f : t_.init_fluents) {
      Key k{f.function.value()};
      for (auto o : f.args) {
        k.push_back(o.value());
      }
      if (f.numeric) {
        s.numbers[k] = f.number;
      } else if (f.object) {
        s.objects[k] = f.object->value();
      }
    }
    return s;
  }

  std::vector<uint32_t> objects_of(const TypeSpec & spec) const
  {
    std::vector<uint32_t> out;
    for (size_t o = 0; o < t_.object_names.size(); ++o) {
      if (t_.types.fits(t_.object_types[o], spec)) {
        out.push_back(static_cast<uint32_t>(o));
      }
    }
    return out;
  }

  /// @brief Applies the action called `name` with these parameters, if some copy applies.
  bool apply(
    const std::string & name, const std::vector<uint32_t> & params, const SimState & s,
    SimState * next) const
  {
    std::set<Key> der = derived(s);
    for (const auto & a : t_.actions) {
      if (a.name != name || a.parameters != params.size()) {
        continue;
      }
      // Parameters must have the declared types
      bool typed = true;
      for (size_t i = 0; i < params.size(); ++i) {
        typed = typed && t_.types.fits(t_.object_types[params[i]], a.variables[i].types);
      }
      if (!typed) {
        continue;
      }
      std::vector<uint32_t> b(a.variables.size(), 0);
      std::copy(params.begin(), params.end(), b.begin());
      bool found = false;
      enumerate(a.variables, a.parameters, a.parameters + a.hidden, b, [&]() {
          if (holds(a.precondition, a.variables, b, s, der)) {
            if (next) {
              *next = effects(a, b, s, der);
            }
            found = true;
            return false;
          }
          return true;
        });
      if (found) {
        return true;
      }
    }
    return false;
  }

  bool goal(const SimState & s) const
  {
    std::set<Key> der = derived(s);
    std::vector<uint32_t> b;
    return holds(t_.goal, {}, b, s, der);
  }

  std::set<Key> derived(const SimState & s) const
  {
    // The walk asks many times about the same state
    if (cached_state_ && *cached_state_ == s) {
      return cached_derived_;
    }
    std::set<Key> der = compute_derived(s);
    cached_state_ = s;
    cached_derived_ = der;
    return der;
  }

  std::set<Key> compute_derived(const SimState & s) const
  {
    std::set<Key> der;
    for (const auto & stratum : strata_) {
      for (bool changed = true; changed; ) {
        changed = false;
        for (size_t i : stratum) {
          const auto & ax = t_.axioms[i];
          std::vector<uint32_t> b(ax.variables.size(), 0);
          enumerate(ax.variables, 0, ax.head_arity, b, [&]() {
              Key k{ax.head.value()};
              for (uint32_t v = 0; v < ax.head_arity; ++v) {
                k.push_back(b[v]);
              }
              if (der.count(k)) {
                return true;
              }
              bool body = false;
              enumerate(ax.variables, ax.head_arity, ax.head_arity + ax.existential, b, [&]() {
                body = holds(ax.body, ax.variables, b, s, der);
                return !body;
                });
              if (body) {
                der.insert(k);
                changed = true;
              }
              return true;
            });
        }
      }
    }
    return der;
  }

private:
  // Calls f for every binding of variables [from, to); stops when f returns false
  bool enumerate(
    const std::vector<Variable> & vars, uint32_t from, uint32_t to, std::vector<uint32_t> & b,
    const std::function<bool()> & f) const
  {
    if (from >= to) {
      return f();
    }
    for (uint32_t o : objects_of(vars[from].types)) {
      b[from] = o;
      if (!enumerate(vars, from + 1, to, b, f)) {
        return false;
      }
    }
    return true;
  }

  static void visit_literals(
    const Condition & c, bool under_forall,
    const std::function<void(const Literal &, bool)> & f)
  {
    for (const auto & l : c.literals) {
      f(l, under_forall);
    }
    for (const auto & q : c.foralls) {
      visit_literals(q.body, true, f);
    }
  }

  uint32_t term(const Term & x, const std::vector<uint32_t> & b, const SimState & s) const
  {
    switch (x.kind) {
      case Term::Kind::Variable: return b.at(x.index);
      case Term::Kind::Object: return x.index;
      case Term::Kind::Function: {
          Key k{x.index};
          for (const auto & a : x.args) {
            k.push_back(term(a, b, s));
          }
          auto it = s.objects.find(k);
          return it == s.objects.end() ? UINT32_MAX : it->second;
        }
    }
    return 0;
  }

  double number(const Expr & e, const std::vector<uint32_t> & b, const SimState & s) const
  {
    auto c = [&](size_t i) {return number(e.children.at(i), b, s);};
    switch (e.op) {
      case ExprOp::Constant: return e.value;
      case ExprOp::Fluent: {
          Key k{e.function.value()};
          for (const auto & a : e.args) {
            k.push_back(term(a, b, s));
          }
          auto it = s.numbers.find(k);
          return it == s.numbers.end() ? std::nan("") : it->second;
        }
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
      case ExprOp::Sub: return c(0) - c(1);
      case ExprOp::Neg: return -c(0);
      case ExprOp::Div: return c(0) / c(1);
      case ExprOp::Pow: return std::pow(c(0), c(1));
      default: return std::nan("");
    }
  }

  bool holds(
    const Condition & cond, const std::vector<Variable> & vars, std::vector<uint32_t> & b,
    const SimState & s, const std::set<Key> & der) const
  {
    for (const auto & l : cond.literals) {
      Key k{l.atom.predicate.value()};
      for (const auto & a : l.atom.args) {
        k.push_back(term(a, b, s));
      }
      bool v = t_.derived[l.atom.predicate.index()] ? der.count(k) != 0 : s.atoms.count(k) != 0;
      if (v != l.positive) {
        return false;
      }
    }
    for (const auto & c : cond.comparisons) {
      bool v;
      if (c.objects) {
        bool eq = term(c.lhs_term, b, s) == term(c.rhs_term, b, s);
        v = c.op == CompareOp::Eq ? eq : !eq;
      } else {
        double l = number(c.lhs, b, s);
        double r = number(c.rhs, b, s);
        switch (c.op) {
          case CompareOp::Eq: v = l == r; break;
          case CompareOp::Ne: v = l != r; break;
          case CompareOp::Lt: v = l < r; break;
          case CompareOp::Le: v = l <= r; break;
          case CompareOp::Gt: v = l > r; break;
          case CompareOp::Ge: v = l >= r; break;
        }
      }
      if (!v) {
        return false;
      }
    }
    for (const auto & q : cond.foralls) {
      bool all = true;
      std::function<bool(size_t)> rec = [&](size_t i) {
          if (i == q.variables.size()) {
            all = holds(q.body, vars, b, s, der);
            return all;
          }
          if (b.size() <= q.variables[i]) {
            b.resize(q.variables[i] + 1);
          }
          for (uint32_t o : objects_of(vars[q.variables[i]].types)) {
            b[q.variables[i]] = o;
            if (!rec(i + 1)) {
              return false;
            }
          }
          return true;
        };
      rec(0);
      if (!all) {
        return false;
      }
    }
    return true;
  }

  SimState effects(
    const Action & a, std::vector<uint32_t> & b, const SimState & s,
    const std::set<Key> & der) const
  {
    std::vector<Key> adds;
    std::vector<Key> dels;
    std::vector<std::tuple<Key, ast::AssignOp, double>> nums;
    std::vector<std::pair<Key, std::optional<uint32_t>>> objs;
    for (const auto & e : a.effects) {
      std::function<void(size_t)> rec = [&](size_t i) {
          if (i < e.variables.size()) {
            for (uint32_t o : objects_of(a.variables[e.variables[i]].types)) {
              b[e.variables[i]] = o;
              rec(i + 1);
            }
            return;
          }
          if (!holds(e.condition, a.variables, b, s, der)) {
            return;
          }
          switch (e.kind) {
            case Effect::Kind::Add:
            case Effect::Kind::Delete: {
                Key k{e.atom.predicate.value()};
                for (const auto & x : e.atom.args) {
                  k.push_back(term(x, b, s));
                }
                (e.kind == Effect::Kind::Add ? adds : dels).push_back(k);
                break;
              }
            case Effect::Kind::Numeric: {
                Key k{e.function.value()};
                for (const auto & x : e.args) {
                  k.push_back(term(x, b, s));
                }
                nums.emplace_back(k, e.op, number(e.value, b, s));
                break;
              }
            case Effect::Kind::ObjectAssign: {
                Key k{e.function.value()};
                for (const auto & x : e.args) {
                  k.push_back(term(x, b, s));
                }
                objs.emplace_back(k, e.object ? std::optional<uint32_t>(term(*e.object, b, s)) :
                  std::nullopt);
                break;
              }
          }
        };
      rec(0);
    }
    SimState n = s;
    for (const auto & k : dels) {
      n.atoms.erase(k);
    }
    for (const auto & k : adds) {
      n.atoms.insert(k);
    }
    for (const auto & [k, op, v] : nums) {
      double & slot = n.numbers[k];
      double old = s.numbers.count(k) ? s.numbers.at(k) : 0.0;
      switch (op) {
        case ast::AssignOp::Assign: slot = v; break;
        case ast::AssignOp::Increase: slot += v; break;
        case ast::AssignOp::Decrease: slot -= v; break;
        case ast::AssignOp::ScaleUp: slot = old * v; break;
        case ast::AssignOp::ScaleDown: slot = old / v; break;
      }
    }
    for (const auto & [k, v] : objs) {
      if (v) {
        n.objects[k] = *v;
      } else {
        n.objects.erase(k);
      }
    }
    return n;
  }

  const Task & t_;
  std::vector<std::vector<size_t>> strata_;
  mutable std::optional<SimState> cached_state_;
  mutable std::set<Key> cached_derived_;
};

}  // namespace kairos::lifted::testing

#endif  // KAIROS__TESTS__SUPPORT__LIFTED_SIMULATOR_HPP_
