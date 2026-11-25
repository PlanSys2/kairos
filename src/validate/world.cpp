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


#include "validate/world.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <functional>
#include <initializer_list>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "pddl/printer.hpp"

namespace kairos::val
{

// --- Evaluator -------------------------------------------------------------------------------

std::optional<sem::FunctionId> Evaluator::function(Symbol s) const
{
  if (function_cache_.size() <= s.index()) {
    function_cache_.resize(s.index() + 1, -2);
  }
  int64_t & slot = function_cache_[s.index()];
  if (slot == -2) {
    auto f = world_.dm.functions.find(symbols_.name(s));
    slot = f ? static_cast<int64_t>(f->value()) : -1;
  }
  if (slot < 0) {
    return std::nullopt;
  }
  return sem::FunctionId(static_cast<uint32_t>(slot));
}

std::optional<sem::PredicateId> Evaluator::predicate(Symbol s) const
{
  if (predicate_cache_.size() <= s.index()) {
    predicate_cache_.resize(s.index() + 1, -2);
  }
  int64_t & slot = predicate_cache_[s.index()];
  if (slot == -2) {
    auto p = world_.dm.predicates.find(symbols_.name(s));
    slot = p ? static_cast<int64_t>(p->value()) : -1;
  }
  if (slot < 0) {
    return std::nullopt;
  }
  return sem::PredicateId(static_cast<uint32_t>(slot));
}

std::optional<ObjectId> Evaluator::object(Symbol s) const
{
  if (object_cache_.size() <= s.index()) {
    object_cache_.resize(s.index() + 1, -2);
  }
  int64_t & slot = object_cache_[s.index()];
  if (slot == -2) {
    auto o = world_.pm.objects.find(symbols_.name(s));
    slot = o ? static_cast<int64_t>(o->value()) : -1;
  }
  if (slot < 0) {
    return std::nullopt;
  }
  return ObjectId(static_cast<uint32_t>(slot));
}

void Evaluator::for_each_binding(
  const std::vector<ast::TypedName> & params, Bindings & b, const std::function<bool()> & body)
{
  std::vector<sem::TypeSpec> specs;
  for (const auto & p : params) {
    specs.push_back(world_.resolve(p.types, symbols_));
  }
  std::function<bool(size_t)> rec = [&](size_t i) {
      if (i == params.size()) {
        return body();
      }
      for (uint32_t o : world_.objects_of(specs[i])) {
        b.emplace_back(params[i].name, o);
        bool go_on = rec(i + 1);
        b.pop_back();
        if (!go_on) {
          return false;
        }
      }
      return true;
    };
  rec(0);
}

bool Evaluator::mentions(FormulaId id, Symbol var) const
{
  auto in_term = [&](TermId t) {
      const ast::Term & term = nodes_.terms[t];
      if (term.kind == ast::TermKind::Variable) {
        return term.name == var;
      }
      return term.kind == ast::TermKind::Function;  // nested object fluents: assume it does
    };
  std::function<bool(ExprId)> in_expr = [&](ExprId e) {
      const ast::Expr & x = nodes_.exprs[e];
      if (x.kind == ast::ExprKind::Term && in_term(x.term)) {
        return true;
      }
      for (TermId a : x.args) {
        if (in_term(a)) {
          return true;
        }
      }
      for (ExprId ch : x.children) {
        if (in_expr(ch)) {
          return true;
        }
      }
      return false;
    };
  const ast::Formula & f = nodes_.formulas[id];
  for (TermId t : f.args) {
    if (in_term(t)) {
      return true;
    }
  }
  if (f.kind == ast::FormulaKind::Compare && (in_expr(f.lhs) || in_expr(f.rhs))) {
    return true;
  }
  for (const auto & p : f.params) {
    if (p.name == var) {
      return false;  // shadowed
    }
  }
  for (FormulaId ch : f.children) {
    if (mentions(ch, var)) {
      return true;
    }
  }
  return false;
}

void Evaluator::for_each_joined_binding(
  const Vars & outer, FormulaId body,
  const Ctx & c, Bindings & b, const std::function<bool()> & leaf, size_t existential_from)
{
  auto bound = [&](Symbol n) -> std::optional<uint32_t> {
      for (auto it = b.rbegin(); it != b.rend(); ++it) {
        if (it->first == n) {
          return it->second;
        }
      }
      return std::nullopt;
    };
  // Positive atoms and (= ?x object) of the top conjunction, also those inside its
  // existentials, whose variables then join too (the leaf still checks the whole body)
  Vars vars = outer;
  std::vector<const ast::Formula *> atoms;
  std::vector<std::pair<Symbol, uint32_t>> equals;
  auto mentioned = [&](Symbol n, size_t atoms_from, size_t equals_from) {
      for (size_t a = atoms_from; a < atoms.size(); ++a) {
        for (TermId t : atoms[a]->args) {
          if (nodes_.terms[t].kind == ast::TermKind::Variable && nodes_.terms[t].name == n) {
            return true;
          }
        }
      }
      for (size_t e = equals_from; e < equals.size(); ++e) {
        if (equals[e].first == n) {
          return true;
        }
      }
      return false;
    };
  std::function<void(const ast::Formula &)> collect = [&](const ast::Formula & f) {
      if (f.kind == ast::FormulaKind::Atom) {
        atoms.push_back(&f);
      } else if (f.kind == ast::FormulaKind::And) {
        for (FormulaId ch : f.children) {
          collect(nodes_.formulas[ch]);
        }
      } else if (f.kind == ast::FormulaKind::Compare && f.compare == ast::CompareOp::Eq) {
        const ast::Expr & l = nodes_.exprs[f.lhs];
        const ast::Expr & r = nodes_.exprs[f.rhs];
        if (l.kind != ast::ExprKind::Term || r.kind != ast::ExprKind::Term) {
          return;
        }
        const ast::Term & x = nodes_.terms[l.term];
        const ast::Term & y = nodes_.terms[r.term];
        for (const auto & [v, o] : {std::pair{&x, &y}, std::pair{&y, &x}}) {
          if (v->kind == ast::TermKind::Variable && o->kind == ast::TermKind::Name) {
            if (auto obj = object(o->name)) {
              equals.emplace_back(v->name, obj->value());
            }
          }
        }
      } else if (f.kind == ast::FormulaKind::Exists) {
        // Only if its variables shadow nothing, and each is in one of its atoms (one bound by
        // nothing would multiply the bindings instead)
        for (const auto & p : f.params) {
          bool clash = bound(p.name).has_value();
          for (const auto & v : vars) {
            clash = clash || v.first == p.name;
          }
          if (clash) {
            return;
          }
        }
        size_t vars_from = vars.size(), atoms_from = atoms.size(), equals_from = equals.size();
        for (const auto & p : f.params) {
          vars.emplace_back(p.name, world_.resolve(p.types, symbols_));
        }
        collect(nodes_.formulas[f.children.at(0)]);
        for (const auto & p : f.params) {
          if (!mentioned(p.name, atoms_from, equals_from)) {
            vars.resize(vars_from);
            atoms.resize(atoms_from);
            equals.resize(equals_from);
            return;
          }
        }
      }
    };
  collect(nodes_.formulas[body]);
  std::vector<bool> unused(vars.size(), false);
  for (size_t k = existential_from; k < outer.size(); ++k) {
    unused[k] = !mentions(body, vars[k].first);
  }
  std::vector<const std::vector<uint32_t> *> allowed(vars.size(), nullptr);
  std::vector<bool> assigned(vars.size(), false);
  auto local = [&](Symbol n) -> int {
      for (size_t k = vars.size(); k-- > 0; ) {
        if (vars[k].first == n) {
          return static_cast<int>(k);
        }
      }
      return -1;
    };
  auto allowed_of = [&](size_t k) -> const std::vector<uint32_t> & {
      if (!allowed[k]) {
        allowed[k] = &world_.objects_of(vars[k].second);
      }
      return *allowed[k];
    };
  // An atom argument: a known object, or the local variable it binds (kUnbound + index)
  constexpr int64_t kUnbound = int64_t{1} << 40;
  std::function<bool(size_t)> rec = [&](size_t i) {
      while (i < vars.size() && assigned[i]) {
        ++i;
      }
      if (i == vars.size()) {
        return leaf();
      }
      // A variable equal to an object takes it
      for (const auto & [n, o] : equals) {
        if (local(n) == static_cast<int>(i)) {
          const auto & al = allowed_of(i);
          if (!std::binary_search(al.begin(), al.end(), o)) {
            return true;
          }
          b.emplace_back(vars[i].first, o);
          assigned[i] = true;
          bool go_on = rec(i + 1);
          assigned[i] = false;
          b.pop_back();
          return go_on;
        }
      }
      // The atom mentioning vars[i] with the fewest other unbound variables
      const ast::Formula * best = nullptr;
      std::vector<int64_t> best_want;
      size_t best_free = vars.size() + 1;
      for (const ast::Formula * a : atoms) {
        auto pred = predicate(a->name);
        if (!pred) {
          continue;
        }
        std::vector<int64_t> want;
        bool mentions = false, usable = true;
        size_t free = 0;
        for (TermId t : a->args) {
          const ast::Term & term = nodes_.terms[t];
          Symbol n = term.name;
          if (term.kind == ast::TermKind::Variable) {
            int k = local(n);
            if (k >= 0 && !assigned[static_cast<size_t>(k)]) {
              mentions = mentions || static_cast<size_t>(k) == i;
              free += static_cast<size_t>(k) == i ? 0 : 1;
              want.push_back(kUnbound + k);
            } else if (auto v = bound(n)) {
              want.push_back(*v);
            } else {
              usable = false;
            }
          } else if (term.kind == ast::TermKind::Name) {
            auto o = object(term.name);
            if (!o) {
              usable = false;
            } else {
              want.push_back(o->value());
            }
          } else {
            usable = false;  // object fluent
          }
        }
        if (mentions && usable && free < best_free) {
          best = a;
          best_want = std::move(want);
          best_free = free;
          if (free == 0) {
            break;
          }
        }
      }
      if (!best) {
        for (uint32_t o : allowed_of(i)) {
          if (unused[i] && o != allowed_of(i).front()) {
            break;
          }
          b.emplace_back(vars[i].first, o);
          assigned[i] = true;
          bool go_on = rec(i + 1);
          assigned[i] = false;
          b.pop_back();
          if (!go_on) {
            return false;
          }
        }
        return true;
      }
      auto pred = predicate(best->name);
      bool derived = world_.dm.predicates.items[*pred].derived;
      if (derived) {
        world_.ensure_derived(c.now);
      }
      // Tuples of the unbound variables, collected before recursing, so no reference outlives
      // an insertion of derived atoms
      std::vector<size_t> slots;
      for (int64_t w : best_want) {
        if (w >= kUnbound) {
          size_t k = static_cast<size_t>(w - kUnbound);
          if (std::find(slots.begin(), slots.end(), k) == slots.end()) {
            slots.push_back(k);
          }
        }
      }
      const std::vector<Key> & keys = derived ? c.now.derived_index[pred->value()] :
        c.now.atoms_of(pred->value());
      std::vector<uint32_t> tuples;
      std::vector<int64_t> value(vars.size());
      for (const Key & key : keys) {
        for (size_t k : slots) {
          value[k] = -1;
        }
        bool ok = key.size() == best_want.size() + 1;
        for (size_t j = 0; j < best_want.size() && ok; ++j) {
          uint32_t x = key[j + 1];
          int64_t w = best_want[j];
          if (w < kUnbound) {
            ok = w == x;
          } else {
            int64_t & v = value[static_cast<size_t>(w - kUnbound)];
            ok = v < 0 || v == x;
            v = x;
          }
        }
        for (size_t k : slots) {
          if (!ok) {
            break;
          }
          const auto & al = allowed_of(k);
          ok = std::binary_search(al.begin(), al.end(), static_cast<uint32_t>(value[k]));
        }
        if (ok) {
          for (size_t k : slots) {
            tuples.push_back(static_cast<uint32_t>(value[k]));
          }
        }
      }
      size_t width = slots.size();
      std::vector<size_t> order(tuples.size() / width);
      for (size_t t = 0; t < order.size(); ++t) {
        order[t] = t;
      }
      auto row = [&](size_t t) {return tuples.data() + t * width;};
      std::sort(order.begin(), order.end(), [&](size_t x, size_t y) {
          return std::lexicographical_compare(row(x), row(x) + width, row(y), row(y) + width);
        });
      order.erase(std::unique(order.begin(), order.end(), [&](size_t x, size_t y) {
          return std::equal(row(x), row(x) + width, row(y));
        }), order.end());
      for (size_t t : order) {
        for (size_t s = 0; s < width; ++s) {
          b.emplace_back(vars[slots[s]].first, row(t)[s]);
          assigned[slots[s]] = true;
        }
        bool go_on = rec(i + 1);
        for (size_t s = 0; s < width; ++s) {
          assigned[slots[s]] = false;
          b.pop_back();
        }
        if (!go_on) {
          return false;
        }
      }
      return true;
    };
  rec(0);
}

uint32_t Evaluator::object_of(TermId id, const Ctx & c, Bindings & b)
{
  const ast::Term & t = nodes_.terms[id];
  switch (t.kind) {
    case ast::TermKind::Variable:
      for (auto it = b.rbegin(); it != b.rend(); ++it) {
        if (it->first == t.name) {
          return it->second;
        }
      }
      throw EvalError{"unbound variable " + name(t.name)};
    case ast::TermKind::Name: {
        auto o = object(t.name);
        if (!o) {
          throw EvalError{"unknown object " + name(t.name)};
        }
        return o->value();
      }
    case ast::TermKind::Function: {
        Key key = fluent_key(t.name, t.args, c, b);
        if (c.reads) {
          c.reads->fluents.insert(key);
        }
        auto it = c.now.objects.find(key);
        if (it == c.now.objects.end()) {
          throw EvalError{world_.key_text(key, false) + " is undefined"};
        }
        return it->second;
      }
  }
  throw EvalError{"bad term"};
}

Key Evaluator::atom_key(
  Symbol pred, const std::vector<TermId> & args, const Ctx & c, Bindings & b)
{
  auto p = predicate(pred);
  if (!p) {
    throw EvalError{"unknown predicate " + name(pred)};
  }
  Key key{p->value()};
  for (TermId a : args) {
    key.push_back(object_of(a, c, b));
  }
  return key;
}

Key Evaluator::fluent_key(
  Symbol fn, const std::vector<TermId> & args, const Ctx & c, Bindings & b)
{
  auto f = function(fn);
  if (!f) {
    throw EvalError{"unknown function " + name(fn)};
  }
  Key key{f->value()};
  for (TermId a : args) {
    key.push_back(object_of(a, c, b));
  }
  return key;
}

bool Evaluator::is_object_expr(ExprId id) const
{
  const ast::Expr & e = nodes_.exprs[id];
  if (e.kind == ast::ExprKind::Term) {
    return true;
  }
  if (e.kind == ast::ExprKind::Function) {
    auto f = function(e.name);
    return f && !world_.dm.functions.items[*f].numeric;
  }
  return false;
}

uint32_t Evaluator::object_value(ExprId id, const Ctx & c, Bindings & b)
{
  const ast::Expr & e = nodes_.exprs[id];
  if (e.kind == ast::ExprKind::Term) {
    return object_of(e.term, c, b);
  }
  Key key = fluent_key(e.name, e.args, c, b);
  if (c.reads) {
    c.reads->fluents.insert(key);
  }
  auto it = c.now.objects.find(key);
  if (it == c.now.objects.end()) {
    throw EvalError{world_.key_text(key, false) + " is undefined"};
  }
  return it->second;
}

double Evaluator::number(ExprId id, const Ctx & c, Bindings & b)
{
  const ast::Expr & e = nodes_.exprs[id];
  auto child = [&](size_t i) {return number(e.children.at(i), c, b);};
  switch (e.kind) {
    case ast::ExprKind::Number: return e.number;
    case ast::ExprKind::Term: throw EvalError{"object used as a number"};
    case ast::ExprKind::Function: {
        Key key = fluent_key(e.name, e.args, c, b);
        if (c.reads) {
          c.reads->fluents.insert(key);
        }
        auto it = c.now.numbers.find(key);
        if (it == c.now.numbers.end()) {
          throw EvalError{world_.key_text(key, false) + " is undefined"};
        }
        return it->second;
      }
    case ast::ExprKind::Add: {
        double s = 0;
        for (size_t i = 0; i < e.children.size(); ++i) {
          s += child(i);
        }
        return s;
      }
    case ast::ExprKind::Mul: {
        double p = 1;
        for (size_t i = 0; i < e.children.size(); ++i) {
          p *= child(i);
        }
        return p;
      }
    case ast::ExprKind::Sub: return child(0) - child(1);
    case ast::ExprKind::Neg: return -child(0);
    case ast::ExprKind::Div: {
        double d = child(1);
        if (d == 0.0) {
          throw EvalError{"division by zero in " + pddl::to_pddl(nodes_, symbols_, id)};
        }
        return child(0) / d;
      }
    case ast::ExprKind::Pow: return std::pow(child(0), child(1));
    case ast::ExprKind::Math: {
        double x = child(0);
        std::string f = name(e.name);
        if (f == "sin") {return std::sin(x);}
        if (f == "cos") {return std::cos(x);}
        if (f == "tan") {return std::tan(x);}
        if (f == "asin") {return std::asin(x);}
        if (f == "acos") {return std::acos(x);}
        if (f == "atan") {return std::atan(x);}
        return std::fabs(x);
      }
    case ast::ExprKind::Duration:
      if (!c.duration) {
        throw EvalError{"?duration used outside a durative action"};
      }
      return *c.duration;
    case ast::ExprKind::TotalTime:
      if (!c.total_time) {
        throw EvalError{"total-time used outside the metric"};
      }
      return *c.total_time;
    case ast::ExprKind::ContinuousT:
      if (!c.t) {
        throw EvalError{"#t used outside a continuous effect"};
      }
      return *c.t;
    case ast::ExprKind::IsViolated: {
        if (!c.prefs) {
          return 0.0;
        }
        auto it = c.prefs->find(name(e.name));
        return it == c.prefs->end() ? 0.0 : it->second;
      }
  }
  throw EvalError{"bad expression"};
}

bool Evaluator::compare(const ast::Formula & f, const Ctx & c, Bindings & b)
{
  if (is_object_expr(f.lhs) || is_object_expr(f.rhs)) {
    return object_value(f.lhs, c, b) == object_value(f.rhs, c, b);
  }
  double l = number(f.lhs, c, b);
  double r = number(f.rhs, c, b);
  // Processes and events: only rounding (two events on the two sides of a value would both hold
  // with the tolerance of actions, and undo each other)
  double tol = c.exact ? 1e-12 * std::max({1.0, std::fabs(l), std::fabs(r)}) :
    (c.duration_check ? world_.options.duration_tolerance : world_.options.tolerance);
  switch (f.compare) {
    case ast::CompareOp::Eq: return std::fabs(l - r) <= tol;
    case ast::CompareOp::Le: return l <= r + tol;
    case ast::CompareOp::Ge: return l >= r - tol;
    case ast::CompareOp::Lt: return l < r;
    case ast::CompareOp::Gt: return l > r;
  }
  return false;
}

bool Evaluator::holds(FormulaId id, const Ctx & c, Bindings & b)
{
  const ast::Formula & f = nodes_.formulas[id];
  switch (f.kind) {
    case ast::FormulaKind::And:
      for (FormulaId ch : f.children) {
        if (!holds(ch, c, b)) {
          return false;
        }
      }
      return true;
    case ast::FormulaKind::Or:
      for (FormulaId ch : f.children) {
        if (holds(ch, c, b)) {
          return true;
        }
      }
      return false;
    case ast::FormulaKind::Not: {
        const ast::Formula & ch = nodes_.formulas[f.children.at(0)];
        if (ch.kind == ast::FormulaKind::Compare && ch.compare == ast::CompareOp::Eq) {
          try {
            return !holds(f.children.at(0), c, b);
          } catch (const EvalError &) {
            return true;  // an undefined value equals nothing (as ENHSP)
          }
        }
        return !holds(f.children.at(0), c, b);
      }
    case ast::FormulaKind::Imply:
      return !holds(f.children.at(0), c, b) || holds(f.children.at(1), c, b);
    case ast::FormulaKind::Exists: {
        Vars vars;
        for (const auto & p : f.params) {
          vars.emplace_back(p.name, world_.resolve(p.types, symbols_));
        }
        bool result = false;
        for_each_joined_binding(vars, f.children.at(0), c, b, [&]() {
            result = holds(f.children.at(0), c, b);
            return !result;
          });
        return result;
      }
    case ast::FormulaKind::Forall: {
        bool exists = false;
        bool result = !exists;
        for_each_binding(f.params, b, [&]() {
            bool v = holds(f.children.at(0), c, b);
            if (exists && v) {
              result = true;
              return false;
            }
            if (!exists && !v) {
              result = false;
              return false;
            }
            return true;
          });
        return result;
      }
    case ast::FormulaKind::Atom: {
        Key key = atom_key(f.name, f.args, c, b);
        if (c.reads) {
          c.reads->atoms.insert(key);
        }
        if (world_.dm.predicates.items[sem::PredicateId(key[0])].derived) {
          world_.ensure_derived(c.now);
          return c.now.derived.count(key) != 0;
        }
        return c.now.atoms.count(key) != 0;
      }
    case ast::FormulaKind::Compare: return compare(f, c, b);
    case ast::FormulaKind::AtStart:
      if (c.phase == Phase::Start || c.phase == Phase::Instant) {
        return holds(f.children.at(0), c, b);
      }
      if (c.phase == Phase::End && c.start != nullptr) {
        Ctx s{*c.start, nullptr, Phase::Instant, c.duration};
        return holds(f.children.at(0), s, b);
      }
      return true;
    case ast::FormulaKind::AtEnd:
      return c.phase == Phase::End || c.phase == Phase::Instant ?
             holds(f.children.at(0), c, b) : true;
    case ast::FormulaKind::OverAll:
      return c.phase == Phase::Invariant ? holds(f.children.at(0), c, b) : true;
    case ast::FormulaKind::Preference: {
        bool v = holds(f.children.at(0), c, b);
        if (!v && c.prefs) {
          (*c.prefs)[f.name.valid() ? name(f.name) : std::string("anonymous")] += 1;
        }
        return true;
      }
    case ast::FormulaKind::Modal:
      throw EvalError{"modal operator outside :constraints"};
  }
  return false;
}

std::string Evaluator::compare_text(FormulaId id, const Ctx & c, Bindings & b)
{
  static const char * ops[] = {"=", "<", "<=", ">", ">="};
  const ast::Formula & f = nodes_.formulas[id];
  std::string text = pddl::to_pddl(nodes_, symbols_, id);
  try {
    if (is_object_expr(f.lhs) || is_object_expr(f.rhs)) {
      return text + ": " + world_.object_name(object_value(f.lhs, c, b)) + " is not " +
             world_.object_name(object_value(f.rhs, c, b));
    }
    char buffer[160];
    std::snprintf(buffer, sizeof(buffer), ": %g %s %g is false", number(f.lhs, c, b),
      ops[static_cast<int>(f.compare)], number(f.rhs, c, b));
    return text + buffer;
  } catch (const EvalError & e) {
    return text + ": " + e.message;
  }
}

std::string Evaluator::why(FormulaId id, const Ctx & c, Bindings & b)
{
  const ast::Formula & f = nodes_.formulas[id];
  switch (f.kind) {
    case ast::FormulaKind::And:
      for (FormulaId ch : f.children) {
        if (!holds(ch, c, b)) {
          return why(ch, c, b);
        }
      }
      break;
    case ast::FormulaKind::AtStart:
    case ast::FormulaKind::AtEnd:
    case ast::FormulaKind::OverAll:
      return why(f.children.at(0), c, b);
    case ast::FormulaKind::Atom: {
        Key key = atom_key(f.name, f.args, c, b);
        return world_.key_text(key, true) + " is false";
      }
    case ast::FormulaKind::Not:
      if (nodes_.formulas[f.children.at(0)].kind == ast::FormulaKind::Atom) {
        const ast::Formula & a = nodes_.formulas[f.children.at(0)];
        return world_.key_text(atom_key(a.name, a.args, c, b), true) + " is true";
      }
      break;
    case ast::FormulaKind::Compare:
      return compare_text(id, c, b);
    case ast::FormulaKind::Forall: {
        std::string reason;
        for_each_binding(f.params, b, [&]() {
            if (!holds(f.children.at(0), c, b)) {
              std::string where;
              for (size_t i = b.size() - f.params.size(); i < b.size(); ++i) {
                where += (where.empty() ? "" : ", ") + name(b[i].first) + " = " +
                world_.object_name(b[i].second);
              }
              reason = "for " + where + ": " + why(f.children.at(0), c, b);
              return false;
            }
            return true;
          });
        if (!reason.empty()) {
          return reason;
        }
        break;
      }
    default:
      break;
  }
  std::string text = pddl::to_pddl(nodes_, symbols_, id);
  if (!b.empty()) {
    text += " with";
    for (const auto & [n, o] : b) {
      text += " " + name(n) + " = " + world_.object_name(o);
    }
  }
  return text + " is false";
}

void Evaluator::effects(EffectId id, const Ctx & c, Bindings & b, bool active, Changes & out)
{
  const ast::Effect & e = nodes_.effects[id];
  switch (e.kind) {
    case ast::EffectKind::And:
      for (EffectId ch : e.children) {
        effects(ch, c, b, active, out);
      }
      return;
    case ast::EffectKind::Forall:
      for_each_binding(e.params, b, [&]() {
          effects(e.children.at(0), c, b, active, out);
          return true;
        });
      return;
    case ast::EffectKind::When: {
        // Only check the condition if the effect has something for this phase
        Changes tmp;
        effects(e.children.at(0), c, b, active, tmp);
        bool any = !tmp.adds.empty() || !tmp.dels.empty() || !tmp.nums.empty() ||
          !tmp.objs.empty();
        if (any && holds(e.condition, c, b)) {
          out.adds.insert(out.adds.end(), tmp.adds.begin(), tmp.adds.end());
          out.dels.insert(out.dels.end(), tmp.dels.begin(), tmp.dels.end());
          out.nums.insert(out.nums.end(), tmp.nums.begin(), tmp.nums.end());
          out.objs.insert(out.objs.end(), tmp.objs.begin(), tmp.objs.end());
        }
        return;
      }
    case ast::EffectKind::Add:
    case ast::EffectKind::Delete:
      if (active) {
        Key key = atom_key(e.name, e.args, c, b);
        (e.kind == ast::EffectKind::Add ? out.adds : out.dels).push_back(std::move(key));
      }
      return;
    case ast::EffectKind::Assign: {
        if (!active || has_continuous(id)) {
          return;
        }
        const ast::Expr & target = nodes_.exprs[e.target];
        Key key = fluent_key(target.name, target.args, c, b);
        auto f = function(target.name);
        if (f && !world_.dm.functions.items[*f].numeric) {
          const ast::Expr & v = nodes_.exprs[e.value];
          std::optional<uint32_t> value;
          bool undefined = v.kind == ast::ExprKind::Term &&
            nodes_.terms[v.term].kind == ast::TermKind::Name &&
            name(nodes_.terms[v.term].name) == "undefined";
          if (!undefined) {
            value = object_value(e.value, c, b);
          }
          out.objs.push_back(Changes::Obj{std::move(key), value});
          return;
        }
        out.nums.push_back(Changes::Num{std::move(key), e.op, number(e.value, c, b)});
        return;
      }
    case ast::EffectKind::AtStart:
      effects(e.children.at(0), c, b, c.phase == Phase::Start, out);
      return;
    case ast::EffectKind::AtEnd:
      effects(e.children.at(0), c, b, c.phase == Phase::End, out);
      return;
  }
}

bool Evaluator::has_continuous(EffectId id) const
{
  const ast::Effect & e = nodes_.effects[id];
  if (e.kind != ast::EffectKind::Assign) {
    return false;
  }
  std::function<bool(ExprId)> has_t = [&](ExprId x) {
      const ast::Expr & ex = nodes_.exprs[x];
      if (ex.kind == ast::ExprKind::ContinuousT) {
        return true;
      }
      return std::any_of(ex.children.begin(), ex.children.end(), has_t);
    };
  return has_t(e.value);
}

bool Evaluator::has_when(EffectId id) const
{
  const ast::Effect & e = nodes_.effects[id];
  if (e.kind == ast::EffectKind::When) {
    return true;
  }
  return std::any_of(e.children.begin(), e.children.end(),
           [&](EffectId ch) {return has_when(ch);});
}

void Evaluator::rates(
  EffectId id, const Ctx & c, Bindings & b, std::vector<std::pair<Key, double>> & out)
{
  const ast::Effect & e = nodes_.effects[id];
  switch (e.kind) {
    case ast::EffectKind::And:
      for (EffectId ch : e.children) {
        rates(ch, c, b, out);
      }
      return;
    case ast::EffectKind::Forall:
      for_each_binding(e.params, b, [&]() {
          rates(e.children.at(0), c, b, out);
          return true;
        });
      return;
    case ast::EffectKind::When:
      if (holds(e.condition, c, b)) {
        rates(e.children.at(0), c, b, out);
      }
      return;
    case ast::EffectKind::Assign: {
        if (!has_continuous(id)) {
          return;
        }
        const ast::Expr & target = nodes_.exprs[e.target];
        Key key = fluent_key(target.name, target.args, c, b);
        Ctx one = c;
        one.t = 1.0;
        Ctx zero = c;
        zero.t = 0.0;
        double rate = number(e.value, one, b) - number(e.value, zero, b);
        out.emplace_back(std::move(key), e.op == ast::AssignOp::Decrease ? -rate : rate);
        return;
      }
    default:
      return;
  }
}

// --- Initial state ---------------------------------------------------------------------------

void World::initial_state(State & state)
{
  const auto & p = problem;
  auto & ev = problem_eval;
  Bindings none;
  Evaluator::Ctx c{state};
  for (const auto & item : p.init) {
    if (item.time) {
      continue;
    }
    if (item.kind == ast::InitItem::Kind::Fact) {
      if (item.negated) {
        continue;
      }
      auto pred = dm.predicates.find(p.symbols.name(item.name));
      if (!pred) {
        continue;
      }
      Key key{pred->value()};
      for (TermId a : item.args) {
        key.push_back(ev.object_of(a, c, none));
      }
      state.atoms.insert(std::move(key));
      continue;
    }
    auto fn = dm.functions.find(p.symbols.name(item.name));
    if (!fn) {
      continue;  // ignored, as the semantic analysis warned
    }
    Key key{fn->value()};
    for (TermId a : item.args) {
      key.push_back(ev.object_of(a, c, none));
    }
    const ast::Expr & v = p.nodes.exprs[item.value];
    bool undefined = v.kind == ast::ExprKind::Term &&
      p.symbols.name(p.nodes.terms[v.term].name) == "undefined";
    if (dm.functions.items[*fn].numeric) {
      state.numbers[key] = v.number;
    } else if (!undefined) {
      state.objects[key] = ev.object_of(v.term, c, none);
    }
  }
  // Action costs: (total-cost) starts at 0 when the problem does not say (as FD and the IPC do)
  if (auto tc = dm.functions.find("total-cost");
    tc && dm.functions.items[*tc].params.empty() && dm.functions.items[*tc].numeric)
  {
    state.numbers.try_emplace(Key{tc->value()}, 0.0);
  }
}

}  // namespace kairos::val
