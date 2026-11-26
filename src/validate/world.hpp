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


#ifndef KAIROS__VALIDATE__WORLD_HPP_
#define KAIROS__VALIDATE__WORLD_HPP_

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

#include "kairos/validate.hpp"
#include "pddl/ast.hpp"
#include "semantic/model.hpp"

// The states of a problem and the evaluation of formulas, expressions and effects of the syntax
// tree on them: what the validator is built on, and what anything else that follows a plan
// state by state can use (it does not depend on the planner's task).

namespace kairos::val
{

using ast::EffectId;
using ast::ExprId;
using ast::FormulaId;
using ast::TermId;
using sem::ObjectId;

// A ground atom or fluent: head id followed by the object ids of its arguments; inline up to
// kInline values, so most keys never allocate
class Key
{
public:
  static constexpr size_t kInline = 7;

  Key() = default;
  Key(std::initializer_list<uint32_t> values)
  {
    for (uint32_t v : values) {
      push_back(v);
    }
  }

  void push_back(uint32_t v)
  {
    if (size_ < kInline) {
      inline_[size_] = v;
    } else {
      if (size_ == kInline) {
        heap_.assign(inline_.begin(), inline_.end());
      }
      heap_.push_back(v);
    }
    ++size_;
  }

  size_t size() const {return size_;}
  const uint32_t * begin() const {return size_ <= kInline ? inline_.data() : heap_.data();}
  const uint32_t * end() const {return begin() + size_;}
  uint32_t operator[](size_t i) const {return begin()[i];}

  bool operator==(const Key & o) const
  {
    if (size_ != o.size_) {
      return false;
    }
    const uint32_t * a = begin();
    const uint32_t * b = o.begin();
    for (uint32_t i = 0; i < size_; ++i) {
      if (a[i] != b[i]) {
        return false;
      }
    }
    return true;
  }

private:
  uint32_t size_ = 0;
  std::array<uint32_t, kInline> inline_{};
  std::vector<uint32_t> heap_;
};

struct KeyHash
{
  size_t operator()(const Key & k) const noexcept
  {
    uint64_t h = 1469598103934665603ull;
    for (uint32_t v : k) {
      h ^= v;
      h *= 1099511628211ull;
    }
    return static_cast<size_t>(h);
  }
};

// Set of keys with open addressing (linear probing, tombstones for erase): no node per key
class KeySet
{
public:
  bool insert(Key key)
  {
    if ((used_ + 1) * 4 > slots_.size() * 3) {
      grow();
    }
    size_t h = KeyHash{}(key);
    size_t mask = slots_.size() - 1;
    size_t tomb = kNone;
    for (size_t i = h & mask; ; i = (i + 1) & mask) {
      Slot & slot = slots_[i];
      if (slot.state == kEmpty) {
        Slot & target = tomb != kNone ? slots_[tomb] : slot;
        used_ += tomb != kNone ? 0 : 1;
        target = Slot{std::move(key), h, kFull};
        ++size_;
        return true;
      }
      if (slot.state == kTomb) {
        tomb = tomb == kNone ? i : tomb;
      } else if (slot.hash == h && slot.key == key) {
        return false;
      }
    }
  }

  size_t count(const Key & key) const {return find(key) != kNone ? 1 : 0;}

  void erase(const Key & key)
  {
    size_t i = find(key);
    if (i != kNone) {
      slots_[i].state = kTomb;
      --size_;
    }
  }

  void clear()
  {
    slots_.clear();
    size_ = used_ = 0;
  }

  size_t size() const {return size_;}

  class Iterator
  {
public:
    Iterator(const KeySet * set, size_t i)
    : set_(set), i_(i) {skip();}
    const Key & operator*() const {return set_->slots_[i_].key;}
    Iterator & operator++()
    {
      ++i_;
      skip();
      return *this;
    }
    bool operator!=(const Iterator & o) const {return i_ != o.i_;}

private:
    void skip()
    {
      while (i_ < set_->slots_.size() && set_->slots_[i_].state != kFull) {
        ++i_;
      }
    }
    const KeySet * set_;
    size_t i_;
  };
  Iterator begin() const {return Iterator(this, 0);}
  Iterator end() const {return Iterator(this, slots_.size());}

private:
  static constexpr uint8_t kEmpty = 0, kFull = 1, kTomb = 2;
  static constexpr size_t kNone = ~size_t{0};
  struct Slot
  {
    Key key;
    size_t hash = 0;
    uint8_t state = kEmpty;
  };

  size_t find(const Key & key) const
  {
    if (slots_.empty()) {
      return kNone;
    }
    size_t h = KeyHash{}(key);
    size_t mask = slots_.size() - 1;
    for (size_t i = h & mask; ; i = (i + 1) & mask) {
      const Slot & slot = slots_[i];
      if (slot.state == kEmpty) {
        return kNone;
      }
      if (slot.state == kFull && slot.hash == h && slot.key == key) {
        return i;
      }
    }
  }

  void grow()
  {
    std::vector<Slot> old = std::move(slots_);
    size_t n = 16;
    while (n * 3 < (size_ + 1) * 8) {
      n *= 2;  // at most 3/8 full after growing
    }
    slots_.assign(n, Slot{});
    size_ = used_ = 0;
    for (Slot & slot : old) {
      if (slot.state == kFull) {
        insert(std::move(slot.key));
      }
    }
  }

  std::vector<Slot> slots_;
  size_t size_ = 0;
  size_t used_ = 0;  // full and tombstones
};

struct State
{
  KeySet atoms;
  std::unordered_map<Key, double, KeyHash> numbers;
  std::unordered_map<Key, uint32_t, KeyHash> objects;  // object fluents
  double time = 0.0;

  // Derived atoms, computed on demand and dropped on a change of what the axioms read
  mutable KeySet derived;
  mutable bool derived_ready = false;
  // Atoms by predicate, to bind variables from true atoms instead of every object
  mutable std::unordered_map<uint32_t, std::vector<Key>> atom_index;
  mutable bool atom_index_ready = false;
  mutable std::unordered_map<uint32_t, std::vector<Key>> derived_index;

  void changed(bool derived_too = true) const
  {
    if (derived_too) {
      derived.clear();
      derived_ready = false;
      derived_index.clear();
    }
    atom_index.clear();
    atom_index_ready = false;
  }

  const std::vector<Key> & atoms_of(uint32_t pred) const
  {
    if (!atom_index_ready) {
      for (const auto & k : atoms) {
        atom_index[k[0]].push_back(k);
      }
      atom_index_ready = true;
    }
    return atom_index[pred];
  }

  void add_derived(Key key) const
  {
    derived_index[key[0]].push_back(key);
    derived.insert(std::move(key));
  }
};

// Something in the plan or the model the validator reports as an error
struct EvalError
{
  std::string message;
};

// Something the validator cannot check yet
struct Unsupported
{
  std::string message;
};

enum class Phase : uint8_t {Instant, Start, End, Invariant};

struct Tracking
{
  KeySet atoms;    // atoms read
  KeySet fluents;  // numeric and object fluents read
};

// Variable -> object; variables are symbols of the evaluator that binds and reads them
using Bindings = std::vector<std::pair<Symbol, uint32_t>>;
using Vars = std::vector<std::pair<Symbol, sem::TypeSpec>>;

struct Changes
{
  std::vector<Key> adds;
  std::vector<Key> dels;
  struct Num
  {
    Key key;
    ast::AssignOp op;
    double value;
  };
  std::vector<Num> nums;
  struct Obj
  {
    Key key;
    std::optional<uint32_t> value;
  };
  std::vector<Obj> objs;
};

class World;

// Evaluates the formulas, expressions and effects of one file (domain or problem)
class Evaluator
{
public:
  struct Ctx
  {
    explicit Ctx(
      const State & state, const State * start_state = nullptr, Phase p = Phase::Instant,
      std::optional<double> d = std::nullopt)
    : now(state), start(start_state), phase(p), duration(d) {}

    const State & now;
    const State * start = nullptr;  // state at the action start, for "at start" in when-conditions
    Phase phase = Phase::Instant;
    std::optional<double> duration;
    std::optional<double> total_time;
    std::optional<double> t;        // value of #t
    bool duration_check = false;    // comparing ?duration: use the duration tolerance
    bool exact = false;             // processes and events: a tolerance for rounding only
    Tracking * reads = nullptr;
    std::map<std::string, uint32_t> * prefs = nullptr;
  };

  Evaluator(World & world, const ast::Nodes & nodes, const SymbolTable & symbols)
  : world_(world), nodes_(nodes), symbols_(symbols) {}

  bool holds(FormulaId id, const Ctx & c, Bindings & b);
  double number(ExprId id, const Ctx & c, Bindings & b);
  uint32_t object_of(TermId id, const Ctx & c, Bindings & b);
  void effects(EffectId id, const Ctx & c, Bindings & b, bool active, Changes & out);
  void rates(
    EffectId id, const Ctx & c, Bindings & b, std::vector<std::pair<Key, double>> & out);
  std::string why(FormulaId id, const Ctx & c, Bindings & b);
  bool has_continuous(EffectId id) const;
  bool has_when(EffectId id) const;

  std::string name(Symbol s) const {return std::string(symbols_.name(s));}
  const ast::Nodes & nodes() const {return nodes_;}

  // Lookups by symbol, cached: the model tables search by name
  std::optional<sem::PredicateId> predicate(Symbol s) const;
  std::optional<sem::FunctionId> function(Symbol s) const;
  std::optional<ObjectId> object(Symbol s) const;

  // Iterates over all bindings of typed variables
  void for_each_binding(
    const std::vector<ast::TypedName> & params, Bindings & b,
    const std::function<bool()> & body);

  /**
   * @brief Like for_each_binding, but a variable that appears in a positive atom of `body` (or
   * of its top conjunction) takes only the values of the true atoms of that predicate that
   * agree with what is already bound: much fewer than every object of its type. A variable
   * in (= ?x object) takes that object. The existentials of the conjunction join too, so the
   * leaf may run more than once for the same values of `vars`. The variables from
   * `existential_from` on that `body` never mentions take only one object (any witness does).
   */
  void for_each_joined_binding(
    const Vars & vars, FormulaId body,
    const Ctx & c, Bindings & b, const std::function<bool()> & leaf,
    size_t existential_from = 0);

  /// Whether the variable appears in the formula
  bool mentions(FormulaId id, Symbol var) const;

private:
  mutable std::vector<int64_t> predicate_cache_;  // per symbol: -2 not looked up, -1 none
  mutable std::vector<int64_t> function_cache_;
  mutable std::vector<int64_t> object_cache_;

  Key atom_key(Symbol pred, const std::vector<TermId> & args, const Ctx & c, Bindings & b);
  Key fluent_key(Symbol fn, const std::vector<TermId> & args, const Ctx & c, Bindings & b);
  bool is_object_expr(ExprId id) const;
  uint32_t object_value(ExprId id, const Ctx & c, Bindings & b);
  bool compare(const ast::Formula & f, const Ctx & c, Bindings & b);
  std::string compare_text(FormulaId id, const Ctx & c, Bindings & b);

  World & world_;
  const ast::Nodes & nodes_;
  const SymbolTable & symbols_;
};

// Problem-wide data: objects by type, derived rules, names
class World
{
public:
  World(
    const ast::Domain & d, const sem::DomainModel & domain_model, const ast::Problem & p,
    const sem::ProblemModel & problem_model, const ValidateOptions & validate_options)
  : domain(d), dm(domain_model), problem(p), pm(problem_model), options(validate_options),
    domain_eval(*this, d.nodes, d.symbols), problem_eval(*this, p.nodes, p.symbols)
  {
    // Derived predicates, by strata
    std::map<std::string, int> level;
    for (const auto & dv : d.derived) {
      level[std::string(d.symbols.name(dv.head.name))] = 0;
    }
    std::map<std::string, std::vector<std::pair<std::string, bool>>> deps;
    for (const auto & dv : d.derived) {
      std::string head(d.symbols.name(dv.head.name));
      collect_deps(dv.body, true, deps[head], level);
    }
    for (bool changed = true; changed; ) {
      changed = false;
      for (auto & [head, ds] : deps) {
        for (const auto & [q, positive] : ds) {
          int need = level[q] + (positive ? 0 : 1);
          if (need > level[head] && need <= static_cast<int>(level.size())) {
            level[head] = need;
            changed = true;
          }
        }
      }
    }
    // Within a stratum, strongly connected components of the positive dependencies, so only
    // recursive rules need a fixpoint (Tarjan emits dependencies first)
    std::map<std::string, std::vector<size_t>> rules_of;
    for (size_t i = 0; i < d.derived.size(); ++i) {
      rules_of[std::string(d.symbols.name(d.derived[i].head.name))].push_back(i);
    }
    std::map<std::string, int> index, low;
    std::vector<std::string> stack;
    std::set<std::string> on_stack;
    int counter = 0;
    std::vector<std::pair<int, RuleGroup>> found;  // (level, group)
    std::function<void(const std::string &)> visit = [&](const std::string & v) {
        index[v] = low[v] = counter++;
        stack.push_back(v);
        on_stack.insert(v);
        bool self_loop = false;
        for (const auto & [q, positive] : deps[v]) {
          if (level[q] != level[v]) {
            continue;
          }
          self_loop = self_loop || q == v;
          if (!index.count(q)) {
            visit(q);
            low[v] = std::min(low[v], low[q]);
          } else if (on_stack.count(q)) {
            low[v] = std::min(low[v], index[q]);
          }
        }
        if (low[v] != index[v]) {
          return;
        }
        RuleGroup g;
        size_t members = 0;
        for (std::string w; w != v; ) {
          w = stack.back();
          stack.pop_back();
          on_stack.erase(w);
          ++members;
          const auto & rs = rules_of[w];
          g.rules.insert(g.rules.end(), rs.begin(), rs.end());
        }
        g.recursive = members > 1 || self_loop;
        found.emplace_back(level[v], std::move(g));
      };
    for (const auto & [head, rs] : rules_of) {
      if (!index.count(head)) {
        visit(head);
      }
    }
    std::stable_sort(
      found.begin(), found.end(), [](const auto & a, const auto & b) {return a.first < b.first;});
    for (auto & [l, g] : found) {
      groups.push_back(std::move(g));
    }
    for (const auto & dv : d.derived) {
      collect_reads(dv.body);
    }
  }

  /// Whether atoms of this predicate are read by an axiom (a change of them drops the derived)
  bool axioms_read(uint32_t predicate) const {return axiom_predicates.count(predicate) != 0;}

  const ast::Domain & domain;
  const sem::DomainModel & dm;
  const ast::Problem & problem;
  const sem::ProblemModel & pm;
  const ValidateOptions & options;
  Evaluator domain_eval;
  Evaluator problem_eval;

  /// @brief The initial state of the problem, without its timed literals: atoms, numbers
  /// (with (total-cost) at 0 if the problem does not say) and object fluents.
  void initial_state(State & state);
  struct RuleGroup
  {
    std::vector<size_t> rules;
    bool recursive = false;
  };
  std::vector<RuleGroup> groups;  // by stratum, dependencies first
  std::set<uint32_t> axiom_predicates;  // read by the axioms
  bool axioms_read_fluents = false;

  const std::vector<uint32_t> & objects_of(const sem::TypeSpec & spec)
  {
    auto it = by_type_.find(spec);
    if (it != by_type_.end()) {
      return it->second;
    }
    std::vector<uint32_t> out;
    for (size_t i = 0; i < pm.objects.items.size(); ++i) {
      if (dm.types.fits(pm.objects.items[ObjectId(static_cast<uint32_t>(i))].types, spec)) {
        out.push_back(static_cast<uint32_t>(i));
      }
    }
    return by_type_.emplace(spec, std::move(out)).first->second;
  }

  sem::TypeSpec resolve(const std::vector<Symbol> & types, const SymbolTable & symbols) const
  {
    sem::TypeSpec spec;
    for (Symbol s : types) {
      if (auto t = dm.types.find(symbols.name(s))) {
        spec.push_back(*t);
      }
    }
    if (types.empty()) {
      spec.push_back(dm.types.object());
    }
    return spec;
  }

  std::string object_name(uint32_t o) const {return pm.objects.items[ObjectId(o)].name;}

  std::string key_text(const Key & k, bool predicate) const
  {
    std::string out = "(" + (predicate ? dm.predicates.items[sem::PredicateId(k[0])].name :
      dm.functions.items[sem::FunctionId(k[0])].name);
    for (size_t i = 1; i < k.size(); ++i) {
      out += " " + object_name(k[i]);
    }
    return out + ")";
  }

  void ensure_derived(const State & s)
  {
    if (s.derived_ready) {
      return;
    }
    s.derived_ready = true;
    for (const auto & group : groups) {
      for (bool changed = true; changed; changed = changed && group.recursive) {
        changed = false;
        for (size_t r : group.rules) {
          const auto & rule = domain.derived[r];
          auto pred = dm.predicates.find(domain.symbols.name(rule.head.name));
          if (!pred) {
            continue;
          }
          const auto & param_types = dm.predicates.items[*pred].params;
          Vars vars;
          for (size_t i = 0; i < rule.head.params.size(); ++i) {
            vars.emplace_back(rule.head.params[i].name, param_types[i]);
          }
          // Existential variables join with the head ones
          size_t head_vars = vars.size();
          FormulaId body = rule.body;
          while (domain.nodes.formulas[body].kind == ast::FormulaKind::Exists) {
            const ast::Formula & e = domain.nodes.formulas[body];
            for (const auto & p : e.params) {
              vars.emplace_back(p.name, resolve(p.types, domain.symbols));
            }
            body = e.children.at(0);
          }
          Bindings b;
          Evaluator::Ctx c{s};
          // Head bindings from the true atoms of the body where possible
          domain_eval.for_each_joined_binding(vars, body, c, b, [&]() {
              Key key{pred->value()};
              for (size_t k = 0; k < head_vars; ++k) {
                for (auto it = b.rbegin(); it != b.rend(); ++it) {
                  if (it->first == vars[k].first) {
                    key.push_back(it->second);
                    break;
                  }
                }
              }
              if (!s.derived.count(key) && domain_eval.holds(body, c, b)) {
                s.add_derived(std::move(key));
                changed = true;
              }
              return true;
            }, head_vars);
        }
      }
    }
  }

private:
  void collect_reads(FormulaId id)
  {
    const ast::Formula & f = domain.nodes.formulas[id];
    if (f.kind == ast::FormulaKind::Atom) {
      if (auto pred = dm.predicates.find(domain.symbols.name(f.name))) {
        axiom_predicates.insert(pred->value());
      }
      for (TermId t : f.args) {
        axioms_read_fluents = axioms_read_fluents ||
          domain.nodes.terms[t].kind == ast::TermKind::Function;
      }
    } else if (f.kind == ast::FormulaKind::Compare) {
      axioms_read_fluents = axioms_read_fluents || reads_fluent(f.lhs) || reads_fluent(f.rhs);
    }
    for (FormulaId ch : f.children) {
      collect_reads(ch);
    }
  }

  bool reads_fluent(ExprId id) const
  {
    const ast::Expr & e = domain.nodes.exprs[id];
    if (e.kind == ast::ExprKind::Function ||
      (e.kind == ast::ExprKind::Term && domain.nodes.terms[e.term].kind == ast::TermKind::Function))
    {
      return true;
    }
    for (ExprId ch : e.children) {
      if (reads_fluent(ch)) {
        return true;
      }
    }
    return false;
  }

  void collect_deps(
    FormulaId id, bool positive, std::vector<std::pair<std::string, bool>> & out,
    const std::map<std::string, int> & derived)
  {
    const ast::Formula & f = domain.nodes.formulas[id];
    if (f.kind == ast::FormulaKind::Atom) {
      std::string n(domain.symbols.name(f.name));
      if (derived.count(n)) {
        out.emplace_back(n, positive);
      }
      return;
    }
    bool flip = f.kind == ast::FormulaKind::Not;
    for (size_t i = 0; i < f.children.size(); ++i) {
      bool p = flip ? !positive : positive;
      if (f.kind == ast::FormulaKind::Imply && i == 0) {
        p = !positive;
      }
      collect_deps(f.children[i], p, out, derived);
    }
  }

  std::map<sem::TypeSpec, std::vector<uint32_t>> by_type_;
};

}  // namespace kairos::val

#endif  // KAIROS__VALIDATE__WORLD_HPP_
