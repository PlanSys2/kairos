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

#include "validate/validator.hpp"

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

namespace
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

  // Derived atoms, computed on demand and dropped on any change
  mutable KeySet derived;
  mutable bool derived_ready = false;
  // Atoms by predicate, to bind variables from true atoms instead of every object
  mutable std::unordered_map<uint32_t, std::vector<Key>> atom_index;
  mutable bool atom_index_ready = false;
  mutable std::unordered_map<uint32_t, std::vector<Key>> derived_index;

  void changed() const
  {
    derived.clear();
    derived_ready = false;
    atom_index.clear();
    atom_index_ready = false;
    derived_index.clear();
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
  std::optional<ObjectId> object(Symbol s) const;

  // Iterates over all bindings of typed variables
  void for_each_binding(
    const std::vector<ast::TypedName> & params, Bindings & b,
    const std::function<bool()> & body);

  /**
   * @brief Like for_each_binding, but a variable that appears in a positive atom of `body` (or
   * of its top conjunction) takes only the values of the true atoms of that predicate that
   * agree with what is already bound: much fewer than every object of its type.
   */
  void for_each_joined_binding(
    const Vars & vars, FormulaId body,
    const Ctx & c, Bindings & b, const std::function<bool()> & leaf);

private:
  mutable std::vector<int64_t> predicate_cache_;  // per symbol: -2 not looked up, -1 none
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
  }

  const ast::Domain & domain;
  const sem::DomainModel & dm;
  const ast::Problem & problem;
  const sem::ProblemModel & pm;
  const ValidateOptions & options;
  Evaluator domain_eval;
  Evaluator problem_eval;
  struct RuleGroup
  {
    std::vector<size_t> rules;
    bool recursive = false;
  };
  std::vector<RuleGroup> groups;  // by stratum, dependencies first

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
            });
        }
      }
    }
  }

private:
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

// --- Evaluator -------------------------------------------------------------------------------

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

void Evaluator::for_each_joined_binding(
  const Vars & vars, FormulaId body,
  const Ctx & c, Bindings & b, const std::function<bool()> & leaf)
{
  // Positive atoms of the top conjunction
  std::vector<const ast::Formula *> atoms;
  const ast::Formula & top = nodes_.formulas[body];
  auto collect = [&](const ast::Formula & f) {
      if (f.kind == ast::FormulaKind::Atom) {
        atoms.push_back(&f);
      }
    };
  if (top.kind == ast::FormulaKind::And) {
    for (FormulaId ch : top.children) {
      collect(nodes_.formulas[ch]);
    }
  } else {
    collect(top);
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
  auto bound = [&](Symbol n) -> std::optional<uint32_t> {
      for (auto it = b.rbegin(); it != b.rend(); ++it) {
        if (it->first == n) {
          return it->second;
        }
      }
      return std::nullopt;
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
  auto f = world_.dm.functions.find(symbols_.name(fn));
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
    auto f = world_.dm.functions.find(symbols_.name(e.name));
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
  double tol = c.duration_check ? world_.options.duration_tolerance : world_.options.tolerance;
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
    case ast::FormulaKind::Not: return !holds(f.children.at(0), c, b);
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
        auto f = world_.dm.functions.find(symbols_.name(target.name));
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

// --- Validation ------------------------------------------------------------------------------

struct StepInfo
{
  const PlanStep * step = nullptr;
  size_t index = 0;            // position in the plan (1-based in messages)
  bool durative = false;
  size_t action = 0;           // index into actions or durative_actions
  Bindings bindings;
  double start = 0.0;
  double end = 0.0;
  std::unique_ptr<State> start_state;  // for when-conditions mixing at start and at end
  bool started = false;
  bool ended = false;
};

struct EventInfo
{
  enum class Kind {Instant, Start, End, Til} kind;
  double time;
  size_t index;  // step or init item
  Tracking reads;
  Changes changes;
  int order;     // tie-break for sorting
};

class Validator
{
public:
  Validator(World & world, const Plan & plan, ValidationReport & report)
  : w_(world), plan_(plan), report_(report) {}

  void run();

private:
  [[noreturn]] void fail(const std::string & code, const std::string & message, uint32_t line)
  {
    report_.diagnostics.push_back(
      Diagnostic::error(code, message,
      SourceLocation{w_.options.plan_filename, line, 0}));
    throw Stop{};
  }

  struct Stop
  {
  };

  std::string step_name(const StepInfo & s) const
  {
    char t[64];
    std::snprintf(t, sizeof(t), "%.3f", s.step->time);
    return "step " + std::to_string(s.index) + " " + s.step->action() + " at " + t;
  }

  void resolve_steps();
  void initial_state();
  void check_global_constraints(const State & s, const std::string & when);
  bool interfere(const EventInfo & a, const EventInfo & b) const;
  bool integrate(double from, double to);  // true if some fluent changed
  void check_invariants(const State & s, const std::string & when);
  void record_trajectory();
  void finish();
  bool constraint(FormulaId id, Evaluator & ev, Bindings & b);

  World & w_;
  const Plan & plan_;
  ValidationReport & report_;
  std::vector<StepInfo> steps_;
  std::vector<size_t> running_;  // durative steps started and not ended
  State state_;
  std::vector<std::pair<double, State>> trajectory_;
  bool record_ = false;
  double makespan_ = 0.0;
};

void Validator::resolve_steps()
{
  const auto & d = w_.domain;
  std::map<std::string, std::pair<bool, size_t>> actions;
  for (size_t i = 0; i < d.actions.size(); ++i) {
    actions[std::string(d.symbols.name(d.actions[i].name))] = {false, i};
  }
  for (size_t i = 0; i < d.durative_actions.size(); ++i) {
    actions[std::string(d.symbols.name(d.durative_actions[i].name))] = {true, i};
  }
  steps_.resize(plan_.steps.size());
  for (size_t i = 0; i < plan_.steps.size(); ++i) {
    const PlanStep & ps = plan_.steps[i];
    StepInfo & s = steps_[i];
    s.step = &ps;
    s.index = i + 1;
    auto a = actions.find(ps.name);
    if (a == actions.end()) {
      fail("validate.unknown_action", step_name(s) + ": the domain has no action '" + ps.name +
        "'", ps.line);
    }
    s.durative = a->second.first;
    s.action = a->second.second;
    const auto & params = s.durative ? d.durative_actions[s.action].params :
      d.actions[s.action].params;
    if (params.size() != ps.args.size()) {
      fail("validate.wrong_arity", step_name(s) + ": '" + ps.name + "' takes " +
        std::to_string(params.size()) + " argument(s), " + std::to_string(ps.args.size()) +
        " given", ps.line);
    }
    for (size_t k = 0; k < params.size(); ++k) {
      auto o = w_.pm.objects.find(ps.args[k]);
      if (!o) {
        fail("validate.unknown_object", step_name(s) + ": unknown object '" + ps.args[k] + "'",
          ps.line);
      }
      sem::TypeSpec required = w_.resolve(params[k].types, d.symbols);
      if (!w_.dm.types.fits(w_.pm.objects.items[*o].types, required)) {
        fail("validate.type_mismatch", step_name(s) + ": argument " + std::to_string(k + 1) +
          " must be " + w_.dm.types.to_string(required) + ", '" + ps.args[k] + "' is " +
          w_.dm.types.to_string(w_.pm.objects.items[*o].types), ps.line);
      }
      s.bindings.emplace_back(params[k].name, o->value());
    }
    s.start = ps.time;
    s.end = ps.time + (s.durative ? ps.duration : 0.0);
    if (!s.durative && ps.duration > w_.options.tolerance) {
      report_.diagnostics.push_back(
        Diagnostic::warning(
          "validate.duration_on_instant", step_name(s) + ": '" + ps.name +
          "' is not durative; its duration is ignored",
          SourceLocation{w_.options.plan_filename, ps.line, 0}));
    }
  }
}

void Validator::initial_state()
{
  const auto & p = w_.problem;
  auto & ev = w_.problem_eval;
  Bindings none;
  Evaluator::Ctx c{state_};
  for (const auto & item : p.init) {
    if (item.time) {
      continue;
    }
    if (item.kind == ast::InitItem::Kind::Fact) {
      if (item.negated) {
        continue;
      }
      auto pred = w_.dm.predicates.find(p.symbols.name(item.name));
      if (!pred) {
        continue;
      }
      Key key{pred->value()};
      for (TermId a : item.args) {
        key.push_back(ev.object_of(a, c, none));
      }
      state_.atoms.insert(std::move(key));
      continue;
    }
    auto fn = w_.dm.functions.find(p.symbols.name(item.name));
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
    if (w_.dm.functions.items[*fn].numeric) {
      state_.numbers[key] = v.number;
    } else if (!undefined) {
      state_.objects[key] = ev.object_of(v.term, c, none);
    }
  }
  // Action costs: (total-cost) starts at 0 when the problem does not say (as FD and the IPC do)
  if (auto tc = w_.dm.functions.find("total-cost");
    tc && w_.dm.functions.items[*tc].params.empty() && w_.dm.functions.items[*tc].numeric)
  {
    state_.numbers.try_emplace(Key{tc->value()}, 0.0);
  }
}

bool Validator::interfere(const EventInfo & a, const EventInfo & b) const
{
  auto writes_read = [](const EventInfo & x, const EventInfo & y) {
      for (const auto & k : x.changes.adds) {
        if (y.reads.atoms.count(k)) {
          return true;
        }
      }
      for (const auto & k : x.changes.dels) {
        if (y.reads.atoms.count(k)) {
          return true;
        }
      }
      for (const auto & n : x.changes.nums) {
        if (y.reads.fluents.count(n.key)) {
          return true;
        }
      }
      for (const auto & o : x.changes.objs) {
        if (y.reads.fluents.count(o.key)) {
          return true;
        }
      }
      return false;
    };
  if (writes_read(a, b) || writes_read(b, a)) {
    return true;
  }
  auto contains = [](const std::vector<Key> & v, const Key & k) {
      return std::find(v.begin(), v.end(), k) != v.end();
    };
  for (const auto & k : a.changes.adds) {
    if (contains(b.changes.dels, k)) {
      return true;
    }
  }
  for (const auto & k : b.changes.adds) {
    if (contains(a.changes.dels, k)) {
      return true;
    }
  }
  auto additive = [](ast::AssignOp op) {
      return op == ast::AssignOp::Increase || op == ast::AssignOp::Decrease;
    };
  for (const auto & x : a.changes.nums) {
    for (const auto & y : b.changes.nums) {
      if (x.key == y.key && !(additive(x.op) && additive(y.op))) {
        return true;
      }
    }
  }
  for (const auto & x : a.changes.objs) {
    for (const auto & y : b.changes.objs) {
      if (x.key == y.key) {
        return true;
      }
    }
  }
  return false;
}

bool Validator::integrate(double from, double to)
{
  if (to <= from || running_.empty()) {
    return false;
  }
  // Rates of the running actions, evaluated at the start of the interval
  std::unordered_map<Key, double, KeyHash> rates;
  std::vector<std::pair<Key, double>> contributions;
  for (size_t r : running_) {
    StepInfo & s = steps_[r];
    const auto & a = w_.domain.durative_actions[s.action];
    if (!a.effect.valid()) {
      continue;
    }
    Bindings b = s.bindings;
    Tracking reads;
    Evaluator::Ctx c{state_, nullptr, Phase::Invariant, s.step->duration};
    c.reads = &reads;
    w_.domain_eval.rates(a.effect, c, b, contributions);
    for (const auto & [k, rate] : contributions) {
      rates[k] += rate;
    }
    // A rate that depends on a continuously changing fluent is not linear
    for (const auto & k : reads.fluents) {
      if (rates.count(k)) {
        throw Unsupported{"continuous effects whose rate depends on another continuous change"
                " (non-linear dynamics)"};
      }
    }
    contributions.clear();
  }
  if (rates.empty()) {
    return false;
  }
  double dt = to - from;
  for (const auto & [k, rate] : rates) {
    auto it = state_.numbers.find(k);
    if (it == state_.numbers.end()) {
      fail("validate.undefined_fluent", w_.key_text(k, false) +
        " changes continuously but has no value", 0);
    }
    it->second += rate * dt;
  }
  state_.changed();
  return true;
}

void Validator::check_invariants(const State & s, const std::string & when)
{
  for (size_t r : running_) {
    StepInfo & st = steps_[r];
    const auto & a = w_.domain.durative_actions[st.action];
    if (!a.condition.valid()) {
      continue;
    }
    Bindings b = st.bindings;
    Evaluator::Ctx c{s, nullptr, Phase::Invariant, st.step->duration};
    try {
      if (!w_.domain_eval.holds(a.condition, c, b)) {
        fail("validate.invariant_failed", step_name(st) + ": over-all condition broken " +
          when + ": " + w_.domain_eval.why(a.condition, c, b), st.step->line);
      }
    } catch (const EvalError & e) {
      fail("validate.evaluation", step_name(st) + ": " + e.message, st.step->line);
    }
  }
}

void Validator::check_global_constraints(const State & s, const std::string & when)
{
  for (const auto & gc : w_.domain.global_constraints) {
    Bindings b;
    w_.domain_eval.for_each_binding(gc.params, b, [&]() {
        Evaluator::Ctx c{s};
        if (!w_.domain_eval.holds(gc.condition, c, b)) {
          fail("validate.global_constraint", "constraint '" +
            std::string(w_.domain.symbols.name(gc.name)) + "' broken " + when + ": " +
            w_.domain_eval.why(gc.condition, c, b), 0);
        }
        return true;
      });
  }
}

void Validator::record_trajectory()
{
  if (record_) {
    State copy;
    copy.atoms = state_.atoms;
    copy.numbers = state_.numbers;
    copy.objects = state_.objects;
    trajectory_.emplace_back(state_.time, std::move(copy));
  }
}

void Validator::run()
{
  if (!w_.domain.processes.empty() || !w_.domain.events.empty()) {
    throw Unsupported{"processes and events (PDDL+) cannot be validated yet"};
  }
  record_ = w_.domain.constraints.valid() || w_.problem.constraints.valid();
  resolve_steps();
  initial_state();
  record_trajectory();
  check_global_constraints(state_, "in the initial state");

  // Events
  std::vector<EventInfo> events;
  int order = 0;
  for (size_t i = 0; i < steps_.size(); ++i) {
    if (steps_[i].durative) {
      events.push_back({EventInfo::Kind::Start, steps_[i].start, i, {}, {}, order++});
      events.push_back({EventInfo::Kind::End, steps_[i].end, i, {}, {}, order++});
    } else {
      events.push_back({EventInfo::Kind::Instant, steps_[i].start, i, {}, {}, order++});
    }
  }
  for (size_t i = 0; i < w_.problem.init.size(); ++i) {
    if (w_.problem.init[i].time) {
      events.push_back({EventInfo::Kind::Til, *w_.problem.init[i].time, i, {}, {}, order++});
    }
  }
  std::stable_sort(events.begin(), events.end(),
    [](const EventInfo & a, const EventInfo & b) {return a.time < b.time;});

  const double tol = w_.options.tolerance;
  std::vector<EventInfo> previous;
  double previous_time = 0.0;
  bool have_previous = false;
  double clock = 0.0;
  size_t i = 0;
  while (i < events.size()) {
    // A happening: events at the same time
    size_t j = i;
    while (j < events.size() && events[j].time - events[i].time <= tol) {
      ++j;
    }
    double t = events[i].time;
    bool changed = integrate(clock, t);
    clock = t;
    state_.time = t;
    if (changed) {
      // Continuous change: the over-all conditions must still hold at the end of the interval
      check_invariants(state_, "before the happening at " + std::to_string(t));
    }

    // Conditions and effects of every event, on the state before the happening
    std::vector<EventInfo> group(events.begin() + static_cast<std::ptrdiff_t>(i),
      events.begin() + static_cast<std::ptrdiff_t>(j));
    for (auto & ev : group) {
      if (ev.kind == EventInfo::Kind::Til) {
        const auto & item = w_.problem.init[ev.index];
        Bindings none;
        Evaluator::Ctx c{state_};
        auto & pe = w_.problem_eval;
        if (item.kind == ast::InitItem::Kind::Fact) {
          auto pred = w_.dm.predicates.find(w_.problem.symbols.name(item.name));
          if (!pred) {
            continue;
          }
          Key key{pred->value()};
          for (TermId a : item.args) {
            key.push_back(pe.object_of(a, c, none));
          }
          (item.negated ? ev.changes.dels : ev.changes.adds).push_back(std::move(key));
        } else {
          auto fn = w_.dm.functions.find(w_.problem.symbols.name(item.name));
          if (!fn) {
            continue;
          }
          Key key{fn->value()};
          for (TermId a : item.args) {
            key.push_back(pe.object_of(a, c, none));
          }
          const ast::Expr & v = w_.problem.nodes.exprs[item.value];
          if (w_.dm.functions.items[*fn].numeric) {
            ev.changes.nums.push_back({std::move(key), ast::AssignOp::Assign, v.number});
          } else {
            ev.changes.objs.push_back({std::move(key), pe.object_of(v.term, c, none)});
          }
        }
        continue;
      }
      StepInfo & s = steps_[ev.index];
      Bindings b = s.bindings;
      try {
        if (!s.durative) {
          const auto & a = w_.domain.actions[s.action];
          Evaluator::Ctx c{state_};
          c.reads = &ev.reads;
          c.prefs = &report_.violated_preferences;
          if (a.precondition.valid() && !w_.domain_eval.holds(a.precondition, c, b)) {
            Evaluator::Ctx plain{state_};
            fail("validate.precondition_failed", step_name(s) + ": precondition false: " +
              w_.domain_eval.why(a.precondition, plain, b), s.step->line);
          }
          if (a.effect.valid()) {
            w_.domain_eval.effects(a.effect, c, b, true, ev.changes);
          }
          continue;
        }
        const auto & a = w_.domain.durative_actions[s.action];
        Phase phase = ev.kind == EventInfo::Kind::Start ? Phase::Start : Phase::End;
        Evaluator::Ctx c{state_, s.start_state.get(), phase, s.step->duration};
        c.reads = &ev.reads;
        c.prefs = &report_.violated_preferences;
        if (phase == Phase::Start) {
          Evaluator::Ctx dc{state_, nullptr, Phase::Instant, s.step->duration};
          dc.duration_check = true;
          if (!w_.domain_eval.holds(a.duration, dc, b)) {
            fail("validate.duration_mismatch", step_name(s) + ": duration " +
              std::to_string(s.step->duration) + " does not satisfy " +
              pddl::to_pddl(w_.domain.nodes, w_.domain.symbols, a.duration) + ": " +
              w_.domain_eval.why(a.duration, dc, b), s.step->line);
          }
        }
        if (a.condition.valid() && !w_.domain_eval.holds(a.condition, c, b)) {
          Evaluator::Ctx plain{state_, s.start_state.get(), phase, s.step->duration};
          fail("validate.precondition_failed", step_name(s) + ": " +
            (phase == Phase::Start ? "at start" : "at end") + " condition false: " +
            w_.domain_eval.why(a.condition, plain, b), s.step->line);
        }
        if (a.effect.valid()) {
          w_.domain_eval.effects(a.effect, c, b, false, ev.changes);
        }
      } catch (const EvalError & e) {
        fail("validate.evaluation", step_name(s) + ": " + e.message, s.step->line);
      }
    }

    // Simultaneous events must not interfere; nor events closer than epsilon
    auto describe = [&](const EventInfo & ev) {
        if (ev.kind == EventInfo::Kind::Til) {
          return std::string("a timed initial literal");
        }
        std::string what = ev.kind == EventInfo::Kind::Start ? "the start of " :
          ev.kind == EventInfo::Kind::End ? "the end of " : "";
        return what + step_name(steps_[ev.index]);
      };
    auto line_of = [&](const EventInfo & ev) {
        return ev.kind == EventInfo::Kind::Til ? 0u : steps_[ev.index].step->line;
      };
    for (size_t x = 0; x < group.size(); ++x) {
      for (size_t y = x + 1; y < group.size(); ++y) {
        if (group[x].kind == EventInfo::Kind::Start && group[y].kind == EventInfo::Kind::End &&
          group[x].index == group[y].index)
        {
          continue;
        }
        if (interfere(group[x], group[y])) {
          fail("validate.interference", describe(group[x]) + " and " + describe(group[y]) +
            " happen at the same time and interfere", line_of(group[y]));
        }
      }
    }
    if (have_previous && t - previous_time < w_.options.epsilon - tol) {
      for (const auto & pv : previous) {
        for (const auto & ev : group) {
          if (!interfere(pv, ev)) {
            continue;
          }
          std::string message = describe(pv) + " and " + describe(ev) + " interfere and are "
            "less than " + std::to_string(w_.options.epsilon) + " apart";
          if (w_.options.strict_epsilon) {
            fail("validate.too_close", message, line_of(ev));
          }
          report_.diagnostics.push_back(
            Diagnostic::warning("validate.too_close", message,
            SourceLocation{w_.options.plan_filename, line_of(ev), 0}));
        }
      }
    }

    // Starts whose when-conditions read "at start" need the state before the happening
    for (const auto & ev : group) {
      if (ev.kind != EventInfo::Kind::Start) {
        continue;
      }
      StepInfo & s = steps_[ev.index];
      const auto & a = w_.domain.durative_actions[s.action];
      if (a.effect.valid() && w_.domain_eval.has_when(a.effect)) {
        s.start_state = std::make_unique<State>();
        s.start_state->atoms = state_.atoms;
        s.start_state->numbers = state_.numbers;
        s.start_state->objects = state_.objects;
      }
    }

    // Apply all effects together: deletes, then adds (an action that deletes and adds the same
    // atom keeps it), then fluents
    State before;
    before.numbers = state_.numbers;
    for (const auto & ev : group) {
      for (const auto & k : ev.changes.dels) {
        state_.atoms.erase(k);
      }
    }
    for (const auto & ev : group) {
      for (const auto & k : ev.changes.adds) {
        state_.atoms.insert(k);
      }
    }
    for (const auto & ev : group) {
      for (const auto & n : ev.changes.nums) {
        auto old = before.numbers.find(n.key);
        auto & slot = state_.numbers[n.key];
        if (n.op != ast::AssignOp::Assign && old == before.numbers.end()) {
          fail("validate.undefined_fluent", w_.key_text(n.key, false) +
            " is changed but has no value", line_of(ev));
        }
        switch (n.op) {
          case ast::AssignOp::Assign: slot = n.value; break;
          case ast::AssignOp::Increase: slot += n.value; break;
          case ast::AssignOp::Decrease: slot -= n.value; break;
          case ast::AssignOp::ScaleUp: slot *= n.value; break;
          case ast::AssignOp::ScaleDown:
            if (n.value == 0.0) {
              fail("validate.evaluation", "scale-down by zero", line_of(ev));
            }
            slot /= n.value;
            break;
        }
      }
      for (const auto & o : ev.changes.objs) {
        if (o.value) {
          state_.objects[o.key] = *o.value;
        } else {
          state_.objects.erase(o.key);
        }
      }
    }
    state_.changed();

    for (const auto & ev : group) {
      if (ev.kind == EventInfo::Kind::Start) {
        StepInfo & s = steps_[ev.index];
        s.started = true;
        running_.push_back(ev.index);
      } else if (ev.kind == EventInfo::Kind::End) {
        steps_[ev.index].ended = true;
        running_.erase(std::find(running_.begin(), running_.end(), ev.index));
      }
      if (ev.kind != EventInfo::Kind::Til) {
        makespan_ = std::max(makespan_, t);
      }
    }
    check_invariants(state_, "after the happening at " + std::to_string(t));
    check_global_constraints(state_, "at " + std::to_string(t));
    // In sequential plans the state after step k (at position k) is state k+1
    state_.time = plan_.temporal ? t : t + 1.0;
    record_trajectory();

    previous = std::move(group);
    previous_time = t;
    have_previous = true;
    i = j;
  }
  // Sequential plans: one time unit per step
  if (!plan_.temporal) {
    makespan_ = static_cast<double>(plan_.steps.size());
  }
  finish();
}

bool Validator::constraint(FormulaId id, Evaluator & ev, Bindings & b)
{
  const ast::Formula & f = ev.nodes().formulas[id];
  auto at = [&](size_t k, FormulaId g) {
      Evaluator::Ctx c{trajectory_[k].second};
      return ev.holds(g, c, b);
    };
  const size_t n = trajectory_.size();
  switch (f.kind) {
    case ast::FormulaKind::And:
      for (FormulaId ch : f.children) {
        if (!constraint(ch, ev, b)) {
          return false;
        }
      }
      return true;
    case ast::FormulaKind::Forall: {
        bool ok = true;
        ev.for_each_binding(f.params, b, [&]() {
            ok = constraint(f.children.at(0), ev, b);
            return ok;
          });
        return ok;
      }
    case ast::FormulaKind::Preference: {
        bool v = constraint(f.children.at(0), ev, b);
        if (!v) {
          report_.violated_preferences[f.name.valid() ? ev.name(f.name) : "anonymous"] += 1;
        }
        return true;
      }
    case ast::FormulaKind::AtEnd:
      return at(n - 1, f.children.at(0));
    case ast::FormulaKind::Modal:
      break;
    default:
      return at(n - 1, id);
  }
  FormulaId p = f.children.at(0);
  auto time = [&](size_t k) {return trajectory_[k].first;};
  switch (f.modal) {
    case ast::ModalOp::Always:
      for (size_t k = 0; k < n; ++k) {
        if (!at(k, p)) {
          return false;
        }
      }
      return true;
    case ast::ModalOp::Sometime:
      for (size_t k = 0; k < n; ++k) {
        if (at(k, p)) {
          return true;
        }
      }
      return false;
    case ast::ModalOp::Within:
      for (size_t k = 0; k < n && time(k) <= f.times[0]; ++k) {
        if (at(k, p)) {
          return true;
        }
      }
      return false;
    case ast::ModalOp::AtMostOnce: {
        int rises = 0;
        bool was = false;
        for (size_t k = 0; k < n; ++k) {
          bool v = at(k, p);
          rises += v && !was;
          was = v;
        }
        return rises <= 1;
      }
    case ast::ModalOp::SometimeAfter: {
        FormulaId q = f.children.at(1);
        for (size_t k = 0; k < n; ++k) {
          if (at(k, p)) {
            bool found = false;
            for (size_t m = k; m < n && !found; ++m) {
              found = at(m, q);
            }
            if (!found) {
              return false;
            }
          }
        }
        return true;
      }
    case ast::ModalOp::SometimeBefore: {
        FormulaId q = f.children.at(1);
        bool seen_q = false;
        for (size_t k = 0; k < n; ++k) {
          if (at(k, p) && !seen_q) {
            return false;
          }
          seen_q = seen_q || at(k, q);
        }
        return true;
      }
    case ast::ModalOp::AlwaysWithin: {
        FormulaId q = f.children.at(1);
        for (size_t k = 0; k < n; ++k) {
          if (at(k, p)) {
            bool found = false;
            for (size_t m = k; m < n && time(m) <= time(k) + f.times[0] && !found; ++m) {
              found = at(m, q);
            }
            if (!found) {
              return false;
            }
          }
        }
        return true;
      }
    case ast::ModalOp::HoldDuring:
      for (size_t k = 0; k < n; ++k) {
        if (time(k) >= f.times[0] && time(k) < f.times[1] && !at(k, p)) {
          return false;
        }
      }
      return true;
    case ast::ModalOp::HoldAfter:
      for (size_t k = 0; k < n; ++k) {
        if (time(k) > f.times[0] && !at(k, p)) {
          return false;
        }
      }
      return true;
  }
  return true;
}

void Validator::finish()
{
  report_.makespan = makespan_;
  const auto & p = w_.problem;
  if (p.goal.valid() && w_.options.check_goal) {
    Bindings b;
    Evaluator::Ctx c{state_};
    c.prefs = &report_.violated_preferences;
    try {
      if (!w_.problem_eval.holds(p.goal, c, b)) {
        Evaluator::Ctx plain{state_};
        fail("validate.goal_not_reached",
          "the goal does not hold at the end of the plan: " + w_.problem_eval.why(p.goal, plain,
          b), 0);
      }
    } catch (const EvalError & e) {
      fail("validate.evaluation", "evaluating the goal: " + e.message, 0);
    }
  }
  try {
    Bindings b;
    if (w_.domain.constraints.valid() && !constraint(w_.domain.constraints, w_.domain_eval, b)) {
      fail("validate.constraint_violated", "the plan violates the domain :constraints", 0);
    }
    if (p.constraints.valid() && !constraint(p.constraints, w_.problem_eval, b)) {
      fail("validate.constraint_violated", "the plan violates the problem :constraints", 0);
    }
  } catch (const EvalError & e) {
    fail("validate.evaluation", "evaluating constraints: " + e.message, 0);
  }
  if (p.metric) {
    Bindings b;
    Evaluator::Ctx c{state_};
    c.total_time = makespan_;
    c.prefs = &report_.violated_preferences;
    try {
      report_.metric = w_.problem_eval.number(p.metric->expr, c, b);
    } catch (const EvalError & e) {
      report_.diagnostics.push_back(
        Diagnostic::warning("validate.metric", "the metric cannot be computed: " + e.message));
    }
  }
  report_.status = ValidationStatus::Valid;
}

}  // namespace

ValidationReport validate(
  const ast::Domain & domain, const sem::DomainModel & domain_model,
  const ast::Problem & problem, const sem::ProblemModel & problem_model, const Plan & plan,
  const ValidateOptions & options)
{
  ValidationReport report;
  report.status = ValidationStatus::Invalid;
  World world(domain, domain_model, problem, problem_model, options);
  Validator v(world, plan, report);
  try {
    v.run();
  } catch (const Unsupported & u) {
    report.status = ValidationStatus::Unsupported;
    report.diagnostics.push_back(Diagnostic::error("validate.unsupported", u.message));
  } catch (const EvalError & e) {
    report.status = ValidationStatus::Invalid;
    report.diagnostics.push_back(Diagnostic::error("validate.evaluation", e.message));
  } catch (...) {
    // Stop: the failure is already in the diagnostics
    report.status = ValidationStatus::Invalid;
  }
  return report;
}

}  // namespace kairos::val
