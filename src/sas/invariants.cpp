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

#include "sas/invariants.hpp"

#include <algorithm>
#include <chrono>
#include <deque>
#include <functional>
#include <map>
#include <optional>
#include <random>
#include <set>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace kairos::sas
{

using ground::CompareOp;
using ground::Timing;
using sem::ObjectId;

const InvariantPart * Invariant::part(sem::PredicateId p) const
{
  for (const auto & x : parts) {
    if (x.predicate == p) {
      return &x;
    }
  }
  return nullptr;
}

namespace
{

// --- Terms and constraint systems (as FD's constraints.py) ----------------------------------

enum class SymKind : uint8_t {Object, Var, Param};

struct Sym
{
  SymKind kind = SymKind::Object;
  uint32_t id = 0;

  uint64_t key() const {return (uint64_t{static_cast<uint8_t>(kind)} << 32) | id;}
  bool constant() const {return kind == SymKind::Object;}
  bool operator==(const Sym &) const = default;
};

Sym var(uint32_t v) {return Sym{SymKind::Var, v};}
Sym param(int p) {return Sym{SymKind::Param, static_cast<uint32_t>(p)};}

constexpr uint32_t kEqual = UINT32_MAX;  // the "=" predicate

struct Lit
{
  uint32_t pred = 0;
  std::vector<Sym> args;
  bool negated = false;

  Lit negate() const
  {
    Lit l = *this;
    l.negated = !negated;
    return l;
  }
};

using Pair = std::pair<Sym, Sym>;
using EqConj = std::vector<Pair>;

// Equivalence classes of a conjunction of equalities; inconsistent if two objects meet
class Classes
{
public:
  explicit Classes(const EqConj & eqs)
  {
    for (const auto & [a, b] : eqs) {
      unite(a.key(), b.key());
    }
    for (const auto & [a, b] : eqs) {
      for (const Sym & s : {a, b}) {
        if (s.constant()) {
          uint64_t r = find(s.key());
          auto [it, fresh] = object_.emplace(r, s.key());
          if (!fresh && it->second != s.key()) {
            consistent_ = false;
          }
        }
      }
    }
  }

  bool consistent() const {return consistent_;}

  uint64_t rep(const Sym & s) const
  {
    if (!parent_.count(s.key())) {
      return s.key();
    }
    uint64_t r = find(s.key());
    auto it = object_.find(r);
    return it == object_.end() ? r : it->second;
  }

  bool is_constant(const Sym & s) const
  {
    if (s.constant()) {
      return true;
    }
    if (!parent_.count(s.key())) {
      return false;
    }
    return object_.count(find(s.key())) > 0;
  }

private:
  uint64_t find(uint64_t x) const
  {
    while (true) {
      auto it = parent_.find(x);
      if (it == parent_.end() || it->second == x) {
        return x;
      }
      x = it->second;
    }
  }

  void unite(uint64_t a, uint64_t b)
  {
    parent_.try_emplace(a, a);
    parent_.try_emplace(b, b);
    uint64_t ra = find(a), rb = find(b);
    if (ra != rb) {
      parent_[rb] = ra;
    }
  }

  std::unordered_map<uint64_t, uint64_t> parent_;
  std::unordered_map<uint64_t, uint64_t> object_;  // root -> object in the class
  bool consistent_ = true;
};

// A conjunction of disjunctions of equality conjunctions, inequality disjunctions and terms
// that may not become constants. Solvable if one choice per disjunction satisfies the rest.
struct System
{
  std::vector<std::vector<EqConj>> dnfs;
  std::vector<std::vector<Pair>> ineqs;
  std::vector<Sym> not_constant;

  void add_conj(EqConj c) {dnfs.push_back({std::move(c)});}

  void extend(const System & o)
  {
    dnfs.insert(dnfs.end(), o.dnfs.begin(), o.dnfs.end());
    ineqs.insert(ineqs.end(), o.ineqs.begin(), o.ineqs.end());
    not_constant.insert(not_constant.end(), o.not_constant.begin(), o.not_constant.end());
  }

  bool solvable() const
  {
    for (const auto & d : dnfs) {
      if (d.empty()) {
        return false;
      }
    }
    std::vector<size_t> pick(dnfs.size(), 0);
    while (true) {
      EqConj all;
      for (size_t i = 0; i < dnfs.size(); ++i) {
        const auto & c = dnfs[i][pick[i]];
        all.insert(all.end(), c.begin(), c.end());
      }
      Classes cls(all);
      bool ok = cls.consistent();
      for (size_t i = 0; ok && i < not_constant.size(); ++i) {
        ok = !cls.is_constant(not_constant[i]);
      }
      for (size_t i = 0; ok && i < ineqs.size(); ++i) {
        ok = std::any_of(
          ineqs[i].begin(), ineqs[i].end(),
          [&](const Pair & p) {return cls.rep(p.first) != cls.rep(p.second);});
      }
      if (ok) {
        return true;
      }
      size_t k = 0;
      for (; k < pick.size(); ++k) {
        if (++pick[k] < dnfs[k].size()) {
          break;
        }
        pick[k] = 0;
      }
      if (k == pick.size()) {
        return false;
      }
    }
  }
};

EqConj zip(const std::vector<Sym> & a, const std::vector<Sym> & b)
{
  EqConj out;
  for (size_t i = 0; i < a.size() && i < b.size(); ++i) {
    out.emplace_back(a[i], b[i]);
  }
  return out;
}

// The conjunction of all these literals must be satisfiable
void ensure_conjunction_sat(System & sys, std::initializer_list<const std::vector<Lit> *> parts)
{
  std::map<uint32_t, std::vector<const Lit *>> pos, neg;
  for (const auto * part : parts) {
    for (const auto & l : *part) {
      if (l.pred == kEqual) {
        if (l.negated) {
          sys.ineqs.push_back({{l.args[0], l.args[1]}});
        } else {
          sys.add_conj({{l.args[0], l.args[1]}});
        }
      } else {
        (l.negated ? neg : pos)[l.pred].push_back(&l);
      }
    }
  }
  for (const auto & [p, ps] : pos) {
    auto it = neg.find(p);
    if (it == neg.end()) {
      continue;
    }
    for (const Lit * a : ps) {
      for (const Lit * b : it->second) {
        if (!a->args.empty()) {
          sys.ineqs.push_back(zip(b->args, a->args));
        }
      }
    }
  }
}

void ensure_inequality(System & sys, const Lit & a, const Lit & b)
{
  if (a.pred == b.pred && !a.args.empty()) {
    sys.ineqs.push_back(zip(a.args, b.args));
  }
}

EqConj cover(const Invariant & inv, const Lit & l)
{
  const InvariantPart * p = inv.part(sem::PredicateId(l.pred));
  EqConj out;
  for (size_t i = 0; i < p->args.size(); ++i) {
    if (p->args[i] != kCounted) {
      out.emplace_back(param(p->args[i]), l.args[i]);
    }
  }
  return out;
}

// --- Actions as the balance check sees them -------------------------------------------------

struct BEffect
{
  std::vector<uint32_t> params;  // forall variables
  std::vector<Lit> condition;
  Lit literal;
  bool usable = true;            // false: may not balance (depends on the start state)
};

struct BAction
{
  std::vector<uint32_t> params;  // variables the action binds
  std::vector<Lit> pre;
  std::vector<BEffect> effects;
  std::vector<BEffect> heavy;    // effects and a renamed copy of each universal one
  // End of a durative action: unconditional start deletes of at-start conditions
  const BAction * start = nullptr;
  std::vector<Lit> start_pre;
  std::vector<BEffect> reserved;
};

// --- Invariant parts and refinement ---------------------------------------------------------

std::map<int, Sym> parameters_of(const InvariantPart & part, const Lit & l)
{
  std::map<int, Sym> out;
  for (size_t i = 0; i < part.args.size(); ++i) {
    if (static_cast<int>(i) != part.omitted) {
      out.emplace(part.args[i], l.args[i]);
    }
  }
  return out;
}

// New parts for `other` (a delete) that let it balance `own` (an add), as FD's
// InvariantPart.possible_matches
std::vector<InvariantPart> possible_matches(
  const InvariantPart & part, const Lit & own, const Lit & other)
{
  std::vector<InvariantPart> out;
  int allowed = static_cast<int>(other.args.size()) - static_cast<int>(part.arity());
  if (allowed != 0 && allowed != 1) {
    return out;
  }
  // own argument -> invariant parameters taking it
  std::vector<std::pair<uint64_t, std::vector<int>>> own_to_params;
  for (const auto & [p, s] : parameters_of(part, own)) {
    auto it = std::find_if(
      own_to_params.begin(), own_to_params.end(),
      [&](const auto & kv) {return kv.first == s.key();});
    if (it == own_to_params.end()) {
      own_to_params.emplace_back(s.key(), std::vector<int>{p});
    } else {
      it->second.push_back(p);
    }
  }
  // other argument -> its positions, in order of appearance
  std::vector<std::pair<uint64_t, std::vector<size_t>>> other_to_pos;
  for (size_t i = 0; i < other.args.size(); ++i) {
    uint64_t k = other.args[i].key();
    auto it = std::find_if(
      other_to_pos.begin(), other_to_pos.end(), [&](const auto & kv) {return kv.first == k;});
    if (it == other_to_pos.end()) {
      other_to_pos.emplace_back(k, std::vector<size_t>{i});
    } else {
      it->second.push_back(i);
    }
  }
  std::vector<std::pair<std::vector<size_t>, std::vector<int>>> factored;
  for (const auto & [k, positions] : other_to_pos) {
    std::vector<int> params;
    for (const auto & kv : own_to_params) {
      if (kv.first == k) {
        params = kv.second;
      }
    }
    int diff = static_cast<int>(params.size()) - static_cast<int>(positions.size());
    if (diff >= 1 || diff <= -2 || (diff == -1 && !allowed)) {
      return out;
    }
    if (diff) {
      params.push_back(kCounted);
      allowed = 0;
    }
    factored.emplace_back(positions, params);
  }
  // Every combination of a bijection per entry
  std::vector<std::vector<std::vector<int>>> perms;
  for (auto & [positions, params] : factored) {
    std::vector<std::vector<int>> ps;
    std::vector<int> p = params;
    std::sort(p.begin(), p.end());
    do {
      ps.push_back(p);
    } while (std::next_permutation(p.begin(), p.end()));
    perms.push_back(std::move(ps));
  }
  std::vector<size_t> pick(perms.size(), 0);
  while (true) {
    InvariantPart np;
    np.predicate = sem::PredicateId(other.pred);
    np.args.assign(other.args.size(), kCounted);
    for (size_t e = 0; e < factored.size(); ++e) {
      const auto & positions = factored[e].first;
      const auto & img = perms[e][pick[e]];
      for (size_t i = 0; i < positions.size(); ++i) {
        if (img[i] == kCounted) {
          np.omitted = static_cast<int>(positions[i]);
        } else {
          np.args[positions[i]] = img[i];
        }
      }
    }
    out.push_back(np);
    size_t k = 0;
    for (; k < pick.size(); ++k) {
      if (++pick[k] < perms[k].size()) {
        break;
      }
      pick[k] = 0;
    }
    if (k == pick.size()) {
      break;
    }
  }
  return out;
}

class Finder
{
public:
  Finder(
    const lifted::Task & t, const ground::Task & g, const InvariantOptions & o, Budget * b)
  : t_(t), g_(g), options_(o), budget_(b) {}

  std::vector<Invariant> run(InvariantStats * stats)
  {
    auto t0 = std::chrono::steady_clock::now();
    build_actions();
    std::deque<Invariant> queue;
    std::set<Invariant> seen;
    auto enqueue = [&](Invariant inv) {
        std::sort(inv.parts.begin(), inv.parts.end());
        if (seen.size() < options_.max_candidates && !seen.count(inv)) {
          seen.insert(inv);
          queue.push_back(std::move(inv));
        }
      };
    for (size_t p = 0; p < t_.predicate_names.size(); ++p) {
      if (!candidate_[p]) {
        continue;
      }
      auto n = static_cast<int>(t_.predicate_params[p].size());
      InvariantPart whole{sem::PredicateId(static_cast<uint32_t>(p)), {}, -1};
      for (int i = 0; i < n; ++i) {
        whole.args.push_back(i);
      }
      enqueue(Invariant{{whole}});
      for (int omitted = 0; omitted < n; ++omitted) {
        InvariantPart part{sem::PredicateId(static_cast<uint32_t>(p)), {}, omitted};
        for (int i = 0; i < n - 1; ++i) {
          if (i == omitted) {
            part.args.push_back(kCounted);
          }
          part.args.push_back(i);
        }
        if (omitted == n - 1) {
          part.args.push_back(kCounted);
        }
        enqueue(Invariant{{part}});
      }
    }
    std::vector<Invariant> found;
    bool limit = false;
    while (!queue.empty()) {
      double s = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
      if (s > options_.max_seconds || (budget_ && budget_->exhausted())) {
        limit = true;
        break;
      }
      Invariant inv = std::move(queue.front());
      queue.pop_front();
      if (check(inv, enqueue)) {
        found.push_back(std::move(inv));
      }
    }
    if (stats) {
      stats->candidates = seen.size();
      stats->invariants = found.size();
      stats->limit_reached = limit || seen.size() >= options_.max_candidates;
      stats->milliseconds = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - t0).count();
    }
    return found;
  }

private:
  // --- From the lifted task ---

  std::optional<Sym> sym(const lifted::Term & x) const
  {
    switch (x.kind) {
      case lifted::Term::Kind::Variable: return var(x.index);
      case lifted::Term::Kind::Object: return Sym{SymKind::Object, x.index};
      default: return std::nullopt;  // object fluents: not representable
    }
  }

  void literals(const lifted::Condition & c, std::vector<Lit> & out) const
  {
    for (const auto & l : c.literals) {
      Lit x{l.atom.predicate.value(), {}, !l.positive};
      bool ok = true;
      for (const auto & a : l.atom.args) {
        auto s = sym(a);
        ok = ok && s.has_value();
        if (s) {
          x.args.push_back(*s);
        }
      }
      if (ok) {
        out.push_back(std::move(x));  // dropping a literal only weakens what is known
      }
    }
    for (const auto & cmp : c.comparisons) {
      if (!cmp.objects || (cmp.op != CompareOp::Eq && cmp.op != CompareOp::Ne)) {
        continue;
      }
      auto a = sym(cmp.lhs_term), b = sym(cmp.rhs_term);
      if (a && b) {
        out.push_back(Lit{kEqual, {*a, *b}, cmp.op == CompareOp::Ne});
      }
    }
  }

  // Effects at one moment; false-free: unrepresentable atoms mark their predicate unsafe
  void effects(
    const std::vector<lifted::Effect> & list, std::initializer_list<Timing> timings,
    std::vector<BEffect> & out)
  {
    for (const auto & e : list) {
      if ((e.kind != lifted::Effect::Kind::Add && e.kind != lifted::Effect::Kind::Delete) ||
        std::find(timings.begin(), timings.end(), e.timing) == timings.end())
      {
        continue;
      }
      BEffect b;
      b.params = e.variables;
      b.literal = Lit{e.atom.predicate.value(), {}, e.kind == lifted::Effect::Kind::Delete};
      bool ok = true;
      for (const auto & a : e.atom.args) {
        auto s = sym(a);
        ok = ok && s.has_value();
        if (s) {
          b.literal.args.push_back(*s);
        }
      }
      if (!ok) {
        unsafe_[e.atom.predicate.index()] = true;
        continue;
      }
      literals(e.condition, b.condition);
      if (!e.start_condition.empty()) {
        b.usable = false;  // depends on the state at start: cannot balance at end
      }
      out.push_back(std::move(b));
    }
  }

  // Parameter pairs never equal in the reachable instances are different
  void inequalities(
    BAction & a, uint32_t parameters, const std::vector<std::vector<ObjectId>> & instances)
  {
    for (uint32_t i = 0; i < parameters; ++i) {
      for (uint32_t j = i + 1; j < parameters; ++j) {
        bool equal = std::any_of(
          instances.begin(), instances.end(),
          [&](const std::vector<ObjectId> & args) {return args[i] == args[j];});
        if (!equal) {
          a.pre.push_back(Lit{kEqual, {var(i), var(j)}, true});
        }
      }
    }
  }

  void finish(BAction & a, uint32_t nvars)
  {
    // The heavy version: universal effects twice, the copy with fresh variables
    uint32_t fresh = nvars;
    for (const auto & e : a.effects) {
      a.heavy.push_back(e);
      if (!e.params.empty()) {
        std::map<uint32_t, uint32_t> rename;
        for (uint32_t v : e.params) {
          rename[v] = fresh++;
        }
        auto ren = [&](Lit & l) {
            for (auto & s : l.args) {
              if (s.kind == SymKind::Var && rename.count(s.id)) {
                s.id = rename[s.id];
              }
            }
          };
        BEffect copy = e;
        for (auto & v : copy.params) {
          v = rename[v];
        }
        ren(copy.literal);
        for (auto & l : copy.condition) {
          ren(l);
        }
        a.heavy.push_back(std::move(copy));
      }
    }
  }

  void add_instant(
    const lifted::Action & la, const std::vector<std::vector<ObjectId>> & instances)
  {
    BAction a;
    for (uint32_t v = 0; v < la.parameters + la.hidden; ++v) {
      a.params.push_back(v);
    }
    literals(la.precondition, a.pre);
    effects(la.effects, {Timing::Instant}, a.effects);
    inequalities(a, la.parameters, instances);
    finish(a, static_cast<uint32_t>(la.variables.size()));
    actions_.push_back(std::move(a));
  }

  void build_actions()
  {
    unsafe_.assign(t_.predicate_names.size(), false);
    auto by_schema = [](const auto & list, size_t n) {
        std::vector<std::vector<std::vector<ObjectId>>> out(n);
        for (const auto & a : list) {
          out[a.schema].push_back(a.args);
        }
        return out;
      };
    auto ia = by_schema(g_.actions, t_.actions.size());
    auto ip = by_schema(g_.processes, t_.processes.size());
    auto ie = by_schema(g_.events, t_.events.size());
    auto id = by_schema(g_.durative_actions, t_.durative_actions.size());
    actions_.reserve(
      t_.actions.size() + t_.processes.size() + t_.events.size() +
      2 * t_.durative_actions.size());
    for (size_t i = 0; i < t_.actions.size(); ++i) {
      add_instant(t_.actions[i], ia[i]);
    }
    for (size_t i = 0; i < t_.processes.size(); ++i) {
      add_instant(t_.processes[i], ip[i]);
    }
    for (size_t i = 0; i < t_.events.size(); ++i) {
      add_instant(t_.events[i], ie[i]);
    }
    for (size_t i = 0; i < t_.durative_actions.size(); ++i) {
      const auto & da = t_.durative_actions[i];
      auto nvars = static_cast<uint32_t>(da.variables.size());
      BAction s;
      for (uint32_t v = 0; v < da.parameters + da.hidden; ++v) {
        s.params.push_back(v);
      }
      literals(da.at_start, s.pre);
      effects(da.effects, {Timing::Start}, s.effects);
      inequalities(s, da.parameters, id[i]);
      finish(s, nvars);
      actions_.push_back(std::move(s));
      const BAction * start = &actions_.back();

      BAction e;
      e.params = start->params;
      literals(da.over_all, e.pre);
      literals(da.at_end, e.pre);
      effects(da.effects, {Timing::End}, e.effects);
      inequalities(e, da.parameters, id[i]);
      e.start = start;
      e.start_pre = start->pre;
      // Unconditional start deletes of atoms required at start: reserved for the end
      for (const auto & d : start->effects) {
        if (!d.literal.negated || !d.params.empty() || !d.condition.empty()) {
          continue;
        }
        bool required = std::any_of(
          start->pre.begin(), start->pre.end(), [&](const Lit & l) {
            return !l.negated && l.pred == d.literal.pred && l.args == d.literal.args;
          });
        if (required) {
          e.reserved.push_back(d);
        }
      }
      finish(e, nvars);
      actions_.push_back(std::move(e));
    }
    // Candidates: changing basic predicates whose effects are representable and that no
    // timed initial literal adds
    candidate_.assign(t_.predicate_names.size(), false);
    threats_.assign(t_.predicate_names.size(), {});
    for (size_t a = 0; a < actions_.size(); ++a) {
      for (const auto & e : actions_[a].effects) {
        candidate_[e.literal.pred] = true;
        if (!e.literal.negated) {
          auto & t = threats_[e.literal.pred];
          if (t.empty() || t.back() != a) {
            t.push_back(a);
          }
        }
      }
    }
    for (const auto & ev : t_.timed_events) {
      if (ev.fact && !ev.negated) {
        unsafe_[ev.fact->first.index()] = true;
      }
    }
    for (size_t p = 0; p < candidate_.size(); ++p) {
      candidate_[p] = candidate_[p] && !t_.derived[p] && !unsafe_[p];
    }
  }

  // --- The balance check ---

  bool covered(const Invariant & inv, const Lit & l) const
  {
    return l.pred != kEqual && inv.part(sem::PredicateId(l.pred)) != nullptr;
  }

  template<class Enqueue>
  bool check(const Invariant & inv, Enqueue & enqueue)
  {
    for (const auto & p : inv.parts) {
      if (unsafe_[p.predicate.index()]) {
        return false;
      }
    }
    // Actions adding some part, each once (marks reused across candidates)
    todo_.clear();
    ++stamp_;
    if (mark_.size() < actions_.size()) {
      mark_.assign(actions_.size(), 0);
    }
    for (const auto & p : inv.parts) {
      for (size_t a : threats_[p.predicate.index()]) {
        if (mark_[a] != stamp_) {
          mark_[a] = stamp_;
          todo_.push_back(a);
        }
      }
    }
    auto & todo = todo_;
    // Random order: candidates usually fail early, on some action. Refining from every
    // unbalanced action instead finds a few more invariants but explodes the candidates.
    while (!todo.empty()) {
      size_t pos = std::uniform_int_distribution<size_t>(0, todo.size() - 1)(rng_);
      std::swap(todo[pos], todo.back());
      const BAction & a = actions_[todo.back()];
      todo.pop_back();
      if (too_heavy(inv, a) || unbalanced(inv, a, enqueue)) {
        return false;
      }
    }
    return true;
  }

  bool too_heavy(const Invariant & inv, const BAction & a) const
  {
    std::vector<const BEffect *> adds;
    for (const auto & e : a.heavy) {
      if (!e.literal.negated && covered(inv, e.literal)) {
        adds.push_back(&e);
      }
    }
    for (size_t i = 0; i < adds.size(); ++i) {
      for (size_t j = i + 1; j < adds.size(); ++j) {
        System sys;
        ensure_inequality(sys, adds[i]->literal, adds[j]->literal);
        sys.add_conj(cover(inv, adds[i]->literal));
        sys.add_conj(cover(inv, adds[j]->literal));
        std::vector<Lit> n1{adds[i]->literal.negate()}, n2{adds[j]->literal.negate()};
        ensure_conjunction_sat(sys, {&a.pre, &adds[i]->condition, &adds[j]->condition, &n1, &n2});
        if (sys.solvable()) {
          return true;
        }
      }
    }
    return false;
  }

  template<class Enqueue>
  bool unbalanced(const Invariant & inv, const BAction & a, Enqueue & enqueue)
  {
    std::vector<const BEffect *> dels;
    for (const auto & e : a.effects) {
      if (e.literal.negated && e.usable && covered(inv, e.literal)) {
        dels.push_back(&e);
      }
    }
    for (const auto & e : a.effects) {
      if (!e.literal.negated && covered(inv, e.literal) &&
        add_unbalanced(inv, a, e, dels, enqueue))
      {
        return true;
      }
    }
    return false;
  }

  using ByPred = std::map<uint32_t, std::vector<Lit>>;

  // Solvable if the add is guaranteed to come with the delete of another atom of its instance
  std::optional<System> balance_system(const BEffect & add, const BEffect & del, const ByPred & by)
  const
  {
    System sys;
    std::vector<Lit> needed = del.condition;
    needed.push_back(del.literal.negate());
    for (const auto & l : needed) {
      std::vector<EqConj> options;
      auto it = by.find(l.pred);
      if (it != by.end()) {
        for (const auto & m : it->second) {
          if (m.negated == l.negated) {
            options.push_back(zip(l.args, m.args));
          }
        }
      }
      if (options.empty()) {
        return std::nullopt;
      }
      sys.dnfs.push_back(std::move(options));
    }
    ensure_inequality(sys, add.literal, del.literal);
    return sys;
  }

  // `adds`: the add effects that would cancel the delete if they hit the same atom (PDDL:
  // delete, then add), so the delete must differ from all of them
  bool balances(
    const Invariant & inv, const BEffect & del, const BEffect & add, const ByPred & by,
    const EqConj & add_cover, const System & params, const std::vector<BEffect> & adds) const
  {
    auto bs = balance_system(add, del, by);
    if (!bs) {
      return false;
    }
    for (const auto & e : adds) {
      if (!e.literal.negated && e.literal.pred == del.literal.pred) {
        if (e.literal.args.empty() || e.literal.args == del.literal.args) {
          return false;  // always cancelled
        }
        ensure_inequality(*bs, e.literal, del.literal);
      }
    }
    System sys;
    sys.add_conj(add_cover);
    sys.add_conj(cover(inv, del.literal));
    sys.extend(*bs);
    sys.extend(params);
    return sys.solvable();
  }

  template<class Enqueue>
  bool add_unbalanced(
    const Invariant & inv, const BAction & a, const BEffect & add,
    const std::vector<const BEffect *> & dels, Enqueue & enqueue)
  {
    ByPred by;
    for (const auto & l : a.pre) {
      by[l.pred].push_back(l);
    }
    for (const auto & l : add.condition) {
      by[l.pred].push_back(l);
    }
    Lit was_false = add.literal.negate();
    by[was_false.pred].push_back(was_false);
    EqConj add_cover = cover(inv, add.literal);
    // The balance must hold for every instantiation that threatens the invariant: the action
    // parameters keep their freedom
    std::vector<uint32_t> ps = a.params;
    ps.insert(ps.end(), add.params.begin(), add.params.end());
    System params;
    Classes cls(add_cover);
    for (uint32_t p : ps) {
      if (!cls.is_constant(var(p))) {
        params.not_constant.push_back(var(p));
      }
    }
    for (size_t i = 0; i < ps.size(); ++i) {
      for (size_t j = i + 1; j < ps.size(); ++j) {
        if (cls.rep(var(ps[i])) != cls.rep(var(ps[j]))) {
          params.ineqs.push_back({{var(ps[i]), var(ps[j])}});
        }
      }
    }
    for (const BEffect * d : dels) {
      if (balances(inv, *d, add, by, add_cover, params, a.effects)) {
        return false;
      }
    }
    // End of a durative action: what the start reserved, unless the start adds to the
    // invariant itself (the reservation would count twice)
    if (a.start && !a.reserved.empty()) {
      bool start_adds = std::any_of(
        a.start->effects.begin(), a.start->effects.end(),
        [&](const BEffect & e) {return !e.literal.negated && covered(inv, e.literal);});
      if (!start_adds) {
        ByPred with_start = by;
        for (const auto & l : a.start_pre) {
          with_start[l.pred].push_back(l);
        }
        for (const auto & d : a.reserved) {
          if (covered(inv, d.literal) &&
            balances(inv, d, add, with_start, add_cover, params, a.start->effects))
          {
            return false;
          }
        }
      }
    }
    refine(inv, add, a, enqueue);
    return true;
  }

  template<class Enqueue>
  void refine(const Invariant & inv, const BEffect & add, const BAction & a, Enqueue & enqueue)
  {
    const InvariantPart * part = inv.part(sem::PredicateId(add.literal.pred));
    auto try_del = [&](const BEffect & d) {
        if (!d.literal.negated || inv.part(sem::PredicateId(d.literal.pred)) ||
          unsafe_[d.literal.pred] || t_.derived[d.literal.pred])
        {
          return;
        }
        for (auto & m : possible_matches(*part, add.literal, d.literal)) {
          Invariant bigger = inv;
          bigger.parts.push_back(std::move(m));
          enqueue(std::move(bigger));
        }
      };
    for (const auto & d : a.effects) {
      try_del(d);
    }
    for (const auto & d : a.reserved) {
      try_del(d);
    }
  }

  const lifted::Task & t_;
  const ground::Task & g_;
  InvariantOptions options_;
  Budget * budget_;
  std::vector<BAction> actions_;
  std::vector<bool> unsafe_;
  std::vector<bool> candidate_;
  std::vector<std::vector<size_t>> threats_;
  std::mt19937 rng_{314159};  // deterministic
  std::vector<size_t> todo_;
  std::vector<uint32_t> mark_;
  uint32_t stamp_ = 0;
};

// FD orders atoms by predicate name, then argument names
struct AtomOrder
{
  const ground::Task & g;
  bool operator()(ground::AtomId a, ground::AtomId b) const
  {
    const auto & x = g.atoms[a];
    const auto & y = g.atoms[b];
    const auto & px = g.predicate_names[x.predicate.index()];
    const auto & py = g.predicate_names[y.predicate.index()];
    if (px != py) {
      return px < py;
    }
    for (size_t i = 0; i < x.args.size() && i < y.args.size(); ++i) {
      const auto & ox = g.object_names[x.args[i].index()];
      const auto & oy = g.object_names[y.args[i].index()];
      if (ox != oy) {
        return ox < oy;
      }
    }
    return x.args.size() < y.args.size();
  }
};

}  // namespace

std::vector<Invariant> find_invariants(
  const lifted::Task & lifted, const ground::Task & ground, const InvariantOptions & options,
  Budget * budget, InvariantStats * stats)
{
  Finder f(lifted, ground, options, budget);
  return f.run(stats);
}

std::vector<std::vector<ground::AtomId>> mutex_groups(
  const std::vector<Invariant> & invariants, const ground::Task & g)
{
  using Key = std::vector<uint32_t>;
  std::map<Key, ground::AtomId> atom_of;
  for (ground::AtomId a = 0; a < g.atoms.size(); ++a) {
    Key k{g.atoms[a].predicate.value()};
    for (auto o : g.atoms[a].args) {
      k.push_back(o.value());
    }
    atom_of.emplace(std::move(k), a);
  }
  std::map<uint32_t, std::vector<size_t>> by_pred;
  for (size_t i = 0; i < invariants.size(); ++i) {
    for (const auto & p : invariants[i].parts) {
      by_pred[p.predicate.value()].push_back(i);
    }
  }
  // Instances with exactly one atom true at the start
  std::vector<std::pair<size_t, Key>> order;
  std::map<std::pair<size_t, Key>, int> count;
  for (ground::AtomId a : g.init_atoms) {
    const auto & info = g.atoms[a];
    auto it = by_pred.find(info.predicate.value());
    if (it == by_pred.end()) {
      continue;
    }
    for (size_t i : it->second) {
      const InvariantPart * part = invariants[i].part(info.predicate);
      Key params(invariants[i].arity(), 0);
      for (size_t pos = 0; pos < part->args.size(); ++pos) {
        if (part->args[pos] != kCounted) {
          params[static_cast<size_t>(part->args[pos])] = info.args[pos].value();
        }
      }
      auto key = std::make_pair(i, params);
      if (count[key]++ == 0) {
        order.push_back(key);
      }
    }
  }
  std::vector<std::vector<ground::AtomId>> out;
  for (const auto & key : order) {
    if (count[key] != 1) {
      continue;
    }
    std::vector<ground::AtomId> group;
    for (const auto & part : invariants[key.first].parts) {
      Key k{part.predicate.value()};
      for (int a : part.args) {
        k.push_back(a == kCounted ? 0 : key.second[static_cast<size_t>(a)]);
      }
      if (part.omitted < 0) {
        if (auto it = atom_of.find(k); it != atom_of.end()) {
          group.push_back(it->second);
        }
        continue;
      }
      for (uint32_t o = 0; o < g.object_names.size(); ++o) {
        k[static_cast<size_t>(part.omitted) + 1] = o;
        if (auto it = atom_of.find(k); it != atom_of.end()) {
          group.push_back(it->second);
        }
      }
    }
    std::sort(group.begin(), group.end(), AtomOrder{g});
    out.push_back(std::move(group));
  }
  AtomOrder less{g};
  std::sort(
    out.begin(), out.end(), [&](const auto & a, const auto & b) {
      return std::lexicographical_compare(a.begin(), a.end(), b.begin(), b.end(), less);
    });
  return out;
}

std::vector<std::vector<ground::AtomId>> choose_groups(
  const std::vector<std::vector<ground::AtomId>> & groups, const ground::Task & g)
{
  std::set<ground::AtomId> negative(g.goal.negative.begin(), g.goal.negative.end());
  // Groups as FD's GroupCoverQueue (partial encoding): a chosen atom leaves every group
  std::vector<std::vector<ground::AtomId>> members;
  for (const auto & grp : groups) {
    std::vector<ground::AtomId> m;
    for (ground::AtomId a : grp) {
      if (!negative.count(a) && std::find(m.begin(), m.end(), a) == m.end()) {
        m.push_back(a);
      }
    }
    members.push_back(std::move(m));
  }
  std::vector<size_t> size(members.size());
  std::unordered_map<ground::AtomId, std::vector<size_t>> by_atom;
  size_t max_size = 0;
  for (size_t i = 0; i < members.size(); ++i) {
    size[i] = members[i].size();
    max_size = std::max(max_size, size[i]);
    for (ground::AtomId a : members[i]) {
      by_atom[a].push_back(i);
    }
  }
  std::vector<std::vector<size_t>> by_size(max_size + 1);
  for (size_t i = 0; i < members.size(); ++i) {
    by_size[size[i]].push_back(i);
  }
  std::vector<bool> taken(g.atoms.size(), false);
  std::vector<std::vector<ground::AtomId>> out;
  while (max_size > 1) {
    auto & list = by_size[max_size];
    if (list.empty()) {
      --max_size;
      continue;
    }
    size_t top = list.back();
    list.pop_back();
    if (size[top] != max_size) {
      by_size[size[top]].push_back(top);  // shrank since it was filed
      continue;
    }
    std::vector<ground::AtomId> chosen;
    for (ground::AtomId a : members[top]) {
      if (!taken[a]) {
        chosen.push_back(a);
      }
    }
    for (ground::AtomId a : chosen) {
      taken[a] = true;
      for (size_t i : by_atom[a]) {
        --size[i];
      }
    }
    out.push_back(std::move(chosen));
  }
  return out;
}

std::string to_string(const lifted::Task & task, const Invariant & invariant)
{
  std::string out = "{";
  for (size_t i = 0; i < invariant.parts.size(); ++i) {
    const auto & p = invariant.parts[i];
    out += (i ? ", " : "") + task.predicate_names.at(p.predicate.index()) + "(";
    for (size_t j = 0; j < p.args.size(); ++j) {
      out += (j ? " " : "") + (p.args[j] == kCounted ? std::string("*") :
        "?v" + std::to_string(p.args[j]));
    }
    out += ")";
  }
  return out + "}";
}

}  // namespace kairos::sas
