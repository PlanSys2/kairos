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

#include "lift/lift.hpp"

#include <algorithm>
#include <bit>  // NOLINT(build/include_order): cpplint does not know this header
#include <cassert>
#include <cstring>
#include <limits>
#include <queue>
#include <set>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace kairos::lift
{

namespace
{

using Tuple = std::vector<uint32_t>;

// An argument: a variable (>= 0) or an object (-(object + 1))
using Arg = int32_t;

Arg arg_of(const lifted::Term & t)
{
  return t.kind == lifted::Term::Kind::Variable ? static_cast<Arg>(t.index) :
         -static_cast<Arg>(t.index) - 1;
}

struct Pattern
{
  uint32_t pred = 0;
  std::vector<Arg> args;
};

struct Compare
{
  Arg a;
  Arg b;
  bool equal;
};

struct Schema
{
  std::string name;
  uint32_t vars = 0;
  uint32_t params = 0;
  std::vector<std::vector<uint32_t>> domain;  // objects per variable
  std::vector<std::vector<uint8_t>> in_domain;  // per variable, per object: in its domain
  std::vector<Pattern> pos;                   // static and fluent, joined in this order
  std::vector<Pattern> neg;
  std::vector<Compare> compare;
  std::vector<Pattern> add;
  std::vector<Pattern> del;
};

// The true atoms of each predicate: tuples of its arity, sorted, flat (arity 0: {0} if true)
struct Relation
{
  uint32_t arity = 0;
  std::vector<uint32_t> data;

  size_t size() const {return arity == 0 ? data.size() : data.size() / arity;}
  const uint32_t * at(size_t i) const {return data.data() + i * std::max(arity, 1u);}

  bool contains(const uint32_t * t) const
  {
    uint32_t w = std::max(arity, 1u);
    size_t lo = 0, hi = size();
    while (lo < hi) {
      size_t mid = (lo + hi) / 2;
      int c = std::memcmp(at(mid), t, w * sizeof(uint32_t));
      if (c == 0) {
        return true;
      }
      if (c < 0) {
        lo = mid + 1;
      } else {
        hi = mid;
      }
    }
    return false;
  }
  // Inserts or erases a tuple, keeping the order
  void insert(const uint32_t * t)
  {
    uint32_t w = std::max(arity, 1u);
    size_t lo = 0, hi = size();
    while (lo < hi) {
      size_t mid = (lo + hi) / 2;
      int c = std::memcmp(at(mid), t, w * sizeof(uint32_t));
      if (c == 0) {
        return;
      }
      if (c < 0) {
        lo = mid + 1;
      } else {
        hi = mid;
      }
    }
    data.insert(data.begin() + static_cast<std::ptrdiff_t>(lo * w), t, t + w);
  }
  void erase(const uint32_t * t)
  {
    uint32_t w = std::max(arity, 1u);
    size_t lo = 0, hi = size();
    while (lo < hi) {
      size_t mid = (lo + hi) / 2;
      int c = std::memcmp(at(mid), t, w * sizeof(uint32_t));
      if (c == 0) {
        auto from = data.begin() + static_cast<std::ptrdiff_t>(mid * w);
        data.erase(from, from + w);
        return;
      }
      if (c < 0) {
        lo = mid + 1;
      } else {
        hi = mid;
      }
    }
  }
};

// Open addressing from 64-bit keys to 32-bit values; clear() is O(1) (a generation stamp)
class FlatMap
{
public:
  void clear()
  {
    n_ = 0;
    if (++gen_ == 0) {  // wrapped: stamps of old generations could look current
      std::fill(stamp_.begin(), stamp_.end(), 0);
      gen_ = 1;
    }
  }

  const uint32_t * find(uint64_t k) const
  {
    if (keys_.empty()) {
      return nullptr;
    }
    for (size_t i = slot(k); ; i = (i + 1) & mask_) {
      if (stamp_[i] != gen_) {
        return nullptr;
      }
      if (keys_[i] == k) {
        return &vals_[i];
      }
    }
  }

  bool contains(uint64_t k) const {return find(k) != nullptr;}

  // False if the key was there already (its value stays)
  bool insert(uint64_t k, uint32_t v)
  {
    if ((n_ + 1) * 2 > keys_.size()) {
      grow();
    }
    for (size_t i = slot(k); ; i = (i + 1) & mask_) {
      if (stamp_[i] != gen_) {
        stamp_[i] = gen_;
        keys_[i] = k;
        vals_[i] = v;
        ++n_;
        return true;
      }
      if (keys_[i] == k) {
        return false;
      }
    }
  }

private:
  size_t slot(uint64_t k) const
  {
    return static_cast<size_t>((k * 0x9e3779b97f4a7c15ULL) >> 20) & mask_;
  }

  void grow()
  {
    std::vector<uint64_t> keys;
    std::vector<uint32_t> vals;
    std::vector<uint32_t> stamp;
    keys.swap(keys_);
    vals.swap(vals_);
    stamp.swap(stamp_);
    size_t cap = std::max<size_t>(64, keys.size() * 2);
    keys_.assign(cap, 0);
    vals_.assign(cap, 0);
    stamp_.assign(cap, 0);
    mask_ = cap - 1;
    uint32_t old = gen_;
    gen_ = 1;
    n_ = 0;
    for (size_t i = 0; i < keys.size(); ++i) {
      if (stamp[i] == old) {
        insert(keys[i], vals[i]);
      }
    }
  }

  std::vector<uint64_t> keys_;
  std::vector<uint32_t> vals_;
  std::vector<uint32_t> stamp_;
  size_t mask_ = 0;
  size_t n_ = 0;
  uint32_t gen_ = 1;
};

using State = std::vector<Relation>;  // per fluent predicate (index into Lift::fluent_of)

// The tuples of a relation by the value at one position: (value, tuple) sorted
using Index = std::vector<std::pair<uint32_t, uint32_t>>;

Index index_of(const Relation & r, uint32_t pos)
{
  Index out;
  out.reserve(r.size());
  for (size_t i = 0; i < r.size(); ++i) {
    out.emplace_back(r.at(i)[pos], static_cast<uint32_t>(i));
  }
  std::sort(out.begin(), out.end());
  return out;
}

class Lift
{
public:
  Lift(const lifted::Task & t, Budget * budget, Logger log, Stats & stats)
  : t_(t), budget_(budget), log_(std::move(log)), stats_(stats)
  {
    build();
  }

  ~Lift()
  {
    if (budget_ && accounted_) {
      budget_->account(-static_cast<std::ptrdiff_t>(accounted_));
    }
  }
  Lift(const Lift &) = delete;
  Lift & operator=(const Lift &) = delete;

  std::optional<Plan> run(const std::function<bool()> & stop);
  std::optional<Plan> search(const std::function<bool()> & stop);
  struct OutOfBudget {};  // thrown where a state has too many successors to list

private:
  void build();
  void successors(const State & s, std::vector<std::pair<uint32_t, Tuple>> & out);
  void join(
    const Schema & sc, const State & s, size_t k, Tuple & a,
    std::vector<uint8_t> & bound, uint32_t si, std::vector<std::pair<uint32_t, Tuple>> & out);
  void rest(
    const Schema & sc, const State & s, size_t v, Tuple & a,
    std::vector<uint8_t> & bound, uint32_t si, std::vector<std::pair<uint32_t, Tuple>> & out);
  bool holds(const Pattern & p, const State & s, const Tuple & a) const;
  bool comparisons_ok(const Schema & sc, const Tuple & a, const std::vector<uint8_t> & bound) const;
  State apply(const State & s, const Schema & sc, const Tuple & a) const;
  uint32_t goal_count(const State & s) const;
  // FF on the delete relaxation, without grounding: layers of facts from the state, each from
  // the action instances with a precondition fact of the layer before; stops at the goal.
  // UINT32_MAX if the goal cannot be reached
  // Actions of the relaxed plan applicable in the state (preferred), as (schema, arguments)
  using Preferred = std::vector<std::pair<uint32_t, Tuple>>;
  uint32_t ff(const State & s, Preferred * preferred = nullptr);
  uint32_t ff_or_throw(const State & s, Preferred * preferred = nullptr);
  struct TooLarge {};
  // Scratch of ff(), kept between calls (cleared in O(1))
  std::vector<FlatMap> ff_info_;
  FlatMap ff_memo_;
  static constexpr size_t kFFCap = 3000000;  // facts a relaxed exploration may reach
  static constexpr uint64_t kFFWork = 200000000;  // tuples it may visit

  const lifted::Task & t_;
  Budget * budget_;
  Logger log_;
  Stats & stats_;
  std::vector<Schema> schemas_;
  std::vector<int32_t> fluent_of_;      // per predicate: index in State, or -1 (static)
  std::vector<Relation> static_;        // per predicate (static ones)
  State init_;
  std::vector<std::pair<Pattern, bool>> goal_;  // ground patterns, positive or not
  size_t accounted_ = 0;  // bytes told to the budget
  bool use_ff_ = true;
  // Indexes by position: of the static relations (built once), of the fluent ones of the state
  // being expanded (built when first used, per expansion)
  std::vector<std::vector<std::optional<Index>>> static_index_;
  std::vector<std::vector<Index>> fluent_index_;
  std::vector<std::vector<uint32_t>> fluent_stamp_;
  uint32_t expansion_ = 0;
  const Index & index(uint32_t pred, uint32_t pos, const State & s);
};

void Lift::build()
{
  size_t np = t_.predicate_names.size();
  std::vector<uint32_t> arity(np, 0);
  for (size_t p = 0; p < np; ++p) {
    arity[p] = static_cast<uint32_t>(t_.predicate_params[p].size());
  }
  std::vector<uint8_t> changes(np, 0);
  for (const auto & a : t_.actions) {
    for (const auto & e : a.effects) {
      if (e.kind == lifted::Effect::Kind::Add || e.kind == lifted::Effect::Kind::Delete) {
        changes[e.atom.predicate.value()] = 1;
      }
    }
  }
  fluent_of_.assign(np, -1);
  static_.resize(np);
  for (size_t p = 0; p < np; ++p) {
    static_[p].arity = arity[p];
    if (changes[p]) {
      fluent_of_[p] = static_cast<int32_t>(init_.size());
      init_.push_back(Relation{arity[p], {}});
    }
  }
  for (const auto & [pred, args] : t_.init_facts) {
    Tuple tup;
    for (auto o : args) {
      tup.push_back(o.value());
    }
    if (tup.empty()) {
      tup.push_back(0);
    }
    int32_t f = fluent_of_[pred.value()];
    (f >= 0 ? init_[static_cast<size_t>(f)] : static_[pred.value()]).insert(tup.data());
  }
  // Objects per type
  size_t no = t_.object_names.size();
  for (const auto & a : t_.actions) {
    Schema sc;
    sc.name = a.name;
    sc.vars = static_cast<uint32_t>(a.variables.size());
    sc.params = a.parameters;
    for (const auto & v : a.variables) {
      std::vector<uint32_t> objs;
      for (uint32_t o = 0; o < no; ++o) {
        if (t_.types.fits(t_.object_types[o], v.types)) {
          objs.push_back(o);
        }
      }
      std::vector<uint8_t> in(no, 0);
      for (uint32_t o : objs) {
        in[o] = 1;
      }
      sc.in_domain.push_back(std::move(in));
      sc.domain.push_back(std::move(objs));
    }
    auto pattern = [&](const lifted::Atom & at) {
        Pattern p;
        p.pred = at.predicate.value();
        for (const auto & term : at.args) {
          p.args.push_back(arg_of(term));
        }
        return p;
      };
    for (const auto & l : a.precondition.literals) {
      (l.positive ? sc.pos : sc.neg).push_back(pattern(l.atom));
    }
    for (const auto & c : a.precondition.comparisons) {
      sc.compare.push_back({arg_of(c.lhs_term), arg_of(c.rhs_term), c.op == lifted::CompareOp::Eq});
    }
    for (const auto & e : a.effects) {
      if (e.kind == lifted::Effect::Kind::Add) {
        sc.add.push_back(pattern(e.atom));
      } else if (e.kind == lifted::Effect::Kind::Delete) {
        sc.del.push_back(pattern(e.atom));
      }
    }
    // Join order: next, the atom with the most arguments already known (constants and
    // variables of the atoms before it); then static ones, then the smaller
    {
      std::vector<Pattern> left = std::move(sc.pos);
      std::vector<uint8_t> known(sc.vars, 0);
      sc.pos.clear();
      while (!left.empty()) {
        size_t best = 0;
        auto score = [&](const Pattern & q) {
            int bound_args = 0;
            for (Arg x : q.args) {
              bound_args += x < 0 || known[static_cast<size_t>(x)];
            }
            bool st = fluent_of_[q.pred] < 0;
            size_t size = st ? static_[q.pred].size() : 1000000;
            return std::make_tuple(bound_args > 0 ? 1 : 0, bound_args, st ? 1 : 0,
                     -static_cast<int64_t>(size));
          };
        for (size_t i = 1; i < left.size(); ++i) {
          if (score(left[i]) > score(left[best])) {
            best = i;
          }
        }
        for (Arg x : left[best].args) {
          if (x >= 0) {
            known[static_cast<size_t>(x)] = 1;
          }
        }
        sc.pos.push_back(std::move(left[best]));
        left.erase(left.begin() + static_cast<std::ptrdiff_t>(best));
      }
    }
    schemas_.push_back(std::move(sc));
  }
  static_index_.resize(np);
  for (size_t q = 0; q < np; ++q) {
    static_index_[q].resize(std::max(arity[q], 1u));
  }
  fluent_index_.resize(init_.size());
  fluent_stamp_.resize(init_.size());
  for (size_t f = 0; f < init_.size(); ++f) {
    fluent_index_[f].resize(std::max(init_[f].arity, 1u));
    fluent_stamp_[f].assign(std::max(init_[f].arity, 1u), 0);
  }
  for (const auto & l : t_.goal.literals) {
    Pattern p;
    p.pred = l.atom.predicate.value();
    for (const auto & term : l.atom.args) {
      p.args.push_back(arg_of(term));
    }
    goal_.emplace_back(std::move(p), l.positive);
  }
}

const Index & Lift::index(uint32_t pred, uint32_t pos, const State & s)
{
  int32_t f = fluent_of_[pred];
  if (f < 0) {
    auto & slot = static_index_[pred][pos];
    if (!slot) {
      slot = index_of(static_[pred], pos);
    }
    return *slot;
  }
  auto fi = static_cast<size_t>(f);
  if (fluent_stamp_[fi][pos] != expansion_) {
    fluent_stamp_[fi][pos] = expansion_;
    fluent_index_[fi][pos] = index_of(s[fi], pos);
  }
  return fluent_index_[fi][pos];
}

bool Lift::holds(const Pattern & p, const State & s, const Tuple & a) const
{
  thread_local Tuple tup;
  tup.clear();
  for (Arg x : p.args) {
    tup.push_back(x >= 0 ? a[static_cast<size_t>(x)] : static_cast<uint32_t>(-x - 1));
  }
  if (tup.empty()) {
    tup.push_back(0);
  }
  int32_t f = fluent_of_[p.pred];
  return (f >= 0 ? s[static_cast<size_t>(f)] : static_[p.pred]).contains(tup.data());
}

bool Lift::comparisons_ok(
  const Schema & sc, const Tuple & a, const std::vector<uint8_t> & bound) const
{
  for (const auto & c : sc.compare) {
    bool ba = c.a < 0 || bound[static_cast<size_t>(c.a)];
    bool bb = c.b < 0 || bound[static_cast<size_t>(c.b)];
    if (!ba || !bb) {
      continue;
    }
    uint32_t va = c.a >= 0 ? a[static_cast<size_t>(c.a)] : static_cast<uint32_t>(-c.a - 1);
    uint32_t vb = c.b >= 0 ? a[static_cast<size_t>(c.b)] : static_cast<uint32_t>(-c.b - 1);
    if ((va == vb) != c.equal) {
      return false;
    }
  }
  return true;
}

void Lift::join(
  const Schema & sc, const State & s, size_t k, Tuple & a, std::vector<uint8_t> & bound,
  uint32_t si, std::vector<std::pair<uint32_t, Tuple>> & out)
{
  if (k == sc.pos.size()) {
    rest(sc, s, 0, a, bound, si, out);
    return;
  }
  const Pattern & p = sc.pos[k];
  int32_t f = fluent_of_[p.pred];
  const Relation & r = f >= 0 ? s[static_cast<size_t>(f)] : static_[p.pred];
  size_t n = r.size();
  uint32_t w = r.arity;
  // A position already known: only the tuples with that value there
  const std::pair<uint32_t, uint32_t> * from = nullptr;
  const std::pair<uint32_t, uint32_t> * to = nullptr;
  for (uint32_t j = 0; j < w; ++j) {
    Arg x = p.args[j];
    if (x < 0 || bound[static_cast<size_t>(x)]) {
      uint32_t v = x < 0 ? static_cast<uint32_t>(-x - 1) : a[static_cast<size_t>(x)];
      const Index & idx = index(p.pred, j, s);
      auto range = std::equal_range(
        idx.begin(), idx.end(), std::make_pair(v, 0u),
        [](const auto & l, const auto & r2) {return l.first < r2.first;});
      from = idx.data() + (range.first - idx.begin());
      to = idx.data() + (range.second - idx.begin());
      n = static_cast<size_t>(to - from);
      break;
    }
  }
  std::vector<size_t> newly;
  for (size_t i = 0; i < n; ++i) {
    const uint32_t * tup = r.at(from ? from[i].second : i);
    bool ok = true;
    newly.clear();
    for (uint32_t j = 0; j < w && ok; ++j) {
      Arg x = p.args[j];
      if (x < 0) {
        ok = tup[j] == static_cast<uint32_t>(-x - 1);
      } else if (bound[static_cast<size_t>(x)]) {
        ok = a[static_cast<size_t>(x)] == tup[j];
      } else {
        // Its type: the object must be in the variable's domain
        const auto & dom = sc.domain[static_cast<size_t>(x)];
        ok = std::binary_search(dom.begin(), dom.end(), tup[j]);
        if (ok) {
          a[static_cast<size_t>(x)] = tup[j];
          bound[static_cast<size_t>(x)] = 1;
          newly.push_back(static_cast<size_t>(x));
        }
      }
    }
    if (ok && comparisons_ok(sc, a, bound)) {
      join(sc, s, k + 1, a, bound, si, out);
    }
    for (size_t x : newly) {
      bound[x] = 0;
    }
  }
}

void Lift::rest(
  const Schema & sc, const State & s, size_t v, Tuple & a, std::vector<uint8_t> & bound,
  uint32_t si, std::vector<std::pair<uint32_t, Tuple>> & out)
{
  while (v < sc.vars && bound[v]) {
    ++v;
  }
  if (v == sc.vars) {
    for (const auto & p : sc.neg) {
      if (holds(p, s, a)) {
        return;
      }
    }
    // Before the list doubles: room for it (an action without preconditions and six
    // parameters over 40 objects has 4 billion instances)
    if (out.size() == out.capacity() && budget_ &&
      (budget_->exhausted() ||
      !budget_->reserve(out.capacity() * (sizeof(out[0]) + sc.vars * sizeof(uint32_t)))))
    {
      throw OutOfBudget{};
    }
    out.emplace_back(si, a);
    return;
  }
  for (uint32_t o : sc.domain[v]) {
    a[v] = o;
    bound[v] = 1;
    if (comparisons_ok(sc, a, bound)) {
      rest(sc, s, v + 1, a, bound, si, out);
    }
    bound[v] = 0;
  }
}

void Lift::successors(const State & s, std::vector<std::pair<uint32_t, Tuple>> & out)
{
  out.clear();
  for (uint32_t si = 0; si < schemas_.size(); ++si) {
    const Schema & sc = schemas_[si];
    Tuple a(sc.vars, 0);
    std::vector<uint8_t> bound(sc.vars, 0);
    join(sc, s, 0, a, bound, si, out);
  }
}

State Lift::apply(const State & s, const Schema & sc, const Tuple & a) const
{
  State n = s;
  Tuple tup;
  auto ground = [&](const Pattern & p) {
      tup.clear();
      for (Arg x : p.args) {
        tup.push_back(x >= 0 ? a[static_cast<size_t>(x)] : static_cast<uint32_t>(-x - 1));
      }
      if (tup.empty()) {
        tup.push_back(0);
      }
    };
  for (const auto & p : sc.del) {
    ground(p);
    n[static_cast<size_t>(fluent_of_[p.pred])].erase(tup.data());
  }
  for (const auto & p : sc.add) {
    ground(p);
    n[static_cast<size_t>(fluent_of_[p.pred])].insert(tup.data());
  }
  return n;
}

uint32_t Lift::ff(const State & s, Preferred * preferred)
{
  try {
    return ff_or_throw(s, preferred);
  } catch (const TooLarge &) {
    return goal_count(s);  // too much to explore from here: the goal count
  }
}

uint32_t Lift::ff_or_throw(const State & s, Preferred * preferred)
{
  size_t nf = s.size();
  size_t facts = 0;
  uint64_t work = 0;    // tuples visited
  uint64_t checks = 0;
  // Reached facts per fluent predicate (tuples appended), with level and supporter
  struct Facts
  {
    uint32_t arity = 0;
    std::vector<uint32_t> data;
    size_t size() const {return data.size() / std::max(arity, 1u);}
    const uint32_t * at(size_t i) const {return data.data() + i * std::max(arity, 1u);}
  };
  struct Info
  {
    uint32_t level;
    uint32_t schema;  // supporter, UINT32_MAX at level 0
    uint32_t args;    // offset into supports
  };
  std::vector<Facts> all(nf);
  // Per fluent predicate: fact key -> index into infos
  auto & info = ff_info_;
  info.resize(nf);
  for (auto & m : info) {
    m.clear();
  }
  std::vector<Info> infos;
  auto info_of = [&](size_t f, uint64_t key) -> const Info * {
      const uint32_t * i = info[f].find(key);
      return i ? &infos[*i] : nullptr;
    };
  auto add_info = [&](size_t f, uint64_t key, const Info & in) {
      if (!info[f].insert(key, static_cast<uint32_t>(infos.size()))) {
        return false;
      }
      infos.push_back(in);
      return true;
    };
  std::vector<uint32_t> supports;
  auto key_of = [](const uint32_t * tup, uint32_t w) {
      uint64_t h = 1469598103934665603ULL;
      for (uint32_t j = 0; j < w; ++j) {
        h = (h ^ tup[j]) * 1099511628211ULL;
        h ^= h >> 29;
      }
      return h;
    };
  for (size_t f = 0; f < nf; ++f) {
    all[f].arity = s[f].arity;
    all[f].data = s[f].data;
    uint32_t w = std::max(s[f].arity, 1u);
    for (size_t i = 0; i < s[f].size(); ++i) {
      add_info(f, key_of(s[f].at(i), w), Info{0, UINT32_MAX, 0});
    }
  }
  // The positive goal facts still missing
  std::vector<std::pair<uint32_t, Tuple>> goals;
  for (const auto & [gp, positive] : goal_) {
    if (!positive) {
      continue;  // the relaxation ignores what must be false
    }
    Tuple g;
    for (Arg x : gp.args) {
      g.push_back(static_cast<uint32_t>(-x - 1));
    }
    if (g.empty()) {
      g.push_back(0);
    }
    int32_t f = fluent_of_[gp.pred];
    if (f < 0) {
      if (!static_[gp.pred].contains(g.data())) {
        return UINT32_MAX;
      }
      continue;
    }
    goals.emplace_back(static_cast<uint32_t>(f), std::move(g));
  }
  auto reached = [&](uint32_t f, const Tuple & g) {
      return info[f].contains(key_of(g.data(), static_cast<uint32_t>(g.size())));
    };
  auto all_goals = [&]() {
      return std::all_of(goals.begin(), goals.end(), [&](const auto & g) {
                 return reached(g.first, g.second);
             });
    };
  // Indexes of the reached facts by position, rebuilt each layer
  std::vector<std::vector<Index>> idx(nf);
  std::vector<std::vector<uint32_t>> idx_stamp(nf);
  for (size_t f = 0; f < nf; ++f) {
    idx[f].resize(std::max(s[f].arity, 1u));
    idx_stamp[f].assign(std::max(s[f].arity, 1u), UINT32_MAX);
  }
  uint32_t layer = 0;
  // New facts of the last layer: [from[f], all[f].size())
  std::vector<size_t> from(nf, 0);
  std::vector<std::vector<uint32_t>> pending(nf);  // facts of the next layer
  size_t found = 0;
  Tuple fact;
  while (!all_goals()) {
    if (budget_ && budget_->exhausted()) {
      return UINT32_MAX;
    }
    found = 0;
    std::vector<size_t> upto(nf);
    for (size_t f = 0; f < nf; ++f) {
      upto[f] = all[f].size();
    }
    for (uint32_t si = 0; si < schemas_.size(); ++si) {
      const Schema & sc = schemas_[si];
      // Projection: two partial bindings that agree on the variables still read (by the atoms
      // left, the effects and the comparisons) reach the same facts; the second is skipped
      bool project = sc.vars <= 64;
      std::vector<uint64_t> atom_mask(sc.pos.size(), 0);
      uint64_t fixed_mask = 0;
      if (project) {
        auto mask_of = [](const Pattern & q) {
            uint64_t m = 0;
            for (Arg x : q.args) {
              m |= x >= 0 ? uint64_t{1} << x : 0;
            }
            return m;
          };
        for (size_t q = 0; q < sc.pos.size(); ++q) {
          atom_mask[q] = mask_of(sc.pos[q]);
        }
        for (const auto & ad : sc.add) {
          fixed_mask |= mask_of(ad);
        }
        for (const auto & c : sc.compare) {
          fixed_mask |= (c.a >= 0 ? uint64_t{1} << c.a : 0) | (c.b >= 0 ? uint64_t{1} << c.b : 0);
        }
      }
      auto & memo = ff_memo_;
      for (size_t d = 0; d < sc.pos.size(); ++d) {
        memo.clear();
        int32_t df = fluent_of_[sc.pos[d].pred];
        if (df < 0 || from[static_cast<size_t>(df)] == upto[static_cast<size_t>(df)]) {
          continue;  // no new fact for this atom
        }
        // Join: atom d (first) over the new facts, the others over everything reached, in a
        // fixed order: next the atom with most positions known (constants, bound variables),
        // the smaller relation on ties
        std::vector<size_t> ord{d};
        {
          std::vector<uint8_t> in_ord(sc.pos.size(), 0);
          std::vector<uint8_t> b(sc.vars, 0);
          auto bind = [&](size_t q) {
              in_ord[q] = 1;
              for (Arg x : sc.pos[q].args) {
                if (x >= 0) {
                  b[static_cast<size_t>(x)] = 1;
                }
              }
            };
          bind(d);
          while (ord.size() < sc.pos.size()) {
            size_t best = SIZE_MAX, best_known = 0, best_size = 0;
            for (size_t q = 0; q < sc.pos.size(); ++q) {
              if (in_ord[q]) {
                continue;
              }
              size_t known = 0;
              for (Arg x : sc.pos[q].args) {
                known += x < 0 || b[static_cast<size_t>(x)];
              }
              int32_t qf = fluent_of_[sc.pos[q].pred];
              size_t size = qf < 0 ? static_[sc.pos[q].pred].size() : upto[static_cast<size_t>(qf)];
              if (best == SIZE_MAX || known > best_known ||
                (known == best_known && size < best_size))
              {
                best = q;
                best_known = known;
                best_size = size;
              }
            }
            ord.push_back(best);
            bind(best);
          }
        }
        // Before depth k: the variables bound (have) and those read from there on (need)
        std::vector<uint64_t> have(ord.size() + 1, 0), need(ord.size() + 1, fixed_mask);
        if (project) {
          for (size_t k = 0; k < ord.size(); ++k) {
            have[k + 1] = have[k] | atom_mask[ord[k]];
          }
          for (size_t k = ord.size(); k-- > 0; ) {
            need[k] = need[k + 1] | atom_mask[ord[k]];
          }
        }
        Tuple a(sc.vars, 0);
        std::vector<uint8_t> bound(sc.vars, 0);
        std::vector<std::vector<uint32_t>> bound_at(ord.size());  // bound at each depth
        std::function<void(size_t)> step = [&](size_t k) {
            if (k == sc.pos.size()) {
              // Variables only in effects: every object of their type
              std::function<void(uint32_t)> fill = [&](uint32_t v) {
                  while (v < sc.vars && bound[v]) {
                    ++v;
                  }
                  if (v == sc.vars) {
                    for (const auto & ad : sc.add) {
                      fact.clear();
                      for (Arg x : ad.args) {
                        fact.push_back(x >= 0 ? a[static_cast<size_t>(x)] :
                          static_cast<uint32_t>(-x - 1));
                      }
                      if (fact.empty()) {
                        fact.push_back(0);
                      }
                      auto f = static_cast<uint32_t>(fluent_of_[ad.pred]);
                      uint64_t key = key_of(fact.data(), static_cast<uint32_t>(fact.size()));
                      auto off = static_cast<uint32_t>(supports.size());
                      if (add_info(f, key, Info{layer + 1, si, off})) {
                        supports.insert(supports.end(), a.begin(), a.end());
                        pending[f].insert(pending[f].end(), fact.begin(), fact.end());
                        ++found;
                        if (++facts > kFFCap) {
                          throw TooLarge{};
                        }
                      }
                    }
                    if (++work > kFFWork) {
                      throw TooLarge{};
                    }
                    return;
                  }
                  for (uint32_t o : sc.domain[v]) {
                    a[v] = o;
                    bound[v] = 1;
                    if (comparisons_ok(sc, a, bound)) {
                      fill(v + 1);
                    }
                    bound[v] = 0;
                  }
                };
              fill(0);
              return;
            }
            if (project && k > 0 && (have[k] & ~need[k])) {
              // Bound variables nothing reads any more: only the others tell bindings apart
              uint64_t key = 0x9e3779b97f4a7c15ULL + k;
              for (uint64_t m = have[k] & need[k]; m; m &= m - 1) {
                auto v = static_cast<uint32_t>(std::countr_zero(m));
                key = (key ^ (a[v] + 0x632be59bd9b4e019ULL * (v + 1))) * 0xff51afd7ed558ccdULL;
                key ^= key >> 32;
              }
              if (!memo.insert(key, 0)) {
                return;  // the same continuation as a binding before
              }
            }
            // The atom with the fewest candidates next (the new-facts atom first)
            struct Range
            {
              const uint32_t * base;
              uint32_t w;
              size_t lo;
              size_t n;
              const std::pair<uint32_t, uint32_t> * ifrom;
            };
            auto range_of = [&](size_t pk) {
                const Pattern & p = sc.pos[pk];
                int32_t pf = fluent_of_[p.pred];
                Range r{nullptr, 0, 0, 0, nullptr};
                size_t hi;
                if (pf < 0) {
                  r.base = static_[p.pred].data.data();
                  hi = static_[p.pred].size();
                  r.w = static_[p.pred].arity;
                } else {
                  auto fi = static_cast<size_t>(pf);
                  r.base = all[fi].data.data();
                  r.w = all[fi].arity;
                  r.lo = pk == d ? from[fi] : 0;
                  hi = upto[fi];
                }
                r.n = hi - r.lo;
                if (pk == d && pf >= 0) {
                  return r;  // a range already
                }
                for (uint32_t j = 0; j < r.w; ++j) {
                  Arg x = p.args[j];
                  if (x < 0 || bound[static_cast<size_t>(x)]) {
                    uint32_t v = x < 0 ? static_cast<uint32_t>(-x - 1) :
                      a[static_cast<size_t>(x)];
                    const Index * index;
                    if (pf < 0) {
                      index = &this->index(p.pred, j, s);
                    } else {
                      auto fi = static_cast<size_t>(pf);
                      if (idx_stamp[fi][j] != layer) {
                        idx_stamp[fi][j] = layer;
                        Index built;
                        for (size_t i = 0; i < upto[fi]; ++i) {
                          built.emplace_back(all[fi].at(i)[j], static_cast<uint32_t>(i));
                        }
                        std::sort(built.begin(), built.end());
                        idx[fi][j] = std::move(built);
                      }
                      index = &idx[fi][j];
                    }
                    auto er = std::equal_range(
                      index->begin(), index->end(), std::make_pair(v, 0u),
                      [](const auto & l, const auto & r2) {return l.first < r2.first;});
                    size_t m = static_cast<size_t>(er.second - er.first);
                    if (!r.ifrom || m < r.n) {
                      r.ifrom = index->data() + (er.first - index->begin());
                      r.n = m;
                    }
                  }
                }
                return r;
              };
            size_t pk = ord[k];
            Range rg = range_of(pk);
            work += 1 + rg.n;
            if (work > kFFWork || ((++checks & 255) == 0 && budget_ && budget_->exhausted())) {
              throw TooLarge{};
            }
            const Pattern & p = sc.pos[pk];
            const uint32_t * base = rg.base;
            uint32_t w = rg.w;
            uint32_t ww = std::max(w, 1u);
            size_t lo = rg.lo;
            size_t n = rg.n;
            const std::pair<uint32_t, uint32_t> * ifrom = rg.ifrom;
            auto & newly = bound_at[k];
            for (size_t i = 0; i < n; ++i) {
              size_t row = ifrom ? ifrom[i].second : lo + i;
              const uint32_t * tup = base + row * ww;
              bool ok = true;
              newly.clear();
              for (uint32_t j = 0; j < w && ok; ++j) {
                Arg x = p.args[j];
                if (x < 0) {
                  ok = tup[j] == static_cast<uint32_t>(-x - 1);
                } else if (bound[static_cast<size_t>(x)]) {
                  ok = a[static_cast<size_t>(x)] == tup[j];
                } else {
                  ok = sc.in_domain[static_cast<size_t>(x)][tup[j]] != 0;
                  if (ok) {
                    a[static_cast<size_t>(x)] = tup[j];
                    bound[static_cast<size_t>(x)] = 1;
                    newly.push_back(static_cast<uint32_t>(x));
                  }
                }
              }
              if (ok && comparisons_ok(sc, a, bound)) {
                step(k + 1);
              }
              for (uint32_t x : newly) {
                bound[x] = 0;
              }
            }
          };
        step(0);
      }
      // Actions without fluent preconditions: their effects once, at layer 0
      if (layer == 0) {
        bool fluent = std::any_of(sc.pos.begin(), sc.pos.end(), [&](const Pattern & q) {
              return fluent_of_[q.pred] >= 0;
            });
        if (!fluent) {
          Tuple a(sc.vars, 0);
          std::vector<uint8_t> bound(sc.vars, 0);
          // reuse the successor join over the static relations
          std::vector<std::pair<uint32_t, Tuple>> inst;
          join(sc, s, 0, a, bound, si, inst);
          for (auto & [unused, args] : inst) {
            for (const auto & ad : sc.add) {
              fact.clear();
              for (Arg x : ad.args) {
                fact.push_back(x >= 0 ? args[static_cast<size_t>(x)] :
                  static_cast<uint32_t>(-x - 1));
              }
              if (fact.empty()) {
                fact.push_back(0);
              }
              auto f = static_cast<uint32_t>(fluent_of_[ad.pred]);
              uint64_t key = key_of(fact.data(), static_cast<uint32_t>(fact.size()));
              auto off = static_cast<uint32_t>(supports.size());
              if (add_info(f, key, Info{1, si, off})) {
                supports.insert(supports.end(), args.begin(), args.end());
                pending[f].insert(pending[f].end(), fact.begin(), fact.end());
                ++found;
              }
            }
          }
        }
      }
    }
    if (found == 0) {
      return UINT32_MAX;  // nothing new: the goal is out of reach
    }
    for (size_t f = 0; f < nf; ++f) {
      from[f] = upto[f];
      all[f].data.insert(all[f].data.end(), pending[f].begin(), pending[f].end());
      pending[f].clear();
    }
    ++layer;
  }
  // The relaxed plan: back from the goal through the supporters
  std::set<std::pair<uint32_t, uint32_t>> plan;  // (schema, args offset)
  std::vector<std::pair<uint32_t, Tuple>> open = goals;
  while (!open.empty()) {
    auto [f, g] = std::move(open.back());
    open.pop_back();
    const Info * it = info_of(f, key_of(g.data(), static_cast<uint32_t>(g.size())));
    if (!it || it->level == 0) {
      continue;
    }
    if (!plan.insert({it->schema, it->args}).second) {
      continue;
    }
    const Schema & sc = schemas_[it->schema];
    const uint32_t * args = supports.data() + it->args;
    bool applicable = true;
    for (const auto & p : sc.pos) {
      int32_t pf = fluent_of_[p.pred];
      if (pf < 0) {
        continue;
      }
      Tuple pre;
      for (Arg x : p.args) {
        pre.push_back(x >= 0 ? args[x] : static_cast<uint32_t>(-x - 1));
      }
      if (pre.empty()) {
        pre.push_back(0);
      }
      const Info * pi = info_of(static_cast<size_t>(pf),
          key_of(pre.data(), static_cast<uint32_t>(pre.size())));
      applicable = applicable && pi && pi->level == 0;
      open.emplace_back(static_cast<uint32_t>(pf), std::move(pre));
    }
    if (preferred && applicable) {
      preferred->emplace_back(it->schema, Tuple(args, args + sc.vars));
    }
  }
  return static_cast<uint32_t>(plan.size());
}

uint32_t Lift::goal_count(const State & s) const
{
  uint32_t missing = 0;
  Tuple none;
  for (const auto & [p, positive] : goal_) {
    if (holds(p, s, none) != positive) {
      ++missing;
    }
  }
  return missing;
}

std::optional<Plan> Lift::run(const std::function<bool()> & stop)
{
  try {
    return search(stop);
  } catch (const OutOfBudget &) {
    return std::nullopt;  // the budget says why
  }
}

std::optional<Plan> Lift::search(const std::function<bool()> & stop)
{
  struct Node
  {
    uint32_t parent;
    uint32_t schema;
    Tuple args;
    uint32_t h;      // goal count
    uint32_t ff;     // FF, or the parent's until expanded (lazy)
    bool evaluated;
  };
  std::vector<State> states;
  std::vector<Node> nodes;
  // A state is the XOR of two independent hashes of its atoms (128 bits): duplicates are
  // found from the parent's and what the action changes, without building the state
  auto mix = [](uint64_t x) {
      x ^= x >> 33;
      x *= 0xff51afd7ed558ccdULL;
      x ^= x >> 33;
      x *= 0xc4ceb9fe1a85ec53ULL;
      x ^= x >> 33;
      return x;
    };
  auto atom_hash = [&](uint32_t f, const uint32_t * tup, uint32_t w, uint64_t seed) {
      uint64_t h = mix(seed ^ (static_cast<uint64_t>(f) + 0x9e3779b97f4a7c15ULL));
      for (uint32_t j = 0; j < w; ++j) {
        h = mix(h ^ (tup[j] + 0x632be59bd9b4e019ULL));
      }
      return h;
    };
  struct Key
  {
    uint64_t a;
    uint64_t b;
    bool operator==(const Key &) const = default;
  };
  struct KeyHash
  {
    size_t operator()(const Key & k) const noexcept {return static_cast<size_t>(k.a);}
  };
  std::unordered_set<Key, KeyHash> seen;
  std::vector<Key> keys;
  auto key_of = [&](const State & s) {
      Key k{0, 0};
      for (size_t f = 0; f < s.size(); ++f) {
        uint32_t w = std::max(s[f].arity, 1u);
        for (size_t i = 0; i < s[f].size(); ++i) {
          k.a ^= atom_hash(static_cast<uint32_t>(f), s[f].at(i), w, 1);
          k.b ^= atom_hash(static_cast<uint32_t>(f), s[f].at(i), w, 2);
        }
      }
      return k;
    };
  // Novelty of width 1 per goal count, of the atoms each action adds (as Powerlifted)
  std::vector<std::unordered_set<uint64_t>> novel;
  auto atom_key = [](uint32_t pred, const uint32_t * tup, uint32_t w) {
      uint64_t h = (0x9e3779b97f4a7c15ULL + pred) * 0xff51afd7ed558ccdULL;
      for (uint32_t j = 0; j < w; ++j) {
        h = (h ^ tup[j]) * 1099511628211ULL;
        h ^= h >> 29;
      }
      return h;
    };
  // The goal atoms of fluent predicates: how many goal literals want each true and false
  std::unordered_map<uint64_t, std::pair<int32_t, int32_t>> goal_atoms;
  for (const auto & [gp, positive] : goal_) {
    int32_t f = fluent_of_[gp.pred];
    if (f < 0) {
      continue;
    }
    Tuple g;
    for (Arg x : gp.args) {
      g.push_back(static_cast<uint32_t>(-x - 1));
    }
    if (g.empty()) {
      g.push_back(0);
    }
    auto & c = goal_atoms[atom_key(static_cast<uint32_t>(f), g.data(),
        static_cast<uint32_t>(g.size()))];
    (positive ? c.first : c.second) += 1;
  }
  // The goal count changes by what an action makes true or false
  auto goal_delta = [&](uint32_t f, const Tuple & g, bool added) {
      auto it = goal_atoms.find(atom_key(f, g.data(), static_cast<uint32_t>(g.size())));
      if (it == goal_atoms.end()) {
        return 0;
      }
      int32_t d = it->second.second - it->second.first;
      return added ? d : -d;
    };
  auto note = [&](uint32_t h, uint64_t key) {
      if (novel.size() <= h) {
        novel.resize(h + 1);
      }
      return novel[h].insert(key).second;
    };
  using Entry = std::pair<uint64_t, uint32_t>;  // key, node
  std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> by_novelty, by_h, by_pref;
  uint64_t order = 0;
  // By novelty and goal count; and by FF (the parent's: evaluated lazily, on expansion)
  auto push = [&](uint32_t id, uint32_t h, uint32_t w, uint32_t f) {
      ++order;
      by_novelty.emplace((static_cast<uint64_t>(w) << 56) | (static_cast<uint64_t>(h) << 32) |
        (order & 0xffffffffULL), id);
      by_h.emplace((static_cast<uint64_t>(std::min(f, 0xffffffu)) << 32) |
        (order & 0xffffffffULL), id);
    };
  states.push_back(init_);
  keys.push_back(key_of(init_));
  seen.insert(keys.back());
  uint32_t h0 = goal_count(init_);
  uint32_t f0 = h0;
  Preferred preferred;
  if (use_ff_ && h0 > 0) {
    try {
      f0 = ff_or_throw(init_, &preferred);
    } catch (const TooLarge &) {
      use_ff_ = false;  // too large to explore relaxed from the start: the goal count alone
      log_.info(LogChannel::Search, "lifted: relaxed exploration too large, goal count only");
    }
  }
  if (f0 == UINT32_MAX) {
    return std::nullopt;  // the goal is out of reach even relaxed
  }
  nodes.push_back({UINT32_MAX, 0, {}, h0, f0, true});
  for (size_t f = 0; f < init_.size(); ++f) {
    for (size_t i = 0; i < init_[f].size(); ++i) {
      note(h0, atom_key(static_cast<uint32_t>(f), init_[f].at(i), std::max(init_[f].arity, 1u)));
    }
  }
  push(0, h0, 1, f0);
  stats_.states = 1;
  std::vector<uint8_t> closed(1, 0);
  std::vector<std::pair<uint32_t, Tuple>> succ;
  uint32_t turn = 0;
  uint32_t best = h0;
  uint32_t best_ff = f0;
  // Expansions from the preferred queue first after each better FF (as Fast Downward's boost)
  uint32_t boost = 0;
  constexpr uint32_t kBoost = 1000;
  // The preferred actions of the state being expanded, by hash
  std::unordered_set<uint64_t> prefer;
  auto action_key = [](uint32_t si, const Tuple & args) {
      uint64_t h = (0xcbf29ce484222325ULL + si) * 0xff51afd7ed558ccdULL;
      for (uint32_t v : args) {
        h = (h ^ v) * 1099511628211ULL;
        h ^= h >> 31;
      }
      return h;
    };
  Tuple tup;
  while (!by_novelty.empty() || !by_h.empty() || !by_pref.empty()) {
    if ((budget_ && budget_->exhausted()) || (stop && stop())) {
      return std::nullopt;
    }
    decltype(by_h) * q = nullptr;
    if (boost > 0 && !by_pref.empty()) {
      --boost;
      q = &by_pref;
    } else {
      decltype(by_h) * order3[3] = {&by_novelty, &by_h, &by_pref};
      for (uint32_t i = 0; i < 3 && !q; ++i) {
        auto * c = order3[(turn + i) % 3];
        q = c->empty() ? nullptr : c;
      }
      turn = (turn + 1) % 3;
    }
    uint32_t id = q->top().second;
    q->pop();
    if (closed[id]) {
      continue;
    }
    closed[id] = 1;
    if (nodes[id].h == 0) {
      // The plan back from the goal
      std::vector<uint32_t> path;
      for (uint32_t n = id; nodes[n].parent != UINT32_MAX; n = nodes[n].parent) {
        path.push_back(n);
      }
      std::reverse(path.begin(), path.end());
      Plan plan;
      for (uint32_t n : path) {
        const Schema & sc = schemas_[nodes[n].schema];
        PlanStep step;
        step.time = static_cast<double>(plan.steps.size());
        step.name = sc.name;
        for (uint32_t v = 0; v < sc.params; ++v) {
          step.args.push_back(t_.object_names[nodes[n].args[v]]);
        }
        plan.steps.push_back(std::move(step));
      }
      return plan;
    }
    if (id != 0) {
      const Node & nd = nodes[id];
      states[id] = apply(states[nd.parent], schemas_[nd.schema], nd.args);
      assert(goal_count(states[id]) == nd.h);  // kept incrementally
      size_t bytes = 0;
      for (const auto & r : states[id]) {
        bytes += sizeof(Relation) + r.data.size() * sizeof(uint32_t);
      }
      if (budget_) {
        if (!budget_->reserve(bytes)) {
          return std::nullopt;
        }
        budget_->account(static_cast<std::ptrdiff_t>(bytes));
        accounted_ += bytes;
      }
    }
    if (use_ff_ && id != 0) {
      preferred.clear();
      nodes[id].evaluated = true;
      nodes[id].ff = ff(states[id], &preferred);
      if (nodes[id].ff == UINT32_MAX) {
        continue;  // a dead end
      }
      if (nodes[id].ff < best_ff) {
        best_ff = nodes[id].ff;
        boost += kBoost;
      }
    }
    prefer.clear();
    for (const auto & [si, args] : preferred) {
      prefer.insert(action_key(si, args));
    }
    preferred.clear();
    ++stats_.expanded;
    ++expansion_;  // the indexes of the fluent relations are of another state now
    successors(states[id], succ);
    for (auto & [si, args] : succ) {
      const Schema & sc = schemas_[si];
      // Its key: the parent's, less the atoms it deletes that hold, plus those it adds that
      // do not (an atom both deleted and added stays)
      const State & s = states[id];
      Key k = keys[id];
      thread_local std::vector<std::pair<uint32_t, Tuple>> dels, adds;
      dels.clear();
      adds.clear();
      auto ground = [&](const Pattern & p) {
          Tuple g;
          for (Arg x : p.args) {
            g.push_back(x >= 0 ? args[static_cast<size_t>(x)] : static_cast<uint32_t>(-x - 1));
          }
          if (g.empty()) {
            g.push_back(0);
          }
          return std::make_pair(static_cast<uint32_t>(fluent_of_[p.pred]), std::move(g));
        };
      for (const auto & p : sc.add) {
        adds.push_back(ground(p));
      }
      for (const auto & p : sc.del) {
        auto d = ground(p);
        if (std::find(adds.begin(), adds.end(), d) == adds.end()) {
          dels.push_back(std::move(d));
        }
      }
      std::sort(adds.begin(), adds.end());
      adds.erase(std::unique(adds.begin(), adds.end()), adds.end());
      std::sort(dels.begin(), dels.end());
      dels.erase(std::unique(dels.begin(), dels.end()), dels.end());
      int32_t delta = 0;
      for (const auto & [f, g] : dels) {
        if (s[f].contains(g.data())) {
          k.a ^= atom_hash(f, g.data(), static_cast<uint32_t>(g.size()), 1);
          k.b ^= atom_hash(f, g.data(), static_cast<uint32_t>(g.size()), 2);
          delta += goal_delta(f, g, false);
        }
      }
      for (const auto & [f, g] : adds) {
        if (!s[f].contains(g.data())) {
          k.a ^= atom_hash(f, g.data(), static_cast<uint32_t>(g.size()), 1);
          k.b ^= atom_hash(f, g.data(), static_cast<uint32_t>(g.size()), 2);
          delta += goal_delta(f, g, true);
        }
      }
      ++stats_.generated;
      if (!seen.insert(k).second) {
        continue;
      }
      auto nid = static_cast<uint32_t>(nodes.size());
      // Its state is built when it is expanded (from the parent's, kept): most never are
      auto h = static_cast<uint32_t>(static_cast<int32_t>(nodes[id].h) + delta);
      states.emplace_back();
      keys.push_back(k);
      bool fresh = false;
      for (const auto & p : sc.add) {
        tup.clear();
        for (Arg x : p.args) {
          tup.push_back(x >= 0 ? args[static_cast<size_t>(x)] : static_cast<uint32_t>(-x - 1));
        }
        if (tup.empty()) {
          tup.push_back(0);
        }
        fresh = note(h, atom_key(static_cast<uint32_t>(fluent_of_[p.pred]), tup.data(),
          static_cast<uint32_t>(tup.size()))) || fresh;
      }
      size_t bytes = sizeof(Node) + sizeof(State) + sizeof(Key) + args.size() * sizeof(uint32_t) +
        96;
      if (budget_) {
        if (!budget_->reserve(bytes)) {
          return std::nullopt;
        }
        budget_->account(static_cast<std::ptrdiff_t>(bytes));
        accounted_ += bytes;
      }
      uint32_t inherited = h == 0 ? 0 : (use_ff_ ? nodes[id].ff : h);
      nodes.push_back({id, si, args, h, inherited, h == 0});
      closed.push_back(0);
      ++stats_.states;
      if (h < best) {
        best = h;
        log_.debug(LogChannel::Search, "lifted: goal count {} ({} expanded, {} states)", h,
          stats_.expanded, stats_.states);
      }
      push(nid, h, fresh ? 1u : 2u, inherited);
      if (prefer.count(action_key(si, args))) {
        by_pref.emplace((static_cast<uint64_t>(std::min(inherited, 0xffffffu)) << 32) |
          (order & 0xffffffffULL), nid);
      }
    }
  }
  return std::nullopt;
}

}  // namespace

std::string unsupported(const lifted::Task & t)
{
  // Object fluents as terms (static ones too): the engine reads arguments as objects
  auto function_term = [](const lifted::Term & x) {return x.kind == lifted::Term::Kind::Function;};
  auto has_function = [&](const lifted::Condition & c) {
      for (const auto & l : c.literals) {
        if (std::any_of(l.atom.args.begin(), l.atom.args.end(), function_term)) {
          return true;
        }
      }
      return std::any_of(c.comparisons.begin(), c.comparisons.end(),
               [&](const lifted::Comparison & cmp) {
                 return cmp.objects && (function_term(cmp.lhs_term) || function_term(cmp.rhs_term));
               });
    };
  if (has_function(t.goal)) {
    return "object fluents";
  }
  if (!t.durative_actions.empty() || !t.processes.empty() || !t.events.empty()) {
    return "durative actions, processes or events";
  }
  if (!t.axioms.empty()) {
    return "derived predicates";
  }
  if (!t.timed_events.empty()) {
    return "timed initial literals";
  }
  if (!t.goal.foralls.empty() || !t.goal.comparisons.empty()) {
    return "a goal with quantifiers or comparisons";
  }
  for (const auto & a : t.actions) {
    if (has_function(a.precondition)) {
      return "object fluents (" + a.name + ")";
    }
    if (!a.precondition.foralls.empty()) {
      return "quantified preconditions (" + a.name + ")";
    }
    for (const auto & c : a.precondition.comparisons) {
      if (!c.objects || (c.op != lifted::CompareOp::Eq && c.op != lifted::CompareOp::Ne)) {
        return "numeric conditions (" + a.name + ")";
      }
    }
    for (const auto & e : a.effects) {
      if (e.timing != lifted::Timing::Instant || !e.condition.empty() || !e.variables.empty()) {
        return "conditional or quantified effects (" + a.name + ")";
      }
      if (e.kind == lifted::Effect::Kind::ObjectAssign ||
        std::any_of(e.atom.args.begin(), e.atom.args.end(), function_term))
      {
        return "object fluents (" + a.name + ")";
      }
      if (e.kind == lifted::Effect::Kind::Numeric &&
        t.function_names[e.function.value()] != "total-cost")
      {
        return "numeric effects (" + a.name + ")";
      }
    }
  }
  for (const auto & l : t.goal.literals) {
    for (const auto & term : l.atom.args) {
      if (term.kind != lifted::Term::Kind::Object) {
        return "a goal with variables";
      }
    }
  }
  return "";
}

std::optional<Plan> plan(
  const lifted::Task & task, Budget * budget, Logger logger, Stats & stats,
  const std::function<bool()> & stop)
{
  Lift lift(task, budget, std::move(logger), stats);
  return lift.run(stop);
}

}  // namespace kairos::lift
