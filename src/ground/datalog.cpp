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

#include "ground/datalog.hpp"

#include <algorithm>
#include <map>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace kairos::datalog
{

namespace
{

uint64_t mix(uint64_t h, uint64_t v)
{
  h ^= v + 0x9e3779b97f4a7c15ULL + (h << 6) + (h >> 2);
  return h;
}

uint64_t finish(uint64_t h)
{
  h ^= h >> 33;
  h *= 0xff51afd7ed558ccdULL;
  h ^= h >> 33;
  return h;
}

uint64_t hash_args(uint32_t predicate, std::span<const uint32_t> args)
{
  uint64_t h = mix(0xcbf29ce484222325ULL, predicate);
  for (uint32_t a : args) {
    h = mix(h, a);
  }
  return finish(h);
}

std::set<uint32_t> vars_of(const Atom & a)
{
  std::set<uint32_t> out;
  for (const auto & t : a.args) {
    if (t.is_variable) {
      out.insert(t.value);
    }
  }
  return out;
}

}  // namespace

void Program::Index::insert(uint64_t key, uint32_t f)
{
  if ((used + 1) * 2 > keys.size()) {
    // Rehash into a table twice as big
    std::vector<uint64_t> old_keys = std::move(keys);
    std::vector<uint32_t> old_heads = std::move(heads);
    size_t n = std::max<size_t>(16, old_keys.size() * 2);
    keys.assign(n, 0);
    heads.assign(n, kUnbound);
    for (size_t i = 0; i < old_keys.size(); ++i) {
      if (old_heads[i] != kUnbound) {
        size_t j = old_keys[i] & (n - 1);
        while (heads[j] != kUnbound) {
          j = (j + 1) & (n - 1);
        }
        keys[j] = old_keys[i];
        heads[j] = old_heads[i];
      }
    }
  }
  size_t mask = keys.size() - 1;
  size_t j = key & mask;
  while (heads[j] != kUnbound && keys[j] != key) {
    j = (j + 1) & mask;
  }
  auto e = static_cast<uint32_t>(fact.size());
  fact.push_back(f);
  next.push_back(heads[j]);
  if (heads[j] == kUnbound) {
    keys[j] = key;
    ++used;
  }
  heads[j] = e;
}

uint32_t Program::Index::find(uint64_t key) const
{
  if (keys.empty()) {
    return kUnbound;
  }
  size_t mask = keys.size() - 1;
  size_t j = key & mask;
  while (heads[j] != kUnbound) {
    if (keys[j] == key) {
      return heads[j];
    }
    j = (j + 1) & mask;
  }
  return kUnbound;
}

void Program::mark_static(uint32_t predicate)
{
  static_.at(predicate) = true;
}

double Program::distinct(uint32_t predicate, size_t position)
{
  auto & cache = distinct_[predicate];
  if (cache.empty()) {
    cache.assign(arity_[predicate], -1.0);
  }
  if (cache[position] < 0) {
    std::vector<uint32_t> values;
    for (uint32_t f : by_predicate_[predicate]) {
      values.push_back(args(f)[position]);
    }
    std::sort(values.begin(), values.end());
    cache[position] = static_cast<double>(
      std::unique(values.begin(), values.end()) - values.begin());
  }
  return cache[position];
}

uint32_t Program::add_predicate(uint32_t arity, std::string name)
{
  static_.push_back(false);
  distinct_.emplace_back();
  arity_.push_back(arity);
  names_.push_back(std::move(name));
  by_predicate_.emplace_back();
  triggers_.emplace_back();
  return static_cast<uint32_t>(arity_.size() - 1);
}

std::span<const uint32_t> Program::args(uint32_t fact) const
{
  const auto & f = facts_[fact];
  return std::span<const uint32_t>(pool_.data() + f.offset, arity_[f.predicate]);
}

constexpr uint64_t kEmpty = UINT64_MAX;

uint32_t Program::lookup(uint32_t predicate, std::span<const uint32_t> key, uint32_t hash) const
{
  if (slots_.empty()) {
    return kUnbound;
  }
  size_t mask = slots_.size() - 1;
  for (size_t i = hash & mask; slots_[i] != kEmpty; i = (i + 1) & mask) {
    if (static_cast<uint32_t>(slots_[i] >> 32) != hash) {
      continue;  // the hash in the slot saves touching the fact
    }
    auto f = static_cast<uint32_t>(slots_[i]);
    if (facts_[f].predicate == predicate) {
      auto y = args(f);
      if (std::equal(key.begin(), key.end(), y.begin(), y.end())) {
        return f;
      }
    }
  }
  return kUnbound;
}

uint32_t Program::find(uint32_t predicate, std::span<const uint32_t> args) const
{
  if (predicate >= arity_.size() || args.size() != arity_[predicate]) {
    return kUnbound;
  }
  return lookup(predicate, args, static_cast<uint32_t>(hash_args(predicate, args)));
}

void Program::grow()
{
  std::vector<uint64_t> old = std::move(slots_);
  size_t n = std::max<size_t>(64, old.size() * 2);
  slots_.assign(n, kEmpty);
  for (uint64_t v : old) {
    if (v != kEmpty) {
      size_t i = (v >> 32) & (n - 1);
      while (slots_[i] != kEmpty) {
        i = (i + 1) & (n - 1);
      }
      slots_[i] = v;
    }
  }
}

uint32_t Program::intern(uint32_t predicate, std::span<const uint32_t> args)
{
  if (args.size() != arity_.at(predicate)) {
    throw std::logic_error("datalog: wrong number of arguments for " + names_[predicate]);
  }
  auto hash = static_cast<uint32_t>(hash_args(predicate, args));
  uint32_t found = lookup(predicate, args, hash);
  if (found != kUnbound) {
    return found;
  }
  if ((facts_.size() + 1) * 2 > slots_.size()) {
    grow();
  }
  auto id = static_cast<uint32_t>(facts_.size());
  facts_.push_back(FactRef{predicate, static_cast<uint32_t>(pool_.size())});
  pool_.insert(pool_.end(), args.begin(), args.end());
  size_t mask = slots_.size() - 1;
  size_t i = hash & mask;
  while (slots_[i] != kEmpty) {
    i = (i + 1) & mask;
  }
  slots_[i] = (uint64_t{hash} << 32) | id;
  by_predicate_[predicate].push_back(id);
  return id;
}

void Program::add_fact(uint32_t predicate, std::span<const uint32_t> args)
{
  intern(predicate, args);
}

void Program::add_rule(Rule rule)
{
  std::set<uint32_t> body;
  for (const auto & a : rule.body) {
    auto v = vars_of(a);
    body.insert(v.begin(), v.end());
  }
  auto check_bound = [&](uint32_t v) {
      if (v >= rule.variables || !body.count(v)) {
        throw std::logic_error("datalog: variable not bound by the body of a rule for " +
                names_.at(rule.head.predicate));
      }
    };
  for (uint32_t v : vars_of(rule.head)) {
    check_bound(v);
  }
  for (const auto & c : rule.checks) {
    for (uint32_t v : c.variables) {
      check_bound(v);
    }
  }
  compile(std::move(rule));
}

void Program::compile(Rule rule)
{
  if (rule.body.empty()) {
    ground_rules_.push_back(std::move(rule));
    return;
  }
  auto emit = [&](Atom head, std::vector<Atom> conds, std::vector<Check> checks) {
      CompiledRule r;
      r.head = std::move(head);
      r.size = static_cast<uint8_t>(conds.size());
      r.variables = rule.variables;
      r.checks = std::move(checks);
      for (size_t i = 0; i < conds.size(); ++i) {
        r.cond[i] = std::move(conds[i]);
      }
      if (r.size == 2) {
        auto a = vars_of(r.cond[0]);
        for (uint32_t v : vars_of(r.cond[1])) {
          if (a.count(v)) {
            r.key.push_back(v);
          }
        }
      }
      auto id = static_cast<uint32_t>(rules_.size());
      for (uint8_t side = 0; side < r.size; ++side) {
        auto & t = triggers_[r.cond[side].predicate];
        t.emplace_back(id, side);
      }
      rules_.push_back(std::move(r));
    };

  std::vector<Atom> conds = std::move(rule.body);
  std::vector<bool> attached(rule.checks.size(), false);
  auto take_checks = [&](const std::set<uint32_t> & bound) {
      std::vector<Check> out;
      for (size_t i = 0; i < rule.checks.size(); ++i) {
        if (attached[i]) {
          continue;
        }
        const auto & vs = rule.checks[i].variables;
        if (std::all_of(vs.begin(), vs.end(), [&](uint32_t v) {return bound.count(v) > 0;})) {
          attached[i] = true;
          out.push_back(rule.checks[i]);
        }
      }
      return out;
    };

  // Size estimates, as databases do: |A x B| = |A| |B| / prod max(V(A, v), V(B, v)) over the
  // shared variables, V(R, v) being the number of values v takes in R. Static atoms count them
  // in their facts; other atoms take each variable's smallest static count.
  constexpr double kUnknown = 1000.0;
  struct Estimate
  {
    double size = 1.0;
    std::map<uint32_t, double> values;
  };
  std::vector<double> domain(rule.variables, kUnknown);
  std::vector<Estimate> est;
  for (const auto & c : conds) {
    Estimate e;
    if (static_[c.predicate]) {
      e.size = static_cast<double>(by_predicate_[c.predicate].size());
      for (size_t i = 0; i < c.args.size(); ++i) {
        if (c.args[i].is_variable) {
          double n = distinct(c.predicate, i);
          auto & slot = e.values.try_emplace(c.args[i].value, n).first->second;
          slot = std::min(slot, n);
          domain[c.args[i].value] = std::min(domain[c.args[i].value], n);
        }
      }
    }
    est.push_back(std::move(e));
  }
  for (size_t k = 0; k < conds.size(); ++k) {
    if (!static_[conds[k].predicate]) {
      for (uint32_t v : vars_of(conds[k])) {
        est[k].values[v] = domain[v];
        est[k].size *= std::max(domain[v], 1.0);
      }
    }
  }
  auto join = [](const Estimate & a, const Estimate & b) {
      Estimate r;
      r.size = a.size * b.size;
      for (const auto & [v, n] : a.values) {
        auto it = b.values.find(v);
        if (it != b.values.end()) {
          r.size /= std::max({n, it->second, 1.0});
        }
      }
      for (const auto * x : {&a, &b}) {
        for (const auto & [v, n] : x->values) {
          auto & slot = r.values.try_emplace(v, n).first->second;
          slot = std::min(slot, n);
        }
      }
      for (auto & [v, n] : r.values) {
        n = std::min(n, std::max(r.size, 1.0));
      }
      return r;
    };

  // Greedy joins: the pair with the smallest estimated result first (more shared variables
  // on ties)
  while (conds.size() > 2) {
    size_t bi = 0, bj = 1;
    double best = -1.0;
    int64_t best_shared = -1;
    for (size_t i = 0; i < conds.size(); ++i) {
      auto vi = vars_of(conds[i]);
      for (size_t j = i + 1; j < conds.size(); ++j) {
        int64_t shared = 0;
        for (uint32_t v : vars_of(conds[j])) {
          shared += vi.count(v) ? 1 : 0;
        }
        double e = join(est[i], est[j]).size;
        if (best < 0 || e < best || (e == best && shared > best_shared)) {
          best = e;
          best_shared = shared;
          bi = i;
          bj = j;
        }
      }
    }
    std::set<uint32_t> bound = vars_of(conds[bi]);
    for (uint32_t v : vars_of(conds[bj])) {
      bound.insert(v);
    }
    std::vector<Check> checks = take_checks(bound);
    // Keep what later conditions, the head or pending checks need
    std::set<uint32_t> needed = vars_of(rule.head);
    for (size_t k = 0; k < conds.size(); ++k) {
      if (k != bi && k != bj) {
        auto v = vars_of(conds[k]);
        needed.insert(v.begin(), v.end());
      }
    }
    for (size_t i = 0; i < rule.checks.size(); ++i) {
      if (!attached[i]) {
        needed.insert(rule.checks[i].variables.begin(), rule.checks[i].variables.end());
      }
    }
    Atom inter;
    for (uint32_t v : bound) {
      if (needed.count(v)) {
        inter.args.push_back(Term::var(v));
      }
    }
    inter.predicate = add_predicate(
      static_cast<uint32_t>(inter.args.size()), "join:" + names_[rule.head.predicate]);
    emit(inter, {conds[bi], conds[bj]}, std::move(checks));
    Estimate e = join(est[bi], est[bj]);
    std::erase_if(e.values, [&](const auto & kv) {return !needed.count(kv.first);});
    conds[bi] = inter;
    est[bi] = std::move(e);
    conds.erase(conds.begin() + static_cast<std::ptrdiff_t>(bj));
    est.erase(est.begin() + static_cast<std::ptrdiff_t>(bj));
  }
  std::set<uint32_t> all;
  for (const auto & c : conds) {
    auto v = vars_of(c);
    all.insert(v.begin(), v.end());
  }
  emit(std::move(rule.head), std::move(conds), take_checks(all));
}

bool Program::unify(const Atom & pattern, uint32_t fact, uint32_t * binding) const
{
  auto a = args(fact);
  for (size_t i = 0; i < pattern.args.size(); ++i) {
    const Term & t = pattern.args[i];
    if (!t.is_variable) {
      if (a[i] != t.value) {
        return false;
      }
    } else if (binding[t.value] == kUnbound) {
      binding[t.value] = a[i];
    } else if (binding[t.value] != a[i]) {
      return false;
    }
  }
  return true;
}

uint64_t Program::key_of(const CompiledRule & r, const uint32_t * binding) const
{
  uint64_t h = 0x84222325cbf29ce4ULL;
  for (uint32_t v : r.key) {
    h = mix(h, binding[v]);
  }
  return finish(h);
}

void Program::fire(const Atom & head, const std::vector<Check> & checks, const uint32_t * binding)
{
  for (const auto & c : checks) {
    if (!c.test(binding)) {
      return;
    }
  }
  head_.clear();
  for (const auto & t : head.args) {
    head_.push_back(t.is_variable ? binding[t.value] : t.value);
  }
  intern(head.predicate, head_);
}

void Program::process(uint32_t fact)
{
  const auto & triggers = triggers_[facts_[fact].predicate];
  for (const auto & [rid, side] : triggers) {
    CompiledRule & r = rules_[rid];
    scratch_a_.assign(r.variables, kUnbound);
    if (!unify(r.cond[side], fact, scratch_a_.data())) {
      continue;
    }
    if (r.size == 1) {
      fire(r.head, r.checks, scratch_a_.data());
      continue;
    }
    uint64_t key = key_of(r, scratch_a_.data());
    r.index[side].insert(key, fact);
    const Index & other = r.index[1 - side];
    for (uint32_t e = other.find(key); e != kUnbound; e = other.next[e]) {
      ++stats_.join_probes;
      scratch_b_ = scratch_a_;
      if (unify(r.cond[1 - side], other.fact[e], scratch_b_.data())) {
        fire(r.head, r.checks, scratch_b_.data());
      }
    }
  }
}

bool Program::solve(Budget * budget)
{
  for (const auto & r : ground_rules_) {
    fire(r.head, r.checks, nullptr);
  }
  ground_rules_.clear();
  stats_.rules = rules_.size();
  while (queue_ < facts_.size()) {
    process(static_cast<uint32_t>(queue_++));
    if (budget && budget->exhausted()) {  // cheap: full checks every check_interval calls
      stats_.facts = facts_.size();
      return false;
    }
  }
  stats_.facts = facts_.size();
  return true;
}

}  // namespace kairos::datalog
