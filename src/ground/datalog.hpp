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
/// \brief Datalog engine for relaxed reachability (positive rules, no negation).

#ifndef KAIROS__GROUND__DATALOG_HPP_
#define KAIROS__GROUND__DATALOG_HPP_

#include <span>  // cpplint takes it for a C header

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>
#include <utility>
#include <vector>

#include "core/budget.hpp"

namespace kairos::datalog
{

/// @brief A rule variable (index into the rule's binding) or a constant.
struct Term
{
  bool is_variable = true;
  uint32_t value = 0;

  static Term var(uint32_t v) {return Term{true, v};}
  static Term constant(uint32_t c) {return Term{false, c};}
};

struct Atom
{
  uint32_t predicate = 0;
  std::vector<Term> args;
};

/// @brief A test on variables, run as soon as all of them are bound.
struct Check
{
  std::vector<uint32_t> variables;
  std::function<bool(const uint32_t * binding)> test;
};

/// @brief head :- body, checks. Every head and check variable must occur in the body.
struct Rule
{
  Atom head;
  std::vector<Atom> body;
  uint32_t variables = 0;  // binding size
  std::vector<Check> checks;
};

struct Stats
{
  size_t facts = 0;
  size_t rules = 0;         // after splitting into joins
  size_t join_probes = 0;   // partner facts tried
};

/**
 * @brief Least model of a positive Datalog program.
 *
 * Rules are split into binary joins (greedily, joining first the conditions that share most
 * variables) and evaluated from a queue of new facts: each fact is matched against the
 * conditions it can fill and joined with the stored partners of the same key, so every pair is
 * tried once (semi-naive evaluation).
 */
class Program
{
public:
  static constexpr uint32_t kUnbound = UINT32_MAX;

  uint32_t add_predicate(uint32_t arity, std::string name = {});
  void add_fact(uint32_t predicate, std::span<const uint32_t> args);
  void add_rule(Rule rule);

  /// @brief No rule derives this predicate: its facts, added before the rules, guide the
  /// order of joins.
  void mark_static(uint32_t predicate);

  /// @brief Computes the model. False if the budget ran out (the model is then partial).
  bool solve(Budget * budget = nullptr);

  uint32_t predicates() const {return static_cast<uint32_t>(arity_.size());}
  uint32_t arity(uint32_t predicate) const {return arity_[predicate];}
  const std::string & name(uint32_t predicate) const {return names_[predicate];}

  /// @brief Facts of a predicate, as fact ids, in the order they were found.
  const std::vector<uint32_t> & facts(uint32_t predicate) const {return by_predicate_[predicate];}
  std::span<const uint32_t> args(uint32_t fact) const;
  uint32_t predicate_of(uint32_t fact) const {return facts_[fact].predicate;}

  /// @brief Fact id of a ground atom, or kUnbound if it is not in the model.
  uint32_t find(uint32_t predicate, std::span<const uint32_t> args) const;

  const Stats & stats() const {return stats_;}

private:
  struct FactRef
  {
    uint32_t predicate;
    uint32_t offset;
  };

  // Facts of one side of a join, grouped by the hash of their key: chains of entries
  struct Index
  {
    std::vector<uint64_t> keys;    // open addressing table of key hashes
    std::vector<uint32_t> heads;   // first entry of each key, kUnbound if the slot is free
    std::vector<uint32_t> fact;    // entries
    std::vector<uint32_t> next;
    size_t used = 0;

    void insert(uint64_t key, uint32_t f);
    uint32_t find(uint64_t key) const;  // first entry or kUnbound
  };

  struct CompiledRule
  {
    Atom head;
    Atom cond[2];
    uint8_t size = 1;                    // 1 or 2 conditions
    uint32_t variables = 0;
    std::vector<uint32_t> key;           // variables shared by both conditions
    std::vector<Check> checks;
    Index index[2];
  };

  void compile(Rule rule);
  bool unify(const Atom & pattern, uint32_t fact, uint32_t * binding) const;
  void fire(const Atom & head, const std::vector<Check> & checks, const uint32_t * binding);
  uint32_t intern(uint32_t predicate, std::span<const uint32_t> args);
  uint32_t lookup(uint32_t predicate, std::span<const uint32_t> args, uint32_t hash) const;
  void grow();
  // Whether an allocation of this size fits in the budget (once one does not, nothing grows)
  bool room(size_t bytes);
  template<typename T>
  bool room_to_add(const std::vector<T> & v, size_t n = 1)
  {
    return v.size() + n <= v.capacity() || room(std::max(2 * v.capacity(), v.size() + n) *
           sizeof(T));
  }
  double distinct(uint32_t predicate, size_t position);
  void process(uint32_t fact);
  uint64_t key_of(const CompiledRule & r, const uint32_t * binding) const;

  std::vector<uint32_t> arity_;
  std::vector<bool> static_;
  std::vector<std::vector<double>> distinct_;  // static predicates: values per position
  std::vector<std::string> names_;
  Budget * budget_ = nullptr;  // during solve()
  bool full_ = false;          // an allocation did not fit
  std::vector<uint32_t> pool_;
  std::vector<FactRef> facts_;
  std::vector<std::vector<uint32_t>> by_predicate_;
  std::vector<uint64_t> slots_;  // open addressing set: hash << 32 | fact id (kEmpty if free)
  std::vector<CompiledRule> rules_;
  std::vector<std::vector<std::pair<uint32_t, uint8_t>>> triggers_;  // per predicate
  std::vector<Rule> ground_rules_;                                    // empty bodies
  size_t queue_ = 0;  // next fact to process
  std::vector<uint32_t> scratch_a_, scratch_b_, head_;
  Stats stats_;
};

}  // namespace kairos::datalog

#endif  // KAIROS__GROUND__DATALOG_HPP_
