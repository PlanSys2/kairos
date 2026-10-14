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

#include "state/registry.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <functional>
#include <stdexcept>
#include <utility>
#include <vector>

namespace kairos::state
{

namespace
{

std::vector<uint32_t> widths(const sas::Task & task)
{
  std::vector<uint32_t> out;
  for (const auto & v : task.variables) {
    out.push_back(bits_for(v.size()));
  }
  return out;
}

uint64_t mix(uint64_t h, uint64_t v)
{
  h ^= v + 0x9e3779b97f4a7c15ULL + (h << 6) + (h >> 2);
  return h;
}

// Bits of a number for hashing and comparing: -0.0 is 0.0, every NaN is the same
uint64_t canonical(double x)
{
  if (std::isnan(x)) {
    return 0x7ff8000000000000ULL;
  }
  if (x == 0.0) {
    return 0;
  }
  uint64_t b;
  std::memcpy(&b, &x, sizeof(b));
  return b;
}

// Numeric variables nothing reads (conditions, durations, effect values, axioms, the goal),
// such as a cost the metric adds up: they do not tell states apart. Kept, they made states
// that differ only in what was spent look new (PlanSys2's combat: millions after the first plan)
std::vector<uint8_t> unread_numbers(const sas::Task & t)
{
  std::vector<uint8_t> read(t.numeric_fluents.size(), 0);
  std::function<void(const ground::Expr &)> expr = [&](const ground::Expr & e) {
      if (e.op == ground::ExprOp::Fluent && e.fluent < read.size()) {
        read[e.fluent] = 1;
      }
      for (const auto & c : e.children) {
        expr(c);
      }
    };
  auto cond = [&](const sas::Condition & c) {
      for (const auto & n : c.numeric) {
        expr(n.lhs.tree);
        expr(n.rhs.tree);
      }
    };
  auto effects = [&](const std::vector<sas::Effect> & es) {
      for (const auto & e : es) {
        cond(e.condition);
        cond(e.start_condition);
        if (e.kind == sas::Effect::Kind::Numeric) {
          expr(e.value.tree);
        }
      }
    };
  for (const auto * list : {&t.actions, &t.processes, &t.events}) {
    for (const auto & a : *list) {
      cond(a.pre);
      effects(a.effects);
    }
  }
  for (const auto & a : t.durative_actions) {
    cond(a.at_start);
    cond(a.over_all);
    cond(a.at_end);
    for (const auto & d : a.duration) {
      expr(d.value.tree);
    }
    effects(a.effects);
  }
  for (const auto & ax : t.axioms) {
    cond(ax.body);
  }
  for (const auto & ev : t.timed_events) {
    effects(ev.effects);
  }
  cond(t.goal);
  cond(t.always);
  effects(t.clock_rates);
  for (const auto & r : t.clock_rates) {
    read[r.num] = 1;  // what time changes tells states apart (the time itself, for the plan)
  }
  for (const auto & d : t.clock_deadlines) {
    read[d.clock] = 1;
    if (d.bound) {
      read[*d.bound] = 1;
    }
  }
  std::vector<uint8_t> unread(read.size());
  for (size_t v = 0; v < read.size(); ++v) {
    unread[v] = !read[v];
  }
  return unread;
}

}  // namespace

StateRegistry::StateRegistry(const sas::Task & task, Budget * budget, bool merge_unread)
: task_(task), budget_(budget), layout_(widths(task)), axioms_(task),
  words_(std::max<size_t>(layout_.words(), 1)), num_vars_(task.numeric_fluents.size()),
  scratch_(words_), scratch_numbers_(num_vars_)
{
  if (task.initial_numeric.size() != num_vars_) {
    throw std::invalid_argument("StateRegistry: initial numbers do not match the variables");
  }
  unread_ = unread_numbers(task);
  for (size_t v = 0; v < num_vars_; ++v) {
    // One undefined at first may be assigned later, and then changed: being defined matters
    unread_[v] = unread_[v] && merge_unread && task.initial_numeric[v].has_value();
  }
  any_unread_ = std::any_of(unread_.begin(), unread_.end(), [](uint8_t u) {return u != 0;});
}

StateRegistry::~StateRegistry()
{
  if (budget_ && accounted_) {
    budget_->account(-static_cast<std::ptrdiff_t>(accounted_));
  }
}

void StateRegistry::account(size_t bytes)
{
  accounted_ += bytes;
  if (budget_) {
    budget_->account(static_cast<std::ptrdiff_t>(bytes));
  }
}

StateId StateRegistry::initial()
{
  if (initial_ == kNoState) {
    std::vector<uint32_t> values = task_.initial;
    std::vector<double> numbers(num_vars_);
    for (size_t i = 0; i < num_vars_; ++i) {
      numbers[i] = task_.initial_numeric[i] ? *task_.initial_numeric[i] : kUndefined;
    }
    initial_ = insert(values, numbers).first;
  }
  return initial_;
}

uint64_t StateRegistry::hash(const uint64_t * packed, const double * numbers) const
{
  uint64_t h = 0xcbf29ce484222325ULL;
  for (size_t i = 0; i < words_; ++i) {
    h = mix(h, packed[i]);
  }
  for (size_t i = 0; i < num_vars_; ++i) {
    h = mix(h, canonical(numbers[i]));
  }
  h ^= h >> 33;
  h *= 0xff51afd7ed558ccdULL;
  h ^= h >> 33;
  return h;
}

bool StateRegistry::equal(StateId id, const uint64_t * packed, const double * numbers) const
{
  if (std::memcmp(this->packed(id), packed, words_ * sizeof(uint64_t)) != 0) {
    return false;
  }
  if (!num_vars_) {
    return true;
  }
  const double * n = nums(id);
  for (size_t i = 0; i < num_vars_; ++i) {
    if (canonical(n[i]) != canonical(numbers[i])) {
      return false;
    }
  }
  return true;
}

constexpr uint64_t kFree = ~uint64_t{0};

void StateRegistry::grow_table()
{
  size_t n = std::max<size_t>(1024, table_.size() * 2);
  account((n - table_.size()) * sizeof(uint64_t));
  table_.assign(n, kFree);
  for (StateId id = 0; id < count_; ++id) {
    uint64_t h = hash(packed(id), num_vars_ ? nums(id) : nullptr);
    size_t i = h & (n - 1);
    while (table_[i] != kFree) {
      i = (i + 1) & (n - 1);
    }
    table_[i] = (h & 0xffffffff00000000ULL) | id;
  }
}

std::pair<StateId, bool> StateRegistry::insert(
  std::vector<uint32_t> & values, std::span<const double> numbers)
{
  if (values.size() != task_.variables.size() || numbers.size() != num_vars_) {
    throw std::invalid_argument("StateRegistry::insert: wrong state size");
  }
  if (any_unread_) {
    // What nothing reads keeps its initial value: such states are the same state
    scratch_numbers_.assign(numbers.begin(), numbers.end());
    for (size_t v = 0; v < num_vars_; ++v) {
      if (unread_[v]) {
        scratch_numbers_[v] = *task_.initial_numeric[v];
      }
    }
    numbers = scratch_numbers_;
  }
  if (!axioms_.empty()) {
    axioms_.evaluate(values.data(), numbers.data());
  }
  std::fill(scratch_.begin(), scratch_.end(), 0);
  for (size_t v = 0; v < values.size(); ++v) {
    layout_.set(scratch_.data(), v, values[v]);
  }
  if ((count_ + 1) * 4 > table_.size() * 3) {  // load up to 3/4
    grow_table();
  }
  uint64_t h = hash(scratch_.data(), numbers.data());
  uint64_t tag = h & 0xffffffff00000000ULL;
  size_t mask = table_.size() - 1;
  size_t i = h & mask;
  for (; table_[i] != kFree; i = (i + 1) & mask) {
    auto other = static_cast<StateId>(table_[i]);
    if ((table_[i] & 0xffffffff00000000ULL) == tag &&
      equal(other, scratch_.data(), numbers.data()))
    {
      return {other, false};
    }
  }
  if (count_ >= kNoState) {
    throw std::length_error("StateRegistry: too many states");
  }
  if (count_ % kBlock == 0) {
    blocks_.push_back(std::make_unique<uint64_t[]>(kBlock * words_));
    account(kBlock * words_ * sizeof(uint64_t));
    if (num_vars_) {
      num_blocks_.push_back(std::make_unique<double[]>(kBlock * num_vars_));
      account(kBlock * num_vars_ * sizeof(double));
    }
  }
  auto id = static_cast<StateId>(count_++);
  std::memcpy(blocks_[id / kBlock].get() + (id % kBlock) * words_, scratch_.data(),
    words_ * sizeof(uint64_t));
  if (num_vars_) {
    std::copy(numbers.begin(), numbers.end(), num_blocks_[id / kBlock].get() + (id % kBlock) *
      num_vars_);
  }
  table_[i] = tag | id;
  return {id, true};
}

void StateRegistry::unpack(StateId id, std::vector<uint32_t> & values) const
{
  values.resize(task_.variables.size());
  const uint64_t * p = packed(id);
  for (size_t v = 0; v < values.size(); ++v) {
    values[v] = static_cast<uint32_t>(layout_.get(p, v));
  }
}

std::span<const double> StateRegistry::numbers(StateId id) const
{
  if (!num_vars_) {
    return {};
  }
  return std::span<const double>(nums(id), num_vars_);
}

size_t StateRegistry::bytes() const
{
  return blocks_.size() * kBlock * words_ * sizeof(uint64_t) +
         num_blocks_.size() * kBlock * num_vars_ * sizeof(double) +
         table_.size() * sizeof(uint64_t);
}

}  // namespace kairos::state
