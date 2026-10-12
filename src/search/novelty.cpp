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

#include "search/novelty.hpp"

#include <algorithm>
#include <bit>  // NOLINT(build/include_order): cpplint does not know this header
#include <cmath>
#include <cstring>
#include <utility>
#include <vector>

namespace kairos::search
{

namespace
{

uint64_t mix(uint64_t x)
{
  // splitmix64 finalizer
  x ^= x >> 30;
  x *= 0xbf58476d1ce4e5b9ull;
  x ^= x >> 27;
  x *= 0x94d049bb133111ebull;
  x ^= x >> 31;
  return x;
}

uint64_t combine(uint64_t a, uint64_t b)
{
  return mix(a ^ (b + 0x9e3779b97f4a7c15ull + (a << 6) + (a >> 2)));
}

}  // namespace

Novelty::Novelty(
  size_t variables, size_t numeric, int max_width, size_t memory_cap_bytes, Budget * budget)
: variables_(variables), numeric_(numeric), max_width_(max_width < 1 ? 1 : max_width),
  cap_(memory_cap_bytes), budget_(budget)
{
  table_.assign(1024, 0);
  accounted_ = table_.size() * sizeof(uint64_t);
  if (budget_) {
    budget_->account(static_cast<std::ptrdiff_t>(accounted_));
  }
}

Novelty::~Novelty()
{
  if (budget_) {
    budget_->account(-static_cast<std::ptrdiff_t>(accounted_));
  }
}

bool Novelty::insert(uint64_t key)
{
  key |= 1;  // never 0, the empty mark
  if ((size_ + 1) * 4 > table_.size() * 3) {
    grow();
  }
  size_t mask = table_.size() - 1;
  for (size_t i = key & mask; ; i = (i + 1) & mask) {
    if (table_[i] == 0) {
      table_[i] = key;
      ++size_;
      return true;
    }
    if (table_[i] == key) {
      return false;
    }
  }
}

void Novelty::grow()
{
  std::vector<uint64_t> old = std::move(table_);
  table_.assign(old.size() * 2, 0);
  size_t added = (table_.size() - old.size()) * sizeof(uint64_t);
  accounted_ += added;
  if (budget_) {
    budget_->account(static_cast<std::ptrdiff_t>(added));
  }
  size_ = 0;
  size_t mask = table_.size() - 1;
  for (uint64_t k : old) {
    if (k == 0) {
      continue;
    }
    size_t i = k & mask;
    while (table_[i] != 0) {
      i = (i + 1) & mask;
    }
    table_[i] = k;
    ++size_;
  }
}

int Novelty::evaluate(
  uint64_t partition, const uint32_t * values, const double * numbers,
  const uint32_t * parent_values, const double * parent_numbers)
{
  features_.clear();
  for (size_t v = 0; v < variables_; ++v) {
    features_.push_back(combine(v, values[v]));
  }
  for (size_t n = 0; n < numeric_; ++n) {
    double x = numbers[n];
    // Undefined is one more value; -0 is 0
    uint64_t bits = std::isnan(x) ? 0x7ff8dead0000beefull :
      std::bit_cast<uint64_t>(x == 0.0 ? 0.0 : x);
    features_.push_back(combine(variables_ + n, bits));
  }
  uint64_t p = mix(partition + 0x51ed27ull);
  int w = 3;
  for (uint64_t f : features_) {
    if (insert(combine(p, f))) {
      w = 1;
    }
  }
  if (max_width_ < 2 || pairs_dropped_) {
    return w == 1 ? 1 : 3;
  }
  if (accounted_ > cap_) {
    pairs_dropped_ = true;
    return w == 1 ? 1 : 3;
  }
  bool new_pair = false;
  auto pair = [&](size_t i, size_t j) {
      // The cap holds inside one evaluation too: a state of 20 000 features has 200 million
      // pairs, and doubling the table for them went past the memory limit
      if ((size_ + 1) * 4 > table_.size() * 3 && table_.size() * 2 * sizeof(uint64_t) > cap_) {
        pairs_dropped_ = true;
        return false;
      }
      size_t a = std::min(i, j), b = std::max(i, j);
      return insert(combine(combine(p ^ 0xa5a5a5a5ull, features_[a]), features_[b]));
    };
  if (parent_values && features_.size() > kIncremental) {
    // Large states: only pairs with a feature that changed from the parent (as BFWS does); the
    // others were in the parent already, but for a change of partition
    changed_.clear();
    for (size_t v = 0; v < variables_; ++v) {
      if (values[v] != parent_values[v]) {
        changed_.push_back(v);
      }
    }
    for (size_t n = 0; n < numeric_; ++n) {
      double x = numbers[n], y = parent_numbers ? parent_numbers[n] : x;
      if (!(x == y || (std::isnan(x) && std::isnan(y)))) {
        changed_.push_back(variables_ + n);
      }
    }
    for (size_t i : changed_) {
      for (size_t j = 0; j < features_.size() && !pairs_dropped_; ++j) {
        if (j != i && pair(i, j)) {
          new_pair = true;
        }
      }
    }
  } else if (features_.size() <= kIncremental) {
    for (size_t i = 0; i < features_.size() && !pairs_dropped_; ++i) {
      for (size_t j = i + 1; j < features_.size() && !pairs_dropped_; ++j) {
        if (pair(i, j)) {
          new_pair = true;
        }
      }
    }
  }
  // A large state without its parent (the initial one): only its features, not all its pairs
  return w == 1 ? 1 : (new_pair ? 2 : 3);
}

}  // namespace kairos::search
