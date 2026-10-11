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

#include "state/packing.hpp"

#include <algorithm>
#include <numeric>
#include <stdexcept>
#include <vector>

namespace kairos::state
{

uint32_t bits_for(uint64_t domain)
{
  uint32_t bits = 1;
  while (bits < 64 && (uint64_t{1} << bits) < domain) {
    ++bits;
  }
  return bits;
}

PackedLayout::PackedLayout(const std::vector<uint32_t> & bits)
: slots_(bits.size())
{
  std::vector<size_t> order(bits.size());
  std::iota(order.begin(), order.end(), size_t{0});
  std::stable_sort(
    order.begin(), order.end(), [&](size_t a, size_t b) {return bits[a] > bits[b];});
  std::vector<uint32_t> used;  // bits used per word
  for (size_t v : order) {
    uint32_t b = bits[v];
    if (b == 0 || b > 64) {
      throw std::invalid_argument("PackedLayout: a variable needs 1 to 64 bits");
    }
    size_t w = 0;
    while (w < used.size() && used[w] + b > 64) {
      ++w;
    }
    if (w == used.size()) {
      used.push_back(0);
    }
    slots_[v].word = static_cast<uint32_t>(w);
    slots_[v].shift = used[w];
    slots_[v].mask = b == 64 ? ~uint64_t{0} : (uint64_t{1} << b) - 1;
    used[w] += b;
  }
  words_ = used.size();
}

}  // namespace kairos::state
