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
/// \brief Packing of variable values into 64-bit words.

#ifndef KAIROS__STATE__PACKING_HPP_
#define KAIROS__STATE__PACKING_HPP_

#include <cstddef>
#include <cstdint>
#include <vector>

namespace kairos::state
{

/// @brief Bits needed to store values 0 .. domain-1 (at least 1).
uint32_t bits_for(uint64_t domain);

/**
 * @brief Where each variable lives in a packed state: the fewest bits it needs, in a word it
 * does not straddle. Variables are placed widest first, each in the first word with room
 * (first fit decreasing), as FD's IntPacker does.
 */
class PackedLayout
{
public:
  PackedLayout() = default;
  /// @brief One entry per variable: its number of bits (1 to 64).
  explicit PackedLayout(const std::vector<uint32_t> & bits);

  size_t words() const {return words_;}
  size_t variables() const {return slots_.size();}

  uint64_t get(const uint64_t * packed, size_t var) const
  {
    const Slot & s = slots_[var];
    return (packed[s.word] >> s.shift) & s.mask;
  }

  void set(uint64_t * packed, size_t var, uint64_t value) const
  {
    const Slot & s = slots_[var];
    packed[s.word] = (packed[s.word] & ~(s.mask << s.shift)) | ((value & s.mask) << s.shift);
  }

private:
  struct Slot
  {
    uint32_t word = 0;
    uint32_t shift = 0;
    uint64_t mask = 0;
  };

  std::vector<Slot> slots_;
  size_t words_ = 0;
};

}  // namespace kairos::state

#endif  // KAIROS__STATE__PACKING_HPP_
