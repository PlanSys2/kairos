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
/// \brief "Did you mean ...?" suggestions for misspelled names.

#ifndef KAIROS__CORE__SUGGEST_HPP_
#define KAIROS__CORE__SUGGEST_HPP_

#include <cstddef>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace kairos
{

/// @brief Edit distance with adjacent transpositions (optimal string alignment).
size_t edit_distance(std::string_view a, std::string_view b);

/**
 * @brief The candidate closest to `name`, if close enough to be a likely typo: at most 2 edits,
 * and fewer than half the length of `name`. Ties keep the first candidate.
 */
std::optional<std::string> closest_match(
  std::string_view name, const std::vector<std::string> & candidates);

/// @brief " (did you mean 'x'?)" or an empty string.
std::string did_you_mean(std::string_view name, const std::vector<std::string> & candidates);

}  // namespace kairos

#endif  // KAIROS__CORE__SUGGEST_HPP_
