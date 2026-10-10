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

#include "core/suggest.hpp"

#include <algorithm>
#include <string>
#include <vector>

namespace kairos
{

size_t edit_distance(std::string_view a, std::string_view b)
{
  const size_t n = a.size();
  const size_t m = b.size();
  std::vector<std::vector<size_t>> d(n + 1, std::vector<size_t>(m + 1, 0));
  for (size_t i = 0; i <= n; ++i) {
    d[i][0] = i;
  }
  for (size_t j = 0; j <= m; ++j) {
    d[0][j] = j;
  }
  for (size_t i = 1; i <= n; ++i) {
    for (size_t j = 1; j <= m; ++j) {
      size_t cost = a[i - 1] == b[j - 1] ? 0 : 1;
      d[i][j] = std::min({d[i - 1][j] + 1, d[i][j - 1] + 1, d[i - 1][j - 1] + cost});
      if (i > 1 && j > 1 && a[i - 1] == b[j - 2] && a[i - 2] == b[j - 1]) {
        d[i][j] = std::min(d[i][j], d[i - 2][j - 2] + 1);
      }
    }
  }
  return d[n][m];
}

std::optional<std::string> closest_match(
  std::string_view name, const std::vector<std::string> & candidates)
{
  std::optional<std::string> best;
  size_t best_distance = 3;  // more than 2 edits is not a typo
  for (const auto & c : candidates) {
    if (c == name) {
      continue;
    }
    // Skip candidates whose length alone rules them out (cheap with many candidates)
    size_t diff = c.size() > name.size() ? c.size() - name.size() : name.size() - c.size();
    if (diff >= best_distance) {
      continue;
    }
    size_t d = edit_distance(name, c);
    if (d < best_distance && 2 * d < name.size()) {
      best_distance = d;
      best = c;
    }
  }
  return best;
}

std::string did_you_mean(std::string_view name, const std::vector<std::string> & candidates)
{
  auto match = closest_match(name, candidates);
  return match ? " (did you mean '" + *match + "'?)" : std::string();
}

}  // namespace kairos
