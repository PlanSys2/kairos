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
/// \brief Helpers for PDDL front-end tests: test data paths, file reading, diagnostic matching.

#ifndef KAIROS__TESTS__SUPPORT__PDDL_HELPERS_HPP_
#define KAIROS__TESTS__SUPPORT__PDDL_HELPERS_HPP_

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "kairos/diagnostics.hpp"

#ifndef KAIROS_TEST_DATA_DIR
#error "KAIROS_TEST_DATA_DIR must point to tests/data"
#endif

namespace kairos::testing
{

inline std::filesystem::path data_path(const std::string & relative)
{
  return std::filesystem::path(KAIROS_TEST_DATA_DIR) / relative;
}

inline std::string read_file(const std::filesystem::path & path)
{
  std::ifstream in(path, std::ios::binary);
  std::stringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

/// @brief "code@line:column" for each diagnostic, easy to compare in tests.
inline std::vector<std::string> codes(const std::vector<Diagnostic> & diagnostics)
{
  std::vector<std::string> out;
  for (const auto & d : diagnostics) {
    std::string s = d.code;
    if (d.location) {
      s += "@" + std::to_string(d.location->line) + ":" + std::to_string(d.location->column);
    }
    out.push_back(s);
  }
  return out;
}

/// @brief All diagnostics, one per line, for failure messages.
inline std::string dump(const std::vector<Diagnostic> & diagnostics)
{
  std::string out;
  for (const auto & d : diagnostics) {
    out += to_string(d) + "\n";
  }
  return out;
}

}  // namespace kairos::testing

#endif  // KAIROS__TESTS__SUPPORT__PDDL_HELPERS_HPP_
