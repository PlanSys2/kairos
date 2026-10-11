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
/// \brief Shell completion scripts of the kairos command (`kairos --completion SHELL`).

#ifndef KAIROS__CLI__COMPLETION_HPP_
#define KAIROS__CLI__COMPLETION_HPP_

#include <map>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace kairos::cli
{

/// @brief What the command accepts, so the scripts never drift from the parser.
struct CompletionSpec
{
  std::vector<std::string> commands;                   // solve, validate, check
  std::map<std::string, std::vector<std::string>> options;  // per command ("" before one)
  std::map<std::string, std::vector<std::string>> values;   // --engine -> auto m1 m2 m3
  std::map<std::string, std::vector<std::string>> lists;    // --debug=a,b (comma lists)
  std::vector<std::string> file_options;               // --output, --problem
  std::vector<std::string> dir_options;                // --dump-dir
  std::vector<std::string> number_options;             // --timeout... (nothing to offer)
};

/// @brief The script for "bash", "zsh" or "fish"; nullopt for another shell.
std::optional<std::string> completion_script(std::string_view shell, const CompletionSpec & spec);

}  // namespace kairos::cli

#endif  // KAIROS__CLI__COMPLETION_HPP_
