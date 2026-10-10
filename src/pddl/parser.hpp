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
/// \brief PDDL parser: text to ast::Domain / ast::Problem, with error recovery.

#ifndef KAIROS__PDDL__PARSER_HPP_
#define KAIROS__PDDL__PARSER_HPP_

#include <cstdint>
#include <string_view>

#include "kairos/result.hpp"
#include "pddl/ast.hpp"

namespace kairos::pddl
{

struct ParseOptions
{
  uint32_t max_errors = 50;   // stop after this many errors
  uint32_t max_depth = 1000;  // maximum parenthesis nesting
};

/**
 * @brief Parses a domain.
 *
 * Accepts the whole PDDL 3.1 + PDDL+ syntax, whether or not the planner supports it. Sections may
 * come in any order. After an error inside a section or action, the parser skips to its closing
 * parenthesis and goes on, so one call reports as many errors as possible. Unknown requirements
 * are warnings. Any error makes the result a failure.
 */
Result<ast::Domain> parse_domain(
  std::string_view text, std::string_view filename = "domain.pddl",
  const ParseOptions & options = {});

/// @brief Parses a problem, on its own (names are checked against the domain later).
Result<ast::Problem> parse_problem(
  std::string_view text, std::string_view filename = "problem.pddl",
  const ParseOptions & options = {});

enum class FileKind : uint8_t {Domain, Problem, Unknown};

/// @brief Looks at "(define (domain" or "(define (problem" without parsing the rest.
FileKind detect_file_kind(std::string_view text);

/// @brief ENHSP's built-in unary math functions: sin cos tan asin acos atan abs.
bool is_math_function(std::string_view name);

/// @brief Requirement names Kairos knows (with the colon, lower case).
bool is_known_requirement(std::string_view name);

}  // namespace kairos::pddl

#endif  // KAIROS__PDDL__PARSER_HPP_
