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
/// \brief Diagnostics: errors, warnings and notes with an optional source position.

#ifndef KAIROS__DIAGNOSTICS_HPP_
#define KAIROS__DIAGNOSTICS_HPP_

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "kairos/export.hpp"

namespace kairos
{

enum class Severity
{
  Error,
  Warning,
  Note
};

/// @brief Position in a source file. Lines and columns start at 1.
struct SourceLocation
{
  std::string file;
  uint32_t line = 0;
  uint32_t column = 0;

  bool operator==(const SourceLocation & other) const = default;
};

/**
 * @brief A message about the input or the planning process.
 *
 * `code` is stable (e.g. "parse.unexpected_token") and is what tests and tools should match;
 * `message` is for humans and may change.
 */
struct KAIROS_API Diagnostic
{
  Severity severity = Severity::Error;
  std::string code;
  std::string message;
  std::optional<SourceLocation> location;

  bool operator==(const Diagnostic & other) const = default;

  static Diagnostic error(
    std::string code, std::string message,
    std::optional<SourceLocation> location = std::nullopt);
  static Diagnostic warning(
    std::string code, std::string message,
    std::optional<SourceLocation> location = std::nullopt);
  static Diagnostic note(
    std::string code, std::string message,
    std::optional<SourceLocation> location = std::nullopt);
};

/// @brief "error", "warning" or "note".
KAIROS_API std::string_view to_string(Severity severity);

/// @brief "file:line:column: severity: message [code]"; the position part is omitted if unknown.
KAIROS_API std::string to_string(const Diagnostic & diagnostic);

/**
 * @brief Like to_string(), followed by the offending source line and a caret under the column,
 * as compilers do:
 *
 *     domain.pddl:7:5: error: expected '(' starting a formula, found keyword ':effect'
 *         7 |     :effect (robot_at ?r ?to))
 *           |     ^
 *
 * `source` is the text of the file the diagnostic refers to. Without a usable location only
 * the first line is produced.
 */
KAIROS_API std::string to_string(const Diagnostic & diagnostic, std::string_view source);

/// @brief True if any diagnostic is an error.
KAIROS_API bool has_errors(const std::vector<Diagnostic> & diagnostics);

}  // namespace kairos

#endif  // KAIROS__DIAGNOSTICS_HPP_
