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

#include "kairos/diagnostics.hpp"

#include <algorithm>
#include <string>
#include <utility>

namespace kairos
{

namespace
{

Diagnostic make(
  Severity severity, std::string code, std::string message,
  std::optional<SourceLocation> location)
{
  return Diagnostic{severity, std::move(code), std::move(message), std::move(location)};
}

}  // namespace

Diagnostic Diagnostic::error(
  std::string code, std::string message, std::optional<SourceLocation> location)
{
  return make(Severity::Error, std::move(code), std::move(message), std::move(location));
}

Diagnostic Diagnostic::warning(
  std::string code, std::string message, std::optional<SourceLocation> location)
{
  return make(Severity::Warning, std::move(code), std::move(message), std::move(location));
}

Diagnostic Diagnostic::note(
  std::string code, std::string message, std::optional<SourceLocation> location)
{
  return make(Severity::Note, std::move(code), std::move(message), std::move(location));
}

std::string_view to_string(Severity severity)
{
  switch (severity) {
    case Severity::Error: return "error";
    case Severity::Warning: return "warning";
    case Severity::Note: return "note";
  }
  return "unknown";
}

std::string to_string(const Diagnostic & diagnostic)
{
  std::string out;
  if (diagnostic.location) {
    const auto & loc = *diagnostic.location;
    out += loc.file.empty() ? std::string("<input>") : loc.file;
    if (loc.line > 0) {
      out += ":" + std::to_string(loc.line);
      if (loc.column > 0) {
        out += ":" + std::to_string(loc.column);
      }
    }
    out += ": ";
  }
  out += to_string(diagnostic.severity);
  out += ": ";
  out += diagnostic.message;
  if (!diagnostic.code.empty()) {
    out += " [" + diagnostic.code + "]";
  }
  return out;
}

bool has_errors(const std::vector<Diagnostic> & diagnostics)
{
  return std::any_of(
    diagnostics.begin(), diagnostics.end(),
    [](const Diagnostic & d) {return d.severity == Severity::Error;});
}

}  // namespace kairos
