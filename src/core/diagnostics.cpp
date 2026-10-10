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

std::string to_string(const Diagnostic & diagnostic, std::string_view source)
{
  std::string out = to_string(diagnostic);
  if (!diagnostic.location || diagnostic.location->line == 0) {
    return out;
  }
  // Find the line (same line-break rules as the lexer: \n, \r\n and \r)
  uint32_t line = 1;
  size_t start = source.starts_with("\xEF\xBB\xBF") ? 3 : 0;
  size_t i = start;
  while (line < diagnostic.location->line && i < source.size()) {
    char c = source[i++];
    if (c == '\r' && i < source.size() && source[i] == '\n') {
      ++i;
    }
    if (c == '\n' || c == '\r') {
      ++line;
      start = i;
    }
  }
  if (line != diagnostic.location->line || start > source.size()) {
    return out;
  }
  size_t end = start;
  while (end < source.size() && source[end] != '\n' && source[end] != '\r') {
    ++end;
  }
  std::string_view text = source.substr(start, end - start);
  std::string number = std::to_string(diagnostic.location->line);
  out += "\n " + number + " | " + std::string(text);
  if (diagnostic.location->column > 0 && diagnostic.location->column <= text.size() + 1) {
    // Keep tabs so the caret lines up with the text above
    std::string pad;
    for (size_t k = 0; k + 1 < diagnostic.location->column; ++k) {
      pad += text[k] == '\t' ? '\t' : ' ';
    }
    out += "\n " + std::string(number.size(), ' ') + " | " + pad + "^";
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
