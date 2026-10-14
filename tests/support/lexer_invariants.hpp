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
/// \brief Properties every tokenization must satisfy, shared by tests and the fuzzer.

#ifndef KAIROS__TESTS__SUPPORT__LEXER_INVARIANTS_HPP_
#define KAIROS__TESTS__SUPPORT__LEXER_INVARIANTS_HPP_

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "pddl/lexer.hpp"

namespace kairos::pddl::testing
{

/**
 * @brief Checks `tokenize(source)`; returns a description of the first violation, if any.
 *
 * - Exactly one End token, at the end.
 * - Every other token has non-empty text that is a view into `source`, in increasing order and
 *   without overlap.
 * - Each token's line and column match its offset (recomputed independently).
 * - Text between tokens is only whitespace, comments or a leading BOM.
 */
inline std::optional<std::string> check_lexer_invariants(std::string_view source)
{
  std::vector<Token> tokens = tokenize(source);
  if (tokens.empty() || tokens.back().kind != TokenKind::End) {
    return "missing final End token";
  }

  size_t cursor = source.starts_with("\xEF\xBB\xBF") ? 3 : 0;
  // Line and column of `cursor`, advanced alongside it
  uint32_t line = 1;
  uint32_t column = 1;
  auto walk_to = [&](size_t target) {
      while (cursor < target) {
        char c = source[cursor];
        if (c == '\r' && cursor + 1 < source.size() && source[cursor + 1] == '\n') {
          ++cursor;
        }
        if (c == '\n' || c == '\r') {
          ++line;
          column = 1;
        } else {
          ++column;
        }
        ++cursor;
      }
    };

  for (size_t i = 0; i < tokens.size(); ++i) {
    const Token & t = tokens[i];
    bool last = i + 1 == tokens.size();
    if ((t.kind == TokenKind::End) != last) {
      return "End token at position " + std::to_string(i) + " of " +
             std::to_string(tokens.size());
    }
    const char * begin = source.data();
    if (t.joined) {
      // "? x": the '?', blanks, then the name (the text is "?x", kept apart)
      size_t q = source.find('?', cursor);
      if (t.kind != TokenKind::Variable || q == std::string_view::npos) {
        return "joined token without its '?'";
      }
      size_t at = q + 1;
      while (at < source.size() && (source[at] == ' ' || source[at] == '\t')) {
        ++at;
      }
      if (source.substr(at, t.text.size() - 1) != t.text.substr(1)) {
        return "joined token whose name is not after its '?'";
      }
      walk_to(q);
      if (t.line != line || t.column != column) {
        return "wrong position for joined token " + std::to_string(i);
      }
      walk_to(at + t.text.size() - 1);
      continue;
    }
    if (t.text.data() < begin || t.text.data() + t.text.size() > begin + source.size()) {
      if (!(last && t.text.empty())) {
        return "token text outside the source";
      }
    }
    size_t offset = last ? source.size() : static_cast<size_t>(t.text.data() - begin);
    if (offset < cursor) {
      return "tokens overlap or go backwards at offset " + std::to_string(offset);
    }

    // Skipped text: whitespace and comments only
    bool in_comment = false;
    for (size_t k = cursor; k < offset; ++k) {
      char c = source[k];
      if (c == '\n' || c == '\r') {
        in_comment = false;
      } else if (c == ';') {
        in_comment = true;
      } else if (!in_comment && c != ' ' && c != '\t' && c != '\f' && c != '\v') {
        return "non-blank text skipped at offset " + std::to_string(k);
      }
    }

    walk_to(offset);
    if (t.line != line || t.column != column) {
      return "wrong position for token " + std::to_string(i) + ": got " +
             std::to_string(t.line) + ":" + std::to_string(t.column) + ", expected " +
             std::to_string(line) + ":" + std::to_string(column);
    }
    if (!last) {
      if (t.text.empty()) {
        return "empty token text at offset " + std::to_string(offset);
      }
      walk_to(offset + t.text.size());
    }
  }
  return std::nullopt;
}

}  // namespace kairos::pddl::testing

#endif  // KAIROS__TESTS__SUPPORT__LEXER_INVARIANTS_HPP_
