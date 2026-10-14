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
/// \brief PDDL lexer: splits source text into tokens with line and column.

#ifndef KAIROS__PDDL__LEXER_HPP_
#define KAIROS__PDDL__LEXER_HPP_

#include <cstdint>
#include <optional>
#include <string_view>
#include <vector>

#include "kairos/diagnostics.hpp"

namespace kairos::pddl
{

enum class TokenKind : uint8_t
{
  LParen,      // (
  RParen,      // )
  Name,        // move-robot, r1, 3rd-floor
  Variable,    // ?x
  Keyword,     // :action
  Number,      // 12, -1.5, 2e3
  Operator,    // + - * / < <= > >= = ^
  ContinuousT,  // #t
  End,         // end of input (returned forever once reached)
  Error        // see Token::error
};

enum class LexError : uint8_t
{
  None,
  InvalidCharacter,  // a word with characters not allowed in PDDL
  NumberOutOfRange,  // a number that does not fit a double
  EmptyPrefix        // "?" or ":" alone
};

/**
 * @brief A token. `text` points into the source, which must outlive the token.
 *
 * Lines and columns start at 1; columns count bytes (a tab is one column).
 */
struct Token
{
  TokenKind kind = TokenKind::End;
  std::string_view text;
  uint32_t line = 0;
  uint32_t column = 0;
  double number = 0.0;           // value of a Number token
  LexError error = LexError::None;
  bool joined = false;           // "? x" read as "?x" (SMTPlan's benchmarks): text kept apart
};

/**
 * @brief Streaming lexer.
 *
 * Text is split into words at whitespace, parentheses and ';' (comments run to the end of the
 * line). Each word is then classified, so "move-robot" is a Name, "-" an Operator and "-1" a
 * Number. Names are not lower-cased here: that happens when they are interned.
 *
 * Lenient on purpose, because real domains do these things: names may start with a digit or
 * contain '.', '@' and '_'; a UTF-8 BOM at the start is skipped; "\r\n" and a lone "\r" end
 * lines; "? x" is the variable "?x" (marked `joined`). Bad input never throws: it yields Error
 * tokens.
 */
class Lexer
{
public:
  explicit Lexer(std::string_view source);

  /// @brief Next token; End forever once the input is consumed.
  Token next();

  /// @brief Next token without consuming it.
  const Token & peek();

private:
  Token scan();
  void skip_space_and_comments();
  void advance(size_t n);

  std::string_view source_;
  size_t pos_ = 0;
  uint32_t line_ = 1;
  uint32_t column_ = 1;
  std::optional<Token> peeked_;
};

/// @brief All tokens of `source`, ending with one End token.
std::vector<Token> tokenize(std::string_view source);

/// @brief Human-readable kind name, e.g. "'('" or "name".
std::string_view to_string(TokenKind kind);

/// @brief Diagnostic for an Error token ("lex.invalid_character", ...).
Diagnostic to_diagnostic(const Token & token, std::string_view filename);

/// @brief Characters allowed inside a name besides letters and digits.
bool is_name_character(char c);

}  // namespace kairos::pddl

#endif  // KAIROS__PDDL__LEXER_HPP_
