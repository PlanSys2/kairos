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

#include "pddl/lexer.hpp"

#include <array>
#include <charconv>
#include <cmath>
#include <functional>
#include <limits>
#include <mutex>
#include <set>
#include <string>
#include <utility>

namespace kairos::pddl
{

namespace
{

// "?" + name, kept for the whole run (tokens outlive their lexer in tokenize())
std::string_view joined_text(std::string_view name)
{
  static std::mutex mutex;
  static std::set<std::string, std::less<>> texts;
  std::string text = "?" + std::string(name);
  std::lock_guard<std::mutex> lock(mutex);
  return *texts.insert(std::move(text)).first;
}

enum CharClass : uint8_t
{
  kOther = 0,       // invalid inside a word
  kSpace = 1 << 0,
  kDelimiter = 1 << 1,  // ( ) ;
  kName = 1 << 2,   // letters, digits, - _ . @
  kOperator = 1 << 3,  // + - * / < > =
};

constexpr std::array<uint8_t, 256> make_classes()
{
  std::array<uint8_t, 256> t{};
  for (char c : std::string_view(" \t\n\r\f\v")) {
    t[static_cast<unsigned char>(c)] = kSpace;
  }
  for (char c : std::string_view("();")) {
    t[static_cast<unsigned char>(c)] = kDelimiter;
  }
  for (int c = 'a'; c <= 'z'; ++c) {
    t[static_cast<size_t>(c)] = kName;
    t[static_cast<size_t>(c - 'a' + 'A')] = kName;
  }
  for (int c = '0'; c <= '9'; ++c) {
    t[static_cast<size_t>(c)] = kName;
  }
  for (char c : std::string_view("_.@")) {
    t[static_cast<unsigned char>(c)] |= kName;
  }
  t[static_cast<unsigned char>('-')] = kName | kOperator;
  for (char c : std::string_view("+*/<>=^")) {
    t[static_cast<unsigned char>(c)] |= kOperator;
  }
  return t;
}

constexpr auto kClasses = make_classes();

uint8_t cls(char c) {return kClasses[static_cast<unsigned char>(c)];}
bool is_digit(char c) {return c >= '0' && c <= '9';}

bool is_operator_word(std::string_view w)
{
  return w == "+" || w == "-" || w == "*" || w == "/" || w == "<" || w == "<=" || w == ">" ||
         w == ">=" || w == "=" || w == "^";
}

// [-+]? digits ('.' digits)? ([eE] [-+]? digits)?
bool is_number_syntax(std::string_view w)
{
  size_t i = 0;
  auto digits = [&]() {
      size_t start = i;
      while (i < w.size() && is_digit(w[i])) {
        ++i;
      }
      return i > start;
    };
  if (i < w.size() && (w[i] == '-' || w[i] == '+')) {
    ++i;
  }
  if (!digits()) {
    return false;
  }
  if (i < w.size() && w[i] == '.') {
    ++i;
    if (!digits()) {
      return false;
    }
  }
  if (i < w.size() && (w[i] == 'e' || w[i] == 'E')) {
    ++i;
    if (i < w.size() && (w[i] == '-' || w[i] == '+')) {
      ++i;
    }
    if (!digits()) {
      return false;
    }
  }
  return i == w.size();
}

bool all_name_chars(std::string_view w)
{
  for (char c : w) {
    if (!(cls(c) & kName)) {
      return false;
    }
  }
  return true;
}

uint32_t saturating_add(uint32_t a, size_t b)
{
  constexpr uint32_t max = std::numeric_limits<uint32_t>::max();
  return b >= max - a ? max : static_cast<uint32_t>(a + b);
}

}  // namespace

bool is_name_character(char c)
{
  return (cls(c) & kName) != 0;
}

Lexer::Lexer(std::string_view source)
: source_(source)
{
  if (source_.starts_with("\xEF\xBB\xBF")) {
    pos_ = 3;  // UTF-8 BOM: not counted as columns
  }
}

const Token & Lexer::peek()
{
  if (!peeked_) {
    peeked_ = scan();
  }
  return *peeked_;
}

Token Lexer::next()
{
  if (peeked_) {
    Token t = *peeked_;
    peeked_.reset();
    return t;
  }
  return scan();
}

void Lexer::advance(size_t n)
{
  pos_ += n;
  column_ = saturating_add(column_, n);
}

void Lexer::skip_space_and_comments()
{
  while (pos_ < source_.size()) {
    char c = source_[pos_];
    if (c == '\n' || c == '\r') {
      // "\r\n" is one line break
      bool crlf = c == '\r' && pos_ + 1 < source_.size() && source_[pos_ + 1] == '\n';
      pos_ += crlf ? 2u : 1u;
      line_ = saturating_add(line_, 1);
      column_ = 1;
    } else if (cls(c) & kSpace) {
      advance(1);
    } else if (c == ';') {
      while (pos_ < source_.size() && source_[pos_] != '\n' && source_[pos_] != '\r') {
        advance(1);
      }
    } else {
      return;
    }
  }
}

Token Lexer::scan()
{
  skip_space_and_comments();

  Token t;
  t.line = line_;
  t.column = column_;

  if (pos_ >= source_.size()) {
    t.kind = TokenKind::End;
    t.text = source_.substr(source_.size());
    return t;
  }

  char c = source_[pos_];
  if (c == '(' || c == ')') {
    t.kind = c == '(' ? TokenKind::LParen : TokenKind::RParen;
    t.text = source_.substr(pos_, 1);
    advance(1);
    return t;
  }

  // A word runs until whitespace or a delimiter
  size_t end = pos_;
  while (end < source_.size() && !(cls(source_[end]) & (kSpace | kDelimiter))) {
    ++end;
  }
  std::string_view word = source_.substr(pos_, end - pos_);
  advance(word.size());
  t.text = word;

  auto error = [&t](LexError e) {
      t.kind = TokenKind::Error;
      t.error = e;
      return t;
    };

  if (word == "#t") {
    t.kind = TokenKind::ContinuousT;
    return t;
  }
  if (word.front() == '?' || word.front() == ':') {
    std::string_view rest = word.substr(1);
    if (rest.empty() && word.front() == '?') {
      // "? x": the variable ?x, if a name follows on the same line
      size_t at = pos_;
      while (at < source_.size() && (source_[at] == ' ' || source_[at] == '\t')) {
        ++at;
      }
      size_t stop = at;
      while (stop < source_.size() && !(cls(source_[stop]) & (kSpace | kDelimiter))) {
        ++stop;
      }
      std::string_view name = source_.substr(at, stop - at);
      if (!name.empty() && name.front() != '?' && name.front() != ':' && all_name_chars(name)) {
        advance(stop - pos_);
        t.text = joined_text(name);
        t.kind = TokenKind::Variable;
        t.joined = true;
        return t;
      }
    }
    if (rest.empty()) {
      return error(LexError::EmptyPrefix);
    }
    if (!all_name_chars(rest)) {
      return error(LexError::InvalidCharacter);
    }
    t.kind = word.front() == '?' ? TokenKind::Variable : TokenKind::Keyword;
    return t;
  }
  if (is_operator_word(word)) {
    t.kind = TokenKind::Operator;
    return t;
  }
  if (is_number_syntax(word)) {
    // from_chars does not accept a leading '+'
    std::string_view digits = word.front() == '+' ? word.substr(1) : word;
    double value = 0.0;
    auto [ptr, ec] = std::from_chars(digits.data(), digits.data() + digits.size(), value);
    if (ec == std::errc::result_out_of_range || !std::isfinite(value)) {
      return error(LexError::NumberOutOfRange);
    }
    if (ec != std::errc() || ptr != digits.data() + digits.size()) {
      return error(LexError::InvalidCharacter);
    }
    t.kind = TokenKind::Number;
    t.number = value;
    return t;
  }
  if (all_name_chars(word)) {
    t.kind = TokenKind::Name;
    return t;
  }
  return error(LexError::InvalidCharacter);
}

std::vector<Token> tokenize(std::string_view source)
{
  std::vector<Token> tokens;
  Lexer lexer(source);
  while (true) {
    tokens.push_back(lexer.next());
    if (tokens.back().kind == TokenKind::End) {
      return tokens;
    }
  }
}

std::string_view to_string(TokenKind kind)
{
  switch (kind) {
    case TokenKind::LParen: return "'('";
    case TokenKind::RParen: return "')'";
    case TokenKind::Name: return "name";
    case TokenKind::Variable: return "variable";
    case TokenKind::Keyword: return "keyword";
    case TokenKind::Number: return "number";
    case TokenKind::Operator: return "operator";
    case TokenKind::ContinuousT: return "'#t'";
    case TokenKind::End: return "end of input";
    case TokenKind::Error: return "invalid token";
  }
  return "unknown";
}

Diagnostic to_diagnostic(const Token & token, std::string_view filename)
{
  SourceLocation loc{std::string(filename), token.line, token.column};
  std::string word(token.text);
  switch (token.error) {
    case LexError::NumberOutOfRange:
      return Diagnostic::error("lex.number_out_of_range", "number out of range: " + word, loc);
    case LexError::EmptyPrefix:
      return Diagnostic::error(
        "lex.empty_name", "'" + word + "' must be followed by a name", loc);
    case LexError::InvalidCharacter:
    case LexError::None:
      break;
  }
  return Diagnostic::error("lex.invalid_character", "invalid characters in '" + word + "'", loc);
}

}  // namespace kairos::pddl
