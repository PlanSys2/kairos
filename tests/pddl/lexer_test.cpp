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

#include <gtest/gtest.h>

#include <cstdint>
#include <random>
#include <string>
#include <string_view>
#include <vector>

#include "pddl/lexer.hpp"
#include "support/lexer_invariants.hpp"

namespace kairos::pddl
{
namespace
{

using K = TokenKind;

// Kinds of all tokens except the final End
std::vector<K> kinds(std::string_view source)
{
  std::vector<K> result;
  for (const auto & t : tokenize(source)) {
    if (t.kind != K::End) {
      result.push_back(t.kind);
    }
  }
  return result;
}

std::vector<std::string> texts(std::string_view source)
{
  std::vector<std::string> result;
  for (const auto & t : tokenize(source)) {
    if (t.kind != K::End) {
      result.emplace_back(t.text);
    }
  }
  return result;
}

Token single(std::string_view source)
{
  auto tokens = tokenize(source);
  EXPECT_EQ(tokens.size(), 2u) << "expected one token in '" << source << "'";
  return tokens.front();
}

// --- Token kinds ---------------------------------------------------------------------------

TEST(LexerTest, Parentheses)
{
  EXPECT_EQ(kinds("()"), (std::vector<K>{K::LParen, K::RParen}));
  EXPECT_EQ(kinds(")("), (std::vector<K>{K::RParen, K::LParen}));
  EXPECT_EQ(kinds("((()))").size(), 6u);
}

TEST(LexerTest, Names)
{
  for (std::string_view name : {"robot", "move-robot", "at_robby", "r1", "Kitchen", "wp.1",
      "a@b", "3rd-floor", "x", "--"})
  {
    Token t = single(name);
    EXPECT_EQ(t.kind, K::Name) << name;
    EXPECT_EQ(t.text, name);
  }
}

TEST(LexerTest, NamesKeepTheirCase)
{
  EXPECT_EQ(single("MoveRobot").text, "MoveRobot");
}

TEST(LexerTest, VariablesAndKeywords)
{
  Token v = single("?robot-1");
  EXPECT_EQ(v.kind, K::Variable);
  EXPECT_EQ(v.text, "?robot-1");
  Token k = single(":durative-action");
  EXPECT_EQ(k.kind, K::Keyword);
  EXPECT_EQ(k.text, ":durative-action");
  EXPECT_EQ(single("?X").kind, K::Variable);
  EXPECT_EQ(single(":Requirements").kind, K::Keyword);
}

TEST(LexerTest, Operators)
{
  for (std::string_view op : {"+", "-", "*", "/", "<", "<=", ">", ">=", "=", "^"}) {
    Token t = single(op);
    EXPECT_EQ(t.kind, K::Operator) << op;
    EXPECT_EQ(t.text, op);
  }
}

TEST(LexerTest, ContinuousTime)
{
  EXPECT_EQ(single("#t").kind, K::ContinuousT);
  EXPECT_EQ(kinds("(* #t 2)"),
    (std::vector<K>{K::LParen, K::Operator, K::ContinuousT, K::Number, K::RParen}));
}

TEST(LexerTest, MinusIsOperatorOrNumberSignOrPartOfAName)
{
  EXPECT_EQ(kinds("(- ?x 1)"),
    (std::vector<K>{K::LParen, K::Operator, K::Variable, K::Number, K::RParen}));
  EXPECT_EQ(single("-1").kind, K::Number);
  EXPECT_EQ(single("-1").number, -1.0);
  EXPECT_EQ(single("-x").kind, K::Name);
  // Typed list separator
  EXPECT_EQ(texts("?r - robot"), (std::vector<std::string>{"?r", "-", "robot"}));
}

// --- Numbers -------------------------------------------------------------------------------

TEST(LexerTest, NumberValues)
{
  struct Case
  {
    std::string_view text;
    double value;
  };
  for (const auto & c : std::vector<Case>{
        {"0", 0.0}, {"42", 42.0}, {"007", 7.0}, {"1.5", 1.5}, {"-2.25", -2.25}, {"+3", 3.0},
        {"2e3", 2000.0}, {"2E-2", 0.02}, {"1.5e+1", 15.0}, {"-0", -0.0}, {"0.001", 0.001}})
  {
    Token t = single(c.text);
    EXPECT_EQ(t.kind, K::Number) << c.text;
    EXPECT_DOUBLE_EQ(t.number, c.value) << c.text;
    EXPECT_EQ(t.text, c.text);
  }
}

TEST(LexerTest, AlmostNumbersAreNames)
{
  // Not valid PDDL numbers; they become names and the parser rejects them where a number goes
  for (std::string_view w : {"1.", ".5", "1e", "1.2.3", "12abc"}) {
    EXPECT_EQ(single(w).kind, K::Name) << w;
  }
}

TEST(LexerTest, NumberOutOfRange)
{
  Token t = single("1e400");
  EXPECT_EQ(t.kind, K::Error);
  EXPECT_EQ(t.error, LexError::NumberOutOfRange);
  EXPECT_EQ(single("-1e400").error, LexError::NumberOutOfRange);
  // Tiny values underflow towards zero but are still representable or rounded
  Token tiny = single("1e-300");
  EXPECT_EQ(tiny.kind, K::Number);
}

TEST(LexerTest, VeryLongNumber)
{
  std::string digits(400, '9');
  EXPECT_EQ(single(digits).error, LexError::NumberOutOfRange);
  std::string long_fraction = "0." + std::string(500, '1');
  Token t = single(long_fraction);
  EXPECT_EQ(t.kind, K::Number);
  EXPECT_NEAR(t.number, 0.1111111111, 1e-9);
}

// --- Word boundaries -----------------------------------------------------------------------

TEST(LexerTest, ParenthesesSplitWords)
{
  EXPECT_EQ(texts("(at ?r ?l)"), (std::vector<std::string>{"(", "at", "?r", "?l", ")"}));
  EXPECT_EQ(texts("(a(b)c)"), (std::vector<std::string>{"(", "a", "(", "b", ")", "c", ")"}));
  EXPECT_EQ(texts("?x)"), (std::vector<std::string>{"?x", ")"}));
}

TEST(LexerTest, CommentsSplitWordsAndRunToEndOfLine)
{
  EXPECT_EQ(texts("robot; a comment (with parens)\nnext"),
    (std::vector<std::string>{"robot", "next"}));
  EXPECT_EQ(texts("; only a comment"), (std::vector<std::string>{}));
  EXPECT_EQ(texts("a ;; b\r\nc"), (std::vector<std::string>{"a", "c"}));
  EXPECT_EQ(texts("a;\rb"), (std::vector<std::string>{"a", "b"}));
}

TEST(LexerTest, AllWhitespaceKinds)
{
  EXPECT_EQ(texts(" a\tb\nc\r\nd\re\ff\vg "),
    (std::vector<std::string>{"a", "b", "c", "d", "e", "f", "g"}));
}

// --- Positions -----------------------------------------------------------------------------

TEST(LexerTest, LinesAndColumns)
{
  auto tokens = tokenize("(define\n  (domain d))");
  ASSERT_EQ(tokens.size(), 8u);
  EXPECT_EQ(tokens[0].line, 1u);  // (
  EXPECT_EQ(tokens[0].column, 1u);
  EXPECT_EQ(tokens[1].column, 2u);  // define
  EXPECT_EQ(tokens[2].line, 2u);  // (
  EXPECT_EQ(tokens[2].column, 3u);
  EXPECT_EQ(tokens[3].column, 4u);  // domain
  EXPECT_EQ(tokens[4].column, 11u);  // d
  EXPECT_EQ(tokens[7].kind, K::End);
  EXPECT_EQ(tokens[7].line, 2u);
  EXPECT_EQ(tokens[7].column, 14u);
}

TEST(LexerTest, WindowsAndOldMacLineEndings)
{
  auto crlf = tokenize("a\r\nb\r\n\r\nc");
  EXPECT_EQ(crlf[1].line, 2u);
  EXPECT_EQ(crlf[2].line, 4u);
  EXPECT_EQ(crlf[2].column, 1u);
  auto cr = tokenize("a\rb\r\rc");
  EXPECT_EQ(cr[1].line, 2u);
  EXPECT_EQ(cr[2].line, 4u);
}

TEST(LexerTest, TabsCountAsOneColumn)
{
  auto tokens = tokenize("\t\tx");
  EXPECT_EQ(tokens[0].column, 3u);
}

TEST(LexerTest, PositionsAfterComments)
{
  auto tokens = tokenize("; header\n; more\n  (x)");
  EXPECT_EQ(tokens[0].line, 3u);
  EXPECT_EQ(tokens[0].column, 3u);
}

TEST(LexerTest, ByteOrderMarkIsSkipped)
{
  auto tokens = tokenize("\xEF\xBB\xBF(define)");
  ASSERT_EQ(tokens.size(), 4u);
  EXPECT_EQ(tokens[0].kind, K::LParen);
  EXPECT_EQ(tokens[0].column, 1u);
  // A BOM elsewhere is not skipped
  EXPECT_EQ(tokenize("a \xEF\xBB\xBF")[1].kind, K::Error);
}

// --- Errors ---------------------------------------------------------------------------------

TEST(LexerTest, InvalidCharactersGiveErrorTokens)
{
  for (std::string_view w : {"a{b", "caf\xC3\xA9", "#", "#tt", "?x!", ":key$", "[x]", "\"str\"",
      "a<b", "{", "1e+"})
  {
    Token t = single(w);
    EXPECT_EQ(t.kind, K::Error) << w;
    EXPECT_EQ(t.error, LexError::InvalidCharacter) << w;
    EXPECT_EQ(t.text, w);
  }
}

TEST(LexerTest, NulByteIsInvalid)
{
  std::string source("a\0b", 3);
  Token t = single(source);
  EXPECT_EQ(t.kind, K::Error);
  EXPECT_EQ(t.text.size(), 3u);
}

TEST(LexerTest, EmptyPrefixes)
{
  EXPECT_EQ(single("?").error, LexError::EmptyPrefix);
  EXPECT_EQ(single(":").error, LexError::EmptyPrefix);
  EXPECT_EQ(kinds("(?)"), (std::vector<K>{K::LParen, K::Error, K::RParen}));
}

TEST(LexerTest, LexingContinuesAfterErrors)
{
  EXPECT_EQ(kinds("(a{ b)"), (std::vector<K>{K::LParen, K::Error, K::Name, K::RParen}));
  auto tokens = tokenize("x\n  b@d{ y");
  EXPECT_EQ(tokens[1].kind, K::Error);
  EXPECT_EQ(tokens[1].line, 2u);
  EXPECT_EQ(tokens[1].column, 3u);
  EXPECT_EQ(tokens[2].text, "y");
}

TEST(LexerTest, ErrorDiagnostics)
{
  auto tokens = tokenize("\n  a{b 1e999 ?");
  auto invalid = to_diagnostic(tokens[0], "d.pddl");
  EXPECT_EQ(invalid.code, "lex.invalid_character");
  EXPECT_EQ(invalid.severity, Severity::Error);
  EXPECT_EQ(*invalid.location, (SourceLocation{"d.pddl", 2, 3}));
  EXPECT_NE(invalid.message.find("a{b"), std::string::npos);
  EXPECT_EQ(to_diagnostic(tokens[1], "d.pddl").code, "lex.number_out_of_range");
  EXPECT_EQ(to_diagnostic(tokens[2], "d.pddl").code, "lex.empty_name");
  EXPECT_EQ(to_diagnostic(tokens[2], "d.pddl").location->column, 13u);
}

// --- End of input, peek ---------------------------------------------------------------------

TEST(LexerTest, EmptyAndBlankInputs)
{
  for (std::string_view s : {"", "   ", "\n\n", "; comment", "; comment\n", "\xEF\xBB\xBF"}) {
    auto tokens = tokenize(s);
    ASSERT_EQ(tokens.size(), 1u) << "'" << s << "'";
    EXPECT_EQ(tokens[0].kind, K::End);
  }
  auto end = tokenize("\n\n  ");
  EXPECT_EQ(end[0].line, 3u);
  EXPECT_EQ(end[0].column, 3u);
}

TEST(LexerTest, EndRepeatsForever)
{
  Lexer lexer("x");
  EXPECT_EQ(lexer.next().kind, K::Name);
  for (int i = 0; i < 5; ++i) {
    EXPECT_EQ(lexer.next().kind, K::End);
    EXPECT_EQ(lexer.peek().kind, K::End);
  }
}

TEST(LexerTest, PeekDoesNotConsume)
{
  Lexer lexer("(a b)");
  EXPECT_EQ(lexer.peek().kind, K::LParen);
  EXPECT_EQ(lexer.peek().kind, K::LParen);
  EXPECT_EQ(lexer.next().kind, K::LParen);
  EXPECT_EQ(lexer.peek().text, "a");
  EXPECT_EQ(lexer.next().text, "a");
  EXPECT_EQ(lexer.next().text, "b");
  EXPECT_EQ(lexer.peek().kind, K::RParen);
  EXPECT_EQ(lexer.next().kind, K::RParen);
  EXPECT_EQ(lexer.next().kind, K::End);
}

TEST(LexerTest, PeekKeepsPositions)
{
  Lexer lexer("a\n b");
  lexer.next();
  const Token & p = lexer.peek();
  EXPECT_EQ(p.line, 2u);
  EXPECT_EQ(p.column, 2u);
  Token n = lexer.next();
  EXPECT_EQ(n.line, 2u);
  EXPECT_EQ(n.column, 2u);
}

// --- Whole fragments ------------------------------------------------------------------------

TEST(LexerTest, DurativeActionFragment)
{
  const char * src =
    "(:durative-action move\n"
    "  :parameters (?r - robot ?from ?to - room)\n"
    "  :duration (= ?duration (/ (distance ?from ?to) (speed ?r)))\n"
    "  :condition (and (at start (at ?r ?from)) (over all (>= (battery ?r) 10)))\n"
    "  :effect (and (at end (at ?r ?to)) (at start (decrease (battery ?r) 5.5))))\n";
  auto tokens = tokenize(src);
  int names = 0, variables = 0, keywords = 0, numbers = 0, operators = 0, parens = 0;
  for (const auto & t : tokens) {
    switch (t.kind) {
      case K::Name: ++names; break;
      case K::Variable: ++variables; break;
      case K::Keyword: ++keywords; break;
      case K::Number: ++numbers; break;
      case K::Operator: ++operators; break;
      case K::LParen: case K::RParen: ++parens; break;
      case K::Error: ADD_FAILURE() << "unexpected error token " << t.text; break;
      default: break;
    }
  }
  EXPECT_EQ(keywords, 5);
  EXPECT_EQ(numbers, 2);
  EXPECT_EQ(operators, 5);  // - - = / >=
  EXPECT_EQ(variables, 13);
  EXPECT_EQ(parens % 2, 0);
  EXPECT_GT(names, 10);
  EXPECT_EQ(testing::check_lexer_invariants(src), std::nullopt);
}

TEST(LexerTest, IsNameCharacter)
{
  EXPECT_TRUE(is_name_character('a'));
  EXPECT_TRUE(is_name_character('Z'));
  EXPECT_TRUE(is_name_character('7'));
  EXPECT_TRUE(is_name_character('-'));
  EXPECT_TRUE(is_name_character('_'));
  EXPECT_FALSE(is_name_character('?'));
  EXPECT_FALSE(is_name_character('('));
  EXPECT_FALSE(is_name_character(' '));
  EXPECT_FALSE(is_name_character('\xC3'));
}

TEST(LexerTest, KindNames)
{
  EXPECT_EQ(to_string(K::LParen), "'('");
  EXPECT_EQ(to_string(K::Name), "name");
  EXPECT_EQ(to_string(K::End), "end of input");
  EXPECT_EQ(to_string(K::ContinuousT), "'#t'");
}

TEST(LexerTest, LargeInputLineNumbers)
{
  std::string src;
  for (int i = 0; i < 200000; ++i) {
    src += "(at r" + std::to_string(i) + " room)\n";
  }
  auto tokens = tokenize(src);
  ASSERT_EQ(tokens.size(), 200000u * 5 + 1);
  const Token & last_name = tokens[tokens.size() - 3];
  EXPECT_EQ(last_name.text, "room");
  EXPECT_EQ(last_name.line, 200000u);
  EXPECT_EQ(tokens.back().line, 200001u);
}

// --- Properties on random input -------------------------------------------------------------

std::string random_source(std::mt19937 & rng, size_t length)
{
  // Biased towards PDDL-like characters, plus some bytes that must be rejected
  static const std::string alphabet =
    "()()()  \n\t\r;?:-+*/<>=#.@_aAzZ09019e\xEF\xBB\xBF{}\"\x01\x80\xFF";
  std::uniform_int_distribution<size_t> pick(0, alphabet.size() - 1);
  std::string s;
  s.reserve(length);
  for (size_t i = 0; i < length; ++i) {
    s.push_back(alphabet[pick(rng)]);
  }
  return s;
}

TEST(LexerPropertyTest, InvariantsHoldOnRandomInput)
{
  for (uint32_t seed = 0; seed < 2000; ++seed) {
    std::mt19937 rng(seed);
    std::string src = random_source(rng, seed % 300);
    auto violation = testing::check_lexer_invariants(src);
    ASSERT_EQ(violation, std::nullopt) << "seed " << seed;
  }
}

TEST(LexerPropertyTest, InvariantsHoldOnHandPickedEdgeCases)
{
  for (std::string_view src : {"", "\r", "\r\n", "\n\r", ";", "(", ")", "?", ":", "#", "#t",
      "-", "--1", "1e", "\xEF\xBB", "\xEF\xBB\xBF\xEF\xBB\xBF", "a;b\rc", "\r\r\n\n", "( ;)\n)"})
  {
    EXPECT_EQ(testing::check_lexer_invariants(src), std::nullopt) << "'" << src << "'";
  }
}

}  // namespace
}  // namespace kairos::pddl
