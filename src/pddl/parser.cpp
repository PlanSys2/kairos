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

#include "pddl/parser.hpp"

#include <algorithm>
#include <array>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "pddl/lexer.hpp"
#include "pddl/printer.hpp"

namespace kairos::pddl
{

namespace
{

using ast::Loc;

constexpr std::array<std::string_view, 25> kKnownRequirements = {
  ":strips", ":typing", ":negative-preconditions", ":disjunctive-preconditions", ":equality",
  ":existential-preconditions", ":universal-preconditions", ":quantified-preconditions",
  ":conditional-effects", ":fluents", ":numeric-fluents", ":object-fluents", ":adl",
  ":durative-actions", ":duration-inequalities", ":continuous-effects", ":derived-predicates",
  ":timed-initial-literals", ":timed-initial-fluents", ":preferences", ":constraints",
  ":action-costs", ":time", ":non-deterministic", ":action-expansions"};

// Thrown to unwind to the nearest recovery point (the error is already recorded)
struct ParseError
{
};

// Thrown to stop parsing altogether (too many errors, end of input while recovering)
struct Abort
{
};

// Where formulas may use timed operators, modal operators and preferences
struct GdContext
{
  bool timed = false;        // at start / at end / over all
  bool constraints = false;  // always, sometime, within, ...
};

bool iequals(std::string_view a, std::string_view lower)
{
  if (a.size() != lower.size()) {
    return false;
  }
  for (size_t i = 0; i < a.size(); ++i) {
    char c = a[i];
    if (c >= 'A' && c <= 'Z') {
      c = static_cast<char>(c - 'A' + 'a');
    }
    if (c != lower[i]) {
      return false;
    }
  }
  return true;
}

class Parser
{
public:
  Parser(
    std::string_view text, std::string_view filename, const ParseOptions & options,
    SymbolTable & symbols, ast::Nodes & nodes)
  : lexer_(text), file_(filename), options_(options), symbols_(symbols), nodes_(nodes) {}

  void parse_domain(ast::Domain & domain)
  {
    domain_ = &domain;
    run(
      [&]() {
        parse_header("domain", domain.name, domain.loc);
        while (!at_close()) {
          section([&](const Token & keyword) {domain_section(keyword);});
        }
        finish();
      });
  }

  void parse_problem(ast::Problem & problem)
  {
    problem_ = &problem;
    run(
      [&]() {
        parse_header("problem", problem.name, problem.loc);
        while (!at_close()) {
          section([&](const Token & keyword) {problem_section(keyword);});
        }
        finish();
      });
  }

  std::vector<Diagnostic> take_diagnostics() {return std::move(diagnostics_);}

private:
  // --- Driver and recovery -------------------------------------------------------------------

  template<class F>
  void run(F && body)
  {
    try {
      body();
    } catch (const ParseError &) {
      // Error in the file header: nothing sensible to recover
    } catch (const Abort &) {
    }
  }

  // "(define (<kind> <name>)"
  void parse_header(std::string_view kind, Symbol & name, Loc & loc)
  {
    Token open = expect(TokenKind::LParen, "'(define'");
    loc = loc_of(open);
    expect_word("define");
    expect(TokenKind::LParen, "'(" + std::string(kind) + "'");
    expect_word(kind);
    name = intern(expect(TokenKind::Name, std::string(kind) + " name"));
    expect(TokenKind::RParen, "')'");
  }

  // Closing parenthesis of define, then nothing else
  void finish()
  {
    expect(TokenKind::RParen, "')'");
    // Stray closing parentheses are common and harmless; anything else is an error
    bool warned = false;
    while (peek().kind == TokenKind::RParen) {
      Token extra = raw_next();
      if (!warned) {
        warning("parse.extra_parentheses", "extra ')' after the end of the definition", extra);
        warned = true;
      }
    }
    const Token & extra = peek();
    if (extra.kind != TokenKind::End) {
      error("parse.trailing_tokens", "unexpected text after the end of the definition", extra);
    }
  }

  // Runs `body`, which parses one parenthesized element. On error, skips to the end of that
  // element; if the error came before anything was consumed, skips the offending token so the
  // caller's loop always makes progress.
  template<class F>
  void guarded(F && body)
  {
    uint32_t depth = depth_ + 1;
    uint64_t consumed = consumed_;
    try {
      body();
    } catch (const ParseError &) {
      if (depth_ >= depth) {
        recover(depth);
      } else if (consumed_ == consumed) {
        const Token & t = peek();
        if (t.kind == TokenKind::End) {
          eof_error(t);
        }
        if (t.kind != TokenKind::RParen) {
          raw_next();
        }
      }
    }
  }

  // A "(:keyword ...)" section
  template<class F>
  void section(F && body)
  {
    guarded(
      [&]() {
        expect(TokenKind::LParen, "'(' starting a section");
        Token keyword = expect(TokenKind::Keyword, "section keyword");
        body(keyword);
      });
  }

  // Skips tokens until the list opened at `depth` is closed
  void recover(uint32_t depth)
  {
    while (depth_ >= depth) {
      Token t = raw_next();
      if (t.kind == TokenKind::End) {
        if (!eof_reported_) {
          eof_reported_ = true;
          add(Diagnostic::error("parse.unexpected_end", "unexpected end of input", where(t)));
        }
        throw Abort{};
      }
    }
  }

  // --- Tokens --------------------------------------------------------------------------------

  // Next token, tracking depth, without reporting lexer errors (used when skipping)
  Token raw_next()
  {
    Token t = lexer_.next();
    ++consumed_;
    if (t.kind == TokenKind::LParen) {
      ++depth_;
    } else if (t.kind == TokenKind::RParen && depth_ > 0) {
      --depth_;
    }
    return t;
  }

  Token next()
  {
    Token t = raw_next();
    if (t.kind == TokenKind::Error) {
      add(to_diagnostic(t, file_));
      throw ParseError{};
    }
    if (t.kind == TokenKind::LParen && depth_ > options_.max_depth) {
      error("parse.nesting_too_deep",
        "nesting deeper than " + std::to_string(options_.max_depth) + " levels", t);
    }
    return t;
  }

  const Token & peek() {return lexer_.peek();}

  bool at_close()
  {
    const Token & t = peek();
    if (t.kind == TokenKind::End) {
      eof_error(t);
    }
    return t.kind == TokenKind::RParen;
  }

  Token expect(TokenKind kind, const std::string & what)
  {
    const Token & t = peek();
    if (t.kind != kind) {
      unexpected(t, what);
    }
    return next();
  }

  void expect_word(std::string_view lower)
  {
    const Token & t = peek();
    if (t.kind != TokenKind::Name || !iequals(t.text, lower)) {
      unexpected(t, "'" + std::string(lower) + "'");
    }
    next();
  }

  bool peek_is_word(std::string_view lower)
  {
    const Token & t = peek();
    return t.kind == TokenKind::Name && iequals(t.text, lower);
  }

  // --- Errors --------------------------------------------------------------------------------

  SourceLocation where(const Token & t) const {return SourceLocation{file_, t.line, t.column};}
  Loc loc_of(const Token & t) const {return Loc{t.line, t.column};}

  void add(Diagnostic d)
  {
    bool is_error = d.severity == Severity::Error;
    diagnostics_.push_back(std::move(d));
    if (is_error && ++errors_ >= options_.max_errors) {
      diagnostics_.push_back(
        Diagnostic::error(
          "parse.too_many_errors",
          "too many errors (" + std::to_string(errors_) + "), giving up"));
      throw Abort{};
    }
  }

  [[noreturn]] void error(const std::string & code, const std::string & message, const Token & t)
  {
    add(Diagnostic::error(code, message, where(t)));
    throw ParseError{};
  }

  void warning(const std::string & code, const std::string & message, const Token & t)
  {
    add(Diagnostic::warning(code, message, where(t)));
  }

  [[noreturn]] void eof_error(const Token & t)
  {
    if (!eof_reported_) {
      eof_reported_ = true;
      add(Diagnostic::error("parse.unexpected_end", "unexpected end of input", where(t)));
    }
    throw Abort{};
  }

  [[noreturn]] void unexpected(const Token & t, const std::string & expected)
  {
    if (t.kind == TokenKind::End) {
      eof_error(t);
    }
    if (t.kind == TokenKind::Error) {
      next();  // reports the lexer error and throws
    }
    std::string found = t.kind == TokenKind::LParen || t.kind == TokenKind::RParen ?
      std::string(to_string(t.kind)) :
      std::string(to_string(t.kind)) + " '" + std::string(t.text) + "'";
    error("parse.unexpected_token", "expected " + expected + ", found " + found, t);
  }

  Symbol intern(const Token & t) {return symbols_.intern(t.text);}

  // --- Typed lists ---------------------------------------------------------------------------

  // Either a type name or (either t1 t2 ...)
  std::vector<Symbol> parse_type()
  {
    Token t = peek();  // a copy: peek()'s reference changes as tokens are consumed
    if (t.kind == TokenKind::Name) {
      return {intern(next())};
    }
    if (t.kind == TokenKind::LParen) {
      next();
      expect_word("either");
      std::vector<Symbol> types;
      while (!at_close()) {
        types.push_back(intern(expect(TokenKind::Name, "type name")));
      }
      next();
      if (types.empty()) {
        error("parse.empty_either", "(either) needs at least one type", t);
      }
      return types;
    }
    unexpected(t, "type");
  }

  // "a b - t c - (either x y) d" up to (not including) ')'
  std::vector<ast::TypedName> parse_typed_list(TokenKind element, const char * what)
  {
    std::vector<ast::TypedName> result;
    size_t untyped_from = 0;
    while (!at_close()) {
      const Token & t = peek();
      if (is_glued_type(t)) {
        // "?x -robot": the type was written without a space after '-'
        Token glued = next();
        assign_type(result, untyped_from, {symbols_.intern(glued.text.substr(1))}, glued);
      } else if (t.kind == element) {
        Token name = next();
        result.push_back(ast::TypedName{intern(name), {}, loc_of(name)});
      } else if (t.kind == TokenKind::Operator && t.text == "-") {
        Token dash = next();
        assign_type(result, untyped_from, parse_type(), dash);
      } else {
        unexpected(t, what);
      }
    }
    return result;
  }

  static bool is_glued_type(const Token & t)
  {
    return t.kind == TokenKind::Name && t.text.size() > 1 && t.text.front() == '-' &&
           t.text[1] != '-';
  }

  // Gives `types` to the names added since `untyped_from`
  void assign_type(
    std::vector<ast::TypedName> & list, size_t & untyped_from, const std::vector<Symbol> & types,
    const Token & at)
  {
    if (untyped_from == list.size()) {
      warning("parse.type_without_names", "type given to an empty list of names; ignored", at);
    }
    for (size_t i = untyped_from; i < list.size(); ++i) {
      list[i].types = types;
    }
    untyped_from = list.size();
  }

  static void append(std::vector<ast::TypedName> & to, std::vector<ast::TypedName> more)
  {
    to.insert(to.end(), more.begin(), more.end());
  }

  // "(" typed variables ")"
  std::vector<ast::TypedName> parse_parameters()
  {
    expect(TokenKind::LParen, "'(' starting a parameter list");
    auto params = parse_typed_list(TokenKind::Variable, "variable");
    next();
    return params;
  }

  // --- Terms and expressions -----------------------------------------------------------------

  ast::TermId add_term(ast::Term term) {return nodes_.terms.push_back(std::move(term));}
  ast::ExprId add_expr(ast::Expr expr) {return nodes_.exprs.push_back(std::move(expr));}
  ast::FormulaId add_formula(ast::Formula f) {return nodes_.formulas.push_back(std::move(f));}
  ast::EffectId add_effect(ast::Effect e) {return nodes_.effects.push_back(std::move(e));}

  ast::TermId parse_term()
  {
    const Token & t = peek();
    if (t.kind == TokenKind::Name || t.kind == TokenKind::Variable) {
      Token tok = next();
      ast::Term term;
      term.kind = tok.kind == TokenKind::Name ? ast::TermKind::Name : ast::TermKind::Variable;
      term.name = intern(tok);
      term.loc = loc_of(tok);
      return add_term(std::move(term));
    }
    if (t.kind == TokenKind::LParen) {
      Token open = next();
      ast::Term term;
      term.kind = ast::TermKind::Function;
      term.name = intern(expect(TokenKind::Name, "function name"));
      term.loc = loc_of(open);
      while (!at_close()) {
        term.args.push_back(parse_term());
      }
      next();
      return add_term(std::move(term));
    }
    unexpected(t, "term");
  }

  std::vector<ast::TermId> parse_terms_until_close()
  {
    std::vector<ast::TermId> args;
    while (!at_close()) {
      args.push_back(parse_term());
    }
    next();
    return args;
  }

  ast::ExprId parse_expr()
  {
    const Token & t = peek();
    ast::Expr e;
    e.loc = loc_of(t);
    switch (t.kind) {
      case TokenKind::Number:
        e.kind = ast::ExprKind::Number;
        e.number = next().number;
        return add_expr(std::move(e));
      case TokenKind::ContinuousT:
        next();
        e.kind = ast::ExprKind::ContinuousT;
        return add_expr(std::move(e));
      case TokenKind::Variable:
        if (iequals(t.text, "?duration")) {
          next();
          e.kind = ast::ExprKind::Duration;
          return add_expr(std::move(e));
        }
        e.kind = ast::ExprKind::Term;
        e.term = parse_term();
        return add_expr(std::move(e));
      case TokenKind::Name:
        if (iequals(t.text, "total-time")) {
          next();
          e.kind = ast::ExprKind::TotalTime;
          return add_expr(std::move(e));
        }
        e.kind = ast::ExprKind::Term;
        e.term = parse_term();
        return add_expr(std::move(e));
      case TokenKind::LParen:
        return parse_compound_expr();
      default:
        unexpected(t, "numeric expression");
    }
  }

  ast::ExprId parse_compound_expr()
  {
    Token open = next();
    ast::Expr e;
    e.loc = loc_of(open);
    const Token & head = peek();
    if (head.kind == TokenKind::Operator) {
      Token op = next();
      while (!at_close()) {
        e.children.push_back(parse_expr());
      }
      next();
      size_t n = e.children.size();
      if (op.text == "+" || op.text == "*") {
        e.kind = op.text == "+" ? ast::ExprKind::Add : ast::ExprKind::Mul;
        if (n < 2) {
          error("parse.operator_arity", "'" + std::string(op.text) + "' needs two or more operands",
            op);
        }
      } else if (op.text == "-") {
        if (n != 1 && n != 2) {
          error("parse.operator_arity", "'-' needs one or two operands", op);
        }
        e.kind = n == 1 ? ast::ExprKind::Neg : ast::ExprKind::Sub;
      } else if (op.text == "/" || op.text == "^") {
        if (n != 2) {
          error("parse.operator_arity", "'" + std::string(op.text) + "' needs two operands", op);
        }
        e.kind = op.text == "/" ? ast::ExprKind::Div : ast::ExprKind::Pow;
      } else {
        error("parse.unexpected_token",
          "'" + std::string(op.text) + "' is not an arithmetic operator", op);
      }
      return add_expr(std::move(e));
    }
    if (head.kind == TokenKind::ContinuousT) {  // (#t), seen in PDDL+ domains
      next();
      expect(TokenKind::RParen, "')'");
      e.kind = ast::ExprKind::ContinuousT;
      return add_expr(std::move(e));
    }
    if (head.kind == TokenKind::Name) {
      Token name = next();
      if (iequals(name.text, "total-time")) {
        expect(TokenKind::RParen, "')'");
        e.kind = ast::ExprKind::TotalTime;
        return add_expr(std::move(e));
      }
      if (iequals(name.text, "is-violated")) {
        e.kind = ast::ExprKind::IsViolated;
        e.name = intern(expect(TokenKind::Name, "preference name"));
        expect(TokenKind::RParen, "')'");
        return add_expr(std::move(e));
      }
      e.kind = ast::ExprKind::Function;
      e.name = intern(name);
      e.args = parse_terms_until_close();
      return add_expr(std::move(e));
    }
    unexpected(head, "operator or function name");
  }

  // Target of an assignment: (f t1 ... tn), or a bare zero-arity function name
  ast::ExprId parse_function_target()
  {
    const Token & t = peek();
    ast::Expr e;
    e.kind = ast::ExprKind::Function;
    e.loc = loc_of(t);
    if (t.kind == TokenKind::Name) {
      e.name = intern(next());
      return add_expr(std::move(e));
    }
    if (t.kind == TokenKind::LParen) {
      next();
      e.name = intern(expect(TokenKind::Name, "function name"));
      e.args = parse_terms_until_close();
      return add_expr(std::move(e));
    }
    unexpected(t, "function");
  }

  // --- Formulas ------------------------------------------------------------------------------

  ast::FormulaId parse_gd(GdContext ctx)
  {
    Token open = expect(TokenKind::LParen, "'(' starting a formula");
    ast::Formula f;
    f.loc = loc_of(open);
    const Token & head = peek();

    if (head.kind == TokenKind::RParen) {  // ()
      next();
      f.kind = ast::FormulaKind::And;
      return add_formula(std::move(f));
    }

    if (head.kind == TokenKind::Operator) {
      Token op = next();
      f.kind = ast::FormulaKind::Compare;
      if (op.text == "=") {
        f.compare = ast::CompareOp::Eq;
      } else if (op.text == "<") {
        f.compare = ast::CompareOp::Lt;
      } else if (op.text == "<=") {
        f.compare = ast::CompareOp::Le;
      } else if (op.text == ">") {
        f.compare = ast::CompareOp::Gt;
      } else if (op.text == ">=") {
        f.compare = ast::CompareOp::Ge;
      } else {
        error("parse.unexpected_token",
          "'" + std::string(op.text) + "' is not a comparison", op);
      }
      f.lhs = parse_expr();
      f.rhs = parse_expr();
      expect(TokenKind::RParen, "')' closing the comparison");
      return add_formula(std::move(f));
    }

    if (head.kind != TokenKind::Name) {
      unexpected(head, "formula");
    }

    Token word = next();
    std::string_view w = word.text;
    GdContext inner{false, ctx.constraints};

    if (iequals(w, "and") || iequals(w, "or")) {
      f.kind = iequals(w, "and") ? ast::FormulaKind::And : ast::FormulaKind::Or;
      while (!at_close()) {
        f.children.push_back(parse_gd(ctx));
      }
      next();
      return add_formula(std::move(f));
    }
    if (iequals(w, "not")) {
      f.kind = ast::FormulaKind::Not;
      f.children.push_back(parse_gd(ctx));
      expect(TokenKind::RParen, "')' closing 'not'");
      return add_formula(std::move(f));
    }
    if (iequals(w, "imply")) {
      f.kind = ast::FormulaKind::Imply;
      f.children.push_back(parse_gd(ctx));
      f.children.push_back(parse_gd(ctx));
      expect(TokenKind::RParen, "')' closing 'imply'");
      return add_formula(std::move(f));
    }
    if (iequals(w, "exists") || iequals(w, "forall")) {
      f.kind = iequals(w, "exists") ? ast::FormulaKind::Exists : ast::FormulaKind::Forall;
      f.params = parse_parameters();
      f.children.push_back(parse_gd(ctx));
      expect(TokenKind::RParen, "')' closing the quantifier");
      return add_formula(std::move(f));
    }
    if (iequals(w, "preference")) {
      f.kind = ast::FormulaKind::Preference;
      if (peek().kind == TokenKind::Name) {
        f.name = intern(next());
      }
      f.children.push_back(parse_gd(ctx));
      expect(TokenKind::RParen, "')' closing 'preference'");
      return add_formula(std::move(f));
    }
    if (ctx.timed || ctx.constraints) {
      if (iequals(w, "at") && (peek_is_word("end") || (ctx.timed && peek_is_word("start")))) {
        f.kind = peek_is_word("end") ? ast::FormulaKind::AtEnd : ast::FormulaKind::AtStart;
        next();
        f.children.push_back(parse_gd(inner));
        expect(TokenKind::RParen, "')' closing the timed condition");
        return add_formula(std::move(f));
      }
      if (ctx.timed && iequals(w, "over") && peek_is_word("all")) {
        next();
        f.kind = ast::FormulaKind::OverAll;
        f.children.push_back(parse_gd(inner));
        expect(TokenKind::RParen, "')' closing 'over all'");
        return add_formula(std::move(f));
      }
    }
    if (ctx.constraints) {
      if (auto modal = parse_modal(word, f)) {
        return *modal;
      }
    }

    // Atom whose predicate is `word`
    f.kind = ast::FormulaKind::Atom;
    f.name = intern(word);
    f.args = parse_terms_until_close();
    return add_formula(std::move(f));
  }

  std::optional<ast::FormulaId> parse_modal(const Token & word, ast::Formula & f)
  {
    struct ModalSpec
    {
      std::string_view name;
      ast::ModalOp op;
      int times;
      int formulas;
    };
    static constexpr std::array<ModalSpec, 9> kModal = {{
      {"always", ast::ModalOp::Always, 0, 1},
      {"sometime", ast::ModalOp::Sometime, 0, 1},
      {"within", ast::ModalOp::Within, 1, 1},
      {"at-most-once", ast::ModalOp::AtMostOnce, 0, 1},
      {"sometime-after", ast::ModalOp::SometimeAfter, 0, 2},
      {"sometime-before", ast::ModalOp::SometimeBefore, 0, 2},
      {"always-within", ast::ModalOp::AlwaysWithin, 1, 2},
      {"hold-during", ast::ModalOp::HoldDuring, 2, 1},
      {"hold-after", ast::ModalOp::HoldAfter, 1, 1},
    }};
    for (const auto & spec : kModal) {
      if (!iequals(word.text, spec.name)) {
        continue;
      }
      f.kind = ast::FormulaKind::Modal;
      f.modal = spec.op;
      for (int i = 0; i < spec.times; ++i) {
        f.times.push_back(expect(TokenKind::Number, "time").number);
      }
      for (int i = 0; i < spec.formulas; ++i) {
        f.children.push_back(parse_gd(GdContext{false, true}));
      }
      expect(TokenKind::RParen, "')' closing '" + std::string(spec.name) + "'");
      return add_formula(std::move(f));
    }
    return std::nullopt;
  }

  // --- Effects -------------------------------------------------------------------------------

  ast::EffectId parse_effect(bool timed)
  {
    Token open = expect(TokenKind::LParen, "'(' starting an effect");
    ast::Effect e;
    e.loc = loc_of(open);
    const Token & head = peek();

    if (head.kind == TokenKind::RParen) {  // ()
      next();
      e.kind = ast::EffectKind::And;
      return add_effect(std::move(e));
    }
    if (head.kind != TokenKind::Name) {
      unexpected(head, "effect");
    }

    Token word = next();
    std::string_view w = word.text;

    if (iequals(w, "and")) {
      e.kind = ast::EffectKind::And;
      while (!at_close()) {
        e.children.push_back(parse_effect(timed));
      }
      next();
      return add_effect(std::move(e));
    }
    if (iequals(w, "not")) {
      e.kind = ast::EffectKind::Delete;
      expect(TokenKind::LParen, "'(' starting the deleted atom");
      e.name = intern(expect(TokenKind::Name, "predicate name"));
      e.args = parse_terms_until_close();
      expect(TokenKind::RParen, "')' closing 'not'");
      return add_effect(std::move(e));
    }
    if (iequals(w, "forall")) {
      e.kind = ast::EffectKind::Forall;
      e.params = parse_parameters();
      e.children.push_back(parse_effect(timed));
      expect(TokenKind::RParen, "')' closing 'forall'");
      return add_effect(std::move(e));
    }
    if (iequals(w, "when")) {
      e.kind = ast::EffectKind::When;
      e.condition = parse_gd(GdContext{timed, false});
      e.children.push_back(parse_effect(timed));
      expect(TokenKind::RParen, "')' closing 'when'");
      return add_effect(std::move(e));
    }
    if (timed && iequals(w, "at") && (peek_is_word("start") || peek_is_word("end"))) {
      e.kind = peek_is_word("start") ? ast::EffectKind::AtStart : ast::EffectKind::AtEnd;
      next();
      e.children.push_back(parse_effect(false));
      expect(TokenKind::RParen, "')' closing the timed effect");
      return add_effect(std::move(e));
    }
    if (auto op = assign_op(w)) {
      e.kind = ast::EffectKind::Assign;
      e.op = *op;
      e.target = parse_function_target();
      e.value = parse_expr();
      expect(TokenKind::RParen, "')' closing '" + std::string(w) + "'");
      return add_effect(std::move(e));
    }

    e.kind = ast::EffectKind::Add;
    e.name = intern(word);
    e.args = parse_terms_until_close();
    return add_effect(std::move(e));
  }

  static std::optional<ast::AssignOp> assign_op(std::string_view w)
  {
    if (iequals(w, "assign")) {
      return ast::AssignOp::Assign;
    }
    if (iequals(w, "increase")) {
      return ast::AssignOp::Increase;
    }
    if (iequals(w, "decrease")) {
      return ast::AssignOp::Decrease;
    }
    if (iequals(w, "scale-up")) {
      return ast::AssignOp::ScaleUp;
    }
    if (iequals(w, "scale-down")) {
      return ast::AssignOp::ScaleDown;
    }
    return std::nullopt;
  }

  // --- Domain --------------------------------------------------------------------------------

  // List sections (:objects, :init, ...) may repeat: their contents are joined, with a warning
  void list_section(const Token & keyword)
  {
    std::string key = SymbolTable::to_lower(keyword.text);
    if (!seen_sections_.insert(key).second) {
      warning("parse.duplicate_section", "section " + key + " repeated; contents joined", keyword);
    }
  }

  // Single-valued sections (:goal, :metric, ...): a repetition is a warning if identical to
  // the first one and an error otherwise. Returns true if this is a repetition.
  bool single_section(const Token & keyword)
  {
    return !seen_sections_.insert(SymbolTable::to_lower(keyword.text)).second;
  }

  void compare_repeated(const Token & keyword, bool same)
  {
    std::string key = SymbolTable::to_lower(keyword.text);
    if (same) {
      warning("parse.duplicate_section", "section " + key + " repeated identically", keyword);
    } else {
      add(
        Diagnostic::error(
          "parse.duplicate_section", "section " + key + " repeated with a different value",
          where(keyword)));
    }
  }

  // Parses a formula for a single-valued section, keeping the first one on repetition
  void single_formula(const Token & keyword, ast::FormulaId & slot, GdContext ctx)
  {
    bool repeated = single_section(keyword);
    ast::FormulaId f = parse_gd(ctx);
    if (repeated && slot.valid()) {
      compare_repeated(
        keyword, to_pddl(nodes_, symbols_, f) == to_pddl(nodes_, symbols_, slot));
    } else {
      slot = f;
    }
  }

  void parse_requirements(std::vector<ast::Requirement> & out)
  {
    while (!at_close()) {
      Token r = expect(TokenKind::Keyword, "requirement");
      std::string lower = SymbolTable::to_lower(r.text);
      if (!is_known_requirement(lower)) {
        warning("parse.unknown_requirement", "unknown requirement " + lower, r);
      }
      out.push_back(ast::Requirement{intern(r), loc_of(r)});
    }
    next();
  }

  void domain_section(const Token & keyword)
  {
    std::string_view k = keyword.text;
    ast::Domain & d = *domain_;
    if (iequals(k, ":requirements")) {
      list_section(keyword);
      parse_requirements(d.requirements);
    } else if (iequals(k, ":types")) {
      list_section(keyword);
      append(d.types, parse_typed_list(TokenKind::Name, "type name"));
      next();
    } else if (iequals(k, ":constants")) {
      list_section(keyword);
      append(d.constants, parse_typed_list(TokenKind::Name, "constant name"));
      next();
    } else if (iequals(k, ":predicates")) {
      list_section(keyword);
      while (!at_close()) {
        guarded([&]() {d.predicates.push_back(parse_skeleton());});
      }
      next();
    } else if (iequals(k, ":functions")) {
      list_section(keyword);
      parse_functions(d.functions);
    } else if (iequals(k, ":constraints")) {
      single_formula(keyword, d.constraints, GdContext{false, true});
      expect(TokenKind::RParen, "')' closing :constraints");
    } else if (iequals(k, ":action")) {
      d.actions.push_back(parse_action(keyword, false));
    } else if (iequals(k, ":process")) {
      d.processes.push_back(parse_action(keyword, true));
    } else if (iequals(k, ":event")) {
      d.events.push_back(parse_action(keyword, true));
    } else if (iequals(k, ":durative-action")) {
      d.durative_actions.push_back(parse_durative_action(keyword));
    } else if (iequals(k, ":constraint")) {
      d.global_constraints.push_back(parse_global_constraint(keyword));
    } else if (iequals(k, ":derived")) {
      ast::Derived derived;
      derived.loc = loc_of(keyword);
      derived.head = parse_skeleton();
      derived.body = parse_gd(GdContext{});
      expect(TokenKind::RParen, "')' closing :derived");
      d.derived.push_back(std::move(derived));
    } else {
      error("parse.unknown_section", "unknown domain section " + std::string(k), keyword);
    }
  }

  // "(name ?x - t ...)" for predicates, functions and derived heads
  ast::PredicateDecl parse_skeleton()
  {
    Token open = expect(TokenKind::LParen, "'(' starting a declaration");
    ast::PredicateDecl decl;
    decl.loc = loc_of(open);
    decl.name = intern(expect(TokenKind::Name, "name"));
    decl.params = parse_typed_list(TokenKind::Variable, "variable");
    next();
    return decl;
  }

  // Function skeletons, each group optionally followed by "- type"
  void parse_functions(std::vector<ast::FunctionDecl> & out)
  {
    size_t untyped_from = out.size();
    while (!at_close()) {
      const Token & t = peek();
      if (t.kind == TokenKind::LParen) {
        ast::PredicateDecl s = parse_skeleton();
        out.push_back(ast::FunctionDecl{s.name, std::move(s.params), {}, s.loc});
      } else if ((t.kind == TokenKind::Operator && t.text == "-") || is_glued_type(t)) {
        Token dash = next();
        std::vector<Symbol> type = dash.kind == TokenKind::Name ?
          std::vector<Symbol>{symbols_.intern(dash.text.substr(1))} : parse_type();
        if (untyped_from == out.size()) {
          warning("parse.type_without_names", "type given to no function; ignored", dash);
        }
        for (size_t i = untyped_from; i < out.size(); ++i) {
          out[i].type = type;
        }
        untyped_from = out.size();
      } else {
        unexpected(t, "function declaration");
      }
    }
    next();
  }

  ast::Action parse_action(const Token & keyword, bool process_or_event)
  {
    ast::Action a;
    a.loc = loc_of(keyword);
    a.name = intern(expect(TokenKind::Name, "action name"));
    std::set<std::string> seen;
    while (!at_close()) {
      Token key = expect(TokenKind::Keyword, "action keyword");
      std::string k = SymbolTable::to_lower(key.text);
      if (!seen.insert(k).second) {
        error("parse.duplicate_key", k + " appears more than once", key);
      }
      if (k == ":parameters") {
        auto params = parse_parameters();
        a.params.insert(a.params.begin(), params.begin(), params.end());
      } else if (k == ":vars" && !process_or_event) {
        // PDDL 1.2: free variables, equivalent to extra parameters
        warning("parse.vars_as_parameters", ":vars treated as additional parameters", key);
        auto vars = parse_parameters();
        a.params.insert(a.params.end(), vars.begin(), vars.end());
      } else if (k == ":precondition") {
        a.precondition = parse_gd(GdContext{});
      } else if (k == ":effect") {
        a.effect = parse_effect(false);
      } else {
        error("parse.unknown_key",
          "unknown key " + k + " in " + (process_or_event ? "process or event" : "action"), key);
      }
    }
    next();
    return a;
  }

  ast::GlobalConstraint parse_global_constraint(const Token & keyword)
  {
    ast::GlobalConstraint c;
    c.loc = loc_of(keyword);
    c.name = intern(expect(TokenKind::Name, "constraint name"));
    std::set<std::string> seen;
    while (!at_close()) {
      Token key = expect(TokenKind::Keyword, "constraint keyword");
      std::string k = SymbolTable::to_lower(key.text);
      if (!seen.insert(k).second) {
        error("parse.duplicate_key", k + " appears more than once", key);
      }
      if (k == ":parameters") {
        c.params = parse_parameters();
      } else if (k == ":condition") {
        c.condition = parse_gd(GdContext{});
      } else {
        error("parse.unknown_key", "unknown key " + k + " in constraint", key);
      }
    }
    if (!c.condition.valid()) {
      error("parse.missing_condition", "constraint without :condition", keyword);
    }
    next();
    return c;
  }

  ast::DurativeAction parse_durative_action(const Token & keyword)
  {
    ast::DurativeAction a;
    a.loc = loc_of(keyword);
    a.name = intern(expect(TokenKind::Name, "action name"));
    std::set<std::string> seen;
    while (!at_close()) {
      Token key = expect(TokenKind::Keyword, "durative action keyword");
      std::string k = SymbolTable::to_lower(key.text);
      if (!seen.insert(k).second) {
        error("parse.duplicate_key", k + " appears more than once", key);
      }
      if (k == ":parameters") {
        a.params = parse_parameters();
      } else if (k == ":duration") {
        a.duration = parse_gd(GdContext{true, false});
      } else if (k == ":condition") {
        a.condition = parse_gd(GdContext{true, false});
      } else if (k == ":effect") {
        a.effect = parse_effect(true);
      } else {
        error("parse.unknown_key", "unknown key " + k + " in durative action", key);
      }
    }
    if (!a.duration.valid()) {
      error("parse.missing_duration", "durative action without :duration", keyword);
    }
    next();
    return a;
  }

  // --- Problem -------------------------------------------------------------------------------

  void problem_section(const Token & keyword)
  {
    std::string_view k = keyword.text;
    ast::Problem & p = *problem_;
    if (iequals(k, ":domain")) {
      bool repeated = single_section(keyword);
      Symbol name = intern(expect(TokenKind::Name, "domain name"));
      if (repeated) {
        compare_repeated(keyword, name == p.domain_name);
      } else {
        p.domain_name = name;
      }
      expect(TokenKind::RParen, "')' closing :domain");
    } else if (iequals(k, ":requirements")) {
      list_section(keyword);
      parse_requirements(p.requirements);
    } else if (iequals(k, ":objects")) {
      list_section(keyword);
      append(p.objects, parse_typed_list(TokenKind::Name, "object name"));
      next();
    } else if (iequals(k, ":init")) {
      list_section(keyword);
      while (!at_close()) {
        guarded([&]() {p.init.push_back(parse_init_item(true));});
      }
      next();
    } else if (iequals(k, ":goal")) {
      single_formula(keyword, p.goal, GdContext{});
      expect(TokenKind::RParen, "')' closing :goal");
    } else if (iequals(k, ":constraints")) {
      single_formula(keyword, p.constraints, GdContext{false, true});
      expect(TokenKind::RParen, "')' closing :constraints");
    } else if (iequals(k, ":metric")) {
      bool repeated = single_section(keyword);
      ast::Metric m;
      m.loc = loc_of(keyword);
      if (peek_is_word("minimize")) {
        m.minimize = true;
      } else if (peek_is_word("maximize")) {
        m.minimize = false;
      } else {
        unexpected(peek(), "'minimize' or 'maximize'");
      }
      next();
      m.expr = parse_expr();
      expect(TokenKind::RParen, "')' closing :metric");
      if (repeated && p.metric) {
        compare_repeated(
          keyword, m.minimize == p.metric->minimize &&
          to_pddl(nodes_, symbols_, m.expr) == to_pddl(nodes_, symbols_, p.metric->expr));
      } else {
        p.metric = m;
      }
    } else if (iequals(k, ":length")) {
      // PDDL 1 leftover: ignored
      warning("parse.ignored_section", ":length is ignored", keyword);
      recover(depth_);
    } else {
      error("parse.unknown_section", "unknown problem section " + std::string(k), keyword);
    }
  }

  // (p a b), (not (p a b)), (= (f a) v), (at t <item>)
  ast::InitItem parse_init_item(bool allow_timed)
  {
    Token open = expect(TokenKind::LParen, "'(' starting an initial fact");
    ast::InitItem item;
    item.loc = loc_of(open);
    const Token & head = peek();

    if (head.kind == TokenKind::Operator && head.text == "=") {
      next();
      item.kind = ast::InitItem::Kind::Fluent;
      if (peek().kind == TokenKind::Name) {  // zero-arity function without parentheses
        item.name = intern(next());
      } else {
        expect(TokenKind::LParen, "'(' starting the function");
        item.name = intern(expect(TokenKind::Name, "function name"));
        item.args = parse_terms_until_close();
      }
      item.value = parse_expr();
      expect(TokenKind::RParen, "')' closing the initial value");
      return item;
    }
    if (head.kind != TokenKind::Name) {
      unexpected(head, "initial fact");
    }
    Token word = next();
    if (allow_timed && iequals(word.text, "at") && peek().kind == TokenKind::Number) {
      double time = next().number;
      ast::InitItem timed = parse_init_item(false);
      timed.time = time;
      timed.loc = item.loc;
      expect(TokenKind::RParen, "')' closing the timed initial literal");
      return timed;
    }
    if (iequals(word.text, "not")) {
      expect(TokenKind::LParen, "'(' starting the negated fact");
      item.negated = true;
      item.name = intern(expect(TokenKind::Name, "predicate name"));
      item.args = parse_terms_until_close();
      expect(TokenKind::RParen, "')' closing 'not'");
      return item;
    }
    item.name = intern(word);
    item.args = parse_terms_until_close();
    return item;
  }

  Lexer lexer_;
  std::string file_;
  ParseOptions options_;
  SymbolTable & symbols_;
  ast::Nodes & nodes_;
  ast::Domain * domain_ = nullptr;
  ast::Problem * problem_ = nullptr;
  std::vector<Diagnostic> diagnostics_;
  std::set<std::string> seen_sections_;
  uint32_t depth_ = 0;
  uint32_t errors_ = 0;
  uint64_t consumed_ = 0;  // tokens consumed so far, to detect progress
  bool eof_reported_ = false;
};

}  // namespace

bool is_known_requirement(std::string_view name)
{
  return std::find(kKnownRequirements.begin(), kKnownRequirements.end(), name) !=
         kKnownRequirements.end();
}

Result<ast::Domain> parse_domain(
  std::string_view text, std::string_view filename, const ParseOptions & options)
{
  ast::Domain domain;
  domain.file = std::string(filename);
  Parser parser(text, filename, options, domain.symbols, domain.nodes);
  parser.parse_domain(domain);
  auto diagnostics = parser.take_diagnostics();
  if (has_errors(diagnostics)) {
    return Result<ast::Domain>::failure(std::move(diagnostics));
  }
  return Result<ast::Domain>::success(std::move(domain), std::move(diagnostics));
}

Result<ast::Problem> parse_problem(
  std::string_view text, std::string_view filename, const ParseOptions & options)
{
  ast::Problem problem;
  problem.file = std::string(filename);
  Parser parser(text, filename, options, problem.symbols, problem.nodes);
  parser.parse_problem(problem);
  auto diagnostics = parser.take_diagnostics();
  if (has_errors(diagnostics)) {
    return Result<ast::Problem>::failure(std::move(diagnostics));
  }
  return Result<ast::Problem>::success(std::move(problem), std::move(diagnostics));
}

FileKind detect_file_kind(std::string_view text)
{
  Lexer lexer(text);
  if (lexer.next().kind != TokenKind::LParen) {
    return FileKind::Unknown;
  }
  Token define = lexer.next();
  if (define.kind != TokenKind::Name || !iequals(define.text, "define")) {
    return FileKind::Unknown;
  }
  if (lexer.next().kind != TokenKind::LParen) {
    return FileKind::Unknown;
  }
  Token kind = lexer.next();
  if (kind.kind == TokenKind::Name && iequals(kind.text, "domain")) {
    return FileKind::Domain;
  }
  if (kind.kind == TokenKind::Name && iequals(kind.text, "problem")) {
    return FileKind::Problem;
  }
  return FileKind::Unknown;
}

}  // namespace kairos::pddl
