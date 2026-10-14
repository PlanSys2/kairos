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

#include "pddl/dialect.hpp"

#include <algorithm>
#include <map>
#include <set>
#include <string>
#include <utility>

namespace kairos::pddl
{

namespace
{

// '_' and '-' as the same character
std::string canonical(std::string_view name)
{
  std::string out(name);
  std::replace(out.begin(), out.end(), '_', '-');
  return out;
}

class Repair
{
public:
  Repair(const ast::Domain & domain, SymbolTable & symbols, ast::Nodes & nodes, std::string file)
  : symbols_(symbols), nodes_(nodes), file_(std::move(file))
  {
    for (const auto & p : domain.predicates) {
      add(predicates_, domain.symbols.name(p.name));
      arity_[std::string(domain.symbols.name(p.name))] = p.params.size();
    }
    for (const auto & dp : domain.derived) {
      add(predicates_, domain.symbols.name(dp.head.name));
    }
    for (const auto & f : domain.functions) {
      std::string n(domain.symbols.name(f.name));
      add(functions_, n);
      if (f.params.empty()) {
        nullary_.insert(n);
      }
    }
    for (const auto & c : domain.constants) {
      objects_.insert(std::string(domain.symbols.name(c.name)));
    }
  }

  void object(std::string_view name) {objects_.insert(std::string(name));}

  std::vector<Diagnostic> run(std::vector<ast::InitItem> * init)
  {
    for (uint32_t i = 0; i < nodes_.exprs.size(); ++i) {
      ast::Expr & e = nodes_.exprs[ast::ExprId(i)];
      if (e.kind == ast::ExprKind::Term) {
        bare_function(e);
      } else if (e.kind == ast::ExprKind::Function) {
        e.name = rename(e.name, functions_, e.loc);
      }
    }
    for (uint32_t i = 0; i < nodes_.terms.size(); ++i) {
      ast::Term & t = nodes_.terms[ast::TermId(i)];
      if (t.kind == ast::TermKind::Function) {
        t.name = rename(t.name, functions_, t.loc);
      }
    }
    for (uint32_t i = 0; i < nodes_.formulas.size(); ++i) {
      ast::Formula & f = nodes_.formulas[ast::FormulaId(i)];
      if (f.kind == ast::FormulaKind::Atom) {
        f.name = rename(f.name, predicates_, f.loc);
        std::string n(symbols_.name(f.name));
        if (n != "=" && !predicates_.declared.count(n)) {
          // A predicate the domain never declares is never true (as ENHSP): (or)
          warn("dialect.undeclared_predicate", "'" + n + "' is not declared: never true", f.loc);
          f.kind = ast::FormulaKind::Or;
          f.args.clear();
          f.children.clear();
        }
      }
    }
    for (uint32_t i = 0; i < nodes_.effects.size(); ++i) {
      ast::Effect & e = nodes_.effects[ast::EffectId(i)];
      if (e.kind == ast::EffectKind::Add || e.kind == ast::EffectKind::Delete) {
        e.name = rename(e.name, predicates_, e.loc);
      }
    }
    if (init) {
      for (auto & item : *init) {
        item.name = rename(
          item.name, item.kind == ast::InitItem::Kind::Fact ? predicates_ : functions_, item.loc);
      }
      // A fact with other arguments than its predicate's: ignored, as ENHSP does
      std::erase_if(*init, [&](const ast::InitItem & item) {
          if (item.kind != ast::InitItem::Kind::Fact) {
            return false;
          }
          auto it = arity_.find(std::string(symbols_.name(item.name)));
          if (it == arity_.end() || it->second == item.args.size()) {
            return false;
          }
          warn("dialect.init_arity", "in the initial state: '" +
            std::string(symbols_.name(item.name)) + "' takes " + std::to_string(it->second) +
            " argument(s), " + std::to_string(item.args.size()) + " given; the fact is ignored",
            item.loc);
          return true;
        });
    }
    return std::move(diags_);
  }

private:
  struct Names
  {
    std::set<std::string> declared;
    std::map<std::string, std::string> by_canonical;  // "" if two declared names collide
  };

  static void add(Names & names, std::string_view name)
  {
    std::string n(name);
    names.declared.insert(n);
    auto [it, fresh] = names.by_canonical.emplace(canonical(n), n);
    if (!fresh && it->second != n) {
      it->second.clear();
    }
  }

  // A name declared with '_' for '-' (or the reverse): the declared one
  Symbol rename(Symbol name, const Names & names, ast::Loc loc)
  {
    std::string n(symbols_.name(name));
    if (names.declared.count(n)) {
      return name;
    }
    auto it = names.by_canonical.find(canonical(n));
    if (it == names.by_canonical.end() || it->second.empty()) {
      return name;
    }
    if (warned_.insert(n).second) {
      warn("dialect.underscore", "'" + n + "' read as '" + it->second + "' ('_' and '-' differ)",
        loc);
    }
    return symbols_.intern(it->second);
  }

  // `q` for `(q)`: a function without arguments, not an object
  void bare_function(ast::Expr & e)
  {
    const ast::Term & t = nodes_.terms[e.term];
    if (t.kind != ast::TermKind::Name) {
      return;
    }
    std::string n(symbols_.name(t.name));
    if (objects_.count(n)) {
      return;
    }
    if (!nullary_.count(n)) {
      auto it = functions_.by_canonical.find(canonical(n));
      if (it == functions_.by_canonical.end() || !nullary_.count(it->second)) {
        return;
      }
    }
    Symbol name = rename(t.name, functions_, t.loc);
    if (warned_.insert("(" + n + ")").second) {
      warn("dialect.bare_function", "'" + n + "' read as the function (" +
        std::string(symbols_.name(name)) + ")", t.loc);
    }
    e.kind = ast::ExprKind::Function;
    e.name = name;
    e.args.clear();
  }

  void warn(const std::string & code, const std::string & msg, ast::Loc loc)
  {
    diags_.push_back(Diagnostic::warning(code, msg, SourceLocation{file_, loc.line, loc.column}));
  }

  SymbolTable & symbols_;
  ast::Nodes & nodes_;
  std::string file_;
  Names predicates_;
  Names functions_;
  std::set<std::string> nullary_;
  std::map<std::string, size_t> arity_;
  std::set<std::string> objects_;
  std::set<std::string> warned_;
  std::vector<Diagnostic> diags_;
};

// A function declared again with the same arity: one declaration, `either` of both types
void merge_functions(ast::Domain & d, std::vector<Diagnostic> & diags)
{
  std::map<Symbol, size_t> first;
  std::vector<ast::FunctionDecl> kept;
  for (auto & f : d.functions) {
    auto [it, fresh] = first.emplace(f.name, kept.size());
    if (fresh) {
      kept.push_back(std::move(f));
      continue;
    }
    ast::FunctionDecl & k = kept[it->second];
    if (k.params.size() != f.params.size()) {
      kept.push_back(std::move(f));  // the analysis reports it
      continue;
    }
    for (size_t i = 0; i < f.params.size(); ++i) {
      auto & types = k.params[i].types;
      if (types.empty() || f.params[i].types.empty()) {
        types.clear();  // untyped: any object
        continue;
      }
      for (Symbol t : f.params[i].types) {
        if (std::find(types.begin(), types.end(), t) == types.end()) {
          types.push_back(t);
        }
      }
    }
    diags.push_back(
      Diagnostic::warning(
        "dialect.function_redeclared", "function '" + std::string(d.symbols.name(f.name)) +
        "' declared again: its parameters take the types of both declarations",
        SourceLocation{d.file, f.loc.line, f.loc.column}));
  }
  d.functions = std::move(kept);
}

bool reads_t(const ast::Nodes & nodes, ast::ExprId id)
{
  const auto & e = nodes.exprs[id];
  if (e.kind == ast::ExprKind::ContinuousT) {
    return true;
  }
  return std::any_of(e.children.begin(), e.children.end(), [&](ast::ExprId c) {
             return reads_t(nodes, c);
         });
}

// Functions only continuous effects change and nothing assigns: SMTPlan starts them at 0
std::set<std::string> continuous_only(const ast::Domain & d)
{
  std::set<std::string> changed, assigned;
  for (uint32_t i = 0; i < d.nodes.effects.size(); ++i) {
    const auto & e = d.nodes.effects[ast::EffectId(i)];
    if (e.kind != ast::EffectKind::Assign) {
      continue;
    }
    std::string f(d.symbols.name(d.nodes.exprs[e.target].name));
    if (reads_t(d.nodes, e.value)) {
      changed.insert(f);
    } else {
      assigned.insert(f);
    }
  }
  std::set<std::string> out;
  for (const auto & f : changed) {
    if (!assigned.count(f)) {
      out.insert(f);
    }
  }
  return out;
}

// Whether an object of `types` may stand for a parameter of `wanted` (the domain's hierarchy)
bool fits(
  const ast::Domain & d, const std::vector<std::string> & types,
  const std::vector<std::string> & wanted)
{
  if (wanted.empty() || std::find(wanted.begin(), wanted.end(), "object") != wanted.end()) {
    return true;
  }
  std::vector<std::string> todo = types.empty() ? std::vector<std::string>{"object"} : types;
  std::set<std::string> seen;
  while (!todo.empty()) {
    std::string t = todo.back();
    todo.pop_back();
    if (!seen.insert(t).second) {
      continue;
    }
    if (std::find(wanted.begin(), wanted.end(), t) != wanted.end()) {
      return true;
    }
    for (const auto & decl : d.types) {
      if (d.symbols.name(decl.name) == t) {
        for (Symbol parent : decl.types) {
          todo.emplace_back(d.symbols.name(parent));
        }
      }
    }
  }
  return false;
}

// SMTPlan's benchmarks leave such functions without a value: 0, with a warning
void zero_continuous_only(
  ast::Problem & p, const ast::Domain & d, std::vector<Diagnostic> & diags)
{
  auto functions = continuous_only(d);
  if (functions.empty()) {
    return;
  }
  // Objects and constants, with their types
  std::vector<std::pair<std::string, std::vector<std::string>>> objects;
  auto add = [&](const ast::TypedName & o, const SymbolTable & symbols) {
      std::vector<std::string> types;
      for (Symbol t : o.types) {
        types.emplace_back(symbols.name(t));
      }
      objects.emplace_back(std::string(symbols.name(o.name)), std::move(types));
    };
  for (const auto & c : d.constants) {
    add(c, d.symbols);
  }
  for (const auto & o : p.objects) {
    add(o, p.symbols);
  }
  std::set<std::string> given;
  for (const auto & item : p.init) {
    if (item.kind == ast::InitItem::Kind::Fluent && !item.time) {
      std::string key(p.symbols.name(item.name));
      for (auto a : item.args) {
        key += " " + std::string(p.symbols.name(p.nodes.terms[a].name));
      }
      given.insert(key);
    }
  }
  for (const auto & f : d.functions) {
    std::string name(d.symbols.name(f.name));
    if (!functions.count(name)) {
      continue;
    }
    // Every argument tuple of the right types, up to a bound
    std::vector<std::vector<size_t>> choices;
    for (const auto & param : f.params) {
      std::vector<std::string> wanted;
      for (Symbol t : param.types) {
        wanted.emplace_back(d.symbols.name(t));
      }
      std::vector<size_t> ok;
      for (size_t i = 0; i < objects.size(); ++i) {
        if (fits(d, objects[i].second, wanted)) {
          ok.push_back(i);
        }
      }
      choices.push_back(std::move(ok));
    }
    std::vector<size_t> at(choices.size(), 0);
    bool warned = false;
    for (size_t count = 0; count < 100000; ++count) {
      if (std::any_of(choices.begin(), choices.end(), [](const auto & c) {return c.empty();})) {
        break;
      }
      std::string key = name;
      for (size_t k = 0; k < at.size(); ++k) {
        key += " " + objects[choices[k][at[k]]].first;
      }
      if (!given.count(key)) {
        ast::InitItem item;
        item.kind = ast::InitItem::Kind::Fluent;
        item.name = p.symbols.intern(name);
        for (size_t k = 0; k < at.size(); ++k) {
          ast::Term term;
          term.kind = ast::TermKind::Name;
          term.name = p.symbols.intern(objects[choices[k][at[k]]].first);
          item.args.push_back(p.nodes.terms.push_back(std::move(term)));
        }
        ast::Expr zero;
        zero.kind = ast::ExprKind::Number;
        zero.number = 0.0;
        item.value = p.nodes.exprs.push_back(std::move(zero));
        item.loc = p.loc;
        p.init.push_back(std::move(item));
        if (!warned) {
          diags.push_back(
            Diagnostic::warning(
              "dialect.uninitialized", "(" + key + ") has no value, and only continuous "
              "effects change '" + name + "': it starts at 0 (as SMTPlan)",
              SourceLocation{p.file, p.loc.line, p.loc.column}));
          warned = true;
        }
      }
      // Next tuple
      size_t k = 0;
      while (k < at.size() && ++at[k] == choices[k].size()) {
        at[k++] = 0;
      }
      if (k == at.size()) {
        break;
      }
    }
  }
}

}  // namespace

// Names the domain uses as objects but does not declare (ENHSP's chasing cars name the
// problem's cars): constants, of the type of the first parameter they fill
void declare_used_objects(ast::Domain & d, std::vector<Diagnostic> & diags)
{
  std::set<std::string> known;
  for (const auto & c : d.constants) {
    known.insert(std::string(d.symbols.name(c.name)));
  }
  std::map<std::string, const std::vector<ast::TypedName> *> params;
  for (const auto & p : d.predicates) {
    params[std::string(d.symbols.name(p.name))] = &p.params;
  }
  for (const auto & f : d.functions) {
    params[std::string(d.symbols.name(f.name))] = &f.params;
  }
  std::map<std::string, std::pair<std::vector<Symbol>, ast::Loc>> found;
  auto args = [&](Symbol head, const std::vector<ast::TermId> & list) {
      auto ps = params.find(std::string(d.symbols.name(head)));
      for (size_t i = 0; i < list.size(); ++i) {
        const auto & t = d.nodes.terms[list[i]];
        std::string n(d.symbols.name(t.name));
        if (t.kind != ast::TermKind::Name || known.count(n) || n == "undefined") {
          continue;
        }
        std::vector<Symbol> type;
        if (ps != params.end() && i < ps->second->size()) {
          type = (*ps->second)[i].types;
        }
        auto [it, fresh] = found.emplace(n, std::make_pair(type, t.loc));
        if (!fresh && it->second.first.empty()) {
          it->second.first = type;
        }
      }
    };
  for (uint32_t i = 0; i < d.nodes.formulas.size(); ++i) {
    const auto & f = d.nodes.formulas[ast::FormulaId(i)];
    if (f.kind == ast::FormulaKind::Atom) {
      args(f.name, f.args);
    }
  }
  for (uint32_t i = 0; i < d.nodes.exprs.size(); ++i) {
    const auto & e = d.nodes.exprs[ast::ExprId(i)];
    if (e.kind == ast::ExprKind::Function) {
      args(e.name, e.args);
    }
  }
  for (uint32_t i = 0; i < d.nodes.effects.size(); ++i) {
    const auto & e = d.nodes.effects[ast::EffectId(i)];
    if (e.kind == ast::EffectKind::Add || e.kind == ast::EffectKind::Delete) {
      args(e.name, e.args);
    }
  }
  for (auto & [n, info] : found) {
    ast::TypedName c;
    c.name = d.symbols.intern(n);
    c.types = info.first;
    c.loc = info.second;
    d.constants.push_back(std::move(c));
    diags.push_back(
      Diagnostic::warning(
        "dialect.problem_object", "'" + n + "' is not a constant of the domain: taken as one "
        "(the problem must declare it)", SourceLocation{d.file, info.second.line,
          info.second.column}));
  }
}

// A predicate only ever used as a function (ENHSP's chasing cars declare (input_splits) among
// the predicates): a function
void predicates_used_as_functions(ast::Domain & d, std::vector<Diagnostic> & diags)
{
  std::set<Symbol> as_function, as_atom, functions;
  for (const auto & f : d.functions) {
    functions.insert(f.name);
  }
  for (uint32_t i = 0; i < d.nodes.exprs.size(); ++i) {
    const auto & e = d.nodes.exprs[ast::ExprId(i)];
    if (e.kind == ast::ExprKind::Function) {
      as_function.insert(e.name);
    }
  }
  for (uint32_t i = 0; i < d.nodes.formulas.size(); ++i) {
    const auto & f = d.nodes.formulas[ast::FormulaId(i)];
    if (f.kind == ast::FormulaKind::Atom) {
      as_atom.insert(f.name);
    }
  }
  for (uint32_t i = 0; i < d.nodes.effects.size(); ++i) {
    const auto & e = d.nodes.effects[ast::EffectId(i)];
    if (e.kind == ast::EffectKind::Add || e.kind == ast::EffectKind::Delete) {
      as_atom.insert(e.name);
    }
  }
  std::erase_if(d.predicates, [&](const ast::PredicateDecl & p) {
      if (!as_function.count(p.name) || as_atom.count(p.name) || functions.count(p.name)) {
        return false;
      }
      ast::FunctionDecl f;
      f.name = p.name;
      f.params = p.params;
      f.loc = p.loc;
      d.functions.push_back(std::move(f));
      diags.push_back(
        Diagnostic::warning(
          "dialect.predicate_as_function", "'" + std::string(d.symbols.name(p.name)) +
          "' is declared as a predicate but used as a function: a function",
          SourceLocation{d.file, p.loc.line, p.loc.column}));
      return true;
    });
}

std::vector<Diagnostic> repair_domain(ast::Domain & domain)
{
  std::vector<Diagnostic> diags;
  predicates_used_as_functions(domain, diags);
  merge_functions(domain, diags);
  declare_used_objects(domain, diags);
  Repair r(domain, domain.symbols, domain.nodes, domain.file);
  auto more = r.run(nullptr);
  diags.insert(diags.end(), more.begin(), more.end());
  return diags;
}

std::vector<Diagnostic> repair_problem(ast::Problem & problem, const ast::Domain & domain)
{
  Repair r(domain, problem.symbols, problem.nodes, problem.file);
  for (const auto & o : problem.objects) {
    r.object(problem.symbols.name(o.name));
  }
  auto diags = r.run(&problem.init);
  zero_continuous_only(problem, domain, diags);
  return diags;
}

}  // namespace kairos::pddl
