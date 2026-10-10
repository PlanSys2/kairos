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

#include "normalize/normalizer.hpp"

#include <algorithm>
#include <functional>
#include <iterator>
#include <map>
#include <set>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "pddl/printer.hpp"

namespace kairos::lifted
{

bool Condition::operator==(const Condition & other) const
{
  return literals == other.literals && comparisons == other.comparisons &&
         foralls == other.foralls;
}

PredicateId Task::add_predicate(
  const std::string & name, bool is_derived, std::vector<TypeSpec> params)
{
  predicate_names.push_back(name);
  derived.push_back(is_derived);
  predicate_params.push_back(std::move(params));
  return PredicateId(static_cast<uint32_t>(predicate_names.size() - 1));
}

namespace
{

using ast::EffectId;
using ast::ExprId;
using ast::FormulaId;
using ast::TermId;

enum class TimeTag : uint8_t {None, AtStart, OverAll, AtEnd};

// A formula in negation normal form, with resolved symbols
struct NF
{
  enum class Kind : uint8_t {True, False, And, Or, Lit, Cmp, Forall, Exists};

  Kind kind = Kind::True;
  TimeTag time = TimeTag::None;
  std::vector<NF> children;
  Literal lit;
  Comparison cmp;
  std::vector<uint32_t> vars;  // Forall, Exists
};

struct Item
{
  enum class Kind : uint8_t {Lit, Cmp, Forall};

  Kind kind = Kind::Lit;
  TimeTag time = TimeTag::None;
  Literal lit;
  Comparison cmp;
  Quantified forall;
};

// One disjunct of a DNF: a conjunction of items plus the existential variables it uses
struct Clause
{
  std::vector<Item> items;
  std::vector<uint32_t> exist;
};

using Clauses = std::vector<Clause>;

// Where expressions come from: domain or problem file
struct Source
{
  const ast::Nodes & nodes;
  const SymbolTable & symbols;
};

// --- Variable traversal and remapping --------------------------------------------------------

void term_vars(const Term & t, std::set<uint32_t> & out)
{
  if (t.kind == Term::Kind::Variable) {
    out.insert(t.index);
  }
  for (const auto & a : t.args) {
    term_vars(a, out);
  }
}

void expr_vars(const Expr & e, std::set<uint32_t> & out)
{
  for (const auto & a : e.args) {
    term_vars(a, out);
  }
  for (const auto & c : e.children) {
    expr_vars(c, out);
  }
}

void cmp_vars(const Comparison & c, std::set<uint32_t> & out)
{
  if (c.objects) {
    term_vars(c.lhs_term, out);
    term_vars(c.rhs_term, out);
  } else {
    expr_vars(c.lhs, out);
    expr_vars(c.rhs, out);
  }
}

// Free variables of a formula (used but not bound inside it)
void free_vars(const NF & n, std::set<uint32_t> & out)
{
  switch (n.kind) {
    case NF::Kind::Lit:
      for (const auto & a : n.lit.atom.args) {
        term_vars(a, out);
      }
      return;
    case NF::Kind::Cmp:
      cmp_vars(n.cmp, out);
      return;
    case NF::Kind::Forall:
    case NF::Kind::Exists: {
        std::set<uint32_t> inner;
        free_vars(n.children.at(0), inner);
        for (uint32_t v : n.vars) {
          inner.erase(v);
        }
        out.insert(inner.begin(), inner.end());
        return;
      }
    default:
      for (const auto & c : n.children) {
        free_vars(c, out);
      }
  }
}

using VarMap = std::unordered_map<uint32_t, uint32_t>;

void remap(Term & t, const VarMap & m)
{
  if (t.kind == Term::Kind::Variable) {
    t.index = m.at(t.index);
  }
  for (auto & a : t.args) {
    remap(a, m);
  }
}

void remap(Expr & e, const VarMap & m)
{
  for (auto & a : e.args) {
    remap(a, m);
  }
  for (auto & c : e.children) {
    remap(c, m);
  }
}

void remap(Comparison & c, const VarMap & m)
{
  // Only one pair is in use: terms for object comparisons, expressions otherwise
  if (c.objects) {
    remap(c.lhs_term, m);
    remap(c.rhs_term, m);
  } else {
    remap(c.lhs, m);
    remap(c.rhs, m);
  }
}

void remap(Condition & c, const VarMap & m);

void remap(Quantified & q, const VarMap & m)
{
  for (auto & v : q.variables) {
    v = m.at(v);
  }
  remap(q.body, m);
}

void remap(Condition & c, const VarMap & m)
{
  for (auto & l : c.literals) {
    for (auto & a : l.atom.args) {
      remap(a, m);
    }
  }
  for (auto & cmp : c.comparisons) {
    remap(cmp, m);
  }
  for (auto & q : c.foralls) {
    remap(q, m);
  }
}

void remap(Effect & e, const VarMap & m)
{
  for (auto & v : e.variables) {
    v = m.at(v);
  }
  remap(e.condition, m);
  remap(e.start_condition, m);
  for (auto & a : e.atom.args) {
    remap(a, m);
  }
  for (auto & a : e.args) {
    remap(a, m);
  }
  remap(e.value, m);
  if (e.object) {
    remap(*e.object, m);
  }
}

// Remaps a formula into a new variable table, adding the variables it meets
void remap_into(
  NF & n, VarMap & m, const std::vector<Variable> & from, std::vector<Variable> & to)
{
  auto map_var = [&](uint32_t v) {
      auto it = m.find(v);
      if (it != m.end()) {
        return it->second;
      }
      uint32_t nv = static_cast<uint32_t>(to.size());
      to.push_back(from.at(v));
      m.emplace(v, nv);
      return nv;
    };
  std::function<void(Term &)> term = [&](Term & t) {
      if (t.kind == Term::Kind::Variable) {
        t.index = map_var(t.index);
      }
      for (auto & a : t.args) {
        term(a);
      }
    };
  std::function<void(Expr &)> expr = [&](Expr & e) {
      for (auto & a : e.args) {
        term(a);
      }
      for (auto & c : e.children) {
        expr(c);
      }
    };
  n.time = TimeTag::None;
  for (auto & v : n.vars) {
    v = map_var(v);
  }
  if (n.kind == NF::Kind::Lit) {
    for (auto & a : n.lit.atom.args) {
      term(a);
    }
  }
  if (n.kind == NF::Kind::Cmp) {
    if (n.cmp.objects) {
      term(n.cmp.lhs_term);
      term(n.cmp.rhs_term);
    } else {
      expr(n.cmp.lhs);
      expr(n.cmp.rhs);
    }
  }
  for (auto & c : n.children) {
    remap_into(c, m, from, to);
  }
}

CompareOp negate(CompareOp op)
{
  switch (op) {
    case CompareOp::Eq: return CompareOp::Ne;
    case CompareOp::Ne: return CompareOp::Eq;
    case CompareOp::Lt: return CompareOp::Ge;
    case CompareOp::Le: return CompareOp::Gt;
    case CompareOp::Gt: return CompareOp::Le;
    case CompareOp::Ge: return CompareOp::Lt;
  }
  return op;
}


// Negation of a formula already in negation normal form
NF negated(const NF & n)
{
  NF out = n;
  switch (n.kind) {
    case NF::Kind::True: out.kind = NF::Kind::False; break;
    case NF::Kind::False: out.kind = NF::Kind::True; break;
    case NF::Kind::And: out.kind = NF::Kind::Or; break;
    case NF::Kind::Or: out.kind = NF::Kind::And; break;
    case NF::Kind::Forall: out.kind = NF::Kind::Exists; break;
    case NF::Kind::Exists: out.kind = NF::Kind::Forall; break;
    case NF::Kind::Lit: out.lit.positive = !n.lit.positive; break;
    case NF::Kind::Cmp: out.cmp.op = negate(n.cmp.op); break;
  }
  for (auto & c : out.children) {
    c = negated(c);
  }
  return out;
}

// A conjunction of literals, comparisons and foralls: one clause without existentials
bool conjunctive(const NF & n)
{
  switch (n.kind) {
    case NF::Kind::True: case NF::Kind::Lit: case NF::Kind::Cmp: case NF::Kind::Forall:
      return true;
    case NF::Kind::And:
      return std::all_of(n.children.begin(), n.children.end(), conjunctive);
    default:
      return false;
  }
}

Condition to_condition(const std::vector<Item> & items)
{
  Condition c;
  for (const auto & it : items) {
    switch (it.kind) {
      case Item::Kind::Lit: c.literals.push_back(it.lit); break;
      case Item::Kind::Cmp: c.comparisons.push_back(it.cmp); break;
      case Item::Kind::Forall: c.foralls.push_back(it.forall); break;
    }
  }
  return c;
}

void append(Condition & to, const Condition & from)
{
  to.literals.insert(to.literals.end(), from.literals.begin(), from.literals.end());
  to.comparisons.insert(to.comparisons.end(), from.comparisons.begin(), from.comparisons.end());
  to.foralls.insert(to.foralls.end(), from.foralls.begin(), from.foralls.end());
}

// --- Normalizer ------------------------------------------------------------------------------

class Normalizer
{
public:
  Normalizer(
    const ast::Domain & d, const sem::DomainModel & dm, const ast::Problem & p,
    const sem::ProblemModel & pm, const NormalizeOptions & options, Task & task)
  : d_(d), dm_(dm), p_(p), pm_(pm), options_(options), task_(task) {}

  void run();

private:
  using Scope = std::vector<std::pair<std::string, uint32_t>>;

  std::string str(const Source & s, Symbol sym) const {return std::string(s.symbols.name(sym));}

  TypeSpec resolve(const Source & s, const std::vector<Symbol> & names) const
  {
    TypeSpec spec;
    for (Symbol n : names) {
      if (auto t = dm_.types.find(s.symbols.name(n))) {
        spec.push_back(*t);
      }
    }
    if (spec.empty()) {
      spec.push_back(dm_.types.object());
    }
    return spec;
  }

  // Adds typed variables to the owner's table and the scope
  std::vector<uint32_t> declare(
    const Source & s, const std::vector<ast::TypedName> & params, std::vector<Variable> & owner,
    Scope & scope)
  {
    std::vector<uint32_t> out;
    for (const auto & p : params) {
      uint32_t v = static_cast<uint32_t>(owner.size());
      owner.push_back(Variable{str(s, p.name), resolve(s, p.types)});
      scope.emplace_back(str(s, p.name), v);
      out.push_back(v);
    }
    return out;
  }

  Term term(const Source & s, TermId id, const Scope & scope) const
  {
    const ast::Term & t = s.nodes.terms[id];
    std::string n = str(s, t.name);
    switch (t.kind) {
      case ast::TermKind::Variable:
        for (auto it = scope.rbegin(); it != scope.rend(); ++it) {
          if (it->first == n) {
            return Term::variable(it->second);
          }
        }
        return Term::variable(0);  // unreachable after semantic analysis
      case ast::TermKind::Name:
        return Term::object(*pm_.objects.find(n));
      case ast::TermKind::Function: {
          Term f{Term::Kind::Function, dm_.functions.find(n)->value(), {}};
          for (TermId a : t.args) {
            f.args.push_back(term(s, a, scope));
          }
          return f;
        }
    }
    return Term{};
  }

  bool is_object_expr(const Source & s, ExprId id) const
  {
    const ast::Expr & e = s.nodes.exprs[id];
    if (e.kind == ast::ExprKind::Term) {
      return true;
    }
    if (e.kind == ast::ExprKind::Function) {
      auto f = dm_.functions.find(s.symbols.name(e.name));
      return f && !dm_.functions.items[*f].numeric;
    }
    return false;
  }

  Term object_term(const Source & s, ExprId id, const Scope & scope) const
  {
    const ast::Expr & e = s.nodes.exprs[id];
    if (e.kind == ast::ExprKind::Term) {
      return term(s, e.term, scope);
    }
    Term f{Term::Kind::Function, dm_.functions.find(s.symbols.name(e.name))->value(), {}};
    for (TermId a : e.args) {
      f.args.push_back(term(s, a, scope));
    }
    return f;
  }

  Expr expr(const Source & s, ExprId id, const Scope & scope) const
  {
    const ast::Expr & e = s.nodes.exprs[id];
    Expr out;
    for (ExprId c : e.children) {
      out.children.push_back(expr(s, c, scope));
    }
    switch (e.kind) {
      case ast::ExprKind::Number:
        out.op = ExprOp::Constant;
        out.value = e.number;
        break;
      case ast::ExprKind::Term:
        out.op = ExprOp::Constant;  // objects are not numbers; rejected earlier
        break;
      case ast::ExprKind::Function:
        out.op = ExprOp::Fluent;
        out.function = *dm_.functions.find(s.symbols.name(e.name));
        for (TermId a : e.args) {
          out.args.push_back(term(s, a, scope));
        }
        break;
      case ast::ExprKind::Add: out.op = ExprOp::Add; break;
      case ast::ExprKind::Sub: out.op = ExprOp::Sub; break;
      case ast::ExprKind::Mul: out.op = ExprOp::Mul; break;
      case ast::ExprKind::Div: out.op = ExprOp::Div; break;
      case ast::ExprKind::Neg: out.op = ExprOp::Neg; break;
      case ast::ExprKind::Pow: out.op = ExprOp::Pow; break;
      case ast::ExprKind::Math:
        out.op = ExprOp::Math;
        out.name = str(s, e.name);
        break;
      case ast::ExprKind::Duration: out.op = ExprOp::Duration; break;
      case ast::ExprKind::TotalTime: out.op = ExprOp::TotalTime; break;
      case ast::ExprKind::ContinuousT: out.op = ExprOp::ContinuousT; break;
      case ast::ExprKind::IsViolated:
        out.op = ExprOp::Violated;
        out.name = str(s, e.name);
        break;
    }
    return out;
  }

  // --- Formulas to NF ------------------------------------------------------------------------

  NF to_nf(
    const Source & s, FormulaId id, bool positive, TimeTag time, std::vector<Variable> & owner,
    Scope & scope)
  {
    const ast::Formula & f = s.nodes.formulas[id];
    NF n;
    n.time = time;
    auto both = [&](NF::Kind pos, NF::Kind neg) {
        n.kind = positive ? pos : neg;
        for (FormulaId c : f.children) {
          n.children.push_back(to_nf(s, c, positive, time, owner, scope));
        }
        return n;
      };
    switch (f.kind) {
      case ast::FormulaKind::And: return both(NF::Kind::And, NF::Kind::Or);
      case ast::FormulaKind::Or: return both(NF::Kind::Or, NF::Kind::And);
      case ast::FormulaKind::Not: return to_nf(s, f.children.at(0), !positive, time, owner, scope);
      case ast::FormulaKind::Imply:
        n.kind = positive ? NF::Kind::Or : NF::Kind::And;
        n.children.push_back(to_nf(s, f.children.at(0), !positive, time, owner, scope));
        n.children.push_back(to_nf(s, f.children.at(1), positive, time, owner, scope));
        return n;
      case ast::FormulaKind::Exists:
      case ast::FormulaKind::Forall: {
          bool exists = (f.kind == ast::FormulaKind::Exists) == positive;
          n.kind = exists ? NF::Kind::Exists : NF::Kind::Forall;
          size_t mark = scope.size();
          n.vars = declare(s, f.params, owner, scope);
          n.children.push_back(to_nf(s, f.children.at(0), positive, time, owner, scope));
          scope.resize(mark);
          return n;
        }
      case ast::FormulaKind::Atom: {
          n.kind = NF::Kind::Lit;
          n.lit.positive = positive;
          n.lit.atom.predicate = *dm_.predicates.find(s.symbols.name(f.name));
          for (TermId a : f.args) {
            n.lit.atom.args.push_back(term(s, a, scope));
          }
          return n;
        }
      case ast::FormulaKind::Compare: {
          n.kind = NF::Kind::Cmp;
          static const CompareOp ops[] = {CompareOp::Eq, CompareOp::Lt, CompareOp::Le,
            CompareOp::Gt, CompareOp::Ge};
          CompareOp op = ops[static_cast<int>(f.compare)];
          n.cmp.op = positive ? op : negate(op);
          if (is_object_expr(s, f.lhs) || is_object_expr(s, f.rhs)) {
            n.cmp.objects = true;
            n.cmp.lhs_term = object_term(s, f.lhs, scope);
            n.cmp.rhs_term = object_term(s, f.rhs, scope);
          } else {
            n.cmp.lhs = expr(s, f.lhs, scope);
            n.cmp.rhs = expr(s, f.rhs, scope);
          }
          return n;
        }
      case ast::FormulaKind::AtStart:
        return to_nf(s, f.children.at(0), positive, TimeTag::AtStart, owner, scope);
      case ast::FormulaKind::AtEnd:
        return to_nf(s, f.children.at(0), positive, TimeTag::AtEnd, owner, scope);
      case ast::FormulaKind::OverAll:
        return to_nf(s, f.children.at(0), positive, TimeTag::OverAll, owner, scope);
      case ast::FormulaKind::Preference:
        // Soft: does not constrain applicability (handled from phase F4)
        task_.has_soft_goals = true;
        n.kind = NF::Kind::True;
        return n;
      case ast::FormulaKind::Modal:
        task_.has_trajectory_constraints = true;
        n.kind = NF::Kind::True;
        return n;
    }
    return n;
  }

  // --- DNF -----------------------------------------------------------------------------------

  // Replaces `node` by an auxiliary derived predicate over its free variables
  Item make_aux(const NF & node, const std::vector<Variable> & owner)
  {
    std::set<uint32_t> free_set;
    free_vars(node, free_set);
    std::vector<uint32_t> free(free_set.begin(), free_set.end());

    std::vector<Variable> table;
    std::vector<TypeSpec> params;
    VarMap m;
    for (uint32_t v : free) {
      m.emplace(v, static_cast<uint32_t>(table.size()));
      table.push_back(owner.at(v));
      params.push_back(owner.at(v).types);
    }
    PredicateId pred = task_.add_predicate(
      "kairos-aux-" + std::to_string(aux_counter_++), true, std::move(params));
    NF body = node;
    remap_into(body, m, owner, table);
    // Top level: each disjunct is a rule, without limit (else an aux for a big "or" would be
    // that same "or" again, forever)
    bool saved = in_rule_;
    in_rule_ = true;
    Clauses clauses = dnf(body, table, true);
    in_rule_ = saved;
    for (const auto & cl : clauses) {
      task_.axioms.push_back(make_rule(pred, table, free.size(), cl));
    }
    Item it;
    it.kind = Item::Kind::Lit;
    it.time = node.time;
    it.lit.positive = true;
    it.lit.atom.predicate = pred;
    for (uint32_t v : free) {
      it.lit.atom.args.push_back(Term::variable(v));
    }
    return it;
  }

  // A rule with its variables ordered: head, existential, forall-bound
  static Axiom make_rule(
    PredicateId head, const std::vector<Variable> & table, size_t arity, const Clause & cl)
  {
    VarMap m = copy_order(arity, cl.exist, table.size());
    Axiom ax;
    ax.head = head;
    ax.variables = reorder(table, m);
    ax.head_arity = static_cast<uint32_t>(arity);
    ax.existential = distinct(cl.exist);
    ax.body = to_condition(cl.items);
    remap(ax.body, m);
    return ax;
  }

  Clauses aux_clauses(const NF & node, const std::vector<Variable> & owner)
  {
    return Clauses{Clause{{make_aux(node, owner)}, {}}};
  }

  // Conjoins b to a, moving from b when it is no longer needed
  static void conjoin(Clause & a, Clause & b, bool last)
  {
    if (last) {
      a.items.insert(
        a.items.end(), std::make_move_iterator(b.items.begin()),
        std::make_move_iterator(b.items.end()));
    } else {
      a.items.insert(a.items.end(), b.items.begin(), b.items.end());
    }
    a.exist.insert(a.exist.end(), b.exist.begin(), b.exist.end());
  }

  Clauses dnf(const NF & n, std::vector<Variable> & owner, bool top = false)
  {
    const size_t max = std::max<size_t>(1, options_.max_disjuncts);
    switch (n.kind) {
      case NF::Kind::True: return Clauses{Clause{}};
      case NF::Kind::False: return Clauses{};
      case NF::Kind::Lit: {
          Item it;
          it.kind = Item::Kind::Lit;
          it.time = n.time;
          it.lit = n.lit;
          return Clauses{Clause{{it}, {}}};
        }
      case NF::Kind::Cmp: {
          Item it;
          it.kind = Item::Kind::Cmp;
          it.time = n.time;
          it.cmp = n.cmp;
          return Clauses{Clause{{it}, {}}};
        }
      case NF::Kind::Exists: {
          if (!in_rule_ && n.vars.size() > options_.max_hidden_variables) {
            // Each witness would be a copy of the action: an auxiliary predicate instead
            return aux_clauses(n, owner);
          }
          Clauses cs = dnf(n.children.at(0), owner);
          for (auto & c : cs) {
            c.exist.insert(c.exist.end(), n.vars.begin(), n.vars.end());
          }
          return cs;
        }
      case NF::Kind::Forall: {
          if (!in_axiom_ && !conjunctive(n.children.at(0))) {
            // forall x phi = not exists x (not phi): the auxiliary predicate gets the
            // existential, whose body binds x with positive atoms (as FD's translator does).
            // Not inside axioms: it could close a cycle through negation.
            NF ex;
            ex.kind = NF::Kind::Exists;
            ex.time = n.time;
            ex.vars = n.vars;
            ex.children.push_back(negated(n.children.at(0)));
            Item it = make_aux(ex, owner);
            it.lit.positive = false;
            return Clauses{Clause{{it}, {}}};
          }
          Clauses body = dnf(n.children.at(0), owner);
          Item it;
          it.kind = Item::Kind::Forall;
          it.time = n.time;
          it.forall.variables = n.vars;
          if (body.size() == 1 && body[0].exist.empty()) {
            it.forall.body = to_condition(body[0].items);
          } else if (body.empty()) {
            // forall of false: false unless the domain is empty; keep it as an aux predicate
            it.forall.body = to_condition({make_aux(n.children.at(0), owner)});
          } else {
            it.forall.body = to_condition({make_aux(n.children.at(0), owner)});
          }
          return Clauses{Clause{{it}, {}}};
        }
      case NF::Kind::Or: {
          Clauses out;
          for (const auto & c : n.children) {
            Clauses cs = dnf(c, owner);
            out.insert(out.end(), std::make_move_iterator(cs.begin()),
                std::make_move_iterator(cs.end()));
          }
          if (out.size() > max && !top) {
            return aux_clauses(n, owner);
          }
          return out;
        }
      case NF::Kind::And: {
          std::vector<Clauses> parts;
          for (const auto & c : n.children) {
            parts.push_back(dnf(c, owner));
          }
          auto product = [&]() {
              size_t p = 1;
              for (const auto & cs : parts) {
                p *= std::max<size_t>(cs.size(), 1);
                if (p > max) {
                  return p;
                }
              }
              return p;
            };
          // Too many combinations: replace the widest parts by auxiliary predicates
          while (product() > max) {
            size_t widest = 0;
            for (size_t i = 1; i < parts.size(); ++i) {
              if (parts[i].size() > parts[widest].size()) {
                widest = i;
              }
            }
            if (parts[widest].size() <= 1) {
              break;
            }
            parts[widest] = aux_clauses(n.children[widest], owner);
          }
          Clauses out{Clause{}};
          for (auto & cs : parts) {
            if (cs.empty()) {
              return Clauses{};  // a false conjunct
            }
            if (cs.size() == 1) {
              // Common case: extend in place (copying clauses per conjunct is quadratic)
              for (size_t i = 0; i < out.size(); ++i) {
                conjoin(out[i], cs[0], i + 1 == out.size());
              }
              continue;
            }
            Clauses next;
            for (const auto & a : out) {
              for (const auto & b : cs) {
                Clause c = a;
                c.items.insert(c.items.end(), b.items.begin(), b.items.end());
                c.exist.insert(c.exist.end(), b.exist.begin(), b.exist.end());
                next.push_back(std::move(c));
              }
            }
            out = std::move(next);
          }
          return out;
        }
    }
    return Clauses{Clause{}};
  }

  // A conjunctive condition for effects and goals: one clause without existentials, or an
  // auxiliary predicate
  Condition single(const NF & n, std::vector<Variable> & owner)
  {
    Clauses cs = dnf(n, owner);
    if (cs.size() == 1 && cs[0].exist.empty()) {
      return to_condition(cs[0].items);
    }
    return to_condition({make_aux(n, owner)});
  }

  // --- Effects -------------------------------------------------------------------------------

  struct EffectContext
  {
    std::vector<uint32_t> vars;
    std::vector<NF> conditions;
    Timing timing = Timing::Instant;
  };

  bool has_t(const Source & s, ExprId id) const
  {
    const ast::Expr & e = s.nodes.exprs[id];
    if (e.kind == ast::ExprKind::ContinuousT) {
      return true;
    }
    return std::any_of(e.children.begin(), e.children.end(),
             [&](ExprId c) {return has_t(s, c);});
  }

  void flatten(
    const Source & s, EffectId id, EffectContext ctx, std::vector<Variable> & owner,
    Scope & scope, std::vector<Effect> & out)
  {
    const ast::Effect & e = s.nodes.effects[id];
    switch (e.kind) {
      case ast::EffectKind::And:
        for (EffectId c : e.children) {
          flatten(s, c, ctx, owner, scope, out);
        }
        return;
      case ast::EffectKind::Forall: {
          size_t mark = scope.size();
          auto vars = declare(s, e.params, owner, scope);
          ctx.vars.insert(ctx.vars.end(), vars.begin(), vars.end());
          flatten(s, e.children.at(0), ctx, owner, scope, out);
          scope.resize(mark);
          return;
        }
      case ast::EffectKind::When:
        ctx.conditions.push_back(to_nf(s, e.condition, true, TimeTag::None, owner, scope));
        flatten(s, e.children.at(0), ctx, owner, scope, out);
        return;
      case ast::EffectKind::AtStart:
        ctx.timing = Timing::Start;
        flatten(s, e.children.at(0), ctx, owner, scope, out);
        return;
      case ast::EffectKind::AtEnd:
        ctx.timing = Timing::End;
        flatten(s, e.children.at(0), ctx, owner, scope, out);
        return;
      default:
        break;
    }

    Effect eff;
    eff.timing = ctx.timing;
    eff.variables = ctx.vars;
    // Conditions: grouped by when they are read
    std::map<TimeTag, NF> groups;
    for (const auto & c : ctx.conditions) {
      split_by_time(c, groups);
    }
    for (auto & [time, group] : groups) {
      Condition cond = single(group, owner);
      bool at_start = time == TimeTag::AtStart && ctx.timing == Timing::End;
      append(at_start ? eff.start_condition : eff.condition, cond);
    }

    if (e.kind == ast::EffectKind::Add || e.kind == ast::EffectKind::Delete) {
      eff.kind = e.kind == ast::EffectKind::Add ? Effect::Kind::Add : Effect::Kind::Delete;
      eff.atom.predicate = *dm_.predicates.find(s.symbols.name(e.name));
      for (TermId a : e.args) {
        eff.atom.args.push_back(term(s, a, scope));
      }
      out.push_back(std::move(eff));
      return;
    }
    // Assign
    const ast::Expr & target = s.nodes.exprs[e.target];
    auto fn = *dm_.functions.find(s.symbols.name(target.name));
    eff.function = fn;
    for (TermId a : target.args) {
      eff.args.push_back(term(s, a, scope));
    }
    eff.op = e.op;
    if (!dm_.functions.items[fn].numeric) {
      eff.kind = Effect::Kind::ObjectAssign;
      const ast::Expr & v = s.nodes.exprs[e.value];
      bool undefined = v.kind == ast::ExprKind::Term &&
        s.nodes.terms[v.term].kind == ast::TermKind::Name &&
        str(s, s.nodes.terms[v.term].name) == "undefined";
      if (!undefined) {
        eff.object = object_term(s, e.value, scope);
      }
    } else {
      eff.kind = Effect::Kind::Numeric;
      eff.value = expr(s, e.value, scope);
      eff.continuous = has_t(s, e.value);
    }
    out.push_back(std::move(eff));
  }

  // Splits a (conjunctive at the top) formula into groups by time tag
  static void split_by_time(const NF & n, std::map<TimeTag, NF> & groups)
  {
    if (n.kind == NF::Kind::And) {
      for (const auto & c : n.children) {
        split_by_time(c, groups);
      }
      return;
    }
    TimeTag t = n.time == TimeTag::OverAll ? TimeTag::AtEnd : n.time;
    auto & g = groups[t];
    g.kind = NF::Kind::And;
    g.time = t;
    g.children.push_back(n);
  }

  // --- Actions -------------------------------------------------------------------------------

  // Variable order for a copy: parameters, its existential variables, the rest
  static VarMap copy_order(size_t params, const std::vector<uint32_t> & exist, size_t total)
  {
    VarMap m;
    uint32_t next = 0;
    for (uint32_t v = 0; v < params; ++v) {
      m.emplace(v, next++);
    }
    for (uint32_t v : exist) {
      if (!m.count(v)) {
        m.emplace(v, next++);
      }
    }
    for (uint32_t v = 0; v < total; ++v) {
      if (!m.count(v)) {
        m.emplace(v, next++);
      }
    }
    return m;
  }

  static std::vector<Variable> reorder(const std::vector<Variable> & vars, const VarMap & m)
  {
    std::vector<Variable> out(vars.size());
    for (const auto & [from, to] : m) {
      out.at(to) = vars.at(from);
    }
    return out;
  }

  static uint32_t distinct(const std::vector<uint32_t> & v)
  {
    return static_cast<uint32_t>(std::set<uint32_t>(v.begin(), v.end()).size());
  }

  void instant_action(const ast::Action & a, std::vector<Action> & out)
  {
    Source s{d_.nodes, d_.symbols};
    std::vector<Variable> owner;
    Scope scope;
    declare(s, a.params, owner, scope);
    size_t params = owner.size();
    NF pre;
    if (a.precondition.valid()) {
      pre = to_nf(s, a.precondition, true, TimeTag::None, owner, scope);
    }
    std::vector<Effect> effects;
    if (a.effect.valid()) {
      flatten(s, a.effect, EffectContext{}, owner, scope, effects);
    }
    Clauses clauses = dnf(pre, owner);
    for (const auto & cl : clauses) {
      VarMap m = copy_order(params, cl.exist, owner.size());
      Action act;
      act.name = str(s, a.name);
      act.variables = reorder(owner, m);
      act.parameters = static_cast<uint32_t>(params);
      act.hidden = distinct(cl.exist);
      act.precondition = to_condition(cl.items);
      remap(act.precondition, m);
      for (auto e : effects) {
        remap(e, m);
        act.effects.push_back(std::move(e));
      }
      out.push_back(std::move(act));
    }
  }

  void durative_action(const ast::DurativeAction & a)
  {
    Source s{d_.nodes, d_.symbols};
    std::vector<Variable> owner;
    Scope scope;
    declare(s, a.params, owner, scope);
    size_t params = owner.size();

    // Duration constraints: ?duration op value
    std::vector<DurationConstraint> duration;
    std::function<void(FormulaId, Timing)> dur = [&](FormulaId id, Timing timing) {
        const ast::Formula & f = d_.nodes.formulas[id];
        if (f.kind == ast::FormulaKind::And) {
          for (FormulaId c : f.children) {
            dur(c, timing);
          }
          return;
        }
        if (f.kind == ast::FormulaKind::AtStart || f.kind == ast::FormulaKind::AtEnd) {
          dur(f.children.at(0),
            f.kind == ast::FormulaKind::AtStart ? Timing::Start : Timing::End);
          return;
        }
        if (f.kind != ast::FormulaKind::Compare) {
          return;
        }
        bool lhs_is_duration = d_.nodes.exprs[f.lhs].kind == ast::ExprKind::Duration;
        DurationConstraint c;
        c.timing = timing;
        c.value = expr(s, lhs_is_duration ? f.rhs : f.lhs, scope);
        switch (f.compare) {
          case ast::CompareOp::Le: c.op = lhs_is_duration ? CompareOp::Le : CompareOp::Ge; break;
          case ast::CompareOp::Ge: c.op = lhs_is_duration ? CompareOp::Ge : CompareOp::Le; break;
          default: c.op = CompareOp::Eq; break;
        }
        duration.push_back(std::move(c));
      };
    dur(a.duration, Timing::Start);

    NF cond;
    if (a.condition.valid()) {
      cond = to_nf(s, a.condition, true, TimeTag::None, owner, scope);
    }
    std::vector<Effect> effects;
    if (a.effect.valid()) {
      flatten(s, a.effect, EffectContext{}, owner, scope, effects);
    }
    Clauses clauses = dnf(cond, owner);
    for (const auto & cl : clauses) {
      VarMap m = copy_order(params, cl.exist, owner.size());
      DurativeAction act;
      act.name = str(s, a.name);
      act.variables = reorder(owner, m);
      act.parameters = static_cast<uint32_t>(params);
      act.hidden = distinct(cl.exist);
      act.duration = duration;
      for (auto & c : act.duration) {
        remap(c.value, m);
      }
      for (const auto & it : cl.items) {
        Condition * target = &act.at_start;
        if (it.time == TimeTag::OverAll) {
          target = &act.over_all;
        } else if (it.time == TimeTag::AtEnd) {
          target = &act.at_end;
        }
        append(*target, to_condition({it}));
      }
      remap(act.at_start, m);
      remap(act.over_all, m);
      remap(act.at_end, m);
      for (auto e : effects) {
        remap(e, m);
        act.effects.push_back(std::move(e));
      }
      task_.durative_actions.push_back(std::move(act));
    }
  }

  void derived()
  {
    Source s{d_.nodes, d_.symbols};
    for (const auto & dv : d_.derived) {
      auto pred = *dm_.predicates.find(d_.symbols.name(dv.head.name));
      const auto & declared = dm_.predicates.items[pred].params;
      std::vector<Variable> owner;
      Scope scope;
      for (size_t i = 0; i < dv.head.params.size(); ++i) {
        const auto & p = dv.head.params[i];
        TypeSpec types = p.types.empty() && i < declared.size() ? declared[i] :
          resolve(s, p.types);
        owner.push_back(Variable{str(s, p.name), types});
        scope.emplace_back(str(s, p.name), static_cast<uint32_t>(i));
      }
      uint32_t arity = static_cast<uint32_t>(owner.size());
      NF body = to_nf(s, dv.body, true, TimeTag::None, owner, scope);
      in_axiom_ = in_rule_ = true;
      for (const auto & cl : dnf(body, owner)) {
        task_.axioms.push_back(make_rule(pred, owner, arity, cl));
      }
      in_axiom_ = in_rule_ = false;
    }
  }

  void problem_part()
  {
    Source s{p_.nodes, p_.symbols};
    auto object = [&](TermId t) {
        return *pm_.objects.find(p_.symbols.name(p_.nodes.terms[t].name));
      };
    for (const auto & item : p_.init) {
      std::vector<ObjectId> args;
      for (TermId a : item.args) {
        args.push_back(object(a));
      }
      if (item.kind == ast::InitItem::Kind::Fact) {
        auto pred = dm_.predicates.find(p_.symbols.name(item.name));
        if (!pred) {
          continue;
        }
        if (item.time) {
          TimedEvent ev;
          ev.time = *item.time;
          ev.fact = std::make_pair(*pred, args);
          ev.negated = item.negated;
          task_.timed_events.push_back(std::move(ev));
        } else if (!item.negated) {
          task_.init_facts.emplace_back(*pred, std::move(args));
        }
        continue;
      }
      auto fn = dm_.functions.find(p_.symbols.name(item.name));
      if (!fn) {
        continue;  // value for an unknown function: ignored
      }
      InitFluent f;
      f.function = *fn;
      f.args = std::move(args);
      f.numeric = dm_.functions.items[*fn].numeric;
      const ast::Expr & v = p_.nodes.exprs[item.value];
      if (f.numeric) {
        f.number = v.number;
      } else if (std::string(p_.symbols.name(p_.nodes.terms[v.term].name)) != "undefined") {
        f.object = object(v.term);
      }
      if (item.time) {
        TimedEvent ev;
        ev.time = *item.time;
        ev.fluent = std::move(f);
        task_.timed_events.push_back(std::move(ev));
      } else {
        task_.init_fluents.push_back(std::move(f));
      }
    }
    std::stable_sort(task_.timed_events.begin(), task_.timed_events.end(),
      [](const TimedEvent & a, const TimedEvent & b) {return a.time < b.time;});
    // Action costs: (total-cost) starts at 0 when the problem does not say (as FD and the IPC do)
    if (auto tc = dm_.functions.find("total-cost");
      tc && dm_.functions.items[*tc].params.empty() && dm_.functions.items[*tc].numeric &&
      std::none_of(task_.init_fluents.begin(), task_.init_fluents.end(),
      [&](const InitFluent & f) {return f.function == *tc;}))
    {
      InitFluent f;
      f.function = *tc;
      task_.init_fluents.push_back(f);
    }

    if (p_.goal.valid()) {
      std::vector<Variable> owner;
      Scope scope;
      NF g = to_nf(s, p_.goal, true, TimeTag::None, owner, scope);
      Clauses cs = dnf(g, owner);
      if (cs.size() == 1 && cs[0].exist.empty() && owner.empty()) {
        task_.goal = to_condition(cs[0].items);
      } else {
        task_.goal = to_condition({make_aux(g, owner)});
      }
    }
    if (p_.constraints.valid() || d_.constraints.valid()) {
      task_.has_trajectory_constraints = true;
    }

    if (p_.metric) {
      task_.metric.minimize = p_.metric->minimize;
      task_.metric.expr = expr(s, p_.metric->expr, {});
    } else {
      task_.metric.is_default = true;
      task_.metric.expr.op = d_.durative_actions.empty() ? ExprOp::StepCount : ExprOp::TotalTime;
    }
  }

  const ast::Domain & d_;
  const sem::DomainModel & dm_;
  const ast::Problem & p_;
  const sem::ProblemModel & pm_;
  const NormalizeOptions & options_;
  Task & task_;
  size_t aux_counter_ = 0;
  bool in_axiom_ = false;  // normalizing the body of a derived predicate
  bool in_rule_ = false;   // the body of any axiom: existentials are rule variables

public:
  void run_all()
  {
    task_.domain_name = dm_.name;
    task_.problem_name = pm_.name;
    for (const auto & p : dm_.predicates.items) {
      task_.add_predicate(p.name, p.derived, p.params);
    }
    for (const auto & f : dm_.functions.items) {
      task_.function_names.push_back(f.name);
      task_.numeric.push_back(f.numeric);
      task_.function_params.push_back(f.params);
    }
    for (const auto & o : pm_.objects.items) {
      task_.object_names.push_back(o.name);
      task_.object_types.push_back(o.types);
    }
    task_.types = dm_.types;
    task_.features = pm_.features;

    for (const auto & a : d_.actions) {
      instant_action(a, task_.actions);
    }
    for (const auto & a : d_.durative_actions) {
      durative_action(a);
    }
    for (const auto & a : d_.processes) {
      instant_action(a, task_.processes);
    }
    for (const auto & a : d_.events) {
      instant_action(a, task_.events);
    }
    derived();
    problem_part();
  }
};

void Normalizer::run() {run_all();}

// --- Printing --------------------------------------------------------------------------------

class TaskPrinter
{
public:
  explicit TaskPrinter(const Task & t)
  : t_(t) {}

  std::string term(const Term & x, const std::vector<Variable> & vars) const
  {
    switch (x.kind) {
      case Term::Kind::Variable:
        return x.index < vars.size() ? vars[x.index].name : "?#" + std::to_string(x.index);
      case Term::Kind::Object: return t_.object_names.at(x.index);
      case Term::Kind::Function: {
          std::string out = "(" + t_.function_names.at(x.index);
          for (const auto & a : x.args) {
            out += " " + term(a, vars);
          }
          return out + ")";
        }
    }
    return "?";
  }

  std::string expr(const Expr & e, const std::vector<Variable> & vars) const
  {
    auto op = [&](const char * sym) {
        std::string out = std::string("(") + sym;
        for (const auto & c : e.children) {
          out += " " + expr(c, vars);
        }
        return out + ")";
      };
    switch (e.op) {
      case ExprOp::Constant: return pddl::format_number(e.value);
      case ExprOp::Fluent: {
          std::string out = "(" + t_.function_names.at(e.function.index());
          for (const auto & a : e.args) {
            out += " " + term(a, vars);
          }
          return out + ")";
        }
      case ExprOp::Add: return op("+");
      case ExprOp::Sub: return op("-");
      case ExprOp::Mul: return op("*");
      case ExprOp::Div: return op("/");
      case ExprOp::Neg: return op("-");
      case ExprOp::Pow: return op("^");
      case ExprOp::Math: return op(e.name.c_str());
      case ExprOp::Duration: return "?duration";
      case ExprOp::TotalTime: return "(total-time)";
      case ExprOp::ContinuousT: return "#t";
      case ExprOp::Violated: return "(is-violated " + e.name + ")";
      case ExprOp::StepCount: return "(plan-length)";
    }
    return "?";
  }

  static const char * cmp_text(CompareOp op)
  {
    static const char * names[] = {"=", "!=", "<", "<=", ">", ">="};
    return names[static_cast<int>(op)];
  }

  std::string atom(const Atom & a, const std::vector<Variable> & vars) const
  {
    std::string out = "(" + t_.predicate_names.at(a.predicate.index());
    for (const auto & x : a.args) {
      out += " " + term(x, vars);
    }
    return out + ")";
  }

  std::string condition(const Condition & c, const std::vector<Variable> & vars) const
  {
    std::vector<std::string> parts;
    for (const auto & l : c.literals) {
      parts.push_back(l.positive ? atom(l.atom, vars) : "(not " + atom(l.atom, vars) + ")");
    }
    for (const auto & cmp : c.comparisons) {
      parts.push_back(std::string("(") + cmp_text(cmp.op) + " " +
        (cmp.objects ? term(cmp.lhs_term, vars) : expr(cmp.lhs, vars)) + " " +
        (cmp.objects ? term(cmp.rhs_term, vars) : expr(cmp.rhs, vars)) + ")");
    }
    for (const auto & q : c.foralls) {
      std::string v;
      for (uint32_t i : q.variables) {
        v += (v.empty() ? "" : " ") + vars.at(i).name;
      }
      parts.push_back("(forall (" + v + ") " + condition(q.body, vars) + ")");
    }
    if (parts.size() == 1) {
      return parts[0];
    }
    std::string out = "(and";
    for (const auto & p : parts) {
      out += " " + p;
    }
    return out + ")";
  }

  std::string effect(const Effect & e, const std::vector<Variable> & vars) const
  {
    static const char * ops[] = {"assign", "scale-up", "scale-down", "increase", "decrease"};
    std::string what;
    switch (e.kind) {
      case Effect::Kind::Add: what = atom(e.atom, vars); break;
      case Effect::Kind::Delete: what = "(not " + atom(e.atom, vars) + ")"; break;
      case Effect::Kind::Numeric:
      case Effect::Kind::ObjectAssign: {
          std::string target = "(" + t_.function_names.at(e.function.index());
          for (const auto & a : e.args) {
            target += " " + term(a, vars);
          }
          target += ")";
          std::string value = e.kind == Effect::Kind::Numeric ? expr(e.value, vars) :
            (e.object ? term(*e.object, vars) : std::string("undefined"));
          what = std::string("(") + ops[static_cast<int>(e.op)] + " " + target + " " + value +
            ")";
          break;
        }
    }
    if (!e.start_condition.empty()) {
      what = "(when-start " + condition(e.start_condition, vars) + " " + what + ")";
    }
    if (!e.condition.empty()) {
      what = "(when " + condition(e.condition, vars) + " " + what + ")";
    }
    if (!e.variables.empty()) {
      std::string v;
      for (uint32_t i : e.variables) {
        v += (v.empty() ? "" : " ") + vars.at(i).name;
      }
      what = "(forall (" + v + ") " + what + ")";
    }
    if (e.continuous) {
      return "continuous " + what;
    }
    if (e.timing == Timing::Start) {
      return "at-start " + what;
    }
    if (e.timing == Timing::End) {
      return "at-end " + what;
    }
    return what;
  }

  std::string params(const std::vector<Variable> & vars, uint32_t from, uint32_t count) const
  {
    std::string out;
    for (uint32_t i = from; i < from + count && i < vars.size(); ++i) {
      out += (out.empty() ? "" : " ") + vars[i].name + " - " +
        t_.types.to_string(vars[i].types);
    }
    return out;
  }

  std::string action(const char * kind, const Action & a) const
  {
    std::string out = std::string("(") + kind + " " + a.name + " (" +
      params(a.variables, 0, a.parameters) + ")";
    if (a.hidden > 0) {
      out += " :hidden (" + params(a.variables, a.parameters, a.hidden) + ")";
    }
    out += "\n  :precondition " + (a.precondition.empty() ? std::string("()") :
      condition(a.precondition, a.variables));
    for (const auto & e : a.effects) {
      out += "\n  :effect " + effect(e, a.variables);
    }
    return out + ")\n";
  }

  std::string all() const
  {
    std::string out = ";; lifted task " + t_.domain_name + " / " + t_.problem_name + "\n";
    for (const auto & a : t_.actions) {
      out += action(":action", a);
    }
    for (const auto & a : t_.durative_actions) {
      out += "(:durative-action " + a.name + " (" + params(a.variables, 0, a.parameters) + ")";
      if (a.hidden > 0) {
        out += " :hidden (" + params(a.variables, a.parameters, a.hidden) + ")";
      }
      for (const auto & d : a.duration) {
        out += std::string("\n  :duration ") + (d.timing == Timing::End ? "at-end " : "") +
          "(" + cmp_text(d.op) + " ?duration " + expr(d.value, a.variables) + ")";
      }
      if (!a.at_start.empty()) {
        out += "\n  :at-start " + condition(a.at_start, a.variables);
      }
      if (!a.over_all.empty()) {
        out += "\n  :over-all " + condition(a.over_all, a.variables);
      }
      if (!a.at_end.empty()) {
        out += "\n  :at-end " + condition(a.at_end, a.variables);
      }
      for (const auto & e : a.effects) {
        out += "\n  :effect " + effect(e, a.variables);
      }
      out += ")\n";
    }
    for (const auto & a : t_.processes) {
      out += action(":process", a);
    }
    for (const auto & a : t_.events) {
      out += action(":event", a);
    }
    for (const auto & ax : t_.axioms) {
      Atom head{ax.head, {}};
      for (uint32_t i = 0; i < ax.head_arity; ++i) {
        head.args.push_back(Term::variable(i));
      }
      out += "(:axiom " + atom(head, ax.variables) + " " +
        (ax.body.empty() ? std::string("()") : condition(ax.body, ax.variables)) + ")\n";
    }
    out += "(:init";
    for (const auto & [p, args] : t_.init_facts) {
      out += " (" + t_.predicate_names.at(p.index());
      for (auto o : args) {
        out += " " + t_.object_names.at(o.index());
      }
      out += ")";
    }
    out += ")\n(:goal " + (t_.goal.empty() ? std::string("()") : condition(t_.goal, {})) + ")\n";
    out += std::string("(:metric ") + (t_.metric.minimize ? "minimize " : "maximize ") +
      expr(t_.metric.expr, {}) + (t_.metric.is_default ? " ; default" : "") + ")\n";
    return out;
  }

private:
  const Task & t_;
};

}  // namespace

Task normalize(
  const ast::Domain & domain, const sem::DomainModel & domain_model,
  const ast::Problem & problem, const sem::ProblemModel & problem_model,
  const NormalizeOptions & options)
{
  Task task;
  Normalizer n(domain, domain_model, problem, problem_model, options, task);
  n.run();
  return task;
}

std::string to_string(const Task & task)
{
  return TaskPrinter(task).all();
}

std::string to_string(const Task & task, const Condition & c, const std::vector<Variable> & vars)
{
  return TaskPrinter(task).condition(c, vars);
}

}  // namespace kairos::lifted
