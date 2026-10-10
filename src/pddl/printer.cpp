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

#include "pddl/printer.hpp"

#include <array>
#include <charconv>
#include <string>
#include <string_view>
#include <vector>

namespace kairos::pddl
{

namespace
{

using ast::EffectId;
using ast::ExprId;
using ast::FormulaId;
using ast::TermId;

class Printer
{
public:
  Printer(const ast::Nodes & nodes, const SymbolTable & symbols)
  : nodes_(nodes), symbols_(symbols) {}

  std::string out;

  void name(Symbol s) {out += symbols_.name(s);}

  void types(const std::vector<Symbol> & t)
  {
    if (t.size() == 1) {
      name(t[0]);
      return;
    }
    out += "(either";
    for (Symbol s : t) {
      out += ' ';
      name(s);
    }
    out += ')';
  }

  // Groups consecutive names with the same type: "a b - t c"
  void typed_list(const std::vector<ast::TypedName> & list)
  {
    for (size_t i = 0; i < list.size(); ++i) {
      if (i > 0) {
        out += ' ';
      }
      name(list[i].name);
      bool last_of_group = i + 1 == list.size() || list[i + 1].types != list[i].types;
      if (last_of_group && !list[i].types.empty()) {
        out += " - ";
        types(list[i].types);
      }
    }
  }

  void term(TermId id)
  {
    const ast::Term & t = nodes_.terms[id];
    if (t.kind != ast::TermKind::Function) {
      name(t.name);
      return;
    }
    out += '(';
    name(t.name);
    terms(t.args);
    out += ')';
  }

  void terms(const std::vector<TermId> & args)
  {
    for (TermId a : args) {
      out += ' ';
      term(a);
    }
  }

  void expr(ExprId id)
  {
    const ast::Expr & e = nodes_.exprs[id];
    switch (e.kind) {
      case ast::ExprKind::Number: out += format_number(e.number); return;
      case ast::ExprKind::Term: term(e.term); return;
      case ast::ExprKind::Duration: out += "?duration"; return;
      case ast::ExprKind::TotalTime: out += "(total-time)"; return;
      case ast::ExprKind::ContinuousT: out += "#t"; return;
      case ast::ExprKind::IsViolated:
        out += "(is-violated ";
        name(e.name);
        out += ')';
        return;
      case ast::ExprKind::Function:
        out += '(';
        name(e.name);
        terms(e.args);
        out += ')';
        return;
      case ast::ExprKind::Add: op("+", e.children); return;
      case ast::ExprKind::Sub: op("-", e.children); return;
      case ast::ExprKind::Mul: op("*", e.children); return;
      case ast::ExprKind::Div: op("/", e.children); return;
      case ast::ExprKind::Pow: op("^", e.children); return;
      case ast::ExprKind::Math: op(symbols_.name(e.name), e.children); return;
      case ast::ExprKind::Neg: op("-", e.children); return;
    }
  }

  void op(std::string_view symbol, const std::vector<ExprId> & children)
  {
    out += '(';
    out += symbol;
    for (ExprId c : children) {
      out += ' ';
      expr(c);
    }
    out += ')';
  }

  void formula(FormulaId id)
  {
    const ast::Formula & f = nodes_.formulas[id];
    switch (f.kind) {
      case ast::FormulaKind::And: list("and", f.children); return;
      case ast::FormulaKind::Or: list("or", f.children); return;
      case ast::FormulaKind::Not: list("not", f.children); return;
      case ast::FormulaKind::Imply: list("imply", f.children); return;
      case ast::FormulaKind::AtStart: list("at start", f.children); return;
      case ast::FormulaKind::AtEnd: list("at end", f.children); return;
      case ast::FormulaKind::OverAll: list("over all", f.children); return;
      case ast::FormulaKind::Exists:
      case ast::FormulaKind::Forall:
        out += f.kind == ast::FormulaKind::Exists ? "(exists (" : "(forall (";
        typed_list(f.params);
        out += ") ";
        formula(f.children.at(0));
        out += ')';
        return;
      case ast::FormulaKind::Atom:
        out += '(';
        name(f.name);
        terms(f.args);
        out += ')';
        return;
      case ast::FormulaKind::Compare:
        out += '(';
        out += compare_text(f.compare);
        out += ' ';
        expr(f.lhs);
        out += ' ';
        expr(f.rhs);
        out += ')';
        return;
      case ast::FormulaKind::Preference:
        out += "(preference ";
        if (f.name.valid()) {
          name(f.name);
          out += ' ';
        }
        formula(f.children.at(0));
        out += ')';
        return;
      case ast::FormulaKind::Modal:
        out += '(';
        out += modal_text(f.modal);
        for (double t : f.times) {
          out += ' ';
          out += format_number(t);
        }
        for (FormulaId c : f.children) {
          out += ' ';
          formula(c);
        }
        out += ')';
        return;
    }
  }

  void list(std::string_view head, const std::vector<FormulaId> & children)
  {
    out += '(';
    out += head;
    for (FormulaId c : children) {
      out += ' ';
      formula(c);
    }
    out += ')';
  }

  void effect(EffectId id)
  {
    const ast::Effect & e = nodes_.effects[id];
    switch (e.kind) {
      case ast::EffectKind::And:
        out += "(and";
        for (EffectId c : e.children) {
          out += ' ';
          effect(c);
        }
        out += ')';
        return;
      case ast::EffectKind::Forall:
        out += "(forall (";
        typed_list(e.params);
        out += ") ";
        effect(e.children.at(0));
        out += ')';
        return;
      case ast::EffectKind::When:
        out += "(when ";
        formula(e.condition);
        out += ' ';
        effect(e.children.at(0));
        out += ')';
        return;
      case ast::EffectKind::Add:
        out += '(';
        name(e.name);
        terms(e.args);
        out += ')';
        return;
      case ast::EffectKind::Delete:
        out += "(not (";
        name(e.name);
        terms(e.args);
        out += "))";
        return;
      case ast::EffectKind::Assign:
        out += '(';
        out += assign_text(e.op);
        out += ' ';
        expr(e.target);
        out += ' ';
        expr(e.value);
        out += ')';
        return;
      case ast::EffectKind::AtStart:
      case ast::EffectKind::AtEnd:
        out += e.kind == ast::EffectKind::AtStart ? "(at start " : "(at end ";
        effect(e.children.at(0));
        out += ')';
        return;
    }
  }

  static std::string_view compare_text(ast::CompareOp op)
  {
    switch (op) {
      case ast::CompareOp::Eq: return "=";
      case ast::CompareOp::Lt: return "<";
      case ast::CompareOp::Le: return "<=";
      case ast::CompareOp::Gt: return ">";
      case ast::CompareOp::Ge: return ">=";
    }
    return "=";
  }

  static std::string_view assign_text(ast::AssignOp op)
  {
    switch (op) {
      case ast::AssignOp::Assign: return "assign";
      case ast::AssignOp::ScaleUp: return "scale-up";
      case ast::AssignOp::ScaleDown: return "scale-down";
      case ast::AssignOp::Increase: return "increase";
      case ast::AssignOp::Decrease: return "decrease";
    }
    return "assign";
  }

  static std::string_view modal_text(ast::ModalOp op)
  {
    switch (op) {
      case ast::ModalOp::Always: return "always";
      case ast::ModalOp::Sometime: return "sometime";
      case ast::ModalOp::Within: return "within";
      case ast::ModalOp::AtMostOnce: return "at-most-once";
      case ast::ModalOp::SometimeAfter: return "sometime-after";
      case ast::ModalOp::SometimeBefore: return "sometime-before";
      case ast::ModalOp::AlwaysWithin: return "always-within";
      case ast::ModalOp::HoldDuring: return "hold-during";
      case ast::ModalOp::HoldAfter: return "hold-after";
    }
    return "always";
  }

  void requirements(const std::vector<ast::Requirement> & reqs)
  {
    if (reqs.empty()) {
      return;
    }
    out += "  (:requirements";
    for (const auto & r : reqs) {
      out += ' ';
      name(r.name);
    }
    out += ")\n";
  }

  void skeleton(Symbol head, const std::vector<ast::TypedName> & params)
  {
    out += '(';
    name(head);
    if (!params.empty()) {
      out += ' ';
      typed_list(params);
    }
    out += ')';
  }

  void action_like(const char * keyword, const ast::Action & a)
  {
    out += "\n  (";
    out += keyword;
    out += ' ';
    name(a.name);
    out += "\n    :parameters (";
    typed_list(a.params);
    out += ')';
    if (a.precondition.valid()) {
      out += "\n    :precondition ";
      formula(a.precondition);
    }
    if (a.effect.valid()) {
      out += "\n    :effect ";
      effect(a.effect);
    }
    out += ")\n";
  }

private:
  const ast::Nodes & nodes_;
  const SymbolTable & symbols_;
};

}  // namespace

std::string format_number(double value)
{
  std::array<char, 64> buffer{};
  auto [ptr, ec] = std::to_chars(buffer.data(), buffer.data() + buffer.size(), value);
  if (ec != std::errc()) {
    return "0";
  }
  return std::string(buffer.data(), ptr);
}

std::string to_pddl(const ast::Domain & d)
{
  Printer p(d.nodes, d.symbols);
  p.out += "(define (domain ";
  p.name(d.name);
  p.out += ")\n";
  p.requirements(d.requirements);
  if (!d.types.empty()) {
    p.out += "  (:types ";
    p.typed_list(d.types);
    p.out += ")\n";
  }
  if (!d.constants.empty()) {
    p.out += "  (:constants ";
    p.typed_list(d.constants);
    p.out += ")\n";
  }
  if (!d.predicates.empty()) {
    p.out += "  (:predicates";
    for (const auto & pred : d.predicates) {
      p.out += "\n    ";
      p.skeleton(pred.name, pred.params);
    }
    p.out += ")\n";
  }
  if (!d.functions.empty()) {
    p.out += "  (:functions";
    for (const auto & f : d.functions) {
      p.out += "\n    ";
      p.skeleton(f.name, f.params);
      if (!f.type.empty()) {
        p.out += " - ";
        p.types(f.type);
      }
    }
    p.out += ")\n";
  }
  if (d.constraints.valid()) {
    p.out += "  (:constraints ";
    p.formula(d.constraints);
    p.out += ")\n";
  }
  for (const auto & a : d.actions) {
    p.action_like(":action", a);
  }
  for (const auto & a : d.durative_actions) {
    p.out += "\n  (:durative-action ";
    p.name(a.name);
    p.out += "\n    :parameters (";
    p.typed_list(a.params);
    p.out += ")\n    :duration ";
    p.formula(a.duration);
    if (a.condition.valid()) {
      p.out += "\n    :condition ";
      p.formula(a.condition);
    }
    if (a.effect.valid()) {
      p.out += "\n    :effect ";
      p.effect(a.effect);
    }
    p.out += ")\n";
  }
  for (const auto & derived : d.derived) {
    p.out += "\n  (:derived ";
    p.skeleton(derived.head.name, derived.head.params);
    p.out += "\n    ";
    p.formula(derived.body);
    p.out += ")\n";
  }
  for (const auto & a : d.processes) {
    p.action_like(":process", a);
  }
  for (const auto & a : d.events) {
    p.action_like(":event", a);
  }
  for (const auto & c : d.global_constraints) {
    p.out += "\n  (:constraint ";
    p.name(c.name);
    p.out += "\n    :parameters (";
    p.typed_list(c.params);
    p.out += ")\n    :condition ";
    p.formula(c.condition);
    p.out += ")\n";
  }
  p.out += ")\n";
  return p.out;
}

std::string to_pddl(const ast::Problem & pr)
{
  Printer p(pr.nodes, pr.symbols);
  p.out += "(define (problem ";
  p.name(pr.name);
  p.out += ")\n";
  if (pr.domain_name.valid()) {
    p.out += "  (:domain ";
    p.name(pr.domain_name);
    p.out += ")\n";
  }
  p.requirements(pr.requirements);
  if (!pr.objects.empty()) {
    p.out += "  (:objects ";
    p.typed_list(pr.objects);
    p.out += ")\n";
  }
  p.out += "  (:init";
  for (const auto & item : pr.init) {
    p.out += "\n    ";
    if (item.time) {
      p.out += "(at " + format_number(*item.time) + ' ';
    }
    if (item.kind == ast::InitItem::Kind::Fluent) {
      p.out += "(= (";
      p.name(item.name);
      p.terms(item.args);
      p.out += ") ";
      p.expr(item.value);
      p.out += ')';
    } else {
      if (item.negated) {
        p.out += "(not ";
      }
      p.out += '(';
      p.name(item.name);
      p.terms(item.args);
      p.out += ')';
      if (item.negated) {
        p.out += ')';
      }
    }
    if (item.time) {
      p.out += ')';
    }
  }
  p.out += ")\n";
  if (pr.goal.valid()) {
    p.out += "  (:goal ";
    p.formula(pr.goal);
    p.out += ")\n";
  }
  if (pr.constraints.valid()) {
    p.out += "  (:constraints ";
    p.formula(pr.constraints);
    p.out += ")\n";
  }
  if (pr.metric) {
    p.out += pr.metric->minimize ? "  (:metric minimize " : "  (:metric maximize ";
    p.expr(pr.metric->expr);
    p.out += ")\n";
  }
  p.out += ")\n";
  return p.out;
}

std::string to_pddl(const ast::Nodes & nodes, const SymbolTable & symbols, ast::FormulaId id)
{
  Printer p(nodes, symbols);
  p.formula(id);
  return p.out;
}

std::string to_pddl(const ast::Nodes & nodes, const SymbolTable & symbols, ast::EffectId id)
{
  Printer p(nodes, symbols);
  p.effect(id);
  return p.out;
}

std::string to_pddl(const ast::Nodes & nodes, const SymbolTable & symbols, ast::ExprId id)
{
  Printer p(nodes, symbols);
  p.expr(id);
  return p.out;
}

std::string to_pddl(const ast::Nodes & nodes, const SymbolTable & symbols, ast::TermId id)
{
  Printer p(nodes, symbols);
  p.term(id);
  return p.out;
}

}  // namespace kairos::pddl
