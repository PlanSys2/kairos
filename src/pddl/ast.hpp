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
/// \brief Syntax tree of PDDL 3.1 domains and problems, including PDDL+.

#ifndef KAIROS__PDDL__AST_HPP_
#define KAIROS__PDDL__AST_HPP_

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "core/id.hpp"
#include "core/symbol_table.hpp"

namespace kairos::ast
{

/// @brief Position of a node in its file (1-based); 0 means unknown.
struct Loc
{
  uint32_t line = 0;
  uint32_t column = 0;

  bool operator==(const Loc &) const = default;
};

using TermId = Id<struct TermTag>;
using ExprId = Id<struct ExprTag>;
using FormulaId = Id<struct FormulaTag>;
using EffectId = Id<struct EffectTag>;

/// @brief A name with its type: "?r - robot", "kitchen - room", "a - (either b c)".
struct TypedName
{
  Symbol name;
  std::vector<Symbol> types;  // empty: no type given; several: (either ...)
  Loc loc;
  Loc type_loc;  // where the type is written (unknown if no type)
};

// --- Terms -----------------------------------------------------------------------------------

enum class TermKind : uint8_t
{
  Name,      // object or constant
  Variable,  // ?x (name includes the '?')
  Function   // (f t1 ... tn), PDDL 3.1 object fluents
};

struct Term
{
  TermKind kind = TermKind::Name;
  Symbol name;
  std::vector<TermId> args;  // Function only
  Loc loc;
};

// --- Numeric expressions ---------------------------------------------------------------------

enum class ExprKind : uint8_t
{
  Number,
  Term,         // a name or variable used as a value: (= ?x ?y), (assign (f) obj)
  Function,     // (f t1 ... tn)
  Add,          // (+ e1 e2 ...), n-ary
  Sub,          // (- e1 e2)
  Mul,          // (* e1 e2 ...), n-ary
  Div,          // (/ e1 e2)
  Pow,          // (^ e1 e2), ENHSP extension
  Neg,          // (- e)
  Duration,     // ?duration
  TotalTime,    // total-time or (total-time)
  ContinuousT,  // #t
  IsViolated,   // (is-violated pref), metrics only
  Math          // (sin e), (cos e), (abs e)...: ENHSP built-ins, name in `name`
};

struct Expr
{
  ExprKind kind = ExprKind::Number;
  double number = 0.0;           // Number
  Symbol name;                   // Function head, IsViolated preference
  TermId term;                   // Term
  std::vector<TermId> args;      // Function arguments
  std::vector<ExprId> children;  // operators
  Loc loc;
};

// --- Formulas (goal descriptions) ------------------------------------------------------------

enum class FormulaKind : uint8_t
{
  And,         // (and ...); also the empty formula ()
  Or,
  Not,
  Imply,
  Exists,
  Forall,
  Atom,        // (p t1 ... tn)
  Compare,     // (op e1 e2), including (= t1 t2)
  AtStart,     // durative conditions
  AtEnd,       // durative conditions and (at end ...) in constraints
  OverAll,
  Preference,  // (preference [name] gd)
  Modal        // PDDL 3 trajectory constraints
};

enum class CompareOp : uint8_t {Eq, Lt, Le, Gt, Ge};

enum class ModalOp : uint8_t
{
  Always,
  Sometime,
  Within,          // (within t gd)
  AtMostOnce,
  SometimeAfter,   // (sometime-after gd gd)
  SometimeBefore,
  AlwaysWithin,    // (always-within t gd gd)
  HoldDuring,      // (hold-during t1 t2 gd)
  HoldAfter        // (hold-after t gd)
};

struct Formula
{
  FormulaKind kind = FormulaKind::And;
  std::vector<FormulaId> children;
  Symbol name;                    // Atom predicate, Preference name (may be invalid)
  std::vector<TermId> args;       // Atom
  std::vector<TypedName> params;  // Exists, Forall
  CompareOp compare = CompareOp::Eq;
  ExprId lhs;                     // Compare
  ExprId rhs;
  ModalOp modal = ModalOp::Always;
  std::vector<double> times;      // Modal time arguments
  Loc loc;
};

// --- Effects ---------------------------------------------------------------------------------

enum class EffectKind : uint8_t
{
  And,      // also the empty effect ()
  Forall,
  When,
  Add,      // (p t1 ... tn)
  Delete,   // (not (p ...))
  Assign,   // assign, scale-up, scale-down, increase, decrease (also continuous, with #t)
  AtStart,
  AtEnd
};

enum class AssignOp : uint8_t {Assign, ScaleUp, ScaleDown, Increase, Decrease};

struct Effect
{
  EffectKind kind = EffectKind::And;
  std::vector<EffectId> children;  // And, Forall/When/AtStart/AtEnd body (one child)
  std::vector<TypedName> params;   // Forall
  FormulaId condition;             // When
  Symbol name;                     // Add, Delete
  std::vector<TermId> args;        // Add, Delete
  AssignOp op = AssignOp::Assign;
  ExprId target;                   // Assign: the function, (f t1 ... tn)
  ExprId value;                    // Assign: the new value or change
  Loc loc;
};

/// @brief Storage of all nodes of one file; ids index into it.
struct Nodes
{
  IdVector<TermId, Term> terms;
  IdVector<ExprId, Expr> exprs;
  IdVector<FormulaId, Formula> formulas;
  IdVector<EffectId, Effect> effects;
};

// --- Declarations ----------------------------------------------------------------------------

struct Requirement
{
  Symbol name;  // with the colon, e.g. ":durative-actions"
  Loc loc;
};

struct PredicateDecl
{
  Symbol name;
  std::vector<TypedName> params;
  Loc loc;
};

struct FunctionDecl
{
  Symbol name;
  std::vector<TypedName> params;
  std::vector<Symbol> type;  // empty: not given (number); "number"; or object type(s)
  Loc loc;
};

/// @brief :action, :process and :event (the latter two only use precondition and effect).
struct Action
{
  Symbol name;
  std::vector<TypedName> params;
  FormulaId precondition;  // invalid if absent
  EffectId effect;         // invalid if absent
  Loc loc;
};

struct DurativeAction
{
  Symbol name;
  std::vector<TypedName> params;
  FormulaId duration;   // duration constraint, with ExprKind::Duration on one side
  FormulaId condition;  // invalid if absent
  EffectId effect;      // invalid if absent
  Loc loc;
};

/// @brief (:constraint name :parameters (...) :condition gd), ENHSP global constraint.
struct GlobalConstraint
{
  Symbol name;
  std::vector<TypedName> params;
  FormulaId condition;
  Loc loc;
};

struct Derived
{
  PredicateDecl head;
  FormulaId body;
  Loc loc;
};

struct Domain
{
  std::string file;
  SymbolTable symbols;
  Nodes nodes;
  Symbol name;
  Loc loc;
  std::vector<Requirement> requirements;
  std::vector<TypedName> types;
  std::vector<TypedName> constants;
  std::vector<PredicateDecl> predicates;
  std::vector<FunctionDecl> functions;
  FormulaId constraints;  // invalid if absent
  std::vector<Action> actions;
  std::vector<DurativeAction> durative_actions;
  std::vector<Derived> derived;
  std::vector<Action> processes;
  std::vector<Action> events;
  std::vector<GlobalConstraint> global_constraints;
};

// --- Problem ---------------------------------------------------------------------------------

struct InitItem
{
  enum class Kind : uint8_t {Fact, Fluent};

  Kind kind = Kind::Fact;
  std::optional<double> time;  // (at t ...): timed initial literal or fluent
  bool negated = false;        // (not (p ...)), allowed in timed literals
  Symbol name;                 // predicate or function
  std::vector<TermId> args;
  ExprId value;                // Fluent: number or object name
  Loc loc;
};

struct Metric
{
  bool minimize = true;
  ExprId expr;
  Loc loc;
};

struct Problem
{
  std::string file;
  SymbolTable symbols;
  Nodes nodes;
  Symbol name;
  Symbol domain_name;
  Loc loc;
  std::vector<Requirement> requirements;
  std::vector<TypedName> objects;
  std::vector<InitItem> init;
  FormulaId goal;         // invalid if absent
  FormulaId constraints;  // invalid if absent
  std::optional<Metric> metric;
};

}  // namespace kairos::ast

#endif  // KAIROS__PDDL__AST_HPP_
