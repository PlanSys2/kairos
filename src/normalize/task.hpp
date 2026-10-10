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
/// \brief Normalized lifted task: what grounding works on.

#ifndef KAIROS__NORMALIZE__TASK_HPP_
#define KAIROS__NORMALIZE__TASK_HPP_

#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "pddl/ast.hpp"
#include "semantic/model.hpp"

namespace kairos::lifted
{

using sem::FunctionId;
using sem::ObjectId;
using sem::PredicateId;
using sem::TypeSpec;

/// @brief A variable of an action or axiom, indexed in its owner's variable table.
struct Variable
{
  std::string name;  // as written ("?r"), or generated for auxiliary variables
  TypeSpec types;
};

/// @brief A variable (index into the owner's variables), an object, or an object fluent.
struct Term
{
  enum class Kind : uint8_t {Variable, Object, Function};

  Kind kind = Kind::Variable;
  uint32_t index = 0;       // variable index, ObjectId or FunctionId value
  std::vector<Term> args;   // Function only

  static Term variable(uint32_t v) {return Term{Kind::Variable, v, {}};}
  static Term object(ObjectId o) {return Term{Kind::Object, o.value(), {}};}

  bool operator==(const Term &) const = default;
};

struct Atom
{
  PredicateId predicate;
  std::vector<Term> args;

  bool operator==(const Atom &) const = default;
};

struct Literal
{
  Atom atom;
  bool positive = true;

  bool operator==(const Literal &) const = default;
};

enum class ExprOp : uint8_t
{
  Constant, Fluent, Add, Sub, Mul, Div, Neg, Pow, Math, Duration, TotalTime, ContinuousT,
  Violated, StepCount
};

/// @brief Numeric expression with resolved functions and variables.
struct Expr
{
  ExprOp op = ExprOp::Constant;
  double value = 0.0;              // Constant
  FunctionId function;             // Fluent
  std::vector<Term> args;          // Fluent
  std::string name;                // Math function ("sin"...) or preference (Violated)
  std::vector<Expr> children;

  bool operator==(const Expr &) const = default;
};

enum class CompareOp : uint8_t {Eq, Ne, Lt, Le, Gt, Ge};

/// @brief Numeric comparison, or (in)equality between objects.
struct Comparison
{
  CompareOp op = CompareOp::Eq;
  bool objects = false;  // compares terms (lhs_term, rhs_term) instead of expressions
  Expr lhs;
  Expr rhs;
  Term lhs_term;
  Term rhs_term;

  bool operator==(const Comparison &) const = default;
};

struct Quantified;

/// @brief A conjunction: literals, comparisons and universally quantified conjunctions.
struct Condition
{
  std::vector<Literal> literals;
  std::vector<Comparison> comparisons;
  std::vector<Quantified> foralls;

  bool empty() const {return literals.empty() && comparisons.empty() && foralls.empty();}
  bool operator==(const Condition & other) const;
};

/// @brief forall (vars) body: vars are indices into the owner's variables.
struct Quantified
{
  std::vector<uint32_t> variables;
  Condition body;

  bool operator==(const Quantified &) const = default;
};

enum class Timing : uint8_t {Instant, Start, End};

struct Effect
{
  enum class Kind : uint8_t {Add, Delete, Numeric, ObjectAssign};

  Kind kind = Kind::Add;
  Timing timing = Timing::Instant;
  std::vector<uint32_t> variables;  // forall-bound variables
  Condition condition;              // evaluated when the effect happens
  Condition start_condition;        // durative end effects: part of the condition read at start
  Atom atom;                        // Add, Delete
  FunctionId function;              // Numeric, ObjectAssign
  std::vector<Term> args;
  ast::AssignOp op = ast::AssignOp::Assign;
  Expr value;                       // Numeric
  std::optional<Term> object;       // ObjectAssign (nullopt: undefined)
  bool continuous = false;          // uses #t: changes during the action

  bool operator==(const Effect &) const = default;
};

struct DurationConstraint
{
  CompareOp op = CompareOp::Eq;  // ?duration op value (Eq, Le or Ge)
  Timing timing = Timing::Start;
  Expr value;

  bool operator==(const DurationConstraint &) const = default;
};

/// @brief An instantaneous action, a process or an event.
struct Action
{
  std::string name;
  std::vector<Variable> variables;
  uint32_t parameters = 0;  // the first variables; printed in plans
  uint32_t hidden = 0;      // then existential variables, chosen by grounding but not printed
  Condition precondition;
  std::vector<Effect> effects;
};

struct DurativeAction
{
  std::string name;
  std::vector<Variable> variables;
  uint32_t parameters = 0;
  uint32_t hidden = 0;
  std::vector<DurationConstraint> duration;
  Condition at_start;
  Condition over_all;
  Condition at_end;
  std::vector<Effect> effects;  // timing Start or End, or continuous
};

/**
 * @brief One rule for a derived predicate (several rules mean a disjunction).
 *
 * Variables: the head's, then the existential ones (chosen when grounding, like an action's
 * hidden parameters), then the ones bound by foralls inside the body (not chosen).
 */
struct Axiom
{
  PredicateId head;
  std::vector<Variable> variables;
  uint32_t head_arity = 0;
  uint32_t existential = 0;
  Condition body;
};

struct InitFluent
{
  FunctionId function;
  std::vector<ObjectId> args;
  double number = 0.0;
  std::optional<ObjectId> object;  // object fluents
  bool numeric = true;
};

struct TimedEvent
{
  double time = 0.0;
  std::optional<std::pair<PredicateId, std::vector<ObjectId>>> fact;
  bool negated = false;
  std::optional<InitFluent> fluent;
};

struct Metric
{
  bool minimize = true;
  Expr expr;
  bool is_default = false;  // no :metric in the problem
};

struct Task
{
  std::string domain_name;
  std::string problem_name;

  // Symbols, shared with the semantic model (predicates include auxiliary ones)
  std::vector<std::string> predicate_names;
  std::vector<bool> derived;  // per predicate
  std::vector<std::string> function_names;
  std::vector<bool> numeric;  // per function
  std::vector<std::string> object_names;
  std::vector<TypeSpec> object_types;
  sem::TypeHierarchy types;

  std::vector<Action> actions;
  std::vector<DurativeAction> durative_actions;
  std::vector<Action> processes;
  std::vector<Action> events;
  std::vector<Axiom> axioms;

  std::vector<std::pair<PredicateId, std::vector<ObjectId>>> init_facts;
  std::vector<InitFluent> init_fluents;
  std::vector<TimedEvent> timed_events;

  Condition goal;
  Metric metric;

  sem::FeatureSet features;
  bool has_soft_goals = false;        // preferences were set aside (handled from F4)
  bool has_trajectory_constraints = false;

  PredicateId add_predicate(const std::string & name, bool is_derived);
};

}  // namespace kairos::lifted

#endif  // KAIROS__NORMALIZE__TASK_HPP_
