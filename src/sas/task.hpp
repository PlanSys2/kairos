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
/// \brief SAS+ task: multi-valued variables, numeric variables and actions over them.

#ifndef KAIROS__SAS__TASK_HPP_
#define KAIROS__SAS__TASK_HPP_

#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "ground/task.hpp"

namespace kairos::sas
{

using ground::CompareOp;
using ground::ExprOp;
using ground::Timing;
using sem::ObjectId;

using VarId = uint32_t;
using NumVarId = uint32_t;  // same index as the ground task's numeric fluents
constexpr uint32_t kNoAtom = UINT32_MAX;

struct Fact
{
  VarId var = 0;
  uint32_t value = 0;

  bool operator==(const Fact &) const = default;
  auto operator<=>(const Fact &) const = default;
};

/**
 * @brief A multi-valued variable.
 *
 * - Mutex group: one value per atom, then "none of those".
 * - Binary: value 0 is the atom true, 1 false. Derived atoms are binary, with an axiom layer
 *   and default value 1.
 * - Object fluent: one value per object it can take, then "undefined".
 */
struct Variable
{
  enum class Kind : uint8_t {Group, Binary, Derived, ObjectFluent};

  Kind kind = Kind::Binary;
  std::vector<ground::AtomId> atoms;  // Group, Binary, Derived: the atom of each value (or kNoAtom)
  std::vector<ObjectId> objects;      // ObjectFluent: the object of each value but the last
  ground::FluentId fluent = 0;        // ObjectFluent
  int layer = -1;                     // Derived: axiom layer
  uint32_t default_value = 0;         // Derived: 1 (false)

  uint32_t size() const
  {
    return static_cast<uint32_t>(kind == Kind::ObjectFluent ? objects.size() + 1 :
           (kind == Kind::Group ? atoms.size() + 1 : 2));
  }
  uint32_t none() const {return size() - 1;}  // "none of those", false, or undefined
};

/// @brief Σ coef·x + constant.
struct LinearExpr
{
  std::vector<std::pair<NumVarId, double>> terms;
  double constant = 0.0;

  bool operator==(const LinearExpr &) const = default;
};

/// @brief Numeric expression as a tree (always) and as a linear form (when it is one).
struct NumericExpr
{
  ground::Expr tree;
  std::optional<LinearExpr> linear;

  bool operator==(const NumericExpr &) const = default;
};

/// @brief lhs op rhs; `difference` is lhs - rhs (linear when both are).
struct NumericCondition
{
  CompareOp op = CompareOp::Eq;
  NumericExpr lhs;
  NumericExpr rhs;
  std::optional<LinearExpr> difference;

  bool operator==(const NumericCondition &) const = default;
};

/// @brief Two object-fluent variables hold the same object (or not).
struct VarComparison
{
  VarId a = 0;
  VarId b = 0;
  bool equal = true;

  bool operator==(const VarComparison &) const = default;
};

struct Condition
{
  std::vector<Fact> facts;
  std::vector<Fact> not_facts;  // var != value (negated atoms of mutex groups)
  std::vector<NumericCondition> numeric;
  std::vector<VarComparison> vars;

  bool empty() const
  {
    return facts.empty() && not_facts.empty() && numeric.empty() && vars.empty();
  }
  bool operator==(const Condition &) const = default;
};

struct Effect
{
  enum class Kind : uint8_t {Assign, Numeric, Copy};

  Kind kind = Kind::Assign;
  Timing timing = Timing::Instant;
  Condition condition;
  Condition start_condition;           // durative end effects: read at start
  Fact fact;                           // Assign: var := value
  NumVarId num = 0;                    // Numeric
  ast::AssignOp op = ast::AssignOp::Assign;
  NumericExpr value;                   // Numeric
  bool continuous = false;
  VarId target = 0;                    // Copy: target := source (object fluents)
  VarId source = 0;

  bool operator==(const Effect &) const = default;
};

struct Action
{
  uint32_t schema = 0;
  std::vector<ObjectId> args;
  Condition pre;
  std::vector<Fact> changes;    // unconditional var := value
  std::vector<Effect> effects;  // the rest
};

struct DurationConstraint
{
  CompareOp op = CompareOp::Eq;
  Timing timing = Timing::Start;
  NumericExpr value;
};

struct DurativeAction
{
  uint32_t schema = 0;
  std::vector<ObjectId> args;
  std::vector<DurationConstraint> duration;
  Condition at_start;
  Condition over_all;
  Condition at_end;
  std::vector<Fact> start;      // unconditional changes at start
  std::vector<Fact> end;        // and at end
  std::vector<Effect> effects;  // the rest
  bool uses_duration = false;   // ?duration read by an effect or condition
};

struct Axiom
{
  Condition body;
  Fact head;  // derived variable := 0 (true)
};

struct TimedEvent
{
  double time = 0.0;
  std::vector<Effect> effects;
};

struct Metric
{
  bool minimize = true;
  NumericExpr expr;
  bool is_default = false;
};

struct Task
{
  // Names, for printing (copied from the ground task)
  std::string domain_name;
  std::string problem_name;
  std::vector<std::string> predicate_names;
  std::vector<std::string> function_names;
  std::vector<std::string> object_names;
  std::vector<ground::AtomInfo> atoms;
  std::vector<ground::FluentInfo> numeric_fluents;
  std::vector<ground::FluentInfo> object_fluents;
  std::vector<std::string> action_names;
  std::vector<std::string> durative_action_names;
  std::vector<std::string> process_names;
  std::vector<std::string> event_names;

  std::vector<Variable> variables;
  std::vector<std::pair<VarId, uint32_t>> atom_fact;  // per ground atom: its variable and value
  std::vector<uint32_t> initial;                      // value per variable
  std::vector<std::optional<double>> initial_numeric;  // per numeric variable

  std::vector<Action> actions;
  std::vector<DurativeAction> durative_actions;
  std::vector<Action> processes;
  std::vector<Action> events;
  std::vector<Axiom> axioms;     // sorted by the layer of their head
  int axiom_layers = 0;
  std::vector<TimedEvent> timed_events;

  Condition goal;
  bool goal_unreachable = false;
  Metric metric;

  sem::FeatureSet features;
  bool has_soft_goals = false;
  bool has_trajectory_constraints = false;
};

}  // namespace kairos::sas

#endif  // KAIROS__SAS__TASK_HPP_
