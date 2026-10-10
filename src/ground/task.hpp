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
/// \brief Ground task: reachable atoms, fluents and instantiated actions.

#ifndef KAIROS__GROUND__TASK_HPP_
#define KAIROS__GROUND__TASK_HPP_

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "normalize/task.hpp"

namespace kairos::ground
{

using lifted::CompareOp;
using lifted::ExprOp;
using lifted::Timing;
using sem::FunctionId;
using sem::ObjectId;
using sem::PredicateId;
using sem::TypeSpec;

using AtomId = uint32_t;
using FluentId = uint32_t;

/// @brief A ground atom (non-static: static atoms are evaluated away).
struct AtomInfo
{
  PredicateId predicate;
  std::vector<ObjectId> args;
};

/// @brief A ground numeric or object fluent (non-static ones; static ones become constants).
struct FluentInfo
{
  FunctionId function;
  std::vector<ObjectId> args;
};

/// @brief Ground numeric expression; Fluent reads task.numeric_fluents[fluent].
struct Expr
{
  ExprOp op = ExprOp::Constant;
  double value = 0.0;          // Constant
  FluentId fluent = 0;         // Fluent
  std::string name;            // Math function or preference
  std::vector<Expr> children;

  bool operator==(const Expr &) const = default;
};

struct NumericCondition
{
  CompareOp op = CompareOp::Eq;
  Expr lhs;
  Expr rhs;

  bool operator==(const NumericCondition &) const = default;
};

/// @brief An object, or the value of a (non-static) object fluent.
struct ObjectTerm
{
  bool is_fluent = false;
  uint32_t index = 0;  // ObjectId value, or index into task.object_fluents

  bool operator==(const ObjectTerm &) const = default;
};

struct ObjectCondition
{
  bool equal = true;  // = or !=
  ObjectTerm lhs;
  ObjectTerm rhs;

  bool operator==(const ObjectCondition &) const = default;
};

/// @brief A conjunction over non-static atoms and fluents.
struct Condition
{
  std::vector<AtomId> positive;
  std::vector<AtomId> negative;
  std::vector<NumericCondition> numeric;
  std::vector<ObjectCondition> objects;

  bool empty() const
  {
    return positive.empty() && negative.empty() && numeric.empty() && objects.empty();
  }
  bool operator==(const Condition &) const = default;
};

struct Effect
{
  enum class Kind : uint8_t {Add, Delete, Numeric, ObjectAssign};

  Kind kind = Kind::Add;
  Timing timing = Timing::Instant;
  Condition condition;
  Condition start_condition;           // durative end effects: read at start
  AtomId atom = 0;                     // Add, Delete
  FluentId fluent = 0;                 // Numeric (numeric_fluents), ObjectAssign (object_fluents)
  ast::AssignOp op = ast::AssignOp::Assign;
  Expr value;                          // Numeric
  std::optional<ObjectTerm> object;    // ObjectAssign (nullopt: undefined)
  bool continuous = false;

  bool operator==(const Effect &) const = default;
};

struct DurationConstraint
{
  CompareOp op = CompareOp::Eq;
  Timing timing = Timing::Start;
  Expr value;

  bool operator==(const DurationConstraint &) const = default;
};

/// @brief Unconditional changes of atoms at one moment (the common case, kept compact).
/// An atom never appears in both: deleting and adding it leaves it true.
struct AtomChanges
{
  std::vector<AtomId> add;
  std::vector<AtomId> del;

  bool empty() const {return add.empty() && del.empty();}
};

/// @brief An instantiated action, process or event.
struct Action
{
  uint32_t schema = 0;            // index into the lifted task's list it comes from
  std::vector<ObjectId> args;     // printed parameters
  Condition precondition;
  AtomChanges changes;            // unconditional add and delete effects
  std::vector<Effect> effects;    // the rest: conditional, numeric, object fluents
};

struct DurativeAction
{
  uint32_t schema = 0;
  std::vector<ObjectId> args;
  std::vector<DurationConstraint> duration;
  Condition at_start;
  Condition over_all;
  Condition at_end;
  AtomChanges start;              // unconditional atom changes at start
  AtomChanges end;                // and at end
  std::vector<Effect> effects;    // the rest
};

struct Axiom
{
  AtomId head = 0;
  Condition body;
};

struct TimedEvent
{
  enum class Kind : uint8_t {Add, Delete, Numeric, Object};

  double time = 0.0;
  Kind kind = Kind::Add;
  uint32_t index = 0;               // atom, numeric fluent or object fluent
  double number = 0.0;              // Numeric
  std::optional<ObjectId> object;   // Object
};

struct Metric
{
  bool minimize = true;
  Expr expr;
  bool is_default = false;
};

struct Task
{
  std::string domain_name;
  std::string problem_name;
  std::vector<std::string> predicate_names;
  std::vector<bool> derived;  // per predicate
  std::vector<std::string> function_names;
  std::vector<std::string> object_names;

  std::vector<AtomInfo> atoms;
  std::vector<FluentInfo> numeric_fluents;
  std::vector<FluentInfo> object_fluents;

  std::vector<AtomId> init_atoms;                        // true at the start (basic atoms)
  std::vector<std::optional<double>> init_numeric;       // per numeric fluent
  std::vector<std::optional<ObjectId>> init_objects;     // per object fluent

  // Names of the lifted schemas, per list (Action::schema indexes them)
  std::vector<std::string> action_names;
  std::vector<std::string> durative_action_names;
  std::vector<std::string> process_names;
  std::vector<std::string> event_names;

  std::vector<Action> actions;
  std::vector<DurativeAction> durative_actions;
  std::vector<Action> processes;
  std::vector<Action> events;
  std::vector<Axiom> axioms;
  std::vector<TimedEvent> timed_events;

  Condition goal;
  bool goal_unreachable = false;  // the goal needs an atom that no action can reach
  Metric metric;

  sem::FeatureSet features;
  bool has_soft_goals = false;
  bool has_trajectory_constraints = false;

  bool is_derived(AtomId a) const {return derived[atoms[a].predicate.index()];}
};

}  // namespace kairos::ground

#endif  // KAIROS__GROUND__TASK_HPP_
