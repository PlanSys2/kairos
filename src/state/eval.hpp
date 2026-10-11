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
/// \brief Expressions, conditions and axioms on unpacked states.

#ifndef KAIROS__STATE__EVAL_HPP_
#define KAIROS__STATE__EVAL_HPP_

#include <cstdint>
#include <limits>
#include <vector>

#include "sas/task.hpp"

namespace kairos::state
{

/// @brief Value of an undefined fluent (a quiet NaN).
inline constexpr double kUndefined = std::numeric_limits<double>::quiet_NaN();

/// @brief Value of an expression; NaN if it reads an undefined fluent or divides by zero (as
/// the validator, which rejects the step).
double evaluate(const ground::Expr & e, const double * numbers, double duration = kUndefined);

/// @brief The same, through the linear form when there is one.
double evaluate(const sas::NumericExpr & e, const double * numbers, double duration = kUndefined);

/// @brief a op b; false if either is NaN.
bool compare(sas::CompareOp op, double a, double b);

/// @brief The condition holds in the state (values of all variables, numbers per numeric
/// variable).
bool holds(
  const sas::Condition & c, const sas::Task & task, const uint32_t * values, const double * numbers,
  double duration = kUndefined);

/**
 * @brief Derived variables from the basic ones, layer by layer (as FD): in each layer every
 * axiom counts its unsatisfied conditions on variables of that layer; when one fires, its head
 * becomes true and the axioms waiting for it count down.
 *
 * Not thread-safe: it keeps scratch space.
 */
class AxiomEvaluator
{
public:
  explicit AxiomEvaluator(const sas::Task & task);

  /// @brief Sets every derived variable in `values` (the basic ones are read).
  void evaluate(uint32_t * values, const double * numbers) const;

  bool empty() const {return layers_.empty();}

private:
  struct Rule
  {
    sas::Condition fixed;              // on basic or lower-layer variables
    std::vector<sas::VarId> waiting;   // derived variables of its layer it needs true
    sas::VarId head;
  };
  struct Layer
  {
    std::vector<sas::VarId> vars;
    std::vector<uint32_t> rules;
  };

  const sas::Task & task_;
  std::vector<Rule> rules_;
  std::vector<Layer> layers_;
  std::vector<std::vector<uint32_t>> watchers_;  // per variable: rules waiting for it
  mutable std::vector<uint32_t> pending_;        // per rule: conditions still unsatisfied
  mutable std::vector<sas::VarId> queue_;
};

}  // namespace kairos::state

#endif  // KAIROS__STATE__EVAL_HPP_
