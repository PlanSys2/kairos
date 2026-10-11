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
/// \brief Delete relaxation with numeric subgoaling: hmax, hadd and hFF.

#ifndef KAIROS__HEURISTICS__RELAXED_HPP_
#define KAIROS__HEURISTICS__RELAXED_HPP_

#include <cstdint>
#include <functional>
#include <map>
#include <queue>
#include <utility>
#include <vector>

#include "heuristics/heuristic.hpp"

namespace kairos::heuristics
{

/**
 * @brief The relaxed task as a graph, as FD's relaxation heuristics:
 * - nodes: one per fact (variable, value); one per negated fact used (var != value), reached
 *   when another value is; one per action (its precondition holds); one per linear numeric
 *   condition (as e >= 0 or e > 0); the goal;
 * - unary operators: action precondition -> action node; action node and effect condition ->
 *   effect fact (with the action cost); axiom body -> head (cost 0); goal.
 * - a numeric condition not true in the state is reached through any action that moves e up
 *   by d > 0 in that state, repeated k times (k * d covers what is missing).
 *
 * Conditions on derived variables being false, non-linear comparisons and object comparisons
 * count as satisfied (optimistic, so hmax stays admissible).
 */
class RelaxedGraph
{
public:
  enum class Mode : uint8_t {Max, Add};

  RelaxedGraph(const sas::Task & task, const std::vector<double> & costs);

  /// @brief Cost of the goal in the relaxation (kInfinity if unreachable).
  double run(const EvalContext & ctx, Mode mode);

  /// @brief After run(Add): cost of a relaxed plan built from the best supporters, and the
  /// actions of it applicable in the state (preferred operators).
  double relaxed_plan(std::vector<uint32_t> * preferred);

  /// @brief Every numeric effect adds a constant: the numeric part of hmax is admissible.
  bool simple_numeric() const {return simple_numeric_;}

private:
  static constexpr uint32_t kNone = UINT32_MAX;

  struct Op
  {
    std::vector<uint32_t> pre;
    uint32_t effect;
    double base;
    uint32_t action;  // the action whose effect it is, kNone otherwise
  };
  struct NumNode
  {
    sas::LinearExpr e;
    bool strict = false;
    std::vector<uint32_t> contributors;  // actions changing a variable of e
  };
  struct NumEffect
  {
    sas::NumVarId var;
    ast::AssignOp op;
    sas::NumericExpr value;
  };

  uint32_t add_node();
  uint32_t neg_node(sas::VarId var, uint32_t value);
  uint32_t num_node(const sas::LinearExpr & e, bool strict);
  void condition(const sas::Condition & c, std::vector<uint32_t> & out);
  void add_op(std::vector<uint32_t> pre, uint32_t effect, double base, uint32_t action);
  double change(uint32_t action, const NumNode & n, const EvalContext & ctx) const;
  double missing(const NumNode & n, const EvalContext & ctx) const;
  uint32_t repetitions(const NumNode & n, double missing, double change) const;
  void relax(uint32_t node, double cost, uint32_t supporter, uint32_t repeat);

  const sas::Task & task_;
  std::vector<double> costs_;
  bool simple_numeric_ = true;
  uint32_t nodes_ = 0;
  std::vector<uint32_t> fact_offset_;          // per variable
  std::map<std::pair<sas::VarId, uint32_t>, uint32_t> neg_;
  std::vector<std::pair<sas::VarId, uint32_t>> neg_fact_;  // per negated node, in order
  std::vector<uint32_t> neg_id_;                           // and its node id
  std::vector<uint32_t> act_node_;             // per action
  std::vector<NumNode> num_;
  std::vector<uint32_t> num_node_id_;          // per numeric node: its node id
  std::vector<int32_t> node_num_;              // per node: numeric node index or -1
  std::vector<int32_t> node_action_;           // per node: action of an action node or -1
  std::vector<std::vector<NumEffect>> num_effects_;     // per action
  std::vector<std::vector<uint32_t>> action_num_;       // per action: numeric nodes it changes
  uint32_t goal_ = 0;
  std::vector<Op> ops_;
  std::vector<std::vector<uint32_t>> triggers_;  // per node: ops it is a precondition of
  std::vector<uint32_t> act_op_;                 // per action: its precondition op

  // Per evaluation
  Mode mode_ = Mode::Add;
  std::vector<double> cost_;
  std::vector<double> op_cost_;
  std::vector<uint32_t> unsatisfied_;
  std::vector<uint32_t> supporter_;   // op (or action, for numeric nodes)
  std::vector<uint32_t> repeat_;      // numeric nodes: repetitions of the supporter
  std::vector<uint8_t> initial_;      // true in the state itself
  std::vector<double> missing_;       // per numeric node
  std::vector<double> best_ratio_;    // per numeric node: least cost per unit (hmax)
  using Entry = std::pair<double, uint32_t>;
  std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> queue_;
  const EvalContext * ctx_ = nullptr;
};

}  // namespace kairos::heuristics

#endif  // KAIROS__HEURISTICS__RELAXED_HPP_
