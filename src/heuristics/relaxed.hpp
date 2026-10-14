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
#include <optional>
#include <queue>
#include <tuple>
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
 * A derived atom being false is reached when every rule that would make it true has a literal
 * that can be false (FD's negated axioms); with rules that read derived atoms or numbers it
 * counts as satisfied, as do non-linear comparisons and object comparisons (optimistic, so
 * hmax stays admissible).
 */
class RelaxedGraph
{
public:
  enum class Mode : uint8_t {Max, Add};

  /// With simplify (tasks without numeric conditions), one unary operator per effect, without
  /// duplicates or dominated ones (as Fast Downward), when that removes at least half of them
  /// and fits in memory: hmax and hadd do not change; action nodes and action_op() are then
  /// not used.
  RelaxedGraph(
    const sas::Task & task, const std::vector<double> & costs, bool simplify = false);

  /// @brief Cost of the goal in the relaxation (kInfinity if unreachable).
  double run(const EvalContext & ctx, Mode mode);

  /// @brief After run(Add): cost of a relaxed plan built from the best supporters, and the
  /// actions of it applicable in the state (preferred operators).
  double relaxed_plan(std::vector<uint32_t> * preferred);

  /// @brief Every numeric effect adds a constant: the numeric part of hmax is admissible.
  bool simple_numeric() const {return simple_numeric_;}

  // --- The graph, read-only (for landmarks) ---

  static constexpr uint32_t kNone = UINT32_MAX;
  static constexpr uint32_t kStalled = 1000000;  // repetitions of a clock that moves nothing now

  struct Op
  {
    std::vector<uint32_t> pre;
    uint32_t effect;
    double base;
    uint32_t action;  // the action whose effect it is, kNone otherwise
  };

  uint32_t nodes() const {return nodes_;}
  uint32_t fact_node(sas::VarId var, uint32_t value) const {return fact_offset_[var] + value;}
  /// @brief The (variable, value) of a fact node, or nullopt for other nodes.
  std::optional<std::pair<sas::VarId, uint32_t>> fact_of(uint32_t node) const;
  uint32_t goal_node() const {return goal_;}
  /// @brief Replaces the goal by these facts (the next run() reaches them instead); with any,
  /// the goal is reached with the first of them (goal_fact() tells which).
  void set_goal(const std::vector<sas::Fact> & facts, bool any = false);
  /// @brief With an "any" goal: the node of the fact that reached it in the last run().
  uint32_t goal_fact() const {return goal_trigger_;}
  /// @brief Cost of a node in the last run() (infinity if not reached before it stopped).
  double cost(uint32_t node) const
  {
    return node_[node].stamp == epoch_ ? node_[node].cost : kInfinity;
  }
  uint32_t action_node(uint32_t action) const {return act_node_[action];}
  /// @brief The action of an action node, or -1.
  int32_t node_action(uint32_t node) const {return node_action_[node];}
  const std::vector<Op> & ops() const {return ops_;}
  /// @brief Op of an action's precondition (its effect is the action node).
  uint32_t action_op(uint32_t action) const {return act_op_[action];}

  /// @brief Relaxed reachability from the state with `blocked` never reached (kNone: none):
  /// one byte per node, 1 if reachable.
  std::vector<uint8_t> reachable(const EvalContext & ctx, uint32_t blocked);

private:
  struct NumNode
  {
    sas::LinearExpr e;
    bool strict = false;
    std::vector<uint32_t> contributors;  // contributors changing a variable of e
  };
  struct NumEffect
  {
    sas::NumVarId var;
    ast::AssignOp op;
    sas::NumericExpr value;
    uint32_t id = 0;  // index into the per-evaluation cache of values
  };
  // Numeric effects of an action under one condition, reached at `node`: the action's node
  // (no condition), or a node of its own that also needs the condition (as FD's unary
  // operators: PDDL+ waits move a fluent only while the process that changes it runs)
  struct Contributor
  {
    uint32_t action;
    uint32_t node;
    std::vector<NumEffect> effects;
  };

  uint32_t add_node();
  uint32_t neg_node(sas::VarId var, uint32_t value);
  uint32_t false_node(sas::VarId var);
  uint32_t num_node(const sas::LinearExpr & e, bool strict);
  void condition(const sas::Condition & c, std::vector<uint32_t> & out);
  void add_op(std::vector<uint32_t> pre, uint32_t effect, double base, uint32_t action);
  void simplify();
  double change(uint32_t contributor, const NumNode & n, const EvalContext & ctx) const;
  double missing(const NumNode & n, const EvalContext & ctx) const;
  bool state_dependent(uint32_t contributor, const NumNode & n) const;
  uint32_t repetitions(const NumNode & n, double missing, double change) const;
  void relax(uint32_t node, double cost, uint32_t supporter, uint32_t repeat);
  void expand(uint32_t node, double cost);
  void explore(const EvalContext & ctx, bool stop_at_goal);
  void run_all(const EvalContext & ctx);

  const sas::Task & task_;
  std::vector<double> costs_;
  bool simple_numeric_ = true;
  uint32_t nodes_ = 0;
  std::vector<uint32_t> fact_offset_;          // per variable
  std::map<std::pair<sas::VarId, uint32_t>, uint32_t> neg_;
  std::map<sas::VarId, uint32_t> false_of_;  // derived variable: node of it being false
  // Numeric nodes by condition (strict, constant, sorted terms): shared by every action that
  // asks the same
  std::map<std::tuple<bool, double, std::vector<std::pair<sas::NumVarId, double>>>, uint32_t>
  num_ids_;
  std::vector<std::pair<sas::VarId, uint32_t>> neg_fact_;  // per negated node, in order
  std::vector<uint32_t> neg_id_;                           // and its node id
  std::vector<uint32_t> act_node_;             // per action
  std::vector<NumNode> num_;
  std::vector<uint32_t> num_node_id_;          // per numeric node: its node id
  std::vector<int32_t> node_num_;              // per node: numeric node index or -1
  std::vector<int32_t> node_action_;           // per node: action of an action node or -1
  std::vector<Contributor> contrib_;
  std::vector<std::vector<uint32_t>> contrib_num_;  // per contributor: numeric nodes it changes
  std::vector<int32_t> node_contrib_;               // per node: its contributor or -1
  uint32_t goal_ = 0;
  uint32_t goal_op_ = 0;
  uint32_t goal_trigger_ = kNone;
  std::vector<Op> ops_;
  std::vector<std::vector<uint32_t>> triggers_;  // per node: ops it is a precondition of
  std::vector<uint32_t> act_op_;                 // per action: its precondition op

  // Per evaluation
  Mode mode_ = Mode::Add;
  // Per-run values are reset lazily: a node or operator holds this run's values only if its
  // stamp is the run's epoch (a run that stops near the state touches a few of millions).
  // Each in one struct, so the hot loop reads one place per operator
  struct NodeState
  {
    double cost;
    uint32_t stamp = 0;
    uint32_t supporter;  // op (or contributor, for numeric nodes)
    uint32_t repeat;     // numeric nodes: repetitions of the supporter
    uint8_t initial;     // true in the state itself
  };
  struct OpState
  {
    double cost;
    uint32_t stamp = 0;
    uint32_t unsatisfied;
    uint32_t preconditions;  // what unsatisfied starts from
  };
  uint32_t epoch_ = 0;
  std::vector<NodeState> node_;
  std::vector<OpState> op_;
  std::vector<uint32_t> free_ops_;    // operators without preconditions
  std::vector<uint32_t> seen_stamp_;  // relaxed_plan
  uint32_t seen_epoch_ = 0;
  NodeState & touch(uint32_t n)
  {
    NodeState & x = node_[n];
    if (x.stamp != epoch_) {
      x = NodeState{kInfinity, epoch_, kNone, 0, 0};
    }
    return x;
  }
  OpState & touch_op(uint32_t o)
  {
    OpState & x = op_[o];
    if (x.stamp != epoch_) {
      x.cost = 0.0;
      x.stamp = epoch_;
      x.unsatisfied = x.preconditions;
    }
    return x;
  }
  bool fresh(uint32_t n) const {return node_[n].stamp == epoch_;}
  std::vector<double> missing_;       // per numeric node
  // The value of each numeric effect in the state of this evaluation, computed once
  mutable std::vector<double> effect_value_;
  mutable std::vector<uint32_t> effect_stamp_;
  uint32_t effects_ = 0;
  std::vector<double> best_ratio_;    // per numeric node: least cost per unit (hmax)
  // As FD's bucket queue: whole costs in buckets, last in first out within a cost (ties in the
  // best supporters, and so the relaxed plans, follow FD's); other costs in a heap
  struct Entry
  {
    double cost;
    uint64_t seq;
    uint32_t node;
    bool operator>(const Entry & o) const
    {
      return cost != o.cost ? cost > o.cost : seq < o.seq;
    }
  };
  void push(double cost, uint32_t node);
  bool pop(double & cost, uint32_t & node);
  static constexpr size_t kMaxBucket = 1 << 14;  // larger costs go to the heap: no sparse scans
  std::vector<std::vector<uint32_t>> buckets_;
  size_t bucket_ = 0;       // lowest bucket that may hold entries
  size_t in_buckets_ = 0;
  std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> heap_;
  uint64_t seq_ = 0;
  const EvalContext * ctx_ = nullptr;
  uint32_t blocked_ = kNone;
};

}  // namespace kairos::heuristics

#endif  // KAIROS__HEURISTICS__RELAXED_HPP_
