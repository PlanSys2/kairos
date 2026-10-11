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
/// \brief Simple Temporal Network with incremental consistency and cheap undo.

#ifndef KAIROS__TEMPORAL__STN_HPP_
#define KAIROS__TEMPORAL__STN_HPP_

#include <cstddef>
#include <cstdint>
#include <limits>
#include <utility>
#include <vector>

namespace kairos::temporal
{

/**
 * @brief Time points and difference constraints t_j - t_i <= w.
 *
 * Node 0 is the origin (time 0); every other node is at time 0 or later. Each node keeps the
 * earliest and latest time the constraints allow, updated incrementally as constraints come
 * (bound propagation, as Cesta and Oddi): a new constraint only visits the nodes whose bounds
 * change. A constraint that would make the network inconsistent (a negative cycle) is
 * rejected and leaves it as it was. Every change goes to a trail, so undo() goes back to a
 * mark without copying.
 */
class Stn
{
public:
  using Node = uint32_t;
  static constexpr Node kOrigin = 0;
  static constexpr double kInfinity = std::numeric_limits<double>::infinity();

  Stn();

  /// @brief A new time point at time 0 or later.
  Node add_node();
  size_t nodes() const {return lb_.size();}

  /// @brief t_j - t_i <= w. False, and nothing changes, if it makes the network inconsistent.
  bool add(Node i, Node j, double w);
  /// @brief t_j - t_i >= d.
  bool at_least(Node i, Node j, double d) {return add(j, i, -d);}
  /// @brief t_j - t_i == d.
  bool exactly(Node i, Node j, double d);
  /// @brief lo <= t_j - t_i <= hi.
  bool between(Node i, Node j, double lo, double hi);

  /// @brief The earliest time of a node: together they form the earliest schedule.
  double earliest(Node n) const {return lb_[n];}
  /// @brief The latest time (infinity if unbounded).
  double latest(Node n) const {return ub_[n];}

  struct Mark
  {
    size_t trail;
    size_t edges;
    size_t nodes;
  };
  Mark mark() const {return Mark{trail_.size(), edges_.size(), lb_.size()};}
  /// @brief Back to a mark: nodes, constraints and bounds added since are gone.
  void undo(const Mark & m);

  size_t constraints() const {return edges_.size();}

private:
  struct Arc
  {
    Node other;
    double w;
  };
  struct Change
  {
    Node node;
    bool upper;
    double old;
  };

  void set_lb(Node n, double v);
  void set_ub(Node n, double v);
  bool propagate_lb(Node start, Node forbidden);
  bool propagate_ub(Node start, Node forbidden);

  std::vector<double> lb_, ub_;
  std::vector<std::vector<Arc>> out_;  // i -> (j, w): t_j - t_i <= w
  std::vector<std::vector<Arc>> in_;   // j -> (i, w)
  std::vector<std::pair<Node, Node>> edges_;  // in order, for undo
  std::vector<Change> trail_;
  std::vector<Node> queue_;
};

}  // namespace kairos::temporal

#endif  // KAIROS__TEMPORAL__STN_HPP_
