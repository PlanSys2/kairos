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

#include "temporal/stn.hpp"

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

namespace kairos::temporal
{

namespace
{

// Bounds move only by more than this: rounding never loops or reports false cycles
constexpr double kTolerance = 1e-9;

}  // namespace

Stn::Stn()
{
  lb_.push_back(0.0);
  ub_.push_back(0.0);
  out_.emplace_back();
  in_.emplace_back();
}

Stn::Node Stn::add_node()
{
  lb_.push_back(0.0);
  ub_.push_back(kInfinity);
  out_.emplace_back();
  in_.emplace_back();
  return static_cast<Node>(lb_.size() - 1);
}

bool Stn::exactly(Node i, Node j, double d)
{
  Mark m = mark();
  if (add(i, j, d) && add(j, i, -d)) {
    return true;
  }
  undo(m);
  return false;
}

bool Stn::between(Node i, Node j, double lo, double hi)
{
  Mark m = mark();
  if (add(i, j, hi) && add(j, i, -lo)) {
    return true;
  }
  undo(m);
  return false;
}

void Stn::set_lb(Node n, double v)
{
  trail_.push_back(Change{n, false, lb_[n]});
  lb_[n] = v;
}

void Stn::set_ub(Node n, double v)
{
  trail_.push_back(Change{n, true, ub_[n]});
  ub_[n] = v;
}

// t_j - t_i <= w gives t_i >= t_j - w: a larger earliest time of j raises those of the nodes
// before it. Reaching `forbidden` (the other end of the new constraint) closes a negative cycle
bool Stn::propagate_lb(Node start, Node forbidden)
{
  queue_.clear();
  queue_.push_back(start);
  for (size_t q = 0; q < queue_.size(); ++q) {
    Node n = queue_[q];
    for (const Arc & a : in_[n]) {
      double v = lb_[n] - a.w;
      if (v > lb_[a.other] + kTolerance) {
        if (a.other == forbidden || v > ub_[a.other] + kTolerance) {
          return false;
        }
        set_lb(a.other, v);
        queue_.push_back(a.other);
      }
    }
  }
  return true;
}

// t_j <= t_i + w: a smaller latest time of i lowers those of the nodes after it
bool Stn::propagate_ub(Node start, Node forbidden)
{
  queue_.clear();
  queue_.push_back(start);
  for (size_t q = 0; q < queue_.size(); ++q) {
    Node n = queue_[q];
    for (const Arc & a : out_[n]) {
      double v = ub_[n] + a.w;
      if (v < ub_[a.other] - kTolerance) {
        if (a.other == forbidden || v < lb_[a.other] - kTolerance) {
          return false;
        }
        set_ub(a.other, v);
        queue_.push_back(a.other);
      }
    }
  }
  return true;
}

bool Stn::add(Node i, Node j, double w)
{
  if (std::isnan(w) || i >= nodes() || j >= nodes()) {
    return false;
  }
  if (i == j) {
    return w >= -kTolerance;  // t - t <= w
  }
  Mark m = mark();
  out_[i].push_back(Arc{j, w});
  in_[j].push_back(Arc{i, w});
  edges_.emplace_back(i, j);
  bool ok = true;
  double lo = lb_[j] - w;
  if (lo > lb_[i] + kTolerance) {
    ok = lo <= ub_[i] + kTolerance;
    if (ok) {
      set_lb(i, lo);
      ok = propagate_lb(i, j);
    }
  }
  if (ok) {
    double hi = ub_[i] + w;
    if (hi < ub_[j] - kTolerance) {
      ok = hi >= lb_[j] - kTolerance;
      if (ok) {
        set_ub(j, hi);
        ok = propagate_ub(j, i);
      }
    }
  }
  if (!ok) {
    undo(m);
  }
  return ok;
}

void Stn::undo(const Mark & m)
{
  while (edges_.size() > m.edges) {
    auto [i, j] = edges_.back();
    edges_.pop_back();
    out_[i].pop_back();
    in_[j].pop_back();
  }
  while (trail_.size() > m.trail) {
    const Change & c = trail_.back();
    (c.upper ? ub_ : lb_)[c.node] = c.old;
    trail_.pop_back();
  }
  lb_.resize(m.nodes);
  ub_.resize(m.nodes);
  out_.resize(m.nodes);
  in_.resize(m.nodes);
}

}  // namespace kairos::temporal
