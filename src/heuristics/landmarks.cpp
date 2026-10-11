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

#include "heuristics/landmarks.hpp"

#include <algorithm>
#include <cstddef>
#include <cmath>
#include <deque>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "state/eval.hpp"

namespace kairos::heuristics
{

namespace
{

// Fact nodes an op needs: for an action effect, the action's precondition and the effect
// condition; for an axiom, its body
std::vector<uint32_t> needs(const RelaxedGraph & g, const RelaxedGraph::Op & op)
{
  std::vector<uint32_t> out;
  for (uint32_t p : op.pre) {
    int32_t a = g.node_action(p);
    if (a >= 0) {
      const auto & pre = g.ops()[g.action_op(static_cast<uint32_t>(a))].pre;
      out.insert(out.end(), pre.begin(), pre.end());
    } else {
      out.push_back(p);
    }
  }
  std::sort(out.begin(), out.end());
  out.erase(std::unique(out.begin(), out.end()), out.end());
  return out;
}

}  // namespace

LandmarkGraph LandmarkGraph::build(
  const sas::Task & task, RelaxedGraph & graph, const EvalContext & state,
  const std::vector<double> & costs, Budget * budget)
{
  LandmarkGraph lg;
  // Achieving ops per fact node
  std::unordered_map<uint32_t, std::vector<uint32_t>> achievers;
  const auto & ops = graph.ops();
  for (uint32_t o = 0; o < ops.size(); ++o) {
    if (graph.fact_of(ops[o].effect)) {
      achievers[ops[o].effect].push_back(o);
    }
  }
  std::unordered_map<uint32_t, uint32_t> index;  // fact node -> landmark
  std::deque<uint32_t> queue;
  auto add = [&](uint32_t node) -> uint32_t {
      auto it = index.find(node);
      if (it != index.end()) {
        return it->second;
      }
      auto fact = *graph.fact_of(node);
      Landmark l;
      l.fact = sas::Fact{fact.first, fact.second};
      l.node = node;
      l.initially_true = state.values[fact.first] == fact.second;
      double cheapest = std::numeric_limits<double>::infinity();
      for (uint32_t o : achievers[node]) {
        if (ops[o].action != RelaxedGraph::kNone) {
          l.achievers.push_back(ops[o].action);
          cheapest = std::min(cheapest, costs[ops[o].action]);
        } else {
          cheapest = 0.0;  // an axiom
        }
      }
      std::sort(l.achievers.begin(), l.achievers.end());
      l.achievers.erase(std::unique(l.achievers.begin(), l.achievers.end()), l.achievers.end());
      l.cost = std::isinf(cheapest) ? 0.0 : cheapest;
      auto id = static_cast<uint32_t>(lg.landmarks_.size());
      lg.landmarks_.push_back(std::move(l));
      index.emplace(node, id);
      queue.push_back(id);
      return id;
    };
  // The goal facts
  for (uint32_t o = 0; o < ops.size(); ++o) {
    if (ops[o].effect != graph.goal_node()) {
      continue;
    }
    for (uint32_t p : ops[o].pre) {
      if (graph.fact_of(p)) {
        lg.landmarks_[add(p)].goal = true;
      }
    }
  }
  // Backwards: the facts every first achiever needs
  while (!queue.empty()) {
    if (budget && budget->exhausted()) {
      break;  // the landmarks so far are still landmarks
    }
    uint32_t id = queue.front();
    queue.pop_front();
    if (lg.landmarks_[id].initially_true) {
      continue;
    }
    uint32_t node = lg.landmarks_[id].node;
    auto reach = graph.reachable(state, node);
    std::vector<uint32_t> shared;
    bool first = true;
    for (uint32_t o : achievers[node]) {
      const auto & op = ops[o];
      bool usable = std::all_of(
        op.pre.begin(), op.pre.end(), [&](uint32_t p) {return reach[p] != 0;});
      if (!usable) {
        continue;
      }
      auto need = needs(graph, op);
      std::vector<uint32_t> facts;
      for (uint32_t n : need) {
        if (graph.fact_of(n) && n != node) {
          facts.push_back(n);
        }
      }
      if (first) {
        shared = std::move(facts);
        first = false;
      } else {
        std::vector<uint32_t> both;
        std::set_intersection(
          shared.begin(), shared.end(), facts.begin(), facts.end(), std::back_inserter(both));
        shared = std::move(both);
      }
      if (shared.empty()) {
        break;
      }
    }
    for (uint32_t n : shared) {
      uint32_t p = add(n);
      if (p == id) {
        continue;
      }
      auto & children = lg.landmarks_[p].children;
      if (std::find(children.begin(), children.end(), id) == children.end()) {
        children.push_back(id);
        lg.landmarks_[id].parents.push_back(p);
      }
    }
  }
  (void)task;
  return lg;
}

size_t LandmarkGraph::orderings() const
{
  size_t n = 0;
  for (const auto & l : landmarks_) {
    n += l.parents.size();
  }
  return n;
}

namespace
{

class LandmarkCount : public Heuristic
{
public:
  LandmarkCount(const sas::Task & t, const std::vector<double> & costs, Budget * budget)
  : t_(t), costs_(costs), graph_(t, costs), budget_(budget) {}

  ~LandmarkCount() override
  {
    if (budget_ && accounted_) {
      budget_->account(-static_cast<std::ptrdiff_t>(accounted_));
    }
  }

  bool path_dependent() const override {return true;}
  std::string name() const override {return "lmcount";}

  void initial(const EvalContext & ctx) override
  {
    if (!built_) {
      lm_ = LandmarkGraph::build(t_, graph_, ctx, costs_, budget_);
      words_ = (lm_.landmarks().size() + 63) / 64;
      order_ = topological(lm_);
      built_ = true;
    }
    uint64_t * r = slot(ctx.state);
    std::fill(r, r + words_, 0);
    const auto & ls = lm_.landmarks();
    for (uint32_t i = 0; i < ls.size(); ++i) {
      if (holds(i, ctx)) {
        set(r, i);
      }
    }
  }

  void transition(state::StateId parent, uint32_t, const EvalContext & child) override
  {
    if (!built_ || words_ == 0) {
      return;
    }
    scratch_.assign(slot(parent), slot(parent) + words_);
    const auto & ls = lm_.landmarks();
    // Parents before children: a derived landmark holds in the same state as the body of its
    // axiom, so what this transition reaches counts too
    for (uint32_t i : order_) {
      if (get(scratch_.data(), i) || !holds(i, child)) {
        continue;
      }
      bool ready = std::all_of(
        ls[i].parents.begin(), ls[i].parents.end(),
        [&](uint32_t p) {return get(scratch_.data(), p);});
      if (ready) {
        set(scratch_.data(), i);
      }
    }
    bool known = known_.size() > child.state && known_[child.state];
    uint64_t * c = slot(child.state);
    for (size_t w = 0; w < words_; ++w) {
      c[w] = known ? (c[w] & scratch_[w]) : scratch_[w];
    }
  }

  HeuristicResult evaluate(const EvalContext & ctx) override
  {
    HeuristicResult r;
    if (t_.goal_unreachable) {
      r.dead_end = true;
      r.value = kInfinity;
      return r;
    }
    if (!built_) {
      initial(ctx);
    }
    const uint64_t * reached = slot(ctx.state);
    const auto & ls = lm_.landmarks();
    std::vector<uint8_t> marked_action;
    for (uint32_t i = 0; i < ls.size(); ++i) {
      bool interesting = false;
      if (!get(reached, i)) {
        r.value += ls[i].cost;
        interesting = std::all_of(
          ls[i].parents.begin(), ls[i].parents.end(), [&](uint32_t p) {return get(reached, p);});
      } else if (!holds(i, ctx) && needed_again(i, reached)) {
        r.value += ls[i].cost;
        interesting = true;
      }
      if (!interesting) {
        continue;
      }
      for (uint32_t a : ls[i].achievers) {
        if (marked_action.empty()) {
          marked_action.assign(t_.actions.size(), 0);
        }
        if (marked_action[a]) {
          continue;
        }
        marked_action[a] = 1;
        if (state::holds(t_.actions[a].pre, t_, ctx.values, ctx.numbers)) {
          r.preferred.push_back(a);
        }
      }
    }
    return r;
  }

private:
  bool holds(uint32_t i, const EvalContext & ctx) const
  {
    const auto & f = lm_.landmarks()[i].fact;
    return ctx.values[f.var] == f.value;
  }

  bool needed_again(uint32_t i, const uint64_t * reached) const
  {
    const auto & l = lm_.landmarks()[i];
    if (l.goal) {
      return true;
    }
    return std::any_of(
      l.children.begin(), l.children.end(), [&](uint32_t c) {return !get(reached, c);});
  }

  // Landmarks with parents first (orderings form no cycles but, if they did, the rest go last)
  static std::vector<uint32_t> topological(const LandmarkGraph & lg)
  {
    const auto & ls = lg.landmarks();
    std::vector<uint32_t> pending(ls.size()), out;
    std::vector<uint32_t> ready;
    for (uint32_t i = 0; i < ls.size(); ++i) {
      pending[i] = static_cast<uint32_t>(ls[i].parents.size());
      if (pending[i] == 0) {
        ready.push_back(i);
      }
    }
    std::vector<uint8_t> done(ls.size(), 0);
    while (!ready.empty()) {
      uint32_t i = ready.back();
      ready.pop_back();
      out.push_back(i);
      done[i] = 1;
      for (uint32_t c : ls[i].children) {
        if (--pending[c] == 0) {
          ready.push_back(c);
        }
      }
    }
    for (uint32_t i = 0; i < ls.size(); ++i) {
      if (!done[i]) {
        out.push_back(i);
      }
    }
    return out;
  }

  static bool get(const uint64_t * bits, uint32_t i) {return (bits[i / 64] >> (i % 64)) & 1;}
  static void set(uint64_t * bits, uint32_t i) {bits[i / 64] |= uint64_t{1} << (i % 64);}

  uint64_t * slot(state::StateId s)
  {
    size_t need = (static_cast<size_t>(s) + 1) * words_;
    if (data_.size() < need) {
      size_t grown = std::max(need, data_.size() * 2);
      if (budget_) {
        budget_->account(static_cast<std::ptrdiff_t>((grown - data_.size()) * sizeof(uint64_t)));
      }
      accounted_ += (grown - data_.size()) * sizeof(uint64_t);
      data_.resize(grown, 0);
    }
    if (known_.size() <= s) {
      known_.resize(std::max<size_t>(s + 1, known_.size() * 2), 0);
    }
    known_[s] = 1;
    return data_.data() + static_cast<size_t>(s) * words_;
  }

  const sas::Task & t_;
  std::vector<double> costs_;
  RelaxedGraph graph_;
  Budget * budget_;
  bool built_ = false;
  LandmarkGraph lm_;
  std::vector<uint32_t> order_;  // topological, for acceptance
  size_t words_ = 0;
  std::vector<uint64_t> data_;   // reached landmarks per state, words_ words each
  std::vector<uint8_t> known_;
  std::vector<uint64_t> scratch_;
  size_t accounted_ = 0;
};

}  // namespace

std::unique_ptr<Heuristic> make_landmark_count(
  const sas::Task & task, const std::vector<double> & costs, Budget * budget)
{
  return std::make_unique<LandmarkCount>(task, costs, budget);
}

}  // namespace kairos::heuristics
