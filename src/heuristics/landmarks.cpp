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
        lg.landmarks_[p].needed_by.push_back(id);
        lg.landmarks_[id].parents.push_back(p);
      }
    }
  }
  lg.reasonable_orderings(task, budget);
  return lg;
}

// a interferes with b: achieving b before a would mean destroying b to achieve a (FD): a and
// b mutex, every achiever of a destroying b, or a greedy-necessary parent of a mutex with b
bool LandmarkGraph::interferes(const sas::Task & task, uint32_t a, uint32_t b) const
{
  const Landmark & la = landmarks_[a];
  const Landmark & lb = landmarks_[b];
  // Mutex: two values of a variable, or in one of the mutex groups found (also those that did
  // not become variables: (clear b) and (on a b) in blocks)
  auto mutex = [&](const sas::Fact & x, const sas::Fact & y) {
      if (task.variables[x.var].kind == sas::Variable::Kind::Derived || x == y) {
        return false;
      }
      if (x.var == y.var) {
        return true;
      }
      auto gx = groups_of_.find({x.var, x.value});
      auto gy = groups_of_.find({y.var, y.value});
      if (gx == groups_of_.end() || gy == groups_of_.end()) {
        return false;
      }
      for (uint32_t g : gx->second) {
        if (std::binary_search(gy->second.begin(), gy->second.end(), g)) {
          return true;
        }
      }
      return false;
    };
  if (a == b || task.variables[lb.fact.var].kind == sas::Variable::Kind::Derived) {
    return false;
  }
  if (mutex(la.fact, lb.fact)) {
    return true;
  }
  if (!la.achievers.empty() && la.cost > 0.0) {
    bool all = true;
    for (uint32_t act : la.achievers) {
      const auto & changes = task.actions[act].changes;
      all = std::any_of(changes.begin(), changes.end(),
          [&](const sas::Fact & e) {return mutex(e, lb.fact);});
      if (!all) {
        break;
      }
    }
    if (all) {
      return true;
    }
  }
  for (uint32_t x : la.parents) {
    const auto & needed = landmarks_[x].needed_by;
    if (std::find(needed.begin(), needed.end(), a) != needed.end() &&
      mutex(landmarks_[x].fact, lb.fact))
    {
      return true;
    }
  }
  return false;
}

void LandmarkGraph::reasonable_orderings(const sas::Task & task, Budget * budget)
{
  const auto n = static_cast<uint32_t>(landmarks_.size());
  groups_of_.clear();
  for (uint32_t g = 0; g < task.mutex_groups.size(); ++g) {
    for (const auto & f : task.mutex_groups[g]) {
      groups_of_[{f.var, f.value}].push_back(g);
    }
  }
  std::vector<std::pair<uint32_t, uint32_t>> found;  // (before, after)
  for (uint32_t b = 0; b < n; ++b) {
    if (budget && budget->exhausted()) {
      return;
    }
    const Landmark & lb = landmarks_[b];
    if (lb.initially_true) {
      continue;
    }
    if (lb.goal) {
      // A goal achieved before a landmark that destroys it is achieved again
      for (uint32_t a = 0; a < n; ++a) {
        if (interferes(task, a, b)) {
          found.emplace_back(a, b);
        }
      }
      continue;
    }
    // The other parents of what b is needed for, and their ancestors
    std::vector<uint8_t> interesting(n, 0);
    std::vector<uint32_t> stack;
    for (uint32_t c : lb.needed_by) {
      for (uint32_t p : landmarks_[c].parents) {
        if (p != b && !interesting[p]) {
          interesting[p] = 1;
          stack.push_back(p);
        }
      }
    }
    while (!stack.empty()) {
      uint32_t x = stack.back();
      stack.pop_back();
      for (uint32_t p : landmarks_[x].parents) {
        if (p != b && !interesting[p]) {
          interesting[p] = 1;
          stack.push_back(p);
        }
      }
    }
    for (uint32_t a = 0; a < n; ++a) {
      if (interesting[a] && interferes(task, a, b)) {
        found.emplace_back(a, b);
      }
    }
  }
  if (found.empty()) {
    return;
  }
  // Strongly connected components with every ordering: reasonable ones inside one are dropped
  std::vector<std::vector<uint32_t>> succ(n);
  for (uint32_t i = 0; i < n; ++i) {
    succ[i] = landmarks_[i].children;
  }
  for (auto [a, b] : found) {
    succ[a].push_back(b);
  }
  std::vector<int32_t> comp(n, -1), low(n, 0), num(n, -1);
  std::vector<uint32_t> st;
  std::vector<uint8_t> on(n, 0);
  int32_t counter = 0, comps = 0;
  for (uint32_t root = 0; root < n; ++root) {
    if (num[root] >= 0) {
      continue;
    }
    // Iterative Tarjan: (node, next successor)
    std::vector<std::pair<uint32_t, size_t>> work{{root, 0}};
    num[root] = low[root] = counter++;
    st.push_back(root);
    on[root] = 1;
    while (!work.empty()) {
      auto & [v, i] = work.back();
      if (i < succ[v].size()) {
        uint32_t w = succ[v][i++];
        if (num[w] < 0) {
          num[w] = low[w] = counter++;
          st.push_back(w);
          on[w] = 1;
          work.emplace_back(w, 0);
        } else if (on[w]) {
          low[v] = std::min(low[v], num[w]);
        }
        continue;
      }
      if (low[v] == num[v]) {
        uint32_t w;
        do {
          w = st.back();
          st.pop_back();
          on[w] = 0;
          comp[w] = comps;
        } while (w != v);
        ++comps;
      }
      uint32_t done = v;
      work.pop_back();
      if (!work.empty()) {
        low[work.back().first] = std::min(low[work.back().first], low[done]);
      }
    }
  }
  for (auto [a, b] : found) {
    if (comp[a] == comp[b]) {
      continue;
    }
    auto & children = landmarks_[a].children;
    if (std::find(children.begin(), children.end(), b) == children.end()) {
      children.push_back(b);
      landmarks_[b].parents.push_back(a);
      landmarks_[b].reasonable.push_back(a);
    }
  }
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
      future_.assign(lm_.landmarks().size(), 0);
      for (uint32_t i = 0; i < lm_.landmarks().size(); ++i) {
        const auto & f = lm_.landmarks()[i].fact;
        lm_of_.emplace((uint64_t{f.var} << 32) | f.value, i);
      }
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
    // Reached (past, as FD): true in some state of the path, whatever came before
    for (uint32_t i = 0; i < lm_.landmarks().size(); ++i) {
      if (!get(scratch_.data(), i) && holds(i, child)) {
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
    for (uint32_t i = 0; i < ls.size(); ++i) {
      // Future (FD): not reached; or reached, false now, and a goal or needed by a landmark
      // not reached (greedy-necessary); or reached before a reasonable parent
      bool future = !get(reached, i);
      if (!future && !holds(i, ctx)) {
        future = needed_again(i, reached);
      }
      if (!future) {
        future = std::any_of(ls[i].reasonable.begin(), ls[i].reasonable.end(),
            [&](uint32_t p) {return !get(reached, p);});
      }
      if (!future) {
        continue;
      }
      r.value += ls[i].cost;
      future_[i] = 1;
      if (ctx.applicable) {
        continue;
      }
      for (uint32_t a : ls[i].achievers) {
        if (achieves_now(a, ls[i].fact, ctx)) {
          r.preferred.push_back(a);
        }
      }
    }
    if (ctx.applicable) {
      // Preferred, as FD: of the applicable actions (hundreds), those that make a future
      // landmark true now, instead of every achiever of each (thousands in hiking)
      for (uint32_t a : *ctx.applicable) {
        if (achieves_future(a, ctx)) {
          r.preferred.push_back(a);
        }
      }
    } else {
      std::sort(r.preferred.begin(), r.preferred.end());
      r.preferred.erase(std::unique(r.preferred.begin(), r.preferred.end()), r.preferred.end());
    }
    std::fill(future_.begin(), future_.end(), 0);
    return r;
  }

private:
  // Applicable and, here, makes the fact true: an unconditional change, or a conditional
  // effect whose condition holds (as FD; else every action with some conditional effect on it
  // is preferred: rubiks-cube, where all 12 are, needed 18 times the expansions)
  bool achieves_now(uint32_t a, const sas::Fact & f, const EvalContext & ctx) const
  {
    const auto & act = t_.actions[a];
    if (!state::holds(act.pre, t_, ctx.values, ctx.numbers)) {
      return false;
    }
    for (const auto & c : act.changes) {
      if (c == f) {
        return true;
      }
    }
    for (const auto & e : act.effects) {
      if (e.kind == sas::Effect::Kind::Assign && e.fact == f &&
        state::holds(e.condition, t_, ctx.values, ctx.numbers))
      {
        return true;
      }
      if (e.kind == sas::Effect::Kind::Copy && e.target == f.var &&
        ctx.values[e.source] == f.value)
      {
        return true;
      }
    }
    return false;
  }

  bool future_fact(sas::VarId var, uint32_t value) const
  {
    auto it = lm_of_.find((uint64_t{var} << 32) | value);
    return it != lm_of_.end() && future_[it->second];
  }

  // Applicable (known), and makes some future landmark true now
  bool achieves_future(uint32_t a, const EvalContext & ctx) const
  {
    const auto & act = t_.actions[a];
    for (const auto & c : act.changes) {
      if (future_fact(c.var, c.value)) {
        return true;
      }
    }
    for (const auto & e : act.effects) {
      if (e.kind == sas::Effect::Kind::Assign && future_fact(e.fact.var, e.fact.value) &&
        state::holds(e.condition, t_, ctx.values, ctx.numbers))
      {
        return true;
      }
      if (e.kind == sas::Effect::Kind::Copy && future_fact(e.target, ctx.values[e.source])) {
        return true;
      }
    }
    return false;
  }

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
      l.needed_by.begin(), l.needed_by.end(), [&](uint32_t c) {return !get(reached, c);});
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
  size_t words_ = 0;
  std::vector<uint64_t> data_;   // reached landmarks per state, words_ words each
  std::vector<uint8_t> known_;
  std::vector<uint64_t> scratch_;
  std::vector<uint8_t> future_;                   // during evaluate
  std::unordered_map<uint64_t, uint32_t> lm_of_;  // fact (var << 32 | value): its landmark
  size_t accounted_ = 0;
};

}  // namespace

std::unique_ptr<Heuristic> make_landmark_count(
  const sas::Task & task, const std::vector<double> & costs, Budget * budget)
{
  return std::make_unique<LandmarkCount>(task, costs, budget);
}

}  // namespace kairos::heuristics
