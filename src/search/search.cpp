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

#include "search/search.hpp"

#include <algorithm>
#include <cmath>
#include <deque>
#include <memory>
#include <queue>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "heuristics/heuristic.hpp"

namespace kairos::search
{

using state::kNoState;
using state::StateId;

// --- Plan sink -------------------------------------------------------------------------------

PlanSink::PlanSink(std::function<void(const Entry &)> on_plan)
: on_plan_(std::move(on_plan)), start_(std::chrono::steady_clock::now()) {}

bool PlanSink::offer(std::vector<uint32_t> actions, double cost)
{
  Entry e{std::move(actions), cost, std::chrono::steady_clock::now() - start_};
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!(cost < bound_.load(std::memory_order_relaxed))) {
      return false;
    }
    plans_.push_back(e);
    bound_.store(cost, std::memory_order_release);
  }
  if (on_plan_) {
    on_plan_(e);  // outside the lock: the callback may ask for best()
  }
  return true;
}

std::optional<PlanSink::Entry> PlanSink::best() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (plans_.empty()) {
    return std::nullopt;
  }
  return plans_.back();
}

std::vector<PlanSink::Entry> PlanSink::all() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return plans_;
}

namespace
{

constexpr uint32_t kNoAction = UINT32_MAX;

// --- Per-state search information ------------------------------------------------------------

enum Status : uint8_t {kNew = 0, kOpen = 1, kClosed = 2, kDead = 3};

struct Node
{
  StateId parent = kNoState;
  uint32_t action = kNoAction;
  double g = 0.0;
  double h = 0.0;
  uint8_t status = kNew;
};

// Nodes by state id, in blocks that never move (32 bytes per state)
class SearchSpace
{
public:
  explicit SearchSpace(Budget * budget)
  : budget_(budget) {}
  ~SearchSpace()
  {
    if (budget_ && accounted_) {
      budget_->account(-static_cast<std::ptrdiff_t>(accounted_));
    }
  }
  SearchSpace(const SearchSpace &) = delete;
  SearchSpace & operator=(const SearchSpace &) = delete;

  Node & operator[](StateId s)
  {
    size_t b = s / kBlock;
    while (blocks_.size() <= b) {
      blocks_.push_back(std::make_unique<Node[]>(kBlock));
      accounted_ += kBlock * sizeof(Node);
      if (budget_) {
        budget_->account(static_cast<std::ptrdiff_t>(kBlock * sizeof(Node)));
      }
    }
    return blocks_[b][s % kBlock];
  }

  std::vector<uint32_t> path(StateId goal)
  {
    std::vector<uint32_t> out;
    for (StateId s = goal; (*this)[s].parent != kNoState; s = (*this)[s].parent) {
      out.push_back((*this)[s].action);
    }
    std::reverse(out.begin(), out.end());
    return out;
  }

private:
  static constexpr size_t kBlock = 1 << 14;
  Budget * budget_;
  std::vector<std::unique_ptr<Node[]>> blocks_;
  size_t accounted_ = 0;
};

// --- Open lists ------------------------------------------------------------------------------

struct Entry
{
  double key1;
  double key2;
  uint64_t order;    // first in, first out on ties
  StateId state;
  uint32_t action;   // lazy search: the operator to apply to state
  double g;          // eager search: g when pushed, to skip stale entries
};

struct Later
{
  bool operator()(const Entry & a, const Entry & b) const
  {
    if (a.key1 != b.key1) {
      return a.key1 > b.key1;
    }
    if (a.key2 != b.key2) {
      return a.key2 > b.key2;
    }
    return a.order > b.order;
  }
};

using Heap = std::priority_queue<Entry, std::vector<Entry>, Later>;

// All entries, and the preferred ones in a second queue; the queue with the lowest priority
// is popped next, and progress boosts the preferred one (as FD's alternation open list)
class Alternation
{
public:
  explicit Alternation(bool preferred)
  : queues_(preferred ? 2 : 1), priority_(queues_.size(), 0) {}

  void push(const Entry & e, bool preferred)
  {
    queues_[0].push(e);
    if (preferred && queues_.size() > 1) {
      queues_[1].push(e);
    }
  }

  bool empty() const
  {
    return std::all_of(queues_.begin(), queues_.end(), [](const Heap & h) {return h.empty();});
  }

  Entry pop()
  {
    size_t best = queues_.size();
    for (size_t i = 0; i < queues_.size(); ++i) {
      if (!queues_[i].empty() && (best == queues_.size() || priority_[i] < priority_[best])) {
        best = i;
      }
    }
    ++priority_[best];
    Entry e = queues_[best].top();
    queues_[best].pop();
    return e;
  }

  void boost(int amount)
  {
    if (queues_.size() > 1) {
      priority_[1] -= amount;
    }
  }

private:
  std::vector<Heap> queues_;
  std::vector<int64_t> priority_;
};

// --- Engines ---------------------------------------------------------------------------------

class Engine
{
public:
  Engine(SearchContext & ctx, const EngineOptions & o)
  : ctx_(ctx), o_(o), space_(ctx.budget),
    h_(heuristics::make_heuristic(o.heuristic, ctx.task, ctx.costs)) {}

  Outcome run()
  {
    if (!h_) {
      return Outcome::Failed;
    }
    switch (o_.kind) {
      case EngineOptions::Kind::LazyGreedy: return lazy();
      case EngineOptions::Kind::EagerGreedy:
      case EngineOptions::Kind::WeightedAStar: return eager();
      case EngineOptions::Kind::EnforcedHillClimbing: return ehc();
    }
    return Outcome::Failed;
  }

private:
  heuristics::HeuristicResult evaluate(StateId s, double g)
  {
    const auto & v = ctx_.space.values(s);
    const auto & n = ctx_.space.numbers(s);
    heuristics::EvalContext ec{s, v.data(), n.data(), g};
    ++ctx_.stats.evaluated;
    auto r = h_->evaluate(ec);
    if (r.dead_end) {
      ++ctx_.stats.dead_ends;
    }
    return r;
  }

  bool out_of_budget() {return ctx_.budget && ctx_.budget->exhausted();}

  bool pruned(double g) const {return ctx_.prune_with_bound && g >= ctx_.sink.bound();}

  Outcome found(StateId goal)
  {
    // The cost of the path itself: after a reopening, parents may lead there more cheaply than
    // the g stored in the goal node
    auto plan = space_.path(goal);
    double g = 0.0;
    for (uint32_t op : plan) {
      g += ctx_.costs[op];
    }
    if (ctx_.sink.offer(plan, g)) {
      ctx_.logger.info(
        LogChannel::Search, "plan of cost {} with {} actions ({} expanded, {} evaluated)", g,
        plan.size(), ctx_.stats.expanded, ctx_.stats.evaluated);
    }
    return Outcome::Solved;
  }

  void progress(double h)
  {
    if (h < best_h_) {
      best_h_ = h;
      open_->boost(o_.boost);
      ctx_.logger.debug(
        LogChannel::Search, "new best h {} ({} expanded, {} evaluated)", h, ctx_.stats.expanded,
        ctx_.stats.evaluated);
    }
  }

  std::pair<double, double> keys(double g, double h) const
  {
    if (o_.kind == EngineOptions::Kind::WeightedAStar) {
      return {g + o_.weight * h, h};
    }
    return {h, 0.0};
  }

  // Successors of s that are valid steps, as (state, action)
  void successors(StateId s, std::vector<std::pair<StateId, uint32_t>> & out)
  {
    out.clear();
    ops_.clear();
    ctx_.space.applicable(s, ops_);
    for (uint32_t op : ops_) {
      auto next = ctx_.space.successor(s, op);
      ++ctx_.stats.generated;
      if (next) {
        out.emplace_back(*next, op);
      }
    }
  }

  Outcome eager()
  {
    open_ = std::make_unique<Alternation>(o_.preferred);
    StateId s0 = ctx_.registry.initial();
    if (pruned(0.0)) {
      return Outcome::Unsolvable;  // nothing is cheaper than a plan of cost 0
    }
    auto r0 = evaluate(s0, 0.0);
    if (r0.dead_end) {
      return Outcome::Unsolvable;
    }
    Node & n0 = space_[s0];
    n0 = Node{kNoState, kNoAction, 0.0, r0.value, kOpen};
    best_h_ = r0.value;
    if (o_.preferred) {
      preferred_[s0] = std::move(r0.preferred);
    }
    auto [k1, k2] = keys(0.0, r0.value);
    open_->push(Entry{k1, k2, order_++, s0, kNoAction, 0.0}, false);
    std::vector<std::pair<StateId, uint32_t>> succ;
    while (!open_->empty()) {
      if (out_of_budget()) {
        return Outcome::Stopped;
      }
      Entry e = open_->pop();
      Node & n = space_[e.state];
      if (n.status != kOpen || e.g != n.g) {
        continue;  // stale
      }
      if (ctx_.space.is_goal(e.state)) {
        return found(e.state);
      }
      n.status = kClosed;
      ++ctx_.stats.expanded;
      std::unordered_set<uint32_t> pref;
      if (auto it = preferred_.find(e.state); it != preferred_.end()) {
        pref.insert(it->second.begin(), it->second.end());
        preferred_.erase(it);
      }
      double g = n.g;
      successors(e.state, succ);
      for (const auto & [t, op] : succ) {
        double g2 = g + ctx_.costs[op];
        if (pruned(g2)) {
          continue;
        }
        Node & m = space_[t];
        if (m.status == kDead) {
          continue;
        }
        if (m.status == kNew) {
          auto r = evaluate(t, g2);
          if (r.dead_end) {
            m.status = kDead;
            continue;
          }
          m = Node{e.state, op, g2, r.value, kOpen};
          if (o_.preferred && !r.preferred.empty()) {
            preferred_[t] = std::move(r.preferred);
          }
          progress(r.value);
        } else if (g2 < m.g) {
          if (m.status == kClosed) {
            if (!o_.reopen) {
              continue;
            }
            ++ctx_.stats.reopened;
          }
          m.parent = e.state;
          m.action = op;
          m.g = g2;
          m.status = kOpen;
        } else {
          continue;
        }
        auto [a, b] = keys(m.g, m.h);
        open_->push(Entry{a, b, order_++, t, kNoAction, m.g}, pref.count(op) > 0);
      }
    }
    return Outcome::Unsolvable;
  }

  Outcome lazy()
  {
    open_ = std::make_unique<Alternation>(o_.preferred);
    StateId current = ctx_.registry.initial();
    if (pruned(0.0)) {
      return Outcome::Unsolvable;
    }
    auto r = evaluate(current, 0.0);
    if (r.dead_end) {
      return Outcome::Unsolvable;
    }
    space_[current] = Node{kNoState, kNoAction, 0.0, r.value, kClosed};
    best_h_ = r.value;
    std::vector<uint32_t> preferred = std::move(r.preferred);
    while (true) {
      if (ctx_.space.is_goal(current)) {
        return found(current);
      }
      // Expand: one entry per operator, keyed by the parent's h
      ++ctx_.stats.expanded;
      ops_.clear();
      ctx_.space.applicable(current, ops_);
      std::unordered_set<uint32_t> pref(preferred.begin(), preferred.end());
      double h = space_[current].h;
      for (uint32_t op : ops_) {
        open_->push(Entry{h, 0.0, order_++, current, op, 0.0}, pref.count(op) > 0);
      }
      // Next: the first entry whose successor is new and not a dead end
      while (true) {
        if (open_->empty()) {
          return Outcome::Unsolvable;
        }
        if (out_of_budget()) {
          return Outcome::Stopped;
        }
        Entry e = open_->pop();
        auto next = ctx_.space.successor(e.state, e.action);
        ++ctx_.stats.generated;
        if (!next) {
          continue;
        }
        double g2 = space_[e.state].g + ctx_.costs[e.action];
        if (pruned(g2)) {
          continue;
        }
        Node & m = space_[*next];
        if (m.status != kNew) {
          continue;
        }
        auto rr = evaluate(*next, g2);
        if (rr.dead_end) {
          m.status = kDead;
          continue;
        }
        m = Node{e.state, e.action, g2, rr.value, kClosed};
        progress(rr.value);
        current = *next;
        preferred = std::move(rr.preferred);
        break;
      }
    }
  }

  // Breadth-first from the current state until a strictly better one; helpful actions only
  // when asked (as FF)
  Outcome ehc()
  {
    open_ = std::make_unique<Alternation>(false);
    StateId s = ctx_.registry.initial();
    auto r = evaluate(s, 0.0);
    if (r.dead_end) {
      return Outcome::Unsolvable;
    }
    space_[s] = Node{kNoState, kNoAction, 0.0, r.value, kClosed};
    double h = r.value;
    std::vector<uint32_t> helpful = std::move(r.preferred);
    while (true) {
      if (ctx_.space.is_goal(s)) {
        return found(s);
      }
      std::deque<StateId> queue{s};
      std::unordered_map<StateId, std::vector<uint32_t>> help{{s, helpful}};
      std::unordered_set<StateId> seen{s};
      bool better = false;
      while (!queue.empty() && !better) {
        if (out_of_budget()) {
          return Outcome::Stopped;
        }
        StateId t = queue.front();
        queue.pop_front();
        ++ctx_.stats.expanded;
        ops_.clear();
        if (o_.preferred) {
          ops_ = help[t];
        } else {
          ctx_.space.applicable(t, ops_);
        }
        help.erase(t);
        std::vector<uint32_t> ops = ops_;
        for (uint32_t op : ops) {
          auto next = ctx_.space.successor(t, op);
          ++ctx_.stats.generated;
          if (!next || !seen.insert(*next).second) {
            continue;
          }
          double g2 = space_[t].g + ctx_.costs[op];
          auto rr = evaluate(*next, g2);
          if (rr.dead_end) {
            continue;
          }
          Node & m = space_[*next];
          if (m.status == kNew || g2 < m.g) {
            m = Node{t, op, g2, rr.value, kClosed};
          }
          if (rr.value < h || ctx_.space.is_goal(*next)) {
            s = *next;
            h = rr.value;
            helpful = std::move(rr.preferred);
            better = true;
            break;
          }
          help[*next] = std::move(rr.preferred);
          queue.push_back(*next);
        }
      }
      if (!better) {
        return Outcome::Failed;  // a plateau with no way out in this neighbourhood
      }
    }
  }

  SearchContext & ctx_;
  EngineOptions o_;
  SearchSpace space_;
  std::unique_ptr<heuristics::Heuristic> h_;
  std::unique_ptr<Alternation> open_;
  std::unordered_map<StateId, std::vector<uint32_t>> preferred_;
  std::vector<uint32_t> ops_;
  uint64_t order_ = 0;
  double best_h_ = heuristics::kInfinity;
};

}  // namespace

Outcome run_engine(SearchContext & ctx, const EngineOptions & options)
{
  Engine e(ctx, options);
  return e.run();
}

AnytimeResult anytime(SearchContext & ctx, const AnytimeOptions & options)
{
  Outcome o = run_engine(ctx, options.first);
  if (o == Outcome::Failed && options.first.kind == EngineOptions::Kind::EnforcedHillClimbing &&
    options.fallback_after_ehc)
  {
    EngineOptions lazy;
    lazy.heuristic = options.first.heuristic;
    o = run_engine(ctx, lazy);
  }
  switch (o) {
    case Outcome::Stopped: return AnytimeResult::Stopped;
    case Outcome::Unsolvable: return AnytimeResult::Exhausted;
    case Outcome::Failed: return AnytimeResult::Failed;
    case Outcome::Solved: break;
  }
  if (!options.improve || options.weights.empty()) {
    return AnytimeResult::NoMoreImprovement;
  }
  // Restarts, each needing a strictly cheaper plan; with pruning the last weight repeats
  // until a search exhausts the space below the bound
  for (size_t i = 0;; ++i) {
    if (!ctx.prune_with_bound && i >= options.weights.size()) {
      return AnytimeResult::NoMoreImprovement;
    }
    EngineOptions w;
    w.kind = EngineOptions::Kind::WeightedAStar;
    w.weight = options.weights[std::min(i, options.weights.size() - 1)];
    w.heuristic = options.improve_heuristic;
    w.reopen = true;
    o = run_engine(ctx, w);
    if (o == Outcome::Stopped) {
      return AnytimeResult::Stopped;
    }
    if (o == Outcome::Unsolvable) {
      return ctx.prune_with_bound ? AnytimeResult::ProvenOptimal :
             AnytimeResult::NoMoreImprovement;
    }
  }
}

Plan to_plan(const sas::Task & task, const std::vector<uint32_t> & actions)
{
  Plan plan;
  plan.temporal = false;
  for (size_t i = 0; i < actions.size(); ++i) {
    const auto & a = task.actions.at(actions[i]);
    PlanStep step;
    step.time = static_cast<double>(i);
    step.name = task.action_names.at(a.schema);
    for (auto o : a.args) {
      step.args.push_back(task.object_names.at(o.index()));
    }
    plan.steps.push_back(std::move(step));
  }
  return plan;
}

}  // namespace kairos::search
