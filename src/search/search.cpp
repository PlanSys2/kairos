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
#include <array>
#include <cmath>
#include <deque>
#include <map>
#include <memory>
#include <queue>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "heuristics/heuristic.hpp"
#include "search/novelty.hpp"

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

// What an open list holds: a state and, in lazy search, the operator to apply to it (8 bytes;
// lazy search pushes one per operator of every expanded state)
struct Entry
{
  StateId state;
  uint32_t action;
};

using Keys = std::vector<std::pair<double, double>>;  // per group

// Entries by (key1, key2), first in first out within a key: the order of a heap on (key1,
// key2, arrival) at 8 bytes per entry
class KeyedQueue
{
public:
  void push(std::pair<double, double> key, Entry e)
  {
    buckets_[key].push_back(e);
    ++size_;
  }
  bool empty() const {return size_ == 0;}
  size_t size() const {return size_;}
  Entry pop()
  {
    auto it = buckets_.begin();
    Entry e = it->second.front();
    it->second.pop_front();
    if (it->second.empty()) {
      buckets_.erase(it);
    }
    --size_;
    return e;
  }

private:
  std::map<std::pair<double, double>, std::deque<Entry>> buckets_;
  size_t size_ = 0;
};

// One queue per group (a heuristic, or novelty) with every entry, plus one with the preferred
// entries of each heuristic group; the queue with the lowest priority is popped next, and
// progress boosts the preferred ones (as FD's alternation open list) and the novelty one
class Alternation
{
public:
  Alternation(size_t heuristics, bool preferred, bool novelty)
  {
    for (size_t g = 0; g < heuristics; ++g) {
      specs_.push_back({g, false, false});
      if (preferred) {
        specs_.push_back({g, true, true});
      }
    }
    if (novelty) {
      specs_.push_back({heuristics, false, true});
    }
    queues_.resize(specs_.size());
    priority_.assign(specs_.size(), 0);
  }

  ~Alternation()
  {
    if (budget_ && accounted_) {
      budget_->account(-static_cast<std::ptrdiff_t>(accounted_));
    }
  }
  Alternation(const Alternation &) = delete;
  Alternation & operator=(const Alternation &) = delete;

  void set_budget(Budget * budget) {budget_ = budget;}

  void push(const Entry & e, const Keys & keys, bool preferred)
  {
    for (size_t q = 0; q < specs_.size(); ++q) {
      if (specs_[q].preferred_only && !preferred) {
        continue;
      }
      queues_[q].push(keys[specs_[q].group], e);
      ++entries_;
    }
    account();
  }

  bool empty() const
  {
    return std::all_of(
      queues_.begin(), queues_.end(), [](const KeyedQueue & h) {return h.empty();});
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
    --entries_;
    return queues_[best].pop();
  }

  void boost(int amount)
  {
    for (size_t q = 0; q < specs_.size(); ++q) {
      if (specs_[q].boosted) {
        priority_[q] -= amount;
      }
    }
  }

private:
  struct Spec
  {
    size_t group;
    bool preferred_only;
    bool boosted;  // progress boosts it: the preferred queues and novelty
  };
  // The entries count against the memory budget, in steps of 1 MB
  void account()
  {
    size_t bytes = entries_ * sizeof(Entry);
    if (budget_ && (bytes > accounted_ + (size_t{1} << 20) || bytes + (size_t{1} << 20) <
      accounted_))
    {
      budget_->account(static_cast<std::ptrdiff_t>(bytes) -
        static_cast<std::ptrdiff_t>(accounted_));
      accounted_ = bytes;
    }
  }

  std::vector<Spec> specs_;
  std::vector<KeyedQueue> queues_;
  std::vector<int64_t> priority_;
  Budget * budget_ = nullptr;
  size_t entries_ = 0;
  size_t accounted_ = 0;
};

// --- Engines ---------------------------------------------------------------------------------

// What the heuristics say about a state
struct Evaluation
{
  std::vector<double> h;              // per heuristic
  bool dead_end = false;
  std::vector<uint32_t> preferred;    // union, sorted
  int novelty = 0;
};

std::vector<std::string> split(const std::string & names)
{
  std::vector<std::string> out;
  std::string cur;
  for (char c : names) {
    if (c == ',') {
      out.push_back(cur);
      cur.clear();
    } else if (c != ' ') {
      cur += c;
    }
  }
  out.push_back(cur);
  return out;
}

class Engine
{
public:
  Engine(SearchContext & ctx, const EngineOptions & o)
  : ctx_(ctx), o_(o), space_(ctx.budget)
  {
    for (const auto & name : split(o.heuristic)) {
      auto h = heuristics::make_heuristic(name, ctx.task, ctx.costs, ctx.budget);
      if (!h) {
        hs_.clear();
        return;
      }
      path_dependent_ = path_dependent_ || h->path_dependent();
      hs_.push_back(std::move(h));
    }
    extra_h_.resize(hs_.empty() ? 0 : hs_.size() - 1);
    best_h_.assign(hs_.size(), heuristics::kInfinity);
    bool eager = o.kind == EngineOptions::Kind::EagerGreedy ||
      o.kind == EngineOptions::Kind::WeightedAStar;
    if (o.novelty > 0 && eager) {
      goal_count_ = heuristics::make_heuristic("goalcount", ctx.task, ctx.costs);
      for (size_t i = 0; i < hs_.size(); ++i) {
        if (hs_[i]->name() == "lmcount") {
          lm_index_ = static_cast<int>(i);
        }
      }
      novelty_ = std::make_unique<Novelty>(
        ctx.task.variables.size(), ctx.task.initial_numeric.size(), o.novelty, o.novelty_memory,
        ctx.budget);
    }
  }

  Outcome run()
  {
    if (hs_.empty()) {
      return Outcome::Failed;
    }
    ctx_.logger.debug(
      LogChannel::Search, "engine {}: heuristics {}{}", static_cast<int>(o_.kind), o_.heuristic,
      novelty_ ? ", novelty " + std::to_string(novelty_->max_width()) : std::string());
    switch (o_.kind) {
      case EngineOptions::Kind::LazyGreedy: return lazy();
      case EngineOptions::Kind::EagerGreedy:
      case EngineOptions::Kind::WeightedAStar: return eager();
      case EngineOptions::Kind::EnforcedHillClimbing: return ehc();
    }
    return Outcome::Failed;
  }

private:
  heuristics::EvalContext context(StateId s, double g)
  {
    const auto & v = ctx_.space.values(s);
    const auto & n = ctx_.space.numbers(s);
    return heuristics::EvalContext{s, v.data(), n.data(), g};
  }

  void initial(const heuristics::EvalContext & ec)
  {
    for (auto & h : hs_) {
      h->initial(ec);
    }
  }

  void transition(StateId parent, uint32_t op, const heuristics::EvalContext & child)
  {
    if (!path_dependent_) {
      return;
    }
    for (auto & h : hs_) {
      if (h->path_dependent()) {
        h->transition(parent, op, child);
      }
    }
  }

  Evaluation evaluate(const heuristics::EvalContext & ec)
  {
    ++ctx_.stats.evaluated;
    Evaluation r;
    r.h.reserve(hs_.size());
    for (size_t i = 0; i < hs_.size(); ++i) {
      auto hr = hs_[i]->evaluate(ec);
      r.h.push_back(hr.value);
      if (hr.dead_end) {
        r.dead_end = true;
        break;  // sound: no plan from here
      }
      if (o_.preferred) {
        r.preferred.insert(r.preferred.end(), hr.preferred.begin(), hr.preferred.end());
      }
    }
    if (r.dead_end) {
      ++ctx_.stats.dead_ends;
      return r;
    }
    if (hs_.size() > 1 && !r.preferred.empty()) {
      std::sort(r.preferred.begin(), r.preferred.end());
      r.preferred.erase(std::unique(r.preferred.begin(), r.preferred.end()), r.preferred.end());
    }
    if (novelty_) {
      // Coarse partitions, as BFWS: goals not achieved, and landmarks still to reach
      auto goals = goal_count_->evaluate(ec).value;
      double lm = lm_index_ >= 0 ? r.h[static_cast<size_t>(lm_index_)] : 0.0;
      uint64_t partition = static_cast<uint64_t>(std::llround(goals)) * 1000003u +
        static_cast<uint64_t>(std::llround(lm * 1000.0));
      r.novelty = novelty_->evaluate(
        partition, ec.values, ec.numbers,
        parent_values_.empty() ? nullptr : parent_values_.data(),
        parent_values_.empty() ? nullptr : parent_numbers_.data());
      ++widths_[static_cast<size_t>(r.novelty)];
    }
    return r;
  }

  // The h values of a state: the first in its node, the others aside
  void store(StateId s, const Evaluation & r)
  {
    for (size_t i = 1; i < r.h.size(); ++i) {
      auto & v = extra_h_[i - 1];
      if (v.size() <= s) {
        v.resize(std::max<size_t>(s + 1, v.size() * 2), 0.0);
      }
      v[s] = r.h[i];
    }
  }

  double h_of(StateId s, size_t i)
  {
    return i == 0 ? space_[s].h : extra_h_[i - 1][s];
  }

  bool out_of_budget() {return ctx_.budget && ctx_.budget->exhausted();}

  double step_cost(StateId parent, uint32_t op, StateId child, double g) const
  {
    return ctx_.path_cost ? ctx_.path_cost(parent, op, child, g) : g + ctx_.costs[op];
  }

  bool pruned(double g) const {return ctx_.prune_with_bound && g >= ctx_.sink.bound();}

  Outcome found(StateId goal)
  {
    // The cost of the path itself: after a reopening, parents may lead there more cheaply than
    // the g stored in the goal node
    auto plan = space_.path(goal);
    double g = 0.0;
    if (ctx_.path_cost) {
      g = space_[goal].g;  // depends on the path: the one the node keeps
    } else {
      for (uint32_t op : plan) {
        g += ctx_.costs[op];
      }
    }
    if (ctx_.sink.offer(plan, g)) {
      ctx_.logger.info(
        LogChannel::Search, "plan of cost {} with {} actions ({} expanded, {} evaluated)", g,
        plan.size(), ctx_.stats.expanded, ctx_.stats.evaluated);
    }
    return Outcome::Solved;
  }

  void progress(const std::vector<double> & h)
  {
    bool better = false;
    for (size_t i = 0; i < h.size(); ++i) {
      if (h[i] < best_h_[i]) {
        best_h_[i] = h[i];
        better = true;
      }
    }
    if (better) {
      open_->boost(o_.boost);
      ctx_.logger.debug(
        LogChannel::Search, "new best h {} ({} expanded, {} evaluated)", h[0],
        ctx_.stats.expanded, ctx_.stats.evaluated);
    }
  }

  // Keys of each group: the heuristics, then novelty
  Keys keys(double g, const std::vector<double> & h, int novelty) const
  {
    Keys k;
    for (double v : h) {
      if (o_.kind == EngineOptions::Kind::WeightedAStar) {
        k.emplace_back(g + o_.weight * v, v);
      } else {
        k.emplace_back(v, 0.0);
      }
    }
    if (novelty_) {
      k.emplace_back(static_cast<double>(novelty), h[0]);
    }
    return k;
  }

  // Successors of s that are valid steps, as (state, action)
  void successors(StateId s, std::vector<std::pair<StateId, uint32_t>> & out)
  {
    out.clear();
    ops_.clear();
    ctx_.space.applicable(s, ops_);
    for (uint32_t op : ops_) {
      // A wide state (thousands of successors, each filtered) can take long: the budget is
      // checked within it too
      if (out_of_budget()) {
        return;
      }
      auto next = ctx_.space.successor(s, op);
      ++ctx_.stats.generated;
      if (next && (!ctx_.accept || ctx_.accept(s, op, *next))) {
        out.emplace_back(*next, op);
      }
    }
  }

  Outcome eager()
  {
    auto o = eager_search();
    if (novelty_) {
      ctx_.logger.debug(
        LogChannel::Search, "novelty: {} states of width 1, {} of 2, {} more", widths_[1],
        widths_[2], widths_[3]);
    }
    return o;
  }

  Outcome eager_search()
  {
    open_ = std::make_unique<Alternation>(hs_.size(), o_.preferred, novelty_ != nullptr);
    open_->set_budget(ctx_.budget);
    StateId s0 = ctx_.registry.initial();
    if (pruned(0.0)) {
      return Outcome::Unsolvable;  // nothing is cheaper than a plan of cost 0
    }
    auto c0 = context(s0, 0.0);
    initial(c0);
    auto r0 = evaluate(c0);
    if (r0.dead_end) {
      return Outcome::Unsolvable;
    }
    Node & n0 = space_[s0];
    n0 = Node{kNoState, kNoAction, 0.0, r0.h[0], kOpen};
    store(s0, r0);
    best_h_ = r0.h;
    if (o_.preferred) {
      preferred_[s0] = std::move(r0.preferred);
    }
    open_->push(Entry{s0, kNoAction}, keys(0.0, r0.h, r0.novelty), false);
    std::vector<std::pair<StateId, uint32_t>> succ;
    std::vector<double> hs(hs_.size());
    while (!open_->empty()) {
      if (out_of_budget()) {
        return Outcome::Stopped;
      }
      Entry e = open_->pop();
      Node & n = space_[e.state];
      if (n.status != kOpen) {
        continue;  // stale: expanded through a better entry (pushed later, popped first)
      }
      if (ctx_.space.is_goal(e.state)) {
        return found(e.state);
      }
      n.status = kClosed;
      ++ctx_.stats.expanded;
      std::vector<uint32_t> pref;
      if (auto it = preferred_.find(e.state); it != preferred_.end()) {
        pref = std::move(it->second);
        preferred_.erase(it);
      }
      double g = n.g;
      if (novelty_) {
        parent_values_ = ctx_.space.values(e.state);
        parent_numbers_ = ctx_.space.numbers(e.state);
      }
      successors(e.state, succ);
      if (out_of_budget()) {
        return Outcome::Stopped;
      }
      for (const auto & [t, op] : succ) {
        if (out_of_budget()) {
          return Outcome::Stopped;
        }
        double g2 = step_cost(e.state, op, t, g);
        if (pruned(g2)) {
          continue;
        }
        Node & m = space_[t];
        if (m.status == kDead) {
          continue;
        }
        int novelty = 3;
        if (m.status == kNew || path_dependent_) {
          auto ct = context(t, g2);
          transition(e.state, op, ct);
          if (m.status == kNew) {
            auto r = evaluate(ct);
            Node & mm = space_[t];
            if (r.dead_end) {
              mm.status = kDead;
              continue;
            }
            mm = Node{e.state, op, g2, r.h[0], kOpen};
            store(t, r);
            if (o_.preferred && !r.preferred.empty()) {
              preferred_[t] = std::move(r.preferred);
            }
            progress(r.h);
            novelty = r.novelty;
            for (size_t i = 0; i < hs_.size(); ++i) {
              hs[i] = r.h[i];
            }
            bool p = std::find(pref.begin(), pref.end(), op) != pref.end();
            open_->push(Entry{t, kNoAction}, keys(g2, hs, novelty), p);
            continue;
          }
        }
        Node & mm = space_[t];
        if (g2 < mm.g) {
          if (mm.status == kClosed) {
            if (!o_.reopen) {
              continue;
            }
            ++ctx_.stats.reopened;
          }
          mm.parent = e.state;
          mm.action = op;
          mm.g = g2;
          mm.status = kOpen;
        } else {
          continue;
        }
        for (size_t i = 0; i < hs_.size(); ++i) {
          hs[i] = h_of(t, i);
        }
        bool p = std::find(pref.begin(), pref.end(), op) != pref.end();
        open_->push(Entry{t, kNoAction}, keys(mm.g, hs, 3), p);
      }
    }
    return Outcome::Unsolvable;
  }

  Outcome lazy()
  {
    open_ = std::make_unique<Alternation>(hs_.size(), o_.preferred, false);
    open_->set_budget(ctx_.budget);
    StateId current = ctx_.registry.initial();
    if (pruned(0.0)) {
      return Outcome::Unsolvable;
    }
    auto c0 = context(current, 0.0);
    initial(c0);
    auto r = evaluate(c0);
    if (r.dead_end) {
      return Outcome::Unsolvable;
    }
    space_[current] = Node{kNoState, kNoAction, 0.0, r.h[0], kClosed};
    best_h_ = r.h;
    std::vector<double> current_h = r.h;
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
      Keys k = keys(0.0, current_h, 0);
      for (uint32_t op : ops_) {
        open_->push(Entry{current, op}, k, pref.count(op) > 0);
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
        if (!next || (ctx_.accept && !ctx_.accept(e.state, e.action, *next))) {
          continue;
        }
        double g2 = step_cost(e.state, e.action, *next, space_[e.state].g);
        if (pruned(g2)) {
          continue;
        }
        if (space_[*next].status != kNew && !path_dependent_) {
          continue;
        }
        auto cn = context(*next, g2);
        transition(e.state, e.action, cn);
        Node & m = space_[*next];
        if (m.status != kNew) {
          continue;
        }
        auto rr = evaluate(cn);
        if (rr.dead_end) {
          space_[*next].status = kDead;
          continue;
        }
        space_[*next] = Node{e.state, e.action, g2, rr.h[0], kClosed};
        progress(rr.h);
        current = *next;
        current_h = rr.h;
        preferred = std::move(rr.preferred);
        break;
      }
    }
  }

  // Breadth-first from the current state until a strictly better one (by the first
  // heuristic); helpful actions only when asked (as FF)
  Outcome ehc()
  {
    open_ = std::make_unique<Alternation>(1, false, false);
    StateId s = ctx_.registry.initial();
    auto c0 = context(s, 0.0);
    initial(c0);
    auto r = evaluate(c0);
    if (r.dead_end) {
      return Outcome::Unsolvable;
    }
    space_[s] = Node{kNoState, kNoAction, 0.0, r.h[0], kClosed};
    double h = r.h[0];
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
          if (!next || (ctx_.accept && !ctx_.accept(t, op, *next)) ||
            !seen.insert(*next).second)
          {
            continue;
          }
          double g2 = step_cost(t, op, *next, space_[t].g);
          auto cn = context(*next, g2);
          transition(t, op, cn);
          auto rr = evaluate(cn);
          if (rr.dead_end) {
            continue;
          }
          Node & m = space_[*next];
          if (m.status == kNew || g2 < m.g) {
            m = Node{t, op, g2, rr.h[0], kClosed};
          }
          if (rr.h[0] < h || ctx_.space.is_goal(*next)) {
            s = *next;
            h = rr.h[0];
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
  std::vector<std::unique_ptr<heuristics::Heuristic>> hs_;
  bool path_dependent_ = false;
  std::unique_ptr<Novelty> novelty_;
  std::unique_ptr<heuristics::Heuristic> goal_count_;  // novelty partitions
  int lm_index_ = -1;
  std::array<size_t, 4> widths_{};
  std::vector<uint32_t> parent_values_;  // novelty: the state being expanded
  std::vector<double> parent_numbers_;
  std::vector<std::vector<double>> extra_h_;  // heuristics after the first, per state
  std::unique_ptr<Alternation> open_;
  std::unordered_map<StateId, std::vector<uint32_t>> preferred_;
  std::vector<uint32_t> ops_;
  std::vector<double> best_h_;
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
