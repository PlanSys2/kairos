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

#include "search/goals.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <queue>
#include <unordered_map>
#include <utility>
#include <vector>

#include "heuristics/relaxed.hpp"
#include "state/eval.hpp"

namespace kairos::search
{

bool goals_apply(const sas::Task & task, size_t min_goals)
{
  const auto & g = task.goal;
  return !task.goal_unreachable && g.not_facts.empty() && g.numeric.empty() && g.vars.empty() &&
         g.facts.size() >= min_goals;
}

namespace
{

using state::StateId;

class Goals
{
public:
  Goals(SearchContext & ctx, const GoalsOptions & options)
  : ctx_(ctx), o_(options), costs_(plus_one(ctx.costs)), graph_(ctx.task, costs_),
    resources_(numeric_conditions(ctx.task)) {}

  Outcome run()
  {
    const auto & goal = ctx_.task.goal.facts;
    StateId s = ctx_.registry.initial();
    std::vector<uint32_t> plan;
    while (true) {
      if (stopped()) {
        return Outcome::Stopped;
      }
      std::vector<sas::Fact> keep;
      std::vector<sas::Fact> todo;
      {
        const auto & v = ctx_.space.values(s);
        for (const auto & f : goal) {
          (v[f.var] == f.value ? keep : todo).push_back(f);
        }
      }
      if (todo.empty()) {
        double cost = 0.0;
        for (uint32_t a : plan) {
          cost += ctx_.costs[a];
        }
        ctx_.sink.offer(std::move(plan), cost);
        return Outcome::Solved;
      }
      // The nearest goal first (the first the relaxed exploration reaches); if its search
      // fails, the nearest of the others
      bool advanced = false;
      for (size_t k = 0; k < o_.candidates && !todo.empty(); ++k) {
        graph_.set_goal(todo, true);
        auto ec = context(s);
        if (std::isinf(graph_.run(ec, heuristics::RelaxedGraph::Mode::Add))) {
          ctx_.logger.debug(
            LogChannel::Search, "goals: {} goals left, none reachable", todo.size());
          return Outcome::Failed;
        }
        auto f = graph_.fact_of(graph_.goal_fact());
        if (!f) {
          return Outcome::Failed;
        }
        sas::Fact next{f->first, f->second};
        std::vector<sas::Fact> sub = keep;
        sub.push_back(next);
        auto r = reach(s, sub, plan);
        if (r == Reach::Stopped) {
          return Outcome::Stopped;
        }
        if (r == Reach::Reached) {
          ctx_.logger.debug(
            LogChannel::Search, "goals: {} of {} ({} steps, {} expanded)", keep.size() + 1,
            goal.size(), plan.size(), ctx_.stats.expanded);
          advanced = true;
          break;
        }
        todo.erase(std::remove(todo.begin(), todo.end(), next), todo.end());
      }
      if (!advanced) {
        ctx_.logger.debug(LogChannel::Search, "goals: no candidate reached, {} goals done",
          keep.size());
        return Outcome::Failed;
      }
    }
  }

private:
  enum class Reach : uint8_t {Reached, Failed, Stopped};

  static std::vector<double> plus_one(const std::vector<double> & costs)
  {
    // As LAMA: zero-cost actions would make every relaxed plan look as good
    std::vector<double> out(costs);
    for (double & c : out) {
      c += 1.0;
    }
    return out;
  }

  static bool numeric_conditions(const sas::Task & task)
  {
    return std::any_of(
      task.actions.begin(), task.actions.end(),
      [](const sas::Action & a) {return !a.pre.numeric.empty();});
  }

  bool stopped() {return ctx_.budget && ctx_.budget->exhausted();}

  // Whether every goal is still reachable in the relaxation: reaching one goal must not spend
  // what the others need (energy, fuel); then the graph's goal is sub again
  bool alive(StateId s, const std::vector<sas::Fact> & sub)
  {
    graph_.set_goal(ctx_.task.goal.facts);
    auto ec = context(s);
    bool ok = !std::isinf(graph_.run(ec, heuristics::RelaxedGraph::Mode::Add));
    graph_.set_goal(sub);
    return ok;
  }

  heuristics::EvalContext context(StateId s)
  {
    const auto & v = ctx_.space.values(s);
    const auto & n = ctx_.space.numbers(s);
    return heuristics::EvalContext{s, v.data(), n.data(), 0.0};
  }

  bool reached(StateId s, const std::vector<sas::Fact> & sub)
  {
    const auto & v = ctx_.space.values(s);
    return std::all_of(
      sub.begin(), sub.end(), [&](const sas::Fact & f) {return v[f.var] == f.value;});
  }

  // FF of a state for the current goal of the graph, and its preferred operators
  double evaluate(StateId s, std::vector<uint32_t> & preferred)
  {
    ++ctx_.stats.evaluated;
    preferred.clear();
    auto ec = context(s);
    if (std::isinf(graph_.run(ec, heuristics::RelaxedGraph::Mode::Add))) {
      ++ctx_.stats.dead_ends;
      return heuristics::kInfinity;
    }
    double h = graph_.relaxed_plan(&preferred);
    std::sort(preferred.begin(), preferred.end());
    return h;
  }

  // Lazy greedy search from s for the facts of sub (FF, preferred operators boosted on
  // progress, as FD); appends its steps to plan and moves s to where it ends
  Reach reach(StateId & s, const std::vector<sas::Fact> & sub, std::vector<uint32_t> & plan)
  {
    graph_.set_goal(sub);
    struct Item
    {
      double h;
      uint64_t seq;
      StateId parent;
      uint32_t op;
      bool operator>(const Item & o) const {return h != o.h ? h > o.h : seq > o.seq;}
    };
    using Queue = std::priority_queue<Item, std::vector<Item>, std::greater<Item>>;
    Queue all, pref;
    int64_t priority[2] = {0, 0};  // all, pref: the lower is popped next
    uint64_t seq = 0;
    std::unordered_map<StateId, std::pair<StateId, uint32_t>> parent;
    parent.emplace(s, std::make_pair(state::kNoState, UINT32_MAX));
    std::vector<uint32_t> preferred;
    StateId current = s;
    double h = evaluate(current, preferred);
    if (std::isinf(h)) {
      ctx_.logger.debug(LogChannel::Search, "goals: a dead end for the next goal");
      return Reach::Failed;
    }
    double best = h;
    std::vector<uint32_t> ops;
    size_t limit = o_.max_expansions;
    if (reached_ > 0) {
      limit = std::clamp(o_.per_goal * spent_ / reached_, o_.min_expansions, o_.max_expansions);
    }
    for (size_t expanded = 0; expanded < limit; ++expanded) {
      if (stopped()) {
        return Reach::Stopped;
      }
      if (reached(current, sub) && (!resources_ || alive(current, sub))) {
        std::vector<uint32_t> steps;
        for (StateId x = current; x != s; x = parent.at(x).first) {
          steps.push_back(parent.at(x).second);
        }
        plan.insert(plan.end(), steps.rbegin(), steps.rend());
        s = current;
        ++reached_;
        spent_ += expanded + 1;
        return Reach::Reached;
      }
      ++ctx_.stats.expanded;
      ops.clear();
      ctx_.space.applicable(current, ops);
      for (uint32_t op : ops) {
        all.push(Item{h, seq, current, op});
        if (std::binary_search(preferred.begin(), preferred.end(), op)) {
          pref.push(Item{h, seq, current, op});
        }
        ++seq;
      }
      // The next state: from the queue whose turn it is, the first new and valid successor
      bool next = false;
      while (!next && !(all.empty() && pref.empty())) {
        if (stopped()) {
          return Reach::Stopped;
        }
        int q = pref.empty() ? 0 : (all.empty() ? 1 : (priority[1] <= priority[0] ? 1 : 0));
        Queue & queue = q ? pref : all;
        Item it = queue.top();
        queue.pop();
        ++priority[q];
        auto t = ctx_.space.successor(it.parent, it.op);
        ++ctx_.stats.generated;
        if (!t || parent.count(*t) ||
          (ctx_.accept && !ctx_.accept(it.parent, it.op, *t)))
        {
          continue;
        }
        parent.emplace(*t, std::make_pair(it.parent, it.op));
        double ht = evaluate(*t, preferred);
        if (std::isinf(ht)) {
          continue;
        }
        current = *t;
        h = ht;
        next = true;
        if (h < best) {
          best = h;
          priority[1] -= 1000;
        }
      }
      if (!next) {
        ctx_.logger.debug(LogChannel::Search, "goals: search space exhausted for the next goal");
        return Reach::Failed;  // nothing new reachable from s
      }
    }
    ctx_.logger.debug(LogChannel::Search, "goals: expansion limit for the next goal");
    return Reach::Failed;
  }

  SearchContext & ctx_;
  GoalsOptions o_;
  std::vector<double> costs_;
  heuristics::RelaxedGraph graph_;
  bool resources_;  // numeric conditions: a goal reached may leave the others unreachable
  size_t reached_ = 0;  // goals reached by a short search
  size_t spent_ = 0;    // and the expansions they took
};

}  // namespace

Outcome incremental_goals(SearchContext & ctx, const GoalsOptions & options)
{
  if (!goals_apply(ctx.task, 1)) {
    return Outcome::Failed;
  }
  return Goals(ctx, options).run();
}

}  // namespace kairos::search
