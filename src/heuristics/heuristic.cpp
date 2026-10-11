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

#include "heuristics/heuristic.hpp"

#include <algorithm>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "heuristics/relaxed.hpp"
#include "state/eval.hpp"

namespace kairos::heuristics
{

namespace
{

// Change of the metric by each action, if every one is a constant that is not negative
std::optional<std::vector<double>> metric_deltas(const sas::Task & task)
{
  const auto & m = task.metric;
  if (m.is_default || !m.minimize || !m.expr.linear) {
    return std::nullopt;
  }
  std::map<sas::NumVarId, double> weight;
  for (const auto & [v, c] : m.expr.linear->terms) {
    weight[v] = c;
  }
  if (weight.empty()) {
    return std::nullopt;
  }
  std::vector<double> out;
  for (const auto & a : task.actions) {
    double delta = 0.0;
    for (const auto & e : a.effects) {
      if (e.kind != sas::Effect::Kind::Numeric || !weight.count(e.num)) {
        continue;
      }
      bool constant = e.value.linear && e.value.linear->terms.empty() && e.condition.empty() &&
        !e.continuous;
      if (!constant || (e.op != ast::AssignOp::Increase && e.op != ast::AssignOp::Decrease)) {
        return std::nullopt;
      }
      double d = e.value.linear->constant;
      delta += weight[e.num] * (e.op == ast::AssignOp::Increase ? d : -d);
    }
    if (delta < 0.0) {
      return std::nullopt;
    }
    out.push_back(delta);
  }
  return out;
}

bool goal_holds(const sas::Task & t, const EvalContext & ctx)
{
  return !t.goal_unreachable && state::holds(t.goal, t, ctx.values, ctx.numbers);
}

class Blind : public Heuristic
{
public:
  Blind(const sas::Task & t, const std::vector<double> & costs)
  : t_(t)
  {
    for (double c : costs) {
      cheapest_ = std::min(cheapest_, c);
    }
  }
  HeuristicResult evaluate(const EvalContext & ctx) override
  {
    HeuristicResult r;
    if (goal_holds(t_, ctx)) {
      return r;
    }
    r.dead_end = t_.goal_unreachable || t_.actions.empty();
    r.value = r.dead_end ? kInfinity : cheapest_;
    return r;
  }
  bool admissible() const override {return true;}
  std::string name() const override {return "blind";}

private:
  const sas::Task & t_;
  double cheapest_ = kInfinity;
};

// Goal facts and conditions that do not hold
class GoalCount : public Heuristic
{
public:
  explicit GoalCount(const sas::Task & t)
  : t_(t) {}
  HeuristicResult evaluate(const EvalContext & ctx) override
  {
    HeuristicResult r;
    if (t_.goal_unreachable) {
      r.dead_end = true;
      r.value = kInfinity;
      return r;
    }
    const auto & g = t_.goal;
    for (const auto & f : g.facts) {
      r.value += ctx.values[f.var] != f.value ? 1.0 : 0.0;
    }
    for (const auto & f : g.not_facts) {
      r.value += ctx.values[f.var] == f.value ? 1.0 : 0.0;
    }
    for (const auto & n : g.numeric) {
      sas::Condition one;
      one.numeric.push_back(n);
      r.value += state::holds(one, t_, ctx.values, ctx.numbers) ? 0.0 : 1.0;
    }
    for (const auto & v : g.vars) {
      sas::Condition one;
      one.vars.push_back(v);
      r.value += state::holds(one, t_, ctx.values, ctx.numbers) ? 0.0 : 1.0;
    }
    return r;
  }
  std::string name() const override {return "goalcount";}

private:
  const sas::Task & t_;
};

class Relaxed : public Heuristic
{
public:
  enum class Kind : uint8_t {Max, Add, FF};

  Relaxed(const sas::Task & t, const std::vector<double> & costs, Kind kind)
  : t_(t), graph_(t, costs), kind_(kind) {}

  HeuristicResult evaluate(const EvalContext & ctx) override
  {
    HeuristicResult r;
    if (t_.goal_unreachable) {
      r.dead_end = true;
      r.value = kInfinity;
      return r;
    }
    double h = graph_.run(
      ctx, kind_ == Kind::Max ? RelaxedGraph::Mode::Max : RelaxedGraph::Mode::Add);
    if (h == kInfinity) {
      r.dead_end = true;
      r.value = kInfinity;
      return r;
    }
    r.value = kind_ == Kind::FF ? graph_.relaxed_plan(&r.preferred) : h;
    return r;
  }
  bool admissible() const override {return kind_ == Kind::Max && graph_.simple_numeric();}
  std::string name() const override
  {
    return kind_ == Kind::Max ? "hmax" : (kind_ == Kind::Add ? "hadd" : "ff");
  }

private:
  const sas::Task & t_;
  RelaxedGraph graph_;
  Kind kind_;
};

}  // namespace

std::vector<double> action_costs(const sas::Task & task, CostModel model)
{
  if (model == CostModel::Metric) {
    if (auto d = metric_deltas(task)) {
      return *d;
    }
  }
  return std::vector<double>(task.actions.size(), 1.0);
}

bool metric_costs_apply(const sas::Task & task)
{
  return metric_deltas(task).has_value();
}

std::unique_ptr<Heuristic> make_heuristic(
  const std::string & name, const sas::Task & task, const std::vector<double> & costs)
{
  if (name == "blind") {
    return std::make_unique<Blind>(task, costs);
  }
  if (name == "goalcount") {
    return std::make_unique<GoalCount>(task);
  }
  if (name == "hmax") {
    return std::make_unique<Relaxed>(task, costs, Relaxed::Kind::Max);
  }
  if (name == "hadd") {
    return std::make_unique<Relaxed>(task, costs, Relaxed::Kind::Add);
  }
  if (name == "ff") {
    return std::make_unique<Relaxed>(task, costs, Relaxed::Kind::FF);
  }
  return nullptr;
}

}  // namespace kairos::heuristics
