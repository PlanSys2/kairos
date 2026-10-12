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

#include "temporal/durations.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "sas/translate.hpp"
#include "state/eval.hpp"

namespace kairos::temporal
{

namespace
{

using sas::ExprOp;

constexpr uint8_t kUp = 1, kDown = 2, kOther = 4;  // how a variable is read, as in profile

bool has(const ground::Expr & e, ExprOp op)
{
  return e.op == op ||
         std::any_of(e.children.begin(), e.children.end(), [&](const ground::Expr & c) {
             return has(c, op);
         });
}

// #t replaced by ?duration: the total change of a linear continuous effect
void t_to_duration(ground::Expr & e)
{
  if (e.op == ExprOp::ContinuousT) {
    e = ground::Expr{};
    e.op = ExprOp::Duration;
    return;
  }
  for (auto & c : e.children) {
    t_to_duration(c);
  }
}

void fluents(const ground::Expr & e, std::vector<sas::NumVarId> & out)
{
  if (e.op == ExprOp::Fluent) {
    out.push_back(static_cast<sas::NumVarId>(e.fluent));
  }
  for (const auto & c : e.children) {
    fluents(c, out);
  }
}

// How the conditions of the task read each numeric variable
class Readers
{
public:
  explicit Readers(const sas::Task & t)
  {
    for (const auto & a : t.actions) {
      condition(a.pre);
      for (const auto & e : a.effects) {
        effect(e);
      }
    }
    for (const auto & da : t.durative_actions) {
      condition(da.at_start);
      condition(da.over_all);
      condition(da.at_end);
      for (const auto & e : da.effects) {
        effect(e);
      }
    }
    for (const auto & ax : t.axioms) {
      condition(ax.body);
    }
  }

  uint8_t of(sas::NumVarId n) const
  {
    auto it = dirs_.find(n);
    return it == dirs_.end() ? 0 : it->second;
  }

private:
  void mark(const ground::Expr & e, uint8_t d)
  {
    std::vector<sas::NumVarId> vs;
    fluents(e, vs);
    for (auto v : vs) {
      dirs_[v] |= d;
    }
  }

  void condition(const sas::Condition & c)
  {
    for (const auto & n : c.numeric) {
      bool lower = n.op == sas::CompareOp::Ge || n.op == sas::CompareOp::Gt;
      bool upper = n.op == sas::CompareOp::Le || n.op == sas::CompareOp::Lt;
      if (n.difference && (lower || upper)) {
        for (const auto & [v, coef] : n.difference->terms) {
          dirs_[v] |= (coef > 0.0) == lower ? kUp : kDown;
        }
      } else {
        mark(n.lhs.tree, kOther);
        mark(n.rhs.tree, kOther);
      }
    }
  }

  // The value an effect computes reads its fluents; conditional effects read their conditions
  void effect(const sas::Effect & e)
  {
    condition(e.condition);
    condition(e.start_condition);
    if (e.kind == sas::Effect::Kind::Numeric) {
      mark(e.value.tree, kOther);
    }
  }

  std::map<sas::NumVarId, uint8_t> dirs_;
};

ground::Expr constant(double v)
{
  ground::Expr e;
  e.op = ExprOp::Constant;
  e.value = v;
  return e;
}

// lhs >= rhs, evaluated on the trees (no linear form: the bounds read fluents)
sas::NumericCondition at_least(const ground::Expr & lhs, const ground::Expr & rhs)
{
  sas::NumericCondition c;
  c.op = sas::CompareOp::Ge;
  c.lhs.tree = lhs;
  c.rhs.tree = rhs;
  return c;
}

}  // namespace

FixedDurations fix_durations(const sas::Task & task)
{
  FixedDurations out;
  out.task = task;
  out.task.durative_actions.clear();
  Readers readers(task);
  std::vector<double> initial;
  for (const auto & n : task.initial_numeric) {
    initial.push_back(n ? *n : state::kUndefined);
  }
  for (const auto & da : task.durative_actions) {
    bool continuous = std::any_of(
      da.effects.begin(), da.effects.end(), [](const sas::Effect & e) {return e.continuous;});
    bool fixed = da.duration.size() == 1 && da.duration[0].op == sas::CompareOp::Eq;
    if (!da.uses_duration && !continuous) {
      out.task.durative_actions.push_back(da);
      continue;
    }
    std::string name = sas::instance_name(task, da);
    // Continuous effects: their total change at the safest end
    sas::DurativeAction base = da;
    for (auto & e : base.effects) {
      if (!e.continuous) {
        continue;
      }
      if (e.kind != sas::Effect::Kind::Numeric ||
        (e.op != ast::AssignOp::Increase && e.op != ast::AssignOp::Decrease) ||
        has(e.value.tree, ExprOp::Duration))
      {
        out.unsupported = name + ": a continuous effect other than a linear increase or decrease";
        return out;
      }
      // The sign of the rate in the initial state (with #t = 1) says which way it goes
      ground::Expr total = e.value.tree;
      t_to_duration(total);
      double rate = state::evaluate(total, initial.data(), 1.0);
      if (std::isnan(rate)) {
        out.unsupported = name + ": the rate of a continuous effect has no value initially";
        return out;
      }
      bool down = (e.op == ast::AssignOp::Decrease) == (rate >= 0.0);
      auto r = static_cast<uint8_t>(readers.of(e.num) & (kUp | kDown));
      if (readers.of(e.num) & kOther || r == (kUp | kDown)) {
        out.unsupported = name + ": a continuous effect on a variable read in other ways than "
          "\"large enough\" or \"small enough\" alone";
        return out;
      }
      // Pessimistic for the readers: losses as soon as possible, gains as late as possible
      bool early = r == kDown ? !down : down;
      e.timing = early ? sas::Timing::Start : sas::Timing::End;
      e.value.tree = std::move(total);
      e.value.linear.reset();
      e.continuous = false;
      base.uses_duration = true;
      out.changed = true;
    }
    if (fixed) {
      out.task.durative_actions.push_back(std::move(base));
      continue;
    }
    // Inequalities: one action per bound, which must satisfy the other bounds
    std::vector<ground::Expr> lows, highs;
    for (const auto & d : da.duration) {
      if (d.op == sas::CompareOp::Ge || d.op == sas::CompareOp::Gt) {
        lows.push_back(d.value.tree);
      } else if (d.op == sas::CompareOp::Le || d.op == sas::CompareOp::Lt) {
        highs.push_back(d.value.tree);
      } else if (d.op == sas::CompareOp::Eq) {
        lows.push_back(d.value.tree);
        highs.push_back(d.value.tree);
      }
    }
    if (lows.empty()) {
      lows.push_back(constant(0.0));  // a duration is never negative
    }
    auto variant = [&](const ground::Expr & chosen) {
        sas::DurativeAction v = base;
        sas::DurationConstraint c;
        c.op = sas::CompareOp::Eq;
        c.value.tree = chosen;
        v.duration = {c};
        for (const auto & l : lows) {
          v.at_start.numeric.push_back(at_least(chosen, l));
        }
        for (const auto & h : highs) {
          v.at_start.numeric.push_back(at_least(h, chosen));
        }
        out.task.durative_actions.push_back(std::move(v));
      };
    for (const auto & l : lows) {
      variant(l);
    }
    for (const auto & h : highs) {
      variant(h);
    }
    out.changed = true;
  }
  return out;
}

}  // namespace kairos::temporal
