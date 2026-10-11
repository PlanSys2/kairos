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

#include "temporal/profile.hpp"

#include <set>
#include <string>
#include <utility>
#include <vector>

#include "sas/translate.hpp"

namespace kairos::temporal
{

namespace
{

// What something reads: values (v = x), negated values (v != w), whole variables (comparisons,
// copies) and numeric variables
struct Reads
{
  std::set<std::pair<sas::VarId, uint32_t>> facts, not_facts;
  std::set<sas::VarId> whole;
  std::set<sas::NumVarId> nums;

  void merge(const Reads & o)
  {
    facts.insert(o.facts.begin(), o.facts.end());
    not_facts.insert(o.not_facts.begin(), o.not_facts.end());
    whole.insert(o.whole.begin(), o.whole.end());
    nums.insert(o.nums.begin(), o.nums.end());
  }
  // Something that holds while the variable has value x
  bool sees(sas::VarId v, uint32_t x) const
  {
    if (whole.count(v) || facts.count({v, x})) {
      return true;
    }
    for (auto it = not_facts.lower_bound({v, 0}); it != not_facts.end() && it->first == v; ++it) {
      if (it->second != x) {
        return true;
      }
    }
    return false;
  }
  void expr(const ground::Expr & e)
  {
    if (e.op == ground::ExprOp::Fluent) {
      nums.insert(static_cast<sas::NumVarId>(e.fluent));
    }
    for (const auto & c : e.children) {
      expr(c);
    }
  }
  void expr(const sas::NumericExpr & e) {expr(e.tree);}
  void condition(const sas::Condition & c)
  {
    for (const auto & f : c.facts) {
      facts.emplace(f.var, f.value);
    }
    for (const auto & f : c.not_facts) {
      not_facts.emplace(f.var, f.value);
    }
    for (const auto & v : c.vars) {
      whole.insert(v.a);
      whole.insert(v.b);
    }
    for (const auto & n : c.numeric) {
      expr(n.lhs);
      expr(n.rhs);
    }
  }
  void effect(const sas::Effect & e)
  {
    condition(e.condition);
    condition(e.start_condition);
    if (e.kind == sas::Effect::Kind::Numeric) {
      expr(e.value);
    }
    if (e.kind == sas::Effect::Kind::Copy) {
      whole.insert(e.source);
    }
  }
};

// Variables and numeric variables an action changes at a given timing
struct Writes
{
  std::set<std::pair<sas::VarId, uint32_t>> facts;
  std::set<sas::NumVarId> nums;
};

Writes writes(const sas::DurativeAction & a, sas::Timing timing)
{
  Writes w;
  for (const auto & f : timing == sas::Timing::Start ? a.start : a.end) {
    w.facts.emplace(f.var, f.value);
  }
  for (const auto & e : a.effects) {
    if (e.timing != timing) {
      continue;
    }
    if (e.kind == sas::Effect::Kind::Assign) {
      w.facts.emplace(e.fact.var, e.fact.value);
    } else if (e.kind == sas::Effect::Kind::Numeric && !e.continuous) {
      w.nums.insert(e.num);
    } else if (e.kind == sas::Effect::Kind::Copy) {
      w.facts.emplace(e.target, UINT32_MAX);
    }
  }
  return w;
}

Reads reads_of(const sas::DurativeAction & a)
{
  Reads r;
  r.condition(a.at_start);
  r.condition(a.over_all);
  r.condition(a.at_end);
  for (const auto & e : a.effects) {
    r.effect(e);
  }
  for (const auto & d : a.duration) {
    r.expr(d.value);
  }
  return r;
}

Reads reads_of(const sas::Action & a)
{
  Reads r;
  r.condition(a.pre);
  for (const auto & e : a.effects) {
    r.effect(e);
  }
  return r;
}

}  // namespace

TemporalProfile analyze(const sas::Task & task)
{
  TemporalProfile p;
  p.durative = !task.durative_actions.empty();
  p.timed_initial_literals = !task.timed_events.empty();
  if (!p.durative) {
    return p;
  }
  // What each action reads; derived variables read what their axioms read
  std::vector<Reads> durative_reads;
  for (const auto & a : task.durative_actions) {
    durative_reads.push_back(reads_of(a));
    for (const auto & d : a.duration) {
      p.variable_duration = p.variable_duration || d.op != sas::CompareOp::Eq;
    }
    p.duration_in_effects = p.duration_in_effects || a.uses_duration;
  }
  // Instant actions and axioms (a derived atom may be read by anyone): one set
  Reads instant;
  for (const auto & a : task.actions) {
    instant.merge(reads_of(a));
  }
  for (const auto & ax : task.axioms) {
    instant.condition(ax.body);
  }
  auto seen_by_others = [&](size_t self, sas::VarId v, uint32_t x) {
      if (instant.sees(v, x)) {
        return true;
      }
      for (size_t b = 0; b < durative_reads.size(); ++b) {
        if (b != self && durative_reads[b].sees(v, x)) {
          return true;
        }
      }
      return false;
    };
  auto read_by_others = [&](size_t self, sas::NumVarId n) {
      if (instant.nums.count(n)) {
        return true;
      }
      for (size_t b = 0; b < durative_reads.size(); ++b) {
        if (b != self && durative_reads[b].nums.count(n)) {
          return true;
        }
      }
      return false;
    };
  for (size_t a = 0; a < task.durative_actions.size(); ++a) {
    const auto & da = task.durative_actions[a];
    Writes s = writes(da, sas::Timing::Start);
    Writes e = writes(da, sas::Timing::End);
    std::string name = sas::instance_name(task, da);
    // A value set at start and changed again at end is visible only while the action runs
    std::set<sas::VarId> end_vars;
    for (const auto & [v, x] : e.facts) {
      end_vars.insert(v);
    }
    for (const auto & [v, x] : s.facts) {
      if (!end_vars.count(v) || e.facts.count({v, x})) {
        continue;
      }
      // The value between start and end: does anything else need it?
      bool seen = x == UINT32_MAX ? seen_by_others(a, v, UINT32_MAX - 1) : seen_by_others(a, v, x);
      if (seen) {
        p.simple = false;
        p.reasons.push_back(
          name + " sets " + (x == UINT32_MAX ? std::string("an object fluent") :
          sas::fact_name(task, sas::Fact{v, x})) +
          " only while it runs, and other actions can use that");
        break;
      }
    }
    for (auto n : s.nums) {
      if (e.nums.count(n) && read_by_others(a, n)) {
        p.simple = false;
        p.reasons.push_back(
          name + " changes a numeric fluent at start and at end, and other actions read it");
        break;
      }
    }
    if (p.reasons.size() > 20) {
      break;  // enough to explain it
    }
  }
  // Timed initial literals change what some action reads at fixed times
  if (p.timed_initial_literals) {
    p.simple = false;
    p.reasons.push_back("timed initial literals: actions must happen within time windows");
  }
  return p;
}

}  // namespace kairos::temporal
