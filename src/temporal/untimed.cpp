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

#include "temporal/untimed.hpp"

#include <algorithm>
#include <string>
#include <utility>
#include <vector>

#include "sas/translate.hpp"
#include "state/eval.hpp"
#include "temporal/snap.hpp"

namespace kairos::temporal
{

namespace
{

// A duration that is above 0 whatever the state: a constant lower bound
bool surely_positive(const sas::DurativeAction & da)
{
  return std::any_of(da.duration.begin(), da.duration.end(), [](const sas::DurationConstraint & d) {
             const auto & lin = d.value.linear;
             if (!lin || !lin->terms.empty()) {
               return false;
             }
             return ((d.op == sas::CompareOp::Eq || d.op == sas::CompareOp::Ge) &&
                    lin->constant > 0.0) || (d.op == sas::CompareOp::Gt && lin->constant >= 0.0);
         });
}

}  // namespace

Untimed untimed(const sas::Task & task)
{
  Untimed u;
  if (!task.processes.empty() || !task.events.empty()) {
    u.unsupported = "processes or events";
    return u;
  }
  for (const auto & da : task.durative_actions) {
    for (const auto & e : da.effects) {
      if (e.continuous && u.unsupported.empty()) {
        u.unsupported = sas::instance_name(task, da) + " has continuous effects";
      }
    }
  }
  SnapTask s = snap(task);
  if (u.unsupported.empty()) {
    u.unsupported = s.unsupported;
  }
  if (!u.unsupported.empty()) {
    return u;
  }
  u.task = std::move(s.task);
  u.running = s.running;
  if (s.tils < u.task.actions.size()) {
    // Every plan applies every timed literal (the goal holds after the last one, as VAL checks)
    u.task.goal.facts.push_back(u.task.actions.back().changes.front());
    std::sort(u.task.goal.facts.begin(), u.task.goal.facts.end());
  }
  u.task.symmetries.clear();  // the new variables are not in them
  std::vector<sas::VarId> started;
  for (size_t i = 0; i < task.durative_actions.size(); ++i) {
    const auto & da = task.durative_actions[i];
    u.over_all.push_back(da.over_all);
    if (da.over_all.empty() || !surely_positive(da)) {
      continue;  // ending at once breaks nothing it reads
    }
    // Started in this instant: the end waits for an advance, which checks the over all
    sas::VarId fresh = add_variable(
      u.task, {"kairos-started-now " + task.durative_action_names[da.schema]},
      sas::Variable::Kind::Binary, 1, da.args);
    u.task.actions[s.instant + 2 * i].changes.push_back(sas::Fact{fresh, 0});
    auto & end = u.task.actions[s.instant + 2 * i + 1];
    end.pre.facts.push_back(sas::Fact{fresh, 1});
    std::sort(end.pre.facts.begin(), end.pre.facts.end());
    started.push_back(fresh);
  }
  sas::Action advance;
  advance.schema = static_cast<uint32_t>(u.task.action_names.size());
  u.task.action_names.push_back("kairos-advance");
  for (auto v : started) {
    advance.changes.push_back(sas::Fact{v, 1});
  }
  u.advance = static_cast<uint32_t>(u.task.actions.size());
  u.task.actions.push_back(std::move(advance));
  return u;
}

bool Untimed::accept(uint32_t action, const uint32_t * values, const double * numbers) const
{
  if (action != advance) {
    return true;
  }
  if (!task.always.empty() && !state::holds(task.always, task, values, numbers)) {
    return false;
  }
  for (size_t i = 0; i < running.size(); ++i) {
    if (values[running[i]] == 0 && !state::holds(over_all[i], task, values, numbers)) {
      return false;
    }
  }
  return true;
}

}  // namespace kairos::temporal
