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

/// \file
/// \brief What happenings read and write, to order them (engines M2 and M3).

#ifndef KAIROS__TEMPORAL__ACCESS_HPP_
#define KAIROS__TEMPORAL__ACCESS_HPP_

#include <algorithm>
#include <set>
#include <vector>

#include "sas/task.hpp"

namespace kairos::temporal
{

// What a step reads and writes, for ordering: variables and numeric variables
struct Access
{
  std::vector<sas::VarId> reads, writes;
  std::vector<sas::NumVarId> num_reads, num_writes;

  void read(const sas::Condition & c)
  {
    for (const auto & f : c.facts) {
      reads.push_back(f.var);
    }
    for (const auto & f : c.not_facts) {
      reads.push_back(f.var);
    }
    for (const auto & v : c.vars) {
      reads.push_back(v.a);
      reads.push_back(v.b);
    }
    for (const auto & n : c.numeric) {
      read(n.lhs.tree);
      read(n.rhs.tree);
    }
  }
  void read(const ground::Expr & e)
  {
    if (e.op == ground::ExprOp::Fluent) {
      num_reads.push_back(static_cast<sas::NumVarId>(e.fluent));
    }
    for (const auto & ch : e.children) {
      read(ch);
    }
  }
  void effect(const sas::Effect & e)
  {
    read(e.condition);
    read(e.start_condition);
    switch (e.kind) {
      case sas::Effect::Kind::Assign: writes.push_back(e.fact.var); break;
      case sas::Effect::Kind::Numeric:
        read(e.value.tree);
        num_writes.push_back(e.num);
        num_reads.push_back(e.num);  // increase reads the old value
        break;
      case sas::Effect::Kind::Copy:
        reads.push_back(e.source);
        writes.push_back(e.target);
        break;
    }
  }
  void finish(const std::vector<std::vector<sas::VarId>> & derived_base)
  {
    // A derived variable stands for what its axioms read
    std::vector<sas::VarId> extra;
    for (auto v : reads) {
      if (v < derived_base.size()) {
        extra.insert(extra.end(), derived_base[v].begin(), derived_base[v].end());
      }
    }
    reads.insert(reads.end(), extra.begin(), extra.end());
    for (auto * vec : {&reads, &writes}) {
      std::sort(vec->begin(), vec->end());
      vec->erase(std::unique(vec->begin(), vec->end()), vec->end());
    }
    for (auto * vec : {&num_reads, &num_writes}) {
      std::sort(vec->begin(), vec->end());
      vec->erase(std::unique(vec->begin(), vec->end()), vec->end());
    }
  }
};

// For each derived variable, the base variables its axioms read, through other derived ones
inline std::vector<std::vector<sas::VarId>> derived_bases(const sas::Task & t)
{
  std::vector<std::vector<sas::VarId>> direct(t.variables.size());
  for (const auto & ax : t.axioms) {
    auto & d = direct[ax.head.var];
    for (const auto & f : ax.body.facts) {
      d.push_back(f.var);
    }
    for (const auto & f : ax.body.not_facts) {
      d.push_back(f.var);
    }
  }
  std::vector<std::vector<sas::VarId>> out(t.variables.size());
  for (sas::VarId v = 0; v < t.variables.size(); ++v) {
    if (direct[v].empty()) {
      continue;
    }
    std::set<sas::VarId> seen;
    std::vector<sas::VarId> stack = direct[v];
    while (!stack.empty()) {
      auto x = stack.back();
      stack.pop_back();
      if (!seen.insert(x).second) {
        continue;
      }
      stack.insert(stack.end(), direct[x].begin(), direct[x].end());
    }
    out[v].assign(seen.begin(), seen.end());
  }
  return out;
}

}  // namespace kairos::temporal

#endif  // KAIROS__TEMPORAL__ACCESS_HPP_
