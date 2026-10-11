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
#include <map>
#include <set>
#include <utility>
#include <vector>

#include "sas/task.hpp"

namespace kairos::temporal
{

// What a happening reads and writes, for ordering: variables and numeric variables. Over all
// conditions are apart (inv_*): they hold in the open interval of the action, so they need no
// separation from the happenings that give or take them at its start and end
struct Access
{
  std::vector<sas::VarId> reads, writes, inv_reads;
  std::vector<sas::NumVarId> num_reads, num_writes, inv_num_reads;

  void invariant(const sas::Condition & c)
  {
    Access a;
    a.read(c);
    inv_reads.insert(inv_reads.end(), a.reads.begin(), a.reads.end());
    inv_num_reads.insert(inv_num_reads.end(), a.num_reads.begin(), a.num_reads.end());
  }

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
    for (auto * vec : {&reads, &inv_reads}) {
      std::vector<sas::VarId> extra;
      for (auto v : *vec) {
        if (v < derived_base.size()) {
          extra.insert(extra.end(), derived_base[v].begin(), derived_base[v].end());
        }
      }
      vec->insert(vec->end(), extra.begin(), extra.end());
    }
    for (auto * vec : {&reads, &writes, &inv_reads}) {
      std::sort(vec->begin(), vec->end());
      vec->erase(std::unique(vec->begin(), vec->end()), vec->end());
    }
    for (auto * vec : {&num_reads, &num_writes, &inv_num_reads}) {
      std::sort(vec->begin(), vec->end());
      vec->erase(std::unique(vec->begin(), vec->end()), vec->end());
    }
  }
};

// Who a new happening must follow, and by how much: the last writer of what it reads or writes,
// and those that read what it writes since; epsilon, or 0 through over all conditions
template<class Node>
class Orderer
{
public:
  std::map<Node, double> before(const Access & acc, double epsilon) const
  {
    std::map<Node, double> out;
    auto need = [&](Node n, double sep) {
        auto [it, fresh] = out.emplace(n, sep);
        if (!fresh) {
          it->second = std::max(it->second, sep);
        }
      };
    auto scan = [&](const auto & last, const auto & seen_by, const auto & rd, const auto & inv,
      const auto & wr) {
        for (auto v : rd) {
          if (auto it = last.find(v); it != last.end()) {
            need(it->second, epsilon);
          }
        }
        for (auto v : inv) {
          if (auto it = last.find(v); it != last.end()) {
            need(it->second, 0.0);
          }
        }
        for (auto v : wr) {
          if (auto it = last.find(v); it != last.end()) {
            need(it->second, epsilon);
          }
          if (auto it = seen_by.find(v); it != seen_by.end()) {
            for (const auto & [n, invariant] : it->second) {
              need(n, invariant ? 0.0 : epsilon);
            }
          }
        }
      };
    scan(last_write_, readers_, acc.reads, acc.inv_reads, acc.writes);
    scan(last_num_write_, num_readers_, acc.num_reads, acc.inv_num_reads, acc.num_writes);
    return out;
  }

  void record(Node node, const Access & acc)
  {
    auto put = [&](auto & last, auto & seen_by, const auto & rd, const auto & inv,
      const auto & wr) {
        for (auto v : wr) {
          last[v] = node;
          seen_by[v].clear();
        }
        for (auto v : rd) {
          if (!std::binary_search(wr.begin(), wr.end(), v)) {
            seen_by[v].emplace_back(node, false);
          }
        }
        for (auto v : inv) {
          if (!std::binary_search(wr.begin(), wr.end(), v)) {
            seen_by[v].emplace_back(node, true);
          }
        }
      };
    put(last_write_, readers_, acc.reads, acc.inv_reads, acc.writes);
    put(last_num_write_, num_readers_, acc.num_reads, acc.inv_num_reads, acc.num_writes);
  }

private:
  std::map<sas::VarId, Node> last_write_;
  std::map<sas::VarId, std::vector<std::pair<Node, bool>>> readers_;
  std::map<sas::NumVarId, Node> last_num_write_;
  std::map<sas::NumVarId, std::vector<std::pair<Node, bool>>> num_readers_;
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
