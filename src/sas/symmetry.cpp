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

#include "sas/symmetry.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <map>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace kairos::sas
{

namespace
{

// The images of facts and numeric variables under swapping a and b (nullopt: not a symmetry)
struct Mapping
{
  std::vector<std::vector<Fact>> fact;  // per variable and value
  std::vector<VarId> var;
  std::vector<NumVarId> num;
};

ObjectId swap(ObjectId o, ObjectId a, ObjectId b)
{
  return o == a ? b : (o == b ? a : o);
}

std::vector<ObjectId> swapped(std::vector<ObjectId> args, ObjectId a, ObjectId b)
{
  for (auto & o : args) {
    o = swap(o, a, b);
  }
  return args;
}

class Finder
{
public:
  explicit Finder(const Task & t)
  : t_(t)
  {
    for (uint32_t i = 0; i < t.atoms.size(); ++i) {
      atom_of_[{t.atoms[i].predicate.index(), t.atoms[i].args}] = i;
    }
    for (uint32_t i = 0; i < t.numeric_fluents.size(); ++i) {
      fluent_of_[{t.numeric_fluents[i].function.index(), t.numeric_fluents[i].args}] = i;
    }
    for (uint32_t i = 0; i < t.actions.size(); ++i) {
      auto & list = action_of_[{t.actions[i].schema, t.actions[i].args}];
      position_.push_back(static_cast<uint32_t>(list.size()));
      list.push_back(i);
    }
    // What each action reads or writes: one that touches nothing a swap moves is its own image
    touches_.resize(t.actions.size());
    for (uint32_t i = 0; i < t.actions.size(); ++i) {
      auto & tv = touches_[i];
      auto cond = [&](const Condition & c) {
          for (const auto & f : c.facts) {
            tv.vars.push_back(f.var);
          }
          for (const auto & f : c.not_facts) {
            tv.vars.push_back(f.var);
          }
          for (const auto & n : c.numeric) {
            fluents(n.lhs.tree, tv.nums);
            fluents(n.rhs.tree, tv.nums);
          }
          for (const auto & v : c.vars) {
            tv.vars.push_back(v.a);
            tv.vars.push_back(v.b);
          }
        };
      const Action & a = t.actions[i];
      cond(a.pre);
      for (const auto & f : a.changes) {
        tv.vars.push_back(f.var);
      }
      for (const auto & e : a.effects) {
        cond(e.condition);
        cond(e.start_condition);
        tv.vars.push_back(e.fact.var);
        if (e.kind == Effect::Kind::Numeric) {
          tv.nums.push_back(e.num);
          fluents(e.value.tree, tv.nums);
        }
      }
    }
  }

  static void fluents(const ground::Expr & e, std::vector<NumVarId> & out)
  {
    if (e.op == ground::ExprOp::Fluent) {
      out.push_back(e.fluent);
    }
    for (const auto & c : e.children) {
      fluents(c, out);
    }
  }

  std::optional<Symmetry> test(ObjectId a, ObjectId b)
  {
    auto m = mapping(a, b);
    if (!m) {
      return std::nullopt;
    }
    Symmetry s;
    s.a = a;
    s.b = b;
    for (VarId v = 0; v < t_.variables.size(); ++v) {
      for (uint32_t x = 0; x < t_.variables[v].size(); ++x) {
        Fact f{v, x};
        Fact g = m->fact[v][x];
        if (f < g) {
          s.facts.emplace_back(f, g);
        }
      }
    }
    for (NumVarId n = 0; n < m->num.size(); ++n) {
      if (n < m->num[n]) {
        s.numbers.emplace_back(n, m->num[n]);
      }
    }
    // Each action to an action with the same conditions and effects, mapped
    s.image.resize(t_.actions.size());
    for (uint32_t i = 0; i < t_.actions.size(); ++i) {
      const Action & act = t_.actions[i];
      bool mentions = std::find(act.args.begin(), act.args.end(), a) != act.args.end() ||
        std::find(act.args.begin(), act.args.end(), b) != act.args.end();
      if (!mentions) {
        const auto & tv = touches_[i];
        bool moved = std::any_of(tv.vars.begin(), tv.vars.end(), [&](VarId v) {
              return m->var[v] != v || std::any_of(
                m->fact[v].begin(), m->fact[v].end(), [&](Fact f) {return f.var != v;});
            }) ||
          std::any_of(tv.nums.begin(), tv.nums.end(), [&](NumVarId n) {return m->num[n] != n;});
        if (!moved) {
          s.image[i] = i;  // touches nothing the swap moves
          continue;
        }
      }
      auto it = action_of_.find({act.schema, swapped(act.args, a, b)});
      if (it == action_of_.end() || it->second.size() <= position_[i]) {
        return std::nullopt;
      }
      uint32_t j = it->second[position_[i]];
      if (key(act, &*m) != key(t_.actions[j], nullptr)) {
        return std::nullopt;
      }
      s.image[i] = j;
    }
    // The rest of the task to itself
    auto same_set = [&](auto items, auto one) {
        std::vector<std::string> mine, mapped;
        for (const auto & x : items) {
          mine.push_back(one(x, nullptr));
          mapped.push_back(one(x, &*m));
        }
        std::sort(mine.begin(), mine.end());
        std::sort(mapped.begin(), mapped.end());
        return mine == mapped;
      };
    auto axiom = [&](const Axiom & ax, const Mapping * mm) {
        return cond(ax.body, mm) + ">" + fact(ax.head, mm);
      };
    auto effect_key = [&](const Effect & e, const Mapping * mm) {return effect(e, mm);};
    auto deadline = [&](const ClockDeadline & d, const Mapping * mm) {
        return fact(d.when, mm) + "|" + std::to_string(num(d.clock, mm)) + "|" +
               (d.bound ? std::to_string(num(*d.bound, mm)) : std::to_string(d.at)) +
               (d.hard ? "h" : "s");
      };
    if (!same_set(t_.axioms, axiom) || !same_set(t_.clock_rates, effect_key) ||
      !same_set(t_.clock_deadlines, deadline) ||
      cond(t_.goal, &*m) != cond(t_.goal, nullptr) ||
      cond(t_.always, &*m) != cond(t_.always, nullptr) ||
      expr(t_.metric.expr.tree, &*m) != expr(t_.metric.expr.tree, nullptr))
    {
      return std::nullopt;
    }
    return s;
  }

private:
  std::optional<Mapping> mapping(ObjectId a, ObjectId b)
  {
    Mapping m;
    m.fact.resize(t_.variables.size());
    m.var.assign(t_.variables.size(), UINT32_MAX);
    for (VarId v = 0; v < t_.variables.size(); ++v) {
      const auto & var = t_.variables[v];
      m.fact[v].resize(var.size());
      for (uint32_t x = 0; x < var.size(); ++x) {
        auto atom = x < var.atoms.size() ? var.atoms[x] : kNoAtom;
        if (atom == kNoAtom) {
          continue;
        }
        const auto & info = t_.atoms[atom];
        auto it = atom_of_.find({info.predicate.index(), swapped(info.args, a, b)});
        if (it == atom_of_.end()) {
          return std::nullopt;
        }
        auto [w, y] = t_.atom_fact[it->second];
        if (m.var[v] != UINT32_MAX && m.var[v] != w) {
          return std::nullopt;  // a variable split over two
        }
        m.var[v] = w;
        m.fact[v][x] = Fact{w, y};
      }
      if (m.var[v] == UINT32_MAX) {
        m.var[v] = v;  // no atom moves it
      }
    }
    for (VarId v = 0; v < t_.variables.size(); ++v) {
      const auto & var = t_.variables[v];
      VarId w = m.var[v];
      if (t_.variables[w].size() != var.size()) {
        return std::nullopt;
      }
      for (uint32_t x = 0; x < var.size(); ++x) {
        auto atom = x < var.atoms.size() ? var.atoms[x] : kNoAtom;
        if (atom == kNoAtom) {
          m.fact[v][x] = Fact{w, x};  // "none of these" to "none of those"
        }
      }
    }
    m.num.resize(t_.numeric_fluents.size());
    for (NumVarId n = 0; n < t_.numeric_fluents.size(); ++n) {
      const auto & info = t_.numeric_fluents[n];
      auto it = fluent_of_.find({info.function.index(), swapped(info.args, a, b)});
      if (it == fluent_of_.end()) {
        return std::nullopt;
      }
      m.num[n] = it->second;
    }
    return m;
  }

  std::string fact(Fact f, const Mapping * m) const
  {
    if (m) {
      f = m->fact[f.var][f.value];
    }
    return std::to_string(f.var) + "=" + std::to_string(f.value);
  }

  uint32_t num(NumVarId n, const Mapping * m) const {return m ? m->num[n] : n;}

  std::string expr(const ground::Expr & e, const Mapping * m) const
  {
    std::string out = "(" + std::to_string(static_cast<int>(e.op));
    if (e.op == ground::ExprOp::Fluent) {
      out += " f" + std::to_string(num(e.fluent, m));
    } else if (e.op == ground::ExprOp::Constant) {
      uint64_t bits;
      std::memcpy(&bits, &e.value, sizeof bits);
      out += " " + std::to_string(bits);
    }
    if (!e.name.empty()) {
      out += " " + e.name;
    }
    for (const auto & c : e.children) {
      out += " " + expr(c, m);
    }
    return out + ")";
  }

  std::string cond(const Condition & c, const Mapping * m) const
  {
    std::vector<std::string> parts;
    for (const auto & f : c.facts) {
      parts.push_back("+" + fact(f, m));
    }
    for (const auto & f : c.not_facts) {
      parts.push_back("-" + fact(f, m));
    }
    for (const auto & n : c.numeric) {
      parts.push_back(
        "n" + std::to_string(static_cast<int>(n.op)) + expr(n.lhs.tree, m) + expr(n.rhs.tree, m));
    }
    for (const auto & v : c.vars) {
      VarId a = m ? m->var[v.a] : v.a;
      VarId b = m ? m->var[v.b] : v.b;
      parts.push_back(
        "v" + std::to_string(std::min(a, b)) + "," + std::to_string(std::max(a, b)) +
        (v.equal ? "=" : "!"));
    }
    std::sort(parts.begin(), parts.end());
    std::string out;
    for (const auto & p : parts) {
      out += p + ";";
    }
    return out;
  }

  std::string effect(const Effect & e, const Mapping * m) const
  {
    std::string out = std::to_string(static_cast<int>(e.kind)) + "/" +
      std::to_string(static_cast<int>(e.timing)) + "[" + cond(e.condition, m) + "][" +
      cond(e.start_condition, m) + "]";
    switch (e.kind) {
      case Effect::Kind::Assign: out += fact(e.fact, m); break;
      case Effect::Kind::Numeric:
        out += "n" + std::to_string(num(e.num, m)) + "o" +
          std::to_string(static_cast<int>(e.op)) + expr(e.value.tree, m) +
          (e.continuous ? "c" : "");
        break;
      case Effect::Kind::Copy: out += "copy"; break;
    }
    return out;
  }

  std::string key(const Action & a, const Mapping * m) const
  {
    std::vector<std::string> parts;
    for (const auto & f : a.changes) {
      parts.push_back("c" + fact(f, m));
    }
    for (const auto & e : a.effects) {
      parts.push_back("e" + effect(e, m));
    }
    std::sort(parts.begin(), parts.end());
    std::string out = cond(a.pre, m) + "|";
    for (const auto & p : parts) {
      out += p + ";";
    }
    return out;
  }

  const Task & t_;
  std::map<std::pair<uint32_t, std::vector<ObjectId>>, uint32_t> atom_of_;
  std::map<std::pair<uint32_t, std::vector<ObjectId>>, uint32_t> fluent_of_;
  std::map<std::pair<uint32_t, std::vector<ObjectId>>, std::vector<uint32_t>> action_of_;
  std::vector<uint32_t> position_;  // per action: its place among those with its schema and args
  struct Touched
  {
    std::vector<VarId> vars;
    std::vector<NumVarId> nums;
  };
  std::vector<Touched> touches_;
};

}  // namespace

std::vector<Symmetry> find_symmetries(const Task & task, size_t max_actions)
{
  std::vector<Symmetry> out;
  bool object_fluents = std::any_of(
    task.variables.begin(), task.variables.end(),
    [](const Variable & v) {return v.kind == Variable::Kind::ObjectFluent;});
  if (object_fluents || !task.durative_actions.empty() || !task.processes.empty() ||
    !task.events.empty() || !task.timed_events.empty() || task.actions.size() > max_actions)
  {
    return out;
  }
  // Signature of an object: where it appears (schema or predicate or function, and position)
  std::map<ObjectId, std::vector<std::string>> sig;
  auto note = [&](const std::string & what, const std::vector<ObjectId> & args) {
      for (size_t i = 0; i < args.size(); ++i) {
        sig[args[i]].push_back(what + "#" + std::to_string(i));
      }
    };
  for (const auto & a : task.actions) {
    note("a" + std::to_string(a.schema), a.args);
  }
  for (const auto & at : task.atoms) {
    note("p" + std::to_string(at.predicate.index()), at.args);
  }
  for (const auto & f : task.numeric_fluents) {
    note("f" + std::to_string(f.function.index()), f.args);
  }
  std::map<std::vector<std::string>, std::vector<ObjectId>> classes;
  for (auto & [o, s] : sig) {
    std::sort(s.begin(), s.end());
    classes[s].push_back(o);
  }
  Finder finder(task);
  for (auto & [s, objects] : classes) {
    std::sort(objects.begin(), objects.end());
    for (size_t i = 0; i + 1 < objects.size(); ++i) {
      if (auto sym = finder.test(objects[i], objects[i + 1])) {
        out.push_back(std::move(*sym));
      }
    }
  }
  return out;
}

bool invariant(const Symmetry & s, const uint32_t * values, const double * numbers)
{
  for (const auto & [f, g] : s.facts) {
    if ((values[f.var] == f.value) != (values[g.var] == g.value)) {
      return false;
    }
  }
  for (const auto & [x, y] : s.numbers) {
    double a = numbers[x];
    double b = numbers[y];
    if (a != b && !(std::isnan(a) && std::isnan(b))) {
      return false;
    }
  }
  return true;
}

}  // namespace kairos::sas
