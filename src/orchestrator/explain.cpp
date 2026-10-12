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

#include "orchestrator/explain.hpp"

#include <map>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace kairos::orch
{

namespace
{

constexpr int kMaxDepth = 4;
constexpr size_t kMaxAdders = 3;

// A fact with some arguments unknown (nullopt): "(person_at jack ?ro)"
struct Pattern
{
  uint32_t predicate;
  std::vector<std::optional<uint32_t>> args;
  std::vector<std::string> names;  // of the unknown arguments, to print them
};

// Something that makes a fact true: an action (or its effect) or a derived predicate rule
struct Adder
{
  std::string name;
  const std::vector<lifted::Variable> * variables;
  const lifted::Atom * head;
  std::vector<const lifted::Literal *> needs;  // positive preconditions
};

class Explainer
{
public:
  Explainer(const lifted::Task & task, const ground::Task & grounded)
  : t_(task)
  {
    for (const auto & a : grounded.atoms) {
      std::vector<uint32_t> args;
      for (auto o : a.args) {
        args.push_back(o.value());
      }
      reachable_[a.predicate.value()].push_back(std::move(args));
    }
    for (const auto & a : task.actions) {
      add(a.name, a.variables, {&a.precondition}, a.effects);
    }
    for (const auto & a : task.durative_actions) {
      add(a.name, a.variables, {&a.at_start, &a.over_all, &a.at_end}, a.effects);
    }
    for (const auto & ax : task.axioms) {
      Adder ad{t_.predicate_names[ax.head.value()], &ax.variables, nullptr, {}};
      axiom_heads_.emplace_back();
      for (uint32_t i = 0; i < ax.head_arity; ++i) {
        axiom_heads_.back().args.push_back(lifted::Term::variable(i));
      }
      axiom_heads_.back().predicate = ax.head;
      positive(ax.body, ad.needs);
      adders_.push_back(std::move(ad));
      axiom_index_.push_back(adders_.size() - 1);
    }
    // Axiom heads live in axiom_heads_ (stable now): point to them
    for (size_t i = 0; i < axiom_index_.size(); ++i) {
      adders_[axiom_index_[i]].head = &axiom_heads_[i];
    }
  }

  std::vector<std::string> goal()
  {
    std::vector<std::string> out;
    for (const auto & l : t_.goal.literals) {
      if (!l.positive) {
        continue;
      }
      Pattern p = pattern(l.atom, nullptr, {});
      if (reached(p)) {
        continue;
      }
      out.push_back(print(p) + " can never be reached:");
      explain(p, 1, out);
    }
    return out;
  }

private:
  static void positive(const lifted::Condition & c, std::vector<const lifted::Literal *> & out)
  {
    for (const auto & l : c.literals) {
      if (l.positive) {
        out.push_back(&l);
      }
    }
  }

  void add(
    const std::string & name, const std::vector<lifted::Variable> & vars,
    std::vector<const lifted::Condition *> conditions, const std::vector<lifted::Effect> & effects)
  {
    for (const auto & e : effects) {
      if (e.kind != lifted::Effect::Kind::Add) {
        continue;
      }
      Adder ad{name, &vars, &e.atom, {}};
      for (const auto * c : conditions) {
        positive(*c, ad.needs);
      }
      positive(e.condition, ad.needs);
      positive(e.start_condition, ad.needs);
      adders_.push_back(std::move(ad));
    }
  }

  // The fact an atom of a schema stands for, with what the binding knows
  Pattern pattern(
    const lifted::Atom & a, const std::vector<lifted::Variable> * vars,
    const std::vector<std::optional<uint32_t>> & binding) const
  {
    Pattern p{a.predicate.value(), {}, {}};
    for (const auto & term : a.args) {
      std::optional<uint32_t> value;
      std::string name = "?";
      if (term.kind == lifted::Term::Kind::Object) {
        value = term.index;
      } else if (term.kind == lifted::Term::Kind::Variable) {
        if (term.index < binding.size()) {
          value = binding[term.index];
        }
        if (vars && term.index < vars->size()) {
          name = (*vars)[term.index].name;
        }
      }
      p.args.push_back(value);
      p.names.push_back(name);
    }
    return p;
  }

  bool reached(const Pattern & p) const
  {
    auto it = reachable_.find(p.predicate);
    if (it == reachable_.end()) {
      return false;
    }
    for (const auto & args : it->second) {
      bool match = args.size() == p.args.size();
      for (size_t i = 0; match && i < args.size(); ++i) {
        match = !p.args[i] || *p.args[i] == args[i];
      }
      if (match) {
        return true;
      }
    }
    return false;
  }

  std::string print(const Pattern & p) const
  {
    std::string s = "(" + t_.predicate_names[p.predicate];
    for (size_t i = 0; i < p.args.size(); ++i) {
      s += " " + (p.args[i] ? t_.object_names[*p.args[i]] : p.names[i]);
    }
    return s + ")";
  }

  // The binding of an adder's variables that makes its head the pattern (nullopt: it cannot)
  std::optional<std::vector<std::optional<uint32_t>>> unify(
    const Adder & ad, const Pattern & p) const
  {
    std::vector<std::optional<uint32_t>> binding(ad.variables->size());
    if (ad.head->args.size() != p.args.size()) {
      return std::nullopt;
    }
    for (size_t i = 0; i < p.args.size(); ++i) {
      const auto & term = ad.head->args[i];
      if (term.kind == lifted::Term::Kind::Object) {
        if (p.args[i] && *p.args[i] != term.index) {
          return std::nullopt;
        }
      } else if (term.kind == lifted::Term::Kind::Variable && p.args[i]) {
        auto & b = binding[term.index];
        if (b && *b != *p.args[i]) {
          return std::nullopt;
        }
        b = p.args[i];
      }
    }
    return binding;
  }

  void explain(const Pattern & p, int depth, std::vector<std::string> & out)
  {
    std::string indent(static_cast<size_t>(2 * depth), ' ');
    if (!visited_.insert(print(p)).second) {
      return;
    }
    std::vector<std::pair<const Adder *, std::vector<std::optional<uint32_t>>>> adders;
    for (const auto & ad : adders_) {
      if (ad.head->predicate.value() != p.predicate) {
        continue;
      }
      if (auto b = unify(ad, p)) {
        adders.emplace_back(&ad, std::move(*b));
      }
    }
    if (adders.empty()) {
      out.push_back(indent + "no action adds it, and the initial state does not have it");
      return;
    }
    for (size_t k = 0; k < adders.size() && k < kMaxAdders; ++k) {
      const auto & [ad, binding] = adders[k];
      std::vector<Pattern> blockers;
      for (const auto * l : ad->needs) {
        Pattern need = pattern(l->atom, ad->variables, binding);
        if (!reached(need)) {
          blockers.push_back(std::move(need));
        }
      }
      if (blockers.empty()) {
        out.push_back(indent + ad->name + " adds it, but is never applicable (types, "
          "negative or numeric conditions)");
        continue;
      }
      out.push_back(indent + ad->name + " adds it, but needs " + print(blockers.front()) +
        ", which never holds");
      if (depth < kMaxDepth) {
        explain(blockers.front(), depth + 1, out);
      }
    }
    if (adders.size() > kMaxAdders) {
      out.push_back(indent + "(and " + std::to_string(adders.size() - kMaxAdders) +
        " more ways to add it)");
    }
  }

  const lifted::Task & t_;
  std::map<uint32_t, std::vector<std::vector<uint32_t>>> reachable_;
  std::vector<Adder> adders_;
  std::vector<lifted::Atom> axiom_heads_;
  std::vector<size_t> axiom_index_;
  std::set<std::string> visited_;
};

}  // namespace

std::vector<std::string> explain_unreachable_goal(
  const lifted::Task & task, const ground::Task & grounded)
{
  return Explainer(task, grounded).goal();
}

}  // namespace kairos::orch
