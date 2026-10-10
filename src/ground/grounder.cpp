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

#include "ground/grounder.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <map>
#include <set>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "ground/datalog.hpp"

namespace kairos::ground
{

namespace
{

using Key = std::vector<uint32_t>;

struct KeyHash
{
  size_t operator()(const Key & k) const
  {
    uint64_t h = 0xcbf29ce484222325ULL;
    for (uint32_t v : k) {
      h ^= v + 0x9e3779b97f4a7c15ULL + (h << 6) + (h >> 2);
    }
    return h;
  }
};

constexpr uint32_t kNone = datalog::Program::kUnbound;

// Inputs the grounder does not handle yet
struct Unsupported
{
  std::string message;
};

struct OutOfBudget
{
};

using lifted::Variable;
using Binding = std::vector<uint32_t>;

bool compare(CompareOp op, double a, double b)
{
  switch (op) {
    case CompareOp::Eq: return a == b;
    case CompareOp::Ne: return a != b;
    case CompareOp::Lt: return a < b;
    case CompareOp::Le: return a <= b;
    case CompareOp::Gt: return a > b;
    case CompareOp::Ge: return a >= b;
  }
  return false;
}

// Value of an arithmetic node with constant children, or nullopt (not foldable or undefined)
std::optional<double> fold(ExprOp op, const std::vector<Expr> & c)
{
  for (const auto & x : c) {
    if (x.op != ExprOp::Constant) {
      return std::nullopt;
    }
  }
  if (c.empty()) {
    return std::nullopt;
  }
  switch (op) {
    case ExprOp::Add: {
        double s = 0;
        for (const auto & x : c) {
          s += x.value;
        }
        return s;
      }
    case ExprOp::Sub: {
        if (c.size() == 1) {
          return -c[0].value;
        }
        double s = c[0].value;
        for (size_t i = 1; i < c.size(); ++i) {
          s -= c[i].value;
        }
        return s;
      }
    case ExprOp::Mul: {
        double s = 1;
        for (const auto & x : c) {
          s *= x.value;
        }
        return s;
      }
    case ExprOp::Div:
      if (c.size() == 2 && c[1].value != 0.0) {
        return c[0].value / c[1].value;
      }
      return std::nullopt;
    case ExprOp::Neg: return -c[0].value;
    default: return std::nullopt;
  }
}

class Grounder
{
public:
  Grounder(const lifted::Task & t, const GroundOptions & o, Budget * b)
  : t_(t), options_(o), log_(o.logger), budget_(b) {}

  Result<Task> run(GroundStats * stats)
  {
    auto t0 = std::chrono::steady_clock::now();
    try {
      analyze_statics();
      build_program();
      if (!prog_.solve(budget_)) {
        return Result<Task>::failure(budget_error());
      }
      build_task();
      prune_undefined();
      if (options_.drop_useless_actions) {
        drop_useless();
      }
    } catch (const Unsupported & u) {
      return Result<Task>::failure(Diagnostic::error("ground.unsupported", u.message));
    } catch (const OutOfBudget &) {
      return Result<Task>::failure(budget_error());
    }
    if (budget_ && budget_->exhausted()) {
      return Result<Task>::failure(budget_error());
    }
    double ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - t0).count();
    log_.info(
      LogChannel::Ground,
      "grounded in {:.1f} ms: {} atoms, {} numeric fluents, {} actions, {} durative actions, "
      "{} axioms ({} Datalog facts, {} rules, {} instances pruned)",
      ms, out_.atoms.size(), out_.numeric_fluents.size(), out_.actions.size(),
      out_.durative_actions.size(), out_.axioms.size(), prog_.stats().facts,
      prog_.stats().rules, pruned_);
    if (stats) {
      stats->datalog_facts = prog_.stats().facts;
      stats->datalog_rules = prog_.stats().rules;
      stats->pruned_instances = pruned_;
      stats->milliseconds = ms;
    }
    return Result<Task>::success(std::move(out_));
  }

private:
  Diagnostic budget_error() const
  {
    std::string why = "cancelled";
    if (budget_->state() == BudgetState::Timeout) {
      why = "out of time";
    } else if (budget_->state() == BudgetState::OutOfMemory) {
      why = "out of memory";
    }
    return Diagnostic::error(
      "ground.budget", "grounding stopped (" + why + ") after " +
      std::to_string(prog_.stats().facts) + " reachable facts: the task is too large to ground");
  }

  // --- Statics -------------------------------------------------------------------------------

  void mark_effects(const std::vector<lifted::Effect> & effects)
  {
    for (const auto & e : effects) {
      if (e.kind == lifted::Effect::Kind::Add || e.kind == lifted::Effect::Kind::Delete) {
        fluent_pred_[e.atom.predicate.index()] = true;
      } else {
        changing_fn_[e.function.index()] = true;
      }
    }
  }

  void analyze_statics()
  {
    fluent_pred_ = t_.derived;
    changing_fn_.assign(t_.function_names.size(), false);
    for (const auto & a : t_.actions) {
      mark_effects(a.effects);
    }
    for (const auto & a : t_.durative_actions) {
      mark_effects(a.effects);
    }
    for (const auto & a : t_.processes) {
      mark_effects(a.effects);
    }
    for (const auto & a : t_.events) {
      mark_effects(a.effects);
    }
    for (const auto & ev : t_.timed_events) {
      if (ev.fact) {
        fluent_pred_[ev.fact->first.index()] = true;
      }
      if (ev.fluent) {
        changing_fn_[ev.fluent->function.index()] = true;
      }
    }
    static_derived();
    for (const auto & f : t_.init_fluents) {
      Key k{f.function.value()};
      for (auto o : f.args) {
        k.push_back(o.value());
      }
      if (f.numeric) {
        init_num_[k] = f.number;
      } else if (f.object) {
        init_obj_[k] = f.object->value();
      }
    }
  }

  bool static_fn(FunctionId f) const {return !changing_fn_[f.index()];}

  // Derived predicates that only read static facts are static too: Datalog computes them
  // exactly when their rules have no negated derived atoms and no foralls
  void static_derived()
  {
    std::vector<bool> candidate(t_.predicate_names.size(), false);
    for (size_t p = 0; p < candidate.size(); ++p) {
      candidate[p] = t_.derived[p];
    }
    for (bool changed = true; changed; ) {
      changed = false;
      for (const auto & ax : t_.axioms) {
        size_t h = ax.head.index();
        if (!candidate[h]) {
          continue;
        }
        bool ok = ax.body.foralls.empty();
        for (const auto & l : ax.body.literals) {
          size_t q = l.atom.predicate.index();
          bool st = t_.derived[q] ? candidate[q] && l.positive : !fluent_pred_[q];
          ok = ok && st && std::all_of(
            l.atom.args.begin(), l.atom.args.end(),
            [&](const lifted::Term & x) {return static_term(x);});
        }
        for (const auto & c : ax.body.comparisons) {
          ok = ok && (c.objects ? static_term(c.lhs_term) && static_term(c.rhs_term) :
            static_expr(c.lhs) && static_expr(c.rhs));
        }
        if (!ok) {
          candidate[h] = false;
          changed = true;
        }
      }
    }
    for (size_t p = 0; p < candidate.size(); ++p) {
      if (candidate[p]) {
        fluent_pred_[p] = false;
      }
    }
  }

  // --- Terms and expressions on bindings (for checks while solving) --------------------------

  // Object denoted by a term; false if undefined or not static
  bool eval_term(const lifted::Term & x, const uint32_t * b, uint32_t & out) const
  {
    switch (x.kind) {
      case lifted::Term::Kind::Variable: out = b[x.index]; return true;
      case lifted::Term::Kind::Object: out = x.index; return true;
      case lifted::Term::Kind::Function: {
          Key k{x.index};
          for (const auto & a : x.args) {
            uint32_t v;
            if (!eval_term(a, b, v)) {
              return false;
            }
            k.push_back(v);
          }
          auto it = init_obj_.find(k);
          if (it == init_obj_.end()) {
            return false;
          }
          out = it->second;
          return true;
        }
    }
    return false;
  }

  // Terms whose value grounding can compute: no changing object fluent inside
  bool static_term(const lifted::Term & x) const
  {
    if (x.kind != lifted::Term::Kind::Function) {
      return true;
    }
    if (!static_fn(FunctionId(x.index))) {
      return false;
    }
    return std::all_of(
      x.args.begin(), x.args.end(), [&](const lifted::Term & a) {return static_term(a);});
  }

  // Expression made of constants and static fluents only
  bool static_expr(const lifted::Expr & e) const
  {
    switch (e.op) {
      case ExprOp::Constant: return true;
      case ExprOp::Fluent:
        return static_fn(e.function) && std::all_of(
          e.args.begin(), e.args.end(), [&](const lifted::Term & a) {return static_term(a);});
      case ExprOp::Add: case ExprOp::Sub: case ExprOp::Mul: case ExprOp::Div:
      case ExprOp::Neg: case ExprOp::Pow: case ExprOp::Math:
        return std::all_of(
          e.children.begin(), e.children.end(),
          [&](const lifted::Expr & c) {return static_expr(c);});
      default: return false;
    }
  }

  // Every static fluent the expression reads is defined (changing ones are not looked at)
  bool static_defined(const lifted::Expr & e, const uint32_t * b) const
  {
    if (e.op == ExprOp::Fluent && static_fn(e.function)) {
      Key k{e.function.value()};
      for (const auto & a : e.args) {
        uint32_t v;
        if (!static_term(a) || !eval_term(a, b, v)) {
          return !static_term(a);  // a changing argument: unknown, not undefined
        }
        k.push_back(v);
      }
      return init_num_.count(k) > 0;
    }
    return std::all_of(
      e.children.begin(), e.children.end(),
      [&](const lifted::Expr & c) {return static_defined(c, b);});
  }

  void term_vars(const lifted::Term & x, std::set<uint32_t> & out) const
  {
    if (x.kind == lifted::Term::Kind::Variable) {
      out.insert(x.index);
    }
    for (const auto & a : x.args) {
      term_vars(a, out);
    }
  }

  void expr_vars(const lifted::Expr & e, std::set<uint32_t> & out) const
  {
    for (const auto & a : e.args) {
      term_vars(a, out);
    }
    for (const auto & c : e.children) {
      expr_vars(c, out);
    }
  }

  // --- Datalog program -----------------------------------------------------------------------

  uint32_t type_predicate(TypeSpec spec)
  {
    std::sort(spec.begin(), spec.end());
    auto it = type_pred_.find(spec);
    if (it != type_pred_.end()) {
      return it->second;
    }
    std::string name = "type:";
    for (auto ty : spec) {
      name += t_.types[ty].name + ",";
    }
    uint32_t p = cur_->add_predicate(1, name);
    for (uint32_t o = 0; o < t_.object_names.size(); ++o) {
      if (t_.types.fits(t_.object_types[o], spec)) {
        cur_->add_fact(p, std::span<const uint32_t>(&o, 1));
      }
    }
    cur_->mark_static(p);
    type_pred_.emplace(spec, p);
    return p;
  }

  // Relation f(args..., value) of a static object fluent
  uint32_t object_fluent_predicate(FunctionId f)
  {
    auto it = objfn_pred_.find(f.value());
    if (it != objfn_pred_.end()) {
      return it->second;
    }
    uint32_t p = cur_->add_predicate(
      static_cast<uint32_t>(t_.function_params[f.index()].size() + 1),
      "fluent:" + t_.function_names[f.index()]);
    for (const auto & [k, v] : init_obj_) {
      if (k[0] == f.value()) {
        Key args(k.begin() + 1, k.end());
        args.push_back(v);
        cur_->add_fact(p, args);
      }
    }
    cur_->mark_static(p);
    objfn_pred_.emplace(f.value(), p);
    return p;
  }

  datalog::Term dterm(const lifted::Term & x, datalog::Rule & r)
  {
    switch (x.kind) {
      case lifted::Term::Kind::Variable: return datalog::Term::var(x.index);
      case lifted::Term::Kind::Object: return datalog::Term::constant(x.index);
      case lifted::Term::Kind::Function: {
          if (!static_fn(FunctionId(x.index))) {
            throw Unsupported{"object fluent '" + t_.function_names[x.index] +
                    "' changes and is used as an argument of an atom; not supported yet"};
          }
          datalog::Atom a;
          a.predicate = object_fluent_predicate(FunctionId(x.index));
          for (const auto & arg : x.args) {
            a.args.push_back(dterm(arg, r));
          }
          uint32_t v = r.variables++;
          a.args.push_back(datalog::Term::var(v));
          r.body.push_back(std::move(a));
          return datalog::Term::var(v);
        }
    }
    return datalog::Term::var(0);
  }

  datalog::Atom datom(const lifted::Atom & a, datalog::Rule & r)
  {
    datalog::Atom out;
    out.predicate = a.predicate.value();
    for (const auto & x : a.args) {
      out.args.push_back(dterm(x, r));
    }
    return out;
  }

  // Positive atoms to the body; static negations and static comparisons as checks
  void add_condition(const lifted::Condition & c, datalog::Rule & r, bool only_static = false)
  {
    for (const auto & l : c.literals) {
      bool st = !fluent_pred_[l.atom.predicate.index()];
      if (l.positive) {
        if (!only_static || st) {
          r.body.push_back(datom(l.atom, r));
        }
        continue;
      }
      if (!st || !std::all_of(
          l.atom.args.begin(), l.atom.args.end(),
          [&](const lifted::Term & x) {return static_term(x);}))
      {
        continue;
      }
      datalog::Check ch;
      std::set<uint32_t> vs;
      for (const auto & x : l.atom.args) {
        term_vars(x, vs);
      }
      ch.variables.assign(vs.begin(), vs.end());
      ch.test = [this, l](const uint32_t * b) {
          Key args;
          for (const auto & x : l.atom.args) {
            uint32_t v;
            if (!eval_term(x, b, v)) {
              return true;  // an undefined argument: the atom is false, its negation true
            }
            args.push_back(v);
          }
          return cur_->find(l.atom.predicate.value(), args) == kNone;
        };
      r.checks.push_back(std::move(ch));
    }
    for (const auto & cmp : c.comparisons) {
      datalog::Check ch;
      std::set<uint32_t> vs;
      if (cmp.objects) {
        if (!static_term(cmp.lhs_term) || !static_term(cmp.rhs_term)) {
          continue;
        }
        term_vars(cmp.lhs_term, vs);
        term_vars(cmp.rhs_term, vs);
        ch.test = [this, cmp](const uint32_t * b) {
            uint32_t x, y;
            if (!eval_term(cmp.lhs_term, b, x) || !eval_term(cmp.rhs_term, b, y)) {
              return false;
            }
            return (x == y) == (cmp.op == CompareOp::Eq);
          };
      } else {
        if (!static_expr(cmp.lhs) || !static_expr(cmp.rhs)) {
          // Changing fluents: at least the static ones read must be defined
          expr_vars(cmp.lhs, vs);
          expr_vars(cmp.rhs, vs);
          ch.test = [this, cmp](const uint32_t * b) {
              return static_defined(cmp.lhs, b) && static_defined(cmp.rhs, b);
            };
        } else {
          expr_vars(cmp.lhs, vs);
          expr_vars(cmp.rhs, vs);
          size_t n = vs.empty() ? 0 : *vs.rbegin() + 1;
          ch.test = [this, cmp, n](const uint32_t * b) {
              Binding bb(b, b + n);  // only the variables of the comparison are bound
              Expr x, y;
              if (!gexpr(cmp.lhs, bb, x) || !gexpr(cmp.rhs, bb, y)) {
                return false;
              }
              return compare(cmp.op, x.value, y.value);
            };
        }
      }
      ch.variables.assign(vs.begin(), vs.end());
      r.checks.push_back(std::move(ch));
    }
  }

  // Unconditional effects reading undefined static fluents make the action impossible
  void add_effect_checks(const std::vector<lifted::Effect> & effects, datalog::Rule & r)
  {
    for (const auto & e : effects) {
      if (e.kind != lifted::Effect::Kind::Numeric || !e.variables.empty() ||
        !e.condition.empty() || !e.start_condition.empty())
      {
        continue;
      }
      datalog::Check ch;
      std::set<uint32_t> vs;
      expr_vars(e.value, vs);
      ch.variables.assign(vs.begin(), vs.end());
      ch.test = [this, &e](const uint32_t * b) {return static_defined(e.value, b);};
      r.checks.push_back(std::move(ch));
    }
  }

  // Type atoms for the variables in [from, to) that no body atom already restricts
  void add_types(
    const std::vector<Variable> & vars, uint32_t from, uint32_t to, datalog::Rule & r,
    const std::vector<uint32_t> & extra = {})
  {
    auto restricted = [&](uint32_t v) {
        for (const auto & a : r.body) {
          if (a.predicate >= t_.predicate_params.size()) {
            if (type_of_.count(a.predicate) && a.args[0].is_variable && a.args[0].value == v) {
              const TypeSpec & have = type_of_.at(a.predicate);
              if (covers(have, vars[v].types)) {
                return true;
              }
            }
            continue;
          }
          const auto & params = t_.predicate_params[a.predicate];
          for (size_t i = 0; i < a.args.size() && i < params.size(); ++i) {
            if (a.args[i].is_variable && a.args[i].value == v && covers(params[i], vars[v].types)) {
              return true;
            }
          }
        }
        return false;
      };
    auto add = [&](uint32_t v) {
        if (!restricted(v)) {
          uint32_t p = type_predicate(vars[v].types);
          type_of_.emplace(p, sorted(vars[v].types));
          r.body.push_back(datalog::Atom{p, {datalog::Term::var(v)}});
        }
      };
    for (uint32_t v = from; v < to; ++v) {
      add(v);
    }
    for (uint32_t v : extra) {
      add(v);
    }
  }

  static TypeSpec sorted(TypeSpec s)
  {
    std::sort(s.begin(), s.end());
    return s;
  }

  // Every object allowed by `have` belongs to `want`
  bool covers(const TypeSpec & have, const TypeSpec & want) const
  {
    if (have.empty()) {
      return false;
    }
    return std::all_of(
      have.begin(), have.end(), [&](sem::TypeId h) {
        return std::any_of(
          want.begin(), want.end(), [&](sem::TypeId w) {return t_.types.is_subtype(h, w);});
      });
  }

  datalog::Atom instance_atom(uint32_t pred, uint32_t n)
  {
    datalog::Atom a{pred, {}};
    for (uint32_t v = 0; v < n; ++v) {
      a.args.push_back(datalog::Term::var(v));
    }
    return a;
  }

  // Rules making the add effects of a schema instance reachable
  void add_effect_rules(
    const std::vector<lifted::Effect> & effects, const std::vector<Variable> & vars,
    uint32_t start_pred, uint32_t end_pred, uint32_t n)
  {
    for (const auto & e : effects) {
      if (e.kind != lifted::Effect::Kind::Add) {
        continue;
      }
      datalog::Rule r;
      r.variables = static_cast<uint32_t>(vars.size());
      uint32_t base = e.timing == Timing::End ? end_pred : start_pred;
      r.body.push_back(instance_atom(base, n));
      add_condition(e.condition, r);
      add_condition(e.start_condition, r);
      r.head = datom(e.atom, r);
      add_types(vars, 0, 0, r, e.variables);
      cur_->add_rule(std::move(r));
    }
  }

  // Positive equalities between variables and objects become a substitution in the rule, so
  // joins use them as keys instead of filtering a product (as FD's translator does)
  static void substitute_equalities(
    datalog::Rule & r, std::initializer_list<const lifted::Condition *> conds)
  {
    std::vector<uint32_t> parent(r.variables);
    for (uint32_t v = 0; v < r.variables; ++v) {
      parent[v] = v;
    }
    std::vector<std::optional<uint32_t>> value(r.variables);  // object bound to a root
    std::function<uint32_t(uint32_t)> root = [&](uint32_t v) {
        return parent[v] == v ? v : parent[v] = root(parent[v]);
      };
    bool changed = false;
    bool impossible = false;
    for (const auto * c : conds) {
      for (const auto & cmp : c->comparisons) {
        if (!cmp.objects || cmp.op != CompareOp::Eq) {
          continue;
        }
        const auto & x = cmp.lhs_term;
        const auto & y = cmp.rhs_term;
        using K = lifted::Term::Kind;
        if (x.kind == K::Function || y.kind == K::Function) {
          continue;
        }
        if (x.kind == K::Object && y.kind == K::Object) {
          impossible = impossible || x.index != y.index;
          continue;
        }
        if (x.kind == K::Variable && y.kind == K::Variable) {
          uint32_t a = root(x.index), b = root(y.index);
          if (a != b) {
            if (value[a] && value[b] && *value[a] != *value[b]) {
              impossible = true;
            }
            parent[b] = a;
            if (!value[a]) {
              value[a] = value[b];
            }
            changed = true;
          }
          continue;
        }
        uint32_t v = root(x.kind == K::Variable ? x.index : y.index);
        uint32_t o = x.kind == K::Object ? x.index : y.index;
        impossible = impossible || (value[v] && *value[v] != o);
        value[v] = o;
        changed = true;
      }
    }
    if (impossible) {
      // Contradictory equalities: the rule never fires
      r.checks.push_back(datalog::Check{{}, [](const uint32_t *) {return false;}});
      return;
    }
    if (!changed) {
      return;
    }
    std::vector<datalog::Term> map(r.variables);
    for (uint32_t v = 0; v < r.variables; ++v) {
      uint32_t w = root(v);
      map[v] = value[w] ? datalog::Term::constant(*value[w]) : datalog::Term::var(w);
    }
    auto apply = [&](datalog::Atom & a) {
        for (auto & t : a.args) {
          if (t.is_variable) {
            t = map[t.value];
          }
        }
      };
    for (auto & a : r.body) {
      apply(a);
    }
    apply(r.head);
    for (auto & c : r.checks) {
      std::set<uint32_t> vs;
      for (uint32_t v : c.variables) {
        if (map[v].is_variable) {
          vs.insert(map[v].value);
        }
      }
      c.variables.assign(vs.begin(), vs.end());
      // The test reads the original variables: rebuild them from the representatives
      c.test = [map, test = std::move(c.test)](const uint32_t * b) {
          thread_local std::vector<uint32_t> orig;
          orig.resize(map.size());
          for (size_t v = 0; v < map.size(); ++v) {
            orig[v] = map[v].is_variable ? b[map[v].value] : map[v].value;
          }
          return test(orig.data());
        };
    }
  }

  uint32_t instant_rules(const lifted::Action & a, const std::string & kind)
  {
    uint32_t n = a.parameters + a.hidden;
    uint32_t inst = cur_->add_predicate(n, kind + ":" + a.name);
    datalog::Rule r;
    r.variables = static_cast<uint32_t>(a.variables.size());
    add_condition(a.precondition, r);
    add_effect_checks(a.effects, r);
    add_types(a.variables, 0, n, r);
    r.head = instance_atom(inst, n);
    substitute_equalities(r, {&a.precondition});
    cur_->add_rule(std::move(r));
    add_effect_rules(a.effects, a.variables, inst, inst, n);
    return inst;
  }

  // Predicates and facts of the task in the current program
  void add_base()
  {
    for (size_t p = 0; p < t_.predicate_names.size(); ++p) {
      cur_->add_predicate(
        static_cast<uint32_t>(t_.predicate_params[p].size()), t_.predicate_names[p]);
    }
    for (const auto & [p, args] : t_.init_facts) {
      Key k;
      for (auto o : args) {
        k.push_back(o.value());
      }
      cur_->add_fact(p.value(), k);
    }
    for (const auto & ev : t_.timed_events) {
      if (ev.fact && !ev.negated) {
        Key k;
        for (auto o : ev.fact->second) {
          k.push_back(o.value());
        }
        cur_->add_fact(ev.fact->first.value(), k);
      }
    }
  }

  uint32_t axiom_rules(const lifted::Axiom & ax)
  {
    uint32_t n = ax.head_arity + ax.existential;
    uint32_t inst = cur_->add_predicate(n, "axiom:" + t_.predicate_names[ax.head.index()]);
    datalog::Rule r;
    r.variables = static_cast<uint32_t>(ax.variables.size());
    add_condition(ax.body, r);
    add_types(ax.variables, 0, n, r);
    r.head = instance_atom(inst, n);
    substitute_equalities(r, {&ax.body});
    cur_->add_rule(std::move(r));
    datalog::Rule h;
    h.variables = n;
    h.body.push_back(instance_atom(inst, n));
    h.head = instance_atom(ax.head.value(), ax.head_arity);
    cur_->add_rule(std::move(h));
    return inst;
  }

  // Static derived predicates first, in a program of their own: the main program then reads
  // them as facts, also under negation
  std::vector<std::pair<uint32_t, Key>> static_derived_facts()
  {
    std::vector<std::pair<uint32_t, Key>> out;
    bool any = std::any_of(
      t_.axioms.begin(), t_.axioms.end(),
      [&](const lifted::Axiom & ax) {return !fluent_pred_[ax.head.index()];});
    if (!any) {
      return out;
    }
    datalog::Program pre;
    cur_ = &pre;
    add_base();
    for (uint32_t p = 0; p < t_.predicate_names.size(); ++p) {
      if (!fluent_pred_[p] && !t_.derived[p]) {
        pre.mark_static(p);
      }
    }
    for (const auto & ax : t_.axioms) {
      if (!fluent_pred_[ax.head.index()]) {
        axiom_rules(ax);
      }
    }
    if (!pre.solve(budget_)) {
      cur_ = &prog_;
      throw OutOfBudget{};
    }
    for (uint32_t p = 0; p < t_.predicate_names.size(); ++p) {
      if (t_.derived[p] && !fluent_pred_[p]) {
        for (uint32_t f : pre.facts(p)) {
          auto a = pre.args(f);
          out.emplace_back(p, Key(a.begin(), a.end()));
        }
      }
    }
    type_pred_.clear();
    type_of_.clear();
    objfn_pred_.clear();
    cur_ = &prog_;
    return out;
  }

  void build_program()
  {
    auto derived_facts = static_derived_facts();
    cur_ = &prog_;
    add_base();
    for (const auto & [p, k] : derived_facts) {
      prog_.add_fact(p, k);
    }
    for (uint32_t p = 0; p < t_.predicate_names.size(); ++p) {
      if (!fluent_pred_[p]) {
        prog_.mark_static(p);
      }
    }

    for (const auto & a : t_.actions) {
      action_inst_.push_back(instant_rules(a, "action"));
    }
    for (const auto & a : t_.processes) {
      process_inst_.push_back(instant_rules(a, "process"));
    }
    for (const auto & a : t_.events) {
      event_inst_.push_back(instant_rules(a, "event"));
    }
    for (const auto & a : t_.durative_actions) {
      uint32_t n = a.parameters + a.hidden;
      uint32_t start = cur_->add_predicate(n, "start:" + a.name);
      uint32_t end = cur_->add_predicate(n, "end:" + a.name);
      datalog::Rule r;
      r.variables = static_cast<uint32_t>(a.variables.size());
      add_condition(a.at_start, r);
      add_condition(a.over_all, r, true);
      add_condition(a.at_end, r, true);
      for (const auto & d : a.duration) {
        datalog::Check ch;
        std::set<uint32_t> vs;
        expr_vars(d.value, vs);
        ch.variables.assign(vs.begin(), vs.end());
        ch.test = [this, &d](const uint32_t * b) {return static_defined(d.value, b);};
        r.checks.push_back(std::move(ch));
      }
      add_effect_checks(a.effects, r);
      add_types(a.variables, 0, n, r);
      r.head = instance_atom(start, n);
      substitute_equalities(r, {&a.at_start, &a.over_all, &a.at_end});
      cur_->add_rule(std::move(r));

      // The end needs the start: over-all and end conditions may come from start effects
      datalog::Rule e;
      e.variables = static_cast<uint32_t>(a.variables.size());
      e.body.push_back(instance_atom(start, n));
      lifted::Condition rest;
      for (const auto * c : {&a.over_all, &a.at_end}) {
        for (const auto & l : c->literals) {
          if (l.positive && fluent_pred_[l.atom.predicate.index()]) {
            rest.literals.push_back(l);
          }
        }
      }
      add_condition(rest, e);
      e.head = instance_atom(end, n);
      cur_->add_rule(std::move(e));
      add_effect_rules(a.effects, a.variables, start, end, n);
      durative_start_.push_back(start);
      durative_end_.push_back(end);
    }
    for (const auto & ax : t_.axioms) {
      axiom_inst_.push_back(fluent_pred_[ax.head.index()] ? axiom_rules(ax) : kNone);
    }
  }

  // --- Instantiation -------------------------------------------------------------------------

  const std::vector<uint32_t> & objects_of(const TypeSpec & spec)
  {
    TypeSpec k = sorted(spec);
    auto it = objects_.find(k);
    if (it != objects_.end()) {
      return it->second;
    }
    std::vector<uint32_t> out;
    for (uint32_t o = 0; o < t_.object_names.size(); ++o) {
      if (t_.types.fits(t_.object_types[o], spec)) {
        out.push_back(o);
      }
    }
    return objects_.emplace(k, std::move(out)).first->second;
  }

  FluentId numeric_fluent(const Key & k)
  {
    auto it = num_index_.find(k);
    if (it != num_index_.end()) {
      return it->second;
    }
    FluentInfo f;
    f.function = FunctionId(k[0]);
    for (size_t i = 1; i < k.size(); ++i) {
      f.args.push_back(ObjectId(k[i]));
    }
    auto id = static_cast<FluentId>(out_.numeric_fluents.size());
    out_.numeric_fluents.push_back(std::move(f));
    num_index_.emplace(k, id);
    return id;
  }

  FluentId object_fluent(const Key & k)
  {
    auto it = obj_index_.find(k);
    if (it != obj_index_.end()) {
      return it->second;
    }
    FluentInfo f;
    f.function = FunctionId(k[0]);
    for (size_t i = 1; i < k.size(); ++i) {
      f.args.push_back(ObjectId(k[i]));
    }
    auto id = static_cast<FluentId>(out_.object_fluents.size());
    out_.object_fluents.push_back(std::move(f));
    obj_index_.emplace(k, id);
    return id;
  }

  // 0: undefined; 1: defined (out set)
  bool gobj(const lifted::Term & x, const Binding & b, ObjectTerm & out)
  {
    switch (x.kind) {
      case lifted::Term::Kind::Variable: out = ObjectTerm{false, b[x.index]}; return true;
      case lifted::Term::Kind::Object: out = ObjectTerm{false, x.index}; return true;
      case lifted::Term::Kind::Function: {
          Key k{x.index};
          for (const auto & a : x.args) {
            ObjectTerm v;
            if (!gobj(a, b, v)) {
              return false;
            }
            if (v.is_fluent) {
              throw Unsupported{"nested changing object fluents ('" +
                      t_.function_names[x.index] + "') are not supported yet"};
            }
            k.push_back(v.index);
          }
          if (!static_fn(FunctionId(x.index))) {
            out = ObjectTerm{true, object_fluent(k)};
            return true;
          }
          auto it = init_obj_.find(k);
          if (it == init_obj_.end()) {
            return false;
          }
          out = ObjectTerm{false, it->second};
          return true;
        }
    }
    return false;
  }

  // False if undefined (the expression has no value)
  bool gexpr(const lifted::Expr & e, const Binding & b, Expr & out)
  {
    out = Expr{};
    out.op = e.op;
    out.value = e.value;
    out.name = e.name;
    if (e.op == ExprOp::Fluent) {
      Key k{e.function.value()};
      for (const auto & a : e.args) {
        uint32_t v;
        if (a.kind == lifted::Term::Kind::Function && !static_term(a)) {
          throw Unsupported{"changing object fluents inside numeric fluents ('" +
                  t_.function_names[e.function.index()] + "') are not supported yet"};
        }
        if (!eval_term(a, b.data(), v)) {
          return false;
        }
        k.push_back(v);
      }
      if (static_fn(e.function)) {
        auto it = init_num_.find(k);
        if (it == init_num_.end()) {
          return false;
        }
        out.op = ExprOp::Constant;
        out.value = it->second;
        return true;
      }
      out.fluent = numeric_fluent(k);
      return true;
    }
    for (const auto & c : e.children) {
      Expr g;
      if (!gexpr(c, b, g)) {
        return false;
      }
      out.children.push_back(std::move(g));
    }
    if (e.op == ExprOp::Div && out.children.size() == 2 &&
      out.children[1].op == ExprOp::Constant && out.children[1].value == 0.0)
    {
      return false;  // division by zero
    }
    if (auto v = fold(e.op, out.children)) {
      out = Expr{};
      out.value = *v;
    }
    return true;
  }

  // False if the condition can never hold
  bool gcond(const lifted::Condition & c, Binding & b, Condition & out)
  {
    for (const auto & l : c.literals) {
      Key args;
      bool defined = true;
      for (const auto & x : l.atom.args) {
        ObjectTerm v;
        if (!gobj(x, b, v)) {
          defined = false;
          break;
        }
        if (v.is_fluent) {
          throw Unsupported{"changing object fluents as arguments of atoms are not supported yet"};
        }
        args.push_back(v.index);
      }
      if (!defined) {
        if (l.positive) {
          return false;
        }
        continue;
      }
      uint32_t fact = prog_.find(l.atom.predicate.value(), args);
      if (!fluent_pred_[l.atom.predicate.index()]) {
        if ((fact != kNone) != l.positive) {
          return false;
        }
        continue;
      }
      uint32_t atom = fact == kNone ? kNone : fact_atom_[fact];
      if (atom == kNone) {
        if (l.positive) {
          return false;
        }
        continue;
      }
      (l.positive ? out.positive : out.negative).push_back(atom);
    }
    for (const auto & cmp : c.comparisons) {
      if (cmp.objects) {
        ObjectTerm x, y;
        if (!gobj(cmp.lhs_term, b, x) || !gobj(cmp.rhs_term, b, y)) {
          return false;
        }
        bool eq = cmp.op == CompareOp::Eq;
        if (!x.is_fluent && !y.is_fluent) {
          if ((x.index == y.index) != eq) {
            return false;
          }
          continue;
        }
        out.objects.push_back(ObjectCondition{eq, x, y});
        continue;
      }
      NumericCondition n{cmp.op, {}, {}};
      if (!gexpr(cmp.lhs, b, n.lhs) || !gexpr(cmp.rhs, b, n.rhs)) {
        return false;
      }
      if (n.lhs.op == ExprOp::Constant && n.rhs.op == ExprOp::Constant) {
        if (!compare(n.op, n.lhs.value, n.rhs.value)) {
          return false;
        }
        continue;
      }
      out.numeric.push_back(std::move(n));
    }
    for (const auto & q : c.foralls) {
      bool ok = true;
      enumerate(q.variables, 0, b, [&]() {
          ok = gcond(q.body, b, out);
          return ok;
        });
      if (!ok) {
        return false;
      }
    }
    return tidy(out);
  }

  // Sorts and removes duplicates; false if an atom is required both true and false
  static bool tidy(Condition & c)
  {
    for (auto * v : {&c.positive, &c.negative}) {
      std::sort(v->begin(), v->end());
      v->erase(std::unique(v->begin(), v->end()), v->end());
    }
    std::vector<AtomId> both;
    std::set_intersection(
      c.positive.begin(), c.positive.end(), c.negative.begin(), c.negative.end(),
      std::back_inserter(both));
    return both.empty();
  }

  // Checks the budget while instantiating (millions of instances may come); exhausted() is
  // cheap and does the full check every check_interval calls
  void tick()
  {
    if (budget_ && budget_->exhausted()) {
      throw OutOfBudget{};
    }
  }

  // Calls f for every binding of vars[i...]; stops when f returns false
  bool enumerate(
    const std::vector<uint32_t> & vars, size_t i, Binding & b, const std::function<bool()> & f)
  {
    if (i == vars.size()) {
      tick();
      return f();
    }
    for (uint32_t o : objects_of((*current_vars_)[vars[i]].types)) {
      b[vars[i]] = o;
      if (!enumerate(vars, i + 1, b, f)) {
        return false;
      }
    }
    return true;
  }

  // Puts unconditional atom changes in place and sorts them out (add wins over delete)
  static void tidy(AtomChanges & c)
  {
    for (auto * v : {&c.add, &c.del}) {
      std::sort(v->begin(), v->end());
      v->erase(std::unique(v->begin(), v->end()), v->end());
    }
    std::erase_if(
      c.del, [&](AtomId a) {return std::binary_search(c.add.begin(), c.add.end(), a);});
  }

  // False if an unconditional effect makes the instance impossible. Unconditional atom
  // changes go to `start` (instant and at-start effects) or `end`.
  bool geffects(
    const std::vector<lifted::Effect> & effects, Binding & b, std::vector<Effect> & out,
    AtomChanges & start, AtomChanges & end)
  {
    bool possible = true;
    for (const auto & e : effects) {
      bool unconditional = e.variables.empty() && e.condition.empty() &&
        e.start_condition.empty();
      enumerate(e.variables, 0, b, [&]() {
          Effect g;
          g.timing = e.timing;
          g.continuous = e.continuous;
          g.op = e.op;
          if (!gcond(e.condition, b, g.condition) ||
          !gcond(e.start_condition, b, g.start_condition))
          {
            return true;
          }
          switch (e.kind) {
            case lifted::Effect::Kind::Add:
            case lifted::Effect::Kind::Delete: {
                g.kind = e.kind == lifted::Effect::Kind::Add ? Effect::Kind::Add :
                Effect::Kind::Delete;
                Key args;
                for (const auto & x : e.atom.args) {
                  ObjectTerm v;
                  if (!gobj(x, b, v)) {
                    return true;  // an undefined argument: no atom to change
                  }
                  if (v.is_fluent) {
                    throw Unsupported{
                            "changing object fluents as arguments of atoms are not supported yet"};
                  }
                  args.push_back(v.index);
                }
                uint32_t fact = prog_.find(e.atom.predicate.value(), args);
                if (fact == kNone || fact_atom_[fact] == kNone) {
                  if (g.kind == Effect::Kind::Add) {
                    throw std::logic_error(
                      "grounding: an add effect is not in the reachable model");
                  }
                  return true;  // deletes an atom that is never true
                }
                g.atom = fact_atom_[fact];
                break;
              }
            case lifted::Effect::Kind::Numeric: {
                g.kind = Effect::Kind::Numeric;
                Key k{e.function.value()};
                for (const auto & x : e.args) {
                  ObjectTerm v;
                  if (!gobj(x, b, v) || v.is_fluent) {
                    return true;
                  }
                  k.push_back(v.index);
                }
                g.fluent = numeric_fluent(k);
                if (!gexpr(e.value, b, g.value)) {
                  if (unconditional) {
                    possible = false;
                  }
                  return true;
                }
                break;
              }
            case lifted::Effect::Kind::ObjectAssign: {
                g.kind = Effect::Kind::ObjectAssign;
                Key k{e.function.value()};
                for (const auto & x : e.args) {
                  ObjectTerm v;
                  if (!gobj(x, b, v) || v.is_fluent) {
                    return true;
                  }
                  k.push_back(v.index);
                }
                g.fluent = object_fluent(k);
                if (e.object) {
                  ObjectTerm v;
                  if (gobj(*e.object, b, v)) {
                    g.object = v;
                  }
                }
                break;
              }
          }
          if ((g.kind == Effect::Kind::Add || g.kind == Effect::Kind::Delete) &&
          g.condition.empty() && g.start_condition.empty())
          {
            AtomChanges & c = g.timing == Timing::End ? end : start;
            (g.kind == Effect::Kind::Add ? c.add : c.del).push_back(g.atom);
            return true;
          }
          out.push_back(std::move(g));
          return true;
        });
    }
    tidy(start);
    tidy(end);
    return possible;
  }

  Binding bind(uint32_t fact, size_t size)
  {
    Binding b(size, 0);
    auto args = prog_.args(fact);
    std::copy(args.begin(), args.end(), b.begin());
    return b;
  }

  void instantiate_instant(
    const std::vector<lifted::Action> & schemas, const std::vector<uint32_t> & preds,
    std::vector<Action> & out)
  {
    // Reserve once: growing a vector of millions of actions would hold two copies
    size_t total = 0;
    for (uint32_t p : preds) {
      total += prog_.facts(p).size();
    }
    out.reserve(total);
    for (uint32_t s = 0; s < schemas.size(); ++s) {
      const auto & a = schemas[s];
      current_vars_ = &a.variables;
      for (uint32_t fact : prog_.facts(preds[s])) {
        tick();
        Binding b = bind(fact, a.variables.size());
        Action g;
        g.schema = s;
        for (uint32_t i = 0; i < a.parameters; ++i) {
          g.args.push_back(ObjectId(b[i]));
        }
        AtomChanges none;
        if (!gcond(a.precondition, b, g.precondition) ||
          !geffects(a.effects, b, g.effects, g.changes, none))
        {
          ++pruned_;
          continue;
        }
        out.push_back(std::move(g));
      }
    }
  }

  void build_task()
  {
    out_.domain_name = t_.domain_name;
    out_.problem_name = t_.problem_name;
    out_.predicate_names = t_.predicate_names;
    out_.derived = t_.derived;
    out_.function_names = t_.function_names;
    out_.object_names = t_.object_names;
    out_.features = t_.features;
    out_.has_soft_goals = t_.has_soft_goals;
    out_.has_trajectory_constraints = t_.has_trajectory_constraints;
    for (const auto & a : t_.actions) {
      out_.action_names.push_back(a.name);
    }
    for (const auto & a : t_.durative_actions) {
      out_.durative_action_names.push_back(a.name);
    }
    for (const auto & a : t_.processes) {
      out_.process_names.push_back(a.name);
    }
    for (const auto & a : t_.events) {
      out_.event_names.push_back(a.name);
    }

    // Atoms: the reachable facts of changing predicates
    fact_atom_.assign(prog_.stats().facts, kNone);
    for (uint32_t p = 0; p < t_.predicate_names.size(); ++p) {
      if (!fluent_pred_[p]) {
        continue;
      }
      for (uint32_t fact : prog_.facts(p)) {
        fact_atom_[fact] = static_cast<AtomId>(out_.atoms.size());
        AtomInfo info;
        info.predicate = PredicateId(p);
        for (uint32_t o : prog_.args(fact)) {
          info.args.push_back(ObjectId(o));
        }
        out_.atoms.push_back(std::move(info));
      }
    }
    for (const auto & [p, args] : t_.init_facts) {
      if (fluent_pred_[p.index()]) {
        Key k;
        for (auto o : args) {
          k.push_back(o.value());
        }
        out_.init_atoms.push_back(fact_atom_[prog_.find(p.value(), k)]);
      }
    }
    std::sort(out_.init_atoms.begin(), out_.init_atoms.end());
    out_.init_atoms.erase(
      std::unique(out_.init_atoms.begin(), out_.init_atoms.end()), out_.init_atoms.end());
    // Initial values of changing fluents, in their order
    for (const auto & f : t_.init_fluents) {
      if (!static_fn(f.function)) {
        Key k{f.function.value()};
        for (auto o : f.args) {
          k.push_back(o.value());
        }
        f.numeric ? numeric_fluent(k) : object_fluent(k);
      }
    }

    instantiate_instant(t_.actions, action_inst_, out_.actions);
    instantiate_instant(t_.processes, process_inst_, out_.processes);
    instantiate_instant(t_.events, event_inst_, out_.events);

    size_t total = 0;
    for (uint32_t p : durative_end_) {
      total += prog_.facts(p).size();
    }
    out_.durative_actions.reserve(total);
    for (uint32_t s = 0; s < t_.durative_actions.size(); ++s) {
      const auto & a = t_.durative_actions[s];
      current_vars_ = &a.variables;
      for (uint32_t fact : prog_.facts(durative_end_[s])) {
        tick();
        Binding b = bind(fact, a.variables.size());
        DurativeAction g;
        g.schema = s;
        for (uint32_t i = 0; i < a.parameters; ++i) {
          g.args.push_back(ObjectId(b[i]));
        }
        bool ok = gcond(a.at_start, b, g.at_start) && gcond(a.over_all, b, g.over_all) &&
          gcond(a.at_end, b, g.at_end);
        for (const auto & d : a.duration) {
          DurationConstraint gd{d.op, d.timing, {}};
          ok = ok && gexpr(d.value, b, gd.value);
          g.duration.push_back(std::move(gd));
        }
        ok = ok && geffects(a.effects, b, g.effects, g.start, g.end);
        if (!ok) {
          ++pruned_;
          continue;
        }
        out_.durative_actions.push_back(std::move(g));
      }
    }

    for (uint32_t s = 0; s < t_.axioms.size(); ++s) {
      const auto & ax = t_.axioms[s];
      current_vars_ = &ax.variables;
      if (!fluent_pred_[ax.head.index()]) {
        continue;  // a static derived predicate: evaluated away
      }
      for (uint32_t fact : prog_.facts(axiom_inst_[s])) {
        tick();
        Binding b = bind(fact, ax.variables.size());
        Axiom g;
        Key head(b.begin(), b.begin() + ax.head_arity);
        g.head = fact_atom_[prog_.find(ax.head.value(), head)];
        if (!gcond(ax.body, b, g.body)) {
          ++pruned_;
          continue;
        }
        out_.axioms.push_back(std::move(g));
      }
    }

    // Goal
    std::vector<Variable> none;
    current_vars_ = &none;
    Binding empty;
    if (!gcond(t_.goal, empty, out_.goal)) {
      out_.goal = Condition{};
      out_.goal_unreachable = true;
    }

    // Timed events
    for (const auto & ev : t_.timed_events) {
      if (ev.fact) {
        Key k;
        for (auto o : ev.fact->second) {
          k.push_back(o.value());
        }
        uint32_t fact = prog_.find(ev.fact->first.value(), k);
        if (fact == kNone) {
          continue;  // deletes an atom that is never true
        }
        TimedEvent g;
        g.time = ev.time;
        g.kind = ev.negated ? TimedEvent::Kind::Delete : TimedEvent::Kind::Add;
        g.index = fact_atom_[fact];
        out_.timed_events.push_back(g);
      }
      if (ev.fluent) {
        Key k{ev.fluent->function.value()};
        for (auto o : ev.fluent->args) {
          k.push_back(o.value());
        }
        TimedEvent g;
        g.time = ev.time;
        if (ev.fluent->numeric) {
          g.kind = TimedEvent::Kind::Numeric;
          g.index = numeric_fluent(k);
          g.number = ev.fluent->number;
        } else {
          g.kind = TimedEvent::Kind::Object;
          g.index = object_fluent(k);
          g.object = ev.fluent->object;
        }
        out_.timed_events.push_back(g);
      }
    }

    // Metric
    out_.metric.minimize = t_.metric.minimize;
    out_.metric.is_default = t_.metric.is_default;
    if (!gexpr(t_.metric.expr, empty, out_.metric.expr)) {
      out_.metric.expr = Expr{};  // reads an undefined fluent: no metric
    }

    out_.init_numeric.assign(out_.numeric_fluents.size(), std::nullopt);
    for (size_t i = 0; i < out_.numeric_fluents.size(); ++i) {
      Key k{out_.numeric_fluents[i].function.value()};
      for (auto o : out_.numeric_fluents[i].args) {
        k.push_back(o.value());
      }
      if (auto it = init_num_.find(k); it != init_num_.end()) {
        out_.init_numeric[i] = it->second;
      }
    }
    out_.init_objects.assign(out_.object_fluents.size(), std::nullopt);
    for (size_t i = 0; i < out_.object_fluents.size(); ++i) {
      Key k{out_.object_fluents[i].function.value()};
      for (auto o : out_.object_fluents[i].args) {
        k.push_back(o.value());
      }
      if (auto it = init_obj_.find(k); it != init_obj_.end()) {
        out_.init_objects[i] = ObjectId(it->second);
      }
    }
  }

  // --- Actions that change nothing ----------------------------------------------------------

  static void reads(const Expr & e, std::vector<bool> & out)
  {
    if (e.op == ExprOp::Fluent) {
      out[e.fluent] = true;
    }
    for (const auto & c : e.children) {
      reads(c, out);
    }
  }

  static void reads(const Condition & c, std::vector<bool> & out)
  {
    for (const auto & n : c.numeric) {
      reads(n.lhs, out);
      reads(n.rhs, out);
    }
  }

  // Instant actions with no effect, or that only add to an unread (total-cost), can never help
  // a plan (as in FD)
  void drop_useless()
  {
    std::vector<bool> read(out_.numeric_fluents.size(), false);
    auto scan = [&](const std::vector<Effect> & effects) {
        for (const auto & e : effects) {
          reads(e.condition, read);
          reads(e.start_condition, read);
          reads(e.value, read);
        }
      };
    for (const auto * list : {&out_.actions, &out_.processes, &out_.events}) {
      for (const auto & a : *list) {
        reads(a.precondition, read);
        scan(a.effects);
      }
    }
    for (const auto & a : out_.durative_actions) {
      reads(a.at_start, read);
      reads(a.over_all, read);
      reads(a.at_end, read);
      scan(a.effects);
      for (const auto & d : a.duration) {
        reads(d.value, read);
      }
    }
    for (const auto & ax : out_.axioms) {
      reads(ax.body, read);
    }
    reads(out_.goal, read);
    auto cost_only = [&](const Effect & e) {
        if (e.kind != Effect::Kind::Numeric || read[e.fluent]) {
          return false;
        }
        const auto & f = out_.numeric_fluents[e.fluent];
        return f.args.empty() && out_.function_names[f.function.index()] == "total-cost";
      };
    size_t before = out_.actions.size();
    std::erase_if(
      out_.actions, [&](const Action & a) {
        return a.changes.empty() && std::all_of(a.effects.begin(), a.effects.end(), cost_only);
      });
    pruned_ += before - out_.actions.size();
  }

  // --- Fluents that never get a value --------------------------------------------------------

  static bool reads_undefined(const Expr & e, const std::vector<bool> & defined)
  {
    if (e.op == ExprOp::Fluent) {
      return !defined[e.fluent];
    }
    return std::any_of(
      e.children.begin(), e.children.end(),
      [&](const Expr & c) {return reads_undefined(c, defined);});
  }

  static bool cond_undefined(const Condition & c, const std::vector<bool> & defined)
  {
    return std::any_of(
      c.numeric.begin(), c.numeric.end(), [&](const NumericCondition & n) {
        return reads_undefined(n.lhs, defined) || reads_undefined(n.rhs, defined);
      });
  }

  // Drops effects that fail on undefined fluents; false if an unconditional one does
  static bool effects_defined(std::vector<Effect> & effects, const std::vector<bool> & defined)
  {
    bool ok = true;
    std::erase_if(
      effects, [&](const Effect & e) {
        if (e.kind != Effect::Kind::Numeric) {
          return false;
        }
        bool bad = reads_undefined(e.value, defined) ||
        (e.op != ast::AssignOp::Assign && !defined[e.fluent]);
        bool cond_bad = cond_undefined(e.condition, defined) ||
        cond_undefined(e.start_condition, defined);
        if (bad && e.condition.empty() && e.start_condition.empty()) {
          ok = false;
        }
        return bad || cond_bad;
      });
    return ok;
  }

  void prune_undefined()
  {
    if (out_.numeric_fluents.empty()) {
      return;
    }
    for (bool changed = true; changed; ) {
      changed = false;
      std::vector<bool> defined(out_.numeric_fluents.size(), false);
      for (size_t i = 0; i < defined.size(); ++i) {
        defined[i] = out_.init_numeric[i].has_value();
      }
      auto assigns = [&](const std::vector<Effect> & effects) {
          for (const auto & e : effects) {
            if (e.kind == Effect::Kind::Numeric && e.op == ast::AssignOp::Assign) {
              defined[e.fluent] = true;
            }
          }
        };
      for (const auto & a : out_.actions) {
        assigns(a.effects);
      }
      for (const auto & a : out_.durative_actions) {
        assigns(a.effects);
      }
      for (const auto & a : out_.processes) {
        assigns(a.effects);
      }
      for (const auto & a : out_.events) {
        assigns(a.effects);
      }
      for (const auto & ev : out_.timed_events) {
        if (ev.kind == TimedEvent::Kind::Numeric) {
          defined[ev.index] = true;
        }
      }
      auto prune = [&](std::vector<Action> & list) {
          size_t before = list.size();
          std::erase_if(
            list, [&](Action & a) {
              return cond_undefined(a.precondition, defined) ||
                     !effects_defined(a.effects, defined);
            });
          changed = changed || list.size() != before;
          pruned_ += before - list.size();
        };
      prune(out_.actions);
      prune(out_.processes);
      prune(out_.events);
      size_t before = out_.durative_actions.size();
      std::erase_if(
        out_.durative_actions, [&](DurativeAction & a) {
          bool dur = std::any_of(
            a.duration.begin(), a.duration.end(),
            [&](const DurationConstraint & d) {return reads_undefined(d.value, defined);});
          return dur || cond_undefined(a.at_start, defined) ||
                 cond_undefined(a.over_all, defined) || cond_undefined(a.at_end, defined) ||
                 !effects_defined(a.effects, defined);
        });
      changed = changed || out_.durative_actions.size() != before;
      pruned_ += before - out_.durative_actions.size();
      before = out_.axioms.size();
      std::erase_if(
        out_.axioms, [&](const Axiom & ax) {return cond_undefined(ax.body, defined);});
      pruned_ += before - out_.axioms.size();
      if (!changed && cond_undefined(out_.goal, defined)) {
        out_.goal = Condition{};
        out_.goal_unreachable = true;
      }
    }
  }

  const lifted::Task & t_;
  GroundOptions options_;
  Logger log_;
  Budget * budget_;
  datalog::Program prog_;
  datalog::Program * cur_ = &prog_;  // the program rules are being added to
  std::vector<bool> fluent_pred_;
  std::vector<bool> changing_fn_;
  std::unordered_map<Key, double, KeyHash> init_num_;
  std::unordered_map<Key, uint32_t, KeyHash> init_obj_;
  std::map<TypeSpec, uint32_t> type_pred_;
  std::map<uint32_t, TypeSpec> type_of_;
  std::map<uint32_t, uint32_t> objfn_pred_;
  std::map<TypeSpec, std::vector<uint32_t>> objects_;
  std::vector<uint32_t> action_inst_, process_inst_, event_inst_, axiom_inst_;
  std::vector<uint32_t> durative_start_, durative_end_;
  const std::vector<Variable> * current_vars_ = nullptr;
  Task out_;
  std::vector<uint32_t> fact_atom_;
  std::unordered_map<Key, FluentId, KeyHash> num_index_, obj_index_;
  size_t pruned_ = 0;
};

// --- Text ----------------------------------------------------------------------------------

class Printer
{
public:
  explicit Printer(const Task & t)
  : t_(t) {}

  std::string atom(AtomId a) const
  {
    const auto & info = t_.atoms.at(a);
    return call(t_.predicate_names.at(info.predicate.index()), info.args);
  }

  std::string call(const std::string & name, const std::vector<ObjectId> & args) const
  {
    std::string out = "(" + name;
    for (auto o : args) {
      out += " " + t_.object_names.at(o.index());
    }
    return out + ")";
  }

  std::string fluent(const FluentInfo & f) const
  {
    return call(t_.function_names.at(f.function.index()), f.args);
  }

  std::string expr(const Expr & e) const
  {
    switch (e.op) {
      case ExprOp::Constant: {
          std::string s = std::to_string(e.value);
          s.erase(s.find_last_not_of('0') + 1);
          if (!s.empty() && s.back() == '.') {
            s.pop_back();
          }
          return s;
        }
      case ExprOp::Fluent: return fluent(t_.numeric_fluents.at(e.fluent));
      case ExprOp::Duration: return "?duration";
      case ExprOp::TotalTime: return "(total-time)";
      case ExprOp::ContinuousT: return "#t";
      case ExprOp::StepCount: return "(plan-length)";
      case ExprOp::Violated: return "(is-violated " + e.name + ")";
      default: break;
    }
    std::string op;
    switch (e.op) {
      case ExprOp::Add: op = "+"; break;
      case ExprOp::Sub: op = "-"; break;
      case ExprOp::Mul: op = "*"; break;
      case ExprOp::Div: op = "/"; break;
      case ExprOp::Neg: op = "-"; break;
      case ExprOp::Pow: op = "^"; break;
      default: op = e.name; break;
    }
    std::string out = "(" + op;
    for (const auto & c : e.children) {
      out += " " + expr(c);
    }
    return out + ")";
  }

  std::string object(const ObjectTerm & o) const
  {
    return o.is_fluent ? fluent(t_.object_fluents.at(o.index)) :
           t_.object_names.at(o.index);
  }

  static const char * cmp(CompareOp op)
  {
    switch (op) {
      case CompareOp::Eq: return "=";
      case CompareOp::Ne: return "!=";
      case CompareOp::Lt: return "<";
      case CompareOp::Le: return "<=";
      case CompareOp::Gt: return ">";
      case CompareOp::Ge: return ">=";
    }
    return "?";
  }

  std::string condition(const Condition & c) const
  {
    std::vector<std::string> parts;
    for (AtomId a : c.positive) {
      parts.push_back(atom(a));
    }
    for (AtomId a : c.negative) {
      parts.push_back("(not " + atom(a) + ")");
    }
    for (const auto & n : c.numeric) {
      parts.push_back(std::string("(") + cmp(n.op) + " " + expr(n.lhs) + " " + expr(n.rhs) + ")");
    }
    for (const auto & o : c.objects) {
      parts.push_back(
        std::string(o.equal ? "(= " : "(!= ") + object(o.lhs) + " " + object(o.rhs) + ")");
    }
    if (parts.empty()) {
      return "()";
    }
    if (parts.size() == 1) {
      return parts[0];
    }
    std::string out = "(and";
    for (const auto & p : parts) {
      out += " " + p;
    }
    return out + ")";
  }

  std::string effect(const Effect & e) const
  {
    std::string what;
    switch (e.kind) {
      case Effect::Kind::Add: what = atom(e.atom); break;
      case Effect::Kind::Delete: what = "(not " + atom(e.atom) + ")"; break;
      case Effect::Kind::Numeric: {
          static const char * names[] = {"assign", "scale-up", "scale-down", "increase",
            "decrease"};
          what = std::string("(") + names[static_cast<int>(e.op)] + " " +
            fluent(t_.numeric_fluents.at(e.fluent)) + " " + expr(e.value) + ")";
          break;
        }
      case Effect::Kind::ObjectAssign:
        what = "(assign " + fluent(t_.object_fluents.at(e.fluent)) + " " +
          (e.object ? object(*e.object) : std::string("undefined")) + ")";
        break;
    }
    if (!e.condition.empty() || !e.start_condition.empty()) {
      std::string c = condition(e.condition);
      if (!e.start_condition.empty()) {
        c += " (at start " + condition(e.start_condition) + ")";
      }
      what = "(when " + c + " " + what + ")";
    }
    if (e.timing == Timing::Start) {
      what = "(at start " + what + ")";
    } else if (e.timing == Timing::End) {
      what = "(at end " + what + ")";
    }
    return what;
  }

  std::string effects(const std::vector<Effect> & list) const
  {
    std::string out;
    for (const auto & e : list) {
      out += "\n    " + effect(e);
    }
    return out;
  }

  std::string changes(const AtomChanges & c, const char * when = nullptr) const
  {
    std::string out;
    auto wrap = [&](const std::string & x) {
        return when ? std::string("(") + when + " " + x + ")" : x;
      };
    for (AtomId a : c.del) {
      out += "\n    " + wrap("(not " + atom(a) + ")");
    }
    for (AtomId a : c.add) {
      out += "\n    " + wrap(atom(a));
    }
    return out;
  }

  std::string task() const
  {
    std::string out = ";; ground task " + t_.domain_name + " / " + t_.problem_name + "\n";
    out += "(:atoms " + std::to_string(t_.atoms.size()) + ")\n(:init";
    for (AtomId a : t_.init_atoms) {
      out += "\n  " + atom(a);
    }
    for (size_t i = 0; i < t_.numeric_fluents.size(); ++i) {
      if (t_.init_numeric[i]) {
        Expr c;
        c.value = *t_.init_numeric[i];
        out += "\n  (= " + fluent(t_.numeric_fluents[i]) + " " + expr(c) + ")";
      }
    }
    for (size_t i = 0; i < t_.object_fluents.size(); ++i) {
      if (t_.init_objects[i]) {
        out += "\n  (= " + fluent(t_.object_fluents[i]) + " " +
          t_.object_names.at(t_.init_objects[i]->index()) + ")";
      }
    }
    out += ")\n";
    auto instants = [&](const char * kind, const std::vector<Action> & list,
      const std::vector<std::string> & names) {
        for (const auto & a : list) {
          out += std::string("(:") + kind + " " + call(names.at(a.schema), a.args) +
            "\n  :precondition " + condition(a.precondition) + "\n  :effect" +
            changes(a.changes) + effects(a.effects) + ")\n";
        }
      };
    instants("action", t_.actions, t_.action_names);
    for (const auto & a : t_.durative_actions) {
      out += "(:durative-action " + call(t_.durative_action_names.at(a.schema), a.args);
      out += "\n  :duration";
      for (const auto & d : a.duration) {
        out += std::string(" (") + (d.timing == Timing::End ? "at end " : "") + cmp(d.op) +
          " ?duration " + expr(d.value) + ")";
      }
      out += "\n  :at-start " + condition(a.at_start) + "\n  :over-all " +
        condition(a.over_all) + "\n  :at-end " + condition(a.at_end) + "\n  :effect" +
        changes(a.start, "at start") + changes(a.end, "at end") + effects(a.effects) + ")\n";
    }
    instants("process", t_.processes, t_.process_names);
    instants("event", t_.events, t_.event_names);
    for (const auto & ax : t_.axioms) {
      out += "(:derived " + atom(ax.head) + " " + condition(ax.body) + ")\n";
    }
    for (const auto & ev : t_.timed_events) {
      Expr c;
      c.value = ev.time;
      out += "(:at " + expr(c) + " ";
      switch (ev.kind) {
        case TimedEvent::Kind::Add: out += atom(ev.index); break;
        case TimedEvent::Kind::Delete: out += "(not " + atom(ev.index) + ")"; break;
        case TimedEvent::Kind::Numeric: {
            Expr v;
            v.value = ev.number;
            out += "(= " + fluent(t_.numeric_fluents.at(ev.index)) + " " + expr(v) + ")";
            break;
          }
        case TimedEvent::Kind::Object:
          out += "(= " + fluent(t_.object_fluents.at(ev.index)) + " " +
            (ev.object ? t_.object_names.at(ev.object->index()) : std::string("undefined")) +
            ")";
          break;
      }
      out += ")\n";
    }
    out += "(:goal " + (t_.goal_unreachable ? std::string("unreachable") : condition(t_.goal)) +
      ")\n";
    out += std::string("(:metric ") + (t_.metric.minimize ? "minimize " : "maximize ") +
      expr(t_.metric.expr) + (t_.metric.is_default ? " ; default" : "") + ")\n";
    return out;
  }

private:
  const Task & t_;
};

}  // namespace

Result<Task> ground(
  const lifted::Task & task, const GroundOptions & options, Budget * budget, GroundStats * stats)
{
  Grounder g(task, options, budget);
  return g.run(stats);
}

std::string instance_name(
  const Task & task, const std::string & schema, const std::vector<ObjectId> & args)
{
  return Printer(task).call(schema, args);
}

std::string instance_name(const Task & task, const Action & a)
{
  return Printer(task).call(task.action_names.at(a.schema), a.args);
}

std::string instance_name(const Task & task, const DurativeAction & a)
{
  return Printer(task).call(task.durative_action_names.at(a.schema), a.args);
}

std::string atom_name(const Task & task, AtomId atom)
{
  return Printer(task).atom(atom);
}

std::string to_string(const Task & task)
{
  return Printer(task).task();
}

}  // namespace kairos::ground
