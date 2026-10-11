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

#include "sas/translate.hpp"

#include <algorithm>
#include <chrono>
#include <map>
#include <set>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace kairos::sas
{

namespace
{

void add_terms(LinearExpr & out, const LinearExpr & x, double factor)
{
  out.constant += factor * x.constant;
  for (const auto & [v, c] : x.terms) {
    out.terms.emplace_back(v, factor * c);
  }
}

void tidy(LinearExpr & e)
{
  std::sort(e.terms.begin(), e.terms.end());
  std::vector<std::pair<NumVarId, double>> merged;
  for (const auto & [v, c] : e.terms) {
    if (!merged.empty() && merged.back().first == v) {
      merged.back().second += c;
    } else {
      merged.emplace_back(v, c);
    }
  }
  std::erase_if(merged, [](const auto & t) {return t.second == 0.0;});
  e.terms = std::move(merged);
}

struct Unstratified
{
  std::string message;
};

class Translator
{
public:
  Translator(
    const lifted::Task & lt, const ground::Task & g, const TranslateOptions & o, Budget * b,
    ground::Task * owned = nullptr)
  : lt_(lt), g_(g), options_(o), budget_(b), owned_(owned) {}

  Result<Task> run(TranslateStats * stats)
  {
    auto t0 = std::chrono::steady_clock::now();
    copy_names();
    std::vector<std::vector<ground::AtomId>> chosen;
    TranslateStats st;
    if (options_.use_invariants) {
      auto invariants = find_invariants(lt_, g_, options_.invariants, budget_, &st.invariants);
      auto groups = mutex_groups(invariants, g_);
      st.mutex_groups = groups.size();
      chosen = choose_groups(groups, g_);
    }
    if (budget_ && budget_->exhausted()) {
      return Result<Task>::failure(budget_error());
    }
    make_variables(chosen);
    st.group_variables = chosen.size();
    initial_state();
    try {
      actions();
      axioms();
    } catch (const Unstratified & u) {
      return Result<Task>::failure(Diagnostic::error("sas.unstratified", u.message));
    }
    goal_metric_events();
    for (const auto & v : out_.variables) {
      st.binary_variables += v.kind == Variable::Kind::Binary ? 1 : 0;
    }
    st.milliseconds = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - t0).count();
    options_.logger.info(
      LogChannel::Sas,
      "SAS+ in {:.1f} ms: {} variables ({} from {} mutex groups, {} binary), {} invariants "
      "from {} candidates{}, {} axiom layers",
      st.milliseconds, out_.variables.size(), st.group_variables, st.mutex_groups,
      st.binary_variables, st.invariants.invariants, st.invariants.candidates,
      st.invariants.limit_reached ? " (limit reached)" : "", out_.axiom_layers);
    if (stats) {
      *stats = st;
    }
    return Result<Task>::success(std::move(out_));
  }

private:
  Diagnostic budget_error() const
  {
    std::string why = budget_->state() == BudgetState::Timeout ? "out of time" :
      (budget_->state() == BudgetState::OutOfMemory ? "out of memory" : "cancelled");
    return Diagnostic::error("sas.budget", "translation to SAS+ stopped (" + why + ")");
  }

  void copy_names()
  {
    out_.domain_name = g_.domain_name;
    out_.problem_name = g_.problem_name;
    out_.predicate_names = g_.predicate_names;
    out_.function_names = g_.function_names;
    out_.object_names = g_.object_names;
    out_.atoms = g_.atoms;
    out_.numeric_fluents = g_.numeric_fluents;
    out_.object_fluents = g_.object_fluents;
    out_.action_names = g_.action_names;
    out_.durative_action_names = g_.durative_action_names;
    out_.process_names = g_.process_names;
    out_.event_names = g_.event_names;
    out_.features = g_.features;
    out_.has_soft_goals = g_.has_soft_goals;
    out_.has_trajectory_constraints = g_.has_trajectory_constraints;
    out_.goal_unreachable = g_.goal_unreachable;
  }

  // --- Variables -----------------------------------------------------------------------------

  void make_variables(const std::vector<std::vector<ground::AtomId>> & chosen)
  {
    out_.atom_fact.assign(g_.atoms.size(), {kNoAtom, 0});
    for (const auto & group : chosen) {
      Variable v;
      v.kind = Variable::Kind::Group;
      v.atoms = group;
      auto id = static_cast<VarId>(out_.variables.size());
      for (uint32_t i = 0; i < group.size(); ++i) {
        out_.atom_fact[group[i]] = {id, i};
      }
      out_.variables.push_back(std::move(v));
    }
    for (ground::AtomId a = 0; a < g_.atoms.size(); ++a) {
      if (out_.atom_fact[a].first != kNoAtom) {
        continue;
      }
      Variable v;
      v.kind = g_.is_derived(a) ? Variable::Kind::Derived : Variable::Kind::Binary;
      v.atoms = {a, kNoAtom};
      v.default_value = v.kind == Variable::Kind::Derived ? 1 : 0;
      out_.atom_fact[a] = {static_cast<VarId>(out_.variables.size()), 0};
      out_.variables.push_back(std::move(v));
    }
    object_variables();
  }

  // An object fluent takes its initial value, the objects effects and TIFs assign, and what
  // other fluents copied into it hold
  void object_variables()
  {
    size_t n = g_.object_fluents.size();
    std::vector<std::set<uint32_t>> domain(n);
    std::vector<std::set<ground::FluentId>> copies(n);
    for (size_t f = 0; f < n; ++f) {
      if (g_.init_objects[f]) {
        domain[f].insert(g_.init_objects[f]->value());
      }
    }
    auto scan = [&](const std::vector<ground::Effect> & effects) {
        for (const auto & e : effects) {
          if (e.kind == ground::Effect::Kind::ObjectAssign && e.object) {
            if (e.object->is_fluent) {
              copies[e.fluent].insert(e.object->index);
            } else {
              domain[e.fluent].insert(e.object->index);
            }
          }
        }
      };
    for (const auto * list : {&g_.actions, &g_.processes, &g_.events}) {
      for (const auto & a : *list) {
        scan(a.effects);
      }
    }
    for (const auto & a : g_.durative_actions) {
      scan(a.effects);
    }
    for (const auto & ev : g_.timed_events) {
      if (ev.kind == ground::TimedEvent::Kind::Object && ev.object) {
        domain[ev.index].insert(ev.object->value());
      }
    }
    for (bool changed = true; changed; ) {
      changed = false;
      for (size_t f = 0; f < n; ++f) {
        for (ground::FluentId s : copies[f]) {
          for (uint32_t o : domain[s]) {
            changed = domain[f].insert(o).second || changed;
          }
        }
      }
    }
    object_var_.resize(n);
    for (size_t f = 0; f < n; ++f) {
      Variable v;
      v.kind = Variable::Kind::ObjectFluent;
      v.fluent = static_cast<ground::FluentId>(f);
      for (uint32_t o : domain[f]) {
        v.objects.push_back(ObjectId(o));
      }
      object_var_[f] = static_cast<VarId>(out_.variables.size());
      out_.variables.push_back(std::move(v));
    }
  }

  // Value of object o in an object-fluent variable, if it can take it
  std::optional<uint32_t> object_value(VarId v, uint32_t o) const
  {
    const auto & objs = out_.variables[v].objects;
    auto it = std::lower_bound(objs.begin(), objs.end(), ObjectId(o));
    if (it == objs.end() || *it != ObjectId(o)) {
      return std::nullopt;
    }
    return static_cast<uint32_t>(it - objs.begin());
  }

  void initial_state()
  {
    out_.initial.resize(out_.variables.size());
    for (VarId v = 0; v < out_.variables.size(); ++v) {
      out_.initial[v] = out_.variables[v].none();
    }
    for (ground::AtomId a : g_.init_atoms) {
      auto [v, val] = out_.atom_fact[a];
      out_.initial[v] = val;
    }
    for (size_t f = 0; f < g_.object_fluents.size(); ++f) {
      if (g_.init_objects[f]) {
        out_.initial[object_var_[f]] = *object_value(object_var_[f], g_.init_objects[f]->value());
      }
    }
    out_.initial_numeric = g_.init_numeric;
  }

  // --- Conditions and effects ----------------------------------------------------------------

  NumericExpr expr(const ground::Expr & e) const {return NumericExpr{e, linearize(e)};}

  Fact fact_of(ground::AtomId a) const
  {
    auto [v, val] = out_.atom_fact[a];
    return Fact{v, val};
  }

  // False if the condition can never hold
  bool condition(const ground::Condition & c, Condition & out) const
  {
    for (ground::AtomId a : c.positive) {
      out.facts.push_back(fact_of(a));
    }
    for (ground::AtomId a : c.negative) {
      Fact f = fact_of(a);
      if (out_.variables[f.var].kind == Variable::Kind::Group) {
        out.not_facts.push_back(f);
      } else {
        out.facts.push_back(Fact{f.var, 1});
      }
    }
    for (const auto & n : c.numeric) {
      NumericCondition nc{n.op, expr(n.lhs), expr(n.rhs), std::nullopt};
      if (nc.lhs.linear && nc.rhs.linear) {
        LinearExpr d;
        add_terms(d, *nc.lhs.linear, 1.0);
        add_terms(d, *nc.rhs.linear, -1.0);
        tidy(d);
        nc.difference = std::move(d);
      }
      out.numeric.push_back(std::move(nc));
    }
    for (const auto & o : c.objects) {
      if (o.lhs.is_fluent && o.rhs.is_fluent) {
        out.vars.push_back(VarComparison{object_var_[o.lhs.index], object_var_[o.rhs.index],
            o.equal});
        continue;
      }
      const auto & f = o.lhs.is_fluent ? o.lhs : o.rhs;
      const auto & obj = o.lhs.is_fluent ? o.rhs : o.lhs;
      VarId v = object_var_[f.index];
      auto val = object_value(v, obj.index);
      if (o.equal) {
        if (!val) {
          return false;  // the fluent never takes that object
        }
        out.facts.push_back(Fact{v, *val});
      } else {
        if (val) {
          out.not_facts.push_back(Fact{v, *val});
        }
        out.not_facts.push_back(Fact{v, out_.variables[v].none()});  // undefined compares false
      }
    }
    std::sort(out.facts.begin(), out.facts.end());
    out.facts.erase(std::unique(out.facts.begin(), out.facts.end()), out.facts.end());
    std::sort(out.not_facts.begin(), out.not_facts.end());
    out.not_facts.erase(
      std::unique(out.not_facts.begin(), out.not_facts.end()), out.not_facts.end());
    // Two values of one variable, or a value and its negation
    for (size_t i = 1; i < out.facts.size(); ++i) {
      if (out.facts[i].var == out.facts[i - 1].var) {
        return false;
      }
    }
    for (const auto & nf : out.not_facts) {
      if (std::binary_search(out.facts.begin(), out.facts.end(), nf)) {
        return false;
      }
    }
    return true;
  }

  static bool holds(const std::vector<Fact> & facts, const Fact & f)
  {
    return std::find(facts.begin(), facts.end(), f) != facts.end();
  }

  // Unconditional atom changes: deletes of group atoms not known to be true become conditional
  void changes(
    const ground::AtomChanges & c, const std::vector<Fact> & known, Timing timing,
    std::vector<Fact> & out, std::vector<Effect> & effects) const
  {
    std::set<VarId> added;
    for (ground::AtomId a : c.add) {
      Fact f = fact_of(a);
      out.push_back(f);
      added.insert(f.var);
    }
    for (ground::AtomId d : c.del) {
      Fact f = fact_of(d);
      const Variable & v = out_.variables[f.var];
      if (added.count(f.var)) {
        continue;  // another value of the variable is set: the delete is implied
      }
      Fact none{f.var, v.none()};
      if (v.kind != Variable::Kind::Group || holds(known, f)) {
        out.push_back(none);
        continue;
      }
      Effect e;
      e.timing = timing;
      e.condition.facts.push_back(f);
      e.fact = none;
      effects.push_back(std::move(e));
    }
  }

  void effects(
    const std::vector<ground::Effect> & list, const std::vector<Fact> & known,
    const std::set<VarId> & set_unconditionally, std::vector<Effect> & out) const
  {
    for (const auto & ge : list) {
      Effect e;
      e.timing = ge.timing;
      e.continuous = ge.continuous;
      if (!condition(ge.condition, e.condition) ||
        !condition(ge.start_condition, e.start_condition))
      {
        continue;  // never fires
      }
      switch (ge.kind) {
        case ground::Effect::Kind::Add:
          e.fact = fact_of(ge.atom);
          break;
        case ground::Effect::Kind::Delete: {
            Fact f = fact_of(ge.atom);
            const Variable & v = out_.variables[f.var];
            if (v.kind == Variable::Kind::Group && set_unconditionally.count(f.var)) {
              continue;  // an unconditional add of another value wins anyway
            }
            if (v.kind == Variable::Kind::Group && !holds(known, f) &&
              !holds(e.condition.facts, f))
            {
              e.condition.facts.push_back(f);
              std::sort(e.condition.facts.begin(), e.condition.facts.end());
            }
            e.fact = Fact{f.var, v.none()};
            break;
          }
        case ground::Effect::Kind::Numeric:
          e.kind = Effect::Kind::Numeric;
          e.num = ge.fluent;
          e.op = ge.op;
          e.value = expr(ge.value);
          break;
        case ground::Effect::Kind::ObjectAssign: {
            VarId v = object_var_[ge.fluent];
            if (ge.object && ge.object->is_fluent) {
              e.kind = Effect::Kind::Copy;
              e.target = v;
              e.source = object_var_[ge.object->index];
            } else {
              e.fact = Fact{v, ge.object ? *object_value(v, ge.object->index) :
                out_.variables[v].none()};
            }
            break;
          }
      }
      out.push_back(std::move(e));
    }
  }

  static std::set<VarId> vars_of(const std::vector<Fact> & facts, const Task & t)
  {
    std::set<VarId> out;
    for (const auto & f : facts) {
      if (f.value != t.variables[f.var].none()) {
        out.insert(f.var);
      }
    }
    return out;
  }

  bool instant(const ground::Action & ga, Action & a)
  {
    a.schema = ga.schema;
    a.args = ga.args;
    if (!condition(ga.precondition, a.pre)) {
      return false;
    }
    changes(ga.changes, a.pre.facts, Timing::Instant, a.changes, a.effects);
    effects(ga.effects, a.pre.facts, vars_of(a.changes, out_), a.effects);
    return true;
  }

  static bool reads_duration(const ground::Expr & e)
  {
    return e.op == ExprOp::Duration || std::any_of(
      e.children.begin(), e.children.end(), [](const ground::Expr & c) {return reads_duration(c);});
  }

  void actions()
  {
    auto convert = [&](const std::vector<ground::Action> & in, std::vector<Action> & out,
      std::vector<ground::Action> * release) {
        out.reserve(in.size());
        for (size_t i = 0; i < in.size(); ++i) {
          if (budget_ && budget_->exhausted()) {
            return;
          }
          Action a;
          if (instant(in[i], a)) {
            out.push_back(std::move(a));
          }
          if (release) {
            (*release)[i] = ground::Action{};  // frees its conditions and effects
          }
        }
      };
    convert(g_.actions, out_.actions, owned_ ? &owned_->actions : nullptr);
    convert(g_.processes, out_.processes, owned_ ? &owned_->processes : nullptr);
    convert(g_.events, out_.events, owned_ ? &owned_->events : nullptr);
    out_.durative_actions.reserve(g_.durative_actions.size());
    for (size_t i = 0; i < g_.durative_actions.size(); ++i) {
      if (budget_ && budget_->exhausted()) {
        return;
      }
      const auto & gd = g_.durative_actions[i];
      durative(gd);
      if (owned_) {
        owned_->durative_actions[i] = ground::DurativeAction{};
      }
    }
  }

  void durative(const ground::DurativeAction & gd)
  {
    {
      DurativeAction d;
      d.schema = gd.schema;
      d.args = gd.args;
      if (!condition(gd.at_start, d.at_start) || !condition(gd.over_all, d.over_all) ||
        !condition(gd.at_end, d.at_end))
      {
        return;
      }
      for (const auto & c : gd.duration) {
        d.duration.push_back(DurationConstraint{c.op, c.timing, expr(c.value)});
      }
      std::vector<Fact> at_end = d.at_end.facts;
      at_end.insert(at_end.end(), d.over_all.facts.begin(), d.over_all.facts.end());
      changes(gd.start, d.at_start.facts, Timing::Start, d.start, d.effects);
      changes(gd.end, at_end, Timing::End, d.end, d.effects);
      std::vector<Effect> rest;
      std::set<VarId> start_set = vars_of(d.start, out_), end_set = vars_of(d.end, out_);
      for (const auto & ge : gd.effects) {
        const auto & known = ge.timing == Timing::End ? at_end : d.at_start.facts;
        effects({ge}, known, ge.timing == Timing::End ? end_set : start_set, d.effects);
        d.uses_duration = d.uses_duration || reads_duration(ge.value);
      }
      for (const auto & c : gd.duration) {
        d.uses_duration = d.uses_duration || reads_duration(c.value);
      }
      out_.durative_actions.push_back(std::move(d));
    }
  }

  // --- Axioms --------------------------------------------------------------------------------

  void axioms()
  {
    std::vector<Axiom> list;
    for (const auto & ga : g_.axioms) {
      Axiom ax;
      if (!condition(ga.body, ax.body)) {
        continue;
      }
      ax.head = Fact{out_.atom_fact[ga.head].first, 0};
      list.push_back(std::move(ax));
    }
    // Layers: a derived variable read negatively must be complete before
    size_t n = out_.variables.size();
    std::vector<int> layer(n, 0);
    auto is_derived = [&](VarId v) {return out_.variables[v].kind == Variable::Kind::Derived;};
    size_t derived = 0;
    for (VarId v = 0; v < n; ++v) {
      derived += is_derived(v) ? 1u : 0u;
    }
    for (bool changed = true; changed; ) {
      changed = false;
      for (const auto & ax : list) {
        int & h = layer[ax.head.var];
        auto need = [&](VarId v, int extra) {
            if (!is_derived(v)) {
              return;
            }
            int want = layer[v] + extra;
            if (want > h) {
              if (want > static_cast<int>(derived)) {
                throw Unstratified{"derived predicate " +
                        out_.predicate_names[out_.atoms[out_.variables[ax.head.var].atoms[0]]
                          .predicate.index()] + " depends on itself through a negation"};
              }
              h = want;
              changed = true;
            }
          };
        for (const auto & f : ax.body.facts) {
          need(f.var, f.value == 0 ? 0 : 1);
        }
        for (const auto & f : ax.body.not_facts) {
          need(f.var, 1);
        }
      }
    }
    int top = -1;
    for (VarId v = 0; v < n; ++v) {
      if (is_derived(v)) {
        out_.variables[v].layer = layer[v];
        top = std::max(top, layer[v]);
      }
    }
    out_.axiom_layers = top + 1;
    std::stable_sort(
      list.begin(), list.end(), [&](const Axiom & a, const Axiom & b) {
        return layer[a.head.var] < layer[b.head.var];
      });
    out_.axioms = std::move(list);
  }

  // --- The rest ------------------------------------------------------------------------------

  void goal_metric_events()
  {
    if (!out_.goal_unreachable && !condition(g_.goal, out_.goal)) {
      out_.goal = Condition{};
      out_.goal_unreachable = true;
    }
    out_.metric.minimize = g_.metric.minimize;
    out_.metric.is_default = g_.metric.is_default;
    out_.metric.expr = expr(g_.metric.expr);
    for (const auto & ev : g_.timed_events) {
      TimedEvent t;
      t.time = ev.time;
      Effect e;
      switch (ev.kind) {
        case ground::TimedEvent::Kind::Add:
          e.fact = fact_of(ev.index);
          break;
        case ground::TimedEvent::Kind::Delete: {
            Fact f = fact_of(ev.index);
            const Variable & v = out_.variables[f.var];
            if (v.kind == Variable::Kind::Group) {
              e.condition.facts.push_back(f);
            }
            e.fact = Fact{f.var, v.none()};
            break;
          }
        case ground::TimedEvent::Kind::Numeric:
          e.kind = Effect::Kind::Numeric;
          e.num = ev.index;
          e.value.tree.value = ev.number;
          e.value.linear = LinearExpr{{}, ev.number};
          break;
        case ground::TimedEvent::Kind::Object: {
            VarId v = object_var_[ev.index];
            e.fact = Fact{v, ev.object ? *object_value(v, ev.object->value()) :
              out_.variables[v].none()};
            break;
          }
      }
      t.effects.push_back(std::move(e));
      out_.timed_events.push_back(std::move(t));
    }
  }

  const lifted::Task & lt_;
  const ground::Task & g_;
  TranslateOptions options_;
  Budget * budget_;
  ground::Task * owned_;  // released as it is translated, if given
  Task out_;
  std::vector<VarId> object_var_;  // per object fluent
};

// --- Text ------------------------------------------------------------------------------------

class Printer
{
public:
  explicit Printer(const Task & t)
  : t_(t) {}

  std::string call(const std::string & name, const std::vector<ObjectId> & args) const
  {
    std::string out = "(" + name;
    for (auto o : args) {
      out += " " + t_.object_names.at(o.index());
    }
    return out + ")";
  }

  std::string atom(ground::AtomId a) const
  {
    const auto & info = t_.atoms.at(a);
    return call(t_.predicate_names.at(info.predicate.index()), info.args);
  }

  std::string fact(const Fact & f) const
  {
    const Variable & v = t_.variables.at(f.var);
    switch (v.kind) {
      case Variable::Kind::Group:
        return f.value < v.atoms.size() ? atom(v.atoms[f.value]) :
               "(none-of v" + std::to_string(f.var) + ")";
      case Variable::Kind::Binary:
      case Variable::Kind::Derived:
        return f.value == 0 ? atom(v.atoms[0]) : "(not " + atom(v.atoms[0]) + ")";
      case Variable::Kind::ObjectFluent: {
          const auto & fl = t_.object_fluents.at(v.fluent);
          std::string name = call(t_.function_names.at(fl.function.index()), fl.args);
          return "(= " + name + " " + (f.value < v.objects.size() ?
                 t_.object_names.at(v.objects[f.value].index()) : std::string("undefined")) + ")";
        }
    }
    return "?";
  }

  std::string number(double x) const
  {
    std::string s = std::to_string(x);
    s.erase(s.find_last_not_of('0') + 1);
    if (!s.empty() && s.back() == '.') {
      s.pop_back();
    }
    return s;
  }

  std::string linear(const LinearExpr & e) const
  {
    std::string out;
    for (const auto & [v, c] : e.terms) {
      const auto & f = t_.numeric_fluents.at(v);
      out += (out.empty() ? "" : " + ") + number(c) + "*" +
        call(t_.function_names.at(f.function.index()), f.args);
    }
    if (out.empty() || e.constant != 0.0) {
      out += (out.empty() ? "" : " + ") + number(e.constant);
    }
    return out;
  }

  std::string expr(const NumericExpr & e) const
  {
    return e.linear ? "[" + linear(*e.linear) + "]" : "<non-linear>";
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
    for (const auto & f : c.facts) {
      parts.push_back(fact(f));
    }
    for (const auto & f : c.not_facts) {
      parts.push_back("(not " + fact(f) + ")");
    }
    for (const auto & n : c.numeric) {
      parts.push_back(
        std::string("(") + cmp(n.op) + " " + expr(n.lhs) + " " + expr(n.rhs) + ")");
    }
    for (const auto & v : c.vars) {
      parts.push_back(
        std::string(v.equal ? "(= v" : "(!= v") + std::to_string(v.a) + " v" +
        std::to_string(v.b) + ")");
    }
    if (parts.empty()) {
      return "()";
    }
    std::string out = parts.size() == 1 ? parts[0] : "(and";
    if (parts.size() > 1) {
      for (const auto & p : parts) {
        out += " " + p;
      }
      out += ")";
    }
    return out;
  }

  std::string effect(const Effect & e) const
  {
    std::string what;
    switch (e.kind) {
      case Effect::Kind::Assign: what = "v" + std::to_string(e.fact.var) + " := " + fact(e.fact);
        break;
      case Effect::Kind::Numeric: {
          const auto & f = t_.numeric_fluents.at(e.num);
          static const char * names[] = {"assign", "scale-up", "scale-down", "increase",
            "decrease"};
          what = std::string("(") + names[static_cast<int>(e.op)] + " " +
            call(t_.function_names.at(f.function.index()), f.args) + " " + expr(e.value) + ")";
          break;
        }
      case Effect::Kind::Copy:
        what = "v" + std::to_string(e.target) + " := v" + std::to_string(e.source);
        break;
    }
    if (!e.condition.empty()) {
      what = "(when " + condition(e.condition) + " " + what + ")";
    }
    if (e.timing == Timing::Start) {
      what = "(at start " + what + ")";
    } else if (e.timing == Timing::End) {
      what = "(at end " + what + ")";
    }
    return what;
  }

  std::string changes(const std::vector<Fact> & c, const char * when = nullptr) const
  {
    std::string out;
    for (const auto & f : c) {
      std::string s = "v" + std::to_string(f.var) + " := " + fact(f);
      out += "\n    " + (when ? std::string("(") + when + " " + s + ")" : s);
    }
    return out;
  }

  std::string task() const
  {
    std::string out = ";; SAS+ task " + t_.domain_name + " / " + t_.problem_name + "\n";
    for (VarId v = 0; v < t_.variables.size(); ++v) {
      const Variable & var = t_.variables[v];
      static const char * kinds[] = {"group", "binary", "derived", "object"};
      out += "(:variable v" + std::to_string(v) + " " + kinds[static_cast<int>(var.kind)];
      if (var.kind == Variable::Kind::Derived) {
        out += " layer " + std::to_string(var.layer);
      }
      for (uint32_t i = 0; i < var.size(); ++i) {
        out += (i ? " | " : " ") + fact(Fact{v, i});
      }
      out += ")\n";
    }
    out += "(:init";
    for (VarId v = 0; v < t_.variables.size(); ++v) {
      if (t_.variables[v].kind != Variable::Kind::Derived) {
        out += " " + fact(Fact{v, t_.initial[v]});
      }
    }
    out += ")\n";
    auto instants = [&](const char * kind, const std::vector<Action> & list,
      const std::vector<std::string> & names) {
        for (const auto & a : list) {
          out += std::string("(:") + kind + " " + call(names.at(a.schema), a.args) +
            "\n  :precondition " + condition(a.pre) + "\n  :effect" + changes(a.changes);
          for (const auto & e : a.effects) {
            out += "\n    " + effect(e);
          }
          out += ")\n";
        }
      };
    instants("action", t_.actions, t_.action_names);
    for (const auto & a : t_.durative_actions) {
      out += "(:durative-action " + call(t_.durative_action_names.at(a.schema), a.args) +
        "\n  :duration";
      for (const auto & d : a.duration) {
        out += std::string(" (") + cmp(d.op) + " ?duration " + expr(d.value) + ")";
      }
      out += "\n  :at-start " + condition(a.at_start) + "\n  :over-all " +
        condition(a.over_all) + "\n  :at-end " + condition(a.at_end) + "\n  :effect" +
        changes(a.start, "at start") + changes(a.end, "at end");
      for (const auto & e : a.effects) {
        out += "\n    " + effect(e);
      }
      out += ")\n";
    }
    instants("process", t_.processes, t_.process_names);
    instants("event", t_.events, t_.event_names);
    for (const auto & ax : t_.axioms) {
      out += "(:axiom " + fact(ax.head) + " " + condition(ax.body) + ")\n";
    }
    out += "(:goal " + (t_.goal_unreachable ? std::string("unreachable") :
      condition(t_.goal)) + ")\n";
    return out;
  }

private:
  const Task & t_;
};

}  // namespace

std::optional<LinearExpr> linearize(const ground::Expr & e)
{
  LinearExpr out;
  switch (e.op) {
    case ExprOp::Constant:
      out.constant = e.value;
      return out;
    case ExprOp::Fluent:
      out.terms.emplace_back(e.fluent, 1.0);
      return out;
    case ExprOp::Add:
    case ExprOp::Sub:
    case ExprOp::Neg: {
        for (size_t i = 0; i < e.children.size(); ++i) {
          auto c = linearize(e.children[i]);
          if (!c) {
            return std::nullopt;
          }
          bool minus = e.op == ExprOp::Neg || (e.op == ExprOp::Sub && (i > 0 ||
            e.children.size() == 1));
          add_terms(out, *c, minus ? -1.0 : 1.0);
        }
        tidy(out);
        return out;
      }
    case ExprOp::Mul: {
        // A product is linear when every factor but one is constant
        double factor = 1.0;
        std::optional<LinearExpr> variable;
        for (const auto & ch : e.children) {
          auto c = linearize(ch);
          if (!c) {
            return std::nullopt;
          }
          if (c->terms.empty()) {
            factor *= c->constant;
          } else if (variable) {
            return std::nullopt;
          } else {
            variable = std::move(c);
          }
        }
        if (variable) {
          add_terms(out, *variable, factor);
        } else {
          out.constant = factor;
        }
        tidy(out);
        return out;
      }
    case ExprOp::Div: {
        if (e.children.size() != 2) {
          return std::nullopt;
        }
        auto a = linearize(e.children[0]);
        auto b = linearize(e.children[1]);
        if (!a || !b || !b->terms.empty() || b->constant == 0.0) {
          return std::nullopt;
        }
        add_terms(out, *a, 1.0 / b->constant);
        tidy(out);
        return out;
      }
    default:
      return std::nullopt;  // ?duration, total-time, #t, powers, math functions
  }
}

Result<Task> translate(
  const lifted::Task & lifted, const ground::Task & ground, const TranslateOptions & options,
  Budget * budget, TranslateStats * stats)
{
  Translator t(lifted, ground, options, budget);
  auto r = t.run(stats);
  if (r.ok() && budget && budget->exhausted()) {
    std::string why = budget->state() == BudgetState::Timeout ? "out of time" :
      (budget->state() == BudgetState::OutOfMemory ? "out of memory" : "cancelled");
    return Result<Task>::failure(
      Diagnostic::error("sas.budget", "translation to SAS+ stopped (" + why + ")"));
  }
  return r;
}

Result<Task> translate(
  const lifted::Task & lifted, ground::Task && ground, const TranslateOptions & options,
  Budget * budget, TranslateStats * stats)
{
  Translator t(lifted, ground, options, budget, &ground);
  auto r = t.run(stats);
  ground = ground::Task{};
  if (r.ok() && budget && budget->exhausted()) {
    return Result<Task>::failure(
      Diagnostic::error("sas.budget", "translation to SAS+ stopped"));
  }
  return r;
}

std::string fact_name(const Task & task, const Fact & fact)
{
  return Printer(task).fact(fact);
}

std::string instance_name(const Task & task, const Action & a)
{
  return Printer(task).call(task.action_names.at(a.schema), a.args);
}

std::string instance_name(const Task & task, const DurativeAction & a)
{
  return Printer(task).call(task.durative_action_names.at(a.schema), a.args);
}

std::string to_string(const Task & task)
{
  return Printer(task).task();
}

}  // namespace kairos::sas
