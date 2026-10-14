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

#include "semantic/analyzer.hpp"

#include <algorithm>
#include <functional>
#include <map>
#include <set>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "core/suggest.hpp"

namespace kairos::sem
{

namespace
{

using ast::EffectId;
using ast::ExprId;
using ast::FormulaId;
using ast::TermId;

struct Var
{
  std::string name;
  TypeSpec types;
};

// What is allowed at the current point of a formula, effect or expression
struct Ctx
{
  bool timed = false;         // at start / at end / over all may appear here
  bool need_time = false;     // durative top level: everything must be wrapped in time
  bool constraints = false;   // modal operators allowed
  bool duration_var = false;  // ?duration allowed
  bool metric = false;        // total-time and is-violated allowed
  bool continuous = false;    // #t allowed
  bool preferences = true;
  bool in_duration = false;   // inside the :duration constraint
};

struct ExprInfo
{
  enum class Kind {Numeric, Object, Invalid};
  Kind kind = Kind::Invalid;
  TypeSpec types;         // Object
  bool changing = false;  // depends on changing fluents, ?duration or total-time
  bool has_t = false;
  bool functions = false;  // uses any function
};

// Checks formulas, effects and expressions of one file (domain or problem)
class Checker
{
public:
  Checker(
    const ast::Nodes & nodes, const SymbolTable & symbols, std::string file,
    std::vector<Diagnostic> & diags, const TypeHierarchy & types, FeatureSet & features,
    const Table<PredicateId, PredicateInfo> & predicates,
    const Table<FunctionId, FunctionInfo> & functions,
    const Table<ObjectId, ObjectInfo> & objects, std::set<std::string> & preferences)
  : nodes_(nodes), symbols_(symbols), file_(std::move(file)), diags_(diags), types_(types),
    features_(features), predicates_(predicates), functions_(functions), objects_(objects),
    preferences_(preferences) {}

  std::vector<Var> scope;
  std::string context;  // e.g. "action 'move'", appended to messages
  // Negations, disjunctions and quantifiers as written, for :requirements: features_ has what
  // they mean once negations are pushed in ((not (exists ...)) is universal), for the engines
  FeatureSet written;

  std::string str(Symbol s) const {return std::string(symbols_.name(s));}

  std::string in_context(const std::string & msg) const
  {
    return context.empty() ? msg : "in " + context + ": " + msg;
  }

  void error(const std::string & code, const std::string & msg, ast::Loc loc)
  {
    diags_.push_back(Diagnostic::error(code, in_context(msg), where(loc)));
  }

  void note(const std::string & code, const std::string & msg, ast::Loc loc)
  {
    diags_.push_back(Diagnostic::note(code, msg, where(loc)));
  }

  template<class IdT, class Info>
  static std::vector<std::string> names_of(const Table<IdT, Info> & table)
  {
    std::vector<std::string> out;
    out.reserve(table.items.size());
    for (const auto & i : table.items) {
      out.push_back(i.name);
    }
    return out;
  }

  std::vector<std::string> type_names() const
  {
    std::vector<std::string> out;
    for (size_t i = 0; i < types_.size(); ++i) {
      out.push_back(types_[TypeId(static_cast<uint32_t>(i))].name);
    }
    return out;
  }

  void warning(const std::string & code, const std::string & msg, ast::Loc loc)
  {
    diags_.push_back(Diagnostic::warning(code, in_context(msg), where(loc)));
  }

  SourceLocation where(ast::Loc loc) const {return SourceLocation{file_, loc.line, loc.column};}

  // --- Types and parameters ------------------------------------------------------------------

  std::optional<TypeSpec> resolve(const std::vector<Symbol> & names, ast::Loc loc)
  {
    if (names.empty()) {
      return TypeSpec{types_.object()};
    }
    TypeSpec spec;
    for (Symbol s : names) {
      if (auto t = types_.find(str(s))) {
        spec.push_back(*t);
      } else {
        error("semantic.unknown_type",
          "unknown type '" + str(s) + "'" + did_you_mean(str(s), type_names()), loc);
        return std::nullopt;
      }
    }
    return spec;
  }

  // Typed variables; duplicates are errors. Unknown types fall back to object.
  std::vector<Var> variables(const std::vector<ast::TypedName> & params)
  {
    std::vector<Var> out;
    std::set<std::string> seen;
    for (const auto & p : params) {
      std::string n = str(p.name);
      if (!seen.insert(n).second) {
        error("semantic.duplicate_parameter", "parameter " + n + " declared twice", p.loc);
      }
      if (!p.types.empty() && !(p.types.size() == 1 && str(p.types[0]) == "object")) {
        features_.add(Feature::Typing);
      }
      out.push_back(Var{n, resolve(p.types, type_loc(p)).value_or(TypeSpec{})});
    }
    return out;
  }

  static ast::Loc type_loc(const ast::TypedName & p)
  {
    return p.type_loc.line != 0 ? p.type_loc : p.loc;
  }

  const Var * lookup_var(const std::string & name) const
  {
    for (auto it = scope.rbegin(); it != scope.rend(); ++it) {
      if (it->name == name) {
        return &*it;
      }
    }
    return nullptr;
  }

  // RAII scope extension for quantifiers and action parameters
  class Scoped
  {
public:
    Scoped(Checker & c, std::vector<Var> vars)
    : c_(c), n_(vars.size())
    {
      c_.scope.insert(c_.scope.end(), vars.begin(), vars.end());
    }
    ~Scoped() {c_.scope.resize(c_.scope.size() - n_);}
    Scoped(const Scoped &) = delete;
    Scoped & operator=(const Scoped &) = delete;

private:
    Checker & c_;
    size_t n_;
  };

  // --- Terms ---------------------------------------------------------------------------------

  // Types of a term, or nullopt after reporting an error
  std::optional<TypeSpec> term(TermId id)
  {
    const ast::Term & t = nodes_.terms[id];
    std::string n = str(t.name);
    switch (t.kind) {
      case ast::TermKind::Variable: {
          const Var * v = lookup_var(n);
          if (v == nullptr) {
            std::vector<std::string> bound;
            for (const auto & bound_var : scope) {
              bound.push_back(bound_var.name);
            }
            error("semantic.unbound_variable",
              "variable " + n + " is not a parameter or quantified variable" +
              did_you_mean(n, bound), t.loc);
            return std::nullopt;
          }
          return v->types;
        }
      case ast::TermKind::Name: {
          auto o = objects_.find(n);
          if (!o) {
            unknown_object(n, t.loc);
            return std::nullopt;
          }
          return objects_.items[*o].types;
        }
      case ast::TermKind::Function: {
          auto f = functions_.find(n);
          if (!f) {
            error("semantic.unknown_function",
              "unknown function '" + n + "'" + did_you_mean(n, names_of(functions_)), t.loc);
            return std::nullopt;
          }
          const FunctionInfo & info = functions_.items[*f];
          check_args(info.name, info.params, t.args, t.loc);
          if (info.numeric) {
            error("semantic.type_mismatch",
              "numeric function '" + n + "' used where an object is expected", t.loc);
            return std::nullopt;
          }
          features_.add(Feature::ObjectFluents);
          return info.result;
        }
    }
    return std::nullopt;
  }

  void unknown_object(const std::string & n, ast::Loc loc)
  {
    if (objects_alone_) {
      if (warned_names_.insert(n).second) {
        warning("semantic.unknown_symbol_without_domain",
          "'" + n + "' is not in :objects (it may be a domain constant)", loc);
      }
      return;
    }
    error("semantic.unknown_object",
      "unknown object or constant '" + n + "'" + did_you_mean(n, names_of(objects_)), loc);
  }

  void check_args(
    const std::string & head, const std::vector<TypeSpec> & params,
    const std::vector<TermId> & args, ast::Loc loc)
  {
    if (params.size() != args.size()) {
      error("semantic.wrong_arity",
        "'" + head + "' takes " + std::to_string(params.size()) + " argument(s), " +
        std::to_string(args.size()) + " given", loc);
      return;
    }
    std::vector<std::optional<TypeSpec>> specs;
    std::vector<size_t> wrong;
    for (size_t i = 0; i < args.size(); ++i) {
      const ast::Term & t = nodes_.terms[args[i]];
      auto spec = term(args[i]);
      specs.push_back(spec);
      if (!spec) {
        continue;
      }
      if (!accepts(t, *spec, params[i])) {
        wrong.push_back(i);
        error("semantic.type_mismatch",
          "argument " + std::to_string(i + 1) + " of '" + head + "' must be " +
          types_.to_string(params[i]) + ", '" + str(t.name) + "' is " + types_.to_string(*spec),
          t.loc);
      }
    }
    // Two wrong arguments that would fit if exchanged: say so
    if (wrong.size() == 2) {
      size_t a = wrong[0];
      size_t b = wrong[1];
      const ast::Term & ta = nodes_.terms[args[a]];
      const ast::Term & tb = nodes_.terms[args[b]];
      if (accepts(ta, *specs[a], params[b]) && accepts(tb, *specs[b], params[a])) {
        note("semantic.swapped_arguments",
          "arguments " + std::to_string(a + 1) + " and " + std::to_string(b + 1) + " of '" +
          head + "' seem to be swapped", loc);
      }
    }
  }

  // Objects must be of the type; variables only need to be able to be
  bool accepts(const ast::Term & t, const TypeSpec & spec, const TypeSpec & required) const
  {
    return t.kind == ast::TermKind::Variable ? types_.compatible(spec, required) :
           types_.fits(spec, required);
  }

  // --- Expressions ---------------------------------------------------------------------------

  ExprInfo expr(ExprId id, const Ctx & c)
  {
    const ast::Expr & e = nodes_.exprs[id];
    ExprInfo r;
    switch (e.kind) {
      case ast::ExprKind::Number:
        r.kind = ExprInfo::Kind::Numeric;
        return r;
      case ast::ExprKind::Term: {
          auto spec = term(e.term);
          if (spec) {
            r.kind = ExprInfo::Kind::Object;
            r.types = *spec;
          }
          return r;
        }
      case ast::ExprKind::Function: {
          std::string n = str(e.name);
          auto f = functions_.find(n);
          if (!f) {
            error("semantic.unknown_function",
              "unknown function '" + n + "'" + did_you_mean(n, names_of(functions_)), e.loc);
            return r;
          }
          const FunctionInfo & info = functions_.items[*f];
          check_args(info.name, info.params, e.args, e.loc);
          r.functions = true;
          r.changing = info.changes;
          if (info.numeric) {
            features_.add(Feature::NumericFluents);
            r.kind = ExprInfo::Kind::Numeric;
          } else {
            features_.add(Feature::ObjectFluents);
            r.kind = ExprInfo::Kind::Object;
            r.types = info.result;
          }
          return r;
        }
      case ast::ExprKind::Duration:
        if (!c.duration_var) {
          error("semantic.misplaced_duration",
            "?duration can only appear in durative actions", e.loc);
          return r;
        }
        if (!c.in_duration) {
          features_.add(Feature::DurationInEffects);
        }
        r.kind = ExprInfo::Kind::Numeric;
        r.changing = true;
        return r;
      case ast::ExprKind::TotalTime:
        if (!c.metric) {
          error("semantic.misplaced_total_time", "total-time can only appear in :metric", e.loc);
          return r;
        }
        features_.add(Feature::MetricTotalTime);
        r.kind = ExprInfo::Kind::Numeric;
        r.changing = true;
        return r;
      case ast::ExprKind::ContinuousT:
        if (!c.continuous) {
          error("semantic.misplaced_continuous",
            "#t can only appear in increase/decrease effects of durative actions and processes",
            e.loc);
          return r;
        }
        features_.add(Feature::ContinuousEffects);
        r.kind = ExprInfo::Kind::Numeric;
        r.has_t = true;
        return r;
      case ast::ExprKind::IsViolated:
        if (!c.metric) {
          error("semantic.misplaced_is_violated", "is-violated can only appear in :metric", e.loc);
          return r;
        }
        violated_.emplace_back(str(e.name), e.loc);
        r.kind = ExprInfo::Kind::Numeric;
        return r;
      case ast::ExprKind::Add:
      case ast::ExprKind::Sub:
      case ast::ExprKind::Neg:
      case ast::ExprKind::Mul:
      case ast::ExprKind::Div:
      case ast::ExprKind::Pow:
        return arithmetic(e, c);
      case ast::ExprKind::Math: {
          r = arithmetic(e, c);
          features_.add(Feature::NonLinearNumeric);
          return r;
        }
    }
    return r;
  }

  ExprInfo arithmetic(const ast::Expr & e, const Ctx & c)
  {
    ExprInfo r;
    r.kind = ExprInfo::Kind::Numeric;
    int changing_factors = 0;
    bool valid = true;
    std::vector<ExprInfo> parts;
    for (ExprId child : e.children) {
      ExprInfo ci = expr(child, c);
      if (ci.kind == ExprInfo::Kind::Object) {
        error("semantic.type_mismatch", "object used in an arithmetic expression",
          nodes_.exprs[child].loc);
        valid = false;
      } else if (ci.kind == ExprInfo::Kind::Invalid) {
        valid = false;
      }
      r.changing = r.changing || ci.changing;
      r.has_t = r.has_t || ci.has_t;
      r.functions = r.functions || ci.functions;
      changing_factors += ci.changing ? 1 : 0;
      parts.push_back(ci);
    }
    // Non-linear: a product of two changing factors, a division by a changing value, a power
    bool non_linear = false;
    if (e.kind == ast::ExprKind::Mul) {
      non_linear = changing_factors >= 2;
    } else if (e.kind == ast::ExprKind::Div && parts.size() == 2) {
      non_linear = parts[1].changing;
    } else if (e.kind == ast::ExprKind::Pow) {
      non_linear = r.changing;
    }
    if (non_linear) {
      features_.add(Feature::NonLinearNumeric);
    }
    if (!valid) {
      r.kind = ExprInfo::Kind::Invalid;
    }
    return r;
  }

  void numeric(ExprId id, const Ctx & c)
  {
    ExprInfo i = expr(id, c);
    if (i.kind == ExprInfo::Kind::Object) {
      error("semantic.type_mismatch", "object used where a number is expected",
        nodes_.exprs[id].loc);
    }
  }

  // --- Formulas ------------------------------------------------------------------------------

  void formula(FormulaId id, Ctx c, bool positive)
  {
    const ast::Formula & f = nodes_.formulas[id];
    switch (f.kind) {
      case ast::FormulaKind::And:
        if (!positive && f.children.size() > 1) {
          features_.add(Feature::DisjunctivePreconditions);
        }
        for (FormulaId ch : f.children) {
          formula(ch, c, positive);
        }
        return;
      case ast::FormulaKind::Or:
        if (positive && f.children.size() > 1) {
          features_.add(Feature::DisjunctivePreconditions);
        }
        if (f.children.size() > 1) {
          written.add(Feature::DisjunctivePreconditions);
        }
        for (FormulaId ch : f.children) {
          formula(ch, untimed_child(c, f), positive);
        }
        return;
      case ast::FormulaKind::Not:
        written.add(Feature::NegativePreconditions);
        formula(f.children.at(0), untimed_child(c, f), !positive);
        return;
      case ast::FormulaKind::Imply:
        features_.add(Feature::DisjunctivePreconditions);
        written.add(Feature::DisjunctivePreconditions);
        formula(f.children.at(0), untimed_child(c, f), !positive);
        formula(f.children.at(1), untimed_child(c, f), positive);
        return;
      case ast::FormulaKind::Exists:
      case ast::FormulaKind::Forall: {
          bool existential = (f.kind == ast::FormulaKind::Exists) == positive;
          features_.add(
            existential ? Feature::ExistentialPreconditions : Feature::UniversalPreconditions);
          written.add(f.kind == ast::FormulaKind::Exists ? Feature::ExistentialPreconditions :
            Feature::UniversalPreconditions);
          Scoped s(*this, variables(f.params));
          formula(f.children.at(0), c, positive);
          return;
        }
      case ast::FormulaKind::Atom:
        if (c.need_time) {
          untimed(f.loc);
        }
        atom(f, positive);
        return;
      case ast::FormulaKind::Compare:
        if (c.need_time) {
          untimed(f.loc);
        }
        compare(f, c, positive);
        return;
      case ast::FormulaKind::AtStart:
      case ast::FormulaKind::AtEnd:
      case ast::FormulaKind::OverAll: {
          bool at_end_constraint = c.constraints && f.kind == ast::FormulaKind::AtEnd;
          if (!c.timed && !at_end_constraint) {
            error("semantic.misplaced_timed",
              "at start / at end / over all only belong at the top of durative conditions",
              f.loc);
            return;
          }
          Ctx inner = c;
          inner.timed = false;
          inner.need_time = false;
          formula(f.children.at(0), inner, positive);
          return;
        }
      case ast::FormulaKind::Preference: {
          if (!c.preferences) {
            error("semantic.misplaced_preference", "preferences are not allowed here", f.loc);
            return;
          }
          if (!positive) {
            error("semantic.negated_preference", "a preference cannot be negated", f.loc);
          }
          features_.add(Feature::Preferences);
          if (f.name.valid()) {
            preferences_.insert(str(f.name));
          }
          Ctx inner = c;
          inner.preferences = false;
          formula(f.children.at(0), inner, positive);
          return;
        }
      case ast::FormulaKind::Modal: {
          if (!c.constraints) {
            error("semantic.misplaced_modal",
              "modal operators only belong in :constraints", f.loc);
            return;
          }
          features_.add(Feature::TrajectoryConstraints);
          for (double t : f.times) {
            if (t < 0) {
              error("semantic.negative_time", "times in constraints cannot be negative", f.loc);
            }
          }
          for (FormulaId ch : f.children) {
            formula(ch, c, positive);
          }
          return;
        }
    }
  }

  // Inside or/not/imply, the timed wrapper requirement still applies to the operands
  static Ctx untimed_child(Ctx c, const ast::Formula &) {return c;}

  void untimed(ast::Loc loc)
  {
    error("semantic.untimed_condition",
      "conditions of durative actions must be under at start, at end or over all", loc);
  }

  void atom(const ast::Formula & f, bool positive)
  {
    std::string n = str(f.name);
    auto p = predicates_.find(n);
    if (!p) {
      error("semantic.unknown_predicate",
        "unknown predicate '" + n + "'" + did_you_mean(n, names_of(predicates_)), f.loc);
      return;
    }
    const PredicateInfo & info = predicates_.items[*p];
    check_args(info.name, info.params, f.args, f.loc);
    if (!positive) {
      features_.add(Feature::NegativePreconditions);
    }
    if (on_atom) {
      on_atom(*p, positive);
    }
  }

  void compare(const ast::Formula & f, const Ctx & c, bool positive)
  {
    ExprInfo l = expr(f.lhs, c);
    ExprInfo r = expr(f.rhs, c);
    if (l.kind == ExprInfo::Kind::Invalid || r.kind == ExprInfo::Kind::Invalid) {
      return;
    }
    if (l.kind == ExprInfo::Kind::Object || r.kind == ExprInfo::Kind::Object) {
      if (l.kind != r.kind) {
        error("semantic.type_mismatch", "comparison between an object and a number", f.loc);
        return;
      }
      if (f.compare != ast::CompareOp::Eq) {
        error("semantic.type_mismatch", "objects can only be compared with '='", f.loc);
        return;
      }
      features_.add(Feature::Equality);
      if (!positive) {
        features_.add(Feature::NegativePreconditions);
      }
      return;
    }
    features_.add(Feature::NumericFluents);
  }

  // --- Effects -------------------------------------------------------------------------------

  enum class EffectPlace {Instant, Durative, Process, Event};

  void effect(EffectId id, EffectPlace place, bool need_time, const Ctx & base)
  {
    const ast::Effect & e = nodes_.effects[id];
    switch (e.kind) {
      case ast::EffectKind::And:
        for (EffectId ch : e.children) {
          effect(ch, place, need_time, base);
        }
        return;
      case ast::EffectKind::Forall: {
          features_.add(Feature::ConditionalEffects);
          Scoped s(*this, variables(e.params));
          effect(e.children.at(0), place, need_time, base);
          return;
        }
      case ast::EffectKind::When: {
          features_.add(Feature::ConditionalEffects);
          Ctx cond = base;
          cond.timed = place == EffectPlace::Durative;
          cond.need_time = need_time;
          cond.preferences = false;
          formula(e.condition, cond, true);
          effect(e.children.at(0), place, need_time, base);
          return;
        }
      case ast::EffectKind::Add:
      case ast::EffectKind::Delete: {
          if (need_time) {
            untimed_effect(e.loc);
          }
          if (place == EffectPlace::Process) {
            error("semantic.discrete_effect_in_process",
              "processes can only have continuous numeric effects", e.loc);
          }
          std::string n = str(e.name);
          auto p = predicates_.find(n);
          if (!p) {
            error("semantic.unknown_predicate",
              "unknown predicate '" + n + "'" + did_you_mean(n, names_of(predicates_)), e.loc);
            return;
          }
          const PredicateInfo & info = predicates_.items[*p];
          if (info.derived) {
            error("semantic.derived_in_effect",
              "derived predicate '" + n + "' cannot be changed by an effect", e.loc);
          }
          check_args(info.name, info.params, e.args, e.loc);
          return;
        }
      case ast::EffectKind::Assign:
        assign(e, place, need_time, base);
        return;
      case ast::EffectKind::AtStart:
      case ast::EffectKind::AtEnd:
        if (place != EffectPlace::Durative || !need_time) {
          error("semantic.misplaced_timed",
            "at start / at end only belong at the top of durative effects", e.loc);
          return;
        }
        effect(e.children.at(0), place, false, base);
        return;
    }
  }

  void untimed_effect(ast::Loc loc)
  {
    error("semantic.untimed_effect",
      "effects of durative actions must be under at start or at end (or be continuous)", loc);
  }

  void assign(const ast::Effect & e, EffectPlace place, bool need_time, const Ctx & base)
  {
    const ast::Expr & target = nodes_.exprs[e.target];
    std::string n = str(target.name);
    auto f = functions_.find(n);
    if (!f) {
      error("semantic.unknown_function",
        "unknown function '" + n + "'" + did_you_mean(n, names_of(functions_)), target.loc);
      return;
    }
    const FunctionInfo & info = functions_.items[*f];
    check_args(info.name, info.params, target.args, target.loc);

    // (assign (f ...) undefined): PDDL 3.1, an object fluent with no value
    const ast::Expr & raw = nodes_.exprs[e.value];
    if (!info.numeric && e.op == ast::AssignOp::Assign && raw.kind == ast::ExprKind::Term &&
      nodes_.terms[raw.term].kind == ast::TermKind::Name &&
      str(nodes_.terms[raw.term].name) == "undefined")
    {
      features_.add(Feature::ObjectFluents);
      return;
    }
    bool incremental = e.op == ast::AssignOp::Increase || e.op == ast::AssignOp::Decrease;
    Ctx vc = base;
    vc.continuous = incremental &&
      (place == EffectPlace::Process || (place == EffectPlace::Durative && need_time));
    ExprInfo value = expr(e.value, vc);
    bool continuous = value.has_t;

    if (place == EffectPlace::Process && !continuous) {
      error("semantic.discrete_effect_in_process",
        "processes can only have continuous effects (with #t)", e.loc);
    }
    if (need_time && !continuous) {
      untimed_effect(e.loc);
    }

    if (info.numeric) {
      features_.add(Feature::NumericFluents);
      if (value.kind == ExprInfo::Kind::Object) {
        error("semantic.type_mismatch", "numeric function '" + n + "' assigned an object", e.loc);
      }
      if (n == "total-cost" && e.op == ast::AssignOp::Increase) {
        features_.add(Feature::ActionCosts);
      }
      return;
    }
    features_.add(Feature::ObjectFluents);
    if (e.op != ast::AssignOp::Assign) {
      error("semantic.type_mismatch", "object function '" + n + "' can only be assigned", e.loc);
      return;
    }
    bool wrong_object = value.kind == ExprInfo::Kind::Object &&
      !types_.compatible(value.types, info.result);
    if (value.kind == ExprInfo::Kind::Numeric) {
      error("semantic.type_mismatch", "object function '" + n + "' assigned a number", e.loc);
    } else if (wrong_object) {
      error("semantic.type_mismatch",
        "object function '" + n + "' returns " + types_.to_string(info.result), e.loc);
    }
  }

  // Called for every atom checked in a formula (used to build the derived dependency graph)
  std::function<void(PredicateId, bool)> on_atom;
  // is-violated references found in a metric
  std::vector<std::pair<std::string, ast::Loc>> violated_;
  // problem checked without its domain: unknown names are warnings
  bool objects_alone_ = false;

private:
  const ast::Nodes & nodes_;
  const SymbolTable & symbols_;
  std::string file_;
  std::vector<Diagnostic> & diags_;
  const TypeHierarchy & types_;
  FeatureSet & features_;
  const Table<PredicateId, PredicateInfo> & predicates_;
  const Table<FunctionId, FunctionInfo> & functions_;
  const Table<ObjectId, ObjectInfo> & objects_;
  std::set<std::string> & preferences_;
  std::set<std::string> warned_names_;
};

std::string name_of(const SymbolTable & s, Symbol sym) {return std::string(s.name(sym));}

// An object declared again: identical is a warning; with other types it belongs to all of them
// (IPC domains such as elevator 2000 and TMS 2011 rely on this)
void merge_types(
  TypeSpec & current, const TypeSpec & more, const std::string & name, ast::Loc loc,
  const std::string & file, std::vector<Diagnostic> & diags)
{
  SourceLocation where{file, loc.line, loc.column};
  if (current == more) {
    diags.push_back(
      Diagnostic::warning("semantic.duplicate_object", "'" + name + "' declared twice", where));
    return;
  }
  for (TypeId t : more) {
    if (std::find(current.begin(), current.end(), t) == current.end()) {
      current.push_back(t);
    }
  }
  diags.push_back(
    Diagnostic::warning(
      "semantic.object_with_several_types",
      "'" + name + "' declared with several types; it belongs to all of them", where));
}

// The features to check against :requirements: negations, disjunctions and quantifiers as
// written
FeatureSet as_written(const FeatureSet & used, const FeatureSet & written)
{
  FeatureSet polarity;
  polarity.add(Feature::NegativePreconditions);
  polarity.add(Feature::DisjunctivePreconditions);
  polarity.add(Feature::ExistentialPreconditions);
  polarity.add(Feature::UniversalPreconditions);
  FeatureSet out = used.minus(polarity);
  out.merge(written);
  return out;
}

// Requirements that are missing for the features used; one warning per feature
void check_requirements(
  const FeatureSet & used, const std::set<std::string> & declared, const std::string & file,
  ast::Loc loc, std::vector<Diagnostic> & diags)
{
  for (Feature f : used.list()) {
    auto covering = covering_requirements(f);
    if (covering.empty()) {
      continue;
    }
    bool covered = std::any_of(covering.begin(), covering.end(),
        [&](std::string_view r) {return declared.count(std::string(r)) != 0;});
    if (!covered) {
      diags.push_back(
        Diagnostic::warning(
          "semantic.undeclared_requirement",
          "uses " + std::string(to_string(f)) + " but :requirements does not include " +
          std::string(covering.front()), SourceLocation{file, loc.line, loc.column}));
    }
  }
}

// Collects the functions assigned by any effect (the others are static)
void collect_changing(
  const ast::Nodes & nodes, const SymbolTable & symbols, EffectId id,
  std::set<std::string> & out)
{
  if (!id.valid()) {
    return;
  }
  const ast::Effect & e = nodes.effects[id];
  if (e.kind == ast::EffectKind::Assign) {
    out.insert(name_of(symbols, nodes.exprs[e.target].name));
  }
  for (EffectId ch : e.children) {
    collect_changing(nodes, symbols, ch, out);
  }
}

// Tarjan's SCC over derived predicates; a negative edge inside a component is unstratifiable
std::vector<std::string> unstratifiable(
  const std::map<std::string, std::vector<std::pair<std::string, bool>>> & edges)
{
  std::map<std::string, int> index, low;
  std::map<std::string, bool> on_stack;
  std::vector<std::string> stack, bad;
  int counter = 0;
  std::function<void(const std::string &)> visit = [&](const std::string & v) {
      index[v] = low[v] = counter++;
      stack.push_back(v);
      on_stack[v] = true;
      auto it = edges.find(v);
      if (it != edges.end()) {
        for (const auto & [w, positive] : it->second) {
          if (!index.count(w)) {
            visit(w);
            low[v] = std::min(low[v], low[w]);
          } else if (on_stack[w]) {
            low[v] = std::min(low[v], index[w]);
          }
        }
      }
      if (low[v] == index[v]) {
        std::set<std::string> component;
        std::string w;
        do {
          w = stack.back();
          stack.pop_back();
          on_stack[w] = false;
          component.insert(w);
        } while (w != v);
        for (const auto & member : component) {
          auto e = edges.find(member);
          if (e == edges.end()) {
            continue;
          }
          for (const auto & [target, positive] : e->second) {
            if (!positive && component.count(target)) {
              bad.push_back(member);
            }
          }
        }
      }
    };
  for (const auto & [v, out] : edges) {
    if (!index.count(v)) {
      visit(v);
    }
  }
  return bad;
}

}  // namespace

Result<DomainModel> analyze_domain(const ast::Domain & d)
{
  std::vector<Diagnostic> diags;
  DomainModel m;
  const auto & sy = d.symbols;
  auto str = [&](Symbol s) {return name_of(sy, s);};
  auto err = [&](const std::string & code, const std::string & msg, ast::Loc loc) {
      diags.push_back(Diagnostic::error(code, msg, SourceLocation{d.file, loc.line, loc.column}));
    };

  m.name = str(d.name);
  for (const auto & r : d.requirements) {
    m.requirements.insert(str(r.name));
  }

  // Types: declare everything first, then parents
  std::set<std::string> declared_types;
  for (const auto & t : d.types) {
    std::string n = str(t.name);
    if (n == "object") {
      continue;
    }
    if (n == "number") {
      // Old domains (IPC 2004 promela) use it as an object type; functions "- number" stay
      // numeric
      diags.push_back(
        Diagnostic::warning(
          "semantic.number_as_type", "'number' declared as an object type",
          SourceLocation{d.file, t.loc.line, t.loc.column}));
    }
    if (!declared_types.insert(n).second) {
      // Repeated in the list: only an error if given different parents
      continue;
    }
    m.types.declare(n, t.loc);
  }
  for (const auto & t : d.types) {
    std::string n = str(t.name);
    if (n == "object") {
      continue;
    }
    TypeId id = *m.types.find(n);
    for (Symbol parent : t.types) {
      std::string pn = str(parent);
      // Parent types need not be listed on their own
      TypeId pid = m.types.declare(pn, t.loc);
      m.types.add_parent(id, pid);
    }
  }
  if (!d.types.empty()) {
    m.features.add(Feature::Typing);
  }
  if (auto cycle = m.types.finalize()) {
    err("semantic.type_cycle", "type '" + m.types[*cycle].name + "' is its own ancestor",
      m.types[*cycle].loc);
    return Result<DomainModel>::failure(std::move(diags));
  }

  std::set<std::string> prefs_unused;
  FeatureSet scratch;
  Checker probe(d.nodes, sy, d.file, diags, m.types, scratch, m.predicates, m.functions,
    m.constants, prefs_unused);

  // Constants
  for (const auto & c : d.constants) {
    std::string n = str(c.name);
    auto spec = probe.resolve(c.types, Checker::type_loc(c));
    TypeSpec types = spec.value_or(TypeSpec{});
    if (auto existing = m.constants.find(n)) {
      merge_types(m.constants.items[*existing].types, types, n, c.loc, d.file, diags);
      continue;
    }
    m.constants.add(n, ObjectInfo{n, types, true, c.loc});
  }

  // Predicates and functions
  auto param_specs = [&](const std::vector<ast::TypedName> & params) {
      std::vector<TypeSpec> out;
      std::set<std::string> seen;
      for (const auto & p : params) {
        if (!seen.insert(str(p.name)).second) {
          // Only the arity of a declaration matters
          diags.push_back(
            Diagnostic::warning(
              "semantic.duplicate_parameter", "parameter " + str(p.name) + " declared twice",
              SourceLocation{d.file, p.loc.line, p.loc.column}));
        }
        if (!p.types.empty()) {
          m.features.add(Feature::Typing);
        }
        out.push_back(probe.resolve(p.types, Checker::type_loc(p)).value_or(TypeSpec{}));
      }
      return out;
    };
  for (const auto & p : d.predicates) {
    std::string n = str(p.name);
    if (!m.predicates.add(n, PredicateInfo{n, param_specs(p.params), false, p.loc})) {
      err("semantic.duplicate_predicate", "predicate '" + n + "' declared twice", p.loc);
    }
  }
  std::set<std::string> changing;
  for (const auto & a : d.actions) {
    collect_changing(d.nodes, sy, a.effect, changing);
  }
  for (const auto & a : d.durative_actions) {
    collect_changing(d.nodes, sy, a.effect, changing);
  }
  for (const auto & a : d.processes) {
    collect_changing(d.nodes, sy, a.effect, changing);
  }
  for (const auto & a : d.events) {
    collect_changing(d.nodes, sy, a.effect, changing);
  }
  for (const auto & f : d.functions) {
    std::string n = str(f.name);
    FunctionInfo info{n, param_specs(f.params), true, {}, changing.count(n) != 0, f.loc};
    if (!f.type.empty() && !(f.type.size() == 1 && str(f.type[0]) == "number")) {
      info.numeric = false;
      info.result = probe.resolve(f.type, f.loc).value_or(TypeSpec{});
    }
    if (!m.functions.add(n, std::move(info))) {
      err("semantic.duplicate_function", "function '" + n + "' declared twice", f.loc);
    }
  }

  // Derived predicates: declared in :predicates, or declared here implicitly
  for (const auto & dv : d.derived) {
    std::string n = str(dv.head.name);
    auto p = m.predicates.find(n);
    if (!p) {
      diags.push_back(
        Diagnostic::warning(
          "semantic.undeclared_derived",
          "derived predicate '" + n + "' is not in :predicates", SourceLocation{d.file,
            dv.loc.line, dv.loc.column}));
      p = m.predicates.add(n, PredicateInfo{n, param_specs(dv.head.params), true, dv.loc});
    } else if (m.predicates.items[*p].params.size() != dv.head.params.size()) {
      err("semantic.wrong_arity", "derived predicate '" + n + "' has a different arity than "
        "its declaration", dv.loc);
      continue;
    }
    m.predicates.items[*p].derived = true;
    m.features.add(Feature::DerivedPredicates);
  }

  Checker c(d.nodes, sy, d.file, diags, m.types, m.features, m.predicates, m.functions,
    m.constants, m.preferences);
  std::set<std::string> action_names;
  auto unique_action = [&](Symbol name, ast::Loc loc) {
      if (!action_names.insert(str(name)).second) {
        err("semantic.duplicate_action", "action '" + str(name) + "' defined twice", loc);
      }
    };

  for (const auto & a : d.actions) {
    unique_action(a.name, a.loc);
    c.context = "action '" + str(a.name) + "'";
    Checker::Scoped s(c, c.variables(a.params));
    if (a.precondition.valid()) {
      c.formula(a.precondition, Ctx{}, true);
    }
    if (a.effect.valid()) {
      c.effect(a.effect, Checker::EffectPlace::Instant, false, Ctx{});
    }
  }

  for (const auto & a : d.durative_actions) {
    unique_action(a.name, a.loc);
    m.features.add(Feature::DurativeActions);
    c.context = "durative action '" + str(a.name) + "'";
    Checker::Scoped s(c, c.variables(a.params));
    // Duration: comparisons between ?duration and an expression without it
    std::function<void(FormulaId)> duration = [&](FormulaId id) {
        const ast::Formula & f = d.nodes.formulas[id];
        if (f.kind == ast::FormulaKind::And || f.kind == ast::FormulaKind::AtStart ||
          f.kind == ast::FormulaKind::AtEnd)
        {
          for (FormulaId ch : f.children) {
            duration(ch);
          }
          return;
        }
        if (f.kind != ast::FormulaKind::Compare) {
          err("semantic.bad_duration", "the duration must compare ?duration with a value",
            f.loc);
          return;
        }
        bool lhs_dur = d.nodes.exprs[f.lhs].kind == ast::ExprKind::Duration;
        bool rhs_dur = d.nodes.exprs[f.rhs].kind == ast::ExprKind::Duration;
        if (lhs_dur == rhs_dur) {
          err("semantic.bad_duration", "one side of a duration constraint must be ?duration",
            f.loc);
          return;
        }
        if (f.compare == ast::CompareOp::Lt || f.compare == ast::CompareOp::Gt) {
          err("semantic.bad_duration", "durations are bounded with =, <= or >=", f.loc);
          return;
        }
        if (f.compare != ast::CompareOp::Eq) {
          m.features.add(Feature::DurationInequalities);
        }
        Ctx dc;
        dc.duration_var = false;  // ?duration on the other side would be circular
        ExprInfo value = c.expr(lhs_dur ? f.rhs : f.lhs, dc);
        if (value.kind == ExprInfo::Kind::Object) {
          err("semantic.type_mismatch", "a duration must be numeric", f.loc);
        }
        if (value.changing) {
          m.features.add(Feature::VariableDuration);
        }
      };
    duration(a.duration);
    if (a.condition.valid()) {
      Ctx cc;
      cc.timed = true;
      cc.need_time = true;
      cc.duration_var = true;
      c.formula(a.condition, cc, true);
    }
    if (a.effect.valid()) {
      Ctx ec;
      ec.duration_var = true;
      c.effect(a.effect, Checker::EffectPlace::Durative, true, ec);
    }
  }

  for (const auto & p : d.processes) {
    unique_action(p.name, p.loc);
    m.features.add(Feature::Processes);
    c.context = "process '" + str(p.name) + "'";
    Checker::Scoped s(c, c.variables(p.params));
    if (p.precondition.valid()) {
      Ctx pc;
      pc.preferences = false;
      c.formula(p.precondition, pc, true);
    }
    if (p.effect.valid()) {
      c.effect(p.effect, Checker::EffectPlace::Process, false, Ctx{});
    }
  }

  for (const auto & e : d.events) {
    unique_action(e.name, e.loc);
    m.features.add(Feature::Events);
    c.context = "event '" + str(e.name) + "'";
    Checker::Scoped s(c, c.variables(e.params));
    if (e.precondition.valid()) {
      Ctx pc;
      pc.preferences = false;
      c.formula(e.precondition, pc, true);
    }
    if (e.effect.valid()) {
      c.effect(e.effect, Checker::EffectPlace::Event, false, Ctx{});
    }
  }

  for (const auto & gc : d.global_constraints) {
    m.features.add(Feature::GlobalConstraints);
    c.context = "constraint '" + str(gc.name) + "'";
    Checker::Scoped s(c, c.variables(gc.params));
    Ctx gctx;
    gctx.preferences = false;
    c.formula(gc.condition, gctx, true);
  }

  if (d.constraints.valid()) {
    c.context = "the domain constraints";
    Ctx cc;
    cc.constraints = true;
    c.formula(d.constraints, cc, true);
  }

  // Derived bodies, building the dependency graph between derived predicates
  std::map<std::string, std::vector<std::pair<std::string, bool>>> edges;
  for (const auto & dv : d.derived) {
    std::string head = str(dv.head.name);
    auto p = m.predicates.find(head);
    if (!p || !m.predicates.items[*p].derived) {
      continue;
    }
    // Head variables take the declared types when the head leaves them untyped
    std::vector<Var> vars = c.variables(dv.head.params);
    const auto & declared = m.predicates.items[*p].params;
    for (size_t i = 0; i < vars.size() && i < declared.size(); ++i) {
      if (dv.head.params[i].types.empty()) {
        vars[i].types = declared[i];
      }
    }
    c.context = "derived predicate '" + head + "'";
    Checker::Scoped s(c, vars);
    edges[head];
    c.on_atom = [&](PredicateId id, bool positive) {
        if (m.predicates.items[id].derived) {
          edges[head].emplace_back(m.predicates.items[id].name, positive);
        }
      };
    Ctx dctx;
    dctx.preferences = false;
    c.formula(dv.body, dctx, true);
    c.on_atom = nullptr;
  }
  for (const auto & bad : unstratifiable(edges)) {
    auto p = m.predicates.find(bad);
    err("semantic.unstratifiable_derived",
      "derived predicate '" + bad + "' depends negatively on itself", m.predicates.items[*p].loc);
  }

  check_requirements(as_written(m.features, c.written), m.requirements, d.file,
    d.requirements.empty() ? d.loc : d.requirements.front().loc, diags);

  if (has_errors(diags)) {
    return Result<DomainModel>::failure(std::move(diags));
  }
  return Result<DomainModel>::success(std::move(m), std::move(diags));
}

Result<ProblemModel> analyze_problem(
  const ast::Problem & p, const ast::Domain & domain, const DomainModel & dm)
{
  std::vector<Diagnostic> diags;
  ProblemModel m;
  const auto & sy = p.symbols;
  auto str = [&](Symbol s) {return name_of(sy, s);};
  auto loc_of = [&](ast::Loc l) {return SourceLocation{p.file, l.line, l.column};};
  auto err = [&](const std::string & code, const std::string & msg, ast::Loc l) {
      diags.push_back(Diagnostic::error(code, msg, loc_of(l)));
    };
  auto warn = [&](const std::string & code, const std::string & msg, ast::Loc l) {
      diags.push_back(Diagnostic::warning(code, msg, loc_of(l)));
    };

  m.name = str(p.name);
  m.features = dm.features;
  if (!p.domain_name.valid()) {
    warn("semantic.missing_domain_name", "the problem does not name its domain", p.loc);
  } else if (str(p.domain_name) != dm.name) {
    warn("semantic.domain_name_mismatch",
      "problem is for domain '" + str(p.domain_name) + "', not '" + dm.name + "'", p.loc);
  }
  (void)domain;

  // Objects: constants first
  for (const auto & c : dm.constants.items) {
    m.objects.add(c.name, c);
  }
  FeatureSet scratch;
  std::set<std::string> no_prefs;
  Checker probe(p.nodes, sy, p.file, diags, dm.types, scratch, dm.predicates, dm.functions,
    m.objects, no_prefs);
  for (const auto & o : p.objects) {
    std::string n = str(o.name);
    TypeSpec spec = probe.resolve(o.types, Checker::type_loc(o)).value_or(TypeSpec{});
    if (!o.types.empty()) {
      m.features.add(Feature::Typing);
    }
    if (auto existing = m.objects.find(n)) {
      ObjectInfo & prev = m.objects.items[*existing];
      if (prev.constant && prev.types == spec) {
        warn("semantic.redeclared_constant", "'" + n + "' is already a domain constant", o.loc);
      } else {
        merge_types(prev.types, spec, n, o.loc, p.file, diags);
      }
      continue;
    }
    m.objects.add(n, ObjectInfo{n, spec, false, o.loc});
  }

  Checker c(p.nodes, sy, p.file, diags, dm.types, m.features, dm.predicates, dm.functions,
    m.objects, m.preferences);

  // Initial state
  c.context = "the initial state";
  std::unordered_map<std::string, std::string> fluent_values;
  for (const auto & item : p.init) {
    if (item.time) {
      if (*item.time < 0) {
        err("semantic.negative_time", "timed initial facts cannot happen before time 0",
          item.loc);
      }
      m.features.add(
        item.kind == ast::InitItem::Kind::Fact ? Feature::TimedInitialLiterals :
        Feature::TimedInitialFluents);
    }
    for (TermId a : item.args) {
      if (p.nodes.terms[a].kind != ast::TermKind::Name) {
        err("semantic.init_not_ground", "initial facts can only mention objects",
          p.nodes.terms[a].loc);
      }
    }
    std::string n = str(item.name);
    if (item.kind == ast::InitItem::Kind::Fact) {
      auto pred = dm.predicates.find(n);
      if (!pred) {
        // Nothing reads it (ENHSP's instances of other versions of a domain do this)
        warn("semantic.unknown_predicate", "in the initial state: unknown predicate '" + n +
          "'; the fact is ignored" + did_you_mean(n, Checker::names_of(dm.predicates)), item.loc);
        continue;
      }
      const PredicateInfo & info = dm.predicates.items[*pred];
      if (info.derived) {
        err("semantic.derived_in_init", "derived predicate '" + n + "' cannot be initialized",
          item.loc);
      }
      c.check_args(info.name, info.params, item.args, item.loc);
      if (item.negated && !item.time) {
        warn("semantic.negative_init", "negative initial facts have no effect", item.loc);
      }
      continue;
    }
    auto fn = dm.functions.find(n);
    if (!fn) {
      // A value for a function the domain never mentions cannot affect anything (IPC 2023
      // markettrader instances do this)
      warn("semantic.unknown_function",
        "in the initial state: unknown function '" + n + "'; its value is ignored" +
        did_you_mean(n, Checker::names_of(dm.functions)), item.loc);
      continue;
    }
    const FunctionInfo & info = dm.functions.items[*fn];
    c.check_args(info.name, info.params, item.args, item.loc);
    const ast::Expr & value = p.nodes.exprs[item.value];
    std::string value_text;
    if (info.numeric) {
      m.features.add(Feature::NumericFluents);
      if (value.kind != ast::ExprKind::Number) {
        err("semantic.init_value", "initial value of '" + n + "' must be a number", item.loc);
        continue;
      }
      value_text = std::to_string(value.number);
    } else {
      m.features.add(Feature::ObjectFluents);
      if (value.kind != ast::ExprKind::Term ||
        p.nodes.terms[value.term].kind != ast::TermKind::Name)
      {
        err("semantic.init_value", "initial value of '" + n + "' must be an object", item.loc);
        continue;
      }
      std::string obj = str(p.nodes.terms[value.term].name);
      if (obj != "undefined") {
        auto o = m.objects.find(obj);
        if (!o) {
          err("semantic.unknown_object", "in the initial state: unknown object '" + obj + "'" +
            did_you_mean(obj, Checker::names_of(m.objects)), item.loc);
          continue;
        }
        if (!dm.types.fits(m.objects.items[*o].types, info.result)) {
          err("semantic.type_mismatch",
            "'" + n + "' returns " + dm.types.to_string(info.result), item.loc);
        }
      }
      value_text = obj;
    }
    if (!item.time) {
      std::string key = n;
      for (TermId a : item.args) {
        key += " " + str(p.nodes.terms[a].name);
      }
      auto [it, inserted] = fluent_values.emplace(key, value_text);
      if (!inserted && it->second != value_text) {
        warn("semantic.conflicting_init",
          "'(" + key + ")' initialized with two values; the last one is used", item.loc);
        it->second = value_text;
      } else if (!inserted) {
        warn("semantic.duplicate_init", "'(" + key + ")' initialized twice", item.loc);
      }
    }
  }

  if (p.goal.valid()) {
    c.context = "the goal";
    c.formula(p.goal, Ctx{}, true);
  }
  if (p.constraints.valid()) {
    c.context = "the problem constraints";
    Ctx cc;
    cc.constraints = true;
    c.formula(p.constraints, cc, true);
  }
  if (p.metric) {
    c.context = "the metric";
    m.features.add(Feature::Metric);
    Ctx mc;
    mc.metric = true;
    mc.preferences = false;
    ExprInfo info = c.expr(p.metric->expr, mc);
    if (info.kind == ExprInfo::Kind::Object) {
      err("semantic.type_mismatch", "the metric must be numeric", p.metric->loc);
    }
    if (info.functions) {
      m.features.add(Feature::MetricFluents);
    }
    for (const auto & [pref, loc] : c.violated_) {
      if (!m.preferences.count(pref) && !dm.preferences.count(pref)) {
        err("semantic.unknown_preference", "no preference named '" + pref + "'", loc);
      }
    }
  }

  std::set<std::string> requirements = dm.requirements;
  for (const auto & r : p.requirements) {
    requirements.insert(str(r.name));
  }
  check_requirements(as_written(m.features.minus(dm.features), c.written), requirements, p.file,
    p.requirements.empty() ? p.loc : p.requirements.front().loc, diags);

  if (has_errors(diags)) {
    return Result<ProblemModel>::failure(std::move(diags));
  }
  return Result<ProblemModel>::success(std::move(m), std::move(diags));
}

std::vector<Diagnostic> check_problem_alone(const ast::Problem & p)
{
  std::vector<Diagnostic> diags;
  const auto & sy = p.symbols;
  auto str = [&](Symbol s) {return name_of(sy, s);};
  auto loc_of = [&](ast::Loc l) {return SourceLocation{p.file, l.line, l.column};};

  // Without the domain there are no types: every object is just an object
  TypeHierarchy types;
  types.finalize();
  Table<ObjectId, ObjectInfo> objects;
  for (const auto & o : p.objects) {
    std::string n = str(o.name);
    if (!objects.add(n, ObjectInfo{n, {types.object()}, false, o.loc})) {
      diags.push_back(
        Diagnostic::error("semantic.duplicate_object", "object '" + n + "' declared twice",
        loc_of(o.loc)));
    }
  }

  // Predicates and functions are unknown: build permissive tables from their uses
  Table<PredicateId, PredicateInfo> predicates;
  Table<FunctionId, FunctionInfo> functions;
  auto any_params = [&](size_t n) {return std::vector<TypeSpec>(n, TypeSpec{types.object()});};
  for (const auto & item : p.init) {
    std::string n = str(item.name);
    if (item.kind == ast::InitItem::Kind::Fact) {
      predicates.add(n, PredicateInfo{n, any_params(item.args.size()), false, item.loc});
    } else {
      bool numeric = p.nodes.exprs[item.value].kind == ast::ExprKind::Number;
      functions.add(n, FunctionInfo{n, any_params(item.args.size()), numeric,
          {types.object()}, false, item.loc});
    }
  }

  FeatureSet features;
  std::set<std::string> prefs;
  Checker c(p.nodes, sy, p.file, diags, types, features, predicates, functions, objects, prefs);
  c.objects_alone_ = true;

  for (const auto & item : p.init) {
    if (item.time && *item.time < 0) {
      diags.push_back(
        Diagnostic::error(
          "semantic.negative_time", "timed initial facts cannot happen before time 0",
          loc_of(item.loc)));
    }
    for (TermId a : item.args) {
      const ast::Term & t = p.nodes.terms[a];
      if (t.kind != ast::TermKind::Name) {
        diags.push_back(
          Diagnostic::error(
            "semantic.init_not_ground", "initial facts can only mention objects",
            loc_of(t.loc)));
      } else {
        c.term(a);
      }
    }
    if (item.negated && !item.time) {
      diags.push_back(
        Diagnostic::warning(
          "semantic.negative_init", "negative initial facts have no effect",
          loc_of(item.loc)));
    }
  }

  // Goal: only variable binding and names can be checked; predicates may be undeclared here
  std::function<void(FormulaId)> goal = [&](FormulaId id) {
      const ast::Formula & f = p.nodes.formulas[id];
      if (f.kind == ast::FormulaKind::Exists || f.kind == ast::FormulaKind::Forall) {
        Checker::Scoped s(c, c.variables(f.params));
        goal(f.children.at(0));
        return;
      }
      if (f.kind == ast::FormulaKind::Atom) {
        for (TermId a : f.args) {
          c.term(a);
        }
        return;
      }
      if (f.kind == ast::FormulaKind::Preference && f.name.valid()) {
        prefs.insert(str(f.name));
      }
      for (FormulaId ch : f.children) {
        goal(ch);
      }
    };
  if (p.goal.valid()) {
    goal(p.goal);
  }
  if (p.constraints.valid()) {
    goal(p.constraints);
  }
  return diags;
}

}  // namespace kairos::sem
