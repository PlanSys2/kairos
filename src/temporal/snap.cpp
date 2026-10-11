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

#include "temporal/snap.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "sas/translate.hpp"
#include "state/eval.hpp"
#include "temporal/access.hpp"

namespace kairos::temporal
{

namespace
{

bool uses_duration(const ground::Expr & e)
{
  if (e.op == ground::ExprOp::Duration) {
    return true;
  }
  return std::any_of(
    e.children.begin(), e.children.end(), [](const ground::Expr & c) {return uses_duration(c);});
}

// ?duration replaced by the (fixed) duration
void substitute(ground::Expr & e, const ground::Expr & duration)
{
  if (e.op == ground::ExprOp::Duration) {
    e = duration;
    return;
  }
  for (auto & c : e.children) {
    substitute(c, duration);
  }
}

void substitute(sas::Condition & c, const ground::Expr & duration)
{
  for (auto & n : c.numeric) {
    if (uses_duration(n.lhs.tree) || uses_duration(n.rhs.tree)) {
      substitute(n.lhs.tree, duration);
      substitute(n.rhs.tree, duration);
      n.lhs.linear.reset();
      n.rhs.linear.reset();
      n.difference.reset();
    }
  }
}

void substitute(sas::Effect & e, const ground::Expr & duration)
{
  substitute(e.condition, duration);
  if (e.kind == sas::Effect::Kind::Numeric && uses_duration(e.value.tree)) {
    substitute(e.value.tree, duration);
    e.value.linear.reset();
  }
}

// A new variable with its own atoms (for printing): binary with one atom, a group with more
sas::VarId add_variable(
  sas::Task & t, const std::vector<std::string> & names, sas::Variable::Kind kind,
  uint32_t initial)
{
  sas::Variable v;
  v.kind = kind;
  auto var = static_cast<sas::VarId>(t.variables.size());
  for (size_t i = 0; i < names.size(); ++i) {
    t.predicate_names.push_back(names[i]);
    ground::AtomInfo atom;
    atom.predicate = ground::PredicateId(static_cast<uint32_t>(t.predicate_names.size() - 1));
    t.atoms.push_back(atom);
    v.atoms.push_back(ground::AtomId(static_cast<uint32_t>(t.atoms.size() - 1)));
    t.atom_fact.emplace_back(var, static_cast<uint32_t>(i));
  }
  t.variables.push_back(v);
  t.initial.push_back(initial);
  return var;
}

}  // namespace

DurationBounds bounds(const sas::DurativeAction & a, const double * numbers)
{
  DurationBounds b;
  for (const auto & d : a.duration) {
    double v = state::evaluate(d.value, numbers);
    if (std::isnan(v)) {
      continue;
    }
    switch (d.op) {
      case sas::CompareOp::Eq: b.lo = std::max(b.lo, v); b.hi = std::min(b.hi, v); break;
      case sas::CompareOp::Ge: case sas::CompareOp::Gt: b.lo = std::max(b.lo, v); break;
      case sas::CompareOp::Le: case sas::CompareOp::Lt: b.hi = std::min(b.hi, v); break;
      default: break;
    }
  }
  return b;
}

SnapTask snap(const sas::Task & task)
{
  SnapTask s;
  s.task = task;
  s.task.durative_actions.clear();
  s.task.timed_events.clear();
  s.instant = task.actions.size();
  auto starts = static_cast<uint32_t>(s.task.action_names.size());
  for (const auto & n : task.durative_action_names) {
    s.task.action_names.push_back(n);
  }
  // Numeric variables some action changes: a duration that reads them is not fixed
  std::set<sas::NumVarId> written;
  auto note = [&](const std::vector<sas::Effect> & effects) {
      for (const auto & e : effects) {
        if (e.kind == sas::Effect::Kind::Numeric) {
          written.insert(e.num);
        }
      }
    };
  for (const auto & a : task.actions) {
    note(a.effects);
  }
  for (const auto & a : task.durative_actions) {
    note(a.effects);
  }
  for (const auto & ev : task.timed_events) {
    note(ev.effects);
  }
  for (const auto & da : task.durative_actions) {
    std::string name = sas::instance_name(task, da);
    sas::VarId run = add_variable(
      s.task, {"kairos-running " + name}, sas::Variable::Kind::Binary, 1);
    s.running.push_back(run);
    // ?duration: only a fixed duration that no action changes can replace it
    bool fixed = da.duration.size() == 1 && da.duration[0].op == sas::CompareOp::Eq;
    Access reads;
    for (const auto & d : da.duration) {
      reads.read(d.value.tree);
    }
    bool unchanging = std::none_of(
      reads.num_reads.begin(), reads.num_reads.end(),
      [&](sas::NumVarId n) {return written.count(n) > 0;});
    if (da.uses_duration && (!fixed || !unchanging) && s.unsupported.empty()) {
      s.unsupported = name + " uses ?duration with a duration that is not fixed (phase F3)";
    }
    ground::Expr duration = fixed ? da.duration[0].value.tree : ground::Expr{};

    sas::Action start;
    start.schema = starts + da.schema;
    start.args = da.args;
    start.pre = da.at_start;
    start.pre.facts.push_back(sas::Fact{run, 1});
    std::sort(start.pre.facts.begin(), start.pre.facts.end());
    substitute(start.pre, duration);
    start.changes = da.start;
    start.changes.push_back(sas::Fact{run, 0});

    sas::Action end;
    end.schema = start.schema;  // printed with the action's name
    end.args = da.args;
    end.pre = da.at_end;
    end.pre.facts.push_back(sas::Fact{run, 0});
    std::sort(end.pre.facts.begin(), end.pre.facts.end());
    substitute(end.pre, duration);
    end.changes = da.end;
    end.changes.push_back(sas::Fact{run, 1});

    for (auto e : da.effects) {
      if (e.continuous) {
        continue;
      }
      if (!e.start_condition.empty() && s.unsupported.empty()) {
        s.unsupported = name + " has end effects with conditions at start (M3 cannot check "
          "them yet)";
      }
      bool at_end = e.timing == sas::Timing::End;
      e.timing = sas::Timing::Instant;
      e.start_condition = {};
      substitute(e, duration);
      (at_end ? end : start).effects.push_back(std::move(e));
    }
    s.task.actions.push_back(std::move(start));
    s.task.actions.push_back(std::move(end));
  }
  // Timed initial literals: in order, a counter of those that happened (none of the values:
  // none yet)
  s.tils = s.task.actions.size();
  if (!task.timed_events.empty()) {
    std::vector<std::string> names;
    for (size_t i = 0; i < task.timed_events.size(); ++i) {
      names.push_back("kairos-tils-done " + std::to_string(i + 1));
    }
    auto n = static_cast<uint32_t>(names.size());
    sas::VarId counter = add_variable(s.task, names, sas::Variable::Kind::Group, n);
    s.task.action_names.push_back("kairos-til");
    auto schema = static_cast<uint32_t>(s.task.action_names.size() - 1);
    for (uint32_t j = 0; j < n; ++j) {
      const auto & ev = task.timed_events[j];
      sas::Action til;
      til.schema = schema;
      til.pre.facts.push_back(sas::Fact{counter, j == 0 ? n : j - 1});
      til.changes.push_back(sas::Fact{counter, j});
      for (auto e : ev.effects) {
        e.timing = sas::Timing::Instant;
        if (e.kind == sas::Effect::Kind::Assign && e.condition.empty()) {
          til.changes.push_back(e.fact);
        } else {
          til.effects.push_back(std::move(e));
        }
      }
      s.til_time.push_back(ev.time);
      s.task.actions.push_back(std::move(til));
    }
  }
  // Nothing may be running at the end
  for (auto run : s.running) {
    s.task.goal.facts.push_back(sas::Fact{run, 1});
  }
  std::sort(s.task.goal.facts.begin(), s.task.goal.facts.end());
  return s;
}

// --- The filter -------------------------------------------------------------------------------

// The STN of a plan of snap actions, with what each happening read and wrote to order the next
struct TemporalFilter::Builder
{
  Stn stn;
  Orderer<Stn::Node> order;
  // Per durative action while it runs: its start and its end (created with the start, so the
  // STN sees its duration and what must happen before it from the start on)
  struct Open
  {
    Stn::Node start = UINT32_MAX;
    Stn::Node end = UINT32_MAX;
  };
  std::vector<Open> open;
  size_t tils_done = 0;
  Stn::Mark mark{};

  explicit Builder(size_t durative)
  : open(durative) {}
};

namespace
{

std::vector<Access> accesses(const sas::Task & original, const SnapTask & s)
{
  auto bases = derived_bases(original);
  std::vector<Access> out(s.task.actions.size());
  for (uint32_t a = 0; a < s.task.actions.size(); ++a) {
    Access & acc = out[a];
    const auto & sa = s.task.actions[a];
    acc.read(sa.pre);
    for (const auto & f : sa.changes) {
      acc.writes.push_back(f.var);
    }
    for (const auto & e : sa.effects) {
      acc.effect(e);
    }
    if (s.is_start(a) || s.is_end(a)) {
      // Over all conditions: read at start and at end (with no separation: they hold in the
      // open interval), so no write breaks them in between
      acc.invariant(original.durative_actions[s.durative(a)].over_all);
    }
    acc.finish(bases);
  }
  return out;
}

}  // namespace

TemporalFilter::TemporalFilter(
  const sas::Task & original, const SnapTask & snap, state::StateSpace & space, double epsilon)
: original_(original), snap_(snap), space_(space), epsilon_(epsilon)
{
  access_ = accesses(original, snap);
  // What each durative action's end writes (values when known) and its over all reads
  for (size_t i = 0; i < original.durative_actions.size(); ++i) {
    const auto & da = original.durative_actions[i];
    EndWrites w;
    for (const auto & f : da.end) {
      w.facts[f.var] = f.value;
    }
    for (const auto & e : da.effects) {
      if (e.timing != sas::Timing::End) {
        continue;
      }
      if (e.kind == sas::Effect::Kind::Assign) {
        auto it = w.facts.find(e.fact.var);
        bool sure = e.condition.empty() && e.start_condition.empty();
        if (it == w.facts.end()) {
          w.facts[e.fact.var] = sure ? std::optional<uint32_t>(e.fact.value) : std::nullopt;
        } else if (!sure || it->second != e.fact.value) {
          it->second = std::nullopt;
        }
      } else if (e.kind == sas::Effect::Kind::Numeric) {
        w.nums.insert(e.num);
      } else if (e.kind == sas::Effect::Kind::Copy) {
        w.facts[e.target] = std::nullopt;
      }
    }
    end_writes_.push_back(std::move(w));
  }
  // What each timed initial literal changes and reads (but its own counter), by variable
  for (uint32_t j = 0; j < snap.til_time.size(); ++j) {
    const Access & acc = access_[snap.tils + j];
    for (auto v : acc.writes) {
      til_writes_[v].push_back(j);
    }
    for (auto v : acc.reads) {
      til_reads_[v].push_back(j);
    }
    for (auto n : acc.num_writes) {
      til_num_writes_[n].push_back(j);
    }
  }
  if (!snap.til_time.empty()) {
    // The counter orders the literals among themselves, not the other happenings
    sas::VarId counter = snap.task.actions[snap.tils].changes.front().var;
    til_writes_.erase(counter);
    til_reads_.erase(counter);
  }
}

TemporalFilter::~TemporalFilter() = default;

bool TemporalFilter::invariants(state::StateId s)
{
  const auto & values = space_.values(s);
  const auto & numbers = space_.numbers(s);
  for (size_t r = 0; r < snap_.running.size(); ++r) {
    if (values[snap_.running[r]] == 0 &&
      !state::holds(
        original_.durative_actions[r].over_all, snap_.task, values.data(), numbers.data()))
    {
      return false;
    }
  }
  return true;
}

// Adds a happening of `action` to the STN of the builder; false if it becomes inconsistent.
// With `record`, also what it read and wrote, for the happenings after it
// Whether the end of action r breaks the over all condition of action a
bool TemporalFilter::breaks(size_t r, size_t a) const
{
  const auto & w = end_writes_[r];
  const auto & c = original_.durative_actions[a].over_all;
  for (const auto & f : c.facts) {
    if (auto it = w.facts.find(f.var); it != w.facts.end() && it->second != f.value) {
      return true;  // another (or an unknown) value
    }
  }
  for (const auto & f : c.not_facts) {
    if (auto it = w.facts.find(f.var); it != w.facts.end() &&
      (!it->second || *it->second == f.value))
    {
      return true;
    }
  }
  for (const auto & v : c.vars) {
    if (w.facts.count(v.a) || w.facts.count(v.b)) {
      return true;
    }
  }
  if (!w.nums.empty() && !c.numeric.empty()) {
    Access reads;
    reads.read(c);
    for (auto n : reads.num_reads) {
      if (w.nums.count(n)) {
        return true;
      }
    }
  }
  return false;
}

// Adds a happening of `action` to the STN of the builder; false if it becomes inconsistent.
// A start also adds the end of its action; with `record`, what the happening read and wrote is
// kept, for those after it
bool TemporalFilter::add(
  Builder & b, uint32_t action, const DurationBounds & duration, Stn::Node & node,
  bool record) const
{
  const Access & acc = access_[action];
  auto prev = b.order.before(acc, epsilon_);
  bool start = snap_.is_start(action);
  bool end = snap_.is_end(action);
  size_t self = start || end ? snap_.durative(action) : 0;
  Stn::Node end_node = UINT32_MAX;
  if (end) {
    node = b.open[self].end;
    if (node == UINT32_MAX) {
      return false;
    }
    prev.erase(b.open[self].start);  // the duration orders them
    prev.erase(node);
  } else {
    node = b.stn.add_node();
  }
  for (auto [p, sep] : prev) {
    if (!b.stn.at_least(p, node, sep)) {
      return false;
    }
  }
  if (action >= snap_.tils) {
    if (!b.stn.exactly(Stn::kOrigin, node, snap_.til_time[action - snap_.tils])) {
      return false;
    }
  } else if (auto deadline = next_til(b.tils_done, acc)) {
    // Timed initial literals happen anyway: a happening that interferes with one the plan has
    // not applied yet comes before it (to come after, the plan applies that literal first)
    if (!b.stn.add(Stn::kOrigin, node, *deadline)) {
      return false;
    }
  }
  if (start) {
    // Its end, now: duration, deadlines, and how it sits with the ends of the running actions
    end_node = b.stn.add_node();
    if (!b.stn.between(node, end_node, duration.lo, duration.hi)) {
      return false;
    }
    if (auto deadline = next_til(b.tils_done, access_[action + 1])) {
      if (!b.stn.add(Stn::kOrigin, end_node, *deadline)) {
        return false;
      }
    }
    for (size_t r = 0; r < b.open.size(); ++r) {
      if (r == self || b.open[r].end == UINT32_MAX) {
        continue;
      }
      // Its over all condition must hold until it ends (the open interval: the same instant
      // is fine): an end that breaks it does not come before
      if (breaks(r, self) && !b.stn.at_least(end_node, b.open[r].end, 0.0)) {
        return false;
      }
      if (breaks(self, r) && !b.stn.at_least(b.open[r].end, end_node, 0.0)) {
        return false;
      }
    }
  }
  if (record) {
    if (start) {
      b.open[self] = Builder::Open{node, end_node};
    } else if (end) {
      b.open[self] = Builder::Open{};
    } else if (action >= snap_.tils) {
      ++b.tils_done;
    }
    b.order.record(node, acc);
  }
  return true;
}

// The time of the first timed initial literal from `from` on that interferes with a happening
// (changes what it reads or changes, or reads what it changes)
// The latest time a happening can have before the timed initial literals the plan has not
// applied yet (from `from` on): epsilon before one that changes what it reads or changes, or
// reads what it changes; the same instant for one that changes an over all condition
std::optional<double> TemporalFilter::next_til(size_t from, const Access & acc) const
{
  std::optional<double> best;
  auto first = [&](const std::vector<uint32_t> & tils, double sep) {
      auto it = std::lower_bound(tils.begin(), tils.end(), static_cast<uint32_t>(from));
      if (it != tils.end()) {
        double t = snap_.til_time[*it] - sep;
        best = best ? std::min(*best, t) : t;
      }
    };
  auto scan = [&](const auto & vars, const auto & map, double sep) {
      for (auto v : vars) {
        if (auto it = map.find(v); it != map.end()) {
          first(it->second, sep);
        }
      }
    };
  scan(acc.reads, til_writes_, epsilon_);
  scan(acc.writes, til_writes_, epsilon_);
  scan(acc.writes, til_reads_, epsilon_);
  scan(acc.inv_reads, til_writes_, 0.0);
  scan(acc.num_reads, til_num_writes_, epsilon_);
  scan(acc.num_writes, til_num_writes_, epsilon_);
  scan(acc.inv_num_reads, til_num_writes_, 0.0);
  return best;
}

void TemporalFilter::rebuild(state::StateId parent)
{
  std::vector<int32_t> chain;
  for (int32_t n = node_of(parent); n >= 0; n = nodes_[static_cast<size_t>(n)].parent) {
    chain.push_back(n);
  }
  builder_ = std::make_unique<Builder>(snap_.running.size());
  for (auto it = chain.rbegin(); it != chain.rend(); ++it) {
    const Node & n = nodes_[static_cast<size_t>(*it)];
    Stn::Node ignored;
    add(*builder_, n.action, n.duration, ignored, true);  // consistent: it was accepted
  }
  builder_->mark = builder_->stn.mark();
  cached_ = parent;
  makespans_.clear();
}

int32_t TemporalFilter::node_of(state::StateId s) const
{
  if (s == space_.registry().initial()) {
    return -1;
  }
  return s < node_of_.size() ? node_of_[s] : -2;
}

bool TemporalFilter::accept(state::StateId parent, uint32_t action, state::StateId child)
{
  if (node_of(parent) == -2 || !invariants(child)) {
    return false;
  }
  DurationBounds d;
  if (snap_.is_start(action)) {
    d = bounds(original_.durative_actions[snap_.durative(action)], space_.numbers(parent).data());
    if (d.lo > d.hi) {
      return false;
    }
  }
  if (cached_ != parent || !builder_) {
    rebuild(parent);
  }
  // Try the happening on the parent's STN, then undo it: each child is tried alone
  Stn::Node node;
  bool ok = add(*builder_, action, d, node, false);
  if (ok) {
    // The makespan of the plan with it (pending ends included): for a search minimizing it
    double makespan = 0.0;
    for (Stn::Node n = 1; n < builder_->stn.nodes(); ++n) {
      makespan = std::max(makespan, builder_->stn.earliest(n));
    }
    makespans_[{action, child}] = makespan;
  }
  builder_->stn.undo(builder_->mark);
  if (!ok) {
    return false;
  }
  if (node_of(child) == -2) {
    if (node_of_.size() <= child) {
      node_of_.resize(std::max<size_t>(child + 1, node_of_.size() * 2), -2);
    }
    nodes_.push_back(Node{node_of(parent), action, d});
    node_of_[child] = static_cast<int32_t>(nodes_.size() - 1);
  }
  return true;
}

double TemporalFilter::makespan(
  state::StateId parent, uint32_t action, state::StateId child, double fallback) const
{
  if (parent != cached_) {
    return fallback;
  }
  auto it = makespans_.find({action, child});
  return it == makespans_.end() ? fallback : it->second;
}

std::optional<Plan> TemporalFilter::plan(const std::vector<uint32_t> & actions)
{
  Builder b(snap_.running.size());
  std::vector<Stn::Node> node(actions.size());
  state::StateId s = space_.registry().initial();
  for (size_t k = 0; k < actions.size(); ++k) {
    uint32_t a = actions[k];
    DurationBounds d;
    if (snap_.is_start(a)) {
      d = bounds(original_.durative_actions[snap_.durative(a)], space_.numbers(s).data());
    }
    if (!add(b, a, d, node[k], true)) {
      return std::nullopt;
    }
    auto next = space_.successor(s, a);
    if (!next) {
      return std::nullopt;
    }
    s = *next;
  }
  Plan plan;
  plan.temporal = true;
  std::vector<int64_t> open(snap_.running.size(), -1);
  std::vector<PlanStep> steps;
  for (size_t k = 0; k < actions.size(); ++k) {
    uint32_t a = actions[k];
    if (a >= snap_.tils) {
      continue;  // the timed initial literals are not part of the plan
    }
    if (snap_.is_end(a)) {
      auto & st = steps[static_cast<size_t>(open[snap_.durative(a)])];
      st.duration = b.stn.earliest(node[k]) - st.time;
      continue;
    }
    PlanStep step;
    step.time = b.stn.earliest(node[k]);
    const sas::Action & sa = snap_.task.actions[a];
    step.name = snap_.task.action_names.at(sa.schema);
    for (auto o : sa.args) {
      step.args.push_back(snap_.task.object_names.at(o.index()));
    }
    if (snap_.is_start(a)) {
      open[snap_.durative(a)] = static_cast<int64_t>(steps.size());
    }
    steps.push_back(std::move(step));
  }
  plan.steps = std::move(steps);
  std::stable_sort(
    plan.steps.begin(), plan.steps.end(),
    [](const PlanStep & x, const PlanStep & y) {return x.time < y.time;});
  return plan;
}

}  // namespace kairos::temporal
