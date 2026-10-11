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
/// \brief Invariant synthesis (Helmert, AIJ 2009) and mutex groups.

#ifndef KAIROS__SAS__INVARIANTS_HPP_
#define KAIROS__SAS__INVARIANTS_HPP_

#include <cstddef>
#include <string>
#include <vector>

#include "core/budget.hpp"
#include "ground/task.hpp"
#include "normalize/task.hpp"

namespace kairos::sas
{

constexpr int kCounted = -1;

/// @brief One predicate of an invariant: its arguments are invariant parameters (0, 1, ...)
/// and at most one counted position.
struct InvariantPart
{
  sem::PredicateId predicate;
  std::vector<int> args;
  int omitted = -1;  // position of kCounted, -1 if none

  size_t arity() const {return omitted < 0 ? args.size() : args.size() - 1;}
  bool operator==(const InvariantPart & o) const
  {
    return predicate == o.predicate && args == o.args;
  }
  bool operator<(const InvariantPart & o) const
  {
    return predicate != o.predicate ? predicate < o.predicate : args < o.args;
  }
};

/// @brief For every value of the parameters, at most one atom of the parts is true.
struct Invariant
{
  std::vector<InvariantPart> parts;  // sorted, one per predicate

  size_t arity() const {return parts.front().arity();}
  const InvariantPart * part(sem::PredicateId p) const;
  bool operator==(const Invariant & o) const {return parts == o.parts;}
  bool operator<(const Invariant & o) const {return parts < o.parts;}
};

struct InvariantOptions
{
  size_t max_candidates = 100000;  // as FD's translator
  double max_seconds = 300.0;
};

struct InvariantStats
{
  size_t candidates = 0;
  size_t invariants = 0;
  bool limit_reached = false;
  double milliseconds = 0.0;
};

/**
 * @brief Invariants of the task, proved on its lifted actions.
 *
 * As FD's translator: candidates start with one predicate (with or without a counted
 * argument); a candidate is kept if no action adds two atoms of one instance ("too heavy") and
 * every add is balanced by a delete of an atom known to be true. An unbalanced add refines the
 * candidate with the predicates the action deletes.
 *
 * Durative actions are checked as their start and end: an end add may also be balanced by an
 * unconditional start delete of an at-start condition (the action reserves it), when the start
 * adds nothing of the invariant. Predicates added by timed initial literals are left out.
 * The ground task gives the reachable parameters: pairs never equal count as different.
 */
std::vector<Invariant> find_invariants(
  const lifted::Task & lifted, const ground::Task & ground, const InvariantOptions & options = {},
  Budget * budget = nullptr, InvariantStats * stats = nullptr);

/// @brief Mutex groups: instances of the invariants with exactly one atom true at the start,
/// with their reachable atoms (sorted as FD does).
std::vector<std::vector<ground::AtomId>> mutex_groups(
  const std::vector<Invariant> & invariants, const ground::Task & ground);

/// @brief Greedy cover by the largest groups; each atom ends in at most one group. Atoms that
/// appear negated in the goal are left out (they become binary variables). Groups of one atom
/// are not returned.
std::vector<std::vector<ground::AtomId>> choose_groups(
  const std::vector<std::vector<ground::AtomId>> & groups, const ground::Task & ground);

std::string to_string(const lifted::Task & task, const Invariant & invariant);

}  // namespace kairos::sas

#endif  // KAIROS__SAS__INVARIANTS_HPP_
