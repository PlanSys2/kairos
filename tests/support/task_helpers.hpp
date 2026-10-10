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
/// \brief Helpers to build normalized tasks from PDDL text in tests.

#ifndef KAIROS__TESTS__SUPPORT__TASK_HELPERS_HPP_
#define KAIROS__TESTS__SUPPORT__TASK_HELPERS_HPP_

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <utility>

#include "normalize/normalizer.hpp"
#include "pddl/parser.hpp"
#include "semantic/analyzer.hpp"
#include "support/pddl_helpers.hpp"

namespace kairos::testing
{

/// @brief A parsed, analyzed and normalized domain and problem (the task points into the rest).
struct Normalized
{
  ast::Domain domain;
  ast::Problem problem;
  sem::DomainModel dm;
  sem::ProblemModel pm;
  lifted::Task task;
};

inline std::unique_ptr<Normalized> normalize_text(
  const std::string & domain, const std::string & problem,
  const lifted::NormalizeOptions & o = {})
{
  auto n = std::make_unique<Normalized>();
  auto d = pddl::parse_domain(domain);
  EXPECT_TRUE(d.ok()) << dump(d.diagnostics());
  n->domain = std::move(d).value();
  auto p = pddl::parse_problem(problem);
  EXPECT_TRUE(p.ok()) << dump(p.diagnostics());
  n->problem = std::move(p).value();
  auto dm = sem::analyze_domain(n->domain);
  EXPECT_TRUE(dm.ok()) << dump(dm.diagnostics());
  n->dm = std::move(dm).value();
  auto pm = sem::analyze_problem(n->problem, n->domain, n->dm);
  EXPECT_TRUE(pm.ok()) << dump(pm.diagnostics());
  n->pm = std::move(pm).value();
  n->task = lifted::normalize(n->domain, n->dm, n->problem, n->pm, o);
  return n;
}

}  // namespace kairos::testing

#endif  // KAIROS__TESTS__SUPPORT__TASK_HELPERS_HPP_
