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
/// \brief Helpers that take PDDL text through normalization, grounding and SAS+ in tests.

#ifndef KAIROS__TESTS__SUPPORT__PIPELINE_HELPERS_HPP_
#define KAIROS__TESTS__SUPPORT__PIPELINE_HELPERS_HPP_

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <utility>

#include "ground/grounder.hpp"
#include "sas/translate.hpp"
#include "support/task_helpers.hpp"

namespace kairos::testing
{

struct Pipeline
{
  std::unique_ptr<Normalized> n;
  ground::Task ground;
  sas::Task sas;
  sas::TranslateStats stats;
  bool ok = false;
};

inline std::unique_ptr<Pipeline> pipeline(
  const std::string & domain, const std::string & problem,
  const sas::TranslateOptions & options = {}, const ground::GroundOptions & gopt = {})
{
  auto p = std::make_unique<Pipeline>();
  p->n = normalize_text(domain, problem);
  auto g = ground::ground(p->n->task, gopt);
  EXPECT_TRUE(g.ok()) << dump(g.diagnostics());
  if (!g.ok()) {
    return p;
  }
  p->ground = std::move(g).value();
  auto s = sas::translate(p->n->task, p->ground, options, nullptr, &p->stats);
  EXPECT_TRUE(s.ok()) << dump(s.diagnostics());
  if (!s.ok()) {
    return p;
  }
  p->sas = std::move(s).value();
  p->ok = true;
  return p;
}

}  // namespace kairos::testing

#endif  // KAIROS__TESTS__SUPPORT__PIPELINE_HELPERS_HPP_
