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

#include "kairos/validate.hpp"

#include <utility>
#include <vector>

#include "pddl/dialect.hpp"
#include "pddl/parser.hpp"
#include "semantic/analyzer.hpp"
#include "validate/validator.hpp"

namespace kairos
{

std::string_view to_string(ValidationStatus status)
{
  switch (status) {
    case ValidationStatus::Valid: return "valid";
    case ValidationStatus::Invalid: return "invalid";
    case ValidationStatus::Unsupported: return "unsupported";
    case ValidationStatus::BadInput: return "bad input";
  }
  return "unknown";
}

ValidationReport validate_plan(
  std::string_view domain_text, std::string_view problem_text, const Plan & plan,
  const ValidateOptions & options, std::string_view domain_filename,
  std::string_view problem_filename)
{
  ValidationReport bad;
  bad.status = ValidationStatus::BadInput;
  auto append = [&](std::vector<Diagnostic> ds) {
      for (auto & d : ds) {
        if (d.severity == Severity::Error) {
          bad.diagnostics.push_back(std::move(d));
        }
      }
    };
  auto domain = pddl::parse_domain(domain_text, domain_filename);
  auto problem = pddl::parse_problem(problem_text, problem_filename);
  if (!domain.ok() || !problem.ok()) {
    append(domain.take_diagnostics());
    append(problem.take_diagnostics());
    return bad;
  }
  (void)pddl::repair_domain(*domain);
  (void)pddl::repair_problem(*problem, *domain);
  auto dm = sem::analyze_domain(*domain);
  if (!dm.ok()) {
    append(dm.take_diagnostics());
    return bad;
  }
  auto pm = sem::analyze_problem(*problem, *domain, *dm);
  if (!pm.ok()) {
    append(pm.take_diagnostics());
    return bad;
  }
  return val::validate(*domain, *dm, *problem, *pm, plan, options);
}

}  // namespace kairos
