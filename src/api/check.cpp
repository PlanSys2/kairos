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

#include "kairos/check.hpp"

#include <utility>
#include <vector>

#include "pddl/parser.hpp"
#include "semantic/analyzer.hpp"

namespace kairos
{

namespace
{

void append(std::vector<Diagnostic> & to, std::vector<Diagnostic> from)
{
  to.insert(to.end(), std::make_move_iterator(from.begin()), std::make_move_iterator(from.end()));
}

CheckReport finish(CheckReport report)
{
  report.valid = !has_errors(report.diagnostics);
  return report;
}

}  // namespace

CheckReport check_domain(std::string_view domain_text, std::string_view filename)
{
  CheckReport report;
  auto parsed = pddl::parse_domain(domain_text, filename);
  if (!parsed.ok()) {
    report.diagnostics = parsed.take_diagnostics();
    return finish(std::move(report));
  }
  auto model = sem::analyze_domain(*parsed);
  report.diagnostics = parsed.take_diagnostics();
  if (model.ok()) {
    report.features = model->features.names();
  }
  append(report.diagnostics, model.take_diagnostics());
  return finish(std::move(report));
}

CheckReport check_problem(std::string_view problem_text, std::string_view filename)
{
  CheckReport report;
  auto parsed = pddl::parse_problem(problem_text, filename);
  report.diagnostics = parsed.take_diagnostics();
  if (parsed.ok()) {
    append(report.diagnostics, sem::check_problem_alone(*parsed));
  }
  return finish(std::move(report));
}

CheckReport check_problem(
  std::string_view problem_text, std::string_view domain_text,
  std::string_view problem_filename, std::string_view domain_filename)
{
  CheckReport report;
  auto domain = pddl::parse_domain(domain_text, domain_filename);
  auto problem = pddl::parse_problem(problem_text, problem_filename);
  if (!domain.ok() || !problem.ok()) {
    report.diagnostics = domain.take_diagnostics();
    append(report.diagnostics, problem.take_diagnostics());
    return finish(std::move(report));
  }
  auto dmodel = sem::analyze_domain(*domain);
  report.diagnostics = domain.take_diagnostics();
  append(report.diagnostics, problem.take_diagnostics());
  if (!dmodel.ok()) {
    append(report.diagnostics, dmodel.take_diagnostics());
    return finish(std::move(report));
  }
  auto pmodel = sem::analyze_problem(*problem, *domain, *dmodel);
  append(report.diagnostics, dmodel.take_diagnostics());
  if (pmodel.ok()) {
    report.features = pmodel->features.names();
  }
  append(report.diagnostics, pmodel.take_diagnostics());
  return finish(std::move(report));
}

}  // namespace kairos
