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

#include <iostream>

#include "kairos/kairos.hpp"
#include "pddl/parser.hpp"

int main()
{
  // An internal module (the parser) and the public API, both from the static library
  const auto domain = kairos::pddl::parse_domain(
    "(define (domain d) (:predicates (p)) (:action a :effect (p)))");
  if (!domain.ok()) {
    std::cerr << "internal parser failed\n";
    return 1;
  }
  auto public_domain = kairos::parse_domain(
    "(define (domain d) (:predicates (p)) (:action a :effect (p)))");
  auto problem = kairos::parse_problem(
    "(define (problem p) (:domain d) (:init) (:goal (p)))", *public_domain);
  kairos::Planner planner;
  const auto result = planner.solve(*problem, {});
  std::cout << (result.status == kairos::Status::Solved ? "ok" : "not solved") << '\n';
  return result.status == kairos::Status::Solved ? 0 : 1;
}
