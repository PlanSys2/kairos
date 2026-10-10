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
/// \brief Prints syntax trees back as canonical PDDL.

#ifndef KAIROS__PDDL__PRINTER_HPP_
#define KAIROS__PDDL__PRINTER_HPP_

#include <string>

#include "pddl/ast.hpp"

namespace kairos::pddl
{

/**
 * @brief Canonical PDDL text of a domain or problem.
 *
 * Lower-case names, one section per block, two-space indentation, numbers in their shortest
 * exact form. Parsing the output gives back an equivalent tree, so print(parse(print(x))) equals
 * print(x).
 */
std::string to_pddl(const ast::Domain & domain);
std::string to_pddl(const ast::Problem & problem);

/// @brief Single-line text of one node, for messages and debug output.
std::string to_pddl(const ast::Nodes & nodes, const SymbolTable & symbols, ast::FormulaId id);
std::string to_pddl(const ast::Nodes & nodes, const SymbolTable & symbols, ast::EffectId id);
std::string to_pddl(const ast::Nodes & nodes, const SymbolTable & symbols, ast::ExprId id);
std::string to_pddl(const ast::Nodes & nodes, const SymbolTable & symbols, ast::TermId id);

/// @brief Shortest text that reads back as exactly `value` ("2", "0.5", "1e-07").
std::string format_number(double value);

}  // namespace kairos::pddl

#endif  // KAIROS__PDDL__PRINTER_HPP_
