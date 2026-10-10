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
/// \brief SymbolTable: interning of case-insensitive PDDL names.

#ifndef KAIROS__CORE__SYMBOL_TABLE_HPP_
#define KAIROS__CORE__SYMBOL_TABLE_HPP_

#include <deque>
#include <optional>
#include <string>
#include <string_view>
#include <unordered_map>

#include "core/id.hpp"

namespace kairos
{

using Symbol = Id<struct SymbolTag>;

/**
 * @brief Gives each distinct name a Symbol. Names are ASCII-lowercased first, since PDDL is
 * case-insensitive.
 *
 * Symbols are assigned in insertion order (deterministic). The views returned by name() stay
 * valid for the table's lifetime, also after more names are interned.
 */
class SymbolTable
{
public:
  SymbolTable() = default;
  SymbolTable(const SymbolTable & other);
  SymbolTable & operator=(const SymbolTable & other);
  SymbolTable(SymbolTable &&) noexcept = default;
  SymbolTable & operator=(SymbolTable &&) noexcept = default;

  /// @brief Returns the symbol of `name`, creating it if new.
  Symbol intern(std::string_view name);

  /// @brief Returns the symbol of `name` if it was interned.
  std::optional<Symbol> find(std::string_view name) const;

  /// @brief Lower-case text of `symbol`. Throws std::out_of_range for unknown symbols.
  std::string_view name(Symbol symbol) const;

  size_t size() const noexcept {return names_.size();}

  /// @brief ASCII lower-case copy of `text`.
  static std::string to_lower(std::string_view text);

private:
  std::deque<std::string> names_;  // deque: element addresses never move
  std::unordered_map<std::string_view, Symbol> index_;
};

}  // namespace kairos

#endif  // KAIROS__CORE__SYMBOL_TABLE_HPP_
