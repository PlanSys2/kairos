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

#include "core/symbol_table.hpp"

#include <algorithm>
#include <stdexcept>
#include <string>
#include <utility>

namespace kairos
{

SymbolTable::SymbolTable(const SymbolTable & other)
{
  // Rebuild: the index must point into our own strings
  for (const auto & name : other.names_) {
    intern(name);
  }
}

SymbolTable & SymbolTable::operator=(const SymbolTable & other)
{
  if (this != &other) {
    SymbolTable copy(other);
    *this = std::move(copy);
  }
  return *this;
}

std::string SymbolTable::to_lower(std::string_view text)
{
  std::string result(text);
  for (auto & c : result) {
    if (c >= 'A' && c <= 'Z') {
      c = static_cast<char>(c - 'A' + 'a');
    }
  }
  return result;
}

Symbol SymbolTable::intern(std::string_view name)
{
  std::string lower = to_lower(name);
  if (auto it = index_.find(lower); it != index_.end()) {
    return it->second;
  }
  if (names_.size() >= static_cast<size_t>(Symbol::invalid_value)) {
    throw std::length_error("kairos::SymbolTable: symbol space exhausted");
  }
  Symbol symbol(static_cast<Symbol::rep_type>(names_.size()));
  names_.push_back(std::move(lower));
  index_.emplace(std::string_view(names_.back()), symbol);
  return symbol;
}

std::optional<Symbol> SymbolTable::find(std::string_view name) const
{
  auto it = index_.find(to_lower(name));
  if (it == index_.end()) {
    return std::nullopt;
  }
  return it->second;
}

std::string_view SymbolTable::name(Symbol symbol) const
{
  if (!symbol.valid() || symbol.index() >= names_.size()) {
    throw std::out_of_range("kairos::SymbolTable: unknown symbol");
  }
  return names_[symbol.index()];
}

}  // namespace kairos
