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

#include "semantic/model.hpp"

#include <algorithm>
#include <string>
#include <utility>

namespace kairos::sem
{

TypeHierarchy::TypeHierarchy()
{
  types_.push_back(TypeInfo{"object", {}, {}});
  index_.emplace("object", TypeId(0));
}

TypeId TypeHierarchy::declare(const std::string & name, ast::Loc loc)
{
  if (auto it = index_.find(name); it != index_.end()) {
    return it->second;
  }
  TypeId id = types_.push_back(TypeInfo{name, {}, loc});
  index_.emplace(name, id);
  return id;
}

std::optional<TypeId> TypeHierarchy::find(std::string_view name) const
{
  auto it = index_.find(std::string(name));
  if (it == index_.end()) {
    return std::nullopt;
  }
  return it->second;
}

void TypeHierarchy::add_parent(TypeId type, TypeId parent)
{
  auto & parents = types_[type].parents;
  if (std::find(parents.begin(), parents.end(), parent) == parents.end()) {
    parents.push_back(parent);
  }
}

std::optional<TypeId> TypeHierarchy::finalize()
{
  // Types without parents hang from object
  for (TypeId t : types_.ids()) {
    if (t != object() && types_[t].parents.empty()) {
      types_[t].parents.push_back(object());
    }
  }
  // Iterative DFS with colors: 0 new, 1 on stack, 2 done
  ancestors_.assign(types_.size(), {});
  std::vector<int> color(types_.size(), 0);
  for (TypeId start : types_.ids()) {
    if (color[start.index()] != 0) {
      continue;
    }
    std::vector<std::pair<TypeId, size_t>> stack{{start, 0}};
    color[start.index()] = 1;
    while (!stack.empty()) {
      auto & [t, next] = stack.back();
      const auto & parents = types_[t].parents;
      if (next < parents.size()) {
        TypeId p = parents[next++];
        if (color[p.index()] == 1) {
          return p;
        }
        if (color[p.index()] == 0) {
          color[p.index()] = 1;
          stack.emplace_back(p, 0);
        }
        continue;
      }
      auto & anc = ancestors_[t.index()];
      anc.insert(t);
      for (TypeId p : parents) {
        anc.insert(ancestors_[p.index()].begin(), ancestors_[p.index()].end());
      }
      color[t.index()] = 2;
      stack.pop_back();
    }
  }
  return std::nullopt;
}

bool TypeHierarchy::is_subtype(TypeId a, TypeId b) const
{
  if (a == b || b == object()) {
    return true;
  }
  if (a.index() >= ancestors_.size()) {
    return false;
  }
  return ancestors_[a.index()].count(b) != 0;
}

bool TypeHierarchy::fits(const TypeSpec & object_types, const TypeSpec & required) const
{
  if (object_types.empty() || required.empty()) {
    return true;  // unknown type, already reported: do not cascade
  }
  for (TypeId o : object_types) {
    for (TypeId r : required) {
      if (is_subtype(o, r)) {
        return true;
      }
    }
  }
  return false;
}

bool TypeHierarchy::compatible(const TypeSpec & a, const TypeSpec & b) const
{
  if (a.empty() || b.empty()) {
    return true;  // unknown type, already reported: do not cascade
  }
  for (TypeId x : a) {
    for (TypeId y : b) {
      if (is_subtype(x, y) || is_subtype(y, x)) {
        return true;
      }
      // A common descendant
      for (size_t t = 0; t < ancestors_.size(); ++t) {
        if (ancestors_[t].count(x) != 0 && ancestors_[t].count(y) != 0) {
          return true;
        }
      }
    }
  }
  return false;
}

std::string TypeHierarchy::to_string(const TypeSpec & spec) const
{
  if (spec.empty()) {
    return "(unknown type)";
  }
  if (spec.size() == 1) {
    return types_[spec[0]].name;
  }
  std::string out = "(either";
  for (TypeId t : spec) {
    out += " " + types_[t].name;
  }
  return out + ")";
}

}  // namespace kairos::sem
