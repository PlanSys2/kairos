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
/// \brief Checked model of a domain and problem: types, objects, predicates, functions.

#ifndef KAIROS__SEMANTIC__MODEL_HPP_
#define KAIROS__SEMANTIC__MODEL_HPP_

#include <optional>
#include <set>
#include <string>
#include <utility>
#include <string_view>
#include <unordered_map>
#include <vector>

#include "core/id.hpp"
#include "pddl/ast.hpp"
#include "semantic/features.hpp"

namespace kairos::sem
{

using TypeId = Id<struct TypeTag>;
using ObjectId = Id<struct ObjectTag>;
using PredicateId = Id<struct PredicateTag>;
using FunctionId = Id<struct FunctionTag>;

/// @brief One type, or several for (either ...). Empty means an unknown type (already reported),
/// which every check accepts so that one bad type does not cascade into many errors.
using TypeSpec = std::vector<TypeId>;

struct TypeInfo
{
  std::string name;
  std::vector<TypeId> parents;
  ast::Loc loc;
};

/**
 * @brief Type hierarchy rooted at "object" (always TypeId 0). Supports multiple parents, as
 * "t - (either a b)" in :types allows.
 */
class TypeHierarchy
{
public:
  TypeHierarchy();

  TypeId object() const {return TypeId(0);}

  /// @brief Returns the type named `name`, creating it as a child of object if new.
  TypeId declare(const std::string & name, ast::Loc loc);
  std::optional<TypeId> find(std::string_view name) const;
  void add_parent(TypeId type, TypeId parent);

  /// @brief Gives parentless types object as parent; returns a type on a cycle, if any.
  std::optional<TypeId> finalize();

  const TypeInfo & operator[](TypeId id) const {return types_[id];}
  size_t size() const {return types_.size();}

  /// @brief a is b or a descendant of b. Walks up the parents: time proportional to the
  /// depth of the hierarchy, no precomputed tables (a deep hierarchy costs no memory).
  bool is_subtype(TypeId a, TypeId b) const;

  /// @brief An object of type `object_types` belongs to `required`.
  bool fits(const TypeSpec & object_types, const TypeSpec & required) const;

  /// @brief Some object could have both specs (one is a subtype of the other, or they share
  /// a descendant). Used for variables, whose values are not known yet.
  bool compatible(const TypeSpec & a, const TypeSpec & b) const;

  std::string to_string(const TypeSpec & spec) const;

private:
  IdVector<TypeId, TypeInfo> types_;
  std::unordered_map<std::string, TypeId> index_;
  bool multiple_inheritance_ = false;  // some type has several parents
};

struct ObjectInfo
{
  std::string name;
  TypeSpec types;
  bool constant = false;  // declared in the domain
  ast::Loc loc;
};

struct PredicateInfo
{
  std::string name;
  std::vector<TypeSpec> params;
  bool derived = false;
  ast::Loc loc;
};

struct FunctionInfo
{
  std::string name;
  std::vector<TypeSpec> params;
  bool numeric = true;
  TypeSpec result;       // object fluents only
  bool changes = false;  // some effect assigns it (otherwise static)
  ast::Loc loc;
};

/// @brief Name lookup tables shared by domain and problem models.
template<class IdT, class Info>
struct Table
{
  IdVector<IdT, Info> items;
  std::unordered_map<std::string, IdT> index;

  std::optional<IdT> find(std::string_view name) const
  {
    auto it = index.find(std::string(name));
    if (it == index.end()) {
      return std::nullopt;
    }
    return it->second;
  }

  /// @brief Adds `info` under `name`; returns nullopt if the name already exists.
  std::optional<IdT> add(const std::string & name, Info info)
  {
    if (index.count(name) != 0) {
      return std::nullopt;
    }
    IdT id = items.push_back(std::move(info));
    index.emplace(name, id);
    return id;
  }
};

struct DomainModel
{
  std::string name;
  std::set<std::string> requirements;
  TypeHierarchy types;
  Table<ObjectId, ObjectInfo> constants;
  Table<PredicateId, PredicateInfo> predicates;
  Table<FunctionId, FunctionInfo> functions;
  std::set<std::string> preferences;  // preference names used in the domain
  FeatureSet features;
};

struct ProblemModel
{
  std::string name;
  Table<ObjectId, ObjectInfo> objects;  // domain constants first, then problem objects
  std::set<std::string> preferences;    // preference names in goal and constraints
  FeatureSet features;                  // domain and problem together
};

}  // namespace kairos::sem

#endif  // KAIROS__SEMANTIC__MODEL_HPP_
